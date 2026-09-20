package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The single source of truth for "where is the robot, how fast is it going, and where was the
 * turret pointing at time <i>t</i>."
 *
 * <p>Every consumer reads from here -- the shooting solver, the vision transform math, auto-align,
 * and PathPlanner's AutoBuilder. Nothing else in this project owns a pose estimator. This class
 * wraps a {@link SwerveDrivePoseEstimator}, fed odometry at roughly 250 Hz from {@code
 * Drive.periodic()} and vision at roughly 30 Hz from {@code Vision.periodic()}, plus two {@link
 * TimeInterpolatableBuffer}s holding robot pose and turret angle two seconds deep.
 *
 * <p>The turret buffer is what makes the turret-mounted camera usable at all: a vision frame that
 * was captured 80 ms ago must be un-projected using the turret angle <i>at capture time</i>, not
 * the angle right now.
 *
 * <p>All writes happen on the main robot thread. The read side is guarded by a {@link
 * ReentrantReadWriteLock} so that a future background consumer cannot observe a torn {@link
 * Pose2d}.
 */
public class RobotState {
  /** Where a pose observation came from; drives covariance, logging, and the fallback rules. */
  public enum PoseSource {
    /** Wheel odometry fused with the Pigeon2. */
    ODOMETRY,
    /** The turret-mounted Limelight 4. */
    LIMELIGHT_TURRET,
    /** The intake-mounted Limelight 4. */
    LIMELIGHT_INTAKE,
    /** The Meta Quest 3S running QuestNav. */
    QUESTNAV
  }

  /**
   * One timestamped absolute-pose measurement with its covariance.
   *
   * @param pose the measured field pose, blue-origin, in meters and radians
   * @param timestampSec FPGA capture time of the measurement, in seconds
   * @param stdDevs per-axis standard deviations [x, y, theta]; larger means trusted less
   * @param source which sensor produced this measurement
   */
  public record PoseObservation(
      Pose2d pose, double timestampSec, Matrix<N3, N1> stdDevs, PoseSource source) {}

  /**
   * Depth of both interpolation buffers, in seconds.
   *
   * <p>Named rather than repeated, because {@link #addVisionObservation} rejects observations older
   * than this window and the buffers must agree with that rule -- if they drift apart, observations
   * get accepted that {@link #getPoseAt} cannot then answer for.
   */
  private static final double kBufferSeconds = 2.0;

  private static RobotState instance = null;

  private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(kBufferSeconds);
  private final TimeInterpolatableBuffer<Double> turretAngleBuffer =
      TimeInterpolatableBuffer.createDoubleBuffer(kBufferSeconds);

  private final LinearFilter accelXFilter = LinearFilter.movingAverage(5);
  private final LinearFilter accelYFilter = LinearFilter.movingAverage(5);

  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions;
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private Translation2d fieldRelativeAccel = Translation2d.kZero;
  private ChassisSpeeds lastFieldRelativeSpeeds = new ChassisSpeeds();
  private double lastSpeedsTimestampSec = 0.0;

  private final double[] lastUpdateTimestampBySource = new double[PoseSource.values().length];
  private final double[] lastStdDevBySource = new double[PoseSource.values().length];
  private boolean questTrusted = false;
  private boolean questSeeded = false;

  /**
   * Running totals of vision observations accepted and rejected since boot.
   *
   * <p>Logged every loop. A pipeline that is silently rejecting everything is indistinguishable
   * from a pipeline that is not running at all unless both numbers are visible.
   */
  private int visionAcceptedCount = 0;

  private int visionRejectedCount = 0;

  /**
   * Returns the process-wide robot state.
   *
   * @return the singleton, or null if {@link #initialize(SwerveDriveKinematics)} has not run yet
   */
  public static RobotState getInstance() {
    // IMPLEMENTATION (was TODO: "Lazily construct the singleton with Drive.getModuleTranslations()
    // as the kinematics source. Keep it lazy so unit tests can build one without a HAL.")
    //
    // No lazy construction here, deliberately. RobotContainer calls initialize() as its very first
    // statement, before any subsystem exists, so by the time anything can call getInstance() the
    // instance is already there. Constructing lazily from here would hide an ordering bug: whoever
    // called first would silently define the kinematics, and a later caller would get a RobotState
    // built from whatever that first caller happened to pass.
    //
    // Returning null when uninitialized is the loud failure we want -- an immediate NPE at the
    // first use, pointing straight at the missing initialize() call.
    return instance;
  }

  /**
   * Builds the pose estimator and its buffers.
   *
   * @param kinematics the drivetrain kinematics, used to integrate module positions
   */
  private RobotState(SwerveDriveKinematics kinematics) {
    // NOTE: the two final fields below must be assigned here for the file to compile, so this
    // constructor is partly filled in already. Tune the standard deviations rather than rewriting.
    this.kinematics = kinematics;
    this.lastModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.kZero,
            this.lastModulePositions,
            Pose2d.kZero,
            VecBuilder.fill(0.003, 0.003, 0.002),
            VecBuilder.fill(0.5, 0.5, 9999.0));
    // TODO: Confirm the odometry std devs above against your measured wheel slip, and remember the
    // vision std devs here are only placeholders -- every addVisionObservation call supplies its
    // own.
  }

  /**
   * Creates the singleton explicitly, so construction order is deterministic rather than depending
   * on whichever subsystem happens to call {@link #getInstance()} first.
   *
   * @param kinematics the drivetrain kinematics
   *     <p><b>Outputs:</b> assigns the static singleton.
   */
  public static void initialize(SwerveDriveKinematics kinematics) {
    // IMPLEMENTATION (was TODO: "Create the singleton explicitly from RobotContainer before any
    // subsystem uses it, so construction order is deterministic instead of depending on who calls
    // getInstance() first.")
    //
    // Idempotent on purpose: simulation and unit tests may construct a RobotContainer more than
    // once in a single JVM, and silently rebuilding the estimator underneath live subsystems would
    // strand them holding a stale reference.
    if (instance == null) {
      instance = new RobotState(kinematics);
    }
  }

  // ------------------------------------------------------------------------------ write side ----

  /**
   * Feeds one odometry sample into the estimator. Called roughly 250 times per second from the
   * odometry thread via {@code Drive.periodic()}.
   *
   * @param timestampSec FPGA time of the sample, in seconds
   * @param gyroYaw raw gyro yaw at that instant
   * @param modulePositions the four module positions at that instant
   *     <p><b>Outputs:</b> advances the pose estimator and appends to the pose buffer.
   */
  public void addOdometryObservation(
      double timestampSec, Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
    // IMPLEMENTATION (was TODO: "Take the write lock. Call poseEstimator.updateWithTime(...), then
    // poseBuffer.addSample(...). Record lastUpdateTimestampBySource[ODOMETRY].")
    //
    // This is the hot path: it runs roughly 250 times per second, once per sample drained from the
    // Phoenix odometry thread, NOT once per robot loop. Everything in here must stay cheap.
    lock.writeLock().lock();
    try {
      // Advance the estimator to this sample's timestamp. updateWithTime (rather than update) is
      // what lets the estimator interleave these back-dated odometry samples correctly with vision
      // measurements that arrive out of order.
      poseEstimator.updateWithTime(timestampSec, gyroYaw, modulePositions);

      // Keep the raw gyro yaw so resetPose() and Drive can reference the same value the estimator
      // was last advanced with.
      rawGyroRotation = gyroYaw;

      // Snapshot the resulting pose into the interpolation buffer. This is what makes
      // getPoseAt(t) answerable, which is what lets a vision frame captured 80 ms ago be compared
      // against where the robot actually was at capture time.
      poseBuffer.addSample(timestampSec, poseEstimator.getEstimatedPosition());

      lastUpdateTimestampBySource[PoseSource.ODOMETRY.ordinal()] = timestampSec;
    } finally {
      // finally, not a plain unlock: an exception from the estimator would otherwise leave the
      // write lock held forever and deadlock every reader on the next loop.
      lock.writeLock().unlock();
    }
  }

  /**
   * Feeds the measured chassis velocity, and differentiates it into an acceleration estimate.
   *
   * @param timestampSec FPGA time of the sample, in seconds
   * @param robotRelative measured chassis speeds in the robot frame, in meters and radians per
   *     second
   *     <p><b>Outputs:</b> updates the robot-relative, field-relative and acceleration estimates.
   */
  public void addDriveSpeedsObservation(double timestampSec, ChassisSpeeds robotRelative) {
    // IMPLEMENTATION (was TODO: "Under the write lock, store robotRelative, convert to field
    // relative with ChassisSpeeds.fromRobotRelativeSpeeds(...), then differentiate against
    // lastFieldRelativeSpeeds over (timestampSec - lastSpeedsTimestampSec) and push through
    // accelXFilter/accelYFilter to get fieldRelativeAccel.")
    lock.writeLock().lock();
    try {
      robotRelativeSpeeds = robotRelative;

      // Rotate into the field frame using the CURRENT estimated heading. Read the estimator
      // directly rather than calling getEstimatedPose(), which would re-enter the lock -- legal
      // with a ReentrantReadWriteLock, but pointless work on a per-loop path.
      Rotation2d heading = poseEstimator.getEstimatedPosition().getRotation();
      fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, heading);

      // Differentiate velocity into acceleration.
      double dt = timestampSec - lastSpeedsTimestampSec;
      if (dt > 1e-6) {
        // Raw finite difference, then low-pass. Differentiating a noisy signal amplifies its noise,
        // so the moving-average filters are not optional polish -- an unfiltered acceleration
        // estimate is dominated by quantization and is useless as a shot gate.
        double rawAccelX =
            (fieldRelativeSpeeds.vxMetersPerSecond - lastFieldRelativeSpeeds.vxMetersPerSecond)
                / dt;
        double rawAccelY =
            (fieldRelativeSpeeds.vyMetersPerSecond - lastFieldRelativeSpeeds.vyMetersPerSecond)
                / dt;
        fieldRelativeAccel =
            new Translation2d(accelXFilter.calculate(rawAccelX), accelYFilter.calculate(rawAccelY));
      }
      // When dt is zero or negative -- the first call ever, or a timestamp that went backwards --
      // the previous acceleration estimate is simply held. Dividing by it would produce infinity,
      // which then propagates into the readyToShoot gate and silently blocks every shot.

      lastFieldRelativeSpeeds = fieldRelativeSpeeds;
      lastSpeedsTimestampSec = timestampSec;
    } finally {
      lock.writeLock().unlock();
    }
  }

  /**
   * Offers an absolute pose measurement to the estimator, subject to rejection rules.
   *
   * @param observation the candidate measurement with its covariance and source
   *     <p><b>Outputs:</b> may call {@code addVisionMeasurement}; updates per-source bookkeeping;
   *     logs accept and reject counts.
   */
  public void addVisionObservation(PoseObservation observation) {
    // IMPLEMENTATION (was TODO: "Under the write lock, reject the observation if it is older than
    // the buffer window, if its translation differs from getEstimatedPose() by more than
    // Constants.Vision.kMaxAcceptedPoseJumpMeters, or if any stdDev is non-finite. Otherwise call
    // poseEstimator.addVisionMeasurement(...) and update the per-source bookkeeping arrays. Log
    // accepted and rejected counts.")
    lock.writeLock().lock();
    try {
      String rejectReason = null;

      // Rule 1: non-finite covariance. A NaN or infinite standard deviation poisons the estimator's
      // internal matrices permanently -- there is no recovering the estimate afterwards -- so this
      // is checked before anything else.
      Matrix<N3, N1> stdDevs = observation.stdDevs();
      if (stdDevs == null
          || !Double.isFinite(stdDevs.get(0, 0))
          || !Double.isFinite(stdDevs.get(1, 0))
          || !Double.isFinite(stdDevs.get(2, 0))) {
        rejectReason = "NonFiniteStdDev";
      }

      // Rule 2: too old to correlate. The pose buffer is 2 s deep; an observation older than that
      // cannot be checked against where the robot actually was, and the estimator would rewind its
      // history against a pose nobody can verify.
      double now = Timer.getFPGATimestamp();
      if (rejectReason == null && now - observation.timestampSec() > kBufferSeconds) {
        rejectReason = "StaleTimestamp";
      }

      // Rule 3: implausible jump. A single bad tag solve can place the robot metres away. Accepting
      // it teleports the estimate, and every downstream consumer -- turret aim, auto-align, the
      // shot gate -- acts on that teleport before odometry can pull it back.
      if (rejectReason == null) {
        double jump =
            observation
                .pose()
                .getTranslation()
                .getDistance(poseEstimator.getEstimatedPosition().getTranslation());
        if (jump > Constants.Vision.kMaxAcceptedPoseJumpMeters.get()) {
          rejectReason = "PoseJump";
        }
      }

      if (rejectReason != null) {
        visionRejectedCount++;
        // Logged per source AND per reason. When the robot will not shoot because the pose is
        // untrusted, this is the log that says why in one glance instead of three matches.
        Logger.recordOutput(
            "RobotState/Vision/" + observation.source().name() + "/LastRejectReason", rejectReason);
        Logger.recordOutput("RobotState/Vision/RejectedCount", visionRejectedCount);
        return;
      }

      poseEstimator.addVisionMeasurement(observation.pose(), observation.timestampSec(), stdDevs);

      // Per-source bookkeeping. The XY standard deviation is recorded (both axes share a value by
      // construction in Vision.computeStdDevs) so getPoseStdDevEstimate can report how much the
      // most-trusted recent source was actually trusted.
      int sourceIndex = observation.source().ordinal();
      lastUpdateTimestampBySource[sourceIndex] = observation.timestampSec();
      lastStdDevBySource[sourceIndex] = stdDevs.get(0, 0);

      visionAcceptedCount++;
      Logger.recordOutput("RobotState/Vision/AcceptedCount", visionAcceptedCount);
    } finally {
      lock.writeLock().unlock();
    }
  }

  /**
   * Records where the turret was pointing at a given instant. Called once per {@code
   * Turret.periodic()}.
   *
   * @param timestampSec FPGA time of the sample, in seconds
   * @param turretAngleRad robot-relative turret angle at that instant, in radians
   *     <p><b>Outputs:</b> appends to the turret angle buffer.
   */
  public void addTurretObservation(double timestampSec, double turretAngleRad) {
    // IMPLEMENTATION (was TODO: "Under the write lock, turretAngleBuffer.addSample(timestampSec,
    // turretAngleRad).")
    lock.writeLock().lock();
    try {
      turretAngleBuffer.addSample(timestampSec, turretAngleRad);
    } finally {
      lock.writeLock().unlock();
    }
  }

  /**
   * Hard-resets the pose estimate, for example after an auto start pose or a driver re-seed.
   *
   * @param pose the pose to assume, blue-origin, in meters and radians
   * @param gyroYaw the current raw gyro yaw
   * @param modulePositions the current module positions
   *     <p><b>Outputs:</b> resets the estimator, clears the pose buffer, and clears the QuestNav
   *     seeded flag so vision re-seeds the headset.
   */
  public void resetPose(Pose2d pose, Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
    // IMPLEMENTATION (was TODO: "Under the write lock call poseEstimator.resetPosition(gyroYaw,
    // modulePositions, pose) and clear poseBuffer. Also set questSeeded = false so Vision re-seeds
    // QuestNav to the new pose.")
    lock.writeLock().lock();
    try {
      poseEstimator.resetPosition(gyroYaw, modulePositions, pose);
      rawGyroRotation = gyroYaw;

      // The buffer must be cleared, not merely appended to. Its existing samples describe a
      // different coordinate origin, and interpolating across the discontinuity would hand out
      // poses that never existed -- exactly during the moments after an auto start-pose reset when
      // vision is most likely to be consulted.
      poseBuffer.clear();
      poseBuffer.addSample(Timer.getFPGATimestamp(), pose);

      // The headset's origin is now wrong by however far the reset moved us, so force Vision to
      // re-seed it rather than continuing to fuse a pose from a stale frame.
      questSeeded = false;
    } finally {
      lock.writeLock().unlock();
    }
  }

  /**
   * Records whether QuestNav is currently trusted as a pose source.
   *
   * @param trusted true while the headset is tracking well and its battery is healthy
   *     <p><b>Outputs:</b> sets the flag and logs it, so a replay shows exactly when QuestNav
   *     dropped out.
   */
  public void setQuestTrusted(boolean trusted) {
    // IMPLEMENTATION (was TODO: "Write-locked setter used by Vision's health policy; logged so you
    // can see in replay exactly when QuestNav dropped out.")
    lock.writeLock().lock();
    try {
      questTrusted = trusted;
    } finally {
      lock.writeLock().unlock();
    }
    // Logged OUTSIDE the lock. Logger.recordOutput is not guaranteed cheap, and holding the write
    // lock across it would block every reader on the 250 Hz odometry path for no reason.
    Logger.recordOutput("RobotState/QuestTrusted", trusted);
  }

  /**
   * Records whether QuestNav has been seeded to the field frame.
   *
   * @param seeded true once a successful {@code QuestNav.setPose()} has been performed
   *     <p><b>Outputs:</b> sets the flag.
   */
  public void setQuestSeeded(boolean seeded) {
    // IMPLEMENTATION (was TODO: "Write-locked setter; Vision sets this true after a successful
    // QuestNav.setPose().")
    lock.writeLock().lock();
    try {
      questSeeded = seeded;
    } finally {
      lock.writeLock().unlock();
    }
    Logger.recordOutput("RobotState/QuestSeeded", seeded);
  }

  // ------------------------------------------------------------------------------- read side ----

  /**
   * The current best estimate of the robot's field pose.
   *
   * @return estimated pose, blue-origin, in meters and radians
   */
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    // IMPLEMENTATION (was TODO: "Read-locked poseEstimator.getEstimatedPosition().")
    lock.readLock().lock();
    try {
      return poseEstimator.getEstimatedPosition();
    } finally {
      lock.readLock().unlock();
    }
  }

  /**
   * The robot's pose at a past instant, interpolated from the buffer.
   *
   * @param timestampSec FPGA time to query, in seconds
   * @return the interpolated pose, or empty if the timestamp predates the two-second buffer
   */
  public Optional<Pose2d> getPoseAt(double timestampSec) {
    // IMPLEMENTATION (was TODO: "Read-locked poseBuffer.getSample(timestampSec). Returns empty if
    // the timestamp predates the buffer, which callers must treat as "skip this measurement".")
    lock.readLock().lock();
    try {
      // The window check is what makes the documented contract true. WPILib's
      // TimeInterpolatableBuffer.getSample CLAMPS to its endpoints rather than returning empty for
      // an out-of-range timestamp, so a bare delegation quietly answers every query -- correlating
      // a thirty-second-old vision frame against the oldest pose still buffered and reporting that
      // as a valid correlation.
      //
      // Found by M7's rejection-ladder tests: the NO_CORRELATED_POSE rule could never fire, because
      // this method never said no. M2's checkpoint could not have caught it either, since
      // getPoseAt(now - 0.1) is well inside the window.
      //
      // Only the OLD side is guarded, deliberately. A timestamp slightly newer than the last
      // odometry sample is normal -- odometry is discrete and a camera frame can land between
      // samples -- and clamping forward by a few milliseconds is both harmless and correct.
      var samples = poseBuffer.getInternalBuffer();
      if (samples.isEmpty() || timestampSec < samples.firstKey()) {
        return Optional.empty();
      }
      return poseBuffer.getSample(timestampSec);
    } finally {
      lock.readLock().unlock();
    }
  }

  /**
   * The turret angle at a past instant, interpolated from the buffer.
   *
   * @param timestampSec FPGA time to query, in seconds
   * @return robot-relative turret angle in radians, or empty if the timestamp predates the buffer
   */
  public Optional<Double> getTurretAngleAt(double timestampSec) {
    // IMPLEMENTATION (was TODO: "Read-locked turretAngleBuffer.getSample(timestampSec).")
    lock.readLock().lock();
    try {
      // The same window guard getPoseAt carries, and for the same reason: WPILib's
      // TimeInterpolatableBuffer.getSample CLAMPS an out-of-range query to its nearest endpoint
      // rather than returning empty, so a bare delegation answers every question ever asked.
      //
      // This one is worse than the pose version, because of WHAT the wrong answer is. Clamping
      // returns the OLDEST angle still buffered -- up to two seconds stale -- and section 7.2 of
      // the plan exists specifically to stop a turret frame being un-projected with an angle it
      // was not captured at. Rule 7 of the rejection ladder is the enforcement, and rule 7 fires
      // on this method returning empty. Without the guard the rule is unreachable code: every
      // frame gets a transform, some of them 180 degrees wrong, and the resulting pose error looks
      // like a camera calibration problem rather than a buffer problem.
      //
      // Found while implementing M8. M7 could not have caught it -- the intake camera never calls
      // this method -- and neither could M2's checkpoint, which only ever queries 100 ms back.
      //
      // Only the OLD side is guarded, matching getPoseAt. A query a few milliseconds newer than
      // the last turret sample is routine, because the camera frame lands between 20 ms periodic
      // calls, and clamping forward across that gap is both harmless and correct.
      var samples = turretAngleBuffer.getInternalBuffer();
      if (samples.isEmpty() || timestampSec < samples.firstKey()) {
        return Optional.empty();
      }
      return turretAngleBuffer.getSample(timestampSec);
    } finally {
      lock.readLock().unlock();
    }
  }

  /**
   * The chassis velocity in the field frame. This is the velocity the shooting solver compensates
   * for.
   *
   * @return field-relative chassis speeds, in meters and radians per second
   */
  @AutoLogOutput(key = "RobotState/FieldRelativeSpeeds")
  public ChassisSpeeds getFieldRelativeSpeeds() {
    // IMPLEMENTATION (was TODO: "Read-locked copy of fieldRelativeSpeeds.")
    //
    // A COPY is returned, not the field. ChassisSpeeds is mutable, and handing out the live object
    // would let any caller corrupt the shared estimate for everyone else.
    lock.readLock().lock();
    try {
      return new ChassisSpeeds(
          fieldRelativeSpeeds.vxMetersPerSecond,
          fieldRelativeSpeeds.vyMetersPerSecond,
          fieldRelativeSpeeds.omegaRadiansPerSecond);
    } finally {
      lock.readLock().unlock();
    }
  }

  /**
   * The chassis velocity in the robot frame, as PathPlanner's AutoBuilder expects it.
   *
   * @return robot-relative chassis speeds, in meters and radians per second
   */
  @AutoLogOutput(key = "RobotState/RobotRelativeSpeeds")
  public ChassisSpeeds getRobotRelativeSpeeds() {
    // IMPLEMENTATION (was TODO: "Read-locked copy; PathPlanner's AutoBuilder consumes this one.")
    lock.readLock().lock();
    try {
      return new ChassisSpeeds(
          robotRelativeSpeeds.vxMetersPerSecond,
          robotRelativeSpeeds.vyMetersPerSecond,
          robotRelativeSpeeds.omegaRadiansPerSecond);
    } finally {
      lock.readLock().unlock();
    }
  }

  /**
   * The filtered chassis acceleration, used to gate shots taken while the driver is still
   * accelerating.
   *
   * @return field-relative acceleration, in meters per second squared
   */
  @AutoLogOutput(key = "RobotState/FieldRelativeAccel")
  public Translation2d getFieldRelativeAccel() {
    // IMPLEMENTATION (was TODO: "Read-locked copy of the filtered acceleration, m/s^2, blue-origin
    // field frame.")
    //
    // No defensive copy needed: Translation2d is immutable.
    lock.readLock().lock();
    try {
      return fieldRelativeAccel;
    } finally {
      lock.readLock().unlock();
    }
  }

  /**
   * The robot's current heading.
   *
   * @return estimated field heading
   */
  public Rotation2d getRotation() {
    // IMPLEMENTATION (was TODO: "getEstimatedPose().getRotation().")
    return getEstimatedPose().getRotation();
  }

  /**
   * Straight-line distance from the robot to the alliance goal.
   *
   * @return distance to the goal, in meters
   */
  @AutoLogOutput(key = "RobotState/DistanceToGoalMeters")
  public double getDistanceToGoal() {
    // IMPLEMENTATION (was TODO: "getEstimatedPose().getTranslation().getDistance(
    // Constants.Field.goalTranslation()).")
    //
    // NOTE: this is only as meaningful as Constants.Field's goal coordinates, which are still 0,0
    // until someone MEASURES them from the REBUILT field drawings. Until then this reports the
    // distance to the field origin, which is a real number that means nothing.
    return getEstimatedPose().getTranslation().getDistance(Constants.Field.goalTranslation());
  }

  /**
   * How long it has been since either Limelight last contributed a measurement.
   *
   * @return seconds since the most recent accepted Limelight update; positive infinity if none
   */
  @AutoLogOutput(key = "RobotState/VisionStalenessSec")
  public double getVisionStalenessSec() {
    // IMPLEMENTATION (was TODO: "Timer.getFPGATimestamp() minus the most recent of the two
    // Limelight entries in lastUpdateTimestampBySource. Used as a readyToShoot gate.")
    //
    // Only the two Limelights count. QuestNav is deliberately excluded: it reports a pose
    // continuously whether or not it still knows where it is, so including it would mask exactly
    // the drift this gate exists to catch.
    lock.readLock().lock();
    try {
      double mostRecent =
          Math.max(
              lastUpdateTimestampBySource[PoseSource.LIMELIGHT_TURRET.ordinal()],
              lastUpdateTimestampBySource[PoseSource.LIMELIGHT_INTAKE.ordinal()]);

      // 0.0 means "never updated", not "updated at time zero". Returning infinity makes every
      // downstream comparison fail closed rather than silently passing a freshness check.
      if (mostRecent == 0.0) {
        return Double.POSITIVE_INFINITY;
      }
      return Timer.getFPGATimestamp() - mostRecent;
    } finally {
      lock.readLock().unlock();
    }
  }

  /**
   * A single scalar proxy for how much the pose estimate can be trusted right now.
   *
   * @return an effective XY standard deviation in meters; compare against {@code
   *     Constants.Shot.kMaxPoseStdDevMeters}
   */
  @AutoLogOutput(key = "RobotState/PoseConfidence")
  public double getPoseStdDevEstimate() {
    // IMPLEMENTATION (was TODO: "Return a scalar confidence proxy: the smallest recently-applied XY
    // std dev, inflated by getVisionStalenessSec(). Compare against
    // Constants.Shot.kMaxPoseStdDevMeters.")
    //
    // "Smallest" because the best source available is what the estimate is really worth: if the
    // turret camera has a four-tag fix at 2 m, it does not matter that the intake camera's last
    // contribution was a poor one.
    double best;
    lock.readLock().lock();
    try {
      best = Double.POSITIVE_INFINITY;
      for (PoseSource source : PoseSource.values()) {
        if (source == PoseSource.ODOMETRY) {
          continue; // Odometry has no absolute covariance to report.
        }
        double stdDev = lastStdDevBySource[source.ordinal()];
        if (stdDev > 0.0 && stdDev < best) {
          best = stdDev;
        }
      }
    } finally {
      lock.readLock().unlock();
    }

    if (!Double.isFinite(best)) {
      return Double.POSITIVE_INFINITY;
    }

    // Inflate with age. A 4 cm fix from three seconds ago does not describe where the robot is now,
    // and without this term confidence would stay falsely high for as long as the cameras stayed
    // silent -- which is precisely when it should be falling.
    double staleness = getVisionStalenessSec();
    if (!Double.isFinite(staleness)) {
      return Double.POSITIVE_INFINITY;
    }
    return best + staleness * Constants.Vision.kStalenessGrowthMetersPerSec.get();
  }

  /**
   * Whether QuestNav is currently trusted.
   *
   * @return true when the headset is healthy and tracking
   */
  public boolean isQuestTrusted() {
    // IMPLEMENTATION (was TODO: "Read-locked getter.")
    lock.readLock().lock();
    try {
      return questTrusted;
    } finally {
      lock.readLock().unlock();
    }
  }

  /**
   * Whether QuestNav has been seeded to the field frame.
   *
   * @return true once a successful seed has happened
   */
  public boolean isQuestSeeded() {
    // IMPLEMENTATION (was TODO: "Read-locked getter.")
    lock.readLock().lock();
    try {
      return questSeeded;
    } finally {
      lock.readLock().unlock();
    }
  }

  /**
   * Publishes per-source health to the log. Called once per {@code robotPeriodic} from {@code
   * Robot}.
   *
   * <p><b>Outputs:</b> logs staleness and standard deviation for each {@link PoseSource} under
   * {@code RobotState/Sources/<NAME>/...}, so a replay shows exactly which source was driving the
   * estimate at any instant.
   */
  public void logState() {
    // IMPLEMENTATION (was TODO: "Called once per robotPeriodic from Robot. Log per-source staleness
    // and std devs under RobotState/Sources/<NAME>/... so replay shows exactly which source was
    // driving the estimate at any instant.")
    //
    // The pose and velocity are logged EXPLICITLY here rather than relying on the @AutoLogOutput
    // annotations on the getters. AdvantageKit only scans objects it knows about -- subsystems, and
    // anything registered with AutoLogOutputManager -- and RobotState is a plain singleton that is
    // not a field of Robot, so its annotations would otherwise never be picked up.
    // TODO: if you later register this instance with AutoLogOutputManager.addObject(), these
    // explicit records become redundant and can be removed.
    Logger.recordOutput("RobotState/EstimatedPose", getEstimatedPose());
    Logger.recordOutput("RobotState/FieldRelativeSpeeds", getFieldRelativeSpeeds());
    Logger.recordOutput("RobotState/RobotRelativeSpeeds", getRobotRelativeSpeeds());
    Logger.recordOutput("RobotState/FieldRelativeAccel", getFieldRelativeAccel());
    Logger.recordOutput("RobotState/RawGyroRotation", rawGyroRotation);
    Logger.recordOutput("RobotState/DistanceToGoalMeters", getDistanceToGoal());
    Logger.recordOutput("RobotState/VisionStalenessSec", getVisionStalenessSec());
    Logger.recordOutput("RobotState/PoseConfidence", getPoseStdDevEstimate());
    Logger.recordOutput("RobotState/Vision/AcceptedCount", visionAcceptedCount);
    Logger.recordOutput("RobotState/Vision/RejectedCount", visionRejectedCount);

    // Per-source health. Staleness is far more actionable than a raw timestamp: "the turret camera
    // last helped 4 seconds ago" is immediately readable on a graph, where an FPGA timestamp is
    // not.
    double now = Timer.getFPGATimestamp();
    for (PoseSource source : PoseSource.values()) {
      int i = source.ordinal();
      double last = lastUpdateTimestampBySource[i];
      Logger.recordOutput(
          "RobotState/Sources/" + source.name() + "/StalenessSec",
          last == 0.0 ? Double.POSITIVE_INFINITY : now - last);
      Logger.recordOutput(
          "RobotState/Sources/" + source.name() + "/LastStdDev", lastStdDevBySource[i]);
    }
  }
}
