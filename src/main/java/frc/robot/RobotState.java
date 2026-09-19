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
import java.util.Optional;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.littletonrobotics.junction.AutoLogOutput;

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

  private static RobotState instance = null;

  private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(2.0);
  private final TimeInterpolatableBuffer<Double> turretAngleBuffer =
      TimeInterpolatableBuffer.createDoubleBuffer(2.0);

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
   * Returns the process-wide robot state.
   *
   * @return the singleton, or null if {@link #initialize(SwerveDriveKinematics)} has not run yet
   */
  public static RobotState getInstance() {
    // TODO: Return the instance. Prefer calling initialize() from RobotContainer first; if you also
    // want lazy construction here, build it from Drive.getModuleTranslations(). Keeping it lazy is
    // what lets unit tests construct one without a HAL.
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
    // TODO: instance = new RobotState(kinematics). Call this from RobotContainer's constructor
    // before any subsystem is built.
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
    // TODO: Take the write lock. Call poseEstimator.updateWithTime(timestampSec, gyroYaw,
    // modulePositions), then poseBuffer.addSample(timestampSec,
    // poseEstimator.getEstimatedPosition()). Record lastUpdateTimestampBySource[ODOMETRY.ordinal()]
    // and store gyroYaw into rawGyroRotation. Always release the lock in a finally block.
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
    // TODO: Under the write lock, store robotRelative, convert with
    // ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, getEstimatedPose().getRotation()), then
    // differentiate against lastFieldRelativeSpeeds over (timestampSec - lastSpeedsTimestampSec)
    // and push each axis through accelXFilter/accelYFilter. Guard against a zero or negative dt on
    // the first call, which would otherwise produce an infinite acceleration.
  }

  /**
   * Offers an absolute pose measurement to the estimator, subject to rejection rules.
   *
   * @param observation the candidate measurement with its covariance and source
   *     <p><b>Outputs:</b> may call {@code addVisionMeasurement}; updates per-source bookkeeping;
   *     logs accept and reject counts.
   */
  public void addVisionObservation(PoseObservation observation) {
    // TODO: Under the write lock, REJECT when: the timestamp is older than the 2 s buffer window;
    // the translation differs from getEstimatedPose() by more than
    // Constants.Vision.kMaxAcceptedPoseJumpMeters; or any stdDev is non-finite. Otherwise call
    // poseEstimator.addVisionMeasurement(pose, timestampSec, stdDevs) and update
    // lastUpdateTimestampBySource / lastStdDevBySource for the source. Log both counts -- a
    // silently
    // rejecting vision pipeline looks exactly like a vision pipeline that is not running.
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
    // TODO: Under the write lock, turretAngleBuffer.addSample(timestampSec, turretAngleRad).
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
    // TODO: Under the write lock call poseEstimator.resetPosition(gyroYaw, modulePositions, pose),
    // clear poseBuffer, and set questSeeded = false.
  }

  /**
   * Records whether QuestNav is currently trusted as a pose source.
   *
   * @param trusted true while the headset is tracking well and its battery is healthy
   *     <p><b>Outputs:</b> sets the flag and logs it, so a replay shows exactly when QuestNav
   *     dropped out.
   */
  public void setQuestTrusted(boolean trusted) {
    // TODO: Write-locked setter; also Logger.recordOutput("RobotState/QuestTrusted", trusted).
  }

  /**
   * Records whether QuestNav has been seeded to the field frame.
   *
   * @param seeded true once a successful {@code QuestNav.setPose()} has been performed
   *     <p><b>Outputs:</b> sets the flag.
   */
  public void setQuestSeeded(boolean seeded) {
    // TODO: Write-locked setter. Vision sets this true after a successful seed.
  }

  // ------------------------------------------------------------------------------- read side ----

  /**
   * The current best estimate of the robot's field pose.
   *
   * @return estimated pose, blue-origin, in meters and radians
   */
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    // TODO: Read-locked poseEstimator.getEstimatedPosition().
    return Pose2d.kZero;
  }

  /**
   * The robot's pose at a past instant, interpolated from the buffer.
   *
   * @param timestampSec FPGA time to query, in seconds
   * @return the interpolated pose, or empty if the timestamp predates the two-second buffer
   */
  public Optional<Pose2d> getPoseAt(double timestampSec) {
    // TODO: Read-locked poseBuffer.getSample(timestampSec). An empty result means "skip this
    // measurement" -- callers must NEVER silently substitute the current pose, which would
    // reintroduce exactly the latency error this buffer exists to remove.
    return Optional.empty();
  }

  /**
   * The turret angle at a past instant, interpolated from the buffer.
   *
   * @param timestampSec FPGA time to query, in seconds
   * @return robot-relative turret angle in radians, or empty if the timestamp predates the buffer
   */
  public Optional<Double> getTurretAngleAt(double timestampSec) {
    // TODO: Read-locked turretAngleBuffer.getSample(timestampSec). Used by
    // Vision.robotToTurretCameraAt(t).
    return Optional.empty();
  }

  /**
   * The chassis velocity in the field frame. This is the velocity the shooting solver compensates
   * for.
   *
   * @return field-relative chassis speeds, in meters and radians per second
   */
  @AutoLogOutput(key = "RobotState/FieldRelativeSpeeds")
  public ChassisSpeeds getFieldRelativeSpeeds() {
    // TODO: Read-locked copy of fieldRelativeSpeeds. Return a copy, not the field itself.
    return new ChassisSpeeds();
  }

  /**
   * The chassis velocity in the robot frame, as PathPlanner's AutoBuilder expects it.
   *
   * @return robot-relative chassis speeds, in meters and radians per second
   */
  @AutoLogOutput(key = "RobotState/RobotRelativeSpeeds")
  public ChassisSpeeds getRobotRelativeSpeeds() {
    // TODO: Read-locked copy of robotRelativeSpeeds.
    return new ChassisSpeeds();
  }

  /**
   * The filtered chassis acceleration, used to gate shots taken while the driver is still
   * accelerating.
   *
   * @return field-relative acceleration, in meters per second squared
   */
  @AutoLogOutput(key = "RobotState/FieldRelativeAccel")
  public Translation2d getFieldRelativeAccel() {
    // TODO: Read-locked copy of fieldRelativeAccel.
    return Translation2d.kZero;
  }

  /**
   * The robot's current heading.
   *
   * @return estimated field heading
   */
  public Rotation2d getRotation() {
    // TODO: return getEstimatedPose().getRotation().
    return Rotation2d.kZero;
  }

  /**
   * Straight-line distance from the robot to the alliance goal.
   *
   * @return distance to the goal, in meters
   */
  @AutoLogOutput(key = "RobotState/DistanceToGoalMeters")
  public double getDistanceToGoal() {
    // TODO: getEstimatedPose().getTranslation().getDistance(Constants.Field.goalTranslation()).
    return 0.0;
  }

  /**
   * How long it has been since either Limelight last contributed a measurement.
   *
   * @return seconds since the most recent accepted Limelight update; positive infinity if none
   */
  @AutoLogOutput(key = "RobotState/VisionStalenessSec")
  public double getVisionStalenessSec() {
    // TODO: Timer.getFPGATimestamp() minus the later of the two Limelight entries in
    // lastUpdateTimestampBySource. Used as a readyToShoot gate.
    return Double.POSITIVE_INFINITY;
  }

  /**
   * A single scalar proxy for how much the pose estimate can be trusted right now.
   *
   * @return an effective XY standard deviation in meters; compare against {@code
   *     Constants.Shot.kMaxPoseStdDevMeters}
   */
  @AutoLogOutput(key = "RobotState/PoseConfidence")
  public double getPoseStdDevEstimate() {
    // TODO: Return the smallest recently-applied XY std dev, inflated by getVisionStalenessSec() so
    // that confidence decays when vision stops arriving rather than staying falsely high.
    return Double.POSITIVE_INFINITY;
  }

  /**
   * Whether QuestNav is currently trusted.
   *
   * @return true when the headset is healthy and tracking
   */
  public boolean isQuestTrusted() {
    // TODO: Read-locked getter.
    return false;
  }

  /**
   * Whether QuestNav has been seeded to the field frame.
   *
   * @return true once a successful seed has happened
   */
  public boolean isQuestSeeded() {
    // TODO: Read-locked getter.
    return false;
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
    // TODO: Loop PoseSource.values() and Logger.recordOutput the staleness and std dev for each.
  }
}
