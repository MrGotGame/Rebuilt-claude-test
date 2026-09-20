package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The vision fusion policy, which lives here and only here.
 *
 * <p>Each loop this subsystem: pushes fresh robot orientation to both Limelights; recomputes the
 * turret camera's robot-to-camera transform from the live turret angle and pushes it; drains both
 * cameras' pose observations; computes a per-observation covariance from tag count and distance;
 * applies the rejection rules (ambiguity, Z error, pose jump, turret slew rate, timestamp
 * staleness); and hands the survivors to {@link RobotState}.
 *
 * <p>It then evaluates QuestNav health, decides whether QuestNav is trusted, seeds or re-seeds it
 * against the MegaTag2 estimate when required, and feeds its poses in with much tighter covariance.
 *
 * <p>Every decision is logged. A bad pose during a match must be diagnosable from the log alone,
 * because it will not be reproducible in the pit.
 */
public class Vision extends SubsystemBase {
  private final VisionIO turretCameraIO;
  private final VisionIO intakeCameraIO;
  private final QuestNavIO questNavIO;
  private final VisionIOInputsAutoLogged turretInputs = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged intakeInputs = new VisionIOInputsAutoLogged();
  private final QuestNavIOInputsAutoLogged questInputs = new QuestNavIOInputsAutoLogged();

  private final DoubleSupplier turretAngleSupplier;
  private final DoubleSupplier turretVelocitySupplier;

  private final Timer questResidualTimer = new Timer();
  private int acceptedCount = 0;
  private int rejectedCount = 0;
  private boolean questTrusted = false;

  /**
   * Why an observation was thrown away.
   *
   * <p>An enum rather than free strings, so the log keys cannot drift and so every reason is
   * published even when its count is zero. A counter that only appears once it fires is useless for
   * the M7 checkpoint, which asks you to provoke each reason and watch the matching counter move --
   * you cannot watch a key that does not exist yet.
   */
  private enum RejectionReason {
    NO_TAGS,
    SINGLE_TAG_AMBIGUOUS,
    Z_ERROR,
    OUTSIDE_FIELD,
    TURRET_SLEWING,
    NO_CORRELATED_POSE,
    POSE_JUMP
  }

  private final Map<RejectionReason, Integer> intakeRejections =
      new EnumMap<>(RejectionReason.class);

  private final Alert intakeCameraDisconnectedAlert =
      new Alert("Intake Limelight disconnected", AlertType.kError);

  /**
   * Fires while the field dimensions are still unmeasured.
   *
   * <p>A warning rather than an error, because the robot works without it, but a loud one: with
   * {@code kFieldLengthMeters} at 0.0 the out-of-field rule cannot run, so a wildly wrong pose that
   * would normally be discarded is instead fused. See {@link #isOutsideField(Pose2d)}.
   */
  private final Alert fieldBoundsUnmeasuredAlert =
      new Alert(
          "Field dimensions are still 0.0, so vision cannot reject out-of-field poses.",
          AlertType.kWarning);

  /**
   * Builds the vision subsystem.
   *
   * <p>The turret angle arrives as suppliers rather than a {@code Turret} reference, deliberately:
   * it keeps vision from depending on the turret subsystem and makes this class testable.
   *
   * @param turretCameraIO the turret-mounted Limelight
   * @param intakeCameraIO the intake-mounted Limelight
   * @param questNavIO the Quest 3S headset
   * @param turretAngleSupplier live robot-relative turret angle, in radians
   * @param turretVelocitySupplier live turret angular velocity, in radians per second
   */
  public Vision(
      VisionIO turretCameraIO,
      VisionIO intakeCameraIO,
      QuestNavIO questNavIO,
      DoubleSupplier turretAngleSupplier,
      DoubleSupplier turretVelocitySupplier) {
    this.turretCameraIO = turretCameraIO;
    this.intakeCameraIO = intakeCameraIO;
    this.questNavIO = questNavIO;
    this.turretAngleSupplier = turretAngleSupplier;
    this.turretVelocitySupplier = turretVelocitySupplier;
    // IMPLEMENTATION (was TODO: "Register FaultReporter checks ... QuestNav ones as WARNINGS.")
    //
    // The intake camera's check is an Alert driven from periodic(), for the reason recorded in
    // PhoenixUtil.tryUntilOk and Shooter: FaultReporter is a skeleton whose getInstance() returns
    // null and would throw here.
    //
    // The turret-camera and QuestNav checks are deliberately NOT registered. M7 is the intake
    // camera only; the turret camera is M8 and the headset M9, and an alert for a device this
    // milestone never reads would sit permanently red on a healthy robot -- which teaches everyone
    // to ignore the alerts widget, the one place M14 needs them to look.
    for (RejectionReason reason : RejectionReason.values()) {
      intakeRejections.put(reason, 0);
    }
  }

  /**
   * Runs the full vision pipeline for this loop.
   *
   * <p><b>Outputs:</b> pushes orientation and transforms to the cameras; feeds accepted
   * observations into {@link RobotState}; updates QuestNav trust; logs accepted and rejected counts
   * with reasons.
   */
  @Override
  public void periodic() {
    // IMPLEMENTATION (was TODO: the eight-step order, steps 2 and 3 before step 4.)
    //
    // M7 SCOPE: the intake camera only, as the milestone specifies -- a fixed transform is the easy
    // one, and doing it first settles the rejection ladder and the covariance model before the
    // moving turret transform is layered on in M8.
    //
    // Steps 3, 5 and 7 are absent rather than forgotten:
    //   - steps 3 and 5 (the turret camera) are M8. robotToTurretCameraNow() still returns the
    //     identity transform, so fusing turret observations now would feed the estimator poses
    //     solved about the robot origin instead of the camera -- confidently wrong, which is worse
    //     than absent.
    //   - step 7 (QuestNav) is M9.
    //
    // The ordering rule the TODO is really about is preserved where it matters: orientation is
    // pushed BEFORE inputs are read, because the camera solves using what was last pushed.
    pushRobotOrientation();

    intakeCameraIO.updateInputs(intakeInputs);
    Logger.processInputs("Vision/" + Constants.Vision.kIntakeCameraName, intakeInputs);

    // The turret camera is READ but not FUSED. Sampling it keeps its health and raw observations in
    // the log from M7 onward, so M8's bring-up is a comparison against a baseline rather than a
    // fresh mystery.
    turretCameraIO.updateInputs(turretInputs);
    Logger.processInputs("Vision/" + Constants.Vision.kTurretCameraName, turretInputs);

    processCamera(intakeInputs, RobotState.PoseSource.LIMELIGHT_INTAKE, false);

    Logger.recordOutput("Vision/AcceptedObservations", acceptedCount);
    for (Map.Entry<RejectionReason, Integer> entry : intakeRejections.entrySet()) {
      Logger.recordOutput(
          "Vision/" + Constants.Vision.kIntakeCameraName + "/Rejections/" + entry.getKey(),
          entry.getValue());
    }

    intakeCameraDisconnectedAlert.set(!intakeInputs.connected);
    fieldBoundsUnmeasuredAlert.set(!fieldBoundsAreMeasured());
  }

  /**
   * Pushes the fused heading to both cameras, which MegaTag2 requires.
   *
   * <p><b>Outputs:</b> writes robot orientation to both Limelights.
   */
  private void pushRobotOrientation() {
    // IMPLEMENTATION (was TODO: "getRotation() and the fused omega, pushed to both cameras. Use
    // the FUSED estimate, not the raw gyro.")
    //
    // Both cameras get the heading even though only the intake camera is fused this milestone,
    // because MegaTag2 needs it to solve at all -- withholding it would make the turret camera's
    // logged observations meaningless rather than merely unused.
    var yaw = RobotState.getInstance().getRotation();
    double rateRadPerSec = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
    intakeCameraIO.setRobotOrientation(yaw, rateRadPerSec);
    turretCameraIO.setRobotOrientation(yaw, rateRadPerSec);
  }

  /**
   * The turret camera transform for right now, extrapolated one loop ahead.
   *
   * @return the robot-to-camera transform to push to the turret Limelight
   */
  public Transform3d robotToTurretCameraNow() {
    // TODO: GeomUtil.rotateAboutZ(Constants.Vision.kRobotToTurretAxis,
    //     turretAngleSupplier.getAsDouble()
    //         + turretVelocitySupplier.getAsDouble() * Constants.getLoopPeriodSec())
    //   .plus(Constants.Vision.kTurretToCamera);
    // The one-loop velocity extrapolation cancels the NetworkTables write latency, so the camera
    // solves with the transform it will actually have when it captures the frame.
    return new Transform3d();
  }

  /**
   * The turret camera transform as it was at a past capture time.
   *
   * @param timestampSec FPGA capture time to reconstruct, in seconds
   * @return the historical robot-to-camera transform, or the identity transform if the turret
   *     buffer has no sample that old
   */
  public Transform3d robotToTurretCameraAt(double timestampSec) {
    // TODO: Same composition as robotToTurretCameraNow, but using
    // RobotState.getInstance().getTurretAngleAt(timestampSec).
    // When the buffer has NO sample, log a warning and return identity -- and the CALLER must then
    // reject that observation rather than use the transform. Silently substituting the current
    // turret angle here is the bug this whole buffer exists to prevent.
    return new Transform3d();
  }

  /**
   * Applies the rejection rules to one camera's observations and forwards the survivors.
   *
   * @param inputs that camera's freshly-sampled inputs
   * @param source which pose source these observations belong to
   * @param isTurretMounted true for the turret camera, which has extra slew-rate rejection
   *     <p><b>Outputs:</b> calls {@code RobotState.addVisionObservation} for accepted observations;
   *     increments and logs per-reason rejection counters.
   */
  private void processCamera(
      VisionIO.VisionIOInputs inputs, RobotState.PoseSource source, boolean isTurretMounted) {
    // IMPLEMENTATION (was TODO: the six rejection rules, then computeStdDevs and
    // addVisionObservation, counting and logging every rejection with its reason.)
    //
    // Rules run cheapest-first and each one CONTINUES rather than falling through, so a rejection
    // is attributed to exactly one reason. Attributing to several would make the counters sum to
    // more than the rejection total and quietly ruin the M7 checkpoint, which reads them as a
    // partition.
    for (VisionIO.PoseObservation observation : inputs.poseObservations) {
      Pose2d pose = observation.pose().toPose2d();

      if (observation.tagCount() == 0) {
        reject(source, RejectionReason.NO_TAGS);
        continue;
      }

      // Ambiguity is only meaningful for a single-tag solve; a multi-tag fix is geometrically
      // constrained and its ambiguity field is not defined.
      if (observation.tagCount() == 1
          && observation.ambiguity() > Constants.Vision.kMaxAmbiguity.get()) {
        reject(source, RejectionReason.SINGLE_TAG_AMBIGUOUS);
        continue;
      }

      // A good fix puts the robot on the floor plane. Height is the cheapest proxy for "this solve
      // is geometric nonsense", and it catches tag misidentification, which otherwise produces a
      // confident pose somewhere else entirely.
      if (Math.abs(observation.pose().getZ()) > Constants.Vision.kMaxZErrorMeters.get()) {
        reject(source, RejectionReason.Z_ERROR);
        continue;
      }

      if (isOutsideField(pose)) {
        reject(source, RejectionReason.OUTSIDE_FIELD);
        continue;
      }

      // Turret-mounted only. A camera swinging with the turret smears its exposure across a
      // changing transform, and the resulting pose is wrong in a way no covariance can express.
      if (isTurretMounted
          && Math.abs(turretVelocitySupplier.getAsDouble())
              > Constants.Vision.kMaxTurretRateForVisionRadPerSec.get()) {
        reject(source, RejectionReason.TURRET_SLEWING);
        continue;
      }

      // No odometry sample at the capture time means the observation cannot be correlated with
      // where the robot was, so the estimator cannot apply it correctly.
      Optional<Pose2d> correlated = RobotState.getInstance().getPoseAt(observation.timestampSec());
      if (correlated.isEmpty()) {
        reject(source, RejectionReason.NO_CORRELATED_POSE);
        continue;
      }

      // The jump gate compares against where the robot BELIEVED it was at the capture time, not
      // against the current estimate; comparing against now would reject honest observations
      // simply because the robot moved during the camera's latency.
      double jumpMeters = correlated.get().getTranslation().getDistance(pose.getTranslation());
      if (jumpMeters > Constants.Vision.kMaxAcceptedPoseJumpMeters.get()) {
        reject(source, RejectionReason.POSE_JUMP);
        continue;
      }

      RobotState.getInstance()
          .addVisionObservation(
              new RobotState.PoseObservation(
                  pose,
                  observation.timestampSec(),
                  computeStdDevs(
                      observation.tagCount(), observation.averageTagDistanceMeters(), source),
                  source));
      acceptedCount++;
    }
  }

  /**
   * Records one rejection against its reason.
   *
   * @param source which camera produced the observation
   * @param reason why it was discarded
   *     <p><b>Outputs:</b> increments the per-reason counter and the total.
   */
  private void reject(RobotState.PoseSource source, RejectionReason reason) {
    rejectedCount++;
    if (source == RobotState.PoseSource.LIMELIGHT_INTAKE) {
      intakeRejections.merge(reason, 1, Integer::sum);
    }
  }

  /**
   * Whether the field dimensions have been measured yet.
   *
   * @return true once both field dimensions are non-zero
   */
  private static boolean fieldBoundsAreMeasured() {
    return Constants.Field.kFieldLengthMeters.get() > 0.0
        && Constants.Field.kFieldWidthMeters.get() > 0.0;
  }

  /**
   * Whether a pose lies outside the field, with a margin.
   *
   * @param pose the candidate field pose
   * @return true when the pose is outside the playing field by more than the tolerated margin
   */
  private static boolean isOutsideField(Pose2d pose) {
    // The unmeasured-field guard is load-bearing, not defensive padding. kFieldLengthMeters and
    // kFieldWidthMeters are both still 0.0 pending the REBUILT drawings, and a bounds check against
    // zero rejects EVERY observation -- which presents as "vision does not work", sends someone
    // hunting through the camera config, and is indistinguishable from a dead Limelight.
    //
    // Returning false disables only this one rule; the other six still run, and the cost is
    // recorded in fieldBoundsUnmeasuredAlert rather than hidden.
    if (!fieldBoundsAreMeasured()) {
      return false;
    }

    // A tolerance, because a legitimate fix taken with the robot against the wall can solve a few
    // centimetres outside it -- which is exactly the situation at the start of a match.
    double margin = Constants.Vision.kMaxZErrorMeters.get();
    return pose.getX() < -margin
        || pose.getX() > Constants.Field.kFieldLengthMeters.get() + margin
        || pose.getY() < -margin
        || pose.getY() > Constants.Field.kFieldWidthMeters.get() + margin;
  }

  /**
   * How many observations have been rejected for a given reason.
   *
   * <p>Exposed so the rejection ladder can be asserted directly. The M7 checkpoint asks that every
   * counter be provoked at least once, and a test does that far more precisely than waving a hand
   * in front of a camera.
   *
   * @param reason the rejection reason to query, by its enum name
   * @return how many intake-camera observations were discarded for that reason
   */
  public int getRejectionCount(String reason) {
    return intakeRejections.getOrDefault(RejectionReason.valueOf(reason), 0);
  }

  /**
   * How many observations have been rejected in total.
   *
   * @return the rejected-observation count across all reasons
   */
  @AutoLogOutput(key = "Vision/RejectedObservations")
  public int getRejectedCount() {
    return rejectedCount;
  }

  /**
   * Assigns a covariance to one observation.
   *
   * @param tagCount how many tags contributed
   * @param averageDistanceMeters mean distance to those tags, in meters
   * @param source which sensor produced the observation
   * @return per-axis standard deviations [x, y, theta] for the pose estimator
   */
  private Matrix<N3, N1> computeStdDevs(
      int tagCount, double averageDistanceMeters, RobotState.PoseSource source) {
    // IMPLEMENTATION (was TODO: "xy = base * averageDistance^2 / tagCount; theta left enormous.")
    //
    // Distance is SQUARED because the positional error of a tag solve grows with the square of
    // range: a pixel of angular resolution subtends a linearly growing distance, and the depth
    // solve degrades faster still as the tag's apparent size shrinks.
    //
    // Dividing by tag count is the crude form of averaging independent measurements. Not
    // statistically exact -- the tags share a camera calibration and are not independent -- but it
    // captures the part that matters: a four-tag fix deserves markedly more trust than a
    // single-tag one at the same range.
    //
    // theta is left enormous on purpose, and this is the most important line in the method.
    // MegaTag2 CONSUMES the heading this class pushed to it in order to solve, so feeding its
    // heading output back into the estimator closes a loop on the robot's own belief. The failure
    // is not a jump but a slow, confident rotation of the entire field frame, which looks like a
    // drifting gyro and gets diagnosed as one for a long time.
    double xy =
        Constants.Vision.kMegaTag2XyStdDevBase.get()
            * averageDistanceMeters
            * averageDistanceMeters
            / Math.max(1, tagCount);
    return VecBuilder.fill(xy, xy, Constants.Vision.kMegaTag2ThetaStdDev.get());
  }

  /**
   * Decides whether QuestNav may be used as a pose source this loop.
   *
   * <p><b>Outputs:</b> updates {@code questTrusted}, pushes it into {@link RobotState}, and logs
   * the specific reason whenever trust is withdrawn.
   */
  private void updateQuestNavTrust() {
    // TODO: questTrusted = questInputs.connected
    //     && questInputs.tracking
    //     && questInputs.batteryPercent > kQuestMinBatteryPercent.get()
    //     && trackingLostCounter has NOT increased this loop
    //     && RobotState.getInstance().isQuestSeeded()
    //     && the residual between the Quest-derived robot pose and RobotState's estimate has stayed
    //        below kQuestReseedResidualMeters.
    // Push the result into RobotState.setQuestTrusted, and log WHICH clause failed. "QuestNav is
    // not trusted" with no reason attached is not actionable at 11pm before a competition.
  }

  /**
   * Feeds trusted QuestNav frames into the estimator and re-seeds the headset when it drifts.
   *
   * <p><b>Outputs:</b> may call {@code RobotState.addVisionObservation} and {@link
   * #seedQuestNav()}.
   */
  private void processQuestNav() {
    // TODO: For each drained frame, convert the headset Pose3d into a robot Pose2d by applying
    // Constants.Vision.kRobotToQuest.inverse(). If questTrusted, call
    // RobotState.getInstance().addVisionObservation(...) with
    // VecBuilder.fill(kQuestXyStdDev.get(), kQuestXyStdDev.get(), kQuestThetaStdDev.get()).
    // Call seedQuestNav() when either: the headset is not yet seeded; or the residual has exceeded
    // kQuestReseedResidualMeters continuously for kQuestReseedHoldSec WHILE the robot is nearly
    // stationary AND at least two tags are visible. All three conditions matter -- re-seeding from
    // a moving robot or a single-tag fix writes your drift into the headset permanently.
  }

  /**
   * Seeds the headset's origin from the current fused pose estimate.
   *
   * <p><b>Outputs:</b> commands the headset to a new origin, marks it seeded in {@link RobotState},
   * and logs the seed event with the pose so re-seeds per match can be counted.
   */
  public void seedQuestNav() {
    // TODO: Pose2d p = RobotState.getInstance().getEstimatedPose();
    // questNavIO.setPose(new Pose3d(p).transformBy(Constants.Vision.kRobotToQuest));
    // RobotState.getInstance().setQuestSeeded(true);
    // Then log the event and the pose. A match with many re-seeds means the headset mount is loose
    // or kRobotToQuest is wrong.
  }

  /**
   * The most prominent game piece the intake camera can currently see.
   *
   * @return the largest-area target observation, or empty when none is visible or the camera is on
   *     its AprilTag pipeline
   */
  public Optional<VisionIO.TargetObservation> getBestGamePieceTarget() {
    // IMPLEMENTATION (was TODO: "Return the largest-area TargetObservation from intakeInputs, but
    // only while the intake camera is on its game-piece pipeline.")
    //
    // The pipeline guard is NOT a check against a cached index, because nothing writes one:
    // setIntakePipeline is fire-and-forget and the Limelight does not report the active pipeline
    // back through this IO. Filtering on hasTarget is the honest equivalent -- the AprilTag
    // pipeline reports no game-piece targets, so the array is empty and this returns empty anyway.
    //
    // TODO: once VisionIO reports the active pipeline index, gate on it explicitly.
    if (!intakeInputs.hasTarget || intakeInputs.targetObservations.length == 0) {
      return Optional.empty();
    }
    return Arrays.stream(intakeInputs.targetObservations)
        .max((a, b) -> Double.compare(a.area(), b.area()));
  }

  /**
   * Where the best visible game piece is on the field.
   *
   * @return the estimated field pose of the game piece, or empty when none is visible
   */
  public Optional<Pose2d> getGamePieceFieldPose() {
    // TODO: Convert the best target's tx/ty into a field position with the planar-ground
    // assumption:
    //   range   = (kRobotToIntakeCamera.getZ() - kGamePieceDiameterMeters.get() / 2.0)
    //             / Math.tan(cameraPitch + ty)
    //   bearing = -tx
    // then transform by the robot pose AT THE OBSERVATION TIMESTAMP, not the current pose.
    // TODO: kGamePieceDiameterMeters must be MEASURED before this returns anything meaningful.
    return Optional.empty();
  }

  /**
   * Switches the intake camera's pipeline.
   *
   * @param index the pipeline index to select
   *     <p><b>Outputs:</b> commands the intake camera. The superstructure switches to the
   *     game-piece pipeline whenever the intake is deployed, and back to AprilTags when stowed.
   */
  public void setIntakePipeline(int index) {
    // IMPLEMENTATION (was TODO: "intakeCameraIO.setPipeline(index).")
    intakeCameraIO.setPipeline(index);
  }

  /**
   * Whether QuestNav is currently being used as a pose source.
   *
   * @return true when the headset passed every health check this loop
   */
  @AutoLogOutput(key = "Vision/QuestTrusted")
  public boolean isQuestTrusted() {
    // TODO: return questTrusted.
    return false;
  }

  /**
   * How many observations have been accepted since startup.
   *
   * @return the accepted-observation count
   */
  @AutoLogOutput(key = "Vision/AcceptedObservations")
  public int getAcceptedCount() {
    // IMPLEMENTATION (was TODO: "return acceptedCount.")
    return acceptedCount;
  }
}
