package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

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
    // TODO: Register FaultReporter checks: "Turret Limelight disconnected" and "Intake Limelight
    // disconnected" as errors; "QuestNav not tracking" and "QuestNav battery low" as WARNINGS, not
    // errors -- the robot still works without the headset, and a warning that stops the match is
    // worse than the drift it was protecting you from.
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
    // TODO: This exact order matters:
    //   1. questNavIO.commandPeriodic()
    //   2. pushRobotOrientation()
    //   3. turretCameraIO.setRobotToCamera(robotToTurretCameraNow())
    //   4. updateInputs + Logger.processInputs for all three IOs
    //   5. processCamera(turretInputs, PoseSource.LIMELIGHT_TURRET, true)
    //   6. processCamera(intakeInputs, PoseSource.LIMELIGHT_INTAKE, false)
    //   7. updateQuestNavTrust(); then processQuestNav()
    //   8. Log accepted and rejected counts, and every rejection reason
    // Steps 2 and 3 MUST precede step 4: both cameras solve using what you pushed, so pushing after
    // reading means every frame is solved with last loop's data.
  }

  /**
   * Pushes the fused heading to both cameras, which MegaTag2 requires.
   *
   * <p><b>Outputs:</b> writes robot orientation to both Limelights.
   */
  private void pushRobotOrientation() {
    // TODO: Rotation2d yaw = RobotState.getInstance().getRotation();
    // double rate = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
    // then push to both cameras.
    // Use the FUSED estimate, not the raw gyro: MegaTag2 is only as good as this heading, and the
    // fused estimate is the one that has been corrected for gyro drift.
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
    // TODO: For each PoseObservation, REJECT when:
    //   - tagCount == 0
    //   - tagCount == 1 and ambiguity > Constants.Vision.kMaxAmbiguity.get()
    //   - Math.abs(pose.getZ()) > kMaxZErrorMeters.get()   (a good fix is on the floor plane)
    //   - the pose lies outside the field bounds
    //   - isTurretMounted and Math.abs(turretVelocitySupplier.getAsDouble())
    //       > kMaxTurretRateForVisionRadPerSec.get()
    //   - RobotState.getInstance().getPoseAt(timestamp) is empty (too old to correlate)
    // Otherwise compute std devs with computeStdDevs(...) and call
    // RobotState.getInstance().addVisionObservation(...).
    // COUNT AND LOG every rejection with its reason under Vision/<source>/Rejections/<reason>. A
    // silently-rejecting pipeline is indistinguishable from a pipeline that is not running.
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
    // TODO: xy = kMegaTag2XyStdDevBase.get() * averageDistance^2 / tagCount;
    //       theta = kMegaTag2ThetaStdDev.get();
    // The theta value is left deliberately enormous: MegaTag2 CONSUMES your heading to solve, so
    // feeding its heading output back into the estimator creates a self-reinforcing feedback loop
    // that slowly rotates your whole field frame.
    // Return VecBuilder.fill(xy, xy, theta).
    //
    // The placeholder below returns effectively-infinite covariance so that, until this is written,
    // observations are harmless rather than trusted.
    return VecBuilder.fill(9999.0, 9999.0, 9999.0);
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
    // TODO: Return the largest-area TargetObservation from intakeInputs, but only while the intake
    // camera is on its game-piece pipeline. Consumed by DriveCommands.gamePieceAssist.
    return Optional.empty();
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
    // TODO: intakeCameraIO.setPipeline(index).
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
    // TODO: return acceptedCount.
    return 0;
  }
}
