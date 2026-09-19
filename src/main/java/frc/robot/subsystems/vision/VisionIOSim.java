package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;

/**
 * A deliberately simple ideal-camera simulation.
 *
 * <p>Given the ground-truth simulated pose and the field's tag layout, it reports which tags fall
 * inside the camera frustum and emits a noisy {@link PoseObservation}. That is enough to develop
 * and tune the fusion policy without adding PhotonVision as a dependency, and it lets you inject
 * exactly the failure you want to test: no tags, one tag at long range, or a 30 cm outlier.
 */
public class VisionIOSim implements VisionIO {
  private final String name;
  private final Supplier<Pose2d> groundTruthPoseSupplier;
  private Transform3d robotToCamera = new Transform3d();
  private double noiseStdDevMeters = 0.02;
  private boolean forcedBlackout = false;

  /**
   * Builds a simulated camera.
   *
   * @param name the camera's name, used only for log keys
   * @param groundTruthPoseSupplier source of truth for where the robot really is; the maple-sim
   *     ground truth if installed, otherwise {@code RobotState.getInstance()::getEstimatedPose},
   *     which makes the sim self-consistent but not independent -- good enough for state-machine
   *     development, not for tuning covariances
   */
  public VisionIOSim(String name, Supplier<Pose2d> groundTruthPoseSupplier) {
    this.name = name;
    this.groundTruthPoseSupplier = groundTruthPoseSupplier;
  }

  /**
   * Test hook that sets the injected measurement noise.
   *
   * @param stdDevMeters standard deviation of the Gaussian noise added to each observation
   *     <p><b>Outputs:</b> stores the noise level.
   */
  public void setNoise(double stdDevMeters) {
    // TODO: noiseStdDevMeters = stdDevMeters.
  }

  /**
   * Test hook that simulates a camera losing every tag.
   *
   * @param blackout true to emit no observations
   *     <p><b>Outputs:</b> stores the blackout flag, so the fallback rules can be watched working
   *     in simulation before they are trusted at an event.
   */
  public void setBlackout(boolean blackout) {
    // TODO: forcedBlackout = blackout.
  }

  /**
   * Generates simulated observations for this loop.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // TODO: If forcedBlackout, emit no observations but still report connected = true -- a blacked
    // out camera is connected, and conflating the two hides real disconnections.
    // Otherwise: load the tag layout with AprilTagFieldLayout.loadField(...)
    // TODO: CONFIRM the 2026 REBUILT field enum constant name in WPILib 2026.
    // Transform each tag into camera space via robotToCamera, keep tags inside a tunable FOV and
    // range, and emit one PoseObservation at Timer.getFPGATimestamp() minus a simulated 40 ms
    // latency, with Gaussian noise of noiseStdDevMeters added.
  }

  /**
   * Stores the transform, so the sim honours the moving turret exactly like the real camera.
   *
   * @param robotToCamera the transform from the robot origin to the camera lens
   */
  @Override
  public void setRobotToCamera(Transform3d robotToCamera) {
    // TODO: this.robotToCamera = robotToCamera. Honouring this is what lets sim catch a bad
    // turret-transform composition before the robot does.
  }

  /**
   * No-op in simulation; the simulated solver does not need a heading seed.
   *
   * @param yaw the robot's field heading, ignored
   * @param yawRateRadPerSec the robot's yaw rate, ignored
   */
  @Override
  public void setRobotOrientation(Rotation2d yaw, double yawRateRadPerSec) {
    // Intentionally empty.
  }
}
