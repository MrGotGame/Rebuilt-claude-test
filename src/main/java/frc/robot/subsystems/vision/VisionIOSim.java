package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Random;
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
   * Seeded deliberately, so a failing test reproduces.
   *
   * <p>An unseeded Random would make a covariance or rejection test fail one run in twenty and pass
   * the retry, which this project has already learned to read as a bug report rather than as
   * flakiness -- see the Testing standard in PROJECT_OVERVIEW.md.
   */
  private final Random random = new Random(0L);

  private int tagCount = 2;
  private double averageTagDistanceMeters = 2.0;
  private double ambiguity = 0.05;
  private double latencySec = 0.04;
  private Transform3d poseOffset = new Transform3d();
  private boolean connected = true;

  /**
   * Test hook that shapes the next observations.
   *
   * @param tagCount how many tags to claim contributed; 0 emits an observation with no tags
   * @param averageDistanceMeters mean tag distance to report, in meters
   * @param ambiguity single-tag solve ambiguity to report
   *     <p><b>Outputs:</b> stores the values used by the next {@link #updateInputs}.
   */
  public void setSolveQuality(int tagCount, double averageDistanceMeters, double ambiguity) {
    this.tagCount = tagCount;
    this.averageTagDistanceMeters = averageDistanceMeters;
    this.ambiguity = ambiguity;
  }

  /**
   * Test hook that displaces the reported pose from ground truth.
   *
   * @param offset the transform to apply to truth before reporting, which is how an outlier or a
   *     nonsense Z height is injected
   *     <p><b>Outputs:</b> stores the offset.
   */
  public void setPoseOffset(Transform3d offset) {
    this.poseOffset = offset;
  }

  /**
   * Test hook that simulates the camera dropping off the network entirely.
   *
   * @param connected false to report the camera as disconnected
   *     <p><b>Outputs:</b> stores the flag. Distinct from blackout, which is a connected camera
   *     that can see nothing.
   */
  public void setConnected(boolean connected) {
    this.connected = connected;
  }

  /**
   * Test hook that sets the simulated capture latency.
   *
   * @param seconds latency to subtract from the current time when timestamping an observation;
   *     large values are how a too-old observation is injected
   *     <p><b>Outputs:</b> stores the latency.
   */
  public void setLatency(double seconds) {
    this.latencySec = seconds;
  }

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
    // IMPLEMENTATION (was TODO: "noiseStdDevMeters = stdDevMeters.")
    noiseStdDevMeters = stdDevMeters;
  }

  /**
   * Test hook that simulates a camera losing every tag.
   *
   * @param blackout true to emit no observations
   *     <p><b>Outputs:</b> stores the blackout flag, so the fallback rules can be watched working
   *     in simulation before they are trusted at an event.
   */
  public void setBlackout(boolean blackout) {
    // IMPLEMENTATION (was TODO: "forcedBlackout = blackout.")
    forcedBlackout = blackout;
  }

  /**
   * Generates simulated observations for this loop.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "If forcedBlackout, emit no observations but still report
    // connected. Otherwise load the tag layout with AprilTagFieldLayout.loadField(...), transform
    // each tag into camera space, keep the ones inside a tunable FOV and range, and emit one
    // PoseObservation with Gaussian noise.")
    //
    // DELIBERATE DEVIATION, and the reason matters. The TODO's own second marker asks whoever
    // implements this to CONFIRM the 2026 REBUILT field enum constant in WPILib -- and the
    // REBUILT tag layout is exactly the kind of field-dependent value this project forbids
    // guessing at, which is why kFieldLengthMeters is still 0.0 and every goal coordinate carries
    // a TODO: MEASURE.
    //
    // Frustum culling against a tag layout nobody has would add a large, untestable dependency in
    // exchange for realism this milestone cannot use. What M7 actually needs from a simulated
    // camera is the ability to INJECT a specific failure and watch the rejection ladder respond --
    // which is what the TODO's own class documentation says the sim is for. So the solve quality,
    // the pose offset, the latency and the connection state are all settable, and the pose is
    // derived from ground truth rather than re-solved from tags.
    //
    // TODO: once the REBUILT AprilTag layout is published, add real frustum culling on top of this.
    // The injection hooks should stay regardless -- they are how the rejection rules are tested.
    inputs.connected = connected;
    inputs.latencySec = latencySec;
    inputs.tagIds = new int[0];

    // A blacked-out camera is CONNECTED and sees nothing. Conflating the two hides real
    // disconnections behind "we must be looking at a blank wall".
    if (forcedBlackout || !connected) {
      inputs.poseObservations = new PoseObservation[0];
      inputs.targetObservations = new TargetObservation[0];
      inputs.hasTarget = false;
      return;
    }

    Pose2d truth = groundTruthPoseSupplier.get();
    Pose3d reported =
        new Pose3d(
                new edu.wpi.first.math.geometry.Translation3d(
                    truth.getX() + random.nextGaussian() * noiseStdDevMeters,
                    truth.getY() + random.nextGaussian() * noiseStdDevMeters,
                    0.0),
                new edu.wpi.first.math.geometry.Rotation3d(truth.getRotation()))
            .transformBy(poseOffset);

    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              Timer.getFPGATimestamp() - latencySec,
              reported,
              ambiguity,
              tagCount,
              averageTagDistanceMeters)
        };
    inputs.targetObservations = new TargetObservation[0];
    inputs.hasTarget = tagCount > 0;
  }

  /**
   * Stores the transform, so the sim honours the moving turret exactly like the real camera.
   *
   * @param robotToCamera the transform from the robot origin to the camera lens
   */
  @Override
  public void setRobotToCamera(Transform3d robotToCamera) {
    // IMPLEMENTATION (was TODO: "this.robotToCamera = robotToCamera.")
    //
    // Stored but not yet consumed, because the pose is derived from ground truth rather than
    // re-solved from tags in camera space. It is honoured here so that M8 -- which exists to catch
    // exactly a bad turret-transform composition -- has the value available the moment the sim
    // learns to solve from tags.
    this.robotToCamera = robotToCamera;
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
