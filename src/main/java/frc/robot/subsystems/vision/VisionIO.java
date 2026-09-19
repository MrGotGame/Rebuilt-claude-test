package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * One interface for both Limelight 4s.
 *
 * <p>{@link PoseObservation} deliberately carries everything the fusion policy needs to assign a
 * covariance -- tag count, average tag distance, ambiguity, and the capture timestamp -- rather
 * than a bare pose. A single-tag fix at 6 m and a four-tag fix at 2 m deserve wildly different
 * trust, and a bare pose throws away exactly the information needed to tell them apart.
 */
public interface VisionIO {
  /**
   * One absolute pose estimate from a camera.
   *
   * @param timestampSec FPGA capture time, already latency-compensated, in seconds
   * @param pose the estimated field pose, blue-origin
   * @param ambiguity single-tag solve ambiguity; meaningless when {@code tagCount > 1}
   * @param tagCount how many tags contributed
   * @param averageTagDistanceMeters mean distance to the contributing tags, in meters
   */
  public record PoseObservation(
      double timestampSec,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistanceMeters) {}

  /**
   * One target detection, used for game-piece tracking on the intake camera.
   *
   * @param timestampSec FPGA capture time, in seconds
   * @param tx horizontal offset from the crosshair
   * @param ty vertical offset from the crosshair
   * @param area target area as a percentage of the image
   */
  public record TargetObservation(double timestampSec, Rotation2d tx, Rotation2d ty, double area) {}

  /** Everything a camera reports, captured once per loop and logged for replay. */
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public double latencySec = 0.0;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public TargetObservation[] targetObservations = new TargetObservation[0];
    public int[] tagIds = new int[0];
    public boolean hasTarget = false;
  }

  /**
   * Samples the camera into the input struct.
   *
   * @param inputs the struct to populate; every field must be written every call
   */
  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Pushes the current robot-to-camera transform so MegaTag2 can solve in robot space.
   *
   * <p>Must be called <b>every loop</b> for the turret camera, because that transform moves with
   * the turret.
   *
   * @param robotToCamera the transform from the robot origin to the camera lens
   */
  public default void setRobotToCamera(Transform3d robotToCamera) {}

  /**
   * Pushes the robot's current heading, which MegaTag2 requires before every pose read.
   *
   * @param yaw the robot's field heading, from the fused estimate rather than the raw gyro
   * @param yawRateRadPerSec the robot's yaw rate, in radians per second
   */
  public default void setRobotOrientation(Rotation2d yaw, double yawRateRadPerSec) {}

  /**
   * Switches the camera pipeline, for example between AprilTag and game-piece detection.
   *
   * @param index the pipeline index to select
   */
  public default void setPipeline(int index) {}

  /**
   * Forces the camera LEDs on or off.
   *
   * @param on true to force the LEDs on
   */
  public default void setLeds(boolean on) {}
}
