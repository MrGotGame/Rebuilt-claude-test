package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Talks to a Limelight 4 over NetworkTables. Two details here are load-bearing.
 *
 * <p>First, MegaTag2 requires the robot orientation to be written <b>before</b> the botpose entry
 * is read, every loop, or the camera solves against a stale heading.
 *
 * <p>Second, for the turret camera the {@code camerapose_robotspace} entry must be rewritten every
 * loop from the live turret angle. This implementation also predicts one loop ahead to cancel the
 * NetworkTables write latency, and {@code Vision} separately discards frames captured while the
 * turret was slewing quickly.
 *
 * <p><b>Dependency note.</b> {@code LimelightHelpers.java} is not a vendordep -- it is a single
 * file copied from {@code github.com/LimelightVision/limelightlib-wpijava}. This skeleton instead
 * reads the NetworkTables entries directly, so the project keeps zero extra dependencies. Either
 * approach is fine; pick one, do it once, and write down which you chose.
 */
public class VisionIOLimelight implements VisionIO {
  private final String name;
  private final NetworkTable table;
  private final DoubleArrayPublisher robotOrientationPublisher;
  private final DoubleArrayPublisher cameraPosePublisher;
  private final DoublePublisher pipelinePublisher;
  private final DoubleArraySubscriber megaTag2Subscriber;
  private final DoubleArraySubscriber megaTag1Subscriber;
  private final DoubleArraySubscriber targetSubscriber;
  private final IntegerSubscriber tagIdSubscriber;

  /**
   * Binds to one Limelight's NetworkTables entries.
   *
   * <p>NOTE: the publisher and subscriber construction is pre-filled because these fields are
   * final.
   *
   * @param name the Limelight's NetworkTables name, for example {@code "limelight-turret"}
   */
  public VisionIOLimelight(String name) {
    this.name = name;
    this.table = NetworkTableInstance.getDefault().getTable(name);
    this.robotOrientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    this.cameraPosePublisher = table.getDoubleArrayTopic("camerapose_robotspace_set").publish();
    this.pipelinePublisher = table.getDoubleTopic("pipeline").publish();
    this.megaTag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    this.megaTag1Subscriber =
        table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    this.targetSubscriber = table.getDoubleArrayTopic("tx").subscribe(new double[] {});
    this.tagIdSubscriber = table.getIntegerTopic("tid").subscribe(0);
    // TODO: VERIFY every NetworkTables key above against the Limelight 4 docs for YOUR firmware
    // version. Key names have changed between Limelight releases, and a wrong key fails silently as
    // "the camera never sees anything".
    // TODO: The targetSubscriber above is a placeholder -- you likely want separate subscriptions
    // for "tv", "tx", "ty" and "ta" rather than one array topic.
  }

  /**
   * Drains queued pose and target samples.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // TODO: inputs.connected = the botpose subscriber has received a value within a 250 ms window.
    // For each queued botpose_orb_wpiblue sample, the array layout is
    //   [x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgTagDist, avgTagArea]
    // so build a PoseObservation with
    //   timestampSec = sampleTimestampMicros / 1e6 - latencyMs / 1000.0
    // Collect TargetObservations from tx/ty/ta whenever tv == 1.
    // TODO: VERIFY the array layout against your firmware -- it has changed between releases, and
    // reading the wrong index gives a plausible-looking but wrong pose.
  }

  /**
   * Pushes the robot-to-camera transform to the camera.
   *
   * @param robotToCamera the transform from the robot origin to the camera lens
   */
  @Override
  public void setRobotToCamera(Transform3d robotToCamera) {
    // TODO: cameraPosePublisher.set(new double[] {x, y, z, rollDeg, pitchDeg, yawDeg}).
    // NOTE THE UNITS: the Limelight expects DEGREES here, while WPILib's Rotation3d is in RADIANS.
    // This is the single most common turret-camera bug, and it presents as a pose that is wrong by
    // an amount that grows with turret angle.
  }

  /**
   * Pushes the robot heading MegaTag2 needs.
   *
   * @param yaw the robot's field heading
   * @param yawRateRadPerSec the robot's yaw rate, in radians per second
   */
  @Override
  public void setRobotOrientation(Rotation2d yaw, double yawRateRadPerSec) {
    // TODO: robotOrientationPublisher.set(new double[] {yaw.getDegrees(),
    //     Units.radiansToDegrees(yawRateRadPerSec), 0, 0, 0, 0});
    // then NetworkTableInstance.getDefault().flush() so the camera sees it BEFORE it solves this
    // frame. Without the flush the heading can arrive a frame late, which is exactly the stale-
    // heading failure MegaTag2 is most sensitive to.
  }

  /**
   * Switches the camera pipeline.
   *
   * @param index the pipeline index to select
   */
  @Override
  public void setPipeline(int index) {
    // TODO: pipelinePublisher.set(index). Used to switch the intake camera between AprilTag and
    // game-piece pipelines.
  }

  /**
   * Forces the camera LEDs on or off.
   *
   * @param on true to force the LEDs on
   */
  @Override
  public void setLeds(boolean on) {
    // TODO: Write 3 (force on) or 1 (force off) to the "ledMode" entry.
  }
}
