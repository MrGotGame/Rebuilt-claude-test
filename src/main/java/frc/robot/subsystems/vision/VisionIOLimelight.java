package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

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

  // IMPLEMENTATION (was TODO: "The targetSubscriber above is a placeholder -- you likely want
  // separate subscriptions for tv, tx, ty and ta rather than one array topic.")
  //
  // Added rather than replacing targetSubscriber, because that field is final and pre-filled by the
  // skeleton. These four are what getBestGamePieceTarget actually reads.
  private final edu.wpi.first.networktables.DoubleSubscriber tvSubscriber;
  private final edu.wpi.first.networktables.DoubleSubscriber txSubscriber;
  private final edu.wpi.first.networktables.DoubleSubscriber tySubscriber;
  private final edu.wpi.first.networktables.DoubleSubscriber taSubscriber;

  /** How long without a botpose sample before the camera is called disconnected, in seconds. */
  private static final double kDisconnectTimeoutSec = 0.25;

  /** Minimum botpose array length for the layout decoded in {@link #updateInputs}. */
  private static final int kExpectedBotposeLength = 11;

  private double lastPoseReceivedSec = 0.0;

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
    this.tvSubscriber = table.getDoubleTopic("tv").subscribe(0.0);
    this.txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    this.tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    this.taSubscriber = table.getDoubleTopic("ta").subscribe(0.0);
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
    // IMPLEMENTATION (was TODO: "connected = a botpose value within 250 ms; for each queued
    // botpose_orb_wpiblue sample decode [x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan,
    // avgTagDist, avgTagArea] and build a PoseObservation; collect TargetObservations from tx/ty/ta
    // whenever tv == 1.")
    //
    // readQueue() rather than get(): the camera publishes faster than the robot loop, so a plain
    // get() silently discards every frame but the newest. Draining the queue is what lets the
    // estimator apply several observations per loop, each at its own capture time.
    var queue = megaTag2Subscriber.readQueue();

    // Connection is judged on RECENCY of data, not on the topic existing. A Limelight that has
    // been unplugged leaves its NetworkTables entries behind, so an existence check reports a dead
    // camera as healthy for the rest of the match.
    if (queue.length > 0) {
      lastPoseReceivedSec = Timer.getFPGATimestamp();
    }
    inputs.connected = Timer.getFPGATimestamp() - lastPoseReceivedSec < kDisconnectTimeoutSec;

    List<PoseObservation> observations = new ArrayList<>();
    for (var sample : queue) {
      double[] v = sample.value;

      // A short array means a firmware whose layout does not match the one decoded below. Skipping
      // is deliberate: reading past the end would throw, and guessing at the missing fields would
      // produce a plausible-looking but wrong pose, which is worse than no pose.
      if (v.length < kExpectedBotposeLength) {
        continue;
      }

      double latencyMs = v[6];
      int tagCount = (int) v[7];
      double avgTagDist = v[9];

      // The Limelight timestamps its sample when it PUBLISHES, so the capture instant is that
      // moment minus the pipeline latency it reports. Getting this wrong does not look like an
      // error -- it looks like a pose estimate that is subtly wrong only while moving.
      double timestampSec = sample.timestamp / 1.0e6 - latencyMs / 1000.0;

      observations.add(
          new PoseObservation(
              timestampSec,
              new edu.wpi.first.math.geometry.Pose3d(
                  new edu.wpi.first.math.geometry.Translation3d(v[0], v[1], v[2]),
                  new edu.wpi.first.math.geometry.Rotation3d(
                      Math.toRadians(v[3]), Math.toRadians(v[4]), Math.toRadians(v[5]))),
              // MegaTag2 does not publish a per-solve ambiguity; it is a multi-tag orientation-
              // seeded solve. Reporting 0.0 is honest, and Vision only consults ambiguity when
              // tagCount == 1 anyway.
              0.0,
              tagCount,
              avgTagDist));
    }
    inputs.poseObservations = observations.toArray(new PoseObservation[0]);

    boolean hasTarget = tvSubscriber.get() > 0.5;
    inputs.hasTarget = hasTarget;
    inputs.targetObservations =
        hasTarget
            ? new TargetObservation[] {
              new TargetObservation(
                  Timer.getFPGATimestamp(),
                  Rotation2d.fromDegrees(txSubscriber.get()),
                  Rotation2d.fromDegrees(tySubscriber.get()),
                  taSubscriber.get())
            }
            : new TargetObservation[0];

    long tid = tagIdSubscriber.get();
    inputs.tagIds = tid > 0 ? new int[] {(int) tid} : new int[0];
    inputs.latencySec = queue.length > 0 ? queue[queue.length - 1].value[6] / 1000.0 : 0.0;

    // TODO: VERIFY the botpose array layout against your firmware before trusting a pose on the
    // field. It has changed between Limelight releases, and reading the wrong index gives a
    // plausible-looking but wrong pose. The length guard above turns a layout change into "no
    // observations" rather than into a confident lie, but it cannot detect a REORDERED layout of
    // the same length.
  }

  /**
   * Pushes the robot-to-camera transform to the camera.
   *
   * @param robotToCamera the transform from the robot origin to the camera lens
   */
  @Override
  public void setRobotToCamera(Transform3d robotToCamera) {
    // IMPLEMENTATION (was TODO: "cameraPosePublisher.set(new double[] {x, y, z, rollDeg,
    // pitchDeg, yawDeg}).")
    //
    // Degrees, not radians. The Limelight's robotspace entries are in degrees, and a radian value
    // here is accepted without complaint and solves against a camera pointing somewhere else.
    var t = robotToCamera.getTranslation();
    var r = robotToCamera.getRotation();
    cameraPosePublisher.set(
        new double[] {
          t.getX(),
          t.getY(),
          t.getZ(),
          Math.toDegrees(r.getX()),
          Math.toDegrees(r.getY()),
          Math.toDegrees(r.getZ())
        });
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
    // IMPLEMENTATION (was TODO: "robotOrientationPublisher.set(new double[] {yaw.getDegrees(),
    // ...}).")
    //
    // MegaTag2 uses only the yaw; the remaining five are required by the entry's shape and are
    // published as zero. The flush is what makes the ordering guarantee in the class documentation
    // real -- without it the write may not reach the camera before botpose is read this loop, and
    // every frame solves against last loop's heading.
    robotOrientationPublisher.set(
        new double[] {yaw.getDegrees(), Math.toDegrees(yawRateRadPerSec), 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault().flush();
  }

  /**
   * Switches the camera pipeline.
   *
   * @param index the pipeline index to select
   */
  @Override
  public void setPipeline(int index) {
    // IMPLEMENTATION (was TODO: "pipelinePublisher.set(index).")
    pipelinePublisher.set(index);
    // Used to switch the intake camera between AprilTag and
    // game-piece pipelines.
  }

  /**
   * Forces the camera LEDs on or off.
   *
   * @param on true to force the LEDs on
   */
  @Override
  public void setLeds(boolean on) {
    // IMPLEMENTATION (was TODO: "Write 3 (force on) or 1 (force off) to the ledMode entry.")
    table.getEntry("ledMode").setNumber(on ? 3 : 1);
  }
}
