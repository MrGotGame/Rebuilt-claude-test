package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.function.Supplier;

/**
 * Simulated QuestNav headset, with injectable tracking loss and drift so the fallback and re-seed
 * logic can be exercised on a laptop.
 *
 * <p><b>It reports in the headset's own frame, not the field's.</b> That is the entire difficulty
 * QuestNav presents, and a sim that skipped it would make seeding untestable: a real headset's
 * origin is wherever it happened to be when tracking started, so its poses mean nothing until the
 * robot tells it where it is. This class keeps an explicit {@link #questOriginInField} that starts
 * at an arbitrary, deliberately wrong place and is only corrected by {@link #setPose(Pose3d)}.
 *
 * <p>Two failure modes can therefore be injected honestly rather than approximated: SLAM drift,
 * which accumulates in the reported pose and is what the re-seed rule exists to catch, and loss of
 * tracking, which is what the trust gate exists to catch.
 */
public class QuestNavIOSim implements QuestNavIO {
  private final Supplier<Pose2d> groundTruthPoseSupplier;

  /**
   * Robot origin to the headset, the mounting the sim treats as physically true.
   *
   * <p>Taken as a constructor argument rather than read from {@link Constants} for the reason M8
   * recorded about the camera mountings: the shipped constant is still the identity transform, and
   * a headset sitting exactly on the robot origin makes a composition-direction error invisible --
   * applying the transform forwards and backwards give the same answer when the transform is
   * nothing. A test supplies a real offset so the direction is actually under test.
   */
  private final Transform3d robotToQuest;

  /**
   * Where the headset's private coordinate frame sits in the field.
   *
   * <p>Replaces the skeleton's {@code seedOffset}, which named a quantity rather than the thing it
   * describes. This is the pose of the Quest's origin expressed in field coordinates; the reported
   * pose is the true headset pose taken relative to it. Seeding is the act of moving this to
   * wherever it must be for the next report to read as the commanded pose.
   *
   * <p>It starts somewhere arbitrary and wrong on purpose. A headset that happened to boot already
   * aligned to the field would let an unseeded pose fuse correctly, and the test asserting that
   * unseeded frames are refused would pass whether or not the check existed.
   */
  private Pose3d questOriginInField =
      new Pose3d(new Translation3d(-2.5, 1.75, 0.0), new Rotation3d(0.0, 0.0, 0.9));

  private boolean trackingForced = true;
  private boolean connectedForced = true;
  private int batteryPercent = 100;
  private int trackingLostCounter = 0;
  private int frameCount = 0;
  private double driftMetersPerSec = 0.0;
  private double accumulatedDriftMeters = 0.0;
  private double lastUpdateSec = -1.0;

  /** Latency from exposure to the pose reaching robot code, in seconds. */
  private static final double kSimulatedLatencySec = 0.01;

  /**
   * Builds a simulated headset on the shipped mounting constant.
   *
   * @param groundTruthPoseSupplier source of truth for where the robot really is
   */
  public QuestNavIOSim(Supplier<Pose2d> groundTruthPoseSupplier) {
    this(groundTruthPoseSupplier, Constants.Vision.kRobotToQuest);
  }

  /**
   * Builds a simulated headset on an explicit mounting.
   *
   * @param groundTruthPoseSupplier source of truth for where the robot really is
   * @param robotToQuest the true robot-to-headset transform this simulated headset is mounted with
   */
  public QuestNavIOSim(Supplier<Pose2d> groundTruthPoseSupplier, Transform3d robotToQuest) {
    this.groundTruthPoseSupplier = groundTruthPoseSupplier;
    this.robotToQuest = robotToQuest;
  }

  /**
   * Test hook that drops or restores tracking on demand.
   *
   * @param tracking false to simulate lost tracking
   *     <p><b>Outputs:</b> stores the flag and, on a loss, increments the tracking-lost counter so
   *     the trust gate's "counter increased this loop" clause can be exercised.
   */
  public void setTracking(boolean tracking) {
    // IMPLEMENTATION (was TODO: "trackingForced = tracking.")
    //
    // The counter is bumped on the falling EDGE, matching the headset, because the trust gate
    // watches for the counter increasing rather than for the flag being false. Those differ in the
    // case that matters most: a headset that loses and regains tracking between two robot loops
    // reports tracking == true at both samples, and only the counter shows anything happened.
    if (trackingForced && !tracking) {
      trackingLostCounter++;
    }
    trackingForced = tracking;
  }

  /**
   * Test hook that simulates the headset dropping off the network.
   *
   * @param connected false to report the headset as disconnected
   *     <p><b>Outputs:</b> stores the flag. Distinct from losing tracking, which is a connected
   *     headset that no longer knows where it is.
   */
  public void setConnected(boolean connected) {
    this.connectedForced = connected;
  }

  /**
   * Test hook that sets the reported battery level.
   *
   * @param percent battery percentage to report
   *     <p><b>Outputs:</b> stores the value.
   */
  public void setBatteryPercent(int percent) {
    this.batteryPercent = percent;
  }

  /**
   * Test hook that injects slow positional drift.
   *
   * @param metersPerSec drift rate, in meters per second
   *     <p><b>Outputs:</b> stores the drift rate, so the re-seed logic is exercised in simulation.
   */
  public void setDrift(double metersPerSec) {
    // IMPLEMENTATION (was TODO: "driftMetersPerSec = metersPerSec.")
    driftMetersPerSec = metersPerSec;
  }

  /**
   * Emits one simulated pose frame per loop.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "Emit one pose at Timer.getFPGATimestamp() - 0.01, equal to the
    // ground-truth pose composed with kRobotToQuest and seedOffset, plus accumulated drift.")
    double now = Timer.getFPGATimestamp();
    if (lastUpdateSec >= 0.0 && now > lastUpdateSec) {
      accumulatedDriftMeters += driftMetersPerSec * (now - lastUpdateSec);
    }
    lastUpdateSec = now;

    inputs.connected = connectedForced;
    inputs.tracking = connectedForced && trackingForced;
    inputs.batteryPercent = batteryPercent;
    inputs.trackingLostCounter = trackingLostCounter;

    // A headset that is not tracking publishes no poses. Emitting a stale one instead would be the
    // more "helpful" choice and is exactly wrong: the estimator would keep receiving a pose that
    // no longer follows the robot, at QuestNav's very tight covariance, and would follow it.
    if (!inputs.connected || !inputs.tracking) {
      inputs.questPoses = new Pose3d[0];
      inputs.questPoseTimestamps = new double[0];
      inputs.frameCount = frameCount;
      return;
    }

    Pose3d trueHeadsetInField = new Pose3d(groundTruthPoseSupplier.get()).transformBy(robotToQuest);

    // The report is the true pose expressed in the headset's OWN frame. Until a seed has moved
    // questOriginInField onto the field origin, this is an arbitrary pose with no field meaning --
    // which is the property the "unseeded frames are refused" test depends on.
    Pose3d reported = trueHeadsetInField.relativeTo(questOriginInField);

    // Drift is applied after the frame change, because SLAM drift is an error in the headset's own
    // estimate of its own motion, not a change in where its origin sits.
    reported =
        reported.transformBy(
            new Transform3d(new Translation3d(accumulatedDriftMeters, 0.0, 0.0), new Rotation3d()));

    frameCount++;
    inputs.frameCount = frameCount;
    inputs.questPoses = new Pose3d[] {reported};
    inputs.questPoseTimestamps = new double[] {now - kSimulatedLatencySec};
  }

  /**
   * Re-seeds the simulated headset origin.
   *
   * @param fieldToQuest the pose the next report should equal
   *     <p><b>Outputs:</b> moves {@code questOriginInField} so the next report reads as {@code
   *     fieldToQuest}, and clears the accumulated drift.
   */
  @Override
  public void setPose(Pose3d fieldToQuest) {
    // IMPLEMENTATION (was TODO: "Recompute seedOffset so the next reported pose equals
    // fieldToQuest exactly, and zero the accumulated drift.")
    //
    // Solving for the origin rather than storing an offset: the report is defined as
    //     reported = trueHeadsetInField relativeTo questOriginInField
    // so requiring reported == fieldToQuest gives
    //     questOriginInField (+) fieldToQuest = trueHeadsetInField
    // and therefore questOriginInField = trueHeadsetInField (-) fieldToQuest.
    //
    // Note what this does NOT do: it does not make the headset correct. It makes the headset agree
    // with the pose it was handed. If the caller seeds from a poor estimate, that error is written
    // into the headset's frame and persists until the next seed -- which is precisely why section
    // 7.6 only permits a re-seed while the robot is nearly stationary with at least two tags in
    // view, and why this sim reproduces the behaviour rather than snapping to truth.
    Pose3d trueHeadsetInField = new Pose3d(groundTruthPoseSupplier.get()).transformBy(robotToQuest);
    questOriginInField =
        trueHeadsetInField.transformBy(new Transform3d(Pose3d.kZero, fieldToQuest).inverse());
    accumulatedDriftMeters = 0.0;
  }

  /** No-op in simulation. */
  @Override
  public void commandPeriodic() {
    // Intentionally empty. The real implementation delivers queued commands and fires callbacks
    // here; the sim applies setPose synchronously, so there is nothing pending.
  }
}
