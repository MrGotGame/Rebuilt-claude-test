package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.NavigableMap;
import java.util.Random;
import java.util.TreeMap;
import java.util.function.Supplier;

/**
 * A deliberately simple ideal-camera simulation.
 *
 * <p>Given the ground-truth simulated pose and the field's tag layout, it reports which tags fall
 * inside the camera frustum and emits a noisy {@link PoseObservation}. That is enough to develop
 * and tune the fusion policy without adding PhotonVision as a dependency, and it lets you inject
 * exactly the failure you want to test: no tags, one tag at long range, or a 30 cm outlier.
 *
 * <p><b>It solves the way a Limelight solves.</b> A real camera recovers the field pose of its own
 * lens from the tags, and then subtracts whatever robot-to-camera transform was last written to
 * {@code camerapose_robotspace_set} to report a robot pose. Those two transforms are the same only
 * when the pushed value is correct; for a turret-mounted camera they routinely differ, and the
 * difference is the whole subject of M8. So this sim keeps them separate: it composes the field
 * pose of the lens using the <i>true</i> mounting, and un-composes it using the <i>pushed</i> one.
 *
 * <p>Deriving the reported pose straight from ground truth instead -- which is what this class did
 * through M7 -- makes every turret-transform test vacuous. The camera would report the right answer
 * no matter how wrong the transform was, the checkpoint would pass, and the composition order would
 * ship unverified. That is precisely the shape of vacuous pass the project's testing standard
 * lists.
 */
public class VisionIOSim implements VisionIO {
  private final String name;
  private final Supplier<Pose2d> groundTruthPoseSupplier;
  private Transform3d robotToCamera = new Transform3d();
  private double noiseStdDevMeters = 0.02;
  private boolean forcedBlackout = false;

  /**
   * Where the lens physically is, as opposed to where the robot code told the camera it is.
   *
   * <p>Defaults to the pushed transform, which makes the two agree and reproduces the pre-M8
   * behaviour exactly: a camera whose mounting is always exactly what was last published reports
   * ground truth. A turret test overrides it with the real, live mounting so that a stale or
   * mis-composed push shows up as a wrong pose, which is the only way the correction in {@code
   * Vision.processCamera} can be shown to be doing anything.
   */
  private Supplier<Transform3d> trueRobotToCameraSupplier = null;

  /**
   * What the mounting and the pushed transform were at one past instant.
   *
   * @param trueRobotToCamera where the lens actually was
   * @param pushedRobotToCamera what the camera had most recently been told, and therefore what it
   *     subtracted when it reported a robot pose
   */
  private record MountSample(Transform3d trueRobotToCamera, Transform3d pushedRobotToCamera) {}

  /**
   * Mounting history, so an observation can be built for the instant it was captured.
   *
   * <p>A frame reported now was exposed {@code latencySec} ago, and both transforms have moved
   * since. Using this loop's values for a frame captured four loops back would hand the sim the
   * perfect knowledge the real system does not have, and would quietly remove the timing error the
   * capture-time correction exists to undo.
   */
  private final NavigableMap<Double, MountSample> mountHistory = new TreeMap<>();

  /** How much mounting history to retain, matching {@code RobotState}'s buffer depth, seconds. */
  private static final double kMountHistorySec = 2.0;

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
   * Test hook that separates where the lens really is from where the code says it is.
   *
   * @param supplier the live, true robot-to-camera transform, or null to go back to trusting the
   *     pushed value
   *     <p><b>Outputs:</b> stores the supplier, which is sampled once per {@link #updateInputs}.
   *     Until a test sets this, the sim behaves exactly as it did for M7.
   */
  public void setTrueRobotToCamera(Supplier<Transform3d> supplier) {
    this.trueRobotToCameraSupplier = supplier;
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

    // The mounting is sampled BEFORE the observation is built, so this loop's push is available to
    // a frame captured this loop, and pruned to the same depth RobotState buffers poses.
    double now = Timer.getFPGATimestamp();
    mountHistory.put(
        now,
        new MountSample(
            trueRobotToCameraSupplier == null ? robotToCamera : trueRobotToCameraSupplier.get(),
            robotToCamera));
    mountHistory.headMap(now - kMountHistorySec, false).clear();

    Pose2d truth = groundTruthPoseSupplier.get();
    Pose3d truePose =
        new Pose3d(
            new edu.wpi.first.math.geometry.Translation3d(
                truth.getX() + random.nextGaussian() * noiseStdDevMeters,
                truth.getY() + random.nextGaussian() * noiseStdDevMeters,
                0.0),
            new edu.wpi.first.math.geometry.Rotation3d(truth.getRotation()));

    // The capture instant, which is what the timestamp below reports and what the mounting lookup
    // must agree with. floorEntry rather than an interpolation: the pushed transform is a step
    // function -- the camera holds the last value it received until the next one arrives -- and
    // interpolating a step would invent a mounting the camera never used.
    double captureTimeSec = now - latencySec;
    var mountEntry = mountHistory.floorEntry(captureTimeSec);
    MountSample mount =
        mountEntry != null ? mountEntry.getValue() : mountHistory.firstEntry().getValue();

    // The solve, in the two steps a Limelight actually performs. Step one is real: the tags fix the
    // LENS in the field, which is why this uses the true mounting. Step two is arithmetic the
    // camera does on the robot's behalf using the number the robot gave it, which is why it uses
    // the pushed mounting. When they disagree, the reported robot pose is wrong by exactly that
    // disagreement -- and on a turret at 12 rad/s they disagree by most of a metre.
    Pose3d fieldToCamera = truePose.transformBy(mount.trueRobotToCamera());
    Pose3d reported = fieldToCamera.transformBy(mount.pushedRobotToCamera().inverse());

    // Applied last, after the solve, because this hook injects a FAULT in the answer -- a nonsense
    // height, a teleport -- rather than a change in the geometry. Folding it in earlier would let
    // the correction partially undo it and weaken the M7 rejection tests.
    reported = reported.transformBy(poseOffset);

    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              captureTimeSec, reported, ambiguity, tagCount, averageTagDistanceMeters)
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
    // M8 made this load-bearing. The value is now what updateInputs SUBTRACTS to turn the lens's
    // field pose into a reported robot pose, exactly as the camera does with
    // camerapose_robotspace_set -- so pushing a wrong transform here produces a wrong pose here,
    // which is the only reason a turret-transform test can fail.
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
