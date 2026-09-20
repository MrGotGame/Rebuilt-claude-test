package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.GeomUtil;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;
import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;
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

  /**
   * Robot origin to the turret's rotation axis, and that axis at 0 rad to the camera lens.
   *
   * <p>Held as fields rather than read from {@link Constants} at each use so that a test can build
   * this class with a mounting that has a real lever arm. Both constants ship as the identity
   * transform pending CAD measurement, and identity puts the lens exactly on the turret axis --
   * where rotating the turret moves the camera nowhere at all. A checkpoint asserting "the pose did
   * not move when the turret slewed" would then hold for every composition order including the
   * wrong ones, which is the definition of a vacuous pass.
   */
  private final Transform3d robotToTurretAxis;

  private final Transform3d turretToCamera;

  /**
   * Every robot-to-camera transform pushed to the turret camera, by the time it was pushed.
   *
   * <p>Section 7.2 of the plan speaks of recording "the transform it actually pushed" as a single
   * value, but one value is not enough. The camera holds whatever it was last sent until the next
   * write arrives, and a frame surfaces 60-120 ms -- three to six loops -- after it was exposed. By
   * the time an observation is processed, the single most recent push is three to six turret
   * positions newer than the one the camera actually used to solve it. Un-composing with the wrong
   * member of that sequence reintroduces most of the error this mechanism exists to remove.
   *
   * <p>So the pushes are kept as a history and looked up by capture time, and the lookup is {@code
   * floorEntry} rather than an interpolation, because the value in the camera is a step function
   * and interpolating it would invent a transform that was never in effect.
   */
  private final NavigableMap<Double, Transform3d> pushedTurretTransforms = new TreeMap<>();

  /** How much push history to keep, matched to {@code RobotState}'s buffer depth, in seconds. */
  private static final double kPushHistorySec = 2.0;

  /**
   * Robot origin to the Quest headset.
   *
   * <p>A field for the same reason the camera mountings are: the shipped constant is still the
   * identity transform, and a headset sitting exactly on the robot origin makes the composition
   * direction in {@link #processQuestNav()} untestable -- applying the mounting forwards and
   * backwards give the same answer when the mounting is nothing. Tests supply a real offset.
   *
   * <p>Section 7.6 calls this "the single largest error source in this subsystem", and the reason
   * is worth keeping next to the field: QuestNav carries the tightest covariance of any source, so
   * a two-centimetre error in this measurement is a two-centimetre bias that <i>wins</i> the fusion
   * against every camera that disagrees with it.
   */
  private final Transform3d robotToQuest;

  /**
   * How long the Quest-to-estimate residual has been over threshold, continuously.
   *
   * <p>A hold timer rather than an instantaneous comparison, because the residual spikes whenever a
   * camera corrects the estimate -- that is the estimate moving, not the headset drifting. Only a
   * residual that persists says the headset's frame has actually slipped.
   */
  private final Timer questResidualTimer = new Timer();

  private int acceptedCount = 0;
  private int rejectedCount = 0;
  private boolean questTrusted = false;

  /** Last tracking-lost counter seen, to detect a loss that happened between two loops. */
  private int lastTrackingLostCounter = -1;

  /** How many times the headset has been seeded, so re-seeds per match can be read off. */
  private int questSeedCount = 0;

  /** Distance between the Quest-derived robot pose and the fused estimate, in metres. */
  private double questResidualMeters = 0.0;

  /**
   * How many headset frames have been handed to the estimator.
   *
   * <p>Counted separately from the camera total because section 7.7's central rule is that an
   * untrusted source contributes <i>nothing</i> rather than contributing weakly, and only a
   * per-source count can tell those apart. A merged total falling is equally consistent with
   * QuestNav being dropped and with a camera losing sight of a tag.
   */
  private int questObservationsFused = 0;

  /** Why QuestNav is not trusted right now, or "Trusted". */
  private String questTrustReason = "NeverEvaluated";

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
    /**
     * The turret angle buffer does not reach back to the capture instant.
     *
     * <p>Rule 7 of section 7.5. Fires when the frame predates the two-second window, or when the
     * turret has not reported yet. What must never happen is the third option the plan calls out by
     * name: substituting the current turret angle and processing the frame anyway.
     */
    NO_TURRET_ANGLE,
    /**
     * The push log does not reach back to the capture instant.
     *
     * <p>The other half of the same reconstruction, counted separately rather than merged into
     * {@link #NO_TURRET_ANGLE}. Merging them was the first version, and mutation testing showed why
     * it was wrong: with one counter, a defect that made the turret-angle lookup answer for an
     * instant it cannot know was invisible, because the rejection simply reappeared under the same
     * name from the other half of the test. Two counters make the two failures distinguishable both
     * in a test and in the pit, where they have different causes -- a stale frame against a turret
     * that is not publishing.
     */
    NO_PUSHED_TRANSFORM,
    NO_CORRELATED_POSE,
    POSE_JUMP
  }

  /**
   * Rejection counters, per camera.
   *
   * <p>Per camera and not merged, because the two cameras fail for different reasons and a merged
   * counter cannot answer the question anyone actually asks in the pit -- "which camera is being
   * thrown away?". The turret camera is the only one that can register {@code TURRET_SLEWING} or
   * {@code NO_TURRET_ANGLE}, so a merged total would also make those two look like a property of
   * vision in general rather than of one mounting.
   */
  private final Map<RobotState.PoseSource, Map<RejectionReason, Integer>> rejectionsBySource =
      new EnumMap<>(RobotState.PoseSource.class);

  private final Alert intakeCameraDisconnectedAlert =
      new Alert("Intake Limelight disconnected", AlertType.kError);

  private final Alert turretCameraDisconnectedAlert =
      new Alert("Turret Limelight disconnected", AlertType.kError);

  /**
   * Fires while the turret camera's mounting is still unmeasured.
   *
   * <p>Both {@code kRobotToTurretAxis} and {@code kTurretToCamera} ship as the identity transform
   * with a {@code TODO: MEASURE from CAD}, and identity claims the lens sits exactly on the robot
   * origin. Every turret-camera pose is then biased by the real offset -- tens of centimetres --
   * and the bias rotates with the turret, so it reads as a pose that wanders whenever the shooter
   * tracks. Nothing in the code can detect this; only a tape measure can.
   *
   * <p>The observations are still fused rather than suppressed. Suppressing them would leave the
   * fusion path dead on the robot and make the M8 bring-up unperformable, and a silently inert
   * camera is the harder failure to notice of the two. Loud and live beats quiet and absent.
   */
  private final Alert turretTransformUnmeasuredAlert =
      new Alert(
          "Turret camera transform is unmeasured (kRobotToTurretAxis and kTurretToCamera are both"
              + " identity), so its poses carry the full camera offset as bias.",
          AlertType.kWarning);

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
   * Fires while QuestNav is connected but not contributing.
   *
   * <p>A warning, not an error, and the distinction is the plan's: the robot localises perfectly
   * well on MegaTag2 and odometry, and the headset only shortens the gap the robot can tolerate
   * between tag sightings. Badging a working robot red is how a drive team learns to ignore red.
   *
   * <p>The text is deliberately incomplete on its own -- the specific failing clause is in {@code
   * Vision/QuestTrustReason}, because there are six ways to lose trust and "QuestNav is not
   * trusted" is not actionable at 11pm before a competition.
   */
  private final Alert questNotTrustedAlert =
      new Alert("QuestNav is not being trusted; see Vision/QuestTrustReason.", AlertType.kWarning);

  /**
   * Fires when the headset battery is low enough to stop trusting it soon.
   *
   * <p>Section 7.7 is blunt that this is "a <i>last</i> warning, not the primary defense" -- the
   * headset should be on a powered feed from the robot, and seeing this alert at an event means
   * that feed is not working, not that someone forgot to charge it.
   */
  private final Alert questBatteryLowAlert =
      new Alert("QuestNav battery is low; check the powered USB-C feed.", AlertType.kWarning);

  /**
   * Fires while the headset mounting is still unmeasured.
   *
   * <p>The most consequential of the three unmeasured-transform alerts, because QuestNav carries
   * the tightest covariance of any source: a bias here is not averaged away by the cameras, it
   * overrides them.
   */
  private final Alert questTransformUnmeasuredAlert =
      new Alert(
          "kRobotToQuest is unmeasured (identity), so every QuestNav pose carries the headset's"
              + " mounting offset as bias -- and its tight covariance makes that bias win.",
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
    this(
        turretCameraIO,
        intakeCameraIO,
        questNavIO,
        turretAngleSupplier,
        turretVelocitySupplier,
        Constants.Vision.kRobotToTurretAxis,
        Constants.Vision.kTurretToCamera,
        Constants.Vision.kRobotToQuest);
  }

  /**
   * Builds the vision subsystem with explicit sensor mountings.
   *
   * <p>The robot uses the five-argument constructor, which supplies the mountings from {@link
   * Constants}. This one exists because those constants are all still the identity transform
   * awaiting a CAD measurement, and a transform test run against identity proves nothing: with the
   * lens on the turret's rotation axis, every composition order -- correct and backwards alike --
   * leaves the camera in the same place when the turret turns, and with the headset on the robot
   * origin, composing its mounting forwards and backwards give the same answer. A test needs a real
   * lever arm to have anything to measure.
   *
   * @param turretCameraIO the turret-mounted Limelight
   * @param intakeCameraIO the intake-mounted Limelight
   * @param questNavIO the Quest 3S headset
   * @param turretAngleSupplier live robot-relative turret angle, in radians
   * @param turretVelocitySupplier live turret angular velocity, in radians per second
   * @param robotToTurretAxis robot origin to the turret's rotation axis
   * @param turretToCamera the turret frame at 0 rad to the camera lens
   * @param robotToQuest robot origin to the Quest headset
   */
  public Vision(
      VisionIO turretCameraIO,
      VisionIO intakeCameraIO,
      QuestNavIO questNavIO,
      DoubleSupplier turretAngleSupplier,
      DoubleSupplier turretVelocitySupplier,
      Transform3d robotToTurretAxis,
      Transform3d turretToCamera,
      Transform3d robotToQuest) {
    this.turretCameraIO = turretCameraIO;
    this.intakeCameraIO = intakeCameraIO;
    this.questNavIO = questNavIO;
    this.turretAngleSupplier = turretAngleSupplier;
    this.turretVelocitySupplier = turretVelocitySupplier;
    this.robotToTurretAxis = robotToTurretAxis;
    this.turretToCamera = turretToCamera;
    this.robotToQuest = robotToQuest;
    // IMPLEMENTATION (was TODO: "Register FaultReporter checks ... QuestNav ones as WARNINGS.")
    //
    // The checks are Alerts driven from periodic(), for the reason recorded in
    // PhoenixUtil.tryUntilOk and Shooter: FaultReporter is a skeleton whose getInstance() returns
    // null and would throw here.
    //
    // All five are now registered. Each was deliberately withheld until the milestone that
    // actually reads its device -- the turret camera at M8, the headset at M9 -- because an alert
    // for hardware the code never consults sits permanently red on a healthy robot, which teaches
    // everyone to ignore the alerts widget, the one place M14 needs them to look.
    //
    // The QuestNav pair are WARNINGS while the Limelight pair are ERRORS, and the plan is explicit
    // about why: the robot shoots without the headset. MegaTag2 plus odometry is a complete
    // localisation solution; QuestNav only shortens the tolerable gap between tag sightings. An
    // error badge for a degraded-but-working robot is how a drive team learns to ignore red.
    for (RobotState.PoseSource source :
        new RobotState.PoseSource[] {
          RobotState.PoseSource.LIMELIGHT_TURRET, RobotState.PoseSource.LIMELIGHT_INTAKE
        }) {
      Map<RejectionReason, Integer> counters = new EnumMap<>(RejectionReason.class);
      for (RejectionReason reason : RejectionReason.values()) {
        counters.put(reason, 0);
      }
      rejectionsBySource.put(source, counters);
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
    // M9 completes the sequence: all eight steps, in the plan's order.
    //
    // Step 1 is first for a reason that is not obvious from its name. commandPeriodic() is what
    // delivers a queued setPose to the headset and what fires questnavlib's event callbacks;
    // running it after the frames are drained would delay every seed by a full loop, and a seed
    // that lands one loop late is a seed against a pose the robot has already left.
    questNavIO.commandPeriodic();

    // Steps 2 and 3 run before step 4 and that is the whole ordering rule: a Limelight solves the
    // frame it is about to publish using the orientation and the camera transform it has ALREADY
    // been sent. Reading botpose before writing them gets a pose solved against last loop's
    // heading and last loop's turret angle -- which is not an error, does not log, and simply
    // makes the estimate quietly worse whenever the robot is turning or the turret is tracking.
    pushRobotOrientation();
    pushTurretCameraTransform();

    intakeCameraIO.updateInputs(intakeInputs);
    Logger.processInputs("Vision/" + Constants.Vision.kIntakeCameraName, intakeInputs);

    turretCameraIO.updateInputs(turretInputs);
    Logger.processInputs("Vision/" + Constants.Vision.kTurretCameraName, turretInputs);

    questNavIO.updateInputs(questInputs);
    Logger.processInputs("Vision/QuestNav", questInputs);

    // Turret camera first, matching the plan's step order. Order between the two cameras does not
    // change the fused result -- the estimator sorts by timestamp internally -- but keeping it
    // fixed keeps the log deterministic when the same match is replayed.
    processCamera(turretInputs, RobotState.PoseSource.LIMELIGHT_TURRET, true);
    processCamera(intakeInputs, RobotState.PoseSource.LIMELIGHT_INTAKE, false);

    // Step 7, and the cameras genuinely must come first. The trust gate and the re-seed rule both
    // compare the headset against the fused estimate, and seeding writes that estimate into the
    // headset's frame -- so running QuestNav first would judge and seed it against an estimate
    // that is one loop stale, at exactly the moments the cameras have just corrected it.
    updateQuestNavTrust();
    processQuestNav();

    Logger.recordOutput("Vision/AcceptedObservations", acceptedCount);
    logRejections(RobotState.PoseSource.LIMELIGHT_TURRET, Constants.Vision.kTurretCameraName);
    logRejections(RobotState.PoseSource.LIMELIGHT_INTAKE, Constants.Vision.kIntakeCameraName);

    intakeCameraDisconnectedAlert.set(!intakeInputs.connected);
    turretCameraDisconnectedAlert.set(!turretInputs.connected);
    fieldBoundsUnmeasuredAlert.set(!fieldBoundsAreMeasured());
    turretTransformUnmeasuredAlert.set(!turretMountingIsMeasured());

    // Only while the headset is actually present. A team that does not run QuestNav, or runs it on
    // a robot where it is unplugged, must not be shown a permanent warning about a device they are
    // not using -- that is the same "teaches everyone to ignore alerts" failure the camera alerts
    // were withheld to avoid.
    questNotTrustedAlert.set(questInputs.connected && !questTrusted);
    questBatteryLowAlert.set(
        questInputs.connected
            && questInputs.batteryPercent >= 0
            && questInputs.batteryPercent <= Constants.Vision.kQuestMinBatteryPercent.get());
    questTransformUnmeasuredAlert.set(questInputs.connected && !questMountingIsMeasured());
  }

  /**
   * Publishes every rejection counter for one camera.
   *
   * @param source the camera whose counters to publish
   * @param cameraName that camera's NetworkTables name, which is also its log subtree
   *     <p><b>Outputs:</b> writes {@code Vision/<camera>/Rejections/<reason>} for every reason,
   *     including the ones still at zero.
   */
  private void logRejections(RobotState.PoseSource source, String cameraName) {
    for (Map.Entry<RejectionReason, Integer> entry : rejectionsBySource.get(source).entrySet()) {
      Logger.recordOutput(
          "Vision/" + cameraName + "/Rejections/" + entry.getKey(), entry.getValue());
    }
  }

  /**
   * Computes and pushes the turret camera's live transform, and records what was pushed.
   *
   * <p><b>Outputs:</b> writes the transform to the turret camera, appends it to the push history,
   * prunes history older than the buffer window, and logs the pushed value.
   */
  private void pushTurretCameraTransform() {
    // The push and the record are one operation on purpose. If they are ever separated, they will
    // eventually disagree -- and a push history that does not match what the camera was actually
    // sent produces a correction that is confidently wrong, which is strictly worse than no
    // correction at all, because the rejection ladder has nothing to catch it with.
    Transform3d pushed = robotToTurretCameraNow();
    turretCameraIO.setRobotToCamera(pushed);

    double now = Timer.getFPGATimestamp();
    pushedTurretTransforms.put(now, pushed);
    pushedTurretTransforms.headMap(now - kPushHistorySec, false).clear();

    Logger.recordOutput("Vision/" + Constants.Vision.kTurretCameraName + "/RobotToCamera", pushed);
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
    // IMPLEMENTATION (was TODO: "rotateAboutZ(kRobotToTurretAxis, angle + velocity * loopPeriod)
    // then .plus(kTurretToCamera). The one-loop velocity extrapolation cancels the NetworkTables
    // write latency.")
    //
    // The lead term is a PREDICTION, and the sign is what makes it one: the value being computed
    // now will not be in the camera until after the write lands, so the angle it should describe
    // is the angle the turret will have reached by then, not the one it has. Extrapolating the
    // wrong way doubles the error instead of cancelling it, and at a 12 rad/s cruise one loop is
    // 13.8 degrees of bearing -- large enough to matter, small enough that nobody would guess the
    // sign was the cause.
    //
    // The lead is deliberately approximate. It only has to remove most of the write latency;
    // whatever it leaves behind is removed exactly by the capture-time correction in
    // processCamera, which uses the angle the turret genuinely had. Two mechanisms, and the second
    // one does not depend on the first being good.
    return turretCameraTransformAt(
        turretAngleSupplier.getAsDouble()
            + turretVelocitySupplier.getAsDouble() * Constants.getLoopPeriodSec());
  }

  /**
   * Composes the turret camera transform for one turret angle.
   *
   * @param turretAngleRad robot-relative turret angle, in radians
   * @return the robot-to-camera transform the lens has at that angle
   */
  private Transform3d turretCameraTransformAt(double turretAngleRad) {
    // Factored out so the live push, the historical reconstruction and any future consumer cannot
    // drift apart. The whole correction rests on the two compositions being identical apart from
    // which angle they are handed; writing the chain twice is how they stop being identical.
    return GeomUtil.rotateAboutZ(robotToTurretAxis, turretAngleRad).plus(turretToCamera);
  }

  /**
   * The turret camera transform as it was at a past capture time.
   *
   * @param timestampSec FPGA capture time to reconstruct, in seconds
   * @return the historical robot-to-camera transform, or the identity transform if the turret
   *     buffer has no sample that old
   */
  public Transform3d robotToTurretCameraAt(double timestampSec) {
    // IMPLEMENTATION (was TODO: "Same composition, but using getTurretAngleAt(timestampSec). When
    // the buffer has NO sample, log a warning and return identity -- and the CALLER must then
    // reject that observation.")
    //
    // The signature is kept as the plan specifies it, but the identity sentinel is not how this
    // class consumes it. Identity is a perfectly legal transform, indistinguishable at the call
    // site from a camera genuinely mounted at the robot origin -- which, with kTurretToCamera
    // still unmeasured, is exactly the value a healthy call returns today. A caller that forgot to
    // check would therefore be indistinguishable from one that checked and passed.
    //
    // So the internal path uses turretCameraTransformAtCapture, which returns an Optional the
    // compiler makes impossible to ignore. This method stays for external and logging callers and
    // keeps the documented contract.
    Optional<Transform3d> transform = turretCameraTransformAtCapture(timestampSec);
    if (transform.isEmpty()) {
      Logger.recordOutput(
          "Vision/" + Constants.Vision.kTurretCameraName + "/TransformLookupMissedAt",
          timestampSec);
      return new Transform3d();
    }
    return transform.get();
  }

  /**
   * The turret camera transform at a past capture instant, or empty when it cannot be known.
   *
   * @param timestampSec FPGA capture time to reconstruct, in seconds
   * @return the historical robot-to-camera transform, or empty when the turret angle buffer does
   *     not reach back that far
   */
  private Optional<Transform3d> turretCameraTransformAtCapture(double timestampSec) {
    // Empty, never a fallback. Section 7.2 names "use the current angle as a fallback" as the bug
    // this entire buffer exists to prevent: the frame is 80 ms old precisely because the turret was
    // busy, so the current angle is the one value guaranteed to be wrong, and the resulting pose is
    // wrong without being implausible -- it lands on the field, at a sane height, within the jump
    // gate. Nothing downstream can catch it.
    return RobotState.getInstance()
        .getTurretAngleAt(timestampSec)
        .map(this::turretCameraTransformAt);
  }

  /**
   * What the turret camera had most recently been told its transform was, at a past instant.
   *
   * @param timestampSec FPGA capture time to look up, in seconds
   * @return the transform the camera was using then, or empty when no push is recorded that early
   */
  private Optional<Transform3d> pushedTurretTransformAt(double timestampSec) {
    // floorEntry, not the nearest entry and not an interpolation: the camera holds the last value
    // it received until the next write arrives, so the transform in effect at any instant is the
    // most recent push at or before it. Interpolating between two pushes would reconstruct a
    // transform the camera never held, and rounding to the nearest would sometimes pick one from
    // the future.
    var entry = pushedTurretTransforms.floorEntry(timestampSec);
    return entry == null ? Optional.empty() : Optional.of(entry.getValue());
  }

  /**
   * How fast the turret was slewing when a frame was captured.
   *
   * @param timestampSec FPGA capture time, in seconds
   * @return the turret's angular rate at that instant, in radians per second
   */
  private double turretRateAtCapture(double timestampSec) {
    // Derived from the angle buffer rather than read from the live velocity supplier, because the
    // live value describes the turret NOW and the question is about a frame exposed 60-120 ms ago.
    // A turret that has just finished a fast slew and stopped reports zero rate, and every
    // motion-blurred frame still in flight from that slew would be waved through by a live-value
    // check -- the exact frames rule 5 exists to discard, arriving at the exact moment the shooter
    // is settling onto a target and the estimate matters most.
    //
    // The fallback to the live value covers only the first loops after boot, when the buffer is
    // too shallow to difference. Those frames are then caught by NO_TURRET_ANGLE anyway.
    double loopPeriodSec = Constants.getLoopPeriodSec();
    Optional<Double> atCapture = RobotState.getInstance().getTurretAngleAt(timestampSec);
    Optional<Double> beforeCapture =
        RobotState.getInstance().getTurretAngleAt(timestampSec - loopPeriodSec);
    if (atCapture.isPresent() && beforeCapture.isPresent()) {
      return (atCapture.get() - beforeCapture.get()) / loopPeriodSec;
    }
    return turretVelocitySupplier.getAsDouble();
  }

  /**
   * Whether the turret camera's mounting has been measured yet.
   *
   * @return false while both mounting transforms are still the identity
   */
  private boolean turretMountingIsMeasured() {
    return !robotToTurretAxis.equals(new Transform3d())
        || !turretToCamera.equals(new Transform3d());
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
      Pose3d reportedPose = observation.pose();

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

      // ---- Turret-mounted only: the two rules and the correction that make a moving camera
      // usable. They run here, ahead of every geometric test, for two reasons. The blur rule is
      // free and discards the frame before any work is spent on it; and the geometric tests below
      // must judge the CORRECTED pose, because the pose the camera reported was computed about a
      // camera transform that has since been superseded and is not the robot's pose at all.
      if (isTurretMounted) {
        // A camera swinging with the turret smears its exposure across a changing transform.
        // MegaTag2's corner error inflates in a way no covariance model captures, so the frame is
        // discarded outright rather than down-weighted; the next one is 20 ms away.
        if (Math.abs(turretRateAtCapture(observation.timestampSec()))
            > Constants.Vision.kMaxTurretRateForVisionRadPerSec.get()) {
          reject(source, RejectionReason.TURRET_SLEWING);
          continue;
        }

        // The turret angle is checked FIRST, and the two halves are attributed separately. The
        // ordering is load-bearing: a defect in the angle lookup that made it answer for an
        // instant it cannot know would otherwise hide behind the push-log check, which misses for
        // the same frames and would report the rejection under the same name. Mutation testing
        // found exactly that -- the fallback the plan forbids survived with a merged counter.
        Optional<Transform3d> actualAtCapture =
            turretCameraTransformAtCapture(observation.timestampSec());
        if (actualAtCapture.isEmpty()) {
          reject(source, RejectionReason.NO_TURRET_ANGLE);
          continue;
        }

        Optional<Transform3d> pushedAtCapture = pushedTurretTransformAt(observation.timestampSec());
        if (pushedAtCapture.isEmpty()) {
          reject(source, RejectionReason.NO_PUSHED_TRANSFORM);
          continue;
        }

        // Section 7.2's two lines, and they are the reason M8 exists.
        //
        //   fieldToCamera      = reportedRobotPose (+) T_pushed
        //   correctedRobotPose = fieldToCamera     (-) T_actualAtCapture
        //
        // The first line undoes arithmetic the camera did on our behalf. A Limelight does not
        // measure a robot pose; it measures where its own LENS is, and then subtracts whatever
        // robot-to-camera transform we last sent it. Re-adding that same transform recovers the
        // only thing it actually observed, which is the lens in the field.
        //
        // The second line redoes the subtraction with the transform the turret genuinely had when
        // the shutter opened, read from the angle buffer. Whatever the push got wrong -- write
        // latency, an imperfect velocity lead, a turret that accelerated after the push -- is
        // removed here exactly, because the lens position is a measurement and both transforms are
        // just bookkeeping about the same instant.
        //
        // Skipping this and trusting the push is the failure the M8 checkpoint hunts: the
        // estimated pose swings back and forth in step with the turret, which reads as a vibrating
        // robot and gets blamed on odometry, on the tags, on the carpet -- on anything but the
        // camera that is pointing somewhere other than where it was told.
        Pose3d fieldToCamera = reportedPose.transformBy(pushedAtCapture.get());
        reportedPose = fieldToCamera.transformBy(actualAtCapture.get().inverse());
        Logger.recordOutput(
            "Vision/" + Constants.Vision.kTurretCameraName + "/CorrectedPose", reportedPose);
      }

      Pose2d pose = reportedPose.toPose2d();

      // A good fix puts the robot on the floor plane. Height is the cheapest proxy for "this solve
      // is geometric nonsense", and it catches tag misidentification, which otherwise produces a
      // confident pose somewhere else entirely.
      if (Math.abs(reportedPose.getZ()) > Constants.Vision.kMaxZErrorMeters.get()) {
        reject(source, RejectionReason.Z_ERROR);
        continue;
      }

      if (isOutsideField(pose)) {
        reject(source, RejectionReason.OUTSIDE_FIELD);
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
    Map<RejectionReason, Integer> counters = rejectionsBySource.get(source);
    if (counters != null) {
      counters.merge(reason, 1, Integer::sum);
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
    return getRejectionCount(RobotState.PoseSource.LIMELIGHT_INTAKE, reason);
  }

  /**
   * How many of one camera's observations have been rejected for a given reason.
   *
   * @param source which camera to query
   * @param reason the rejection reason to query, by its enum name
   * @return how many of that camera's observations were discarded for that reason
   */
  public int getRejectionCount(RobotState.PoseSource source, String reason) {
    Map<RejectionReason, Integer> counters = rejectionsBySource.get(source);
    return counters == null ? 0 : counters.getOrDefault(RejectionReason.valueOf(reason), 0);
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
    // IMPLEMENTATION (was TODO: the six clauses, pushed into RobotState, logging WHICH one failed.)
    //
    // Written as a cascade of named reasons rather than one boolean conjunction. A single `&&`
    // chain computes the same answer and throws away the only thing anyone needs at an event: not
    // THAT the headset is distrusted, but WHICH of six independent things is wrong. Those six have
    // six different fixes -- a cable, a battery, a mount, a tag, a re-seed, a replacement headset
    // -- and guessing between them in a six-minute queue is how a match is lost.
    //
    // Trust is BINARY, never a reduced weight, and section 7.7 is emphatic about it: "a degraded
    // source is dropped, never blended in weakly". A source that is wrong in an unknown way has no
    // correct covariance, so there is no honest number to down-weight it to.
    String reason = null;

    if (!questInputs.connected) {
      reason = "Disconnected";
    } else if (!questInputs.tracking) {
      reason = "NotTracking";
    } else if (questInputs.batteryPercent <= Constants.Vision.kQuestMinBatteryPercent.get()) {
      // Also catches the -1 the IO reports for a headset that will not say. An unknown battery is
      // not a healthy one.
      reason = "BatteryLow:" + questInputs.batteryPercent;
    } else if (lastTrackingLostCounter >= 0
        && questInputs.trackingLostCounter > lastTrackingLostCounter) {
      // The counter, not the flag, and this clause is the reason the counter exists. A headset
      // jarred by a defender can lose and regain tracking entirely between two robot loops, so
      // isTracking() reads true at both samples and the flag shows nothing. What the headset
      // reports after such an event is a pose whose frame may have shifted -- so the recovery is
      // not "resume", it is "re-seed", and this clause is what forces that.
      reason = "TrackingLostThisLoop";
    } else if (!RobotState.getInstance().isQuestSeeded()) {
      // An unseeded headset reports in its own arbitrary frame, whose origin is wherever it
      // happened to be when tracking started. Those poses are not wrong so much as meaningless.
      reason = "NotSeeded";
    } else if (questResidualTimer.isRunning()
        && questResidualTimer.get() > Constants.Vision.kQuestReseedHoldSec.get()) {
      // Sustained disagreement with the cameras. Not an instantaneous comparison: the residual
      // spikes every time a camera corrects the estimate, and that is the ESTIMATE moving, not the
      // headset drifting. Only a residual that persists past the hold means the headset's frame
      // has actually slipped.
      reason = "ResidualSustained:" + String.format("%.2f", questResidualMeters);
    }

    questTrusted = reason == null;
    questTrustReason = questTrusted ? "Trusted" : reason;
    lastTrackingLostCounter = questInputs.trackingLostCounter;

    RobotState.getInstance().setQuestTrusted(questTrusted);
    Logger.recordOutput("Vision/QuestTrusted", questTrusted);
    Logger.recordOutput("Vision/QuestTrustReason", questTrustReason);
    Logger.recordOutput("Vision/QuestResidualMeters", questResidualMeters);
    Logger.recordOutput("Vision/QuestSeedCount", questSeedCount);
    Logger.recordOutput("Vision/QuestObservationsFused", questObservationsFused);
  }

  /**
   * Feeds trusted QuestNav frames into the estimator and re-seeds the headset when it drifts.
   *
   * <p><b>Outputs:</b> may call {@code RobotState.addVisionObservation} and {@link
   * #seedQuestNav()}.
   */
  private void processQuestNav() {
    // IMPLEMENTATION (was TODO: convert each frame through kRobotToQuest.inverse(), fuse when
    // trusted, and seed or re-seed on the three conditions.)
    if (questInputs.questPoses.length == 0) {
      // No frames is not an error -- it is what a headset that is not tracking looks like, and the
      // residual timer is deliberately left alone so a drift that was already being timed is not
      // forgiven by a momentary gap in the stream.
      return;
    }

    // The residual is measured on the NEWEST frame only. Every frame in this loop's batch says
    // nearly the same thing about a 20 ms window, so averaging them would just be a slower way of
    // reading the last one.
    Pose3d newestHeadsetPose = questInputs.questPoses[questInputs.questPoses.length - 1];
    Pose2d newestRobotPose = questPoseToRobotPose(newestHeadsetPose);
    questResidualMeters =
        newestRobotPose
            .getTranslation()
            .getDistance(RobotState.getInstance().getEstimatedPose().getTranslation());

    // The hold timer. Started on the first loop over threshold, stopped and reset the moment the
    // residual recovers, so it measures CONTINUOUS disagreement rather than cumulative.
    if (questResidualMeters > Constants.Vision.kQuestReseedResidualMeters.get()) {
      if (!questResidualTimer.isRunning()) {
        questResidualTimer.restart();
      }
    } else {
      questResidualTimer.stop();
      questResidualTimer.reset();
    }

    // Fusing comes before seeding, so a frame is never fused on the same loop it was seeded from.
    // It would be circular if it were: the seed writes the current estimate into the headset, and
    // the headset would then report that estimate straight back as an independent measurement at
    // the tightest covariance in the system -- the estimator would be told its own belief and
    // become more confident in it for no reason.
    if (questTrusted) {
      for (int i = 0; i < questInputs.questPoses.length; i++) {
        RobotState.getInstance()
            .addVisionObservation(
                new RobotState.PoseObservation(
                    questPoseToRobotPose(questInputs.questPoses[i]),
                    questInputs.questPoseTimestamps[i],
                    VecBuilder.fill(
                        Constants.Vision.kQuestXyStdDev.get(),
                        Constants.Vision.kQuestXyStdDev.get(),
                        Constants.Vision.kQuestThetaStdDev.get()),
                    RobotState.PoseSource.QUESTNAV));
        acceptedCount++;
        questObservationsFused++;
      }
    }

    // ---- Seeding and re-seeding.
    if (!RobotState.getInstance().isQuestSeeded()) {
      // The first seed. Gated on a multi-tag fix exactly like a re-seed, because a seed taken from
      // a poor estimate is not a neutral starting point -- it writes that error into the headset's
      // frame and it stays there until the next seed. There is no hurry: an unseeded headset costs
      // nothing, since it is simply not fused.
      if (multipleTagsVisible()) {
        seedQuestNav();
      }
      return;
    }

    // A re-seed needs all three conditions, and each one is load-bearing:
    //
    //   sustained residual -- an instantaneous one is usually the cameras correcting the estimate;
    //   nearly stationary  -- a moving robot's estimate is its worst, and any timestamp mismatch
    //                         between the headset and the estimator turns into position error;
    //   two or more tags   -- a single-tag fix is the least trustworthy thing the cameras produce.
    //
    // Drop any one and the re-seed can write MORE error into the headset than the drift it was
    // correcting, permanently and invisibly, because the headset then reports that error as
    // confident truth. This is the one place in the vision stack where acting is worse than
    // waiting: drift is bounded and self-announcing, a bad seed is neither.
    if (questResidualTimer.isRunning()
        && questResidualTimer.get() > Constants.Vision.kQuestReseedHoldSec.get()
        && robotIsNearlyStationary()
        && multipleTagsVisible()) {
      seedQuestNav();
    }
  }

  /**
   * Converts a headset pose into the robot pose it implies.
   *
   * @param fieldToQuest the headset's reported field pose
   * @return the corresponding robot field pose
   */
  private Pose2d questPoseToRobotPose(Pose3d fieldToQuest) {
    // INVERSE, because the stored transform runs robot-to-quest and this is going the other way.
    // Composing it forwards instead is a silent error worth naming: it produces a pose displaced
    // by twice the mounting offset, in a direction that rotates with the robot's heading, which
    // reads as a pose that is fine going one way down the field and wrong coming back.
    return fieldToQuest.transformBy(robotToQuest.inverse()).toPose2d();
  }

  /**
   * Whether the robot is moving slowly enough for a re-seed to be safe.
   *
   * @return true when both translational and angular speed are near zero
   */
  private boolean robotIsNearlyStationary() {
    var speeds = RobotState.getInstance().getFieldRelativeSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            < Constants.Vision.kQuestReseedMaxSpeedMetersPerSec.get()
        && Math.abs(speeds.omegaRadiansPerSecond)
            < Constants.Vision.kQuestReseedMaxOmegaRadPerSec.get();
  }

  /**
   * Whether either camera currently has a multi-tag fix.
   *
   * @return true when at least one camera reports an observation with two or more tags
   */
  private boolean multipleTagsVisible() {
    // Either camera, because the question is "is the estimate trustworthy right now", and a
    // multi-tag fix from the intake camera answers it just as well as one from the turret. The
    // observations are read directly rather than through the rejection ladder because this is a
    // question about what is VISIBLE, not about what survived fusion.
    for (VisionIO.PoseObservation observation : turretInputs.poseObservations) {
      if (observation.tagCount() >= 2) {
        return true;
      }
    }
    for (VisionIO.PoseObservation observation : intakeInputs.poseObservations) {
      if (observation.tagCount() >= 2) {
        return true;
      }
    }
    return false;
  }

  /**
   * Seeds the headset's origin from the current fused pose estimate.
   *
   * <p><b>Outputs:</b> commands the headset to a new origin, marks it seeded in {@link RobotState},
   * and logs the seed event with the pose so re-seeds per match can be counted.
   */
  public void seedQuestNav() {
    // IMPLEMENTATION (was TODO: "getEstimatedPose, transformBy kRobotToQuest, setPose,
    // setQuestSeeded(true), and log the event with its pose.")
    //
    // The transform is composed HERE and not in the IO, because setPose commands the HEADSET's
    // pose, not the robot's. Handing it a robot pose seeds the headset to where the robot's origin
    // is, and every subsequent QuestNav measurement then arrives offset by the mounting -- a
    // constant bias at the tightest covariance in the system, which is the hardest kind to spot
    // because it never moves and never spikes.
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Pose3d headsetPose = new Pose3d(robotPose).transformBy(robotToQuest);

    questNavIO.setPose(headsetPose);
    RobotState.getInstance().setQuestSeeded(true);

    // The residual timer is cleared as part of the seed. Leaving it running would let the hold
    // that triggered this re-seed immediately trigger the next one, before any new frame has had a
    // chance to show whether the seed worked -- a re-seed every loop, each writing the estimate
    // into the headset, which would look like the headset tracking the estimate perfectly.
    questResidualTimer.stop();
    questResidualTimer.reset();
    questResidualMeters = 0.0;

    questSeedCount++;
    Logger.recordOutput("Vision/QuestSeedCount", questSeedCount);
    Logger.recordOutput("Vision/QuestSeedPose", robotPose);
    Logger.recordOutput("QuestNav/Event", "Seeded");
    // A rising seed count across matches is a physical diagnosis, not a software one: the headset
    // mount is loose, or kRobotToQuest is wrong. Counting them is the only way anyone notices.
  }

  /**
   * Whether the headset mounting has been measured yet.
   *
   * @return false while {@code kRobotToQuest} is still the identity transform
   */
  private boolean questMountingIsMeasured() {
    return !robotToQuest.equals(new Transform3d());
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
    // IMPLEMENTATION (was TODO: the planar-ground projection, transformed by the robot pose AT THE
    // OBSERVATION TIMESTAMP.)
    Optional<VisionIO.TargetObservation> target = getBestGamePieceTarget();
    if (target.isEmpty()) {
      return Optional.empty();
    }

    // The camera's own geometry. The pitch is the camera's, extracted from the mounting rather
    // than stored separately, so a re-measured mounting cannot leave a stale pitch behind.
    double cameraHeightMeters = Constants.Vision.kRobotToIntakeCamera.getZ();
    double cameraPitchRad = Constants.Vision.kRobotToIntakeCamera.getRotation().getY();

    // Until the mounting and the game piece are measured this is arithmetic on zeros, and the
    // guard below is what stops it being CONFIDENT arithmetic on zeros. With everything at 0.0 the
    // elevation angle is exactly 0 and the range is 0/0 -- NaN, which would propagate into a drive
    // command. Refusing to answer is the honest response to "we have not measured the robot yet".
    if (cameraHeightMeters <= 0.0 || Constants.Field.kGamePieceDiameterMeters.get() <= 0.0) {
      return Optional.empty();
    }

    // The planar-ground assumption: the piece sits on the carpet, so its centre is one radius up,
    // and the vertical angle to it fixes the range. A NEGATIVE elevation is the camera looking
    // down at the floor, which is the only case with a solution -- looking level or up at a piece
    // on the ground means the ray never reaches the carpet, and the tangent would hand back a
    // negative or enormous range rather than failing.
    double elevationRad = cameraPitchRad + target.get().ty().getRadians();
    if (elevationRad >= -1e-6) {
      return Optional.empty();
    }
    double heightDifferenceMeters =
        cameraHeightMeters - Constants.Field.kGamePieceDiameterMeters.get() / 2.0;
    double rangeMeters = heightDifferenceMeters / -Math.tan(elevationRad);
    if (!Double.isFinite(rangeMeters) || rangeMeters <= 0.0) {
      return Optional.empty();
    }

    // Bearing is NEGATED: a Limelight reports tx positive to the right of its crosshair, while
    // WPILib yaw is positive counter-clockwise, which is to the left.
    Rotation2d bearing = target.get().tx().unaryMinus();

    // AT THE OBSERVATION TIMESTAMP, not now. The detection is one camera latency old, and a robot
    // driving at 4 m/s covers 32 cm in 80 ms -- comfortably more than the piece is wide, so the
    // current pose would place it somewhere the intake will not find it. Falling back to the
    // current pose when the buffer cannot answer would hide exactly that error, so the observation
    // is dropped instead, the same rule the turret camera follows.
    Optional<Pose2d> robotPoseAtCapture =
        RobotState.getInstance().getPoseAt(target.get().timestampSec());
    if (robotPoseAtCapture.isEmpty()) {
      return Optional.empty();
    }

    // The piece's position in the CAMERA's horizontal frame, then out to the field through the
    // camera mounting and the robot pose.
    Translation2d pieceInCameraFrame =
        new Translation2d(rangeMeters * bearing.getCos(), rangeMeters * bearing.getSin());
    Translation2d cameraInRobotFrame =
        new Translation2d(
            Constants.Vision.kRobotToIntakeCamera.getX(),
            Constants.Vision.kRobotToIntakeCamera.getY());
    Rotation2d cameraYaw =
        Rotation2d.fromRadians(Constants.Vision.kRobotToIntakeCamera.getRotation().getZ());
    Translation2d pieceInRobotFrame =
        cameraInRobotFrame.plus(pieceInCameraFrame.rotateBy(cameraYaw));

    Pose2d robotPose = robotPoseAtCapture.get();
    Translation2d pieceInField =
        robotPose.getTranslation().plus(pieceInRobotFrame.rotateBy(robotPose.getRotation()));

    // The heading is the bearing FROM the robot TO the piece. A game piece has no orientation of
    // its own worth reporting, and this is the heading a pickup approach wants.
    Rotation2d approachHeading = pieceInField.minus(robotPose.getTranslation()).getAngle();
    Pose2d piecePose = new Pose2d(pieceInField, approachHeading);
    Logger.recordOutput("Vision/GamePieceFieldPose", piecePose);
    return Optional.of(piecePose);
    // TODO: MEASURE kGamePieceDiameterMeters and kRobotToIntakeCamera. Until both exist this
    // method returns empty by the guard above, which is deliberate -- an unmeasured robot should
    // decline to aim rather than aim at the origin.
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
    // IMPLEMENTATION (was TODO: "return questTrusted.")
    return questTrusted;
  }

  /**
   * Why QuestNav is not currently trusted.
   *
   * @return the specific failing clause, or {@code "Trusted"} when the headset is healthy
   */
  public String getQuestTrustReason() {
    // Exposed rather than left to the log alone so the degradation rules can be asserted by
    // reason, not merely by outcome. Six different faults all produce questTrusted == false, and a
    // test that checks only the boolean cannot tell whether the right one fired -- which is how a
    // gate ends up passing for a reason nobody intended.
    return questTrustReason;
  }

  /**
   * How many times the headset has been seeded since boot.
   *
   * @return the seed count, including the first seed
   */
  @AutoLogOutput(key = "Vision/QuestSeedCount")
  public int getQuestSeedCount() {
    return questSeedCount;
  }

  /**
   * How far the headset currently disagrees with the fused estimate.
   *
   * @return the residual in meters, or 0.0 when no headset frame has been processed
   */
  public double getQuestResidualMeters() {
    return questResidualMeters;
  }

  /**
   * How many headset frames have been handed to the estimator since boot.
   *
   * @return the fused QuestNav observation count
   */
  public int getQuestObservationsFused() {
    return questObservationsFused;
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
