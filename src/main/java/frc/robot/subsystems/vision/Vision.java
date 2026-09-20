package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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
        Constants.Vision.kTurretToCamera);
  }

  /**
   * Builds the vision subsystem with an explicit turret camera mounting.
   *
   * <p>The robot uses the five-argument constructor, which supplies the mounting from {@link
   * Constants}. This one exists because those constants are both still the identity transform
   * awaiting a CAD measurement, and a turret-transform test run against identity proves nothing:
   * with the lens on the rotation axis, every composition order -- correct and backwards alike --
   * leaves the camera in the same place when the turret turns. A test needs a real lever arm to
   * have anything to measure.
   *
   * @param turretCameraIO the turret-mounted Limelight
   * @param intakeCameraIO the intake-mounted Limelight
   * @param questNavIO the Quest 3S headset
   * @param turretAngleSupplier live robot-relative turret angle, in radians
   * @param turretVelocitySupplier live turret angular velocity, in radians per second
   * @param robotToTurretAxis robot origin to the turret's rotation axis
   * @param turretToCamera the turret frame at 0 rad to the camera lens
   */
  public Vision(
      VisionIO turretCameraIO,
      VisionIO intakeCameraIO,
      QuestNavIO questNavIO,
      DoubleSupplier turretAngleSupplier,
      DoubleSupplier turretVelocitySupplier,
      Transform3d robotToTurretAxis,
      Transform3d turretToCamera) {
    this.turretCameraIO = turretCameraIO;
    this.intakeCameraIO = intakeCameraIO;
    this.questNavIO = questNavIO;
    this.turretAngleSupplier = turretAngleSupplier;
    this.turretVelocitySupplier = turretVelocitySupplier;
    this.robotToTurretAxis = robotToTurretAxis;
    this.turretToCamera = turretToCamera;
    // IMPLEMENTATION (was TODO: "Register FaultReporter checks ... QuestNav ones as WARNINGS.")
    //
    // The camera checks are Alerts driven from periodic(), for the reason recorded in
    // PhoenixUtil.tryUntilOk and Shooter: FaultReporter is a skeleton whose getInstance() returns
    // null and would throw here.
    //
    // The turret camera's alert is registered as of M8, now that its observations are fused. It
    // was deliberately absent through M7, when nothing read the camera: an alert for a device the
    // code never consults sits permanently red on a healthy robot, which teaches everyone to
    // ignore the alerts widget -- the one place M14 needs them to look. The QuestNav checks stay
    // absent for exactly that reason until M9 reads the headset.
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
    // M8 completes both Limelights. Step 7 (QuestNav) remains absent because it is M9; the
    // headset's IO is not even sampled, so nothing reads a value nobody has verified.
    //
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

    // Turret camera first, matching the plan's step order. Order between the two cameras does not
    // change the fused result -- the estimator sorts by timestamp internally -- but keeping it
    // fixed keeps the log deterministic when the same match is replayed.
    processCamera(turretInputs, RobotState.PoseSource.LIMELIGHT_TURRET, true);
    processCamera(intakeInputs, RobotState.PoseSource.LIMELIGHT_INTAKE, false);

    Logger.recordOutput("Vision/AcceptedObservations", acceptedCount);
    logRejections(RobotState.PoseSource.LIMELIGHT_TURRET, Constants.Vision.kTurretCameraName);
    logRejections(RobotState.PoseSource.LIMELIGHT_INTAKE, Constants.Vision.kIntakeCameraName);

    intakeCameraDisconnectedAlert.set(!intakeInputs.connected);
    turretCameraDisconnectedAlert.set(!turretInputs.connected);
    fieldBoundsUnmeasuredAlert.set(!fieldBoundsAreMeasured());
    turretTransformUnmeasuredAlert.set(!turretMountingIsMeasured());
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
