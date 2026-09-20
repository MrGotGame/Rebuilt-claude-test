package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * M8's checkpoint, discharged as a test: <b>slew the turret through 180 degrees with the robot
 * stationary and a tag in view, and the estimated pose must not move.</b>
 *
 * <p>The plan calls this "the highest-value single test in the project", and names the failure it
 * detects: if the pose swings with the turret, the composition order in {@code
 * GeomUtil.rotateAboutZ} is backwards. Running it here rather than on a field means it runs on
 * every push, with the turret angle known exactly instead of read off a dashboard.
 *
 * <p>Three things had to be true before this test could mean anything, and each was a real gap:
 *
 * <ol>
 *   <li><b>The camera must be able to lie.</b> {@link VisionIOSim} reported ground truth directly
 *       through M7, so no transform error could ever exist and every ordering would have passed. It
 *       now solves the way a Limelight does -- fixing its lens in the field using the true
 *       mounting, then subtracting the mounting the robot code pushed -- so a wrong transform
 *       produces a wrong pose. {@link #theSimulatedCameraIsMisledByAStaleTransform()} is the
 *       standing proof that it still can, because the day that stops being true, every other case
 *       here passes for free.
 *   <li><b>The mounting must have a lever arm.</b> {@code kRobotToTurretAxis} and {@code
 *       kTurretToCamera} are both still the identity transform awaiting a CAD measurement, and a
 *       lens sitting exactly on the rotation axis does not move when the axis turns. Every case
 *       below therefore builds {@code Vision} with a plausible measured mounting.
 *   <li><b>The expected geometry must be computed independently.</b> {@link
 *       #trueRobotToCamera(double)} works the camera's position out with sine and cosine and never
 *       calls the composition under test. Deriving it from {@code GeomUtil.rotateAboutZ} would make
 *       a backwards composition cancel against itself and the suite would certify the bug.
 * </ol>
 */
public class TurretCameraTransformTest {
  private static final double LOOP_SEC = 0.02;

  /**
   * Where the robot really is, and stays.
   *
   * <p>Independent of the estimator on purpose. Pointing the simulated camera at {@code
   * RobotState::getEstimatedPose}, as {@code RobotContainer} does in SIM, makes vision agree with
   * whatever the estimator already believes -- so the estimate could wander anywhere and the camera
   * would follow it, reporting no error at any point. An absolute truth is what makes "the pose did
   * not move" a measurement instead of a tautology.
   */
  private static final Pose2d TRUTH = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(25.0));

  // A plausible measured mounting: the turret axis ahead of and above the robot origin, and the
  // lens out on the turret at a radius, tilted up to see tags. The numbers are illustrative -- the
  // real ones are a CAD measurement nobody has taken yet -- but the LEVER ARM is what matters: the
  // lens is 0.23 m from the rotation axis, so a half turn sweeps it through 0.46 m, and any
  // mis-composition shows up as tens of centimetres rather than as rounding.
  private static final double AXIS_X = 0.15;
  private static final double AXIS_Y = 0.05;
  private static final double AXIS_Z = 0.60;
  private static final double LENS_X = 0.22;
  private static final double LENS_Y = 0.06;
  private static final double LENS_Z = 0.10;
  private static final double CAMERA_PITCH_RAD = Math.toRadians(-18.0);

  private static final Transform3d ROBOT_TO_TURRET_AXIS =
      new Transform3d(new Translation3d(AXIS_X, AXIS_Y, AXIS_Z), new Rotation3d());
  private static final Transform3d TURRET_TO_CAMERA =
      new Transform3d(
          new Translation3d(LENS_X, LENS_Y, LENS_Z), new Rotation3d(0.0, CAMERA_PITCH_RAD, 0.0));

  /** The checkpoint's own tolerance: "does not move by more than a couple of centimetres". */
  private static final double CHECKPOINT_TOLERANCE_METERS = 0.02;

  private VisionIOSim turretSim;
  private Vision vision;

  /** Live turret angle in radians, driven by each test rather than by a turret subsystem. */
  private double turretAngleRad = 0.0;

  private double turretVelocityRadPerSec = 0.0;

  /**
   * Where the camera lens actually is, worked out from first principles.
   *
   * <p>Deliberately not expressed with {@code Transform3d} composition. The lens offset is rotated
   * in the plane by the turret angle and added to the axis; Z is untouched because the rotation is
   * about Z. The camera's orientation is the turret's yaw carrying the fixed camera pitch, which
   * {@code Rotation3d(roll, pitch, yaw)} expresses directly because that constructor builds Rz(yaw)
   * Ry(pitch) Rx(roll) -- the same order the physical chain applies them in.
   *
   * @param turretAngleRad robot-relative turret angle, in radians
   * @return the true robot-to-camera transform at that angle
   */
  private static Transform3d trueRobotToCamera(double turretAngleRad) {
    double cos = Math.cos(turretAngleRad);
    double sin = Math.sin(turretAngleRad);
    return new Transform3d(
        new Translation3d(
            AXIS_X + LENS_X * cos - LENS_Y * sin,
            AXIS_Y + LENS_X * sin + LENS_Y * cos,
            AXIS_Z + LENS_Z),
        new Rotation3d(0.0, CAMERA_PITCH_RAD, turretAngleRad));
  }

  /**
   * Brings up the HAL, parks the clock, and builds vision over a turret camera with a lever arm.
   */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();

    RobotState.initialize(new SwerveDriveKinematics(Drive.getModuleTranslations()));

    turretSim = new VisionIOSim(Constants.Vision.kTurretCameraName, () -> TRUTH);

    // Noise off. This suite measures transform error, and Gaussian noise on top of it would only
    // decide how tight the tolerance could be without making the test say anything more.
    turretSim.setNoise(0.0);

    // The camera's real mounting, which the robot code does not get to see. Everything in this
    // suite rests on this being computed independently of the code under test.
    turretSim.setTrueRobotToCamera(() -> trueRobotToCamera(turretAngleRad));

    vision =
        new Vision(
            turretSim,
            new VisionIO() {},
            new QuestNavIO() {},
            () -> turretAngleRad,
            () -> turretVelocityRadPerSec,
            ROBOT_TO_TURRET_AXIS,
            TURRET_TO_CAMERA);

    seedOdometryAndTurret();
  }

  /** Releases the HAL and unregisters the subsystem so cases cannot leak into each other. */
  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().unregisterSubsystem(vision);
    CommandScheduler.getInstance().cancelAll();
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  /** The module positions of a robot that is not moving, which is every case in this suite. */
  private static SwerveModulePosition[] stationaryModules() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = new SwerveModulePosition(0.0, Rotation2d.kZero);
    }
    return positions;
  }

  /**
   * Fills both history buffers so an observation has something to correlate and un-project against.
   *
   * <p><b>Outputs:</b> resets the estimator to {@link #TRUTH} and pushes a second of stationary
   * odometry and turret samples.
   */
  private void seedOdometryAndTurret() {
    RobotState.getInstance().resetPose(TRUTH, Rotation2d.kZero, stationaryModules());
    for (int i = 0; i < 50; i++) {
      RobotState.getInstance()
          .addOdometryObservation(Timer.getFPGATimestamp(), Rotation2d.kZero, stationaryModules());
      RobotState.getInstance().addTurretObservation(Timer.getFPGATimestamp(), turretAngleRad);
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  /**
   * Runs one loop with the turret slewing at a fixed rate.
   *
   * <p>The turret sample is written before {@code vision.periodic()} because that is the order the
   * scheduler produces on the robot: {@code Turret.periodic} files its angle as its first action,
   * so the buffer already covers this instant by the time vision reads a frame.
   *
   * @param rateRadPerSec how fast the turret is turning this loop
   *     <p><b>Outputs:</b> advances the turret angle, both buffers, vision, and the clock.
   */
  private void stepWithTurretRate(double rateRadPerSec) {
    turretVelocityRadPerSec = rateRadPerSec;

    RobotState.getInstance()
        .addOdometryObservation(Timer.getFPGATimestamp(), Rotation2d.kZero, stationaryModules());
    RobotState.getInstance().addTurretObservation(Timer.getFPGATimestamp(), turretAngleRad);

    vision.periodic();

    turretAngleRad += rateRadPerSec * LOOP_SEC;
    SimHooks.stepTiming(LOOP_SEC);
  }

  /** How far the fused estimate currently is from where the robot really is, in metres. */
  private static double estimateErrorMeters() {
    return RobotState.getInstance()
        .getEstimatedPose()
        .getTranslation()
        .getDistance(TRUTH.getTranslation());
  }

  /**
   * How many of the turret camera's observations were discarded for a reason.
   *
   * @param reason the rejection reason, by enum name
   * @return that counter's value
   */
  private int turretRejections(String reason) {
    return vision.getRejectionCount(RobotState.PoseSource.LIMELIGHT_TURRET, reason);
  }

  // ----------------------------------------------------------------- the checkpoint itself ----

  /**
   * <b>The M8 checkpoint.</b> The robot is stationary; the turret sweeps a half turn; the estimated
   * pose must stay put.
   *
   * <p>The slew runs at 1.0 rad/s, comfortably under {@code kMaxTurretRateForVisionRadPerSec}, so
   * the frames are accepted rather than discarded as motion-blurred. That matters: a slew fast
   * enough to trip rule 5 would leave the pose unmoved because nothing was fused at all, and the
   * checkpoint would pass on a robot whose transform maths was entirely wrong. The assertion that
   * observations were accepted is what closes that hole.
   */
  @Test
  void slewingTheTurretThroughHalfATurnDoesNotMoveTheEstimatedPose() {
    double rateRadPerSec = 1.0;
    int loops = (int) Math.ceil(Math.PI / rateRadPerSec / LOOP_SEC);

    turretSim.setSolveQuality(4, 2.0, 0.0);

    double worstErrorMeters = 0.0;
    for (int i = 0; i < loops; i++) {
      stepWithTurretRate(rateRadPerSec);
      worstErrorMeters = Math.max(worstErrorMeters, estimateErrorMeters());
    }

    assertTrue(
        turretAngleRad > Math.PI - 0.05,
        "the turret did not actually complete a half turn; it reached " + turretAngleRad + " rad");
    assertTrue(
        vision.getAcceptedCount() > 0,
        "no turret observation was fused, so the pose stayed put for the wrong reason -- this"
            + " checkpoint is only meaningful when the camera is actually feeding the estimator");
    assertEquals(
        0,
        turretRejections("TURRET_SLEWING"),
        "the slew was fast enough to trip the blur gate, which would have made this test vacuous");
    assertTrue(
        worstErrorMeters < CHECKPOINT_TOLERANCE_METERS,
        "the estimated pose swung with the turret by "
            + worstErrorMeters
            + " m over a half turn, against a tolerance of "
            + CHECKPOINT_TOLERANCE_METERS
            + " m. Per the M8 checkpoint the first suspect is the composition order in"
            + " GeomUtil.rotateAboutZ.");
  }

  /**
   * The simulated camera really can be fooled, so the checkpoint has something to catch.
   *
   * <p>A guard against the whole suite going quietly vacuous. If {@link VisionIOSim} ever goes back
   * to reporting ground truth regardless of the transform it was pushed -- which is exactly what it
   * did through M7 -- then every case above passes no matter how wrong the geometry is. This case
   * fails first and says why.
   *
   * <p>It drives the sim directly rather than through {@code Vision}, because the point is a
   * property of the simulated camera and nothing else.
   */
  @Test
  void theSimulatedCameraIsMisledByAStaleTransform() {
    VisionIOSim sim = new VisionIOSim("stale-transform-probe", () -> TRUTH);
    sim.setNoise(0.0);
    sim.setLatency(0.0);
    sim.setSolveQuality(4, 2.0, 0.0);

    // The lens is physically at a quarter turn; the robot code believes it is at zero.
    sim.setTrueRobotToCamera(() -> trueRobotToCamera(Math.PI / 2.0));
    sim.setRobotToCamera(trueRobotToCamera(0.0));

    var inputs = new VisionIO.VisionIOInputs();
    sim.updateInputs(inputs);

    assertEquals(1, inputs.poseObservations.length, "the probe emitted no observation");
    double errorMeters =
        inputs
            .poseObservations[0]
            .pose()
            .toPose2d()
            .getTranslation()
            .getDistance(TRUTH.getTranslation());
    assertTrue(
        errorMeters > 0.2,
        "a camera told the wrong mounting still reported the right pose, off by only "
            + errorMeters
            + " m. The simulation is not modelling the transform at all, and every turret"
            + " transform test in this file is therefore vacuous.");
  }

  // ------------------------------------------------------- the two mechanisms, individually ----

  /**
   * The capture-time transform describes the turret as it was, not as it is.
   *
   * <p>Section 7.2's second mechanism in isolation. After a slew, the transform reconstructed for a
   * moment 200 ms ago must match the angle the turret held then -- and must differ from the one for
   * right now, or the reconstruction is not reconstructing anything.
   */
  @Test
  void theCaptureTimeTransformDescribesTheTurretAsItWasNotAsItIs() {
    turretSim.setSolveQuality(4, 2.0, 0.0);

    double pastTimestampSec = Timer.getFPGATimestamp();
    double angleThenRad = turretAngleRad;

    for (int i = 0; i < 10; i++) {
      stepWithTurretRate(1.0);
    }

    Transform3d reconstructed = vision.robotToTurretCameraAt(pastTimestampSec);
    Transform3d truthThen = trueRobotToCamera(angleThenRad);

    assertEquals(
        0.0,
        reconstructed.getTranslation().getDistance(truthThen.getTranslation()),
        1e-6,
        "the reconstructed transform does not match where the camera actually was at that instant");

    // And it is genuinely a different answer from "now", or the lookup could be ignoring its
    // argument entirely and this test would pass on a method that returns the live transform.
    Transform3d nowTransform = vision.robotToTurretCameraNow();
    assertTrue(
        nowTransform.getTranslation().getDistance(reconstructed.getTranslation()) > 0.01,
        "the past and present transforms are identical after a 0.2 rad slew, so the timestamp"
            + " argument is not being used");
  }

  /**
   * The one-loop lead points where the turret is going, not where it has been.
   *
   * <p>The sign of the extrapolation, asserted on its own. Getting it backwards does not fail
   * loudly -- it doubles the error the term was added to remove -- and at a 12 rad/s cruise one
   * loop is 13.8 degrees of bearing, so the cost is real and the cause would be invisible.
   */
  @Test
  void theOneLoopLeadPointsWhereTheTurretIsGoing() {
    turretAngleRad = 0.5;

    turretVelocityRadPerSec = 0.0;
    double parkedYaw = vision.robotToTurretCameraNow().getRotation().getZ();

    turretVelocityRadPerSec = 3.0;
    double movingYaw = vision.robotToTurretCameraNow().getRotation().getZ();

    assertEquals(
        turretAngleRad,
        parkedYaw,
        1e-9,
        "a stationary turret's pushed transform should describe exactly where it is");
    assertEquals(
        turretAngleRad + 3.0 * Constants.getLoopPeriodSec(),
        movingYaw,
        1e-9,
        "the lead term is the wrong size or the wrong sign; it must predict one loop AHEAD");
  }

  // ----------------------------------------------------------------- the rejection rules ----

  /**
   * A frame the turret buffer cannot reach back to is thrown away, never guessed at.
   *
   * <p>Rule 7. The alternative the plan names by name -- substituting the current turret angle --
   * would produce a pose that is wrong by up to the full lever arm while looking entirely
   * plausible: on the field, at a sane height, inside the jump gate. Nothing downstream could catch
   * it.
   */
  @Test
  void aFrameOlderThanTheTurretBufferIsRejected() {
    turretSim.setSolveQuality(4, 2.0, 0.0);

    // Far beyond the two-second window RobotState keeps.
    turretSim.setLatency(30.0);
    for (int i = 0; i < 5; i++) {
      stepWithTurretRate(0.0);
    }

    assertTrue(turretRejections("NO_TURRET_ANGLE") > 0, "NO_TURRET_ANGLE never fired");
    assertEquals(
        0, vision.getAcceptedCount(), "a frame with no reconstructable transform was still fused");

    // The rejection must be attributed to the TURRET ANGLE, not to the push log. Both lookups miss
    // for a frame this old, and the angle is checked first, so seeing the push-log counter move
    // instead means the angle lookup answered for an instant it cannot know -- the current-angle
    // fallback the plan forbids by name. With the two reasons merged into one counter this test
    // passed with that fallback in place, which is how the split came to exist.
    assertEquals(
        0,
        turretRejections("NO_PUSHED_TRANSFORM"),
        "the rejection was blamed on the push log, so the turret angle lookup answered for a"
            + " 30-second-old frame instead of declining");
  }

  /**
   * The correction is exact, not merely close.
   *
   * <p>The checkpoint's two-centimetre tolerance is sized for the failure it was written to catch
   * -- a backwards composition, worth tens of centimetres -- and it is far too loose to notice the
   * capture-time correction being absent, which costs about a centimetre. Mutation testing proved
   * that: deleting the correction entirely, and looking up the newest pushed transform instead of
   * the one in effect at capture, both left the checkpoint green.
   *
   * <p>So this case asserts the property the mechanism actually promises. The correction is exact
   * arithmetic on one instant, not an approximation: whatever the push got wrong is removed by
   * re-deriving against the angle the turret genuinely had. With truth, odometry and vision all
   * agreeing, the residual should be floating-point noise. A millimetre of slack is allowed for the
   * estimator's blending, and nothing is allowed for transform error.
   *
   * <p>The slew runs just under the blur gate, because the error these defects produce is
   * proportional to turret rate and this is the fastest the camera is trusted at.
   */
  @Test
  void theCorrectionIsExactAndNotMerelyClose() {
    double rateRadPerSec = Constants.Vision.kMaxTurretRateForVisionRadPerSec.get() * 0.9;
    turretSim.setSolveQuality(4, 2.0, 0.0);

    // Let the estimator settle onto the vision stream before measuring, so this reports steady
    // state rather than the transient from the first few corrections.
    for (int i = 0; i < 25; i++) {
      stepWithTurretRate(rateRadPerSec);
    }

    double worstErrorMeters = 0.0;
    for (int i = 0; i < 75; i++) {
      stepWithTurretRate(rateRadPerSec);
      worstErrorMeters = Math.max(worstErrorMeters, estimateErrorMeters());
    }

    assertTrue(vision.getAcceptedCount() > 0, "nothing was fused, so nothing was measured");
    assertTrue(
        worstErrorMeters < 0.001,
        "the estimate sat "
            + worstErrorMeters
            + " m from truth during a steady slew. The composition is evidently close, so the"
            + " suspect is not rotateAboutZ but the capture-time correction: either it is not"
            + " running, or it is un-composing with the wrong pushed transform.");
  }

  /**
   * The blur gate refuses a fast slew and permits a slow one.
   *
   * <p>Both halves, because a gate that only ever refuses is indistinguishable from a camera that
   * is switched off -- and that failure presents as "the turret camera contributes nothing", which
   * is what a working blur gate looks like from a distance.
   */
  @Test
  void aFastSlewIsRejectedButASlowOneIsNot() {
    turretSim.setSolveQuality(4, 2.0, 0.0);

    double tooFast = Constants.Vision.kMaxTurretRateForVisionRadPerSec.get() * 2.0;

    // The first frames to arrive after a slew begins were EXPOSED before it began -- they are 40 ms
    // old -- and the gate correctly accepts them, because it judges the moment of capture rather
    // than the moment of arrival. That is the behaviour the next case asserts deliberately; here it
    // is just a settling window to flush before the measurement starts.
    for (int i = 0; i < 4; i++) {
      stepWithTurretRate(tooFast);
    }
    int acceptedBeforeTheSlewWasVisible = vision.getAcceptedCount();

    for (int i = 0; i < 10; i++) {
      stepWithTurretRate(tooFast);
    }
    assertTrue(turretRejections("TURRET_SLEWING") > 0, "the blur gate never fired on a fast slew");
    assertEquals(
        acceptedBeforeTheSlewWasVisible,
        vision.getAcceptedCount(),
        "a frame captured mid-slew was fused");

    double slowEnough = Constants.Vision.kMaxTurretRateForVisionRadPerSec.get() * 0.5;
    for (int i = 0; i < 10; i++) {
      stepWithTurretRate(slowEnough);
    }
    assertTrue(
        vision.getAcceptedCount() > acceptedBeforeTheSlewWasVisible,
        "a slew at half the blur threshold was also refused, so the gate is not a threshold");
  }

  /**
   * The blur gate judges the frame's own moment, not the present one.
   *
   * <p>A turret that has just stopped reports zero velocity while the frames it blurred are still
   * in flight -- 60 to 120 ms of them. A gate reading the live velocity waves every one of those
   * through, and does so exactly when the shooter is settling onto a target and the estimate
   * matters most. This is why the rate is differenced out of the angle buffer at the capture
   * timestamp rather than read from the turret's velocity supplier.
   */
  @Test
  void theBlurGateJudgesTheFramesOwnMomentAndNotThePresent() {
    turretSim.setSolveQuality(4, 2.0, 0.0);

    // A hard slew, then a dead stop. The frames still arriving were captured during the slew.
    double tooFast = Constants.Vision.kMaxTurretRateForVisionRadPerSec.get() * 2.0;
    for (int i = 0; i < 10; i++) {
      stepWithTurretRate(tooFast);
    }

    int rejectedDuringSlew = turretRejections("TURRET_SLEWING");
    int acceptedDuringSlew = vision.getAcceptedCount();

    // Stopped dead: the live velocity supplier now reads exactly zero.
    stepWithTurretRate(0.0);
    stepWithTurretRate(0.0);

    assertEquals(0.0, turretVelocityRadPerSec, "the turret was not actually stopped for this case");
    assertTrue(
        turretRejections("TURRET_SLEWING") > rejectedDuringSlew,
        "frames captured during the slew were accepted once the turret stopped moving, so the blur"
            + " gate is reading the live velocity instead of the rate at capture time");
    assertEquals(
        acceptedDuringSlew,
        vision.getAcceptedCount(),
        "a frame captured mid-slew was fused because the turret had stopped by the time it"
            + " arrived");
  }

  /**
   * The fixed camera is untouched by any of this.
   *
   * <p>A guard against the turret rules leaking onto the intake camera, which has a constant
   * mounting and must keep working while the turret does whatever it likes. A regression here would
   * present as the intake camera falling silent whenever the shooter tracks -- precisely when the
   * turret camera is least able to cover for it.
   */
  @Test
  void theIntakeCameraIsUnaffectedByTheTurret() {
    VisionIOSim intakeSim = new VisionIOSim(Constants.Vision.kIntakeCameraName, () -> TRUTH);
    intakeSim.setNoise(0.0);
    intakeSim.setSolveQuality(4, 2.0, 0.0);

    CommandScheduler.getInstance().unregisterSubsystem(vision);
    vision =
        new Vision(
            new VisionIO() {},
            intakeSim,
            new QuestNavIO() {},
            () -> turretAngleRad,
            () -> turretVelocityRadPerSec,
            ROBOT_TO_TURRET_AXIS,
            TURRET_TO_CAMERA);

    // A slew far past the blur threshold, which must have no bearing on a frame-fixed camera.
    for (int i = 0; i < 20; i++) {
      stepWithTurretRate(Constants.Vision.kMaxTurretRateForVisionRadPerSec.get() * 4.0);
    }

    assertTrue(vision.getAcceptedCount() > 0, "the intake camera stopped contributing");
    assertEquals(
        0,
        vision.getRejectionCount("TURRET_SLEWING"),
        "the intake camera was penalised for the turret slewing");
    assertEquals(
        0,
        vision.getRejectionCount("NO_TURRET_ANGLE"),
        "the intake camera was asked for a turret transform it does not need");
    assertTrue(
        estimateErrorMeters() < CHECKPOINT_TOLERANCE_METERS,
        "the fixed camera moved the estimate, which it cannot do if its transform is constant");
  }

  /**
   * The transform lookup refuses to answer for an instant the buffer does not cover.
   *
   * <p>{@code TimeInterpolatableBuffer.getSample} clamps an out-of-range query to its nearest
   * endpoint rather than returning empty, so the obvious one-line implementation of {@code
   * getTurretAngleAt} answers every question ever asked -- with an angle up to two seconds stale.
   * Rule 7 fires on this method returning empty, so without the window guard the rule is
   * unreachable code and the rejection above could never happen. Asserted here because the bug is
   * invisible from the call site: a wrong angle and a right one have the same type.
   */
  @Test
  void theTurretBufferRefusesQueriesItCannotAnswer() {
    double beforeAnySample = Timer.getFPGATimestamp() - 60.0;

    assertTrue(
        RobotState.getInstance().getTurretAngleAt(beforeAnySample).isEmpty(),
        "the turret buffer answered for an instant a minute before it held any sample, which means"
            + " it is clamping to its oldest value instead of declining");
    assertTrue(
        RobotState.getInstance().getTurretAngleAt(Timer.getFPGATimestamp()).isPresent(),
        "the turret buffer declined an instant it does cover, which would reject every frame");
  }

  /**
   * A corrected turret pose is what reaches the estimator, not the reported one.
   *
   * <p>End to end, at one instant rather than over a sweep: with the turret parked well off zero,
   * the pose the camera reports and the pose the estimator receives must differ, and it is the
   * corrected one that matches truth. If the correction were removed, the estimate would settle on
   * the reported pose instead and this case would fail with roughly the lever arm as its error.
   */
  @Test
  void theCorrectedPoseIsWhatReachesTheEstimator() {
    turretSim.setSolveQuality(4, 2.0, 0.0);

    // Park the turret a long way round, and let the buffers cover the new angle.
    turretAngleRad = Math.toRadians(150.0);
    for (int i = 0; i < 20; i++) {
      stepWithTurretRate(0.0);
    }

    assertTrue(vision.getAcceptedCount() > 0, "nothing was fused, so there is nothing to check");
    assertTrue(
        estimateErrorMeters() < CHECKPOINT_TOLERANCE_METERS,
        "with the turret parked at 150 degrees the estimate is "
            + estimateErrorMeters()
            + " m from truth; the camera's offset is being credited to the robot");

    // And the geometry really was non-trivial at that angle: the lens is nowhere near the robot
    // origin, so "close to truth" was not available for free.
    Transform3d mounting = trueRobotToCamera(turretAngleRad);
    assertTrue(
        new Pose3d().transformBy(mounting).getTranslation().getNorm() > 0.2,
        "the camera sits too close to the robot origin for this case to distinguish anything");
  }
}
