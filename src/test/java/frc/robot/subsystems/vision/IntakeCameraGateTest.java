package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IdealPivotIO;
import frc.robot.subsystems.intake.Intake;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The intake camera is only fused from the one pivot position its transform describes.
 *
 * <p>The plan assumed this camera was bolted to the frame -- section 7.2 says "the intake camera
 * needs none of this: its transform is the constant {@code kRobotToIntakeCamera}, pushed once at
 * construction". The CAD says otherwise: it rides the pivoting intake about 0.4 m from the axis,
 * nearly three times the turret camera's 0.139 m lever arm, so a 60 degree deploy sweeps the lens
 * through roughly 0.4 m.
 *
 * <p>Rather than give it M8's full moving-transform machinery, the fusion is gated on the intake
 * being settled at its intaking position -- the pose {@code kRobotToIntakeCamera} was measured in.
 * These cases pin down the two halves of that gate and, more importantly, the timing: <b>the gate
 * judges the moment of capture, not the moment of arrival.</b> An intake that has just reached its
 * setpoint still has frames in flight that were exposed somewhere along the arc, and those arrive
 * looking entirely ordinary.
 */
public class IntakeCameraGateTest {
  private static final double LOOP_SEC = 0.02;
  private static final Pose2d TRUTH = new Pose2d(2.0, 3.0, Rotation2d.kZero);
  private static final Transform3d NONE = new Transform3d();

  private VisionIOSim intakeSim;
  private Vision vision;

  /** Whether the intake is settled at its camera pose, driven by each case. */
  private boolean intakeAtCameraPose = true;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();

    RobotState.initialize(new SwerveDriveKinematics(Drive.getModuleTranslations()));

    intakeAtCameraPose = true;
    intakeSim = new VisionIOSim(Constants.Vision.kIntakeCameraName, () -> TRUTH);
    intakeSim.setNoise(0.0);
    intakeSim.setSolveQuality(3, 2.0, 0.0);

    vision =
        new Vision(
            new VisionIO() {},
            intakeSim,
            new QuestNavIO() {},
            () -> 0.0,
            () -> 0.0,
            () -> intakeAtCameraPose,
            NONE,
            NONE,
            NONE);

    seedOdometry();

    // Warm the pose history, blacked out so it contributes no observations of its own.
    intakeSim.setBlackout(true);
    step(4);
    intakeSim.setBlackout(false);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().unregisterSubsystem(vision);
    CommandScheduler.getInstance().cancelAll();
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  private static SwerveModulePosition[] stationary() {
    SwerveModulePosition[] p = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      p[i] = new SwerveModulePosition(0.0, Rotation2d.kZero);
    }
    return p;
  }

  private void seedOdometry() {
    RobotState.getInstance().resetPose(TRUTH, Rotation2d.kZero, stationary());
    for (int i = 0; i < 50; i++) {
      RobotState.getInstance()
          .addOdometryObservation(Timer.getFPGATimestamp(), Rotation2d.kZero, stationary());
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  private void step(int loops) {
    for (int i = 0; i < loops; i++) {
      RobotState.getInstance()
          .addOdometryObservation(Timer.getFPGATimestamp(), Rotation2d.kZero, stationary());
      RobotState.getInstance()
          .addDriveSpeedsObservation(Timer.getFPGATimestamp(), new ChassisSpeeds());
      // The turret angle buffer must stay fed even though this suite is about the intake: without
      // it M8's NO_TURRET_ANGLE rule refuses every turret-camera frame, and the case asserting the
      // turret camera is unaffected would fail for a reason that has nothing to do with the gate.
      RobotState.getInstance().addTurretObservation(Timer.getFPGATimestamp(), 0.0);
      vision.periodic();
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  private int rejections() {
    return vision.getRejectionCount(RobotState.PoseSource.LIMELIGHT_INTAKE, "INTAKE_NOT_DEPLOYED");
  }

  /** With the intake at its camera pose, observations are fused normally. */
  @Test
  void atTheCameraPoseTheIntakeCameraIsFused() {
    intakeAtCameraPose = true;
    step(6);

    assertTrue(
        vision.getAcceptedCount() > 0, "the intake camera contributed nothing while in pose");
    assertEquals(0, rejections(), "the gate rejected frames captured while the intake was in pose");
  }

  /**
   * Away from that pose, nothing is fused.
   *
   * <p>Asserted as a count of accepted observations rather than as a rejection count alone, because
   * a gate that rejects <i>and</i> fuses is indistinguishable from a working one by its counter.
   */
  @Test
  void awayFromTheCameraPoseNothingIsFused() {
    intakeAtCameraPose = false;

    // Flush first. Frames exposed during the warm-up, when the intake WAS in pose, are still in
    // flight and are correctly accepted -- that is the very latency this gate exists to respect,
    // so counting them here would be measuring the wrong thing.
    step(4);
    int acceptedBefore = vision.getAcceptedCount();
    int rejectedBefore = rejections();

    step(6);

    assertTrue(
        rejections() > rejectedBefore, "the gate never fired with the intake out of position");
    assertEquals(
        acceptedBefore,
        vision.getAcceptedCount(),
        "a frame was fused with the intake out of position, so it was transformed by a mounting"
            + " that did not apply -- worth up to 0.4 m of pose error");
  }

  /**
   * <b>The gate judges the frame's own moment, not the present one.</b>
   *
   * <p>The case this rule exists for, and the one a live check would fail. The intake arrives at
   * its setpoint and {@code isAtIntakingAngle()} goes true, but the frames still in flight were
   * exposed during the swing, somewhere along a 0.4 m arc, and would be fused against the settled
   * transform. Reading the supplier directly instead of the history would accept every one of them,
   * and they would look entirely ordinary doing it.
   */
  @Test
  void theGateJudgesTheFramesOwnMomentAndNotThePresent() {
    // In transit: frames exposed now must never be fused. Flush the warm-up's in-flight frames
    // first, for the reason the previous case records.
    intakeAtCameraPose = false;
    step(4);
    int acceptedDuringTransit = vision.getAcceptedCount();
    step(6);
    assertEquals(
        acceptedDuringTransit, vision.getAcceptedCount(), "a frame exposed in transit was fused");

    // The intake arrives. The camera latency means the next frames to surface were still exposed
    // during the swing.
    intakeAtCameraPose = true;
    int rejectedOnArrival = rejections();
    step(1);

    assertTrue(
        rejections() > rejectedOnArrival,
        "the first frame after arrival was accepted, so the gate is reading the live intake state"
            + " rather than the state at capture time");
    assertEquals(
        acceptedDuringTransit,
        vision.getAcceptedCount(),
        "a frame exposed mid-swing was fused once the intake landed");

    // And once the latency window has passed, the camera does start contributing -- otherwise this
    // would be a gate that never opens, which looks identical from the counter alone.
    step(6);
    assertTrue(
        vision.getAcceptedCount() > acceptedDuringTransit,
        "the camera never resumed contributing after the intake settled, so the gate is stuck shut");
  }

  /**
   * A frame too old to place at all is blamed on the timestamp, not on the intake.
   *
   * <p>Both rules fire for a frame older than the buffers. Attribution matters more than it looks:
   * a counter that names the intake sends someone to look at the intake, when the fault is a stale
   * timestamp and the intake was never involved. The same reasoning M9 applied when it split {@code
   * NO_TURRET_ANGLE} from {@code NO_PUSHED_TRANSFORM}.
   */
  @Test
  void anUnplaceableFrameIsBlamedOnItsTimestampNotOnTheIntake() {
    intakeAtCameraPose = true;
    intakeSim.setLatency(30.0);
    step(5);

    assertTrue(
        vision.getRejectionCount(RobotState.PoseSource.LIMELIGHT_INTAKE, "NO_CORRELATED_POSE") > 0,
        "an unplaceable frame was not attributed to its timestamp");
    assertEquals(
        0,
        rejections(),
        "an unplaceable frame was blamed on the intake's position, which is not what was wrong");
  }

  /**
   * Before the history covers a frame's capture instant, the frame is refused rather than guessed
   * at.
   *
   * <p>The boot transient, and a deliberate test of it. {@code Vision} records the intake's state
   * once per periodic, so for roughly the first camera-latency after construction it cannot say
   * where the intake was when a frame was exposed. The honest answer is to refuse -- the same
   * refuse-rather-than-fall-back rule the turret angle lookup follows, and for the same reason: a
   * fallback here is indistinguishable at the call site from a real answer.
   *
   * <p>Note that odometry is already seeded at this point, so the correlation check passes and the
   * frame reaches the intake gate. Without this case, treating absent history as "in position" goes
   * unnoticed, because every other case warms the history first.
   */
  @Test
  void aFrameOlderThanTheIntakeHistoryIsRefusedNotAssumed() {
    CommandScheduler.getInstance().unregisterSubsystem(vision);
    VisionIOSim freshSim = new VisionIOSim(Constants.Vision.kIntakeCameraName, () -> TRUTH);
    freshSim.setNoise(0.0);
    freshSim.setSolveQuality(3, 2.0, 0.0);

    // Built with the intake reported as fully in position, so the ONLY thing that can refuse this
    // frame is the history not reaching back to when it was captured.
    vision =
        new Vision(
            new VisionIO() {},
            freshSim,
            new QuestNavIO() {},
            () -> 0.0,
            () -> 0.0,
            () -> true,
            NONE,
            NONE,
            NONE);

    step(1);

    assertEquals(
        0,
        vision.getAcceptedCount(),
        "a frame captured before the intake history began was fused anyway, which means absent"
            + " history is being read as 'in position' rather than as 'cannot know'");
    assertTrue(rejections() > 0, "the frame was dropped, but not attributed to the intake gate");
  }

  /**
   * A real intake RESTING deployed feeds the camera; the same intake rocking to feed the hopper
   * does not; and it resumes once the intake rests again.
   *
   * <p>M10b step 4. Every other case here drives the gate with a bare boolean, which tests Vision's
   * half of the gate but not which intake states open it. This wires a real {@link Intake} in
   * through {@link Intake#isAtIntakingAngle()}, exactly as {@code RobotContainer} does, over the
   * ideal pivot and the realistic test geometry. The resting phase is the one that matters most:
   * down-stays-down means the intake spends most of a match in DEPLOYED, and until this step the
   * gate read {@code isIntaking()}, so all of that time would have been silently lost to the
   * camera.
   */
  @Test
  void aRealIntakeRestingDeployedIsFusedButNotWhileRocking() {
    IdealPivotIO pivot = new IdealPivotIO();
    Intake intake = new Intake(pivot, IdealPivotIO.GEOMETRY);
    intake.setDeployGuard(() -> true);

    CommandScheduler.getInstance().unregisterSubsystem(vision);
    VisionIOSim camera = new VisionIOSim(Constants.Vision.kIntakeCameraName, () -> TRUTH);
    camera.setNoise(0.0);
    camera.setSolveQuality(3, 2.0, 0.0);
    vision =
        new Vision(
            new VisionIO() {},
            camera,
            new QuestNavIO() {},
            () -> 0.0,
            () -> 0.0,
            intake::isAtIntakingAngle,
            NONE,
            NONE,
            NONE);

    try {
      // Home (the robot starts stowed), then come to rest deployed, then let the frames exposed on
      // the way down drain out of the latency window before counting anything.
      stepWith(intake, 40);
      intake.setWantedState(Intake.WantedState.DEPLOY_ONLY);
      stepWith(intake, 10);
      assertEquals(Intake.SystemState.DEPLOYED, intake.getSystemState());

      int acceptedAtRest = vision.getAcceptedCount();
      stepWith(intake, 10);
      assertTrue(
          vision.getAcceptedCount() > acceptedAtRest,
          "the camera contributed nothing while the intake rested deployed -- the gate is still"
              + " reading isIntaking(), and this is most of a match");

      // Rocking: flush the resting frames still in flight, then nothing may be fused.
      intake.setWantedState(Intake.WantedState.FEED_ASSIST);
      stepWith(intake, 10);
      int acceptedWhileRocking = vision.getAcceptedCount();
      int rejectedWhileRocking = rejections();
      stepWith(intake, 20);
      assertEquals(
          acceptedWhileRocking,
          vision.getAcceptedCount(),
          "a frame was fused while the intake rocked, transformed by a mounting that described"
              + " where the camera was only half of the time");
      assertTrue(rejections() > rejectedWhileRocking, "the gate never fired during the rock");

      // At rest again, the camera comes back -- a gate that never reopens looks identical to a
      // working one by its rejection counter.
      intake.setWantedState(Intake.WantedState.DEPLOY_ONLY);
      stepWith(intake, 10);
      int acceptedBackAtRest = vision.getAcceptedCount();
      stepWith(intake, 10);
      assertTrue(
          vision.getAcceptedCount() > acceptedBackAtRest,
          "the camera never resumed once the intake came back to rest");
    } finally {
      CommandScheduler.getInstance().unregisterSubsystem(intake);
    }
  }

  /**
   * Steps a real intake and the vision subsystem together, intake first, as they are registered on
   * the robot.
   *
   * @param intake the intake to step alongside vision
   * @param loops how many 20 ms iterations to run
   *     <p><b>Outputs:</b> runs both periodics and the odometry feed that many times.
   */
  private void stepWith(Intake intake, int loops) {
    for (int i = 0; i < loops; i++) {
      intake.periodic();
      step(1);
    }
  }

  /**
   * The turret camera is unaffected by the intake.
   *
   * <p>A guard against the gate leaking onto the other camera. A regression here would silence the
   * turret camera whenever the intake was stowed -- which is most of a match.
   */
  @Test
  void theTurretCameraIsUnaffectedByTheIntakePosition() {
    VisionIOSim turretSim = new VisionIOSim(Constants.Vision.kTurretCameraName, () -> TRUTH);
    turretSim.setNoise(0.0);
    turretSim.setSolveQuality(3, 2.0, 0.0);

    CommandScheduler.getInstance().unregisterSubsystem(vision);
    vision =
        new Vision(
            turretSim,
            new VisionIO() {},
            new QuestNavIO() {},
            () -> 0.0,
            () -> 0.0,
            () -> false,
            NONE,
            NONE,
            NONE);

    step(8);

    assertTrue(
        vision.getAcceptedCount() > 0,
        "the turret camera stopped contributing because the intake was stowed");
    assertEquals(
        0,
        vision.getRejectionCount(RobotState.PoseSource.LIMELIGHT_TURRET, "INTAKE_NOT_DEPLOYED"),
        "the turret camera was gated on the intake's position, which has nothing to do with it");
  }
}
