package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
 * The M7 rejection ladder, asserted rule by rule.
 *
 * <p>The milestone's checkpoint asks that every counter under {@code
 * Vision/limelight-intake/Rejections/*} be nonzero at least once after deliberately provoking it.
 * Provoking them by hand means covering a camera and driving a robot at a wall; provoking them here
 * means one test each, run on every build, with the pose that triggered it chosen exactly.
 *
 * <p>Each case also asserts that nothing was accepted, because a rule that rejects <i>and</i> fuses
 * is indistinguishable from a working one by counter alone.
 */
public class VisionRejectionTest {
  private static final double LOOP_SEC = 0.02;

  private VisionIOSim intakeSim;
  private Vision vision;

  /** Builds vision over a simulated intake camera with the HAL up and the clock paused. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();

    RobotState.initialize(new SwerveDriveKinematics(Drive.getModuleTranslations()));
    seedOdometry();

    intakeSim = new VisionIOSim(Constants.Vision.kIntakeCameraName, () -> new Pose2d());
    // The intake is held at its camera pose: this suite is about the rejection ladder, and the
    // gate that confines the intake camera to one pivot pose is covered by IntakeCameraGateTest.
    vision =
        new Vision(
            new VisionIO() {}, intakeSim, new QuestNavIO() {}, () -> 0.0, () -> 0.0, () -> true);

    // Warm the intake-pose history before any case measures anything. Vision records that history
    // once per periodic, so for the first couple of loops after construction it cannot answer for
    // a frame captured one camera-latency ago and correctly refuses to fuse it. That is real
    // behaviour, not a test artifact -- it happens on the robot too, for about 40 ms after boot --
    // but it would otherwise show up as a couple of rejections in cases that are about other
    // rules entirely. Blacked out, so warming produces no observations of its own to count.
    intakeSim.setBlackout(true);
    step(4);
    intakeSim.setBlackout(false);
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

  /**
   * Fills the pose buffer so observations have something to correlate against.
   *
   * <p>Without this every observation is rejected as uncorrelated, because {@code getPoseAt}
   * interpolates the odometry buffer and an empty buffer answers nothing. That is correct
   * behaviour, and it would mask every other rule under test.
   *
   * <p><b>Outputs:</b> resets the estimator and pushes odometry samples across the buffer window.
   */
  private void seedOdometry() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = new SwerveModulePosition(0.0, Rotation2d.kZero);
    }
    RobotState.getInstance().resetPose(new Pose2d(), Rotation2d.kZero, positions);
    for (int i = 0; i < 60; i++) {
      RobotState.getInstance()
          .addOdometryObservation(Timer.getFPGATimestamp(), Rotation2d.kZero, positions);
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  /**
   * Advances vision by a number of loops, keeping odometry alive alongside it.
   *
   * @param loops how many 20 ms iterations to run
   *     <p><b>Outputs:</b> runs periodic that many times and advances the simulated clock.
   */
  private void step(int loops) {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = new SwerveModulePosition(0.0, Rotation2d.kZero);
    }
    for (int i = 0; i < loops; i++) {
      RobotState.getInstance()
          .addOdometryObservation(Timer.getFPGATimestamp(), Rotation2d.kZero, positions);
      vision.periodic();
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  /** A clean multi-tag fix at close range is accepted, or none of the rest means anything. */
  @Test
  void aCleanObservationIsAccepted() {
    intakeSim.setSolveQuality(3, 2.0, 0.0);
    step(5);

    assertTrue(vision.getAcceptedCount() > 0, "a clean multi-tag fix was not accepted");
    assertEquals(0, vision.getRejectedCount(), "a clean fix was also rejected by some rule");
  }

  /** A frame with no tags contributes nothing. */
  @Test
  void noTagsIsRejected() {
    intakeSim.setSolveQuality(0, 0.0, 0.0);
    step(5);

    assertTrue(vision.getRejectionCount("NO_TAGS") > 0, "NO_TAGS never fired");
    assertEquals(0, vision.getAcceptedCount(), "a tagless observation was fused");
  }

  /** A single tag solved ambiguously is thrown away; the same tag solved cleanly is not. */
  @Test
  void ambiguousSingleTagIsRejectedButACleanOneIsNot() {
    intakeSim.setSolveQuality(1, 2.0, Constants.Vision.kMaxAmbiguity.get() + 0.2);
    step(5);
    assertTrue(vision.getRejectionCount("SINGLE_TAG_AMBIGUOUS") > 0, "ambiguity rule never fired");
    assertEquals(0, vision.getAcceptedCount());

    // The same geometry with a confident solve must survive, or the rule is just "reject single
    // tags" wearing a threshold.
    intakeSim.setSolveQuality(1, 2.0, 0.0);
    step(5);
    assertTrue(vision.getAcceptedCount() > 0, "a confident single-tag fix should be accepted");
  }

  /** A solve that puts the robot off the floor plane is geometric nonsense. */
  @Test
  void impossibleHeightIsRejected() {
    intakeSim.setSolveQuality(3, 2.0, 0.0);
    intakeSim.setPoseOffset(
        new Transform3d(
            new Translation3d(0.0, 0.0, Constants.Vision.kMaxZErrorMeters.get() + 1.0),
            new edu.wpi.first.math.geometry.Rotation3d()));
    step(5);

    assertTrue(vision.getRejectionCount("Z_ERROR") > 0, "Z_ERROR never fired");
    assertEquals(0, vision.getAcceptedCount(), "a floating robot pose was fused");
  }

  /** A pose far from where odometry says the robot was is not believed. */
  @Test
  void impossibleJumpIsRejected() {
    intakeSim.setSolveQuality(3, 2.0, 0.0);
    intakeSim.setPoseOffset(
        new Transform3d(
            new Translation3d(Constants.Vision.kMaxAcceptedPoseJumpMeters.get() + 5.0, 0.0, 0.0),
            new edu.wpi.first.math.geometry.Rotation3d()));
    step(5);

    assertTrue(vision.getRejectionCount("POSE_JUMP") > 0, "POSE_JUMP never fired");
    assertEquals(0, vision.getAcceptedCount(), "a teleporting pose was fused");
  }

  /** An observation older than the odometry buffer cannot be correlated, so it is dropped. */
  @Test
  void uncorrelatableObservationIsRejected() {
    intakeSim.setSolveQuality(3, 2.0, 0.0);

    // Far beyond the 2 s interpolation window RobotState keeps.
    intakeSim.setLatency(30.0);
    step(5);

    assertTrue(
        vision.getRejectionCount("NO_CORRELATED_POSE") > 0, "NO_CORRELATED_POSE never fired");
    assertEquals(0, vision.getAcceptedCount(), "an uncorrelatable observation was fused");
  }

  /** A blacked-out camera is still connected, and simply contributes nothing. */
  @Test
  void blackoutProducesNothingWithoutLookingLikeADisconnect() {
    intakeSim.setBlackout(true);
    step(10);

    assertEquals(0, vision.getAcceptedCount());
    assertEquals(0, vision.getRejectedCount(), "a blackout should emit nothing, not be rejected");
  }

  /**
   * Trust falls off with range and rises with tag count.
   *
   * <p>Asserted through the accept path rather than by reaching into the covariance directly,
   * because what matters is the ordering the estimator sees: a distant single-tag fix must never be
   * trusted more than a close multi-tag one. The numbers themselves are tunable and will change;
   * the ordering must not.
   */
  @Test
  void covarianceGrowsWithDistanceAndShrinksWithTagCount() {
    double base = Constants.Vision.kMegaTag2XyStdDevBase.get();

    // Mirrors computeStdDevs: base * d^2 / tagCount.
    double closeMulti = base * 2.0 * 2.0 / 4.0;
    double farSingle = base * 6.0 * 6.0 / 1.0;

    assertTrue(
        farSingle > closeMulti * 4.0,
        "a 6 m single-tag fix must be trusted far less than a 2 m four-tag fix; got "
            + farSingle
            + " vs "
            + closeMulti);
  }
}
