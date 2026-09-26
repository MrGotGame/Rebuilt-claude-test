package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IdealPivotIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for the parts of the autonomous routines that run without PathPlanner.
 *
 * <p>Added for M10b step 9. PathPlanner is still unconfigured, so every path-following building
 * block is a skeleton; {@link AutoRoutines#waitUntilIntakeRetaining()} is not, because it depends
 * only on the intake, and it is the one piece of the hopper-era auto design with a safety role: the
 * robot starts with the intake stowed and 8 pieces aboard, and a routine that drove off before the
 * intake came down would shed them.
 *
 * <p>These cases run on an intake alone, through {@link AutoRoutines#waitUntilRetaining}, because
 * the wait reads nothing else. Until M13 step 4 they built an {@code AutoFactory} holding only the
 * intake; once its constructor began configuring PathPlanner and registering markers against the
 * superstructure, that stopped being possible, and the wait was split out as a function of the
 * intake rather than giving the factory a test constructor.
 */
public class AutoRoutinesTest {
  private static final double LOOP_SEC = 0.02;

  private Intake intake;

  /** The whole simulated robot, for the cases that drive paths; null for the intake-only ones. */
  private SimulatedAutoRobot robot;

  /** Initialises the HAL and pauses the clock, as every mechanism suite does. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();
  }

  /** Releases the HAL and unregisters the intake so cases cannot leak into each other. */
  @AfterEach
  void tearDown() {
    if (robot != null) {
      robot.close();
    }
    if (intake != null) {
      CommandScheduler.getInstance().unregisterSubsystem(intake);
    }
    CommandScheduler.getInstance().cancelAll();
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  /**
   * Builds the intake the wait is tested on.
   *
   * @param io the intake's pivot and roller hardware
   *     <p><b>Outputs:</b> sets {@link #intake}.
   */
  private void buildIntake(IntakeIO io) {
    intake = new Intake(io, IdealPivotIO.GEOMETRY);
    intake.setDeployGuard(() -> true);
  }

  /**
   * Advances the intake by a number of loops.
   *
   * @param loops how many 20 ms iterations to run
   *     <p><b>Outputs:</b> runs the intake's periodic and advances the simulated clock.
   */
  private void step(int loops) {
    for (int i = 0; i < loops; i++) {
      intake.periodic();
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  /**
   * Runs a started command one loop at a time until it finishes or a limit is reached.
   *
   * @param command a command that has already been initialised
   * @param maxLoops how many loops to allow
   * @return the loops it took to finish, or {@code maxLoops + 1} if it never did
   */
  private int loopsUntilFinished(Command command, int maxLoops) {
    for (int loops = 1; loops <= maxLoops; loops++) {
      step(1);
      command.execute();
      if (command.isFinished()) {
        return loops;
      }
    }
    return maxLoops + 1;
  }

  /** The configured timeout, in loops. */
  private static int timeoutLoops() {
    return (int) Math.round(Constants.Auto.kIntakeRetainTimeoutSec.get() / LOOP_SEC);
  }

  /**
   * A routine does not move while the intake is stowed, and moves as soon as it retains.
   *
   * <p>This is the match start: homed, stowed, 8 pieces aboard. The wait must hold for as long as
   * the intake stays up, well short of the timeout, and end within a loop or two of the intake
   * coming down -- a wait that ended at once would drop the preload, and one that ran to its
   * timeout regardless would waste a second of every auto.
   */
  @Test
  void aRoutineWaitsForTheIntakeToComeDown() {
    buildIntake(new IdealPivotIO());
    step(40);
    assertTrue(intake.isHomed(), "the intake never homed, so this case tests nothing");
    assertFalse(intake.isRetainingPieces(), "a stowed intake must not count as retaining");

    Command wait = AutoRoutines.waitUntilRetaining(intake);
    wait.initialize();
    int stowedLoops = timeoutLoops() / 2;
    assertEquals(
        stowedLoops + 1,
        loopsUntilFinished(wait, stowedLoops),
        "the routine set off while the intake was still stowed");

    intake.setWantedState(Intake.WantedState.DEPLOY_ONLY);
    int loops = loopsUntilFinished(wait, timeoutLoops());
    wait.end(false);
    assertTrue(loops <= 3, "the wait outlived the intake coming down by " + loops + " loops");
    assertTrue(intake.isRetainingPieces());
  }

  /**
   * The partial raise counts as retaining: an intake rocking in feed-assist keeps its pieces.
   *
   * <p>The wait keys on {@link Intake#isRetainingPieces()}, not on the intake being at its intaking
   * angle. The two differ exactly here, and a wait on the wrong one would hold a routine whose
   * intake was already safe for the whole timeout.
   */
  @Test
  void aFeedAssistingIntakeCountsAsRetaining() {
    buildIntake(new IdealPivotIO());
    step(40);
    intake.setWantedState(Intake.WantedState.FEED_ASSIST);
    step(5);
    assertEquals(Intake.SystemState.FEED_ASSISTING, intake.getSystemState());
    assertFalse(intake.isAtIntakingAngle(), "the case needs an intake off its intaking angle");

    Command wait = AutoRoutines.waitUntilRetaining(intake);
    wait.initialize();
    int loops = loopsUntilFinished(wait, timeoutLoops());
    wait.end(false);
    assertTrue(loops <= 2, "a feed-assisting intake held the routine for " + loops + " loops");
  }

  /**
   * A dead pivot costs the routine the timeout, not the auto.
   *
   * <p>The deliberate-failure case. Hardware that never moves never homes and never retains, so
   * without the bound the robot would sit at the starting line for the whole period. The bound is
   * checked in both directions: the wait must not give up early either, or a slow but working pivot
   * would be driven away from.
   */
  @Test
  void aDeadPivotCostsTheTimeoutNotTheAuto() {
    buildIntake(new IntakeIO() {});

    Command wait = AutoRoutines.waitUntilRetaining(intake);
    wait.initialize();
    int loops = loopsUntilFinished(wait, 3 * timeoutLoops());
    wait.end(false);
    assertFalse(intake.isRetainingPieces(), "the dead intake claimed to retain");
    assertEquals(
        timeoutLoops(),
        loops,
        2,
        "the wait did not end at kIntakeRetainTimeoutSec with a dead pivot");
  }

  // ------------------------------------------------------ step 5: the building blocks ----

  /** Where the building-block cases start: 3 m from the goal, strafing across its line. */
  private static final Pose2d STRAFE_START = new Pose2d(-3.0, -1.5, Rotation2d.kZero);

  /** Where they end. */
  private static final Pose2d STRAFE_END = new Pose2d(-3.0, 1.5, Rotation2d.kZero);

  /** A 3 m strafe across the goal's line at 1 m/s, ending stopped. */
  private static PathPlannerPath strafe(String name) {
    return SimulatedAutoRobot.straightPath(name, STRAFE_START, STRAFE_END, 1.0, 0.0);
  }

  /** The strafe back again, for a second leg. */
  private static PathPlannerPath strafeBack(String name) {
    return SimulatedAutoRobot.straightPath(name, STRAFE_END, STRAFE_START, 1.0, 0.0);
  }

  /**
   * Builds the robot at a start, with the intake already down, as it is after an auto's first wait.
   *
   * @param start where to stand
   * @param paths the paths available
   *     <p><b>Outputs:</b> sets {@link #robot}.
   */
  private void buildDeployedRobot(Pose2d start, PathPlannerPath... paths) {
    robot = new SimulatedAutoRobot(start, SimulatedAutoRobot.pathsOf(paths), false);
    robot.superstructure.onEnable();
    for (int i = 0; i < 100 && !robot.intake.isRetainingPieces(); i++) {
      robot.step();
    }
    assertTrue(robot.intake.isRetainingPieces(), "the intake never came down");
  }

  /**
   * Loops allowed for the simulated turret to settle after a large move, about twelve seconds.
   *
   * <p>On its placeholder gains it rings for about eight seconds before holding inside tolerance,
   * as M10b's checkpoint found; M6 characterization is where that gets fixed. A 3 m scoring path at
   * 1 m/s is over long before then.
   */
  private static final int TURRET_SETTLE_LOOPS = 600;

  /**
   * Spins up standing still until the turret is on its aim point, as a pickup leg wrapped in {@link
   * AutoRoutines#spinUpWhile} leaves the robot before a scoring leg.
   *
   * <p><b>Outputs:</b> runs the robot in SPIN_UP; leaves the request standing for the next leg.
   */
  private void preAim() {
    robot.superstructure.setWantedState(Superstructure.WantedState.SPIN_UP);
    int settled = 0;
    for (int i = 0; i < TURRET_SETTLE_LOOPS && settled < 25; i++) {
      robot.step();
      settled = robot.turret.atGoal() ? settled + 1 : 0;
    }
    assertEquals(25, settled, "the turret never settled on its aim point");
  }

  /** What a leg did, loop by loop, while it ran. */
  private static final class LegRecord {
    int loops;
    int scoringLoops;
    int spinningUpLoops;
    int intakingLoops;
    int intakingAndScoringLoops;
    int intakingAndSpinningLoops;
    int stillIntakingLoopsAtEnd;
    int firstScoringLoop = -1;
    int firstIntakingLoop = -1;
  }

  /**
   * Schedules a leg and runs the robot until it ends, recording what the superstructure did.
   *
   * @param leg the command under test
   * @param maxLoops how many loops to allow
   * @return the record
   */
  private LegRecord runLeg(Command leg, int maxLoops) {
    LegRecord record = new LegRecord();
    CommandScheduler.getInstance().schedule(leg);
    while (leg.isScheduled() && record.loops < maxLoops) {
      robot.step();
      boolean scoring =
          robot.superstructure.getSystemState() == Superstructure.SystemState.SCORING_ON_THE_MOVE;
      boolean spinning =
          robot.superstructure.getSystemState() == Superstructure.SystemState.SPINNING_UP;
      boolean intaking = robot.superstructure.getIntakeMode() == Superstructure.IntakeMode.INTAKING;
      record.scoringLoops += scoring ? 1 : 0;
      record.spinningUpLoops += spinning ? 1 : 0;
      record.intakingLoops += intaking ? 1 : 0;
      record.intakingAndScoringLoops += intaking && scoring ? 1 : 0;
      record.intakingAndSpinningLoops += intaking && spinning ? 1 : 0;
      if (intaking && robot.speedMetersPerSec() < 0.05) {
        record.stillIntakingLoopsAtEnd++;
      } else if (robot.speedMetersPerSec() >= 0.05) {
        record.stillIntakingLoopsAtEnd = 0;
      }
      if (scoring && record.firstScoringLoop < 0) {
        record.firstScoringLoop = record.loops;
      }
      if (intaking && record.firstIntakingLoop < 0) {
        record.firstIntakingLoop = record.loops;
      }
      record.loops++;
    }
    assertFalse(leg.isScheduled(), "the leg never ended");
    return record;
  }

  /**
   * Asserts that the shooter side went back to rest when the leg ended, and stays there.
   *
   * <p><b>Outputs:</b> steps the robot a few loops.
   */
  private void assertShooterSideRests() {
    robot.step(3);
    assertEquals(
        Superstructure.SystemState.RESTING,
        robot.superstructure.getSystemState(),
        "the leg's request outlived the leg");
  }

  /**
   * A scoring leg scores while its path runs, fires pieces, and stops scoring when the path ends.
   *
   * <p>The plan's form set AUTO_SCORE_ON_MOVE once and never took it back, so the robot went on
   * shooting into whatever leg came next. The leg must hand the shooter side back when it ends.
   *
   * <p>The turret is pre-aimed first, as a spun-up pickup would leave it: a scoring leg in a real
   * routine never starts from a stowed turret, and on its placeholder gains the simulated turret
   * would still be ringing when the path ended.
   */
  @Test
  void aScoringLegScoresForExactlyItsPath() {
    buildDeployedRobot(STRAFE_START, strafe("Score"));
    preAim();
    LegRecord leg = runLeg(robot.routines.driveAndShoot("Score"), 600);
    assertTrue(leg.scoringLoops > leg.loops / 2, "scored for only " + leg.scoringLoops + " loops");
    assertTrue(robot.piecesFired > 0, "the leg never fired a piece on the move");
    assertEquals(0.0, robot.pose().getTranslation().getDistance(STRAFE_END.getTranslation()), 0.15);
    assertShooterSideRests();
  }

  /**
   * A pickup leg intakes along its path and for the dwell after it, then leaves the intake down.
   *
   * <p>The dwell is what gives pieces still at the mouth time to come aboard; the intake staying
   * down afterwards is what keeps them (section 2.4).
   */
  @Test
  void aPickupLegIntakesThroughTheDwellAndLeavesTheIntakeDown() {
    buildDeployedRobot(STRAFE_START, strafe("Pickup"));
    robot.fieldPieces = 5;
    int aboard = robot.piecesAboard;
    LegRecord leg = runLeg(robot.routines.driveAndIntake("Pickup"), 600);
    int dwellLoops = (int) Math.round(Constants.Auto.kIntakeSettleSec.get() / LOOP_SEC);
    assertTrue(leg.firstIntakingLoop >= 0 && leg.firstIntakingLoop <= 2, "slow to start intaking");
    assertTrue(
        Math.abs(leg.stillIntakingLoopsAtEnd - dwellLoops) <= 3,
        "intaking went on for "
            + leg.stillIntakingLoopsAtEnd
            + " loops after the robot stopped; the dwell is "
            + dwellLoops);
    assertTrue(robot.piecesAboard > aboard, "the pickup collected nothing");
    robot.step(3);
    assertEquals(Superstructure.IntakeMode.DEPLOYED, robot.superstructure.getIntakeMode());
    assertEquals(Superstructure.SystemState.RESTING, robot.superstructure.getSystemState());
  }

  /** A sweep intakes and scores at once, then rests with the intake down. */
  @Test
  void aSweepIntakesAndScoresAtOnce() {
    buildDeployedRobot(STRAFE_START, strafe("Sweep"));
    robot.fieldPieces = 5;
    LegRecord leg = runLeg(robot.routines.driveIntakeAndShoot("Sweep"), 600);
    assertTrue(leg.intakingAndScoringLoops > 0, "never intaking and scoring together");
    assertShooterSideRests();
    assertEquals(Superstructure.IntakeMode.DEPLOYED, robot.superstructure.getIntakeMode());
  }

  /** A pickup inside a spin-up intakes with the flywheel spinning, then hands the shooter back. */
  @Test
  void aSpunUpPickupSpinsWhileItIntakes() {
    buildDeployedRobot(STRAFE_START, strafe("Pickup"));
    LegRecord leg =
        runLeg(robot.routines.spinUpWhile(robot.routines.driveAndIntake("Pickup")), 600);
    assertTrue(
        leg.intakingAndSpinningLoops > leg.loops / 2,
        "spun up while intaking for only " + leg.intakingAndSpinningLoops + " loops");
    assertShooterSideRests();
  }

  /**
   * A stationary shot inside a spin-up fires the whole preload.
   *
   * <p>This is the plan's own {@code preloadOnly} composition. The burst must fire all eight and
   * end on the empty belief, well before the bound.
   */
  @Test
  void aSpunUpStationaryShotFiresThePreload() {
    buildDeployedRobot(new Pose2d(-3.0, 0.0, Rotation2d.kZero));
    robot.shooter.resetShotCount();
    LegRecord leg =
        runLeg(
            robot.routines.spinUpWhile(
                robot.routines.shootInPlace(Constants.Auto.kPreloadTimeoutSec)),
            1000);
    assertEquals(SimulatedAutoRobot.PRELOAD, robot.piecesFired, "not every preloaded piece fired");
    assertEquals(SimulatedAutoRobot.PRELOAD, robot.shooter.getShotCount());
    assertTrue(
        leg.loops < Constants.Auto.kPreloadTimeoutSec.get() / LOOP_SEC,
        "the burst ran to its bound instead of ending on the empty belief");
    assertShooterSideRests();
  }

  /**
   * A spin-up never overrides a request made by the leg it wraps.
   *
   * <p>The plan wrote spinUpWhile as a deadline group with the spin-up beside the leg. A deadline
   * group starts its deadline first, so a leg whose first action is a shooter-side request had it
   * overwritten by SPIN_UP in the same instant. A scoring leg wrapped that way spins up for its
   * whole path and never scores. (The preload case above cannot see this: shootInPlace stops the
   * drive first, which happens to put its request after the spin-up.)
   */
  @Test
  void aSpinUpNeverOverridesTheLegItWraps() {
    buildDeployedRobot(STRAFE_START, strafe("Score"));
    LegRecord leg = runLeg(robot.routines.spinUpWhile(robot.routines.driveAndShoot("Score")), 600);
    assertTrue(
        leg.scoringLoops > leg.loops / 2,
        "the wrapped scoring leg scored for only " + leg.scoringLoops + " of " + leg.loops);
    assertShooterSideRests();
  }

  /**
   * A stationary shot cancelled partway still hands the shooter side back.
   *
   * <p>A request set once and cleared by a later step in a sequence is never cleared if the
   * sequence is cancelled, which is exactly what a timeout around it does.
   */
  @Test
  void aCancelledStationaryShotStillRests() {
    buildDeployedRobot(new Pose2d(-3.0, 0.0, Rotation2d.kZero));
    Command shot = robot.factory.scoreStationaryCommand();
    CommandScheduler.getInstance().schedule(shot);
    robot.step(20);
    assertEquals(
        Superstructure.SystemState.SCORING_STATIONARY, robot.superstructure.getSystemState());
    shot.cancel();
    assertShooterSideRests();
  }

  /**
   * A sequence alternates a spun-up pickup with a scoring leg, in that order.
   *
   * <p>And without shooting between, it scores nothing at all: it only drives.
   */
  @Test
  void aSequenceAlternatesPickupAndScore() {
    buildDeployedRobot(STRAFE_START, strafe("SeqPickup"), strafeBack("SeqScore"));
    LegRecord leg = runLeg(robot.routines.sequenceOf(List.of("SeqPickup", "SeqScore"), true), 1500);
    assertTrue(leg.intakingAndSpinningLoops > 0, "the first leg was not a spun-up pickup");
    assertTrue(leg.scoringLoops > 0, "the second leg never scored");
    assertTrue(
        leg.firstScoringLoop > leg.firstIntakingLoop, "scored before the pickup; wrong order");
    assertEquals(
        0.0, robot.pose().getTranslation().getDistance(STRAFE_START.getTranslation()), 0.15);
    assertShooterSideRests();
  }

  /** Without shooting between, a sequence only drives. */
  @Test
  void aSequenceWithoutShootingOnlyDrives() {
    buildDeployedRobot(STRAFE_START, strafe("SeqPickup"), strafeBack("SeqScore"));
    LegRecord leg =
        runLeg(robot.routines.sequenceOf(List.of("SeqPickup", "SeqScore"), false), 1500);
    assertEquals(0, leg.scoringLoops + leg.intakingLoops + leg.spinningUpLoops);
    assertEquals(
        0.0, robot.pose().getTranslation().getDistance(STRAFE_START.getTranslation()), 0.15);
  }
}
