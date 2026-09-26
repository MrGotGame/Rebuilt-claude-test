package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IdealPivotIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
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
}
