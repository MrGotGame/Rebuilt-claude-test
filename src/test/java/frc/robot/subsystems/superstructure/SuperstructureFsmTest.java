package frc.robot.subsystems.superstructure;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

/**
 * Transition tests for the cross-mechanism state machine.
 *
 * <p>These tests build the superstructure on top of simulated IO implementations and step it loop
 * by loop, asserting which states it reaches and, just as importantly, which it refuses to reach.
 *
 * <p>Unlike the two pure-math test classes, these need the WPILib HAL because the subsystems use
 * {@code Timer} and the command scheduler. Call {@code HAL.initialize(500, 0)} in {@link #setUp()},
 * set {@code Constants.disableHal = true} so tunables do not touch NetworkTables, and release the
 * HAL in {@link #tearDown()}.
 *
 * <p>Every test is {@link Disabled} until written; delete the annotation as you implement each.
 */
public class SuperstructureFsmTest {
  private Superstructure superstructure;

  /** Builds a superstructure over simulated IO with the HAL initialised. */
  @BeforeEach
  void setUp() {
    // TODO: Constants.disableHal = true; HAL.initialize(500, 0); construct every subsystem over its
    // XxxIOSim, then the Superstructure. Do NOT construct the real IO classes -- they will try to
    // open CAN devices that do not exist and the failure mode is confusing.
  }

  /** Releases the HAL and clears the command scheduler. */
  @AfterEach
  void tearDown() {
    // TODO: CommandScheduler.getInstance().cancelAll(); HAL.shutdown(); Constants.disableHal =
    // false.
  }

  /**
   * Advances the state machine by a number of loops.
   *
   * @param loops how many 20 ms iterations to run
   *     <p><b>Outputs:</b> runs each subsystem's periodic that many times.
   */
  private void step(int loops) {
    // TODO: Loop calling CommandScheduler.getInstance().run() (or each subsystem's periodic
    // directly) `loops` times, so a test can express "after 50 loops" rather than sleeping.
  }

  /** The robot should home itself and settle into stowed with no input. */
  @Test
  @Disabled("TODO: implement")
  void startsInHomingAndReachesStowed() {
    // TODO: Step until homing completes and assert the state is STOWED.
  }

  /** A score request with no game piece must never reach a scoring state. */
  @Test
  @Disabled("TODO: implement")
  void scoreRequestWithoutPieceNeverReachesScoring() {
    // TODO: Request SCORE_ON_MOVE with the simulated indexer empty, step, and assert the state is
    // SPINNING_UP rather than any SCORING_* state.
  }

  /** Scoring on the move must require a usable shot solution. */
  @Test
  @Disabled("TODO: implement")
  void scoreOnMoveRequiresUsableShotSolution() {
    // TODO: With the shot table still all zeros -- which is its shipped state -- assert the machine
    // lands in WAITING_FOR_TURRET rather than SCORING_ON_THE_MOVE.
  }

  /** A mechanism that cannot home must fault rather than stall forever. */
  @Test
  @Disabled("TODO: implement")
  void homingTimesOutToFaulted() {
    // TODO: Jam the simulated pivot, step past the homing timeout, and assert FAULTED. Without this
    // timeout a jammed intake stalls a motor for the rest of the match.
  }

  /** Only an explicit home request may clear a fault. */
  @Test
  @Disabled("TODO: implement")
  void onlyHomeClearsFaulted() {
    // TODO: From FAULTED, request every other WantedState in turn and assert the state stays
    // FAULTED; then request HOME and assert it leaves.
  }

  /** The intake and turret must not be able to occupy each other's space. */
  @Test
  @Disabled("TODO: implement")
  void intakeAndTurretInterlockHoldsBothWays() {
    // TODO: Assert the intake refuses to deploy while the turret is in the conflict arc, AND that
    // the turret refuses to sweep through the arc while the intake is deployed. Both directions
    // matter -- an interlock enforced in only one direction is not an interlock.
  }

  /** Entry and exit actions must fire exactly once per transition. */
  @Test
  @Disabled("TODO: implement")
  void entryAndExitActionsFireExactlyOnce() {
    // TODO: Count invocations across a transition and assert exactly one each. A repeated onEnter
    // is how a state machine ends up re-seeding an encoder every loop.
  }

  /** Force-shoot must bypass the readiness gate but still aim. */
  @Test
  @Disabled("TODO: implement")
  void forceShootBypassesGatesButStillAims() {
    // TODO: Set ignoreShotTolerances, assert isReadyToScore() is true even with a stale pose, and
    // assert the turret goal is still being set -- forcing a shot should skip the gate, not the
    // aim.
  }

  /** Disabling must leave the machine in a safe state. */
  @Test
  @Disabled("TODO: implement")
  void disablingClearsWantedStateSafely() {
    // TODO: Call the disable path and assert no mechanism is left commanded and no goal leaks into
    // the next enable.
  }
}
