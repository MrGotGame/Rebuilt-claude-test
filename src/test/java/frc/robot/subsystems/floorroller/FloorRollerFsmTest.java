package frc.robot.subsystems.floorroller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Transition tests for the floor roller, standing in for the half of M5's per-mechanism checkpoint
 * that does not need a robot.
 *
 * <p>The checkpoint asks that the mechanism move under closed loop and that every FSM state be
 * reachable and logged. Watching that happen on a graph proves it once; asserting it here proves it
 * on every build, and — more importantly for this repository's standard — it can be
 * <i>falsified</i> by breaking the implementation and confirming a test goes red.
 */
public class FloorRollerFsmTest {
  private static final double LOOP_SEC = 0.02;

  private FloorRoller floorRoller;

  /** Builds a floor roller over simulated IO with the HAL initialised and the clock paused. */
  @BeforeEach
  void setUp() {
    // HAL first, then disableHal. Touching Constants.disableHal initialises Constants, which
    // constructs well over a hundred LoggedTunableNumbers; doing that before the HAL exists is
    // asking for trouble, and doing it after means every tunable reads its compiled-in default and
    // the test is deterministic.
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;

    // The FPGA clock must be driven by the test, not by wall time. Without this, a state timeout
    // measured in seconds would depend on how fast the machine running the suite happens to be.
    SimHooks.pauseTiming();

    floorRoller = new FloorRoller(new FloorRollerIOSim());
  }

  /** Releases the HAL and unregisters the subsystem so cases cannot leak into each other. */
  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().unregisterSubsystem(floorRoller);
    CommandScheduler.getInstance().cancelAll();
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  /**
   * Advances the mechanism by a number of loops.
   *
   * @param loops how many 20 ms iterations to run
   *     <p><b>Outputs:</b> runs periodic that many times and advances the simulated clock with it.
   */
  private void step(int loops) {
    for (int i = 0; i < loops; i++) {
      floorRoller.periodic();
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  /** The machine starts idle and does not claim to be at speed with nothing commanded. */
  @Test
  void startsIdleAndNotAtSpeed() {
    step(5);
    assertEquals(FloorRoller.SystemState.IDLE, floorRoller.getSystemState());

    // The falsification that matters: at rest, velocity and goal are both zero, so a naive
    // |velocity - goal| < tolerance test would report true. If this ever goes green-by-accident,
    // atSpeed() has lost its non-zero-goal term.
    assertFalse(floorRoller.atSpeed(), "a stopped roller must not report itself at speed");
  }

  /** Every SystemState is reachable, which is clause 2 of the M5 checkpoint. */
  @Test
  void everyStateIsReachable() {
    assertStateReached(FloorRoller.WantedState.INTAKE, FloorRoller.SystemState.INTAKING);
    assertStateReached(FloorRoller.WantedState.FEED, FloorRoller.SystemState.FEEDING);
    assertStateReached(FloorRoller.WantedState.EJECT, FloorRoller.SystemState.EJECTING);
    assertStateReached(
        FloorRoller.WantedState.CHARACTERIZE, FloorRoller.SystemState.CHARACTERIZING);
    assertStateReached(FloorRoller.WantedState.IDLE, FloorRoller.SystemState.IDLE);
  }

  /** The roller actually closes the loop: commanded a speed, it converges and reports at-speed. */
  @Test
  void reachesCommandedSpeedUnderClosedLoop() {
    floorRoller.setWantedState(FloorRoller.WantedState.INTAKE);

    // 100 loops is two seconds of simulated time, which is far longer than a roller of this
    // inertia needs; the test is asserting convergence, not spin-up time.
    step(100);

    assertEquals(FloorRoller.SystemState.INTAKING, floorRoller.getSystemState());
    assertTrue(
        floorRoller.atSpeed(),
        "roller did not converge on its goal; measured " + floorRoller.getVelocityRpm());
    assertEquals(
        Constants.FloorRoller.kIntakeRpm.get(),
        floorRoller.getVelocityRpm(),
        Constants.FloorRoller.kToleranceRpm.get(),
        "converged velocity outside the tolerance band");
  }

  /** Eject runs the roller backwards, which is the sign convention the constant carries. */
  @Test
  void ejectRunsTheRollerBackwards() {
    floorRoller.setWantedState(FloorRoller.WantedState.EJECT);
    step(100);

    assertTrue(
        floorRoller.getVelocityRpm() < 0.0,
        "eject must drive the roller negative; measured " + floorRoller.getVelocityRpm());
  }

  /**
   * With hardware that never moves, the machine still transitions but never claims success.
   *
   * <p>This is the deliberate-failure case. The empty {@link FloorRollerIO} is a stand-in for a
   * roller whose motor is dead: the FSM must still walk its states, because state is a software
   * concept, while {@code atSpeed()} must stay false, because speed is a physical one. A test suite
   * that cannot tell those apart would pass against a robot that does not move.
   */
  @Test
  void deadHardwareTransitionsButNeverReportsAtSpeed() {
    CommandScheduler.getInstance().unregisterSubsystem(floorRoller);
    floorRoller = new FloorRoller(new FloorRollerIO() {});

    floorRoller.setWantedState(FloorRoller.WantedState.INTAKE);
    step(100);

    assertEquals(
        FloorRoller.SystemState.INTAKING,
        floorRoller.getSystemState(),
        "the state machine must still run when the mechanism cannot");
    assertFalse(
        floorRoller.atSpeed(), "atSpeed() must not report success against hardware that never ran");
  }

  /**
   * Requests {@code wanted} and asserts the machine settles into {@code expected}.
   *
   * @param wanted the behaviour to request
   * @param expected the state the machine should be running afterwards
   *     <p><b>Outputs:</b> steps the mechanism and fails the test on mismatch.
   */
  private void assertStateReached(
      FloorRoller.WantedState wanted, FloorRoller.SystemState expected) {
    floorRoller.setWantedState(wanted);
    step(5);
    assertEquals(expected, floorRoller.getSystemState(), "request " + wanted + " did not reach");
  }
}
