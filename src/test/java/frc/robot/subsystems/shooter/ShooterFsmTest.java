package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Transition tests for the flywheel.
 *
 * <p>M4 shipped without a suite because Gradle could not be run from the agent sandbox at the time,
 * and the milestone's checkpoint was discharged by watching AdvantageScope instead. That made the
 * flywheel the least-proven mechanism in the project despite being the template the other four were
 * copied from, which is the wrong way round. This closes it.
 */
public class ShooterFsmTest {
  private static final double LOOP_SEC = 0.02;

  private Shooter shooter;

  /** Builds a shooter over simulated IO with the HAL initialised and the clock paused. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();
    shooter = new Shooter(new ShooterIOSim());
  }

  /** Releases the HAL and unregisters the subsystem so cases cannot leak into each other. */
  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().unregisterSubsystem(shooter);
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
      shooter.periodic();
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  /** Replaces the mechanism under test with one built over the given IO. */
  private void rebuildOver(ShooterIO io) {
    CommandScheduler.getInstance().unregisterSubsystem(shooter);
    shooter = new Shooter(io);
  }

  /** Loops needed to outrun the spin-up timeout, with margin. */
  private int loopsPastSpinUpTimeout() {
    return (int) (Constants.Shooter.kSpinUpTimeoutSec.get() / LOOP_SEC) + 30;
  }

  /** IDLE holds the idle speed, but does not claim to be at speed with no shot commanded. */
  @Test
  void idleHoldsIdleSpeedWithoutClaimingReadiness() {
    step(150);

    assertEquals(Shooter.SystemState.IDLE, shooter.getSystemState());
    assertTrue(
        shooter.getVelocityRpm() > 0.0,
        "IDLE is a closed-loop hold at kIdleRpm, not a stop; the wheel should be turning");

    // The falsification: goal is zero at boot while velocity is not, so a bare
    // |velocity - goal| < tolerance test would be false here anyway -- but the inverse case, both
    // zero at rest, is what the non-zero-goal term in atSpeed() actually guards. Either way an
    // idling flywheel with nothing requested must not report itself ready to shoot.
    assertFalse(shooter.atSpeed(), "an idling flywheel must not report readiness");
  }

  /** Every SystemState is reachable. */
  @Test
  void everyStateIsReachable() {
    assertEquals(Shooter.SystemState.IDLE, shooter.getSystemState(), "boot state must be IDLE");

    shooter.setWantedState(Shooter.WantedState.COAST);
    step(5);
    assertEquals(Shooter.SystemState.COASTING, shooter.getSystemState());

    shooter.setGoal(Constants.Shooter.kBenchTestRpm.get());
    step(5);
    assertEquals(Shooter.SystemState.SPINNING_UP, shooter.getSystemState());

    step(300);
    assertEquals(
        Shooter.SystemState.AT_SPEED,
        shooter.getSystemState(),
        "never converged; measured " + shooter.getVelocityRpm());

    shooter.setWantedState(Shooter.WantedState.CHARACTERIZE);
    step(5);
    assertEquals(Shooter.SystemState.CHARACTERIZING, shooter.getSystemState());

    shooter.setWantedState(Shooter.WantedState.IDLE);
    step(5);
    assertEquals(Shooter.SystemState.IDLE, shooter.getSystemState());
  }

  /** Commanded a speed, the flywheel converges on it and says so. */
  @Test
  void reachesCommandedSpeedUnderClosedLoop() {
    shooter.setGoal(Constants.Shooter.kBenchTestRpm.get());
    step(300);

    assertTrue(shooter.atSpeed(), "did not converge; measured " + shooter.getVelocityRpm());
    assertEquals(
        Constants.Shooter.kBenchTestRpm.get(),
        shooter.getVelocityRpm(),
        Constants.Shooter.kToleranceRpm.get(),
        "converged outside the tolerance band");
  }

  /**
   * Predictive readiness leads instantaneous readiness, which is the whole reason it exists.
   *
   * <p>Plan section 6.5 makes {@code willBeAtSpeedIn} gate 3 of the shot ladder: a piece takes
   * roughly 40 ms to reach the wheel, so the question is whether the wheel will be at speed when it
   * arrives. If the projection never leads the measurement there is no cadence being bought and the
   * filtered acceleration estimate is not working.
   */
  @Test
  void predictiveReadinessLeadsInstantaneousReadiness() {
    shooter.setGoal(Constants.Shooter.kBenchTestRpm.get());

    int firstPredicted = -1;
    int firstActual = -1;
    double horizon = Constants.getLoopPeriodSec() * 2.0;

    for (int i = 0; i < 400; i++) {
      shooter.periodic();
      SimHooks.stepTiming(LOOP_SEC);
      if (firstPredicted < 0 && shooter.willBeAtSpeedIn(horizon)) {
        firstPredicted = i;
      }
      if (firstActual < 0 && shooter.atSpeed()) {
        firstActual = i;
      }
    }

    assertTrue(firstActual >= 0, "the flywheel never reached speed at all");
    assertTrue(firstPredicted >= 0, "willBeAtSpeedIn never went true");
    assertTrue(
        firstPredicted < firstActual,
        "prediction did not lead measurement: predicted at loop "
            + firstPredicted
            + ", actual at loop "
            + firstActual);
  }

  /**
   * A goal of zero resolves to IDLE rather than faulting.
   *
   * <p>Not in the plan's specification, and load-bearing: {@code Superstructure} calls {@code
   * setGoal(0.0)} whenever the shooting solver returns an unusable solution, which — with the shot
   * tables still all zeros — is its shipped behaviour today. Without this guard a healthy flywheel
   * would fault after {@code kSpinUpTimeoutSec} on a robot where nothing is broken.
   */
  @Test
  void zeroGoalResolvesToIdleRatherThanFaulting() {
    shooter.setGoal(0.0);
    step(loopsPastSpinUpTimeout());

    assertEquals(Shooter.SystemState.IDLE, shooter.getSystemState());
    assertNotEquals(
        Shooter.SystemState.FAULTED,
        shooter.getSystemState(),
        "a zero goal means nothing to shoot at, not a broken mechanism");
  }

  /** Hardware that cannot reach speed faults rather than blocking the scoring state forever. */
  @Test
  void unreachableSpeedFaultsAndLatches() {
    rebuildOver(new ShooterIO() {});

    shooter.setGoal(Constants.Shooter.kBenchTestRpm.get());
    step(loopsPastSpinUpTimeout());

    assertEquals(
        Shooter.SystemState.FAULTED,
        shooter.getSystemState(),
        "a flywheel that cannot reach speed must fault, not spin up forever");
    assertFalse(shooter.atSpeed(), "a faulted flywheel must never report readiness");

    // The latch: still asking to shoot must not shake it loose.
    shooter.setWantedState(Shooter.WantedState.SHOOT);
    step(50);
    assertEquals(
        Shooter.SystemState.FAULTED, shooter.getSystemState(), "SHOOT must not clear the fault");

    // A change of intent does clear it.
    shooter.setWantedState(Shooter.WantedState.IDLE);
    step(5);
    assertEquals(Shooter.SystemState.IDLE, shooter.getSystemState(), "IDLE must clear the fault");
  }

  /** setGoal clamps into the configured maximum rather than passing an absurd speed through. */
  @Test
  void goalIsClampedToTheConfiguredMaximum() {
    shooter.setGoal(Constants.Shooter.kMaxRpm.get() * 10.0);
    step(2);

    assertEquals(
        Constants.Shooter.kMaxRpm.get(),
        shooter.getGoalRpm(),
        1e-9,
        "an over-range goal must be clamped, not commanded");
  }
}
