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

  /** The simulated IO under {@link #shooter}, kept so cases can inject game pieces into it. */
  private ShooterIOSim sim;

  /** Builds a shooter over simulated IO with the HAL initialised and the clock paused. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();
    sim = new ShooterIOSim();
    shooter = new Shooter(sim);
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

  /**
   * Rebuilds the shooter over a scripted trace, holding a goal the trace already sits exactly at.
   *
   * @param rpm the steady speed, and the goal
   * @return the script, for the case to queue a trace into
   *     <p><b>Outputs:</b> leaves the shooter AT_SPEED with no shot counted.
   */
  private ScriptedShooterIO settleOverScript(double rpm) {
    ScriptedShooterIO scripted = new ScriptedShooterIO(rpm);
    rebuildOver(scripted);
    shooter.setGoal(rpm);
    step(20);
    assertEquals(Shooter.SystemState.AT_SPEED, shooter.getSystemState());
    assertEquals(0, shooter.getShotCount(), "a steady scripted wheel counted a shot");
    return scripted;
  }

  /**
   * A flywheel IO that reports a scripted speed trace instead of simulating a plant.
   *
   * <p>The shot detector's contract is about speed traces, so the direct test of each of its
   * conditions is a trace chosen to reach that condition and no other. The simulated plant cannot
   * always produce one: its controller recovers or brakes on its own schedule, and the first
   * mutation run of this suite found three conditions it never isolated. Queued values are reported
   * one per update, and the held value after that.
   */
  private static final class ScriptedShooterIO implements ShooterIO {
    private final java.util.ArrayDeque<Double> queued = new java.util.ArrayDeque<>();
    private double heldRpm;

    /**
     * Builds a script that reports one steady speed until told otherwise.
     *
     * @param heldRpm the speed reported when nothing is queued, in RPM
     */
    ScriptedShooterIO(double heldRpm) {
      this.heldRpm = heldRpm;
    }

    /**
     * Queues speeds to report, one per update, before returning to the held speed.
     *
     * @param rpms the speeds, in RPM, in order
     *     <p><b>Outputs:</b> appends to the queue.
     */
    void then(double... rpms) {
      for (double rpm : rpms) {
        queued.add(rpm);
      }
    }

    /**
     * Changes the speed reported once the queue is empty.
     *
     * @param rpm the new held speed, in RPM
     *     <p><b>Outputs:</b> sets the held speed.
     */
    void hold(double rpm) {
      heldRpm = rpm;
    }

    /**
     * Reports the next scripted speed, with both motors connected.
     *
     * @param inputs the struct to populate
     */
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      inputs.leaderConnected = true;
      inputs.followerConnected = true;
      inputs.velocityRpm = queued.isEmpty() ? heldRpm : queued.poll();
    }
  }

  /** Loops needed to outrun the spin-up timeout, with margin. */
  private int loopsPastSpinUpTimeout() {
    return (int) (Constants.Shooter.kSpinUpTimeoutSec.get() / LOOP_SEC) + 30;
  }

  /**
   * Spins the wheel up to the bench speed and lets it settle, asserting that it got there.
   *
   * <p><b>Outputs:</b> leaves the shooter AT_SPEED, with no shot counted on the way. Every
   * shot-detection case starts here, so a detector that counts spin-ups fails at this line rather
   * than further down, where the cause would be less obvious.
   */
  private void spinUpAndSettle() {
    shooter.setGoal(Constants.Shooter.kBenchTestRpm.get());
    step(300);
    assertEquals(
        Shooter.SystemState.AT_SPEED,
        shooter.getSystemState(),
        "never converged; measured " + shooter.getVelocityRpm());
    assertEquals(0, shooter.getShotCount(), "the spin-up itself was counted as a shot");
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

  /**
   * A burst of pieces counts one shot per piece.
   *
   * <p>M10b moved shot counting from the indexer to the flywheel (plan section 2.4) precisely
   * because the old counter watched for the indexer emptying, and a hopper feeding back to back
   * never empties between pieces, so a whole burst would have counted as one. Eight pieces is the
   * preload, fired at ten per second, a cadence at which the wheel is still recovering from each
   * piece when the next arrives.
   */
  @Test
  void countsOneShotPerPieceInABurst() {
    spinUpAndSettle();

    for (int piece = 0; piece < 8; piece++) {
      sim.injectShot();
      step(5);
    }
    step(50);

    assertEquals(8, shooter.getShotCount(), "a burst of 8 pieces must count 8 shots");
  }

  /**
   * Nothing that slows or speeds the wheel on purpose counts as a shot, through the real closed
   * loop.
   *
   * <p>This is the INTEGRATION check: every everyday speed change, run through the simulated
   * flywheel and its controller. It is not, on its own, a test of the individual conditions. The
   * first mutation run showed why: this plant brakes a large goal cut at up to ~440 RPM a loop,
   * which is rejected by the arming band AND by "ends below the goal" at once, so deleting either
   * one alone changes nothing here; and it never brakes a small cut faster than ~110 RPM a loop, so
   * the small cut never reaches the "ends below the goal" check at all. The per-condition cases
   * below use a scripted trace instead. The drop-to-idle phase is the exception, and the reason
   * this case earns its place: it is the only thing exercising "holding a goal". The count is
   * checked after every phase so a failure names the phase that caused it.
   */
  @Test
  void deliberateSpeedChangesAreNotShots() {
    spinUpAndSettle();
    double bench = Constants.Shooter.kBenchTestRpm.get();
    double armBand = Constants.Shooter.kShotArmToleranceRpm.get();

    shooter.setGoal(bench - 0.75 * armBand);
    step(150);
    assertEquals(0, shooter.getShotCount(), "a small goal cut, inside the arming band, counted");

    shooter.setGoal(bench);
    step(150);
    assertEquals(0, shooter.getShotCount(), "raising the goal back counted");

    shooter.setGoal(bench - 3.0 * armBand);
    step(150);
    assertEquals(0, shooter.getShotCount(), "a large goal cut counted");

    // Dropping to IDLE is the everyday case: every scoring state ends this way. The goal still
    // holds the last shooting speed, so the wheel starts inside the arming band, and the idle hold
    // then brakes it hard toward kIdleRpm -- a sharp drop, ending below the goal. Only the
    // "holding a goal" condition rejects it.
    shooter.setGoal(bench);
    step(150);
    shooter.setWantedState(Shooter.WantedState.IDLE);
    step(150);
    assertEquals(0, shooter.getShotCount(), "dropping to idle counted");

    shooter.setGoal(bench);
    step(150);
    shooter.setWantedState(Shooter.WantedState.COAST);
    step(150);
    assertEquals(0, shooter.getShotCount(), "coasting down counted");
  }

  /**
   * A dip smaller than the threshold is not a piece, and the harness can tell the difference.
   *
   * <p>The second half is what keeps the first half honest. Without it, a detector that never
   * counts anything would pass the "sub-threshold dip does not count" assertion for free.
   */
  @Test
  void subThresholdDipIsNotAShot() {
    spinUpAndSettle();

    sim.injectShot(0.5 * Constants.Shooter.kShotDipRpm.get());
    step(50);
    assertEquals(0, shooter.getShotCount(), "a dip half the threshold was counted");

    sim.injectShot();
    step(50);
    assertEquals(1, shooter.getShotCount(), "a full-size dip was not counted");
  }

  /**
   * One piece's dip, spread over consecutive loops, counts once; the next piece counts again.
   *
   * <p>The refractory window is the only thing between a dip that spans two 20 ms loops and a
   * double count, so the second drop must be sharp enough to count on its own and land inside
   * {@code kShotRefractorySec}. That trace has to be scripted: the first version of this case
   * injected two pieces one loop apart into the simulated flywheel, whose controller recovered
   * enough between them that the second drop fell under the threshold, and the case passed with the
   * refractory window deleted.
   */
  @Test
  void refractoryWindowCountsOneDipOnce() {
    ScriptedShooterIO scripted = settleOverScript(3000.0);

    scripted.then(2800.0, 2600.0, 2700.0, 2800.0, 2900.0);
    step(2);
    assertEquals(1, shooter.getShotCount(), "one dip across two loops counted more than once");

    step(20);
    scripted.then(2800.0);
    step(5);
    assertEquals(2, shooter.getShotCount(), "a later, separate piece was not counted");
  }

  /**
   * A small goal cut, braked toward, is not a shot: the drop is sharp and the wheel was armed, but
   * it ends above the new goal.
   *
   * <p>The only condition standing between this trace and a count is "ends below the goal". The
   * simulated plant never brakes a small cut this sharply, which is why this needs a script; a real
   * flywheel with stiffer gains can. The positive control proves the scripted harness counts a real
   * dip at the new goal.
   */
  @Test
  void brakingTowardASlightlyLowerGoalIsNotAShot() {
    ScriptedShooterIO scripted = settleOverScript(3000.0);

    shooter.setGoal(2700.0);
    scripted.hold(2700.0);
    scripted.then(2800.0);
    step(20);
    assertEquals(0, shooter.getShotCount(), "braking to a lower goal, ending above it, counted");

    scripted.then(2500.0);
    step(5);
    assertEquals(1, shooter.getShotCount(), "a real dip at the new goal was not counted");
  }

  /**
   * Braking hard across a much lower goal is not a shot: the final drop is sharp and ends below the
   * goal, but it started too far above it.
   *
   * <p>The only condition standing between the last drop of this trace and a count is the arming
   * band. The simulated plant does brake a large cut this hard (~440 RPM a loop) but slows as it
   * approaches the goal, so it never crosses it sharply; a real wheel with stiffer gains can. The
   * positive control proves a real dip at the new goal still counts.
   */
  @Test
  void brakingHardAcrossAMuchLowerGoalIsNotAShot() {
    ScriptedShooterIO scripted = settleOverScript(3000.0);

    shooter.setGoal(1800.0);
    scripted.hold(1800.0);
    scripted.then(2600.0, 2300.0, 1700.0);
    step(20);
    assertEquals(0, shooter.getShotCount(), "braking across a much lower goal counted");

    scripted.then(1600.0);
    step(5);
    assertEquals(1, shooter.getShotCount(), "a real dip at the new goal was not counted");
  }

  /**
   * Time since the last shot is infinite before any shot, tracks the clock after one, and a reset
   * forgets both the time and the count.
   *
   * <p>Infinity rather than zero or the timer's raw value matters to the one live consumer: the
   * superstructure's "probably empty" belief reads a long time since the last shot as evidence of
   * an empty hopper, and a timer that had silently been running since boot would be exactly that
   * evidence, invented.
   */
  @Test
  void secondsSinceLastShotTracksTheClockAndResets() {
    assertEquals(Double.POSITIVE_INFINITY, shooter.getSecondsSinceLastShot(), "before any shot");

    spinUpAndSettle();
    sim.injectShot();
    step(1);
    assertEquals(1, shooter.getShotCount());
    step(10);
    // Eleven, not ten: the shot was stamped inside step(1)'s periodic, before that loop's clock
    // step, so the clock has advanced once in step(1) and ten more times since.
    assertEquals(
        11 * LOOP_SEC, shooter.getSecondsSinceLastShot(), 0.5 * LOOP_SEC, "clock since the shot");

    shooter.resetShotCount();
    assertEquals(0, shooter.getShotCount(), "reset must zero the count");
    assertEquals(
        Double.POSITIVE_INFINITY,
        shooter.getSecondsSinceLastShot(),
        "reset must forget the last shot, not report the time since it");
  }

  /**
   * The burst band accepts a recovering wheel that the normal band rejects.
   *
   * <p>This is the property the superstructure's burst latch depends on. After one piece knocks the
   * wheel down, the tight band says "not ready" and would stop the indexer; the burst band says
   * "ready" and keeps the stream going. If both answered the same, the overload would be
   * decoration.
   *
   * <p>Scripted, and held long enough for the acceleration estimate to settle to zero, so the
   * shortfall being judged is exactly midway between the two bands. The first version injected a
   * dip into the simulated plant, and its answer depended on that plant's steady-state error (it
   * settles ~53 RPM short of goal on the guessed kV) plus the projection term; it passed or failed
   * on how the dip was delivered rather than on the property.
   */
  @Test
  void burstToleranceAcceptsARecoveringWheel() {
    double goal = 3000.0;
    ScriptedShooterIO scripted = settleOverScript(goal);
    double horizon = Constants.getLoopPeriodSec() * 2.0;
    assertTrue(shooter.willBeAtSpeedIn(horizon), "a settled wheel must be ready");

    double midway =
        0.5 * (Constants.Shooter.kToleranceRpm.get() + Constants.Shooter.kBurstToleranceRpm.get());
    scripted.hold(goal - midway);
    step(20);

    assertFalse(
        shooter.willBeAtSpeedIn(horizon), "the normal band accepted a wheel still recovering");
    assertTrue(
        shooter.willBeAtSpeedIn(horizon, Constants.Shooter.kBurstToleranceRpm.get()),
        "the burst band rejected a wheel recovering within it");
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

  // ------------------------------------------------------ M12 step 1: the saturated projection
  // ----

  /**
   * A wheel closing on its goal is ready continuously from the first loop the prediction says so,
   * however hard it was accelerating.
   *
   * <p>The trace is a closed-loop spin-up: fast, then easing into the goal. The acceleration is a
   * five-sample moving average, so near the goal it is still near its peak, and the unsaturated
   * projection overshot the band for several loops after first going true -- the gate opened and
   * then dropped, stopping the indexer mid-burst. That is the stutter M11 found. The positive
   * control proves the trace really does overshoot: the raw projection leaves the band.
   */
  @Test
  void aWheelClosingOnItsGoalStaysReadyOnceReady() {
    ScriptedShooterIO scripted = new ScriptedShooterIO(0.0);
    rebuildOver(scripted);
    double goal = 3000.0;
    shooter.setGoal(goal);
    double horizon = LOOP_SEC * 2.0;
    scripted.then(0, 400, 800, 1200, 1600, 2000, 2400, 2750, 2950, 2990, 3000);
    scripted.hold(goal);

    boolean ready = false;
    boolean rawOvershot = false;
    for (int i = 0; i < 30; i++) {
      step(1);
      double raw = shooter.getVelocityRpm() + shooter.getAccelRpmPerSec() * horizon;
      if (ready) {
        rawOvershot |= Math.abs(raw - goal) >= Constants.Shooter.kToleranceRpm.get();
        assertTrue(
            shooter.willBeAtSpeedIn(horizon),
            "ready, then not ready, at " + shooter.getVelocityRpm() + " RPM, loop " + i);
      }
      ready |= shooter.willBeAtSpeedIn(horizon);
    }
    assertTrue(ready, "the wheel never read ready");
    assertTrue(rawOvershot, "the trace never overshot, so the saturation was never needed");
  }

  /**
   * The saturation only helps a projection that reaches the goal: a wheel still well short of it,
   * or one moving away from it, is not ready.
   *
   * <p>"Never fires early" was the old projection's one guarantee, and saturating it must not spend
   * it. Both traces hold a steady slope so the average has settled on it.
   */
  @Test
  void aProjectionThatDoesNotReachTheGoalIsStillNotReady() {
    ScriptedShooterIO scripted = new ScriptedShooterIO(0.0);
    rebuildOver(scripted);
    shooter.setGoal(3000.0);
    double horizon = LOOP_SEC * 2.0;

    // Short: climbing 100 RPM a loop from 1000, it projects 200 RPM ahead, nowhere near 3000.
    scripted.then(1000, 1100, 1200, 1300, 1400, 1500, 1600);
    step(7);
    assertFalse(shooter.willBeAtSpeedIn(horizon), "a wheel 1,400 RPM short read ready");

    // Away: above the goal and still climbing.
    scripted.then(3200, 3300, 3400, 3500, 3600, 3700, 3800);
    step(7);
    assertFalse(shooter.willBeAtSpeedIn(horizon), "a wheel climbing away from its goal read ready");

    // Away, the other side: below the goal and falling.
    scripted.then(2900, 2800, 2700, 2600, 2500, 2400, 2300);
    step(7);
    assertFalse(shooter.willBeAtSpeedIn(horizon), "a wheel falling away from its goal read ready");
  }
}
