package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IdealPivotIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.PowerBudget.Priority;
import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for {@link PowerBudget}: the tier ordering, the filter, and the brownout prediction.
 *
 * <p>Added for M14 step 3. Most cases build their own budget over a battery voltage they control.
 * The last one uses the robot's own budget, sagging the simulated roboRIO's input voltage, because
 * it is the checkpoint's combined-mode clause and has to go through the real wiring: under a
 * sagging battery the LOW tier gives up current first, and the intake stops rocking while it does.
 */
public class PowerBudgetTest {
  private static final double MIN = 0.5;

  /** The battery voltage the case's own budget reads, volts. */
  private final double[] volts = {12.0};

  private final PowerBudget budget = new PowerBudget(() -> volts[0]);

  private Intake intake;

  /** The intake's ideal pivot, which records the goal the intake last sent it. */
  private IdealPivotIO pivot;

  /** Initialises the HAL and pauses the clock. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();
  }

  /** Releases the HAL. */
  @AfterEach
  void tearDown() {
    if (intake != null) {
      CommandScheduler.getInstance().unregisterSubsystem(intake);
    }
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  /**
   * Runs the case's budget for a number of loops at a voltage.
   *
   * @param v the battery voltage, volts
   * @param loops how many loops
   *     <p><b>Outputs:</b> updates the budget.
   */
  private void run(double v, int loops) {
    volts[0] = v;
    for (int i = 0; i < loops; i++) {
      budget.periodic();
    }
  }

  /**
   * The battery voltage at a given depth into the brownout band.
   *
   * @param severity 0 at the warning voltage, 1 at the critical one
   * @return the voltage, volts
   */
  private static double voltsAtSeverity(double severity) {
    double warn = Constants.Power.kBrownoutWarningVolts.get();
    double critical = Constants.Power.kBrownoutCriticalVolts.get();
    return warn - severity * (warn - critical);
  }

  /**
   * At every severity, a lower tier has given up at least as much current as every higher one, and
   * the drivetrain none.
   *
   * <p>The plan's whole point for this class: the intake gives up its amps well before the
   * flywheel, and the drivetrain never does, so the robot finishes the match able to drive.
   */
  @Test
  void lowerTiersAlwaysGiveUpFirst() {
    for (double s = 0.0; s <= 1.0; s += 0.01) {
      double low = PowerBudget.scalarFor(Priority.LOW, s, MIN);
      double medium = PowerBudget.scalarFor(Priority.MEDIUM, s, MIN);
      double high = PowerBudget.scalarFor(Priority.HIGH, s, MIN);
      assertTrue(low <= medium && medium <= high, "tiers out of order at severity " + s);
      assertEquals(1.0, PowerBudget.scalarFor(Priority.CRITICAL, s, MIN), "the drive was cut");
      assertTrue(low >= MIN - 1e-9, "a tier went below the minimum");
    }
  }

  /**
   * Each tier starts at its own point: LOW at once, MEDIUM a third of the way, HIGH two thirds.
   *
   * <p>Ordering alone would pass for three tiers that all move together.
   */
  @Test
  void eachTierStartsAtItsOwnPoint() {
    assertTrue(PowerBudget.scalarFor(Priority.LOW, 0.2, MIN) < 1.0, "LOW did not start first");
    assertEquals(1.0, PowerBudget.scalarFor(Priority.MEDIUM, 0.2, MIN));
    assertTrue(PowerBudget.scalarFor(Priority.MEDIUM, 0.5, MIN) < 1.0);
    assertEquals(1.0, PowerBudget.scalarFor(Priority.HIGH, 0.5, MIN));
    assertTrue(PowerBudget.scalarFor(Priority.HIGH, 0.9, MIN) < 1.0);
    for (Priority tier : new Priority[] {Priority.LOW, Priority.MEDIUM, Priority.HIGH}) {
      assertEquals(1.0, PowerBudget.scalarFor(tier, 0.0, MIN), tier + " cut on a healthy battery");
      assertEquals(MIN, PowerBudget.scalarFor(tier, 1.0, MIN), 1e-9, tier + " at critical");
    }
  }

  /**
   * On a healthy battery nothing is cut, from the very first loop.
   *
   * <p>The filter starts from the first sample. A filter starting from zero would read a 0 V
   * battery for its first few hundred milliseconds and boot every mechanism throttled.
   */
  @Test
  void aHealthyBatteryCutsNothingFromTheFirstLoop() {
    run(12.0, 1);
    for (Priority tier : Priority.values()) {
      assertEquals(1.0, budget.getScalar(tier), tier + " was cut on the first loop");
    }
    assertEquals(12.0, budget.getFilteredVoltage(), 1e-9);
    assertFalse(budget.isBrownoutImminent());
  }

  /**
   * A sagging battery cuts LOW first, and a falling voltage below the warning is an imminent
   * brownout.
   */
  @Test
  void aSagCutsLowFirstAndWarns() {
    run(12.0, 5);
    double v = voltsAtSeverity(0.25);
    boolean sawImminent = false;
    for (int i = 0; i < 50; i++) {
      run(v, 1);
      sawImminent |= budget.isBrownoutImminent();
    }
    assertTrue(budget.getScalar(Priority.LOW) < 1.0, "a sag into the band did not cut LOW");
    assertEquals(1.0, budget.getScalar(Priority.MEDIUM), "MEDIUM was cut before its point");
    assertEquals(1.0, budget.getScalar(Priority.HIGH));
    assertTrue(sawImminent, "a battery falling through the warning was never called imminent");
  }

  /**
   * A battery recovering inside the band is not an imminent brownout.
   *
   * <p>Imminent means below the warning AND falling. A robot that has just stopped drawing current
   * climbs back through the band, and calling that imminent would throttle it for recovering.
   */
  @Test
  void aRecoveringBatteryIsNotImminent() {
    run(voltsAtSeverity(0.9), 100);
    boolean sawImminent = false;
    volts[0] = voltsAtSeverity(0.3);
    for (int i = 0; i < 10; i++) {
      budget.periodic();
      sawImminent |= budget.isBrownoutImminent();
    }
    assertTrue(budget.getFilteredVoltage() < Constants.Power.kBrownoutWarningVolts.get());
    assertFalse(sawImminent, "a recovering battery was called imminent");
  }

  /**
   * The combined-mode clause of the M14 checkpoint, in simulation: under a sagging battery the
   * intake stops rocking, and rocks again once the battery recovers.
   *
   * <p>Rocking is pure pivot current, so it is the first thing given up when LOW is cut. This goes
   * through the robot's own budget and the simulated roboRIO's input voltage, so the wiring from
   * battery to rock is the real one.
   */
  @Test
  void feedAssistStopsRockingWhileLowIsCut() {
    PowerBudget robotBudget = PowerBudget.getInstance();
    pivot = new IdealPivotIO();
    intake = new Intake(pivot, IdealPivotIO.GEOMETRY);
    intake.setDeployGuard(() -> true);
    try {
      RoboRioSim.setVInVoltage(12.0);
      stepIntake(robotBudget, 60);
      intake.setWantedState(Intake.WantedState.FEED_ASSIST);
      assertTrue(rockGoals(robotBudget).size() > 1, "the intake never rocked on a healthy battery");

      RoboRioSim.setVInVoltage(voltsAtSeverity(0.25));
      stepIntake(robotBudget, 50);
      assertTrue(robotBudget.getScalar(Priority.LOW) < 1.0, "the sag did not cut LOW");
      assertEquals(
          Set.of(IdealPivotIO.FEED_ASSIST), rockGoals(robotBudget), "it rocked with LOW cut");

      RoboRioSim.setVInVoltage(12.0);
      stepIntake(robotBudget, 50);
      assertTrue(rockGoals(robotBudget).size() > 1, "the rock never came back");
    } finally {
      RoboRioSim.setVInVoltage(12.0);
      for (int i = 0; i < 100; i++) {
        robotBudget.periodic();
      }
    }
  }

  /**
   * Steps the intake and the robot's budget together.
   *
   * @param robotBudget the budget to run
   * @param loops how many loops
   *     <p><b>Outputs:</b> runs both and advances the clock.
   */
  private void stepIntake(PowerBudget robotBudget, int loops) {
    for (int i = 0; i < loops; i++) {
      robotBudget.periodic();
      intake.periodic();
      SimHooks.stepTiming(0.02);
    }
  }

  /**
   * The distinct pivot goals the intake commands over two rock periods.
   *
   * @param robotBudget the budget to run alongside
   * @return the set of goals, in radians
   */
  private Set<Double> rockGoals(PowerBudget robotBudget) {
    Set<Double> goals = new HashSet<>();
    int loops = (int) Math.round(2 * Constants.Intake.kFeedAssistPeriodSec.get() / 0.02);
    for (int i = 0; i < loops; i++) {
      stepIntake(robotBudget, 1);
      goals.add(Math.round(pivot.lastGoalRad() * 1e6) / 1e6);
    }
    return goals;
  }
}
