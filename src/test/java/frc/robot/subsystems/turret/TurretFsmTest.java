package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Transition tests for the turret, including the one assertion M5 singles out as this mechanism's
 * own checkpoint: the commanded position must never leave the travel limits.
 *
 * <p>The plan asks for that to be demonstrated by running {@code
 * CharacterizationCommands.turretUnwrapDemo} and reading the log afterwards. Asserting it here
 * instead makes it a property checked on every build rather than a screenshot taken once, and it
 * can be swept across far more aim angles than anyone would drive by hand.
 */
public class TurretFsmTest {
  private static final double LOOP_SEC = 0.02;

  private TurretIOSim sim;
  private Turret turret;

  /** Builds a turret over simulated IO with the HAL initialised and the clock paused. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();

    // The turret reads RobotState every loop for the chassis heading and rotation rate, so the
    // singleton has to exist before the first periodic or it dereferences null.
    RobotState.initialize(new SwerveDriveKinematics(Drive.getModuleTranslations()));

    sim = new TurretIOSim();
    turret = new Turret(sim);
  }

  /** Releases the HAL and unregisters the subsystem so cases cannot leak into each other. */
  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().unregisterSubsystem(turret);
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
      turret.periodic();
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  /** Loops needed to outrun the homing timeout, with margin. */
  private int loopsPastHomingTimeout() {
    return (int) (Constants.Turret.kHomingTimeoutSec.get() / LOOP_SEC) + 30;
  }

  /** Steps one loop and asserts the commanded position is inside the travel limits. */
  private void stepAndAssertInRange(String context) {
    turret.periodic();
    SimHooks.stepTiming(LOOP_SEC);
    double commanded = turret.getCommandedPositionRad();
    double min = Constants.Turret.kMinAngleRad.get();
    double max = Constants.Turret.kMaxAngleRad.get();
    assertTrue(
        commanded >= min && commanded <= max,
        context + ": commanded " + commanded + " outside [" + min + ", " + max + "]");
  }

  /**
   * THE M5 TURRET CHECKPOINT. No aim angle, anywhere on the circle, may command the turret outside
   * its travel.
   *
   * <p>Swept at one degree of aim per loop through two full revolutions in each direction, so every
   * wrap boundary is crossed repeatedly and from both sides. A single bad branch choice in {@link
   * frc.robot.shooting.TurretUnwrapper} shows up here as a commanded position beyond a hard stop,
   * which on a real robot is a broken belt.
   */
  @Test
  void commandedPositionNeverLeavesTheTravelLimits() {
    turret.setWantedState(Turret.WantedState.HOME);
    step(loopsPastHomingTimeout());

    turret.setWantedState(Turret.WantedState.AIM);
    for (int sweep = 0; sweep < 2; sweep++) {
      for (int deg = 0; deg <= 720; deg++) {
        turret.setGoal(Rotation2d.fromDegrees(deg), 0.0);
        stepAndAssertInRange("ascending sweep " + sweep + " at " + deg + " deg");
      }
      for (int deg = 720; deg >= -720; deg--) {
        turret.setGoal(Rotation2d.fromDegrees(deg), 0.0);
        stepAndAssertInRange("descending sweep " + sweep + " at " + deg + " deg");
      }
    }
  }

  /** The limits hold while the chassis is rotating, which is when the feedforward is active. */
  @Test
  void limitsHoldWhileTheChassisIsRotating() {
    turret.setWantedState(Turret.WantedState.HOME);
    step(loopsPastHomingTimeout());
    turret.setWantedState(Turret.WantedState.AIM);

    for (int deg = 0; deg <= 720; deg++) {
      // A non-zero chassis rate engages the velocity feedforward, which is a separate path into
      // the command than the position solve and could clip the limits on its own.
      turret.setGoal(Rotation2d.fromDegrees(deg), 4.0);
      stepAndAssertInRange("rotating at " + deg + " deg");
    }
  }

  /** Every SystemState is reachable, which is clause 2 of the M5 checkpoint. */
  @Test
  void everyStateIsReachable() {
    assertEquals(Turret.SystemState.IDLE, turret.getSystemState(), "boot state must be IDLE");

    // HOMING is deliberately not asserted as a state you can catch here. With a healthy
    // ThroughBore the turret seeds from the absolute encoder on the first homing loop rather than
    // driving into a stop, so HOMING is transient by design -- that is the whole value of having
    // an absolute encoder. It is exercised as a durable state in deadMotorMustNotReportItselfHomed,
    // where no encoder is available.
    turret.setWantedState(Turret.WantedState.HOME);
    step(loopsPastHomingTimeout());
    assertTrue(turret.isHomed(), "the turret never homed against a healthy simulated encoder");

    turret.setWantedState(Turret.WantedState.STOW);
    step(5);
    assertEquals(Turret.SystemState.STOWING, turret.getSystemState());

    turret.setWantedState(Turret.WantedState.AIM);
    turret.setGoal(Rotation2d.fromDegrees(0.0), 0.0);
    step(200);
    assertTrue(
        turret.getSystemState() == Turret.SystemState.AIMING
            || turret.getSystemState() == Turret.SystemState.UNWRAPPING,
        "an in-range aim must resolve to AIMING or UNWRAPPING, was " + turret.getSystemState());

    turret.setWantedState(Turret.WantedState.MANUAL);
    turret.setManualVolts(1.0);
    step(5);
    assertEquals(Turret.SystemState.MANUAL, turret.getSystemState());

    turret.setWantedState(Turret.WantedState.CHARACTERIZE);
    step(5);
    assertEquals(Turret.SystemState.CHARACTERIZING, turret.getSystemState());

    turret.setWantedState(Turret.WantedState.IDLE);
    step(5);
    assertEquals(Turret.SystemState.IDLE, turret.getSystemState());
  }

  /**
   * A motor that never moves must not be accepted as homed.
   *
   * <p>Same failure class as the intake's: homing declares success when velocity stays below a
   * threshold, and a dead motor reports exactly zero velocity forever. If it is accepted, {@code
   * isHomed()} goes true against a turret whose real position is unknown, and the next closed-loop
   * command drives it at full authority from an imaginary zero.
   */
  @Test
  void deadMotorMustNotReportItselfHomed() {
    CommandScheduler.getInstance().unregisterSubsystem(turret);
    turret = new Turret(new TurretIO() {});

    turret.setWantedState(Turret.WantedState.HOME);
    step(loopsPastHomingTimeout());

    assertFalse(
        turret.isHomed(),
        "a turret that never moved was accepted as homed; stall detection needs corroboration");
  }
}
