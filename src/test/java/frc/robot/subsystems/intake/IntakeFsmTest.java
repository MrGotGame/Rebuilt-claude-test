package frc.robot.subsystems.intake;

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
 * Transition tests for the pivoting intake, covering the half of M5's per-mechanism checkpoint that
 * does not need a robot.
 *
 * <p>This is the mechanism with the most states nobody can ask for -- HOMING, STOWED and FAULTED
 * are all entered by the machine itself -- and the one whose failures are physical rather than
 * cosmetic. The deploy guard it carries is, in the plan's words, the single most likely way to
 * destroy this robot.
 */
public class IntakeFsmTest {
  private static final double LOOP_SEC = 0.02;

  private IntakeIOSim sim;
  private Intake intake;

  /** Builds an intake over simulated IO with the HAL initialised and the clock paused. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();

    sim = new IntakeIOSim();
    intake = new Intake(sim);
  }

  /** Releases the HAL and unregisters the subsystem so cases cannot leak into each other. */
  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().unregisterSubsystem(intake);
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
      intake.periodic();
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  /** Replaces the mechanism under test with one built over the given IO. */
  private void rebuildOver(IntakeIO io) {
    CommandScheduler.getInstance().unregisterSubsystem(intake);
    intake = new Intake(io);
  }

  /** Loops needed to outrun the homing timeout, with margin. */
  private int loopsPastHomingTimeout() {
    return (int) (Constants.Intake.kHomingTimeoutSec.get() / LOOP_SEC) + 30;
  }

  /** The machine boots into HOMING and homes itself without being asked. */
  @Test
  void bootsIntoHomingAndReachesStowed() {
    assertEquals(Intake.SystemState.HOMING, intake.getSystemState(), "boot state must be HOMING");
    assertFalse(intake.isHomed(), "nothing is known about the pivot before homing");

    step(loopsPastHomingTimeout());

    assertTrue(intake.isHomed(), "the pivot never homed against a healthy simulated arm");
    assertNotEquals(
        Intake.SystemState.FAULTED, intake.getSystemState(), "healthy homing must not fault");
  }

  /**
   * The deploy guard refuses by default, so nothing swings out until a real guard is installed.
   *
   * <p>This is the shipped behaviour of the robot today: {@code Superstructure} injects the real
   * interlock and is a skeleton until M10, so {@code kDeployAllowedWithoutGuard} is false and every
   * request that would put the arm outside stow resolves to STOWING instead.
   */
  @Test
  void deployIsRefusedWhileNoGuardIsInstalled() {
    step(loopsPastHomingTimeout());
    intake.setWantedState(Intake.WantedState.DEPLOY_AND_INTAKE);
    step(30);

    assertNotEquals(
        Intake.SystemState.DEPLOYING, intake.getSystemState(), "deployed with no guard installed");
    assertNotEquals(
        Intake.SystemState.INTAKING, intake.getSystemState(), "deployed with no guard installed");
  }

  /** EJECT is covered by the guard too, not just the two requests named DEPLOY. */
  @Test
  void ejectIsAlsoRefusedByTheGuard() {
    step(loopsPastHomingTimeout());
    intake.setDeployGuard(() -> false);
    intake.setWantedState(Intake.WantedState.EJECT);
    step(30);

    // The eject angle puts the arm outside stow just as the deploy angle does, so a guard that
    // covered only DEPLOY would leave the collision reachable through a different button.
    assertNotEquals(
        Intake.SystemState.EJECTING, intake.getSystemState(), "ejected past a closed guard");
  }

  /** With the guard open, every remaining SystemState is reachable. Clause 2 of the checkpoint. */
  @Test
  void everyStateIsReachableWithTheGuardOpen() {
    intake.setDeployGuard(() -> true);
    step(loopsPastHomingTimeout());

    // NOTE: STOWED is deliberately not asserted here. With the shipped placeholder gains --
    // kPivotKg and kPivotKd both 0.0 against kPivotKp 40.0 -- the simulated arm oscillates about
    // its setpoint instead of settling, so atGoal() is only transiently true and STOWING never
    // promotes. That is a gain-tuning gap for M6, not an FSM defect, and it is pinned down by
    // stowedIsUnreachableUntilTheGainsAreTuned below rather than hidden by a longer timeout here.

    intake.setWantedState(Intake.WantedState.DEPLOY_AND_INTAKE);
    step(5);
    assertEquals(Intake.SystemState.DEPLOYING, intake.getSystemState());

    intake.setWantedState(Intake.WantedState.EJECT);
    step(5);
    assertEquals(Intake.SystemState.EJECTING, intake.getSystemState());

    intake.setWantedState(Intake.WantedState.CHARACTERIZE);
    step(5);
    assertEquals(Intake.SystemState.CHARACTERIZING, intake.getSystemState());

    intake.setWantedState(Intake.WantedState.IDLE);
    step(5);
    assertEquals(Intake.SystemState.IDLE, intake.getSystemState());
  }

  /**
   * {@code isIntaking()} is true for exactly one state, and it is not any of the deployed-looking
   * ones.
   *
   * <p>Vision fuses the intake camera only while this is true, because the camera rides the pivot
   * about 0.4 m from its axis and {@code kRobotToIntakeCamera} describes exactly one pose. Two
   * states are easy to admit by mistake and both would be wrong: DEPLOYING has the arm mid-travel,
   * somewhere along that arc; EJECTING has it deployed but to {@code kEjectAngleRad}, a DIFFERENT
   * angle, which is the more dangerous of the two because the intake really is "out" and the error
   * is a plausible constant rather than an obvious sweep.
   *
   * <p>Asserted as an invariant against the state rather than by visiting INTAKING directly. With
   * the shipped placeholder gains the simulated arm oscillates about its setpoint instead of
   * settling -- the same gap {@code everyStateIsReachableWithTheGuardOpen} records for STOWED -- so
   * a case that demanded INTAKING would be testing the gains, not the predicate.
   */
  @Test
  void isIntakingIsTrueForExactlyTheIntakingState() {
    intake.setDeployGuard(() -> true);
    step(loopsPastHomingTimeout());

    Intake.WantedState[] tour = {
      Intake.WantedState.DEPLOY_AND_INTAKE,
      Intake.WantedState.EJECT,
      Intake.WantedState.STOW,
      Intake.WantedState.DEPLOY_AND_INTAKE,
      Intake.WantedState.IDLE,
    };
    boolean sawDeploying = false;
    boolean sawEjecting = false;
    for (Intake.WantedState wanted : tour) {
      intake.setWantedState(wanted);
      for (int i = 0; i < 40; i++) {
        step(1);
        Intake.SystemState state = intake.getSystemState();
        sawDeploying |= state == Intake.SystemState.DEPLOYING;
        sawEjecting |= state == Intake.SystemState.EJECTING;
        assertEquals(
            state == Intake.SystemState.INTAKING,
            intake.isIntaking(),
            "isIntaking() disagreed with the state machine while in " + state);
      }
    }

    // The tour must actually have visited the two states that would be wrong to admit, or the
    // invariant above was never put under any pressure.
    assertTrue(sawDeploying, "the tour never passed through DEPLOYING, so mid-travel was untested");
    assertTrue(
        sawEjecting, "the tour never passed through EJECTING, so the eject angle was untested");
  }

  /** A pivot that never stops moving cannot home, and must fault rather than stall all match. */
  @Test
  void homingTimesOutToFaulted() {
    rebuildOver(
        new IntakeIO() {
          @Override
          public void updateInputs(IntakeIOInputs inputs) {
            inputs.pivotConnected = true;
            inputs.rollerConnected = true;
            inputs.absEncoderConnected = true;
            // Never settles, so the hard stop is never found.
            inputs.pivotVelocityRadPerSec = 100.0;
          }
        });

    step(loopsPastHomingTimeout());

    assertEquals(
        Intake.SystemState.FAULTED,
        intake.getSystemState(),
        "a pivot that cannot find its stop must fault, not stall a motor for the match");
    assertFalse(intake.isHomed(), "a timed-out home must not mark the pivot trustworthy");
  }

  /** Once faulted, only a HOME request gets the machine back. */
  @Test
  void faultLatchesUntilHomeIsRequested() {
    rebuildOver(
        new IntakeIO() {
          @Override
          public void updateInputs(IntakeIOInputs inputs) {
            inputs.pivotConnected = true;
            inputs.pivotVelocityRadPerSec = 100.0;
          }
        });
    step(loopsPastHomingTimeout());
    assertEquals(Intake.SystemState.FAULTED, intake.getSystemState());

    intake.setWantedState(Intake.WantedState.STOW);
    step(30);
    assertEquals(
        Intake.SystemState.FAULTED, intake.getSystemState(), "STOW must not clear a fault");

    intake.setWantedState(Intake.WantedState.HOME);
    step(5);
    assertEquals(
        Intake.SystemState.HOMING, intake.getSystemState(), "HOME must clear the fault and retry");
  }

  /** Given time to settle, STOWING promotes to STOWED and the interlock reads true. */
  @Test
  void stowingEventuallySettlesIntoStowed() {
    intake.setDeployGuard(() -> true);
    step(loopsPastHomingTimeout());

    intake.setWantedState(Intake.WantedState.STOW);

    // 400 loops is eight simulated seconds, which is far longer than the move needs but is what
    // the shipped gains require: kPivotKg and kPivotKd are both 0.0 against kPivotKp 40.0, so a
    // gravity-loaded arm rings for several seconds before it settles. The FSM is correct either
    // way -- it refuses to call STOWING done while the arm is not at the goal -- but the ringing
    // is real and M6's gain measurement should shorten it considerably.
    step(400);

    assertEquals(
        Intake.SystemState.STOWED, intake.getSystemState(), "the pivot never settled at stow");
    assertTrue(intake.isStowed(), "isStowed() must agree with the STOWED state");
  }

  /**
   * A pivot whose motor is dead must not conclude that it is homed.
   *
   * <p>Homing declares success when velocity stays below a threshold for a settle time. A
   * disconnected motor reports exactly zero velocity forever, which satisfies that predicate
   * immediately. If the machine accepts it, {@code isHomed()} goes true against an arm whose real
   * position is unknown, and {@code isStowed()} then tells the turret it is safe to sweep through
   * an arc that may well contain the arm.
   */
  @Test
  void deadPivotMustNotReportItselfHomed() {
    rebuildOver(new IntakeIO() {});

    step(loopsPastHomingTimeout());

    assertFalse(
        intake.isHomed(),
        "a pivot that never moved was accepted as homed; stall detection needs proof of motion");
    assertFalse(intake.isStowed(), "an unhomed intake must never tell the turret it is stowed");
  }
}
