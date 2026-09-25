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

  // ---------------------------------------------------------------- M10b: the hopper intake ----

  // The test geometry lives with the ideal pivot in IdealPivotIO, shared with the vision gate
  // tests; these aliases keep the cases below readable.
  private static final double STOWED = IdealPivotIO.STOWED;
  private static final double DEPLOYED = IdealPivotIO.DEPLOYED;
  private static final double FEED_ASSIST = IdealPivotIO.FEED_ASSIST;
  private static final double BAND = IdealPivotIO.BAND;
  private static final Intake.PivotGeometry GEOMETRY = IdealPivotIO.GEOMETRY;

  /** The ideal pivot under {@link #intake} once {@link #rebuildIdeal()} has run. */
  private IdealPivotIO ideal;

  /**
   * Rebuilds the intake over an ideal pivot and the test geometry, homes it, and leaves it STOWED.
   *
   * <p><b>Outputs:</b> replaces {@link #intake}; sets {@link #ideal}.
   */
  private void rebuildIdeal() {
    ideal = new IdealPivotIO();
    CommandScheduler.getInstance().unregisterSubsystem(intake);
    intake = new Intake(ideal, GEOMETRY);
    step(40);
    assertTrue(intake.isHomed(), "the ideal pivot failed to home");

    // The boot request is HOME, and a completed home resolves to STOWING rather than STOWED; the
    // machine parks there until somebody asks for something, which in the robot is the
    // superstructure. Ask for the stow explicitly.
    intake.setWantedState(Intake.WantedState.STOW);
    step(5);
    assertEquals(Intake.SystemState.STOWED, intake.getSystemState());
  }

  /**
   * DEPLOY_ONLY arrives and STAYS in a real DEPLOYED state with the roller stopped.
   *
   * <p>Before M10b it resolved to DEPLOYING forever, a state that permanently claimed "in transit".
   * With the intake now resting deployed for most of a match, every consumer that needs to know the
   * arm has arrived would have been switched off.
   */
  @Test
  void deployOnlySettlesIntoDeployed() {
    rebuildIdeal();
    intake.setDeployGuard(() -> true);

    intake.setWantedState(Intake.WantedState.DEPLOY_ONLY);
    step(5);
    assertEquals(Intake.SystemState.DEPLOYED, intake.getSystemState());

    for (int i = 0; i < 50; i++) {
      step(1);
      assertEquals(Intake.SystemState.DEPLOYED, intake.getSystemState(), "left DEPLOYED at " + i);
      assertEquals(
          0.0, ideal.lastRollerRpm(), "a resting deployed intake must not spin its roller");
    }
  }

  /**
   * A change of request never reports arrival before the arm has moved.
   *
   * <p>The original transitions judged arrival against the goal the PREVIOUS loop commanded, so for
   * one loop after every change of request they reported the new state with the arm still at the
   * old one: stow to intake entered INTAKING with the arm stowed, which the intake-camera gate
   * reads, and deploy to stow entered STOWED with the arm deployed, which the M10 turret interlock
   * reads. The ideal pivot makes the first loop after a request exactly the loop in question.
   */
  @Test
  void noArrivalIsReportedBeforeTheArmMoves() {
    rebuildIdeal();
    intake.setDeployGuard(() -> true);

    intake.setWantedState(Intake.WantedState.DEPLOY_AND_INTAKE);
    step(1);
    assertEquals(
        Intake.SystemState.DEPLOYING,
        intake.getSystemState(),
        "reported INTAKING on the first loop, with the arm still stowed");
    step(5);
    assertEquals(Intake.SystemState.INTAKING, intake.getSystemState());

    intake.setWantedState(Intake.WantedState.STOW);
    step(1);
    assertEquals(
        Intake.SystemState.STOWING,
        intake.getSystemState(),
        "reported STOWED on the first loop, with the arm still deployed");
    assertFalse(intake.isStowed(), "told the turret the arm was stowed while it was deployed");
  }

  /**
   * FEED_ASSIST rocks: it rises first, alternates every half period between the two angles, and
   * runs the roller inward throughout.
   */
  @Test
  void feedAssistRocksBetweenItsTwoAngles() {
    rebuildIdeal();
    intake.setDeployGuard(() -> true);
    intake.setWantedState(Intake.WantedState.DEPLOY_ONLY);
    step(5);

    intake.setWantedState(Intake.WantedState.FEED_ASSIST);
    double periodSec = Constants.Intake.kFeedAssistPeriodSec.get();
    int loops = (int) Math.round(2.0 * periodSec / LOOP_SEC);
    int changes = 0;
    double previousGoal = Double.NaN;
    for (int i = 0; i < loops; i++) {
      step(1);
      assertEquals(Intake.SystemState.FEED_ASSISTING, intake.getSystemState());
      assertEquals(Constants.Intake.kFeedAssistRollerRpm.get(), ideal.lastRollerRpm());
      double goal = ideal.lastGoalRad();
      assertTrue(
          goal == FEED_ASSIST || goal == DEPLOYED, "rocked to an angle outside its two ends");
      if (i == 0) {
        assertEquals(FEED_ASSIST, goal, "the rock must start by rising");
      } else if (goal != previousGoal) {
        changes++;
      }
      previousGoal = goal;
    }

    // Two full periods are four half periods, so three changes of direction.
    assertEquals(3, changes, "the rock did not alternate every half period");
  }

  /**
   * FEED_ASSIST swings the arm out of stow just as DEPLOY does, so the M10 guard covers it too.
   *
   * <p>Pinned while the M10 interlock stands. M10b step 5 replaces the guard with the pivot-band
   * rule, under which the rock -- clear of the band at both ends -- no longer waits on the turret,
   * and this case is rewritten there.
   */
  @Test
  void feedAssistIsRefusedByTheGuardToo() {
    rebuildIdeal();
    intake.setDeployGuard(() -> false);

    intake.setWantedState(Intake.WantedState.FEED_ASSIST);
    step(30);

    assertNotEquals(
        Intake.SystemState.FEED_ASSISTING, intake.getSystemState(), "rocked past a closed guard");
  }

  /**
   * The conflict band follows the arm: true while unhomed and at stow, false once deployed and
   * throughout the rock, and true again the moment a stow is COMMANDED rather than when it lands.
   */
  @Test
  void conflictBandFollowsTheArmAndLeadsItIntoStow() {
    ideal = new IdealPivotIO();
    CommandScheduler.getInstance().unregisterSubsystem(intake);
    intake = new Intake(ideal, GEOMETRY);
    assertTrue(
        intake.isInTurretConflictBand(), "an unhomed arm could be anywhere; must fail closed");

    step(40);
    assertTrue(intake.isInTurretConflictBand(), "the stowed arm is in the turret's way");

    intake.setDeployGuard(() -> true);
    intake.setWantedState(Intake.WantedState.DEPLOY_ONLY);
    step(5);
    assertFalse(intake.isInTurretConflictBand(), "a deployed arm cannot reach the turret");

    intake.setWantedState(Intake.WantedState.FEED_ASSIST);
    for (int i = 0; i < 60; i++) {
      step(1);
      assertFalse(intake.isInTurretConflictBand(), "the rock entered the band at loop " + i);
    }

    // One loop: the stow goal is commanded, but the ideal pivot has not reported arriving yet.
    intake.setWantedState(Intake.WantedState.STOW);
    step(1);
    assertNotEquals(STOWED, ideal.positionRad(), "the arm should not have arrived yet");
    assertTrue(
        intake.isInTurretConflictBand(),
        "a stow on its way into the band must block the turret before the arm arrives");
  }

  /**
   * Pieces are retained anywhere from deploy up to the feed-assist angle, never at stow, and never
   * on an untrusted encoder.
   */
  @Test
  void retainingPiecesTracksTheArm() {
    ideal = new IdealPivotIO();
    CommandScheduler.getInstance().unregisterSubsystem(intake);
    intake = new Intake(ideal, GEOMETRY);
    assertFalse(intake.isRetainingPieces(), "an unhomed arm must not claim to hold pieces");

    step(40);
    assertFalse(intake.isRetainingPieces(), "pieces fall out of a stowed intake");

    intake.setDeployGuard(() -> true);
    intake.setWantedState(Intake.WantedState.DEPLOY_ONLY);
    step(5);
    assertTrue(intake.isRetainingPieces(), "a deployed intake holds its pieces");

    intake.setWantedState(Intake.WantedState.FEED_ASSIST);
    for (int i = 0; i < 60; i++) {
      step(1);
      assertTrue(intake.isRetainingPieces(), "pieces were lost mid-rock at loop " + i);
    }
  }

  /**
   * The band lies at the stow end of travel whichever way the pivot's angles run, includes anything
   * past stow, and fails closed when stow and deploy coincide.
   */
  @Test
  void conflictBandIsDirectionAgnosticAndFailsClosed() {
    double w = BAND;

    // Stow below deploy, as in the test geometry.
    assertTrue(Intake.inTurretConflictBand(-1.2, -1.2, 0.3, w), "stow itself");
    assertTrue(Intake.inTurretConflictBand(-1.2 + 0.9 * w, -1.2, 0.3, w), "inside the band");
    assertFalse(Intake.inTurretConflictBand(-1.2 + 1.1 * w, -1.2, 0.3, w), "just clear of it");
    assertTrue(Intake.inTurretConflictBand(-1.3, -1.2, 0.3, w), "past the stow stop");

    // The mirror image: stow above deploy, as the real robot may turn out to be.
    assertTrue(Intake.inTurretConflictBand(1.2, 1.2, -0.3, w), "stow itself, mirrored");
    assertTrue(Intake.inTurretConflictBand(1.2 - 0.9 * w, 1.2, -0.3, w), "inside, mirrored");
    assertFalse(Intake.inTurretConflictBand(1.2 - 1.1 * w, 1.2, -0.3, w), "clear, mirrored");
    assertTrue(Intake.inTurretConflictBand(1.3, 1.2, -0.3, w), "past stow, mirrored");

    // Placeholder geometry: no direction, so both sides of stow count.
    assertTrue(Intake.inTurretConflictBand(0.9 * w, 0.0, 0.0, w), "degenerate, above");
    assertTrue(Intake.inTurretConflictBand(-0.9 * w, 0.0, 0.0, w), "degenerate, below");
    assertFalse(Intake.inTurretConflictBand(1.1 * w, 0.0, 0.0, w), "degenerate, clear");
  }

  /**
   * The rock alternates on the half period, and holds the raised angle when its period is zero or
   * when rocking is not allowed.
   *
   * <p>The not-allowed case is the low-power fallback, and it is tested here, on the pure function,
   * because {@code PowerBudget} is still the M14 stub whose scalar is always 1.0: no
   * whole-mechanism case can reach the fallback until M14 lands.
   */
  @Test
  void feedAssistGoalAlternatesAndFallsBackToHolding() {
    double period = 0.5;
    assertEquals(FEED_ASSIST, Intake.feedAssistGoalRad(0.0, period, true, DEPLOYED, FEED_ASSIST));
    assertEquals(FEED_ASSIST, Intake.feedAssistGoalRad(0.24, period, true, DEPLOYED, FEED_ASSIST));
    assertEquals(DEPLOYED, Intake.feedAssistGoalRad(0.26, period, true, DEPLOYED, FEED_ASSIST));
    assertEquals(FEED_ASSIST, Intake.feedAssistGoalRad(0.51, period, true, DEPLOYED, FEED_ASSIST));

    assertEquals(
        FEED_ASSIST,
        Intake.feedAssistGoalRad(0.26, 0.0, true, DEPLOYED, FEED_ASSIST),
        "a zero period must hold the raised angle");
    assertEquals(
        FEED_ASSIST,
        Intake.feedAssistGoalRad(0.26, period, false, DEPLOYED, FEED_ASSIST),
        "with rocking disallowed, it must hold the raised angle");
  }

  /**
   * The intake-camera gate admits exactly the two states that hold the deploy angle -- INTAKING and
   * the resting DEPLOYED -- and nothing that merely passes near it.
   *
   * <p>Asserted as an invariant over a tour of every state the ideal pivot can reach, on the
   * pattern of {@code isIntakingIsTrueForExactlyTheIntakingState}. The tour must also be shown to
   * have visited each trap, or the invariant was never put under pressure. The subtlest is the
   * rock: FEED_ASSISTING sits exactly at the deploy angle for half of every period, so a gate that
   * tested position instead of state would admit it there -- with the arm about to swing away
   * inside a camera frame's latency.
   */
  @Test
  void atIntakingAngleIsTrueForExactlyTheRestingAndIntakingStates() {
    rebuildIdeal();
    intake.setDeployGuard(() -> true);

    Intake.WantedState[] tour = {
      Intake.WantedState.DEPLOY_ONLY,
      Intake.WantedState.DEPLOY_AND_INTAKE,
      Intake.WantedState.FEED_ASSIST,
      Intake.WantedState.EJECT,
      Intake.WantedState.DEPLOY_ONLY,
      Intake.WantedState.STOW,
      Intake.WantedState.IDLE,
    };
    java.util.EnumSet<Intake.SystemState> seen = java.util.EnumSet.noneOf(Intake.SystemState.class);
    boolean sawRockAtTheDeployAngle = false;
    for (Intake.WantedState wanted : tour) {
      intake.setWantedState(wanted);
      for (int i = 0; i < 40; i++) {
        step(1);
        Intake.SystemState state = intake.getSystemState();
        seen.add(state);
        sawRockAtTheDeployAngle |=
            state == Intake.SystemState.FEED_ASSISTING && ideal.positionRad() == DEPLOYED;
        assertEquals(
            state == Intake.SystemState.INTAKING || state == Intake.SystemState.DEPLOYED,
            intake.isAtIntakingAngle(),
            "the camera gate disagreed with the state machine while in " + state);
      }
    }

    for (Intake.SystemState trap :
        new Intake.SystemState[] {
          Intake.SystemState.DEPLOYING,
          Intake.SystemState.EJECTING,
          Intake.SystemState.FEED_ASSISTING,
          Intake.SystemState.STOWING,
          Intake.SystemState.STOWED,
          Intake.SystemState.DEPLOYED,
          Intake.SystemState.INTAKING
        }) {
      assertTrue(seen.contains(trap), "the tour never visited " + trap + ", so it went untested");
    }
    assertTrue(
        sawRockAtTheDeployAngle,
        "the rock never reached the deploy angle, so the position trap went untested");
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
   * <p>Until M10b step 4, Vision fused the intake camera only while this was true; it now reads
   * {@code isAtIntakingAngle()}, which also admits the resting DEPLOYED state and has its own
   * invariant case. This one still holds, and its reasoning carries straight over: the camera rides
   * the pivot about 0.4 m from its axis and {@code kRobotToIntakeCamera} describes one pose. Two
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
