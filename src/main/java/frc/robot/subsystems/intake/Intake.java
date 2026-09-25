package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PowerBudget;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The pivoting intake's state machine, written in the 2910 idiom of a requested state plus a
 * current state.
 *
 * <p>Two things make this more than a "go to angle" wrapper.
 *
 * <p>First, a real homing sequence: drive the pivot open loop into its hard stop until velocity
 * stays below a threshold for a settle time, then seed position. This is the fallback when the
 * ThroughBore is unplugged, and it has a timeout so that a jammed intake enters {@link
 * SystemState#FAULTED} instead of stalling a motor for the rest of the match.
 *
 * <p>Second, its half of the intake/turret interlock: the intake refuses to move INTO the
 * turret-conflict band -- the few degrees of travel nearest stow, where the arm lies in the
 * turret's sweep -- while the turret is inside its conflict arc. That interaction is the single
 * most likely way to destroy this robot. (Until M10b step 5 this refused every motion OUT of stow
 * instead, because it assumed the deployed intake was the conflict; on this robot the stowed one
 * is.)
 *
 * <p>M10b (plan section 2.4) made the robot a hopper robot, which adds two resting behaviours: the
 * intake rests DEPLOYED once it has been down, because raising it lets pieces fall out, and while
 * scoring it rocks between deployed and a partial raise ({@link SystemState#FEED_ASSISTING}) to
 * push the hopper into the indexer. It also publishes the two geometric questions the rest of the
 * robot needs: whether the arm is in the turret's way ({@link #isInTurretConflictBand()}), and
 * whether it is low enough to keep pieces aboard ({@link #isRetainingPieces()}).
 */
public class Intake extends SubsystemBase {
  /** What the operator or superstructure has asked the intake to do. */
  public enum WantedState {
    IDLE,
    STOW,
    DEPLOY_AND_INTAKE,
    /** Deployed with the roller stopped: the resting posture while carrying pieces. */
    DEPLOY_ONLY,
    /** Rock between deployed and the partial raise, roller inward, to feed the hopper. */
    FEED_ASSIST,
    EJECT,
    HOME,
    CHARACTERIZE
  }

  /** What the intake is actually doing right now. */
  public enum SystemState {
    IDLE,
    STOWING,
    STOWED,
    DEPLOYING,
    /** At the deploy angle with the roller stopped. Added for M10b. */
    DEPLOYED,
    INTAKING,
    /** Rocking between the deploy and feed-assist angles. Added for M10b. */
    FEED_ASSISTING,
    /**
     * Holding clear of the turret-conflict band because the turret is in its arc, instead of
     * stowing into it. Added for M10b step 5.
     */
    WAITING_FOR_TURRET,
    EJECTING,
    HOMING,
    CHARACTERIZING,
    FAULTED
  }

  /**
   * The pivot angles this machine commands, and the width of the turret-conflict band.
   *
   * <p>Added for M10b, on the precedent M8 set for {@code Vision}'s camera mountings: the geometry
   * is a constructor argument that defaults to the constants, so tests can supply a real one. Every
   * angle constant is still a 0.0 placeholder, which puts stow, deploy, eject and feed-assist at
   * the same point -- geometry in which the band has no direction, the rock has no amplitude, and
   * every assertion about either passes for free. Suppliers rather than numbers, so the default
   * geometry still follows the tunables live.
   *
   * @param stowedRad the stow angle, at the end of travel where the band lies
   * @param deployedRad the deploy angle, where pieces are collected and held
   * @param ejectRad the angle the arm moves to while ejecting
   * @param feedAssistRad the partial-raise end of the feed-assist rock
   * @param turretConflictBandRad width of the band, measured from stow toward deploy
   */
  public record PivotGeometry(
      DoubleSupplier stowedRad,
      DoubleSupplier deployedRad,
      DoubleSupplier ejectRad,
      DoubleSupplier feedAssistRad,
      DoubleSupplier turretConflictBandRad) {
    /**
     * The robot's geometry, read live from the tunables in {@code Constants.Intake}.
     *
     * @return geometry backed by the constants
     */
    public static PivotGeometry fromConstants() {
      return new PivotGeometry(
          Constants.Intake.kStowedAngleRad,
          Constants.Intake.kDeployedAngleRad,
          Constants.Intake.kEjectAngleRad,
          Constants.Intake.kFeedAssistAngleRad,
          Constants.Intake.kTurretConflictBandRad);
    }
  }

  private final IntakeIO io;
  private final PivotGeometry geometry;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();
  private final Timer homingSettleTimer = new Timer();

  /**
   * Device health alerts, one per device.
   *
   * <p>These are plain WPILib {@link Alert}s rather than {@code FaultReporter} registrations,
   * following the precedent set in {@code PhoenixUtil.tryUntilOk} and {@code Shooter}: {@code
   * FaultReporter} is still a skeleton whose {@code getInstance()} returns null, so registering
   * here would throw in this constructor. Swap all three for {@code
   * FaultReporter.getInstance().registerDeviceDisconnect(...)} once that class is implemented; the
   * alert text is already written to match what it would produce.
   *
   * <p>They are separate on purpose. A dead pivot motor stops the intake moving, which is obvious.
   * A dead absolute encoder is not obvious at all: the mechanism still works, because homing falls
   * back to the hard stop, and the only symptom is that the pivot re-homes on every boot instead of
   * knowing where it already is.
   */
  private final Alert pivotDisconnectedAlert =
      new Alert("Intake pivot motor disconnected", AlertType.kError);

  private final Alert rollerDisconnectedAlert =
      new Alert("Intake roller motor disconnected", AlertType.kError);

  private final Alert absEncoderDisconnectedAlert =
      new Alert("Intake pivot absolute encoder disconnected", AlertType.kError);

  /** Fires while the machine has given up on the pivot. See {@link SystemState#FAULTED}. */
  private final Alert faultedAlert =
      new Alert(
          "Intake failed to home within the timeout. Check for a jam, then request HOME again to"
              + " clear.",
          AlertType.kWarning);

  /**
   * Fires while nobody has installed the real interlock.
   *
   * <p>A warning rather than an error: on the robot the superstructure always installs one, so this
   * fires only on a bench. It exists so that "the intake will not stow" reads as a deliberate
   * refusal on the dashboard rather than as a broken mechanism.
   */
  private final Alert noDeployGuardAlert =
      new Alert(
          "No intake/turret guard installed; the intake will refuse to stow until Superstructure"
              + " provides one.",
          AlertType.kWarning);

  private WantedState wantedState = WantedState.HOME;
  private SystemState systemState = SystemState.HOMING;
  private boolean isHomed = false;
  private double characterizationVolts = 0.0;

  /**
   * The answer to "is the turret clear of its arc", consulted only for a motion into the
   * turret-conflict band -- in practice a stow.
   *
   * <p>Deliberately seeded from {@code Constants.Intake.kDeployAllowedWithoutGuard}, which is
   * false, rather than from {@code () -> true}. With nothing tracking where the turret is, the
   * honest answer is "unknown", and unknown must not mean yes for the one interaction the plan
   * calls the most likely way to destroy this robot.
   */
  private BooleanSupplier deployAllowed = () -> Constants.Intake.kDeployAllowedWithoutGuard;

  /** Whether {@link #setDeployGuard(BooleanSupplier)} has ever been called. Drives the alert. */
  private boolean deployGuardInstalled = false;

  /** The pivot angle currently being commanded, so {@link #atGoal()} has something to compare. */
  private double pivotGoalRad = 0.0;

  /** Set by {@link #homing()} when the hard stop has been found, cleared on entry to HOMING. */
  private boolean homingSucceeded = false;

  /**
   * Whether a homing attempt has been asked for and not yet started.
   *
   * <p>This exists because {@code wantedState} cannot distinguish "the operator just pressed HOME"
   * from "HOME is the state we booted in", and HOME is the boot state. Latching FAULTED on {@code
   * wantedState != HOME} therefore never engaged at startup: a pivot that could not home cycled
   * HOMING to FAULTED and straight back every {@code kHomingTimeoutSec}, forever, stalling the
   * motor on every pass -- exactly the failure the timeout exists to prevent.
   *
   * <p>Set by {@link #setWantedState(WantedState)} on a HOME request, cleared on entry to HOMING.
   * Starts true so the robot still homes itself once at boot.
   */
  private boolean homeRequested = true;

  /** Rocking may run only while LOW-priority current is not being cut. Refreshed every loop. */
  private double lowPriorityCurrentScalar = 1.0;

  /**
   * Where the pivot was when a motion into the band was refused, held for as long as the refusal
   * lasts. Latched on entry to {@link SystemState#WAITING_FOR_TURRET} rather than read live, so the
   * hold does not creep with the arm under gravity.
   */
  private double waitHoldRad = 0.0;

  /**
   * Builds the intake around a hardware implementation, with the robot's own geometry.
   *
   * @param io the hardware layer: real, simulated, or an empty implementation for log replay
   */
  public Intake(IntakeIO io) {
    this(io, PivotGeometry.fromConstants());
  }

  /**
   * Builds the intake around a hardware implementation and an explicit geometry.
   *
   * @param io the hardware layer: real, simulated, or an empty implementation for log replay
   * @param geometry the pivot angles and conflict band; tests pass a real one because the constants
   *     are still placeholders
   */
  public Intake(IntakeIO io, PivotGeometry geometry) {
    this.geometry = geometry;
    // IMPLEMENTATION (was TODO: "Start stateTimer, and register FaultReporter checks for
    // pivotConnected, rollerConnected and absEncoderConnected so a dead device is visible on the
    // dashboard rather than presenting as 'the intake just does not work'.")
    //
    // All three checks exist, as the Alert fields above, driven from periodic() rather than
    // registered, because FaultReporter is not implemented yet. See the field documentation.
    this.io = io;
    stateTimer.start();

    // Entry actions for the state the machine BOOTS into. periodic() only fires onEnter on a
    // transition, and systemState is initialised to HOMING at field level, so without this call
    // the boot home runs with none of its entry work done: soft limits stay enabled, which is a
    // software refusal to reach the very hard stop homing is driving towards, and homeRequested is
    // never cleared, so FAULTED cannot latch and a failing home cycles HOMING to FAULTED and back
    // forever. Any FSM whose initial state has entry actions needs this; it is not specific to the
    // intake.
    onEnter(systemState);
  }

  /**
   * Installs the intake's half of the interlock: what stops the arm moving into the turret's way.
   *
   * <p>The name is kept from M10, where this guarded deploying. Since M10b step 5 it is consulted
   * only for a motion INTO the turret-conflict band, which in practice means a stow; see {@code
   * guardedMove}.
   *
   * @param deployAllowed supplier returning false while the turret is inside, or sweeping through,
   *     its conflict arc; stored, not evaluated, at call time
   *     <p><b>Outputs:</b> replaces the stored guard.
   */
  public void setDeployGuard(BooleanSupplier deployAllowed) {
    // IMPLEMENTATION (was TODO: "this.deployAllowed = deployAllowed. The Superstructure injects
    // this.")
    //
    // The supplier is stored and evaluated fresh every loop, never cached as a boolean. The turret
    // moves while the intake is deciding, so a value sampled at injection time would be answering
    // a question about where the turret used to be.
    this.deployAllowed = deployAllowed;
    this.deployGuardInstalled = true;
  }

  /**
   * Runs one iteration of the state machine. Called by the command scheduler every 20 ms.
   *
   * <p><b>Outputs:</b> refreshes and logs inputs; may change {@code systemState}; commands the
   * hardware; pushes the power budget scalar down to the IO layer.
   */
  @Override
  public void periodic() {
    // IMPLEMENTATION (was TODO: "In order: io.updateInputs(inputs); Logger.processInputs('Intake',
    // inputs); apply tunable gains through LoggedTunableGains.ifChanged; then run the transition
    // sequence -- compute handleStateTransition(), and if it differs from systemState call
    // onExit(old), onEnter(new) and reset stateTimer; then applyState(); then log the active state
    // as an enum. Finally push PowerBudget.getInstance().getScalar(Priority.LOW) into
    // io.setCurrentLimitScalar.")
    //
    // DEVIATION: the TODO names LoggedTunableGains, but Constants.Intake exposes the pivot gains as
    // separate LoggedTunableNumbers rather than a LoggedTunableGains object, so the equivalent
    // LoggedTunableNumber.ifChanged overload is used instead. Same semantics, same change
    // detection; it simply matches the shape the constants are actually declared in.
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Inputs", inputs);

    // Gains before the state machine, so a gain edited on the dashboard takes effect on the loop
    // the operator sees the response change. hasChanged() reports true on its first call for a
    // given id, so the compiled-in gains are pushed once on the first loop after boot -- there is
    // deliberately no separate startup path that could drift out of sync with the retune path.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> {
          var gains = new Slot0Configs();
          gains.kP = values[0];
          gains.kD = values[1];
          gains.kS = values[2];
          gains.kV = values[3];
          gains.kG = values[4];
          io.setPivotGains(gains);
        },
        Constants.Intake.kPivotKp,
        Constants.Intake.kPivotKd,
        Constants.Intake.kPivotKs,
        Constants.Intake.kPivotKv,
        Constants.Intake.kPivotKg);

    // LOW priority: the intake is among the first things throttled when the battery sags, because
    // a robot that cannot pick a piece off the floor can still drive and still shoot what it
    // holds. The IO layer deadbands this, so calling it every loop costs nothing on the bus.
    lowPriorityCurrentScalar = PowerBudget.getInstance().getScalar(PowerBudget.Priority.LOW);
    io.setCurrentLimitScalar(lowPriorityCurrentScalar);

    SystemState nextState = handleStateTransition();
    if (nextState != systemState) {
      onExit(systemState);
      stateTimer.restart();
      systemState = nextState;
      onEnter(systemState);
    }

    applyState();

    Logger.recordOutput("Intake/WantedState", wantedState.toString());
    Logger.recordOutput("Intake/SystemState", systemState.toString());
    Logger.recordOutput("Intake/StateTime", stateTimer.get());
    Logger.recordOutput("Intake/PivotGoalRad", pivotGoalRad);
    Logger.recordOutput("Intake/PivotErrorRad", pivotGoalRad - inputs.pivotPositionRad);

    // Logged separately from the state, because "the intake is STOWING" and "the intake is stowing
    // because it is not allowed out" look identical on a state timeline and mean very different
    // things during a match.
    Logger.recordOutput("Intake/DeployAllowed", deployAllowed.getAsBoolean());

    pivotDisconnectedAlert.set(!inputs.pivotConnected);
    rollerDisconnectedAlert.set(!inputs.rollerConnected);
    absEncoderDisconnectedAlert.set(!inputs.absEncoderConnected);
    faultedAlert.set(systemState == SystemState.FAULTED);
    noDeployGuardAlert.set(!deployGuardInstalled);
  }

  /**
   * Decides which state the machine should be in this loop.
   *
   * @return the state to run this iteration; may be the current state
   */
  private SystemState handleStateTransition() {
    // IMPLEMENTATION (was TODO: "switch (wantedState): HOME -> HOMING, or FAULTED once stateTimer
    // exceeds kHomingTimeoutSec; anything else while !isHomed -> HOMING (never move closed loop
    // off an unknown position); STOW -> STOWING, then STOWED once atGoal(); DEPLOY_AND_INTAKE ->
    // DEPLOYING until atGoal(), then INTAKING, but stay STOWED entirely if
    // !deployAllowed.getAsBoolean(); DEPLOY_ONLY -> DEPLOYING; EJECT -> EJECTING; CHARACTERIZE ->
    // CHARACTERIZING; IDLE -> IDLE.")

    // A fault latches until somebody asks to re-home, which is the only request that can plausibly
    // fix it. Without the latch the machine would leave FAULTED the instant the pivot happened to
    // drift inside tolerance, and the alert would flicker instead of telling the pit crew there is
    // something jammed. Same rule as Shooter's fault.
    if (systemState == SystemState.FAULTED && !homeRequested) {
      return SystemState.FAULTED;
    }

    // CHARACTERIZE is checked before the homing gate on purpose. Characterization drives the pivot
    // open loop, which is exactly what you do to a mechanism whose position you do not trust, and
    // requiring a successful home first would make an un-homeable pivot impossible to characterize
    // -- which is the situation you are most likely to be in when you reach for the routine.
    if (wantedState == WantedState.CHARACTERIZE) {
      return SystemState.CHARACTERIZING;
    }

    if (wantedState == WantedState.HOME) {
      // Consume the request HERE as well as in onEnter(HOMING). A HOME that arrives while the
      // machine is ALREADY in HOMING produces no transition, so onEnter never fires and the flag
      // lingers for the rest of the match -- which permanently disables the FAULTED latch above,
      // because that latch is written as "faulted AND nobody is asking to re-home". A jammed pivot
      // then cycles HOMING -> FAULTED -> HOMING forever, stalling the motor on every pass. That is
      // precisely the failure this flag was introduced to prevent, arriving from the other side.
      //
      // Found in M10: Superstructure.homing() issues one HOME to each mechanism on entry, and the
      // intake is already homing itself at boot, so the very first loop of the very first match
      // left the flag set. The turret does not have the bug because it consumes its own request at
      // the top of handleStateTransition regardless of which state it is in.
      homeRequested = false;

      if (homingSucceeded) {
        return SystemState.STOWING;
      }
      // Measured from entry into HOMING, because stateTimer restarts on every transition. A jam
      // must not stall the motor for the rest of the match.
      if (systemState == SystemState.HOMING
          && stateTimer.hasElapsed(Constants.Intake.kHomingTimeoutSec.get())) {
        return SystemState.FAULTED;
      }
      return SystemState.HOMING;
    }

    // Never move closed loop off a position we do not trust. Every remaining request commands a
    // pivot angle, and an angle command against an unseeded encoder drives the arm somewhere
    // arbitrary at full authority -- into a hard stop, or through the turret.
    if (!isHomed) {
      return SystemState.HOMING;
    }

    // Every "have we arrived" below is judged against the TARGET of the state being requested, not
    // against pivotGoalRad. pivotGoalRad is whatever the PREVIOUS loop's handler commanded, so the
    // original atGoal()-based arms reported arrival for one loop after every change of request:
    // asked to deploy from stow, a pivot sitting at its stow goal read "at goal" and the machine
    // entered INTAKING for a loop with the arm still stowed, and asked to stow from deploy it
    // entered STOWED for a loop with the arm still deployed. Found in M10b, where the vision gate
    // (INTAKING) and the pieces-retained question both depend on those states meaning what they
    // say.
    double stowed = geometry.stowedRad().getAsDouble();
    double deployed = geometry.deployedRad().getAsDouble();

    // THE INTAKE HALF OF THE INTERLOCK (M10b step 5, plan section 2.4). Every request that moves
    // the arm is passed through guardedMove(), which refuses only a motion INTO the
    // turret-conflict band -- in practice a stow -- while the guard is shut. M10's rule was the
    // reverse: it refused every motion OUT of stow, because it assumed the deployed intake was the
    // thing in the turret's way. On this robot the stowed intake is.
    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case STOW -> guardedMove(stowed, atAngle(stowed) ? SystemState.STOWED : SystemState.STOWING);
      case DEPLOY_AND_INTAKE -> guardedMove(
          deployed, atAngle(deployed) ? SystemState.INTAKING : SystemState.DEPLOYING);

        // Settles into a real DEPLOYED state rather than lingering in DEPLOYING forever, which is
        // what it did before M10b. With the intake now resting deployed for most of a match, a
        // state that permanently claimed "in transit" would switch off every consumer that needs to
        // know the arm has arrived.
      case DEPLOY_ONLY -> guardedMove(
          deployed, atAngle(deployed) ? SystemState.DEPLOYED : SystemState.DEPLOYING);

        // No arrival test: the rock is always moving, so there is no angle to have arrived at. It
        // is guarded against its raised end, the one nearer the band.
      case FEED_ASSIST -> guardedMove(
          geometry.feedAssistRad().getAsDouble(), SystemState.FEED_ASSISTING);
      case EJECT -> guardedMove(geometry.ejectRad().getAsDouble(), SystemState.EJECTING);

        // Unreachable: both are handled above, before the homing gate. Listed so that adding a
        // WantedState later is a compile error here rather than a silent fall-through.
      case HOME, CHARACTERIZE -> systemState;
    };
  }

  /**
   * Lets a pivot motion proceed unless it would carry the arm into the turret's way while the
   * turret is there.
   *
   * <p>Refused only when all three hold: the guard is shut (the turret is in, or sweeping through,
   * its conflict arc), the arm is currently CLEAR of the band, and the target is IN it. The middle
   * condition is what makes leaving the band always allowed. The band lies at the stow end of
   * travel, so a motion that starts inside it can only be heading out, away from the turret -- and
   * the turret half of the interlock already keeps the turret out of its arc while the arm is in
   * the band, so there is nothing for such a motion to meet. Refusing it would only strand the arm
   * where it is most in the way.
   *
   * <p>A refused motion does not fall back to stowing, which is what M10's guard did: on this robot
   * stowing is the dangerous direction. It waits instead, holding where it was refused, and
   * proceeds on the first loop the guard opens.
   *
   * @param targetRad the angle the requested state drives the pivot to, in radians
   * @param whenAllowed the state to enter if the motion may proceed
   * @return {@code whenAllowed}, or {@link SystemState#WAITING_FOR_TURRET}
   */
  private SystemState guardedMove(double targetRad, SystemState whenAllowed) {
    double stowed = geometry.stowedRad().getAsDouble();
    double deployed = geometry.deployedRad().getAsDouble();
    double width = geometry.turretConflictBandRad().getAsDouble();
    boolean entersBand =
        !inTurretConflictBand(inputs.pivotPositionRad, stowed, deployed, width)
            && inTurretConflictBand(targetRad, stowed, deployed, width);
    if (entersBand && !deployAllowed.getAsBoolean()) {
      return SystemState.WAITING_FOR_TURRET;
    }
    return whenAllowed;
  }

  /**
   * Runs one-shot work when a state is entered.
   *
   * @param state the state being entered
   *     <p><b>Outputs:</b> resets homing bookkeeping or characterization voltage as appropriate.
   */
  private void onEnter(SystemState state) {
    // IMPLEMENTATION (was TODO: "HOMING -> isHomed = false and homingSettleTimer.reset().
    // CHARACTERIZING -> characterizationVolts = 0.")
    switch (state) {
      case HOMING -> {
        isHomed = false;
        homingSucceeded = false;
        homeRequested = false;
        homingSettleTimer.reset();
        homingSettleTimer.stop();

        // Beyond the TODO, and load-bearing: homing deliberately drives the pivot INTO its hard
        // stop, and a soft limit is a software refusal to go there. Leaving the limits enabled
        // would stop the arm short of the stop, velocity would fall to zero anyway, and the
        // mechanism would cheerfully declare itself homed at the wrong angle -- every subsequent
        // position command then off by that error, with nothing reporting a fault.
        io.setPivotSoftLimitsEnabled(false);
      }
      case WAITING_FOR_TURRET -> waitHoldRad = inputs.pivotPositionRad;
      case CHARACTERIZING -> {
        characterizationVolts = 0.0;

        // Same reasoning as homing: a characterization sweep is open loop and is expected to reach
        // the ends of travel.
        io.setPivotSoftLimitsEnabled(false);
      }
      default -> {}
    }
  }

  /**
   * Runs one-shot work when a state is left.
   *
   * @param state the state being left
   *     <p><b>Outputs:</b> on a successful homing exit, seeds the pivot position and marks homed.
   */
  private void onExit(SystemState state) {
    // IMPLEMENTATION (was TODO: "HOMING -> io.seedPivotPosition(Constants.Intake
    // .kStowedAngleRad.get()) and isHomed = true, but ONLY when the exit was a success rather than
    // a timeout into FAULTED.")
    switch (state) {
      case HOMING -> {
        // The homingSucceeded guard is the whole point. Leaving HOMING because the timeout expired
        // means the arm is somewhere unknown, and seeding the encoder there would write a
        // confidently wrong zero -- worse than no zero at all, because isHomed would then be true
        // and every closed-loop command would be trusted.
        if (homingSucceeded) {
          io.seedPivotPosition(geometry.stowedRad().getAsDouble());
          isHomed = true;
        }
        io.setPivotSoftLimitsEnabled(true);
      }
      case CHARACTERIZING -> io.setPivotSoftLimitsEnabled(true);
      default -> {}
    }
  }

  /**
   * Dispatches to the handler for the current state.
   *
   * <p><b>Outputs:</b> commands the hardware for this loop.
   */
  private void applyState() {
    // IMPLEMENTATION (was TODO: "switch (systemState) calling the private handlers below.")
    switch (systemState) {
      case IDLE -> idling();
      case STOWING, STOWED -> stowing();
      case DEPLOYING -> deploying();
      case DEPLOYED -> deployed();
      case INTAKING -> intaking();
      case FEED_ASSISTING -> feedAssisting();
      case WAITING_FOR_TURRET -> waitingForTurret();
      case EJECTING -> ejecting();
      case HOMING -> homing();
      case CHARACTERIZING -> characterizing();
      case FAULTED -> faulted();
    }
  }

  /**
   * Holds position with the roller stopped.
   *
   * <p><b>Outputs:</b> commands the pivot to its existing goal and stops the roller.
   */
  private void idling() {
    // NAMED idling(), NOT idle(). WPILib 2026's Subsystem interface declares a public default
    // idle() returning a Command, so a private idle() here is an attempt to weaken the access of
    // an inherited method and does not compile. The gerund also matches every sibling handler --
    // stowing, deploying, intaking, ejecting, homing, characterizing -- so the collision-avoiding
    // name is the consistent one anyway. Do not "tidy" it back.
    //
    // IDLE holds the last commanded angle rather than going neutral. A pivot with a gravity load
    // and no command falls, and the arm falling is how it arrives in the turret's arc unannounced.
    io.setPivotPosition(pivotGoalRad);
    io.setRollerVelocity(0.0);
  }

  /** Holds the pivot at the stow angle with the roller stopped. */
  private void stowing() {
    // IMPLEMENTATION (was TODO: "io.setPivotPosition(kStowedAngleRad.get());
    // io.setRollerVelocity(0).")
    pivotGoalRad = geometry.stowedRad().getAsDouble();
    io.setPivotPosition(pivotGoalRad);
    io.setRollerVelocity(0.0);
  }

  /** Drives the pivot out to the deploy angle without spinning the roller yet. */
  private void deploying() {
    // IMPLEMENTATION (was TODO: "io.setPivotPosition(kDeployedAngleRad.get());
    // io.setRollerVelocity(0) until atGoal().")
    //
    // The roller stays stopped for the whole of DEPLOYING; the transition to INTAKING is what
    // starts it, which is why this handler does not need the "until atGoal()" test itself.
    // Spinning a roller through the travel would sweep pieces sideways on the way down and draw
    // current the power budget has not been asked for.
    pivotGoalRad = geometry.deployedRad().getAsDouble();
    io.setPivotPosition(pivotGoalRad);
    io.setRollerVelocity(0.0);
  }

  /**
   * Holds the deploy angle with the roller stopped: the resting posture while carrying pieces.
   *
   * <p><b>Outputs:</b> commands the pivot to the deploy angle and stops the roller.
   */
  private void deployed() {
    // Added for M10b. Identical outputs to deploying(); what differs is what the STATE tells the
    // rest of the robot -- that the arm has arrived, rather than that it is on its way.
    pivotGoalRad = geometry.deployedRad().getAsDouble();
    io.setPivotPosition(pivotGoalRad);
    io.setRollerVelocity(0.0);
  }

  /** Holds the deploy angle and runs the roller inward. */
  private void intaking() {
    // IMPLEMENTATION (was TODO: "io.setPivotPosition(kDeployedAngleRad.get());
    // io.setRollerVelocity(kIntakeRollerRpm.get()).")
    pivotGoalRad = geometry.deployedRad().getAsDouble();
    io.setPivotPosition(pivotGoalRad);
    io.setRollerVelocity(Constants.Intake.kIntakeRollerRpm.get());
  }

  /**
   * Rocks the arm between the deploy and feed-assist angles with the roller running inward.
   *
   * <p><b>Outputs:</b> commands the pivot to whichever end of the rock this half-period calls for,
   * and runs the roller at {@code kFeedAssistRollerRpm}.
   */
  private void feedAssisting() {
    // Added for M10b (plan section 2.4). Rocking lets pieces flow off the intake onto the hopper
    // floor and into the indexer better than a static raise. The phase is timed from entry into
    // this state, so every scoring burst starts its rock the same way: rising first.
    pivotGoalRad =
        feedAssistGoalRad(
            stateTimer.get(),
            Constants.Intake.kFeedAssistPeriodSec.get(),
            lowPriorityCurrentScalar >= 1.0,
            geometry.deployedRad().getAsDouble(),
            geometry.feedAssistRad().getAsDouble());
    io.setPivotPosition(pivotGoalRad);
    io.setRollerVelocity(Constants.Intake.kFeedAssistRollerRpm.get());
  }

  /**
   * Holds the arm where a motion into the band was refused, roller stopped, until the turret leaves
   * its arc.
   *
   * <p><b>Outputs:</b> commands the pivot to the angle latched on entry and stops the roller.
   */
  private void waitingForTurret() {
    // Added for M10b step 5. The latched angle is clear of the band by construction -- a motion is
    // only refused while the arm is outside it -- so the goal this publishes keeps
    // isInTurretConflictBand() false, and the turret stays free to finish what it is doing. The
    // roller stops because a refused stow is not a request to collect.
    pivotGoalRad = waitHoldRad;
    io.setPivotPosition(pivotGoalRad);
    io.setRollerVelocity(0.0);
  }

  /**
   * Which end of the feed-assist rock the pivot should be driven to right now.
   *
   * <p>A pure function so the rock can be tested without a pivot plant, which with the placeholder
   * gains rings for seconds and never settles at either end. It starts by rising to the feed-assist
   * angle and alternates every half period. It holds the feed-assist angle still when the period is
   * zero, and also when rocking is not allowed: rocking is pure pivot current, so it is the first
   * thing given up when {@code PowerBudget} starts cutting LOW-priority current. Holding the raised
   * angle still keeps pieces aboard, just with a slower flow.
   *
   * @param timeInStateSec seconds since the rock started
   * @param periodSec full rock period, in seconds; zero or less holds still
   * @param rockingAllowed false to hold the feed-assist angle regardless of the period
   * @param deployedRad the lower end of the rock
   * @param feedAssistRad the raised end of the rock
   * @return the pivot goal for this instant, in radians
   */
  static double feedAssistGoalRad(
      double timeInStateSec,
      double periodSec,
      boolean rockingAllowed,
      double deployedRad,
      double feedAssistRad) {
    if (periodSec <= 0.0 || !rockingAllowed) {
      return feedAssistRad;
    }
    long halfPeriodsElapsed = (long) Math.floor(timeInStateSec / (periodSec / 2.0));
    return halfPeriodsElapsed % 2 == 0 ? feedAssistRad : deployedRad;
  }

  /** Holds the eject angle and runs the roller outward. */
  private void ejecting() {
    // IMPLEMENTATION (was TODO: "io.setPivotPosition(kEjectAngleRad.get());
    // io.setRollerVelocity(kEjectRollerRpm.get()).")
    //
    // kEjectRollerRpm carries its own sign in Constants; it is not negated here. One place decides
    // which way "out" is, and it is the constant, so that a mechanism rebuild reversing the roller
    // is a one-line change rather than a hunt through the FSM.
    pivotGoalRad = geometry.ejectRad().getAsDouble();
    io.setPivotPosition(pivotGoalRad);
    io.setRollerVelocity(Constants.Intake.kEjectRollerRpm.get());
  }

  /** Drives the pivot into its hard stop to establish a zero. */
  private void homing() {
    // IMPLEMENTATION (was TODO: "io.setPivotOpenLoop(12.0 * kHomingDutyCycle.get()). When
    // Math.abs(inputs.pivotVelocityRadPerSec) < kHomingVelocityThresholdRadPerSec, start
    // homingSettleTimer; when it passes kHomingSettleSec, the mechanism is home. Bail out to
    // FAULTED after kHomingTimeoutSec so a jam does not stall the motor all match.")
    //
    // The timeout itself lives in handleStateTransition, because only that method may change
    // state. This handler decides success and records it; the transition method reads the result.
    io.setPivotOpenLoop(12.0 * Constants.Intake.kHomingDutyCycle.get());
    io.setRollerVelocity(0.0);

    // The settle time is what separates "arrived at the stop" from "passing through zero velocity
    // on the way there". A pivot reverses direction at the top of its travel and is momentarily
    // stationary; without the settle window that instant reads as a hard stop and seeds the zero
    // in completely the wrong place.
    boolean moving =
        Math.abs(inputs.pivotVelocityRadPerSec)
            >= Constants.Intake.kHomingVelocityThresholdRadPerSec.get();
    // A hard stop is a stall, and a stall draws current. A dead motor also reports zero velocity
    // but draws nothing, so the current term is what separates "arrived at the stop" from "never
    // started". Requiring proof of motion as well would false-fault a pivot that booted already
    // resting against its stop; requiring current does not, because driving into the stop pulls
    // current immediately whether or not the arm ever moved.
    boolean stalledAgainstSomething =
        !moving && inputs.pivotStatorCurrentAmps >= Constants.Intake.kHomingCurrentAmps.get();

    if (stalledAgainstSomething) {
      homingSettleTimer.start();
      if (homingSettleTimer.hasElapsed(Constants.Intake.kHomingSettleSec.get())) {
        homingSucceeded = true;
      }
    } else {
      homingSettleTimer.reset();
      homingSettleTimer.stop();
    }
  }

  /** Applies the externally-commanded characterization voltage. */
  private void characterizing() {
    // IMPLEMENTATION (was TODO: "io.setPivotOpenLoop(characterizationVolts), driven by
    // CharacterizationCommands.")
    io.setPivotOpenLoop(characterizationVolts);
    io.setRollerVelocity(0.0);
  }

  /** Holds everything stopped until a re-home is requested. */
  private void faulted() {
    // IMPLEMENTATION (was TODO: "io.stop() and hold. Only a HOME request clears FAULTED.")
    //
    // A genuine neutral output, not a closed-loop hold. If the pivot could not reach its stop the
    // cause may well be mechanical, and holding a jammed arm against a stator limit for the rest
    // of a match is how a motor is destroyed. The arm falling under gravity is the lesser harm,
    // and it falls towards stow.
    io.stop();
  }

  /**
   * Requests a new intake behavior.
   *
   * @param wanted the behavior to request
   *     <p><b>Outputs:</b> sets the wanted state; a HOME request also clears the homed flag so the
   *     machine really re-homes.
   */
  public void setWantedState(WantedState wanted) {
    // IMPLEMENTATION (was TODO: "Assign wantedState. If wanted == HOME, also set isHomed =
    // false.")
    //
    // Clearing isHomed here rather than only in onEnter(HOMING) matters for the case where the
    // machine is ALREADY in HOMING: re-requesting HOME would otherwise be a no-op transition, no
    // onEnter would fire, and an operator pressing the button again after a failed attempt would
    // see nothing happen.
    wantedState = wanted;
    if (wanted == WantedState.HOME) {
      isHomed = false;
      homingSucceeded = false;
      homeRequested = true;
    }
  }

  /**
   * Command-based adapter for button bindings.
   *
   * @param wanted the behavior to request when the command runs
   * @return an instant command that requests {@code wanted}
   */
  public Command setWantedStateCommand(WantedState wanted) {
    // Implemented rather than stubbed: returning null here would fail at binding-construction time,
    // long before a student could see why.
    return runOnce(() -> setWantedState(wanted));
  }

  /**
   * Sets the open-loop voltage used while characterizing.
   *
   * @param volts voltage to apply while in {@link SystemState#CHARACTERIZING}
   *     <p><b>Outputs:</b> stores the voltage.
   */
  public void setCharacterizationVolts(double volts) {
    // IMPLEMENTATION (was TODO: "characterizationVolts = volts.")
    characterizationVolts = volts;
  }

  /**
   * Whether the pivot has reached its commanded angle.
   *
   * @return true when the pivot is within {@code kPivotToleranceRad} of its goal
   */
  @AutoLogOutput(key = "Intake/AtGoal")
  public boolean atGoal() {
    // IMPLEMENTATION (was TODO: "Math.abs(inputs.pivotPositionRad - goalRad) <
    // kPivotToleranceRad.get(). You will need to track the current goal angle as a field to
    // compare against.")
    //
    // The goal field is pivotGoalRad, written by whichever handler commanded the pivot this loop.
    //
    // The isHomed term is not in the TODO and is essential: pivotPositionRad is meaningless before
    // the encoder is seeded, so without it a freshly-booted intake sitting at a raw zero would
    // report itself at goal, and handleStateTransition would promote STOWING straight to STOWED --
    // telling the turret it is safe to sweep through an arc containing an arm of unknown position.
    return atAngle(pivotGoalRad);
  }

  /**
   * Whether the pivot is at a given angle, on a trusted encoder.
   *
   * <p>Added for M10b so the transition logic can ask about the angle a request TARGETS rather than
   * the goal the previous loop commanded; see {@link #handleStateTransition()}.
   *
   * @param targetRad the angle to compare against, in radians
   * @return true when homed and within {@code kPivotToleranceRad} of {@code targetRad}
   */
  private boolean atAngle(double targetRad) {
    return isHomed
        && Math.abs(inputs.pivotPositionRad - targetRad)
            < Constants.Intake.kPivotToleranceRad.get();
  }

  /**
   * Whether the arm is, or is about to be, inside the turret's sweep.
   *
   * <p>Added for M10b (plan section 2.4). The turret consumes this from step 5 of the retrofit on,
   * replacing {@code !isStowed()}, which had the conflict backwards: on this robot a DEPLOYED
   * intake cannot reach the turret, and the STOWED one can. The conflict is the band of pivot
   * angles nearest stow, {@code turretConflictBandRad} wide.
   *
   * <p>Both the measured angle and the commanded goal are tested, so the answer turns true the
   * moment a move into the band is commanded rather than when the arm arrives. It fails CLOSED
   * whenever the encoder is untrusted: an unhomed arm could be anywhere, including in the band. The
   * robot starts stowed, so that is also simply true at boot.
   *
   * @return true while the pivot or its goal is in the band, or the pivot is unhomed
   */
  @AutoLogOutput(key = "Intake/InTurretConflictBand")
  public boolean isInTurretConflictBand() {
    if (!isHomed) {
      return true;
    }
    double stowed = geometry.stowedRad().getAsDouble();
    double deployed = geometry.deployedRad().getAsDouble();
    double width = geometry.turretConflictBandRad().getAsDouble();
    return inTurretConflictBand(inputs.pivotPositionRad, stowed, deployed, width)
        || inTurretConflictBand(pivotGoalRad, stowed, deployed, width);
  }

  /**
   * Whether an angle lies in the turret-conflict band.
   *
   * <p>The band is everything on stow's side of an edge {@code widthRad} in from stow toward
   * deploy, INCLUDING anything past stow. Stow is the hard stop, so nothing should read past it,
   * but a reading past it is still the arm at its most dangerous and must not report clear. Written
   * as "which side of the edge" rather than as an interval so that works without a second constant.
   *
   * <p>The direction comes from comparing the two angles, so the band follows the pivot's sign
   * convention on its own. When the two are equal -- which the 0.0 placeholders make true today --
   * there is no direction, and it fails CLOSED by treating everything within {@code widthRad} of
   * stow on either side as in the band.
   *
   * @param angleRad the angle to classify, in radians
   * @param stowedRad the stow angle, in radians
   * @param deployedRad the deploy angle, in radians
   * @param widthRad the band's width from stow, in radians
   * @return true if the angle is in the band
   */
  static boolean inTurretConflictBand(
      double angleRad, double stowedRad, double deployedRad, double widthRad) {
    if (deployedRad < stowedRad) {
      return angleRad >= stowedRad - widthRad;
    }
    if (deployedRad > stowedRad) {
      return angleRad <= stowedRad + widthRad;
    }
    return Math.abs(angleRad - stowedRad) <= widthRad;
  }

  /**
   * Whether the arm is low enough to keep pieces in the robot.
   *
   * <p>Added for M10b (plan section 2.4). Pieces can fall out while the intake is stowed but stay
   * in anywhere from the deploy angle up to the feed-assist angle, so every auto waits on this
   * before its first path: the robot must start stowed, and the first enable deploys it. Measured
   * position, not state, because the question is where the arm physically is.
   *
   * @return true when homed and between the deploy and feed-assist angles, within tolerance
   */
  @AutoLogOutput(key = "Intake/RetainingPieces")
  public boolean isRetainingPieces() {
    return isHomed
        && withinRetainingRange(
            inputs.pivotPositionRad,
            geometry.deployedRad().getAsDouble(),
            geometry.feedAssistRad().getAsDouble(),
            Constants.Intake.kPivotToleranceRad.get());
  }

  /**
   * Whether an angle lies between the deploy and feed-assist angles, widened by a tolerance.
   *
   * @param angleRad the angle to classify, in radians
   * @param deployedRad one end of the range, in radians
   * @param feedAssistRad the other end, in radians; either order
   * @param toleranceRad how far outside the range still counts, in radians
   * @return true if the angle is within the widened range
   */
  static boolean withinRetainingRange(
      double angleRad, double deployedRad, double feedAssistRad, double toleranceRad) {
    double low = Math.min(deployedRad, feedAssistRad) - toleranceRad;
    double high = Math.max(deployedRad, feedAssistRad) + toleranceRad;
    return angleRad >= low && angleRad <= high;
  }

  /**
   * Whether the pivot position is trustworthy.
   *
   * @return true once homing has completed or the absolute encoder has seeded position
   */
  @AutoLogOutput(key = "Intake/IsHomed")
  public boolean isHomed() {
    // IMPLEMENTATION (was TODO: "return isHomed.")
    return isHomed;
  }

  /**
   * Whether the intake is fully retracted.
   *
   * @return true when the state machine is in {@link SystemState#STOWED}; the turret asks this
   *     before sweeping through its conflict arc
   */
  @AutoLogOutput(key = "Intake/IsStowed")
  public boolean isStowed() {
    // IMPLEMENTATION (was TODO: "return systemState == SystemState.STOWED.")
    //
    // Deliberately the STATE and not a position test. STOWED is only reachable through atGoal(),
    // which already requires isHomed, so a true here can never come from an untrusted encoder
    // reading. This is the half of the intake/turret interlock the turret consumes, and it must
    // fail closed: while homing, faulted, or on the way in, the answer is no.
    return systemState == SystemState.STOWED;
  }

  /**
   * Whether the intake is at its intaking position with the roller running.
   *
   * <p>Until M10b this was the intake-camera gate. It no longer is: the intake now also RESTS at
   * that same angle, in {@link SystemState#DEPLOYED}, and a gate reading only this would switch the
   * camera off for most of a match with nothing failing. {@code Vision} reads {@link
   * #isAtIntakingAngle()} instead. This stays as the logged answer to "is the roller collecting".
   *
   * @return true when the state machine is in {@link SystemState#INTAKING}
   */
  @AutoLogOutput(key = "Intake/IsIntaking")
  public boolean isIntaking() {
    return systemState == SystemState.INTAKING;
  }

  /**
   * Whether the pivot has arrived at, and is holding, the one angle the intake camera's mounting
   * describes.
   *
   * <p>Consumed by {@code Vision}, which fuses the intake camera's observations only while this is
   * true (M10b step 4, plan section 2.4). That camera rides the pivot about 0.4 m from its axis, so
   * {@code kRobotToIntakeCamera} describes exactly one pose -- the deploy angle -- and is wrong
   * everywhere else.
   *
   * <p>Two states hold that angle, and both are admitted: {@link SystemState#INTAKING} and {@link
   * SystemState#DEPLOYED}. The second is why this exists. Down-stays-down means the intake rests
   * DEPLOYED for most of a match, and a gate that admitted only INTAKING would switch the camera
   * off for all of it, silently -- the camera simply contributes nothing, and nothing reports it.
   *
   * @return true when the state machine is in INTAKING or DEPLOYED
   */
  @AutoLogOutput(key = "Intake/AtIntakingAngle")
  public boolean isAtIntakingAngle() {
    // The STATE, not a position test, for the same reason isStowed is: both states are reachable
    // only through an arrival check that requires isHomed, so a true here cannot come from an
    // untrusted encoder reading -- and since step 3, that arrival is judged against the deploy
    // angle itself, not against whatever the previous loop commanded.
    //
    // Everything else that puts the arm near the deploy angle is excluded on purpose:
    //  - DEPLOYING: the pivot is in transit, and the camera is somewhere along a 0.4 m arc.
    //  - EJECTING: deployed, but to kEjectAngleRad, a DIFFERENT pivot angle. Accepting it would
    // fuse
    //    poses computed with the wrong transform -- plausible, on the field, and wrong.
    //  - FEED_ASSISTING: the rock passes through the deploy angle every period, but it is moving
    //    through it, and half the time it is up at the feed-assist angle. It is EJECTING's trap
    //    again, arriving from the other direction: out, near the right angle, and wrong.
    return systemState == SystemState.INTAKING || systemState == SystemState.DEPLOYED;
  }

  /**
   * The live pivot angle, published for the AdvantageScope 3D component pose.
   *
   * @return pivot angle at the gearbox output, in radians
   */
  public double getPivotAngleRad() {
    // IMPLEMENTATION (was TODO: "return inputs.pivotPositionRad.")
    return inputs.pivotPositionRad;
  }

  /**
   * The current state of the intake machine.
   *
   * <p>Added beyond the plan's method list, matching {@code Shooter.getSystemState()}, so the FSM
   * can be asserted on from a test rather than only read off a graph.
   *
   * @return the state the machine is running this loop
   */
  public SystemState getSystemState() {
    return systemState;
  }

  /**
   * The pivot velocity, consumed by the characterization routines.
   *
   * @return pivot angular velocity, in radians per second
   */
  public double getCharacterizationVelocityRadPerSec() {
    // IMPLEMENTATION (was TODO: "return inputs.pivotVelocityRadPerSec.")
    return inputs.pivotVelocityRadPerSec;
  }
}
