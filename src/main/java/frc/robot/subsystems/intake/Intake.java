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
 * <p>Second, a deploy guard: the intake refuses to deploy while the turret is inside its conflict
 * arc. That interaction is the single most likely way to destroy this robot.
 */
public class Intake extends SubsystemBase {
  /** What the operator or superstructure has asked the intake to do. */
  public enum WantedState {
    IDLE,
    STOW,
    DEPLOY_AND_INTAKE,
    DEPLOY_ONLY,
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
    INTAKING,
    EJECTING,
    HOMING,
    CHARACTERIZING,
    FAULTED
  }

  private final IntakeIO io;
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
   * <p>A warning rather than an error, because this is the expected state of the robot until M10
   * implements the superstructure. It exists so that "the intake will not deploy" reads as a
   * deliberate refusal on the dashboard rather than as a broken mechanism.
   */
  private final Alert noDeployGuardAlert =
      new Alert(
          "No intake deploy guard installed; the intake will refuse to deploy until Superstructure"
              + " provides one.",
          AlertType.kWarning);

  private WantedState wantedState = WantedState.HOME;
  private SystemState systemState = SystemState.HOMING;
  private boolean isHomed = false;
  private double characterizationVolts = 0.0;

  /**
   * The interim answer to "is it safe to swing the intake out".
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
   * Builds the intake around a hardware implementation.
   *
   * @param io the hardware layer: real, simulated, or an empty implementation for log replay
   */
  public Intake(IntakeIO io) {
    // IMPLEMENTATION (was TODO: "Start stateTimer, and register FaultReporter checks for
    // pivotConnected, rollerConnected and absEncoderConnected so a dead device is visible on the
    // dashboard rather than presenting as 'the intake just does not work'.")
    //
    // All three checks exist, as the Alert fields above, driven from periodic() rather than
    // registered, because FaultReporter is not implemented yet. See the field documentation.
    this.io = io;
    stateTimer.start();
  }

  /**
   * Installs the interlock that prevents deploying into the turret.
   *
   * @param deployAllowed supplier returning false while the turret is inside its conflict arc;
   *     stored, not evaluated, at call time
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
    io.setCurrentLimitScalar(PowerBudget.getInstance().getScalar(PowerBudget.Priority.LOW));

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
    if (systemState == SystemState.FAULTED && wantedState != WantedState.HOME) {
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

    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case STOW -> atGoal() ? SystemState.STOWED : SystemState.STOWING;

        // The guard covers all three states that put the arm outside the stow position, not just
        // the
        // two named DEPLOY. EJECT swings to kEjectAngleRad, which is just as capable of meeting the
        // turret as the deploy angle is.
      case DEPLOY_AND_INTAKE -> {
        if (!deployAllowed.getAsBoolean()) {
          yield atGoal() ? SystemState.STOWED : SystemState.STOWING;
        }
        yield atGoal() ? SystemState.INTAKING : SystemState.DEPLOYING;
      }
      case DEPLOY_ONLY -> {
        if (!deployAllowed.getAsBoolean()) {
          yield atGoal() ? SystemState.STOWED : SystemState.STOWING;
        }
        yield SystemState.DEPLOYING;
      }
      case EJECT -> {
        if (!deployAllowed.getAsBoolean()) {
          yield atGoal() ? SystemState.STOWED : SystemState.STOWING;
        }
        yield SystemState.EJECTING;
      }

        // Unreachable: both are handled above, before the homing gate. Listed so that adding a
        // WantedState later is a compile error here rather than a silent fall-through.
      case HOME, CHARACTERIZE -> systemState;
    };
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
        homingSettleTimer.reset();
        homingSettleTimer.stop();

        // Beyond the TODO, and load-bearing: homing deliberately drives the pivot INTO its hard
        // stop, and a soft limit is a software refusal to go there. Leaving the limits enabled
        // would stop the arm short of the stop, velocity would fall to zero anyway, and the
        // mechanism would cheerfully declare itself homed at the wrong angle -- every subsequent
        // position command then off by that error, with nothing reporting a fault.
        io.setPivotSoftLimitsEnabled(false);
      }
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
          io.seedPivotPosition(Constants.Intake.kStowedAngleRad.get());
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
      case INTAKING -> intaking();
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
    pivotGoalRad = Constants.Intake.kStowedAngleRad.get();
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
    pivotGoalRad = Constants.Intake.kDeployedAngleRad.get();
    io.setPivotPosition(pivotGoalRad);
    io.setRollerVelocity(0.0);
  }

  /** Holds the deploy angle and runs the roller inward. */
  private void intaking() {
    // IMPLEMENTATION (was TODO: "io.setPivotPosition(kDeployedAngleRad.get());
    // io.setRollerVelocity(kIntakeRollerRpm.get()).")
    pivotGoalRad = Constants.Intake.kDeployedAngleRad.get();
    io.setPivotPosition(pivotGoalRad);
    io.setRollerVelocity(Constants.Intake.kIntakeRollerRpm.get());
  }

  /** Holds the eject angle and runs the roller outward. */
  private void ejecting() {
    // IMPLEMENTATION (was TODO: "io.setPivotPosition(kEjectAngleRad.get());
    // io.setRollerVelocity(kEjectRollerRpm.get()).")
    //
    // kEjectRollerRpm carries its own sign in Constants; it is not negated here. One place decides
    // which way "out" is, and it is the constant, so that a mechanism rebuild reversing the roller
    // is a one-line change rather than a hunt through the FSM.
    pivotGoalRad = Constants.Intake.kEjectAngleRad.get();
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
    if (Math.abs(inputs.pivotVelocityRadPerSec)
        < Constants.Intake.kHomingVelocityThresholdRadPerSec.get()) {
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
    return isHomed
        && Math.abs(inputs.pivotPositionRad - pivotGoalRad)
            < Constants.Intake.kPivotToleranceRad.get();
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
