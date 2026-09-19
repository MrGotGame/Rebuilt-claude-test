package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

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

  private WantedState wantedState = WantedState.HOME;
  private SystemState systemState = SystemState.HOMING;
  private boolean isHomed = false;
  private double characterizationVolts = 0.0;
  private BooleanSupplier deployAllowed = () -> true;

  /**
   * Builds the intake around a hardware implementation.
   *
   * @param io the hardware layer: real, simulated, or an empty implementation for log replay
   */
  public Intake(IntakeIO io) {
    this.io = io;
    // TODO: Start stateTimer, and register FaultReporter checks for pivotConnected, rollerConnected
    // and absEncoderConnected so a dead device is visible on the dashboard rather than presenting
    // as "the intake just does not work".
  }

  /**
   * Installs the interlock that prevents deploying into the turret.
   *
   * @param deployAllowed supplier returning false while the turret is inside its conflict arc;
   *     stored, not evaluated, at call time
   *     <p><b>Outputs:</b> replaces the stored guard.
   */
  public void setDeployGuard(BooleanSupplier deployAllowed) {
    // TODO: this.deployAllowed = deployAllowed. The Superstructure injects this.
  }

  /**
   * Runs one iteration of the state machine. Called by the command scheduler every 20 ms.
   *
   * <p><b>Outputs:</b> refreshes and logs inputs; may change {@code systemState}; commands the
   * hardware; pushes the power budget scalar down to the IO layer.
   */
  @Override
  public void periodic() {
    // TODO: In order: io.updateInputs(inputs); Logger.processInputs("Intake", inputs); apply
    // tunable
    // gains through LoggedTunableGains.ifChanged; then run the transition sequence -- compute
    // handleStateTransition(), and if it differs from systemState call onExit(old), onEnter(new)
    // and
    // reset stateTimer; then applyState(); then log the active state as an enum.
    // Finally push PowerBudget.getInstance().getScalar(Priority.LOW) into io.setCurrentLimitScalar.
  }

  /**
   * Decides which state the machine should be in this loop.
   *
   * @return the state to run this iteration; may be the current state
   */
  private SystemState handleStateTransition() {
    // TODO: switch (wantedState):
    //   HOME -> HOMING, or FAULTED once stateTimer exceeds kHomingTimeoutSec
    //   anything else while !isHomed -> HOMING (never move closed loop off an unknown position)
    //   STOW -> STOWING, then STOWED once atGoal()
    //   DEPLOY_AND_INTAKE -> DEPLOYING until atGoal(), then INTAKING; but stay STOWED entirely if
    //       !deployAllowed.getAsBoolean()
    //   DEPLOY_ONLY -> DEPLOYING
    //   EJECT -> EJECTING
    //   CHARACTERIZE -> CHARACTERIZING
    //   IDLE -> IDLE
    return SystemState.IDLE;
  }

  /**
   * Runs one-shot work when a state is entered.
   *
   * @param state the state being entered
   *     <p><b>Outputs:</b> resets homing bookkeeping or characterization voltage as appropriate.
   */
  private void onEnter(SystemState state) {
    // TODO: HOMING -> isHomed = false and homingSettleTimer.reset().
    // CHARACTERIZING -> characterizationVolts = 0.
  }

  /**
   * Runs one-shot work when a state is left.
   *
   * @param state the state being left
   *     <p><b>Outputs:</b> on a successful homing exit, seeds the pivot position and marks homed.
   */
  private void onExit(SystemState state) {
    // TODO: HOMING -> io.seedPivotPosition(Constants.Intake.kStowedAngleRad.get()) and
    // isHomed = true, but ONLY when the exit was a success rather than a timeout into FAULTED.
  }

  /**
   * Dispatches to the handler for the current state.
   *
   * <p><b>Outputs:</b> commands the hardware for this loop.
   */
  private void applyState() {
    // TODO: switch (systemState) calling the private handlers below.
  }

  /** Holds the pivot at the stow angle with the roller stopped. */
  private void stowing() {
    // TODO: io.setPivotPosition(kStowedAngleRad.get()); io.setRollerVelocity(0).
  }

  /** Drives the pivot out to the deploy angle without spinning the roller yet. */
  private void deploying() {
    // TODO: io.setPivotPosition(kDeployedAngleRad.get()); io.setRollerVelocity(0) until atGoal().
  }

  /** Holds the deploy angle and runs the roller inward. */
  private void intaking() {
    // TODO: io.setPivotPosition(kDeployedAngleRad.get());
    // io.setRollerVelocity(kIntakeRollerRpm.get()).
  }

  /** Holds the eject angle and runs the roller outward. */
  private void ejecting() {
    // TODO: io.setPivotPosition(kEjectAngleRad.get()); io.setRollerVelocity(kEjectRollerRpm.get()).
  }

  /** Drives the pivot into its hard stop to establish a zero. */
  private void homing() {
    // TODO: io.setPivotOpenLoop(12.0 * kHomingDutyCycle.get()). When
    // Math.abs(inputs.pivotVelocityRadPerSec) < kHomingVelocityThresholdRadPerSec, start
    // homingSettleTimer; when it passes kHomingSettleSec, the mechanism is home. Bail out to
    // FAULTED after kHomingTimeoutSec so a jam does not stall the motor all match.
  }

  /** Applies the externally-commanded characterization voltage. */
  private void characterizing() {
    // TODO: io.setPivotOpenLoop(characterizationVolts), driven by CharacterizationCommands.
  }

  /** Holds everything stopped until a re-home is requested. */
  private void faulted() {
    // TODO: io.stop() and hold. Only a HOME request clears FAULTED.
  }

  /**
   * Requests a new intake behavior.
   *
   * @param wanted the behavior to request
   *     <p><b>Outputs:</b> sets the wanted state; a HOME request also clears the homed flag so the
   *     machine really re-homes.
   */
  public void setWantedState(WantedState wanted) {
    // TODO: Assign wantedState. If wanted == HOME, also set isHomed = false.
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
    // TODO: characterizationVolts = volts.
  }

  /**
   * Whether the pivot has reached its commanded angle.
   *
   * @return true when the pivot is within {@code kPivotToleranceRad} of its goal
   */
  @AutoLogOutput(key = "Intake/AtGoal")
  public boolean atGoal() {
    // TODO: Math.abs(inputs.pivotPositionRad - goalRad) < kPivotToleranceRad.get(). You will need
    // to
    // track the current goal angle as a field to compare against.
    return false;
  }

  /**
   * Whether the pivot position is trustworthy.
   *
   * @return true once homing has completed or the absolute encoder has seeded position
   */
  @AutoLogOutput(key = "Intake/IsHomed")
  public boolean isHomed() {
    // TODO: return isHomed.
    return false;
  }

  /**
   * Whether the intake is fully retracted.
   *
   * @return true when the state machine is in {@link SystemState#STOWED}; the turret asks this
   *     before sweeping through its conflict arc
   */
  @AutoLogOutput(key = "Intake/IsStowed")
  public boolean isStowed() {
    // TODO: return systemState == SystemState.STOWED.
    return false;
  }

  /**
   * The live pivot angle, published for the AdvantageScope 3D component pose.
   *
   * @return pivot angle at the gearbox output, in radians
   */
  public double getPivotAngleRad() {
    // TODO: return inputs.pivotPositionRad.
    return 0.0;
  }

  /**
   * The pivot velocity, consumed by the characterization routines.
   *
   * @return pivot angular velocity, in radians per second
   */
  public double getCharacterizationVelocityRadPerSec() {
    // TODO: return inputs.pivotVelocityRadPerSec.
    return 0.0;
  }
}
