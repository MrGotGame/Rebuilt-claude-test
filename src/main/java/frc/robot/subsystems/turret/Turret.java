package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * The most subtle subsystem in this project. Four jobs, each of which is easy to get subtly wrong.
 *
 * <ol>
 *   <li>It converts a <i>field-relative</i> aim azimuth into a <i>continuous, in-range,
 *       robot-relative</i> turret setpoint, via {@code TurretUnwrapper}.
 *   <li>It applies a velocity feedforward equal to the target's angular rate minus the chassis yaw
 *       rate, so the turret holds its aim while the drivetrain spins.
 *   <li>It publishes its angle into {@code RobotState}'s interpolation buffer every loop, which is
 *       what makes the turret-mounted camera's moving transform correct.
 *   <li>It refuses to sweep through the intake's arc while the intake is deployed.
 * </ol>
 *
 * <p>When the desired azimuth is simply unreachable it enters {@link SystemState#OUT_OF_RANGE} and
 * reports that upward, so the superstructure can ask the drivetrain to rotate instead. The turret
 * never jams itself against a hard stop trying to reach an impossible angle.
 */
public class Turret extends SubsystemBase {
  /** What the operator or superstructure has asked the turret to do. */
  public enum WantedState {
    IDLE,
    STOW,
    AIM,
    MANUAL,
    HOME,
    CHARACTERIZE
  }

  /** What the turret is actually doing right now. */
  public enum SystemState {
    IDLE,
    STOWING,
    AIMING,
    OUT_OF_RANGE,
    UNWRAPPING,
    MANUAL,
    HOMING,
    CHARACTERIZING,
    FAULTED
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private Rotation2d goalFieldAngle = Rotation2d.kZero;
  private double goalFieldAngularVelocityRadPerSec = 0.0;
  private double manualVolts = 0.0;
  private double characterizationVolts = 0.0;
  private double lastCommandedPositionRad = 0.0;
  private boolean isHomed = false;
  private BooleanSupplier intakeArcBlocked = () -> false;

  /**
   * Builds the turret around a hardware implementation.
   *
   * @param io the hardware layer: real, simulated, or empty for log replay
   */
  public Turret(TurretIO io) {
    this.io = io;
    // TODO: Start stateTimer, register FaultReporter checks for motorConnected and
    // absEncoderConnected, and push the initial soft limits with io.setSoftLimits(
    // kMinAngleRad.get() + kLimitMarginRad.get(), kMaxAngleRad.get() - kLimitMarginRad.get()).
  }

  /**
   * Installs the interlock that keeps the turret out of the deployed intake.
   *
   * @param blocked supplier returning true while the intake occupies the conflict arc; the
   *     superstructure injects {@code () -> !intake.isStowed()}
   *     <p><b>Outputs:</b> replaces the stored guard.
   */
  public void setIntakeArcGuard(BooleanSupplier blocked) {
    // TODO: this.intakeArcBlocked = blocked. Stored, and evaluated fresh each loop.
  }

  /**
   * Runs one iteration of the state machine. Called by the command scheduler every 20 ms.
   *
   * <p><b>Outputs:</b> refreshes and logs inputs; publishes the turret angle into {@code
   * RobotState}; may change {@code systemState}; commands the hardware.
   */
  @Override
  public void periodic() {
    // TODO: The standard five-step periodic, plus three turret-specific pieces:
    //   (a) IMMEDIATELY after Logger.processInputs, call
    //       RobotState.getInstance().addTurretObservation(Timer.getFPGATimestamp(),
    //       inputs.positionRad). Do it first so the buffer sample is as close to the measurement
    //       time as possible -- every millisecond of delay here becomes camera transform error.
    //   (b) Re-apply soft limits and gains through LoggedTunableNumber.ifChanged.
    //   (c) Log the unwrap solution and the field-relative angle the turret is actually pointing.
  }

  /**
   * Decides which state the machine should be in this loop.
   *
   * @return the state to run this iteration
   */
  private SystemState handleStateTransition() {
    // TODO: HOME -> HOMING, or FAULTED on timeout. !isHomed -> HOMING. MANUAL -> MANUAL.
    // CHARACTERIZE -> CHARACTERIZING. STOW -> STOWING.
    // AIM -> compute the unwrap result, then:
    //   !result.reachable()                -> OUT_OF_RANGE
    //   result.requiresLongWayAround()     -> UNWRAPPING, until |error| is small, then AIMING
    //   otherwise                          -> AIMING
    // If intakeArcBlocked.getAsBoolean() and the chosen path passes through the conflict arc, stay
    // in OUT_OF_RANGE rather than sweeping through the intake.
    return SystemState.IDLE;
  }

  /**
   * Runs one-shot work when a state is entered.
   *
   * @param state the state being entered
   *     <p><b>Outputs:</b> resets homing bookkeeping, latches an unwrap branch, or logs an event.
   */
  private void onEnter(SystemState state) {
    // TODO: HOMING -> isHomed = false. UNWRAPPING -> latch the chosen branch so it cannot flip
    // mid-sweep, which would leave the turret oscillating between two equally-distant solutions.
    // OUT_OF_RANGE -> log a one-shot event so it is findable in the match log.
  }

  /**
   * Runs one-shot work when a state is left.
   *
   * @param state the state being left
   *     <p><b>Outputs:</b> on a successful homing exit, seeds position and marks homed.
   */
  private void onExit(SystemState state) {
    // TODO: HOMING -> io.seedPosition(inputs.absolutePositionRad) and isHomed = true.
  }

  /**
   * Dispatches to the handler for the current state.
   *
   * <p><b>Outputs:</b> commands the hardware for this loop.
   */
  private void applyState() {
    // TODO: switch (systemState) calling the private handlers below.
  }

  /** Tracks the goal azimuth with chassis-rotation-cancelling feedforward. */
  private void aiming() {
    // TODO:
    //   double robotRelGoal = goalFieldAngle.getRadians()
    //       - RobotState.getInstance().getRotation().getRadians();
    //   TurretUnwrapper.Result r = TurretUnwrapper.solve(inputs.positionRad, robotRelGoal,
    //       kMinAngleRad.get(), kMaxAngleRad.get(), kUnwrapHysteresisRad.get());
    //   double ff = goalFieldAngularVelocityRadPerSec
    //       - RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
    //   io.setPositionSetpoint(r.targetPositionRad(), ff);
    //   lastCommandedPositionRad = r.targetPositionRad();
    // The MINUS on the chassis omega is the whole trick: the turret must rotate opposite the
    // chassis to stay pointed at a fixed field target. Getting this sign backwards doubles the
    // error instead of cancelling it, and it only shows up while rotating.
  }

  /** Sweeps the long way around to a latched branch target. */
  private void unwrapping() {
    // TODO: Same as aiming() but with the velocity feedforward FORCED TO ZERO and using the latched
    // branch target. During a 300-degree sweep the chassis-cancelling feedforward is meaningless
    // and actively fights the profile.
  }

  /** Holds the closest reachable angle while the drivetrain rotates to help. */
  private void outOfRange() {
    // TODO: Clamp the desired robot-relative angle into [min, max] and hold there, so the turret is
    // already as close as it can be the moment the chassis finishes rotating.
  }

  /** Holds the stow angle. */
  private void stowing() {
    // TODO: io.setPositionSetpoint(Constants.Turret.kStowAngleRad.get(), 0.0).
  }

  /** Establishes a trustworthy zero, preferring the absolute encoder. */
  private void homing() {
    // TODO: If inputs.absEncoderConnected, seed immediately from absolutePositionRad and finish in
    // one loop -- that is the whole point of an output-shaft encoder. Only if it is disconnected,
    // fall back to driving open loop toward one hard stop at a low duty cycle, watching for the
    // velocity to settle, then seeding. Time out to FAULTED.
  }

  /** Applies the operator's manual nudge, still respecting the limits. */
  private void manual() {
    // TODO: io.setOpenLoop(manualVolts), clamped, and refusing to drive further past a soft limit.
  }

  /** Applies the externally-commanded characterization voltage. */
  private void characterizing() {
    // TODO: io.setOpenLoop(characterizationVolts).
  }

  /**
   * Sets the field-relative aim goal. Called every loop from the superstructure with the shooting
   * solver's output.
   *
   * @param fieldAngle the field-relative azimuth to point at
   * @param fieldAngularVelocityRadPerSec how fast that azimuth is itself moving, in radians per
   *     second
   *     <p><b>Outputs:</b> stores both and requests {@link WantedState#AIM}.
   */
  public void setGoal(Rotation2d fieldAngle, double fieldAngularVelocityRadPerSec) {
    // TODO: Store both, then wantedState = WantedState.AIM.
  }

  /**
   * Requests a new turret behavior.
   *
   * @param wanted the behavior to request
   *     <p><b>Outputs:</b> sets the wanted state; a HOME request also clears the homed flag.
   */
  public void setWantedState(WantedState wanted) {
    // TODO: Assign. If wanted == HOME, also set isHomed = false.
  }

  /**
   * Command-based adapter for button bindings.
   *
   * @param wanted the behavior to request when the command runs
   * @return an instant command that requests {@code wanted}
   */
  public Command setWantedStateCommand(WantedState wanted) {
    // Implemented rather than stubbed: returning null would fail at binding-construction time.
    return runOnce(() -> setWantedState(wanted));
  }

  /**
   * Sets the operator's manual jog voltage.
   *
   * @param volts voltage to apply while in {@link SystemState#MANUAL}
   *     <p><b>Outputs:</b> stores the voltage.
   */
  public void setManualVolts(double volts) {
    // TODO: manualVolts = volts.
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
   * The live robot-relative turret angle.
   *
   * @return continuous turret position in the robot frame, in radians
   */
  @AutoLogOutput(key = "Turret/PositionRad")
  public double getPositionRad() {
    // TODO: return inputs.positionRad.
    return 0.0;
  }

  /**
   * Where the turret is pointing in field coordinates.
   *
   * @return field-relative turret azimuth; the AdvantageScope 3D turret component and the vision
   *     transform both key off this
   */
  @AutoLogOutput(key = "Turret/FieldAngle")
  public Rotation2d getFieldAngle() {
    // TODO:
    // Rotation2d.fromRadians(inputs.positionRad).plus(RobotState.getInstance().getRotation()).
    return Rotation2d.kZero;
  }

  /**
   * How fast the turret is slewing.
   *
   * @return turret angular velocity, in radians per second; vision uses this to reject frames
   *     captured mid-slew
   */
  public double getVelocityRadPerSec() {
    // TODO: return inputs.velocityRadPerSec.
    return 0.0;
  }

  /**
   * Whether the turret is genuinely on target.
   *
   * @return true when both the position error and the velocity error are inside tolerance
   */
  @AutoLogOutput(key = "Turret/AtGoal")
  public boolean atGoal() {
    // TODO: Math.abs(inputs.closedLoopErrorRad) < kToleranceRad.get()
    //     AND Math.abs(inputs.velocityRadPerSec - commandedFeedforward)
    //         < kVelocityToleranceRadPerSec.get().
    // BOTH terms matter. A turret sweeping through the correct angle at speed is not on target, and
    // a position-only check will happily fire mid-sweep.
    return false;
  }

  /**
   * Whether the requested azimuth is unreachable.
   *
   * @return true while in {@link SystemState#OUT_OF_RANGE}; the superstructure reads this to decide
   *     to rotate the chassis
   */
  @AutoLogOutput(key = "Turret/IsOutOfRange")
  public boolean isOutOfRange() {
    // TODO: return systemState == SystemState.OUT_OF_RANGE.
    return false;
  }

  /**
   * Whether the turret position is trustworthy.
   *
   * @return true once homed or seeded from the absolute encoder
   */
  public boolean isHomed() {
    // TODO: return isHomed.
    return false;
  }

  /**
   * The turret velocity, consumed by the characterization routines.
   *
   * @return turret angular velocity, in radians per second
   */
  public double getCharacterizationVelocityRadPerSec() {
    // TODO: return inputs.velocityRadPerSec.
    return 0.0;
  }
}
