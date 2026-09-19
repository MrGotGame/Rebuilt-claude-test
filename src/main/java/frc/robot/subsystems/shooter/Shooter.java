package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Flywheel state machine, with one capability that specifically matters for shooting on the move:
 * <b>predictive readiness</b>.
 *
 * <p>A flywheel recovering from a shot takes a fixed, measurable time to return to speed. Rather
 * than waiting for the error to actually fall inside tolerance, {@link #willBeAtSpeedIn(double)}
 * projects the current velocity forward using the measured spin-up slope, so the superstructure can
 * open the feed a few tens of milliseconds early and keep the shot cadence up.
 *
 * <p>It also holds an idle RPM between shots. Spinning up from idle rather than from zero roughly
 * halves recovery time, which is worth about one extra shot per scoring cycle.
 */
public class Shooter extends SubsystemBase {
  /** What the operator or superstructure has asked the flywheel to do. */
  public enum WantedState {
    IDLE,
    COAST,
    SPIN_UP,
    SHOOT,
    CHARACTERIZE
  }

  /** What the flywheel is actually doing right now. */
  public enum SystemState {
    IDLE,
    COASTING,
    SPINNING_UP,
    AT_SPEED,
    CHARACTERIZING,
    FAULTED
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();
  private final LinearFilter accelFilter = LinearFilter.movingAverage(5);

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private double goalRpm = 0.0;
  private double lastVelocityRpm = 0.0;
  private double accelRpmPerSec = 0.0;
  private double characterizationVolts = 0.0;

  /**
   * Builds the shooter around a hardware implementation.
   *
   * @param io the hardware layer: real, simulated, or empty for log replay
   */
  public Shooter(ShooterIO io) {
    this.io = io;
    // TODO: Start stateTimer, and register FaultReporter checks for BOTH motors separately --
    // leaderConnected and followerConnected are independent failures with very different symptoms.
  }

  /**
   * Runs one iteration of the state machine. Called by the command scheduler every 20 ms.
   *
   * <p><b>Outputs:</b> refreshes and logs inputs; updates the acceleration estimate; may change
   * {@code systemState}; commands the hardware.
   */
  @Override
  public void periodic() {
    // TODO: The standard five-step periodic, plus: differentiate inputs.velocityRpm against
    // lastVelocityRpm over the loop period and push it through accelFilter into accelRpmPerSec
    // (this is what willBeAtSpeedIn projects with); push gains via ifChanged; push the PowerBudget
    // scalar at Priority.HIGH; and log goalRpm, the error, and both motors' currents.
  }

  /**
   * Decides which state the machine should be in this loop.
   *
   * @return the state to run this iteration
   */
  private SystemState handleStateTransition() {
    // TODO: CHARACTERIZE -> CHARACTERIZING. IDLE -> IDLE. COAST -> COASTING.
    // SPIN_UP or SHOOT -> SPINNING_UP until atSpeed(), then AT_SPEED.
    // If SPINNING_UP exceeds kSpinUpTimeoutSec -> FAULTED. That timeout matters: a jammed or
    // unpowered flywheel must not silently block the scoring state for the rest of the match.
    return SystemState.IDLE;
  }

  /**
   * Dispatches to the hardware for the current state.
   *
   * <p><b>Outputs:</b> commands the flywheel for this loop.
   */
  private void applyState() {
    // TODO: IDLE -> io.setVelocity(kIdleRpm.get()); COASTING -> io.stop();
    // SPINNING_UP and AT_SPEED -> io.setVelocity(goalRpm);
    // CHARACTERIZING -> io.setOpenLoop(characterizationVolts); FAULTED -> io.stop().
  }

  /**
   * Sets the target flywheel speed. Called every loop from the superstructure with the shooting
   * solver's output.
   *
   * @param rpm desired flywheel speed, in RPM; clamped into the configured maximum
   *     <p><b>Outputs:</b> stores the goal and requests {@link WantedState#SPIN_UP}.
   */
  public void setGoal(double rpm) {
    // TODO: goalRpm = MathUtil.clamp(rpm, 0.0, Constants.Shooter.kMaxRpm.get());
    // wantedState = WantedState.SPIN_UP.
  }

  /**
   * Requests a new flywheel behavior.
   *
   * @param wanted the behavior to request
   *     <p><b>Outputs:</b> sets the wanted state.
   */
  public void setWantedState(WantedState wanted) {
    // TODO: wantedState = wanted.
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
   * Sets the open-loop voltage used while characterizing.
   *
   * @param volts voltage to apply while in {@link SystemState#CHARACTERIZING}
   *     <p><b>Outputs:</b> stores the voltage.
   */
  public void setCharacterizationVolts(double volts) {
    // TODO: characterizationVolts = volts.
  }

  /**
   * Whether the flywheel is at its commanded speed right now.
   *
   * @return true when within {@code kToleranceRpm} of a non-zero goal
   */
  @AutoLogOutput(key = "Shooter/AtSpeed")
  public boolean atSpeed() {
    // TODO: Math.abs(inputs.velocityRpm - goalRpm) < kToleranceRpm.get() && goalRpm > 0.
    return false;
  }

  /**
   * Whether the flywheel will be at speed a short time from now, projecting the current spin-up
   * slope forward.
   *
   * @param horizonSec how far ahead to project, in seconds
   * @return true if the projected speed lands inside the tolerance band
   */
  @AutoLogOutput(key = "Shooter/WillBeAtSpeed")
  public boolean willBeAtSpeedIn(double horizonSec) {
    // TODO: Project inputs.velocityRpm + accelRpmPerSec * horizonSec and test it against the
    // tolerance band around goalRpm. This is what lets the superstructure open the feed slightly
    // before the flywheel actually arrives.
    return false;
  }

  /**
   * The live flywheel speed.
   *
   * @return flywheel speed, in RPM
   */
  public double getVelocityRpm() {
    // TODO: return inputs.velocityRpm.
    return 0.0;
  }

  /**
   * The commanded flywheel speed.
   *
   * @return goal speed, in RPM
   */
  public double getGoalRpm() {
    // TODO: return goalRpm.
    return 0.0;
  }

  /**
   * The flywheel speed in Phoenix native units, for characterization fits.
   *
   * @return flywheel speed, in rotations per second
   */
  public double getCharacterizationVelocityRotPerSec() {
    // TODO: return inputs.velocityRpm / 60.0. CharacterizationCommands fits kS and kV in Phoenix
    // native units, not RPM.
    return 0.0;
  }
}
