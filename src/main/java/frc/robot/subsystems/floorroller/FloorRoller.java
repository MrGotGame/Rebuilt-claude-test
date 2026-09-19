package frc.robot.subsystems.floorroller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * The simplest state machine in this project, and deliberately still a state machine.
 *
 * <p>Uniformity is the point: a student who can read one subsystem can read all six, and the floor
 * roller gets the same timeout, logging and fault behavior as everything else for free.
 */
public class FloorRoller extends SubsystemBase {
  /** What the operator or superstructure has asked the floor roller to do. */
  public enum WantedState {
    IDLE,
    INTAKE,
    FEED,
    EJECT,
    CHARACTERIZE
  }

  /** What the floor roller is actually doing right now. */
  public enum SystemState {
    IDLE,
    INTAKING,
    FEEDING,
    EJECTING,
    CHARACTERIZING
  }

  private final FloorRollerIO io;
  private final FloorRollerIOInputsAutoLogged inputs = new FloorRollerIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();
  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private double characterizationVolts = 0.0;

  /**
   * Builds the floor roller around a hardware implementation.
   *
   * @param io the hardware layer: real, simulated, or empty for log replay
   */
  public FloorRoller(FloorRollerIO io) {
    this.io = io;
    // TODO: Start stateTimer and register a FaultReporter disconnect check on inputs.connected.
  }

  /**
   * Runs one iteration of the state machine. Called by the command scheduler every 20 ms.
   *
   * <p><b>Outputs:</b> refreshes and logs inputs; may change {@code systemState}; commands the
   * hardware; pushes the power budget scalar down to the IO layer.
   */
  @Override
  public void periodic() {
    // TODO: The standard five-step periodic: updateInputs, processInputs, tunable-gain push,
    // transition (with onEnter/onExit and a stateTimer reset), applyState, then log the state enum.
    // Also push PowerBudget.getInstance().getScalar(Priority.LOW) into io.setCurrentLimitScalar.
  }

  /**
   * Decides which state the machine should be in this loop.
   *
   * @return the state to run this iteration
   */
  private SystemState handleStateTransition() {
    // TODO: A direct WantedState -> SystemState mapping. No guards are needed for a roller.
    return SystemState.IDLE;
  }

  /**
   * Dispatches to the hardware for the current state.
   *
   * <p><b>Outputs:</b> commands the roller for this loop.
   */
  private void applyState() {
    // TODO: switch (systemState) dispatching io.setVelocity with kIntakeRpm / kFeedRpm / kEjectRpm,
    // io.stop() for IDLE, or io.setOpenLoop(characterizationVolts) for CHARACTERIZING.
  }

  /**
   * Requests a new roller behavior.
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
   * Whether the roller has reached its commanded speed.
   *
   * @return true when within a tunable tolerance of the goal RPM; the superstructure's feed gate
   *     uses this
   */
  @AutoLogOutput(key = "FloorRoller/AtSpeed")
  public boolean atSpeed() {
    // TODO: Math.abs(inputs.velocityRpm - goalRpm) < a tunable tolerance. Track the goal as a
    // field.
    return false;
  }

  /**
   * The live roller speed.
   *
   * @return roller speed, in RPM
   */
  public double getVelocityRpm() {
    // TODO: return inputs.velocityRpm.
    return 0.0;
  }
}
