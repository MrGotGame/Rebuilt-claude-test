package frc.robot.subsystems.floorroller;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.FaultReporter;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PowerBudget;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
    FEED,
    EJECT,
    CHARACTERIZE
  }

  /** What the floor roller is actually doing right now. */
  public enum SystemState {
    IDLE,
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
   * The speed currently being commanded, in RPM, or zero when the roller is stopped or open loop.
   *
   * <p>Tracked here rather than in {@code FloorRollerIOInputs} because, unlike the flywheel, this
   * mechanism's IO struct carries no goal field. {@link #applyState()} is the only writer, so the
   * goal and the command issued to the hardware cannot disagree -- which is exactly what {@link
   * #atSpeed()} depends on being true.
   */
  private double goalRpm = 0.0;

  /**
   * Builds the floor roller around a hardware implementation.
   *
   * @param io the hardware layer: real, simulated, or empty for log replay
   */
  public FloorRoller(FloorRollerIO io) {
    this.io = io;
    // IMPLEMENTATION (was TODO: "Start stateTimer and register a FaultReporter disconnect check on
    // inputs.connected.")
    //
    // Registered with FaultReporter since M14 step 2. Worth having even though a dead floor roller
    // is fairly obvious from the driver's seat: what it catches is the roller unplugged in the pit
    // and never plugged back in, which otherwise presents as "the intake stopped working" and sends
    // people to the wrong mechanism. An error that does not block scoring.
    FaultReporter.getInstance()
        .registerDeviceDisconnect(
            FaultReporter.canDevice(
                "Floor roller motor", Constants.Can.kFloorRollerId, Constants.Can.kRioBus),
            FaultReporter.Impact.REPORT_ONLY,
            () -> inputs.connected);
    //
    // The timer is started, not restarted, and it is started here rather than on the first loop so
    // that FloorRoller/StateTime is a real number from the very first log frame instead of reading
    // zero until something happens to transition.
    stateTimer.start();
  }

  /**
   * Runs one iteration of the state machine. Called by the command scheduler every 20 ms.
   *
   * <p><b>Outputs:</b> refreshes and logs inputs; may change {@code systemState}; commands the
   * hardware; pushes the power budget scalar down to the IO layer.
   */
  @Override
  public void periodic() {
    // IMPLEMENTATION (was TODO: "The standard five-step periodic: updateInputs, processInputs,
    // tunable-gain push, transition (with onEnter/onExit and a stateTimer reset), applyState, then
    // log the state enum. Also push PowerBudget.getInstance().getScalar(Priority.LOW) into
    // io.setCurrentLimitScalar.")

    // ---- 1. Inputs. The key is "FloorRoller/Inputs" rather than bare "FloorRoller" so that raw
    // hardware inputs and computed outputs stay in separate subtrees, per the log layout in plan
    // section 9.1. Everything below writes under "FloorRoller/" directly.
    io.updateInputs(inputs);
    Logger.processInputs("FloorRoller/Inputs", inputs);

    // Gains are pushed before the state machine runs, so a gain edited on the dashboard takes
    // effect on the same loop the operator sees the response change. hasChanged() reports true on
    // its first call for a given id, which means the compiled-in gains are pushed once on the first
    // loop after boot -- there is deliberately no separate startup path that could drift out of
    // sync with the retune path.
    //
    // Only kP, kS and kV are watched, because those are the only gains this mechanism has. See
    // FloorRollerIOTalonFX.currentGains() for why a roller deliberately runs with no integrator.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> {
          var gains = new Slot0Configs();
          gains.kP = values[0];
          gains.kS = values[1];
          gains.kV = values[2];
          io.setGains(gains);
        },
        Constants.FloorRoller.kKp,
        Constants.FloorRoller.kKs,
        Constants.FloorRoller.kKv);

    // LOW: the floor roller is the first mechanism in the robot to give up amps when the battery
    // sags, alongside the intake. Losing roller torque costs you a slow pickup; losing drivetrain
    // or flywheel current costs you the match. The IO layer deadbands this, so calling it every
    // loop costs nothing on the bus.
    io.setCurrentLimitScalar(PowerBudget.getInstance().getScalar(PowerBudget.Priority.LOW));

    // ---- 2 and 3. Transition, with the timer restarted on every change so that StateTime measures
    // this state rather than the match.
    //
    // There are no onEnter/onExit hooks, unlike the general contract in plan section 2.1. Those
    // exist to reset motion profiles, zero integrators and latch setpoints on entry; this mechanism
    // has no profile, no integrator and a setpoint that applyState() recomputes from a tunable
    // every loop anyway. Empty hooks would be pure ceremony, and ceremony is what stops students
    // reading the hooks that do matter on the turret and the intake pivot.
    SystemState nextState = handleStateTransition();
    if (nextState != systemState) {
      stateTimer.restart();
      systemState = nextState;
    }

    // ---- 4. Drive the hardware for whatever state we are now in.
    applyState();

    // ---- 5. Telemetry. The three state keys are the standard FSM triple from plan section 2.1;
    // graphed as a timeline alongside the other mechanisms they turn a match log into a story.
    Logger.recordOutput("FloorRoller/WantedState", wantedState.toString());
    Logger.recordOutput("FloorRoller/SystemState", systemState.toString());
    Logger.recordOutput("FloorRoller/StateTime", stateTimer.get());
    Logger.recordOutput("FloorRoller/GoalRpm", goalRpm);

    // The error is logged, not just the two series it is made of, because the thing worth spotting
    // in a post-match review is a roller that never reaches its goal while carrying a piece -- and
    // that shows up as a persistent offset here long before anybody notices pickups getting slower.
    Logger.recordOutput("FloorRoller/ErrorRpm", goalRpm - inputs.velocityRpm);
  }

  /**
   * Decides which state the machine should be in this loop.
   *
   * @return the state to run this iteration
   */
  private SystemState handleStateTransition() {
    // IMPLEMENTATION (was TODO: "A direct WantedState -> SystemState mapping. No guards are needed
    // for a roller.")
    //
    // No guards, and specifically no FAULTED state -- which is a real decision rather than an
    // omission. Every other mechanism can fault because it can fail to reach a goal it must reach:
    // a flywheel that never spins up cannot shoot, a pivot that never homes does not know where it
    // is. A floor roller stalling against game pieces is NORMAL: it leads the indexer, so it
    // stalls against the stopped indexer every time a requested shot waits on the gate (plan
    // section 2.4). A stall-timeout fault here would fire on every such wait and train the drive
    // team to ignore the alert list.
    //
    // The cross-mechanism illegal combinations this roller participates in live in exactly one
    // place, Superstructure.applyState (plan section 2.1). Duplicating any of them here would give
    // the robot two opinions about the same interlock, and the one that loses is always the one
    // nobody remembered to update.
    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case FEED -> SystemState.FEEDING;
      case EJECT -> SystemState.EJECTING;
      case CHARACTERIZE -> SystemState.CHARACTERIZING;
    };
  }

  /**
   * Dispatches to the hardware for the current state.
   *
   * <p><b>Outputs:</b> commands the roller for this loop, and updates {@code goalRpm}.
   */
  private void applyState() {
    // IMPLEMENTATION (was TODO: "switch (systemState) dispatching io.setVelocity with kIntakeRpm /
    // kFeedRpm / kEjectRpm, io.stop() for IDLE, or io.setOpenLoop(characterizationVolts) for
    // CHARACTERIZING.") INTAKING and kIntakeRpm were deleted in M10b step 8: the floor roller
    // runs only to feed the indexer, never while intaking (plan section 2.4).
    //
    // The tunables are read every loop rather than latched on entry, so that a speed edited under
    // /Tuning takes effect while the roller is already running. That is the whole workflow for
    // finding a feed speed: hold the button, turn the number up until the indexer never waits.
    switch (systemState) {
      case IDLE -> {
        // A genuine stop rather than a closed-loop hold at zero. Unlike the flywheel, there is
        // nothing to be gained by keeping this wheel moving between uses, and a roller held at zero
        // by a PID loop clamps any piece that is resting against it.
        goalRpm = 0.0;
        io.stop();
      }
      case FEEDING -> commandVelocity(Constants.FloorRoller.kFeedRpm.get());
      case EJECTING -> commandVelocity(Constants.FloorRoller.kEjectRpm.get());
      case CHARACTERIZING -> {
        // goalRpm is cleared because there is no closed-loop goal in open loop. Leaving the last
        // goal standing would make atSpeed() report against a setpoint nobody is commanding, and a
        // characterization sweep would then look like a successful spin-up to the feed gate.
        goalRpm = 0.0;
        io.setOpenLoop(characterizationVolts);
      }
    }
  }

  /**
   * Commands a closed-loop speed and records it as the goal.
   *
   * @param rpm the speed to command, in RPM; negative runs the roller backwards
   *     <p><b>Outputs:</b> writes {@code goalRpm} and commands the roller.
   */
  private void commandVelocity(double rpm) {
    // Both writes live here so the recorded goal and the commanded speed are set from the same
    // value at the same instant. Splitting them across the switch arms above is how atSpeed() ends
    // up comparing the measured speed against a goal the hardware was never asked for.
    goalRpm = rpm;
    io.setVelocity(rpm);
  }

  /**
   * Requests a new roller behavior.
   *
   * @param wanted the behavior to request
   *     <p><b>Outputs:</b> sets the wanted state.
   */
  public void setWantedState(WantedState wanted) {
    // IMPLEMENTATION (was TODO: "wantedState = wanted.")
    wantedState = wanted;
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
    // IMPLEMENTATION (was TODO: "characterizationVolts = volts.")
    characterizationVolts = volts;
  }

  /**
   * Whether the roller has reached its commanded speed.
   *
   * @return true when within a tunable tolerance of the goal RPM; the superstructure's feed gate
   *     uses this
   */
  @AutoLogOutput(key = "FloorRoller/AtSpeed")
  public boolean atSpeed() {
    // IMPLEMENTATION (was TODO: "Math.abs(inputs.velocityRpm - goalRpm) < a tunable tolerance.
    // Track the goal as a field.")
    //
    // The second term is "goalRpm != 0.0", NOT the flywheel's "goalRpm > 0.0". Copying that test
    // across would have made atSpeed() permanently false while EJECTING, because the eject goal is
    // a negative RPM -- and any future feed gate built on it would then hang waiting for a roller
    // that was already running perfectly. What both versions are really guarding is the start of a
    // match, when velocity and goal are both zero and the error is therefore exactly zero: a
    // stopped roller must not report itself at speed.
    return Math.abs(inputs.velocityRpm - goalRpm) < Constants.FloorRoller.kToleranceRpm.get()
        && goalRpm != 0.0;
  }

  /**
   * The live roller speed.
   *
   * @return roller speed, in RPM
   */
  public double getVelocityRpm() {
    // IMPLEMENTATION (was TODO: "return inputs.velocityRpm.")
    return inputs.velocityRpm;
  }

  /**
   * The current state of the floor-roller machine.
   *
   * <p>Added beyond the plan's method list, matching the accessor {@code Shooter} added for the
   * same reason: it lets a bench test or a simulation checkpoint assert that the machine walked the
   * states it should have, rather than that being readable only off an AdvantageScope graph.
   *
   * @return the state the machine is running this loop
   */
  public SystemState getSystemState() {
    return systemState;
  }
}
