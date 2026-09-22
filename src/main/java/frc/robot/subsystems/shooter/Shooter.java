package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PowerBudget;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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

  /**
   * Disconnect alerts, one per motor.
   *
   * <p>These are plain WPILib {@link Alert}s rather than {@code FaultReporter} registrations,
   * following the precedent already set in {@code PhoenixUtil.tryUntilOk}: {@code FaultReporter} is
   * still a skeleton whose {@code getInstance()} returns null, so registering with it here would
   * throw in the {@code Shooter} constructor. Swap both of these for {@code
   * FaultReporter.getInstance().registerDeviceDisconnect(...)} once that class is implemented; the
   * alert text is already written to match what it would produce.
   *
   * <p>They are separate on purpose, for the reason {@link ShooterIO} logs both motors separately:
   * a lost follower and a lost leader are different failures. A dead leader stops the robot
   * shooting, which is obvious. A dead follower leaves the robot shooting weakly at half the torque
   * while the leader quietly draws double current, which is not.
   */
  private final Alert leaderDisconnectedAlert =
      new Alert("Shooter leader motor disconnected", AlertType.kError);

  private final Alert followerDisconnectedAlert =
      new Alert("Shooter follower motor disconnected", AlertType.kError);

  /** Fires when the flywheel gave up trying to reach its goal. See {@link SystemState#FAULTED}. */
  private final Alert spinUpFailedAlert =
      new Alert(
          "Shooter failed to reach speed within the spin-up timeout. Check for a jam, a dead"
              + " follower, or an RPM goal above what the battery can support.",
          AlertType.kWarning);

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
    // IMPLEMENTATION (was TODO: "Start stateTimer, and register FaultReporter checks for BOTH
    // motors separately -- leaderConnected and followerConnected are independent failures with very
    // different symptoms.")
    //
    // The two checks exist, as the Alert fields above; they are driven from periodic() rather than
    // registered, because FaultReporter is not implemented yet. See the field documentation.
    this.io = io;
    stateTimer.start();
  }

  /**
   * Runs one iteration of the state machine. Called by the command scheduler every 20 ms.
   *
   * <p><b>Outputs:</b> refreshes and logs inputs; updates the acceleration estimate; may change
   * {@code systemState}; commands the hardware.
   */
  @Override
  public void periodic() {
    // IMPLEMENTATION (was TODO: "The standard five-step periodic, plus: differentiate
    // inputs.velocityRpm against lastVelocityRpm over the loop period and push it through
    // accelFilter into accelRpmPerSec; push gains via ifChanged; push the PowerBudget scalar at
    // Priority.HIGH; and log goalRpm, the error, and both motors' currents.")

    // ---- 1. Inputs. The key is "Shooter/Inputs" rather than bare "Shooter" so that raw hardware
    // inputs and computed outputs stay in separate subtrees, per the log layout in plan section
    // 9.1. Everything below writes under "Shooter/" directly.
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Inputs", inputs);

    updateAccelerationEstimate();

    // Gains are pushed before the state machine runs, so a gain edited on the dashboard takes
    // effect on the same loop the operator sees the response change. hasChanged() reports true on
    // its first call for a given id, which means the compiled-in gains are pushed once on the first
    // loop after boot -- there is deliberately no separate startup path that could drift out of
    // sync with the retune path.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> {
          var gains = new Slot0Configs();
          gains.kP = values[0];
          gains.kI = values[1];
          gains.kD = values[2];
          gains.kS = values[3];
          gains.kV = values[4];
          gains.kA = values[5];
          io.setGains(gains);
        },
        Constants.Shooter.kKp,
        Constants.Shooter.kKi,
        Constants.Shooter.kKd,
        Constants.Shooter.kKs,
        Constants.Shooter.kKv,
        Constants.Shooter.kKa);

    // HIGH, not CRITICAL: if the battery is sagging, the flywheel gives up amps before the
    // drivetrain does, because a robot that cannot drive cannot even leave the field. It gives them
    // up after the intake and the rollers, because a robot that cannot shoot cannot score. The IO
    // layer deadbands this, so calling it every loop costs nothing on the bus.
    io.setCurrentLimitScalar(PowerBudget.getInstance().getScalar(PowerBudget.Priority.HIGH));

    // ---- 2 and 3. Transition, with the timer restarted on every change so that
    // handleStateTransition can express "I have been in this state too long".
    SystemState nextState = handleStateTransition();
    if (nextState != systemState) {
      stateTimer.restart();
      systemState = nextState;
    }

    // ---- 4. Drive the hardware for whatever state we are now in.
    applyState();

    // ---- 5. Telemetry. The three state keys are the standard FSM triple from plan section 2.1;
    // graphed as a timeline alongside the other mechanisms they turn a match log into a story.
    Logger.recordOutput("Shooter/WantedState", wantedState.toString());
    Logger.recordOutput("Shooter/SystemState", systemState.toString());
    Logger.recordOutput("Shooter/StateTime", stateTimer.get());
    Logger.recordOutput("Shooter/GoalRpm", goalRpm);
    Logger.recordOutput("Shooter/ErrorRpm", goalRpm - inputs.velocityRpm);
    Logger.recordOutput("Shooter/AccelRpmPerSec", accelRpmPerSec);

    // Shooter/WillBeAtSpeed is written here rather than by @AutoLogOutput. See the note on
    // willBeAtSpeedIn for why the annotation had to come off it. The horizon used is the one the
    // superstructure's readiness gate uses (plan section 6.5), so this series and Shooter/AtSpeed
    // can be graphed against each other -- the gap between them is exactly the cadence that
    // predictive readiness buys.
    Logger.recordOutput(
        "Shooter/WillBeAtSpeed", willBeAtSpeedIn(Constants.getLoopPeriodSec() * 2.0));

    // Per-motor currents are already in the input table. The sum is logged as well because it is
    // the number that correlates against a battery sag in Power/TotalCurrent, and adding two series
    // by hand in AdvantageScope during a post-match review is exactly the friction that stops
    // anyone from doing it.
    Logger.recordOutput(
        "Shooter/TotalStatorCurrentAmps",
        inputs.leaderStatorCurrentAmps + inputs.followerStatorCurrentAmps);

    leaderDisconnectedAlert.set(!inputs.leaderConnected);
    followerDisconnectedAlert.set(!inputs.followerConnected);
    spinUpFailedAlert.set(systemState == SystemState.FAULTED);
  }

  /**
   * Updates the filtered flywheel acceleration that {@link #willBeAtSpeedIn(double)} projects with.
   *
   * <p><b>Outputs:</b> writes {@code accelRpmPerSec} and {@code lastVelocityRpm}.
   */
  private void updateAccelerationEstimate() {
    // A five-sample moving average over a numerical derivative. The derivative of a quantised
    // encoder velocity is extremely noisy -- single-loop values swing by hundreds of RPM/s on a
    // wheel that is holding steady -- and willBeAtSpeedIn multiplies this by a horizon, so
    // unfiltered noise would make the readiness gate chatter on and off at 50 Hz and fire the feed
    // at random.
    //
    // Five samples is 100 ms of lag, which is short against the ~1 s spin-up it is measuring.
    double deltaRpm = inputs.velocityRpm - lastVelocityRpm;
    accelRpmPerSec = accelFilter.calculate(deltaRpm / Constants.getLoopPeriodSec());
    lastVelocityRpm = inputs.velocityRpm;
  }

  /**
   * Decides which state the machine should be in this loop.
   *
   * @return the state to run this iteration
   */
  private SystemState handleStateTransition() {
    // IMPLEMENTATION (was TODO: "CHARACTERIZE -> CHARACTERIZING. IDLE -> IDLE. COAST -> COASTING.
    // SPIN_UP or SHOOT -> SPINNING_UP until atSpeed(), then AT_SPEED. If SPINNING_UP exceeds
    // kSpinUpTimeoutSec -> FAULTED.")
    return switch (wantedState) {
      case CHARACTERIZE -> SystemState.CHARACTERIZING;
      case IDLE -> SystemState.IDLE;
      case COAST -> SystemState.COASTING;
      case SPIN_UP, SHOOT -> spinningState();
    };
  }

  /**
   * Resolves the state to run while the flywheel has been asked to spin up or shoot.
   *
   * <p>Split out of {@link #handleStateTransition()} only because the guards below are the whole
   * substance of this FSM and nesting them inside a switch arm makes them hard to read.
   *
   * @return {@link SystemState#IDLE}, {@link SystemState#SPINNING_UP}, {@link SystemState#AT_SPEED}
   *     or {@link SystemState#FAULTED}
   */
  private SystemState spinningState() {
    // A fault latches until somebody stops asking for a shot. Without the latch the machine would
    // leave FAULTED the instant the wheel happened to drift through the tolerance band, and the
    // alert would flicker instead of telling the pit crew there is something to fix. IDLE, COAST
    // and CHARACTERIZE all clear it, because all three are a deliberate change of intent.
    if (systemState == SystemState.FAULTED) {
      return SystemState.FAULTED;
    }

    // A zero goal means "there is nothing to shoot at", not "the mechanism failed". Without this
    // guard, Superstructure calling setGoal(0.0) -- which is what it does when the shooting solver
    // returns an unusable solution -- would drop into SPINNING_UP against an unreachable target and
    // fault three seconds later, taking the scoring state down with it.
    if (goalRpm <= 0.0) {
      return SystemState.IDLE;
    }

    if (atSpeed()) {
      return SystemState.AT_SPEED;
    }

    // The timeout is measured from entry into SPINNING_UP, and the timer restarts on every
    // transition. So recovering from a shot -- AT_SPEED falling back to SPINNING_UP -- gets a fresh
    // full timeout rather than inheriting whatever was left of the original spin-up's budget.
    if (systemState == SystemState.SPINNING_UP
        && stateTimer.hasElapsed(Constants.Shooter.kSpinUpTimeoutSec.get())) {
      return SystemState.FAULTED;
    }

    return SystemState.SPINNING_UP;
  }

  /**
   * Dispatches to the hardware for the current state.
   *
   * <p><b>Outputs:</b> commands the flywheel for this loop.
   */
  private void applyState() {
    // IMPLEMENTATION (was TODO: "IDLE -> io.setVelocity(kIdleRpm.get()); COASTING -> io.stop();
    // SPINNING_UP and AT_SPEED -> io.setVelocity(goalRpm); CHARACTERIZING ->
    // io.setOpenLoop(characterizationVolts); FAULTED -> io.stop().")
    //
    // Note that IDLE is a closed-loop hold at kIdleRpm, not a stop. That is the point of having a
    // separate COASTING state: idling costs a little current continuously and buys back roughly
    // half the spin-up time on every subsequent shot, which is worth about one extra shot per
    // scoring cycle. COAST exists for when the robot genuinely wants the wheel stopped -- the pit,
    // the end of a match, or a brownout the power budget is trying to survive.
    switch (systemState) {
      case IDLE -> io.setVelocity(Constants.Shooter.kIdleRpm.get());
      case COASTING -> io.stop();
      case SPINNING_UP, AT_SPEED -> io.setVelocity(goalRpm);
      case CHARACTERIZING -> io.setOpenLoop(characterizationVolts);

        // FAULTED stops rather than idling. If the wheel could not reach its goal, the reason may
        // well be mechanical, and holding a jammed flywheel against a stator limit for the rest of
        // the match is how a motor is destroyed.
      case FAULTED -> io.stop();
    }
  }

  /**
   * Sets the target flywheel speed. Called every loop from the superstructure with the shooting
   * solver's output.
   *
   * @param rpm desired flywheel speed, in RPM; clamped into the configured maximum
   *     <p><b>Outputs:</b> stores the goal and requests {@link WantedState#SPIN_UP}.
   */
  public void setGoal(double rpm) {
    // IMPLEMENTATION (was TODO: "goalRpm = MathUtil.clamp(rpm, 0.0, Constants.Shooter.kMaxRpm
    // .get()); wantedState = WantedState.SPIN_UP.")
    //
    // The clamp is a last line of defence, not the primary one. ShootingSolver is required to
    // return an unusable ShotParameters rather than a clamped RPM when the shot is out of range --
    // there is a unit test asserting exactly that -- because a silently clamped RPM aims the robot
    // at a target it cannot reach and reports success. The clamp here catches the case where some
    // future caller forgets, and a wheel commanded past kMaxRpm is a real mechanical risk.
    goalRpm = MathUtil.clamp(rpm, 0.0, Constants.Shooter.kMaxRpm.get());
    wantedState = WantedState.SPIN_UP;
  }

  /**
   * Requests a new flywheel behavior.
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
   * Whether the flywheel is at its commanded speed right now.
   *
   * @return true when within {@code kToleranceRpm} of a non-zero goal
   */
  @AutoLogOutput(key = "Shooter/AtSpeed")
  public boolean atSpeed() {
    // IMPLEMENTATION (was TODO: "Math.abs(inputs.velocityRpm - goalRpm) < kToleranceRpm.get() &&
    // goalRpm > 0.")
    //
    // The goalRpm > 0 term is what stops a stopped flywheel from reporting "at speed" at the start
    // of a match, when velocity and goal are both zero and the error is therefore perfectly zero.
    return Math.abs(inputs.velocityRpm - goalRpm) < Constants.Shooter.kToleranceRpm.get()
        && goalRpm > 0.0;
  }

  /**
   * Whether the flywheel will be at speed a short time from now, projecting the current spin-up
   * slope forward.
   *
   * @param horizonSec how far ahead to project, in seconds
   * @return true if the projected speed lands inside the tolerance band
   */
  public boolean willBeAtSpeedIn(double horizonSec) {
    // IMPLEMENTATION (was TODO: "Project inputs.velocityRpm + accelRpmPerSec * horizonSec and test
    // it against the tolerance band around goalRpm.")
    //
    // The skeleton carried an @AutoLogOutput(key = "Shooter/WillBeAtSpeed") annotation here, and it
    // has been REMOVED. That annotation logs the return value of a no-argument method; this is the
    // only annotated method in the project that takes a parameter, and AdvantageKit has no way to
    // decide what horizon to call it with. The key it promised is still written, from periodic(),
    // at the horizon the readiness gate actually uses -- so the documented log layout is unchanged
    // and there is no startup-time reflection hazard.
    //
    // This is gate 3 of the readyToShoot ladder (plan section 6.5). The piece takes roughly 40 ms
    // to travel from the indexer to the wheel, so the question that matters is not "is the wheel at
    // speed" but "will it be at speed when the piece arrives" -- and answering the second one is
    // what removes the one-shot-low that plagues gates built on instantaneous atSpeed().
    //
    // TODO: revisit during M12. A linear projection can overshoot the band while the wheel is
    // accelerating hard, which reads as "not ready" a few loops before it genuinely is ready. That
    // costs a little cadence but never fires early, so it is the safe direction to be wrong in; if
    // the shot-on-the-move cadence needs the last few milliseconds back, the fix is to saturate the
    // projection at the goal rather than to widen the band.
    double projectedRpm = inputs.velocityRpm + accelRpmPerSec * horizonSec;
    return Math.abs(projectedRpm - goalRpm) < Constants.Shooter.kToleranceRpm.get()
        && goalRpm > 0.0;
  }

  /**
   * The live flywheel speed.
   *
   * @return flywheel speed, in RPM
   */
  public double getVelocityRpm() {
    // IMPLEMENTATION (was TODO: "return inputs.velocityRpm.")
    return inputs.velocityRpm;
  }

  /**
   * The commanded flywheel speed.
   *
   * @return goal speed, in RPM
   */
  public double getGoalRpm() {
    // IMPLEMENTATION (was TODO: "return goalRpm.")
    return goalRpm;
  }

  /**
   * The current state of the flywheel machine.
   *
   * <p>Added beyond the plan's method list so that the M4 simulation checkpoint -- watching {@code
   * Shooter/SystemState} walk IDLE, SPINNING_UP, AT_SPEED -- can also be asserted from a test
   * rather than only read off a graph.
   *
   * @return the state the machine is running this loop
   */
  public SystemState getSystemState() {
    return systemState;
  }

  /**
   * The flywheel speed in Phoenix native units, for characterization fits.
   *
   * @return flywheel speed, in rotations per second
   */
  public double getCharacterizationVelocityRotPerSec() {
    // IMPLEMENTATION (was TODO: "return inputs.velocityRpm / 60.0. CharacterizationCommands fits kS
    // and kV in Phoenix native units, not RPM.")
    //
    // Every other accessor on this class is in RPM, which is the unit the whole team talks in. This
    // one is the deliberate exception called out in plan section 5.4, and the name carries the unit
    // so the exception is visible at the call site rather than discovered by a factor of 60.
    return inputs.velocityRpm / 60.0;
  }
}
