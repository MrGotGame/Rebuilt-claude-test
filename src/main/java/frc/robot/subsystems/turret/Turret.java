package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.shooting.TurretUnwrapper;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PowerBudget;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
 *
 * <p><b>One invariant governs this whole class:</b> every position ever handed to the IO layer goes
 * through {@link #commandPosition(double, double)}, which clamps into the travel limits and records
 * what it commanded as {@code Turret/CommandedPositionRad}. That single choke point is what makes
 * M5's turret checkpoint -- "the commanded position never leaves [kMinAngleRad, kMaxAngleRad]" --
 * something a log can actually prove, rather than a property scattered across six handlers and
 * hoped for.
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

  /** Why the turret could not aim where it was asked to. Logged; drives the hold behaviour. */
  private enum OutOfRangeReason {
    /** No branch of the requested direction lies inside the travel limits. */
    UNREACHABLE,
    /** A branch exists, but reaching it would sweep the turret through the deployed intake. */
    INTAKE_BLOCKED
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();

  /**
   * Disconnect and fault alerts.
   *
   * <p>These are plain WPILib {@link Alert}s rather than {@code FaultReporter} registrations,
   * following the precedent already set in {@code PhoenixUtil.tryUntilOk}, {@code Shooter} and
   * {@code FloorRoller}: {@code FaultReporter} is still a skeleton whose {@code getInstance()}
   * returns null, so registering with it in this constructor would throw. Swap all four for {@code
   * FaultReporter.getInstance().registerDeviceDisconnect(...)} once that class exists; the alert
   * text is already written to match what it would produce.
   */
  private final Alert motorDisconnectedAlert =
      new Alert("Turret motor disconnected", AlertType.kError);

  /**
   * A warning rather than an error, because the turret survives losing this encoder.
   *
   * <p>Homing falls back to driving into a hard stop, which works but is slow and puts a load on
   * the mechanism, and the position is only as good as that one stop. Worth knowing about before a
   * match; not worth telling the drive team the robot is broken.
   */
  private final Alert absEncoderDisconnectedAlert =
      new Alert(
          "Turret absolute encoder disconnected -- homing will fall back to finding a hard stop",
          AlertType.kWarning);

  /** Fires when homing gave up. See {@link SystemState#FAULTED}. */
  private final Alert homingFailedAlert =
      new Alert(
          "Turret failed to home within the timeout. Check the absolute encoder wiring, and check"
              + " that the turret is free to move.",
          AlertType.kError);

  /**
   * Fires when the absolute encoder reading does not uniquely identify a turret position.
   *
   * <p>See {@link #resolveAbsoluteSeed(double)} for what this means and why it is only a warning.
   */
  private final Alert absoluteSeedAmbiguousAlert =
      new Alert(
          "Turret absolute encoder reading is ambiguous: the travel range is wider than one"
              + " encoder turn. Re-home with the turret near centre, or verify the angle by eye.",
          AlertType.kWarning);

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
   * The velocity feedforward last handed to the IO layer, in radians per second.
   *
   * <p>{@link #atGoal()} compares the measured velocity against this rather than against zero. A
   * turret correctly counter-rotating at 6 rad/s to hold an aim while the chassis spins is <i>on
   * target</i>; measuring it against zero would call that a miss and block every shot taken while
   * turning, which is the entire point of the mechanism.
   */
  private double lastCommandedVelocityRadPerSec = 0.0;

  /**
   * The unwrap solution computed by the most recent AIM transition.
   *
   * <p>Solved once per loop in {@link #handleStateTransition()} and consumed by the handlers, so
   * the state decision and the position commanded cannot disagree. Solving it twice -- once to
   * decide, once to command -- would let a branch flip between the two calls and command a target
   * the state machine never approved, which is precisely the mid-sweep reversal the latch in {@link
   * #onEnter(SystemState)} exists to prevent.
   */
  private TurretUnwrapper.Result lastUnwrapResult =
      new TurretUnwrapper.Result(0.0, true, false, 0.0);

  /** The robot-relative aim direction behind {@link #lastUnwrapResult}, radians. */
  private double lastRobotRelativeGoalRad = 0.0;

  /**
   * The branch latched on entry to {@link SystemState#UNWRAPPING}, radians.
   *
   * <p>Latched precisely because a 300-degree sweep passes through positions where a different
   * branch is momentarily nearer, and re-solving mid-sweep would reverse the turret halfway.
   */
  private double latchedBranchTargetRad = 0.0;

  /** Why the turret is currently out of range; meaningful only in that state. */
  private OutOfRangeReason outOfRangeReason = OutOfRangeReason.UNREACHABLE;

  /** Count of OUT_OF_RANGE entries this power cycle, logged so the state is findable in a log. */
  private int outOfRangeEventCount = 0;

  /** True once the homing routine has established a trustworthy angle this attempt. */
  private boolean homingComplete = false;

  /** The angle {@link #onExit(SystemState)} will seed once homing completes, radians. */
  private double homingSeedRad = 0.0;

  /**
   * Set by an explicit {@link WantedState#HOME} request, consumed by the next transition.
   *
   * <p>This is what makes a latched {@link SystemState#FAULTED} recoverable. Asking to home is an
   * unambiguous "try again", and it has to work even when {@code wantedState} is already HOME --
   * which a plain comparison against {@code wantedState} could not detect.
   */
  private boolean homeRequested = false;

  /**
   * Builds the turret around a hardware implementation.
   *
   * @param io the hardware layer: real, simulated, or empty for log replay
   */
  public Turret(TurretIO io) {
    this.io = io;
    // IMPLEMENTATION (was TODO: "Start stateTimer, register FaultReporter checks for motorConnected
    // and absEncoderConnected, and push the initial soft limits with io.setSoftLimits(
    // kMinAngleRad.get() + kLimitMarginRad.get(), kMaxAngleRad.get() - kLimitMarginRad.get()).")
    //
    // The four checks exist, as the Alert fields above; they are driven from periodic() rather than
    // registered, because FaultReporter is not implemented yet. See the field documentation.
    //
    // The timer is started, not restarted, and here rather than on the first loop, so that
    // Turret/StateTime is a real number from the very first log frame.
    stateTimer.start();

    // The limits are pushed here as well as from periodic()'s ifChanged, and that is not
    // redundant: TurretIOSim's hard-stop clamp defaults to +-pi, which is far narrower than the
    // real travel, so a simulated turret would spend its first loop believing it was already
    // against a stop. Pushing once at construction means both IO implementations agree about the
    // travel before the first updateInputs runs.
    io.setSoftLimits(softLimitMinRad(), softLimitMaxRad());
  }

  /**
   * Installs the interlock that keeps the turret out of the deployed intake.
   *
   * @param blocked supplier returning true while the intake occupies the conflict arc; the
   *     superstructure injects {@code () -> !intake.isStowed()}
   *     <p><b>Outputs:</b> replaces the stored guard.
   */
  public void setIntakeArcGuard(BooleanSupplier blocked) {
    // IMPLEMENTATION (was TODO: "this.intakeArcBlocked = blocked. Stored, and evaluated fresh each
    // loop.")
    //
    // A supplier rather than a boolean, so the turret reads the intake's CURRENT state every loop
    // instead of a snapshot taken whenever somebody last remembered to push one. An interlock built
    // on a pushed value fails silently the first time a code path forgets to push.
    this.intakeArcBlocked = blocked;
  }

  /**
   * Runs one iteration of the state machine. Called by the command scheduler every 20 ms.
   *
   * <p><b>Outputs:</b> refreshes and logs inputs; publishes the turret angle into {@code
   * RobotState}; may change {@code systemState}; commands the hardware.
   */
  @Override
  public void periodic() {
    // IMPLEMENTATION (was TODO: "The standard five-step periodic, plus three turret-specific
    // pieces: (a) IMMEDIATELY after Logger.processInputs, call
    // RobotState.getInstance().addTurretObservation(Timer.getFPGATimestamp(), inputs.positionRad).
    // Do it first so the buffer sample is as close to the measurement time as possible -- every
    // millisecond of delay here becomes camera transform error. (b) Re-apply soft limits and gains
    // through LoggedTunableNumber.ifChanged. (c) Log the unwrap solution and the field-relative
    // angle the turret is actually pointing.")

    // ---- 1. Inputs. The key is "Turret/Inputs" rather than bare "Turret" so raw hardware inputs
    // and computed outputs stay in separate subtrees, per the log layout in plan section 9.1.
    io.updateInputs(inputs);
    Logger.processInputs("Turret/Inputs", inputs);

    // ---- 1a. The buffer write, FIRST, before any of the work below.
    //
    // M8's turret-mounted camera resolves a tag observation by asking RobotState what angle the
    // turret was at when the shutter opened. That answer is only as good as this buffer, and this
    // buffer is only as fresh as this call. Every line of work placed between processInputs and
    // here would add its own execution time to the gap between when the angle was measured and the
    // timestamp it is filed under -- and at a 12 rad/s cruise, one millisecond of skew is about
    // 0.7 degrees of bearing error on every frame captured while slewing. That error is invisible:
    // it looks like a slightly worse pose estimate, never like a turret bug.
    RobotState.getInstance().addTurretObservation(Timer.getFPGATimestamp(), inputs.positionRad);

    // ---- 1b. Tunables. Both pushes happen before the state machine runs, so a value edited on the
    // dashboard takes effect on the same loop the operator sees the response change. hasChanged()
    // reports true on its first call for a given id, so the compiled-in values are pushed once on
    // the first loop after boot -- there is deliberately no separate startup path that could drift
    // out of sync with the retune path.
    //
    // The limits get their own watcher id. Limits and gains are pushed to different configuration
    // groups on the Talon and there is no reason for a gain edit to cost a soft-limit write on a
    // bus carrying nine devices.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> io.setSoftLimits(softLimitMinRad(), softLimitMaxRad()),
        Constants.Turret.kMinAngleRad,
        Constants.Turret.kMaxAngleRad,
        Constants.Turret.kLimitMarginRad);

    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        values -> {
          var gains = new Slot0Configs();
          gains.kP = values[0];
          gains.kD = values[1];
          gains.kS = values[2];
          gains.kV = values[3];
          gains.kA = values[4];
          io.setGains(gains);
        },
        Constants.Turret.kKp,
        Constants.Turret.kKd,
        Constants.Turret.kKs,
        Constants.Turret.kKv,
        Constants.Turret.kKa);

    // HIGH, alongside the flywheel. A turret that cannot move cannot score, so it gives up amps
    // only after the intake and the rollers have -- and still before the drivetrain, because a
    // robot that cannot drive cannot even leave the field. The IO layer deadbands this, so calling
    // it every loop costs nothing on the bus.
    io.setCurrentLimitScalar(PowerBudget.getInstance().getScalar(PowerBudget.Priority.HIGH));

    // ---- 2 and 3. Transition. onExit runs before the timer restarts and onEnter after, exactly as
    // in plan section 2.1: onExit needs the state it is leaving to still be current, and onEnter
    // needs a timer that already reads zero for the state it is entering.
    SystemState nextState = handleStateTransition();
    if (nextState != systemState) {
      onExit(systemState);
      stateTimer.restart();
      onEnter(nextState);
      systemState = nextState;
    }

    // ---- 4. Drive the hardware for whatever state we are now in.
    applyState();

    // ---- 5. Telemetry. The three state keys are the standard FSM triple from plan section 2.1.
    Logger.recordOutput("Turret/WantedState", wantedState.toString());
    Logger.recordOutput("Turret/SystemState", systemState.toString());
    Logger.recordOutput("Turret/StateTime", stateTimer.get());

    // ---- 5c. The turret-specific logs.
    //
    // CommandedPositionRad together with the two soft-limit series IS the M5 turret checkpoint:
    // drop all three on one graph, run CharacterizationCommands.turretUnwrapDemo, and the claim
    // "the commanded position never leaves the travel limits" is either visibly true or visibly
    // false. Logging the limits rather than remembering them matters because they are tunable --
    // a checkpoint plotted against a hard-coded band would silently stop meaning anything the
    // first time somebody retunes a limit.
    Logger.recordOutput("Turret/CommandedPositionRad", lastCommandedPositionRad);
    Logger.recordOutput("Turret/CommandedVelocityRadPerSec", lastCommandedVelocityRadPerSec);
    Logger.recordOutput("Turret/SoftLimitMinRad", softLimitMinRad());
    Logger.recordOutput("Turret/SoftLimitMaxRad", softLimitMaxRad());

    // The unwrap solution, in full. Branch selection is the single hardest thing in this subsystem
    // to reason about after the fact, and these five series are what turn "the turret went the
    // wrong way" into an answerable question. Fresh says whether they describe this loop or the
    // last loop the turret was aiming.
    Logger.recordOutput("Turret/Unwrap/Fresh", wantedState == WantedState.AIM);
    Logger.recordOutput("Turret/Unwrap/TargetRad", lastUnwrapResult.targetPositionRad());
    Logger.recordOutput("Turret/Unwrap/Reachable", lastUnwrapResult.reachable());
    Logger.recordOutput("Turret/Unwrap/LongWayAround", lastUnwrapResult.requiresLongWayAround());
    Logger.recordOutput("Turret/Unwrap/ErrorRad", lastUnwrapResult.errorRad());
    Logger.recordOutput("Turret/Unwrap/RobotRelativeGoalRad", lastRobotRelativeGoalRad);

    // The aim goal and where the turret actually points, both in the FIELD frame, so they can be
    // graphed against each other directly. Comparing a field-frame goal with a robot-frame position
    // is the mistake this pair exists to make impossible: they differ by the chassis heading, and
    // during a rotate that difference is exactly the quantity the feedforward is cancelling.
    Logger.recordOutput("Turret/GoalFieldAngleRad", goalFieldAngle.getRadians());
    Logger.recordOutput("Turret/FieldAngleRad", getFieldAngle().getRadians());
    Logger.recordOutput("Turret/IsHomed", isHomed);
    Logger.recordOutput("Turret/IntakeArcBlocked", intakeArcBlocked.getAsBoolean());
    Logger.recordOutput("Turret/OutOfRangeReason", outOfRangeReason.toString());
    Logger.recordOutput("Turret/OutOfRangeEvents", outOfRangeEventCount);

    motorDisconnectedAlert.set(!inputs.motorConnected);
    absEncoderDisconnectedAlert.set(!inputs.absEncoderConnected);
    homingFailedAlert.set(systemState == SystemState.FAULTED);
  }

  /**
   * Decides which state the machine should be in this loop.
   *
   * @return the state to run this iteration
   */
  private SystemState handleStateTransition() {
    // IMPLEMENTATION (was TODO: "HOME -> HOMING, or FAULTED on timeout. !isHomed -> HOMING.
    // MANUAL -> MANUAL. CHARACTERIZE -> CHARACTERIZING. STOW -> STOWING. AIM -> compute the unwrap
    // result, then: !result.reachable() -> OUT_OF_RANGE; result.requiresLongWayAround() ->
    // UNWRAPPING, until |error| is small, then AIMING; otherwise -> AIMING. If
    // intakeArcBlocked.getAsBoolean() and the chosen path passes through the conflict arc, stay in
    // OUT_OF_RANGE rather than sweeping through the intake.")

    // An explicit home request always wins, and clears a latched fault. This is the operator's
    // deliberate retry: something was wrong, they fixed it, they are asking again. Consuming the
    // flag here rather than testing wantedState is what lets a second HOME request restart homing
    // when wantedState was already HOME -- otherwise a fault raised while homing would be
    // unrecoverable without first selecting some other state.
    //
    // Note that re-requesting HOME while already HOMING does NOT restart the state timer, because
    // the returned state equals the current one and no transition fires. That is the wanted
    // behaviour: a driver leaning on the button must not be able to postpone the homing timeout
    // indefinitely, which would turn a mechanical jam into a turret that never faults.
    if (homeRequested) {
      homeRequested = false;
      homingComplete = false;
      return SystemState.HOMING;
    }

    // A fault latches, exactly as the flywheel's does, and for the same reason: without the latch
    // the machine would leave FAULTED the moment the timeout condition happened not to hold, and
    // the alert would flicker instead of telling the pit crew there is something to fix.
    //
    // It latches only against the states that NEED a trustworthy zero, though. IDLE, MANUAL and
    // CHARACTERIZE are all deliberate changes of intent that work fine without one -- and MANUAL in
    // particular is how somebody jogs a stuck turret free after homing has failed. Trapping the
    // mechanism in FAULTED would remove the one tool available for fixing the fault.
    if (systemState == SystemState.FAULTED && requiresHoming(wantedState)) {
      return SystemState.FAULTED;
    }

    // Nothing that depends on knowing where the turret is may run before it knows where it is. A
    // turret aiming on an unseeded position is not merely inaccurate: its soft limits are wrong by
    // the same offset, so it will drive into a hard stop believing it is well inside the travel.
    //
    // homingComplete rather than isHomed, because isHomed is set by onExit(HOMING), which cannot
    // run until this method has already decided to leave HOMING. Testing isHomed alone would be a
    // deadlock -- homing would succeed and the machine would never notice.
    if (requiresHoming(wantedState) && !isHomed && !homingComplete) {
      if (systemState == SystemState.HOMING
          && stateTimer.hasElapsed(Constants.Turret.kHomingTimeoutSec.get())) {
        return SystemState.FAULTED;
      }
      return SystemState.HOMING;
    }

    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case MANUAL -> SystemState.MANUAL;
      case CHARACTERIZE -> SystemState.CHARACTERIZING;

        // Homing is finished by the time control reaches here, so HOME has nothing left to ask for.
        // Resolving to IDLE rather than holding HOMING is what stops a turret that was homed by an
        // operator button from sitting in HOMING for the rest of the match with its state timer
        // running, which would make the homing timeout fire spuriously on the next real attempt.
      case HOME -> SystemState.IDLE;
      case STOW -> SystemState.STOWING;
      case AIM -> aimingState();
    };
  }

  /**
   * Resolves which state to run while the turret has been asked to aim.
   *
   * <p>Split out of {@link #handleStateTransition()} because these guards are the whole substance
   * of this FSM, and nesting them inside a switch arm makes them unreadable.
   *
   * @return {@link SystemState#AIMING}, {@link SystemState#UNWRAPPING} or {@link
   *     SystemState#OUT_OF_RANGE}
   */
  private SystemState aimingState() {
    lastUnwrapResult = solveAim();

    // No branch of the requested direction lies inside the travel. Report it rather than clamping
    // and pretending: the superstructure reads isOutOfRange() and rotates the whole chassis, which
    // is the only thing that can actually fix this. A silent clamp would leave the turret aimed
    // somewhere plausible-looking, the shot solver happy, and the piece landing nowhere near the
    // goal -- a failure that looks like bad shot tables rather than like a range problem.
    if (!lastUnwrapResult.reachable()) {
      outOfRangeReason = OutOfRangeReason.UNREACHABLE;
      return SystemState.OUT_OF_RANGE;
    }

    // The mechanical interlock. The target may be perfectly reachable and the PATH to it still
    // illegal, because getting there sweeps through the space the deployed intake occupies. This is
    // the one cross-mechanism rule the turret enforces locally rather than leaving to the
    // superstructure, because only the turret knows which branch it is about to sweep along.
    //
    // Evaluated against the sweep the turret is ACTUALLY about to make -- from where it is now to
    // where it would go -- not against the endpoints of the goal, because a turret already standing
    // inside the arc must not be told its next move is fine merely because the destination is.
    double sweepTarget =
        systemState == SystemState.UNWRAPPING
            ? latchedBranchTargetRad
            : lastUnwrapResult.targetPositionRad();
    // passesThroughArcWrapped, not passesThroughArc. The conflict arc is a piece of PHYSICAL space,
    // and this turret's travel is wider than a full circle, so the unwrapper can park it at -180
    // degrees to point where an operator would say +180 degrees. A raw interval test against an arc
    // configured as 150-210 degrees does not see that branch at all, and the turret sweeps through
    // the deployed intake reporting no conflict. Found by the M10 interlock test.
    if (intakeArcBlocked.getAsBoolean()
        && TurretUnwrapper.passesThroughArcWrapped(
            inputs.positionRad,
            sweepTarget,
            Constants.Turret.kIntakeConflictMinRad.get(),
            Constants.Turret.kIntakeConflictMaxRad.get())) {
      outOfRangeReason = OutOfRangeReason.INTAKE_BLOCKED;
      return SystemState.OUT_OF_RANGE;
    }

    // Already sweeping the long way: stay in UNWRAPPING against the LATCHED branch until the turret
    // arrives. Re-testing requiresLongWayAround() here instead would drop out of UNWRAPPING the
    // moment the remaining distance fell under pi -- about halfway through the sweep -- and the
    // chassis-cancelling feedforward would switch on mid-profile and fight it.
    if (systemState == SystemState.UNWRAPPING) {
      boolean arrived =
          Math.abs(latchedBranchTargetRad - inputs.positionRad)
              < Constants.Turret.kUnwrapExitToleranceRad.get();
      return arrived ? SystemState.AIMING : SystemState.UNWRAPPING;
    }

    // More than half a turn to travel. The sweep gets its own state so the feedforward can be
    // suppressed for its duration and so it is visible as a distinct band on the state timeline --
    // a 300-degree unwrap takes long enough that "why did the turret stop tracking for half a
    // second" is a question somebody will ask about a match log.
    if (lastUnwrapResult.requiresLongWayAround()) {
      return SystemState.UNWRAPPING;
    }

    return SystemState.AIMING;
  }

  /**
   * Runs one-shot work when a state is entered.
   *
   * @param state the state being entered
   *     <p><b>Outputs:</b> resets homing bookkeeping, latches an unwrap branch, or logs an event.
   */
  private void onEnter(SystemState state) {
    // IMPLEMENTATION (was TODO: "HOMING -> isHomed = false. UNWRAPPING -> latch the chosen branch
    // so it cannot flip mid-sweep, which would leave the turret oscillating between two
    // equally-distant solutions. OUT_OF_RANGE -> log a one-shot event so it is findable in the
    // match log.")
    switch (state) {
      case HOMING -> {
        // Clearing isHomed on ENTRY, not on exit, means that anything reading isHomed during the
        // homing routine gets the honest answer. A turret mid-home does not know where it is.
        isHomed = false;
        homingComplete = false;
        homingSeedRad = 0.0;
      }

      case UNWRAPPING -> {
        // THE latch. TurretUnwrapper's hysteresis already resists chatter between two nearly
        // equidistant branches, but during a 300-degree sweep the turret physically passes through
        // the region where the OTHER branch becomes nearer -- so a re-solve halfway across would
        // reverse it, and it would reverse again on the way back. The mechanism oscillates until
        // something breaks. Latching on entry makes the branch a decision taken once.
        latchedBranchTargetRad = lastUnwrapResult.targetPositionRad();
      }

      case OUT_OF_RANGE -> {
        // A counter rather than a bare boolean. OUT_OF_RANGE can be entered and left many times
        // over a match as the robot drives around, and what a post-match review needs is "how often
        // did this happen" -- a rising step plot answers that at a glance, where a boolean that
        // blinks does not. Paired with Turret/OutOfRangeReason it also says which kind. The value
        // is published every loop from periodic(), so the series is continuous rather than a
        // scattering of samples at transition instants.
        outOfRangeEventCount++;
      }

      default -> {
        // Every other state's entry work is either nothing or is recomputed every loop by its
        // handler. Listed explicitly so that adding a state to the enum without deciding what its
        // entry means is a visible omission here rather than a silent fallthrough.
      }
    }
  }

  /**
   * Runs one-shot work when a state is left.
   *
   * @param state the state being left
   *     <p><b>Outputs:</b> on a successful homing exit, seeds position and marks homed.
   */
  private void onExit(SystemState state) {
    // IMPLEMENTATION (was TODO: "HOMING -> io.seedPosition(inputs.absolutePositionRad) and
    // isHomed = true.")
    //
    // DEVIATION FROM THE TODO, deliberate, in two places.
    //
    // 1. The seed is guarded by homingComplete. HOMING is left for two very different reasons:
    //    because homing succeeded, or because it timed out into FAULTED (or the operator switched
    //    to MANUAL mid-home). Seeding unconditionally would mark the turret homed on the strength
    //    of a homing attempt that failed, and the whole point of FAULTED is that the position is
    //    NOT to be trusted. That single missing guard would convert a loud, correct fault into a
    //    turret that silently aims 200 degrees wrong.
    //
    // 2. The angle seeded is homingSeedRad, not inputs.absolutePositionRad directly. The homing
    //    routine has two paths -- the absolute encoder and the hard stop -- and only it knows which
    //    one produced the answer. See homing() and resolveAbsoluteSeed().
    if (state == SystemState.HOMING && homingComplete) {
      io.seedPosition(homingSeedRad);
      isHomed = true;
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
        // IDLE stops rather than holding a closed-loop position. Unlike the flywheel, where idling
        // buys back spin-up time, there is nothing to gain from powering a stationary turret -- the
        // Talon's Brake neutral mode holds it, and a closed-loop hold against a mechanical
        // obstruction would sit at the stator limit indefinitely.
      case IDLE -> commandNeutral();
      case STOWING -> stowing();
      case AIMING -> aiming();
      case UNWRAPPING -> unwrapping();
      case OUT_OF_RANGE -> outOfRange();
      case MANUAL -> manual();
      case HOMING -> homing();
      case CHARACTERIZING -> characterizing();

        // FAULTED stops, for the same reason the flywheel does. If homing failed, the position is
        // not trustworthy, and the very last thing a turret with unknown position should be doing
        // is running a closed-loop profile toward a setpoint computed from it.
      case FAULTED -> commandNeutral();
    }
  }

  /** Tracks the goal azimuth with chassis-rotation-cancelling feedforward. */
  private void aiming() {
    // IMPLEMENTATION (was TODO: "double robotRelGoal = goalFieldAngle.getRadians() -
    // RobotState.getInstance().getRotation().getRadians(); TurretUnwrapper.Result r =
    // TurretUnwrapper.solve(inputs.positionRad, robotRelGoal, kMinAngleRad.get(),
    // kMaxAngleRad.get(), kUnwrapHysteresisRad.get()); double ff =
    // goalFieldAngularVelocityRadPerSec -
    // RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
    // io.setPositionSetpoint(r.targetPositionRad(), ff); lastCommandedPositionRad =
    // r.targetPositionRad().")
    //
    // The solve itself has moved to solveAim(), called once per loop from handleStateTransition --
    // see the note on lastUnwrapResult for why it must not be solved twice.
    //
    // THE SIGN, derived rather than asserted, because a sign error here is the failure mode plan
    // section 6.4 singles out. The turret's position is measured in the ROBOT frame, so where it
    // points in the field is
    //
    //     theta_field = theta_turret_robot + heading_chassis
    //
    // Differentiate: omega_field = omega_turret_robot + omega_chassis. The aim direction must sweep
    // at omega_goal_field, so the motor must actually turn at
    //
    //     omega_turret_robot = omega_goal_field - omega_chassis
    //
    // which is the expression below. Concretely: with a stationary target and the driver spinning
    // the robot at +6 rad/s, the turret must run at -6 rad/s simply to stay still in the field. Get
    // this sign backwards and the feedforward adds to the error instead of cancelling it, so the
    // turret tracks at double rate the wrong way -- and it only shows up while ROTATING, which is
    // why it survives every stationary bench test. The test is a slow in-place spin with
    // Turret/Inputs/positionRad plotted against the estimated pose's rotation: they should move
    // equally and oppositely.
    //
    // Note this is the FUSED chassis rate, not the raw Pigeon2 rate, so a vision glitch that jerks
    // the fused heading also jerks this feedforward. The pose-jump rejection in plan section 7.4 is
    // what keeps that from reaching the turret.
    double feedforwardRadPerSec =
        goalFieldAngularVelocityRadPerSec
            - RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
    commandPosition(lastUnwrapResult.targetPositionRad(), feedforwardRadPerSec);
  }

  /** Sweeps the long way around to a latched branch target. */
  private void unwrapping() {
    // IMPLEMENTATION (was TODO: "Same as aiming() but with the velocity feedforward FORCED TO ZERO
    // and using the latched branch target. During a 300-degree sweep the chassis-cancelling
    // feedforward is meaningless and actively fights the profile.")
    //
    // Both halves matter. The latched target stops the branch flipping halfway across (see
    // onEnter). The zero feedforward is about the profile: Motion Magic is already commanding the
    // turret at cruise velocity toward a target most of a turn away, and adding a term meant to
    // cancel a few rad/s of chassis rotation on top of that does not improve the sweep -- it
    // perturbs a motion profile that is not tracking a moving target at all yet.
    //
    // The cost is honest and bounded: for the duration of the sweep the turret is not compensating
    // for chassis rotation, so it arrives with whatever error the chassis accumulated meanwhile.
    // AIMING corrects that within a few loops, and no shot is permitted during UNWRAPPING because
    // atGoal() is false outside AIMING.
    commandPosition(latchedBranchTargetRad, 0.0);
  }

  /** Holds the closest reachable angle while the drivetrain rotates to help. */
  private void outOfRange() {
    // IMPLEMENTATION (was TODO: "Clamp the desired robot-relative angle into [min, max] and hold
    // there, so the turret is already as close as it can be the moment the chassis finishes
    // rotating.")
    //
    // The two reasons for being here call for genuinely different behaviour, which is why
    // outOfRangeReason exists:
    //
    // UNREACHABLE -- hold the closest legal angle to the goal. The superstructure is rotating the
    // chassis to fix this, and every degree the turret has already taken up is a degree the
    // drivetrain does not have to rotate through before the shot becomes available.
    // TurretUnwrapper already computed that angle: when it reports unreachable it hands back the
    // wrapped direction clamped into the travel limits.
    //
    // INTAKE_BLOCKED -- HOLD POSITION. The goal is reachable; the path is not. Creeping toward the
    // goal here would mean creeping into the arc the interlock exists to keep the turret out of,
    // and clamping into the travel limits would not stop it, because the intake conflict arc sits
    // in the MIDDLE of the travel rather than at an end. Rotating the chassis does not help either
    // -- retracting the intake does -- so there is nothing to get a head start on.
    if (outOfRangeReason == OutOfRangeReason.INTAKE_BLOCKED) {
      commandPosition(inputs.positionRad, 0.0);
      return;
    }
    commandPosition(lastUnwrapResult.targetPositionRad(), 0.0);
  }

  /** Holds the stow angle. */
  private void stowing() {
    // IMPLEMENTATION (was TODO: "io.setPositionSetpoint(Constants.Turret.kStowAngleRad.get(),
    // 0.0).")
    //
    // Read from the tunable every loop rather than latched on entry, so that stow can be adjusted
    // on the dashboard while the turret is sitting in it -- which is how the number gets found.
    // Zero feedforward because stow is a robot-relative angle: it is meant to rotate WITH the
    // chassis, not to hold still in the field, so there is nothing to cancel.
    commandPosition(Constants.Turret.kStowAngleRad.get(), 0.0);
  }

  /** Establishes a trustworthy zero, preferring the absolute encoder. */
  private void homing() {
    // IMPLEMENTATION (was TODO: "If inputs.absEncoderConnected, seed immediately from
    // absolutePositionRad and finish in one loop -- that is the whole point of an output-shaft
    // encoder. Only if it is disconnected, fall back to driving open loop toward one hard stop at a
    // low duty cycle, watching for the velocity to settle, then seeding. Time out to FAULTED.")
    //
    // Nothing is seeded here. This routine decides WHAT to seed and WHEN it is known; the seed
    // itself happens in onExit(HOMING), so that a homing attempt which times out into FAULTED
    // cannot leave a half-finished seed behind.
    if (homingComplete) {
      // Already know the answer; wait for the transition out. Holding neutral rather than
      // continuing to drive matters on the stall path, where "continuing to drive" means pressing
      // the mechanism into a hard stop for another loop.
      commandNeutral();
      return;
    }

    if (inputs.absEncoderConnected) {
      // The fast path, and the reason a ThroughBore is fitted to the gearbox output at all: the
      // turret's angle is known the instant power is applied, with no motion and no stall.
      homingSeedRad = resolveAbsoluteSeed(inputs.absolutePositionRad);
      homingComplete = true;
      commandNeutral();
      return;
    }

    // The fallback. Drive gently toward one hard stop and wait for the mechanism to stop moving.
    // kHomingVolts is negative by convention, so the stop found is the one at kMinAngleRad, and
    // that is the angle seeded -- the two are tied together in the constant's documentation
    // because flipping one without the other zeroes the turret a full travel-width out.
    commandOpenLoop(Constants.Turret.kHomingVolts.get());

    // The grace period is what makes the velocity test mean anything. On the first homing loop the
    // turret is stationary, so "velocity below threshold" is trivially true and the routine would
    // seed the current position as the hard stop -- succeeding instantly and being wrong every
    // time. The mechanism has to be given time to actually start moving first.
    // The current term is not decoration. Velocity alone cannot distinguish the hard stop from a
    // motor that is not turning because it is dead: both read zero. Accepting the second seeds a
    // zero at an imaginary position, isHomed() goes true, and the next closed-loop command drives
    // the turret at full authority from a reference that does not exist.
    if (stateTimer.hasElapsed(Constants.Turret.kHomingMinMotionSec.get())
        && Math.abs(inputs.velocityRadPerSec)
            < Constants.Turret.kHomingVelocityThresholdRadPerSec.get()
        && inputs.statorCurrentAmps >= Constants.Turret.kHomingCurrentAmps.get()) {
      homingSeedRad = Constants.Turret.kMinAngleRad.get();
      homingComplete = true;
      commandNeutral();
    }
  }

  /** Applies the operator's manual nudge, still respecting the limits. */
  private void manual() {
    // IMPLEMENTATION (was TODO: "io.setOpenLoop(manualVolts), clamped, and refusing to drive
    // further past a soft limit.")
    //
    // Two separate protections, because they fail in different ways. The magnitude clamp bounds how
    // hard a mis-scaled joystick axis can drive the mechanism. The limit guard bounds WHERE it can
    // drive it -- and note that it only blocks motion in the offending direction, so a turret that
    // has somehow ended up past a limit can still be jogged back out. A guard that blocked both
    // directions would strand it there.
    double volts =
        MathUtil.clamp(
            manualVolts,
            -Constants.Turret.kManualMaxVolts.get(),
            Constants.Turret.kManualMaxVolts.get());

    // This assumes positive voltage increases position, which is what Constants.Turret.kInverted
    // exists to make true. TODO: VERIFY on the first power-up, by jogging with kTuningMode on and
    // watching Turret/Inputs/positionRad -- if it falls, flip kInverted rather than negating here,
    // or the closed-loop path and this one will disagree about which way is forward.
    if ((inputs.positionRad >= softLimitMaxRad() && volts > 0.0)
        || (inputs.positionRad <= softLimitMinRad() && volts < 0.0)) {
      volts = 0.0;
    }

    // MANUAL deliberately does not require homing -- it is the state an operator uses to jog a
    // jammed turret free, which is exactly when homing has failed. The guard above is therefore
    // only as good as whatever seed the position currently carries, and before homing that is
    // TurretIOTalonFX's boot seed from the absolute encoder: approximately right, not trustworthy.
    // Somebody has to be watching the mechanism. That is the trade, stated plainly.
    commandOpenLoop(volts);
  }

  /** Applies the externally-commanded characterization voltage. */
  private void characterizing() {
    // IMPLEMENTATION (was TODO: "io.setOpenLoop(characterizationVolts).")
    //
    // Unclamped and unguarded, unlike manual(). CharacterizationCommands owns the ramp, and the
    // firmware soft limits in TurretIOTalonFX are what stop a sweep at the end of travel. This path
    // is reachable only with Constants.kTuningMode on, per plan section 2.1.
    commandOpenLoop(characterizationVolts);
  }

  /**
   * Commands a closed-loop position, clamped into the travel limits.
   *
   * <p>The single choke point through which every position command in this class passes. Clamping
   * here rather than in each handler is what makes the invariant in the class documentation
   * checkable: {@code Turret/CommandedPositionRad} is written by this method and nowhere else, so a
   * log proving that series stays inside the limits proves it for every state at once.
   *
   * <p>The clamp is a last line of defence, not the primary one -- {@link TurretUnwrapper} is
   * already required to return an in-range branch or report itself unreachable. It exists for the
   * cases the unwrapper does not cover: a stow angle mistyped on the dashboard, a limit retuned
   * narrower than the turret's current position, a future handler added without reading this note.
   *
   * @param positionRad the desired continuous position, in radians
   * @param velocityFeedforwardRadPerSec the feedforward to apply, in radians per second
   *     <p><b>Outputs:</b> writes {@code lastCommandedPositionRad} and {@code
   *     lastCommandedVelocityRadPerSec}, and commands the turret motor.
   */
  private void commandPosition(double positionRad, double velocityFeedforwardRadPerSec) {
    double clamped =
        TurretUnwrapper.clampIntoRange(positionRad, softLimitMinRad(), softLimitMaxRad());
    lastCommandedPositionRad = clamped;
    lastCommandedVelocityRadPerSec = velocityFeedforwardRadPerSec;
    io.setPositionSetpoint(clamped, velocityFeedforwardRadPerSec);
  }

  /**
   * Commands an open-loop voltage and clears the closed-loop bookkeeping.
   *
   * @param volts the voltage to apply
   *     <p><b>Outputs:</b> clears {@code lastCommandedVelocityRadPerSec} and drives the motor open
   *     loop.
   */
  private void commandOpenLoop(double volts) {
    // The commanded feedforward is cleared because there is no closed-loop command in open loop.
    // Leaving the last one standing would leave atGoal()'s velocity term measured against a rate
    // nobody is asking for -- the same stale-goal trap FloorRoller and Shooter guard against by
    // zeroing goalRpm when they go open loop.
    lastCommandedVelocityRadPerSec = 0.0;
    io.setOpenLoop(volts);
  }

  /**
   * Releases the motor, leaving the configured Brake neutral mode to hold the turret.
   *
   * <p><b>Outputs:</b> clears {@code lastCommandedVelocityRadPerSec} and stops the motor.
   */
  private void commandNeutral() {
    lastCommandedVelocityRadPerSec = 0.0;
    io.stop();
  }

  /**
   * Converts the current field-relative goal into an in-range robot-relative turret position.
   *
   * <p>Also records the robot-relative goal for logging, so a log shows both the field-frame
   * request and the robot-frame question actually asked of the unwrapper.
   *
   * @return the unwrap solution for this loop; never null
   */
  private TurretUnwrapper.Result solveAim() {
    // Field frame to robot frame. The chassis heading is SUBTRACTED because the turret's position
    // is measured relative to the robot: a goal due north while the robot faces north is a turret
    // angle of zero, and the same goal while the robot faces east is a turret angle of +90 degrees.
    // This is the positional counterpart of the rate subtraction in aiming(), and it is the same
    // relationship differentiated -- if these two ever disagree in sign, the feedforward fights the
    // position loop instead of assisting it.
    lastRobotRelativeGoalRad =
        goalFieldAngle.getRadians() - RobotState.getInstance().getRotation().getRadians();

    // DEVIATION FROM THE PLAN'S SNIPPET, deliberate: the unwrapper is handed the MARGINED limits,
    // not the raw kMinAngleRad/kMaxAngleRad the plan's code sample passes. The Talon's firmware
    // guard is set at the margined values, so an unwrapper working in the wider window could pick a
    // branch sitting in the margin -- a branch it calls reachable and the firmware refuses to
    // reach. The turret would then sit at the soft limit, never arriving, with every diagnostic
    // reporting a perfectly reachable goal. Giving both the same window makes "reachable" mean the
    // same thing in software and in firmware.
    //
    // This also strengthens the M5 checkpoint rather than weakening it: a commanded position inside
    // the margined band is inside [kMinAngleRad, kMaxAngleRad] a fortiori.
    return TurretUnwrapper.solve(
        inputs.positionRad,
        lastRobotRelativeGoalRad,
        softLimitMinRad(),
        softLimitMaxRad(),
        Constants.Turret.kUnwrapHysteresisRad.get());
  }

  /**
   * Turns a wrapped absolute-encoder reading into a position inside the travel limits.
   *
   * <p>A single-turn encoder on the output shaft reports a direction, not a position. When the
   * travel range is wider than one full turn -- which it is by default, at roughly 400 degrees --
   * two distinct true positions produce the same reading, and no amount of arithmetic can tell them
   * apart. This method picks the branch nearest turret centre and raises {@link
   * #absoluteSeedAmbiguousAlert} when the choice was genuinely ambiguous.
   *
   * <p>Centre is preferred because a turret is overwhelmingly likely to be parked near its stow
   * angle when the robot is powered on, and because centre leaves the most travel in both
   * directions if the guess is wrong. It is a mitigation, not a fix: <b>the real fix is mechanical
   * or geometric</b> -- keep the total travel under 360 degrees, or gear the encoder so one turret
   * traverse is less than one encoder turn. Until then, a turret parked beyond +/-180 degrees at
   * power-on can be seeded a full turn wrong, and the firmware soft limits cannot catch it because
   * they are computed from the same wrong position.
   *
   * <p>TODO: MEASURE the real mechanical travel and revisit. If it exceeds 360 degrees at the
   * encoder, raise this with the mechanical team before the turret is first driven under Motion
   * Magic.
   *
   * @param wrappedRad the absolute encoder reading, in radians, wrapped into {@code [-pi, pi)}
   * @return the best in-range position consistent with that reading, in radians
   *     <p><b>Outputs:</b> sets the ambiguity alert.
   */
  private double resolveAbsoluteSeed(double wrappedRad) {
    double min = softLimitMinRad();
    double max = softLimitMaxRad();

    // Only k in {-1, 0, +1} needs checking: the travel is well under three full turns, so no
    // further branch can land inside it. Enumerating rather than solving keeps the ambiguity count
    // available, which is the part that has to be reported.
    int inRangeCount = 0;
    double best = Double.NaN;
    for (int k = -1; k <= 1; k++) {
      double candidate = wrappedRad + 2.0 * Math.PI * k;
      if (candidate < min || candidate > max) {
        continue;
      }
      inRangeCount++;
      if (Double.isNaN(best) || Math.abs(candidate) < Math.abs(best)) {
        best = candidate;
      }
    }

    absoluteSeedAmbiguousAlert.set(inRangeCount > 1);

    // No branch in range at all. Physically this should be impossible -- the turret cannot be
    // somewhere it cannot travel -- so reaching here means the encoder offset is wrong or the
    // limits are. Clamping keeps the seed inside the window the firmware guard is about to enforce,
    // which is the least-harmful answer available; the ambiguity alert does not fire, but the
    // turret will visibly refuse to reach half its travel, which is a louder symptom than any
    // alert.
    if (Double.isNaN(best)) {
      return TurretUnwrapper.clampIntoRange(wrappedRad, min, max);
    }
    return best;
  }

  /**
   * Whether a requested behavior needs a trustworthy zero before it may run.
   *
   * @param wanted the behavior being requested
   * @return true if the turret must be homed first
   */
  private static boolean requiresHoming(WantedState wanted) {
    // AIM and STOW both command closed-loop positions computed from the measured angle, so both are
    // meaningless -- and dangerous -- on an unseeded position. HOME obviously needs homing. IDLE,
    // MANUAL and CHARACTERIZE are all open-loop or neutral and are deliberately left available
    // without a zero, because they are the states somebody needs when homing itself has failed.
    return wanted == WantedState.AIM || wanted == WantedState.STOW || wanted == WantedState.HOME;
  }

  /**
   * The lower travel limit, held back from the hard stop by the safety margin.
   *
   * @return lower soft limit, in radians
   */
  private static double softLimitMinRad() {
    return Constants.Turret.kMinAngleRad.get() + Constants.Turret.kLimitMarginRad.get();
  }

  /**
   * The upper travel limit, held back from the hard stop by the safety margin.
   *
   * @return upper soft limit, in radians
   */
  private static double softLimitMaxRad() {
    return Constants.Turret.kMaxAngleRad.get() - Constants.Turret.kLimitMarginRad.get();
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
    // IMPLEMENTATION (was TODO: "Store both, then wantedState = WantedState.AIM.")
    //
    // No clamping and no validation here, deliberately, and unlike Shooter.setGoal which clamps its
    // RPM. A turret angle has no invalid values -- every direction is a legitimate thing to ask
    // for, and whether it is REACHABLE is a question about the travel limits that only the
    // unwrapper can answer, one loop later, with the current position in hand. Clamping the
    // request here would destroy the distinction between "aim there" and "that is out of range",
    // which is the distinction isOutOfRange() exists to report.
    goalFieldAngle = fieldAngle;
    goalFieldAngularVelocityRadPerSec = fieldAngularVelocityRadPerSec;
    wantedState = WantedState.AIM;
  }

  /**
   * Requests a new turret behavior.
   *
   * @param wanted the behavior to request
   *     <p><b>Outputs:</b> sets the wanted state; a HOME request also clears the homed flag.
   */
  public void setWantedState(WantedState wanted) {
    // IMPLEMENTATION (was TODO: "Assign. If wanted == HOME, also set isHomed = false.")
    wantedState = wanted;
    if (wanted == WantedState.HOME) {
      // The flag is what makes a re-home work from FAULTED, and works even when wantedState was
      // already HOME. See the field documentation on homeRequested.
      isHomed = false;
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
    // IMPLEMENTATION (was TODO: "manualVolts = volts.")
    //
    // Stored raw; manual() does the clamping and the limit guard. Keeping the raw value means the
    // operator's actual request is what gets logged if this is ever traced, rather than the clamped
    // version, which is the difference between diagnosing a mis-scaled axis and not.
    manualVolts = volts;
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
   * The live robot-relative turret angle.
   *
   * @return continuous turret position in the robot frame, in radians
   */
  @AutoLogOutput(key = "Turret/PositionRad")
  public double getPositionRad() {
    // IMPLEMENTATION (was TODO: "return inputs.positionRad.")
    return inputs.positionRad;
  }

  /**
   * Where the turret is pointing in field coordinates.
   *
   * @return field-relative turret azimuth; the AdvantageScope 3D turret component and the vision
   *     transform both key off this
   */
  @AutoLogOutput(key = "Turret/FieldAngle")
  public Rotation2d getFieldAngle() {
    // IMPLEMENTATION (was TODO:
    // "Rotation2d.fromRadians(inputs.positionRad).plus(RobotState.getInstance().getRotation()).")
    //
    // Note this composes the INSTANTANEOUS pose, which is right for the 3D component overlay and
    // wrong for a vision transform evaluated at a camera timestamp. The vision path must instead
    // pair RobotState.getTurretAngleAt(t) with getPoseAt(t) -- which is the reason periodic()
    // bothers to fill the turret buffer at all. Using this accessor there would silently introduce
    // exactly the latency the buffer exists to remove.
    return Rotation2d.fromRadians(inputs.positionRad).plus(RobotState.getInstance().getRotation());
  }

  /**
   * How fast the turret is slewing.
   *
   * @return turret angular velocity, in radians per second; vision uses this to reject frames
   *     captured mid-slew
   */
  public double getVelocityRadPerSec() {
    // IMPLEMENTATION (was TODO: "return inputs.velocityRadPerSec.")
    return inputs.velocityRadPerSec;
  }

  /**
   * Whether the turret is genuinely on target.
   *
   * @return true when both the position error and the velocity error are inside tolerance
   */
  @AutoLogOutput(key = "Turret/AtGoal")
  public boolean atGoal() {
    // IMPLEMENTATION (was TODO: "Math.abs(inputs.closedLoopErrorRad) < kToleranceRad.get() AND
    // Math.abs(inputs.velocityRadPerSec - commandedFeedforward) <
    // kVelocityToleranceRadPerSec.get(). BOTH terms matter. A turret sweeping through the correct
    // angle at speed is not on target, and a position-only check will happily fire mid-sweep.")
    //
    // Three terms, not two. The extra one is a DELIBERATE ADDITION to the TODO and it is not
    // optional: Motion Magic's closed-loop error is measured against the profile's INSTANTANEOUS
    // REFERENCE, not against the goal. A profile being tracked well reports an error near zero for
    // the entire sweep, so the TODO's first term alone is true almost from the moment the request
    // is issued. Comparing the measured position against the position actually commanded is what
    // makes the first term mean "arrived" rather than "tracking nicely".
    //
    // The state guard plays the role Shooter's `goalRpm > 0` plays: without it, a turret sitting in
    // IDLE or HOMING reports at-goal because closedLoopErrorRad and the commanded feedforward are
    // both zero and the turret is not moving. Perfect agreement between three quantities that are
    // all zero is not a shot solution. AIMING only -- UNWRAPPING is mid-sweep by definition, and
    // being parked at the stow angle is not being on target.
    if (systemState != SystemState.AIMING) {
      return false;
    }

    return Math.abs(inputs.closedLoopErrorRad) < Constants.Turret.kToleranceRad.get()
        && Math.abs(lastCommandedPositionRad - inputs.positionRad)
            < Constants.Turret.kToleranceRad.get()
        && Math.abs(inputs.velocityRadPerSec - lastCommandedVelocityRadPerSec)
            < Constants.Turret.kVelocityToleranceRadPerSec.get();
  }

  /**
   * Whether the requested azimuth is unreachable.
   *
   * @return true while in {@link SystemState#OUT_OF_RANGE}; the superstructure reads this to decide
   *     to rotate the chassis
   */
  @AutoLogOutput(key = "Turret/IsOutOfRange")
  public boolean isOutOfRange() {
    // IMPLEMENTATION (was TODO: "return systemState == SystemState.OUT_OF_RANGE.")
    //
    // Derived from the state rather than from a separate flag, so it cannot disagree with the state
    // timeline in the log. Two flags describing one fact is how a superstructure ends up rotating
    // the chassis to help a turret that stopped needing help several loops ago.
    return systemState == SystemState.OUT_OF_RANGE;
  }

  /**
   * Whether the turret position is trustworthy.
   *
   * @return true once homed or seeded from the absolute encoder
   */
  public boolean isHomed() {
    // IMPLEMENTATION (was TODO: "return isHomed.")
    return isHomed;
  }

  /**
   * The current state of the turret machine.
   *
   * <p>Added beyond the plan's method list so that the M5 turret checkpoint -- watching the machine
   * walk HOMING, AIMING, UNWRAPPING, OUT_OF_RANGE -- can be asserted from a test rather than only
   * read off a graph. The flywheel's {@code getSystemState} exists for the same reason.
   *
   * @return the state the machine is running this loop
   */
  public SystemState getSystemState() {
    return systemState;
  }

  /**
   * The position most recently commanded to the hardware, after clamping.
   *
   * <p>Added beyond the plan's method list. This is the quantity M5's turret checkpoint is about,
   * and exposing it lets {@code CharacterizationCommands.turretUnwrapDemo} assert the invariant
   * directly instead of re-deriving it from a log after the fact.
   *
   * @return the clamped commanded position, in radians
   */
  public double getCommandedPositionRad() {
    return lastCommandedPositionRad;
  }

  /**
   * The turret velocity, consumed by the characterization routines.
   *
   * @return turret angular velocity, in radians per second
   */
  public double getCharacterizationVelocityRadPerSec() {
    // IMPLEMENTATION (was TODO: "return inputs.velocityRadPerSec.")
    //
    // Radians per second, unlike the flywheel's characterization accessor, which deliberately
    // returns Phoenix native units because that is what its kV is expressed in. The turret has the
    // same underlying mismatch -- Slot0's kV is volts per ROTATION per second (see
    // TurretIOTalonFX.setPositionSetpoint) -- but this accessor keeps the radians the rest of the
    // class speaks in, because the name says so and a silent unit switch behind an identical
    // signature is worse than a conversion the caller can see.
    //
    // TODO: VERIFY when the turret ramp is written in M6. Whichever routine fits kS and kV must
    // divide this by 2*pi before the fit, or the resulting kV will be 2*pi too small and the
    // feedforward will do almost nothing -- which looks exactly like a feedforward that was never
    // wired up.
    return inputs.velocityRadPerSec;
  }
}
