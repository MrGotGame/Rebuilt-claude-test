package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.filter.Debouncer;
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
 * Owns the robot's answer to "do I have a game piece," which gates every scoring state, and counts
 * shots by detecting the velocity dip as each piece leaves. Both are debounced.
 *
 * <p>The shot counter is genuinely useful rather than decorative: it gives a per-match "pieces
 * fired" number in the log to compare against the scoring table, which is how you find out whether
 * a bad match was an aiming problem or an intake problem.
 *
 * <p>M10b (plan section 2.4) turns the robot into a hopper robot, and step 6 of its retrofit adds
 * the two things a whole hopper pressing on this wheel needs: jam recovery ({@link
 * SystemState#UNJAMMING}, {@link SystemState#JAMMED}) and a "loaded" reading ({@link #isLoaded()})
 * that says whether a feed is actually pushing pieces. Both are additive -- the piece detection and
 * shot counter above are untouched here, and go in step 8 once nothing calls them.
 */
public class Indexer extends SubsystemBase {
  /** What the operator or superstructure has asked the indexer to do. */
  public enum WantedState {
    IDLE,
    INDEX,
    HOLD,
    FEED,
    EJECT,
    CHARACTERIZE
  }

  /** What the indexer is actually doing right now. */
  public enum SystemState {
    IDLE,
    INDEXING,
    HOLDING,
    FEEDING,
    /** Reversing briefly to clear a jam, then feeding again. Added for M10b step 6. */
    UNJAMMING,
    /** Gave up after {@code kMaxUnjamAttempts} pulses; stopped until the request changes. */
    JAMMED,
    EJECTING,
    CHARACTERIZING
  }

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();
  private final Debouncer hasPieceDebouncer = new Debouncer(0.0, Debouncer.DebounceType.kBoth);
  private final Debouncer shotDebouncer = new Debouncer(0.0, Debouncer.DebounceType.kRising);
  private final Debouncer loadedDebouncer = new Debouncer(0.0, Debouncer.DebounceType.kBoth);
  private final Debouncer jamDebouncer = new Debouncer(0.0, Debouncer.DebounceType.kRising);

  /**
   * Fires while the indexer has given up clearing a jam. See {@link SystemState#JAMMED}.
   *
   * <p>A warning, not an error: the robot can still drive and intake, and the operator's EJECT is
   * the recovery. What the drive team needs to know is why the robot stopped shooting.
   */
  private final Alert jammedAlert =
      new Alert("Indexer jammed and could not clear it; EJECT to clear", AlertType.kWarning);

  /**
   * Disconnect alert for the indexer motor.
   *
   * <p>A plain WPILib {@link Alert} rather than a {@code FaultReporter} registration, following the
   * precedent set in {@code PhoenixUtil.tryUntilOk}, {@code Shooter} and {@code FloorRoller}:
   * {@code FaultReporter} is still a skeleton whose {@code getInstance()} returns null, so
   * registering with it here would throw in this constructor. Swap it for {@code
   * FaultReporter.getInstance().registerDeviceDisconnect(...)} once that class is implemented; the
   * alert text is already written to match what it would produce.
   */
  private final Alert disconnectedAlert = new Alert("Indexer motor disconnected", AlertType.kError);

  /**
   * Fires when a beam break is configured but no sensor was constructed.
   *
   * <p>This is the second {@code FaultReporter} check the skeleton asked for, and it is registered
   * -- as an alert -- only in the sense that it is evaluated every loop; it can only ever fire when
   * {@code Constants.Indexer.kHasBeamBreak} is true, so a robot with no beam break installed never
   * sees it.
   *
   * <p>It is a warning rather than an error because the robot keeps working without the sensor: the
   * current-and-velocity fallback takes over automatically. What the drive team needs to know is
   * that piece detection has quietly dropped to the less reliable path, which otherwise presents
   * much later as "it sometimes thinks it has a piece when it doesn't".
   */
  private final Alert beamBreakDisconnectedAlert =
      new Alert(
          "Indexer beam break not available; falling back to current-spike piece detection",
          AlertType.kWarning);

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private boolean hasPiece = false;
  private int shotCount = 0;
  private double characterizationVolts = 0.0;

  /**
   * The speed currently being commanded, in RPM, or zero when the indexer is stopped, holding, or
   * open loop.
   *
   * <p>Added beyond the skeleton's field list because the piece-detection fallback needs it: the
   * velocity half of that predicate is "the indexer is running slower than it was asked to", which
   * cannot be evaluated without knowing what it was asked for. {@link #applyState()} is the only
   * writer, so the goal and the command issued to the hardware cannot disagree.
   */
  private double goalRpm = 0.0;

  /**
   * True when the last loop in which a piece was detected was a loop spent {@link
   * SystemState#FEEDING}.
   *
   * <p>This is the arming latch for the shot counter, and it is the whole reason the counter cannot
   * be fooled. A shot is "a piece that was here during a feed and then was not", so a piece that
   * vanishes while HOLDING (it fell out), while EJECTING (it was pushed out on purpose), or a FEED
   * commanded on an already-empty indexer must not count. Because the latch is rewritten on every
   * loop in which a piece is present, it self-clears the moment the machine leaves FEEDING with the
   * piece still aboard -- no explicit reset path to forget.
   */
  private boolean sawPieceWhileFeeding = false;

  /**
   * The last value of the current-and-velocity predicate taken while the indexer was actually being
   * driven, replayed by the fallback path whenever the motor is stopped.
   *
   * <p>Raw rather than debounced, and that distinction is load-bearing. Replaying the debounced
   * answer instead would freeze any transition still inside its debounce window at the moment the
   * motor stopped, and one specific sequence then deadlocks the robot: a shot leaves the indexer,
   * the operator releases FEED before the 60 ms window closes, the goal returns to zero with the
   * fall half-finished, and hasPiece stays true forever -- so every later INDEX request resolves
   * straight to HOLDING and the robot refuses to pick anything up for the rest of the match.
   * Replaying the raw sample lets that in-flight transition finish instead.
   */
  private boolean lastLoadedSample = false;

  /** Time since the commanded speed last changed, used to skip the spin-up transient. */
  private final Timer goalChangeTimer = new Timer();

  /** The goal the settling window is being measured against, so a change restarts it. */
  private double lastGoalRpmForDetection = 0.0;

  /**
   * True once an EJECT request has run for its full duration, latching the machine in IDLE until
   * the request changes.
   *
   * <p>Without this latch, the eject timeout oscillates. The timeout is expressed as "if I have
   * been EJECTING longer than {@code kEjectDurationSec}, go to IDLE", the state timer restarts on
   * every transition, and the operator is normally holding an EJECT button -- so the loop after the
   * timeout fires, the machine is IDLE with a fresh timer and {@code wantedState} still EJECT, and
   * it would start another eject immediately. The robot would then buzz between IDLE and EJECTING
   * at 50 Hz for as long as the button was held. Cleared by {@link #setWantedState(WantedState)}
   * whenever the request changes, which is the same "latch until intent changes" rule {@code
   * Shooter} uses for its fault state.
   */
  private boolean ejectComplete = false;

  /**
   * Which detection path produced {@link #hasPiece}, for the log.
   *
   * <p>Worth a string in the log because this mechanism silently has three answers to the same
   * question and their failure modes are completely different. Reading "Latched" on a graph where
   * somebody expected "BeamBreak" is the difference between a five-minute diagnosis and an hour.
   */
  private String pieceDetectionSource = "None";

  /**
   * Whether a feed is pushing against something, debounced. Only ever true while FEEDING, past the
   * spin-up settle window. See {@link #isLoaded()}.
   */
  private boolean loaded = false;

  /** Whether the jam condition has held for {@code kJamDetectSec}, this loop. */
  private boolean jamDetected = false;

  /**
   * Unjam pulses spent since the current FEED request began. Reset by {@link
   * #setWantedState(WantedState)} whenever the request changes, so every new feed gets a full
   * allowance.
   */
  private int unjamAttempts = 0;

  /**
   * Builds the indexer around a hardware implementation.
   *
   * @param io the hardware layer: real, simulated, or empty for log replay
   */
  public Indexer(IndexerIO io) {
    this.io = io;
    // IMPLEMENTATION (was TODO: "Start stateTimer, and register FaultReporter checks for the motor
    // and -- only when Constants.Indexer.kHasBeamBreak -- the beam break.")
    //
    // Both checks exist, as the Alert fields above; they are driven from periodic() rather than
    // registered, because FaultReporter is not implemented yet. See the field documentation. The
    // "only when kHasBeamBreak" condition lives in the alert's set() call rather than in a
    // conditional registration here, which keeps the constructor free of configuration branches
    // that a later reader would have to re-derive.
    //
    // The timer is started, not restarted, and it is started here rather than on the first loop so
    // that Indexer/StateTime is a real number from the very first log frame instead of reading zero
    // until something happens to transition.
    stateTimer.start();
  }

  /**
   * Runs one iteration of the state machine. Called by the command scheduler every 20 ms.
   *
   * <p><b>Outputs:</b> refreshes and logs inputs; updates piece detection; may change {@code
   * systemState}; commands the hardware; pushes the power budget scalar down to the IO layer.
   */
  @Override
  public void periodic() {
    // IMPLEMENTATION (was TODO: "The standard five-step periodic, but call updatePieceDetection()
    // BETWEEN processInputs and the transition step, so handleStateTransition sees a fresh hasPiece
    // this loop rather than last loop's. Log hasPiece and shotCount.")

    // ---- 1. Inputs. The key is "Indexer/Inputs" rather than bare "Indexer" so that raw hardware
    // inputs and computed outputs stay in separate subtrees, per the log layout in plan section
    // 9.1. Everything below writes under "Indexer/" directly.
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/Inputs", inputs);

    // ---- 1.5. THE DOCUMENTED DEVIATION from the five-step contract in plan section 2.1, and the
    // reason it is worth deviating for. Every other mechanism's transition step reads only state
    // the mechanism itself produced, so a one-loop lag is invisible. This one reads a sensor, and
    // the transition it drives -- INDEXING to HOLDING the instant a piece arrives -- is a race
    // against the piece still moving. Running detection after the transition would mean the indexer
    // keeps pulling for one extra loop (20 ms) after the piece is already seated, which at
    // kIndexRpm is far enough to push it past the indexer and into the flywheel uncommanded.
    //
    // The ordering also matters for the shot counter, which reads systemState while it is still
    // last loop's value: a shot is counted against the state the machine was in when the piece was
    // last seen, which is exactly the state that launched it.
    updatePieceDetection();

    // Load and jam detection read this loop's inputs against LAST loop's state, for the reason the
    // shot counter does: the question is what the wheel was doing while it was being driven.
    updateLoadAndJam();

    // Gains are pushed before the state machine runs, so a gain edited on the dashboard takes
    // effect on the same loop the operator sees the response change. hasChanged() reports true on
    // its first call for a given id, which means the compiled-in gains are pushed once on the first
    // loop after boot -- there is deliberately no separate startup path that could drift out of
    // sync with the retune path.
    //
    // Only kP and kV are watched, because those are the only gains this mechanism has. See
    // IndexerIOTalonFX.currentGains() for why there is no integrator and no kS.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> {
          var gains = new Slot0Configs();
          gains.kP = values[0];
          gains.kV = values[1];
          io.setGains(gains);
        },
        Constants.Indexer.kKp,
        Constants.Indexer.kKv);

    // MEDIUM: the indexer gives up amps after the intake and floor rollers (LOW) but before the
    // flywheel and turret (HIGH) and the drivetrain (CRITICAL). A robot whose indexer is throttled
    // can still shoot what it is already holding, just more slowly. The IO layer deadbands this, so
    // calling it every loop costs nothing on the bus.
    io.setCurrentLimitScalar(PowerBudget.getInstance().getScalar(PowerBudget.Priority.MEDIUM));

    // ---- 2 and 3. Transition, with the timer restarted on every change so that
    // handleStateTransition can express "I have been EJECTING too long".
    //
    // There are no onEnter/onExit hooks. Those exist to reset motion profiles, zero integrators and
    // latch setpoints on entry; this mechanism has no profile, no integrator, and a setpoint that
    // applyState() recomputes from a tunable every loop anyway. Empty hooks would be pure ceremony,
    // and ceremony is what stops students reading the hooks that do matter on the turret and the
    // intake pivot.
    SystemState nextState = handleStateTransition();
    if (nextState != systemState) {
      stateTimer.restart();
      systemState = nextState;
    }

    // ---- 4. Drive the hardware for whatever state we are now in.
    applyState();

    // ---- 5. Telemetry. The three state keys are the standard FSM triple from plan section 2.1;
    // graphed as a timeline alongside the other mechanisms they turn a match log into a story.
    Logger.recordOutput("Indexer/WantedState", wantedState.toString());
    Logger.recordOutput("Indexer/SystemState", systemState.toString());
    Logger.recordOutput("Indexer/StateTime", stateTimer.get());
    Logger.recordOutput("Indexer/GoalRpm", goalRpm);

    // The error is logged, not just the two series it is made of, because it IS the velocity half
    // of the piece-detection predicate. Anyone tuning kHasPieceVelocityDropRpm can graph this
    // series against that threshold directly instead of subtracting two traces by hand.
    Logger.recordOutput("Indexer/ErrorRpm", goalRpm - inputs.velocityRpm);
    Logger.recordOutput("Indexer/PieceDetectionSource", pieceDetectionSource);

    // The TODO also asked to log hasPiece and shotCount, and they ARE logged -- by the
    // @AutoLogOutput annotations on hasPiece() and getShotCount(), under exactly the keys the plan
    // specifies, Indexer/HasPiece and Indexer/ShotCount. They are deliberately not written again
    // from here: two writers on one key is at best redundant and at worst a key whose value depends
    // on which writer ran last. This is the same split Shooter makes between its annotated
    // atSpeed() and its hand-logged Shooter/WillBeAtSpeed.

    // Alerts last, so they describe the loop that has just run. The beam-break alert can only fire
    // when the sensor is configured; a robot built without one never sees it.
    Logger.recordOutput("Indexer/UnjamAttempts", unjamAttempts);
    Logger.recordOutput("Indexer/JamDetected", jamDetected);

    disconnectedAlert.set(!inputs.connected);
    jammedAlert.set(systemState == SystemState.JAMMED);
    beamBreakDisconnectedAlert.set(Constants.Indexer.kHasBeamBreak && !inputs.beamBreakConnected);
  }

  /**
   * Recomputes whether a piece is held, and counts shots as pieces leave.
   *
   * <p><b>Outputs:</b> updates {@code hasPiece} and may increment {@code shotCount}.
   */
  private void updatePieceDetection() {
    // IMPLEMENTATION (was TODO: "If kHasBeamBreak and inputs.beamBreakConnected, then hasPiece =
    // hasPieceDebouncer.calculate(inputs.beamBreakTripped). Otherwise use the current-and-velocity
    // fallback: hasPiece = hasPieceDebouncer.calculate(inputs.statorCurrentAmps >
    // kHasPieceCurrentAmps.get() && (goalRpm - inputs.velocityRpm) >
    // kHasPieceVelocityDropRpm.get()). Then detect a shot: while systemState == FEEDING, a falling
    // edge of hasPiece is a shot; pass it through shotDebouncer and increment shotCount. Remember
    // to set each Debouncer's debounce time from the tunables, since they are constructed with 0.0
    // above.")

    // The debouncers are constructed with a 0.0 window in the field initializers above, so this
    // push is what gives them their real ones. ifChanged fires on its first call for a given id, so
    // the window is set on loop one, ahead of the first calculate() below -- a 0.0 window is a
    // pass-through, and a single pass-through loop on a bouncing sensor is a miscounted shot.
    //
    // setDebounceTime does not reset the debouncer's internal timer, so retuning either window
    // mid-match cannot itself produce a spurious edge.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> {
          hasPieceDebouncer.setDebounceTime(values[0]);
          shotDebouncer.setDebounceTime(values[1]);
        },
        Constants.Indexer.kHasPieceDebounceSec,
        Constants.Indexer.kShotDetectDebounceSec);

    if (Constants.Indexer.kHasBeamBreak && inputs.beamBreakConnected) {
      // The good path. A beam break answers the question directly and answers it whether or not the
      // motor is moving, which is what makes everything below a fallback rather than an equal.
      //
      // Debounced kBoth, not raw: a piece tumbling past the emitter chops the beam several times in
      // a few tens of milliseconds, and an undebounced reading would flicker hasPiece and with it
      // the superstructure's scoring gate. kHasPieceDebounceSec exists to outlast that chatter. Its
      // 60 ms default is a starting guess, not a measurement -- set it from a log of
      // Indexer/Inputs/beamBreakTripped while feeding a piece through by hand, long enough to
      // cover the longest burst of chatter and no longer, since every millisecond here is latency
      // on the scoring gate.
      pieceDetectionSource = "BeamBreak";
      hasPiece = hasPieceDebouncer.calculate(inputs.beamBreakTripped);
    } else {
      // The fallback, plus a DEVIATION FROM THE TODO that is the most important decision in this
      // file.
      //
      // The predicate detects a piece by the load it puts on a TURNING wheel, so it is only
      // meaningful while the indexer is being driven. Evaluated with the motor stopped it reads no
      // current and no velocity shortfall, and therefore reports "no piece" -- which is wrong in
      // precisely the state the robot spends most of its time in, HOLDING a piece it just picked
      // up.
      //
      // Written as the TODO literally specifies, that is not a harmless wrong answer but a limit
      // cycle: INDEXING detects the piece, the machine transitions to HOLDING, HOLDING commands
      // zero, the predicate goes false, hasPiece drops, handleStateTransition sends it straight
      // back to INDEXING, the piece loads the wheel again... at 50 Hz, grinding the piece against
      // the flywheel side of the indexer for as long as INDEX is requested.
      //
      // So when there is nothing to measure against, the last measurement taken while there WAS is
      // replayed. See lastLoadedSample for why the raw sample is replayed rather than the debounced
      // answer -- replaying the debounced one deadlocks the robot in a different way.
      //
      // The honest limitation, which is the argument for installing the beam break: a piece pulled
      // out by hand while the indexer sits stopped cannot be noticed, and hasPiece stays true until
      // something drives the motor again. Every driven state -- INDEXING, FEEDING, EJECTING --
      // re-measures, so the stale answer cannot survive an attempt to use the piece.
      // The unloaded baseline must be established before load can be inferred from a shortfall
      // against it. The comment below correctly identifies the spin-up transient but relies on
      // kHasPieceDebounceSec to outlast it, and it does not: an empty indexer asked to FEED from
      // rest reported a piece, dropped it on reaching speed, and the shot counter recorded a shot
      // that never happened -- in roughly two runs out of three, which reads as flakiness rather
      // than as the deterministic bug it is. Waiting for the wheel to reach its goal once,
      // unloaded,
      // removes the failure by construction instead of by timing margin.
      if (goalRpm != lastGoalRpmForDetection) {
        lastGoalRpmForDetection = goalRpm;
        goalChangeTimer.restart();
      }

      // A settling WINDOW rather than a "has reached goal unloaded" baseline. The baseline version
      // was tried first and is wrong: a wheel that is already carrying a piece when it spins up
      // never reaches its unloaded speed, so it would never establish a baseline and would never
      // detect the piece it is holding. Elapsed time is agnostic about what the wheel is doing.
      boolean settled = goalChangeTimer.hasElapsed(Constants.Indexer.kDetectionSettleSec.get());

      boolean loaded;
      if (goalRpm == 0.0 || !settled) {
        // Stopped, or still spinning up. The predicate cannot say anything either way, so the last
        // sample taken while it could is replayed rather than an answer being invented.
        pieceDetectionSource = goalRpm == 0.0 ? "Latched" : "SpinUp";
        loaded = lastLoadedSample;
      } else {
        // BOTH halves are required, which is not belt-and-braces: a current spike alone fires on
        // every acceleration from rest, and a velocity shortfall alone fires every time a new
        // setpoint is commanded and the wheel has not got there yet. Only their conjunction means
        // "something is loading this wheel".
        //
        // Even so, a spin-up from rest satisfies both halves for a moment -- high current AND a
        // large shortfall -- which is the transient kHasPieceDebounceSec has to outlast. If the
        // indexer reports a phantom piece the instant INDEX is pressed, that is what happened:
        // raise kHasPieceCurrentAmps above the free spin-up current seen in Indexer/Inputs, or
        // lengthen the debounce, in that order.
        //
        // The velocity term is deliberately signed rather than a magnitude comparison. It is true
        // only when the indexer is running SLOWER THAN COMMANDED IN THE COMMANDED DIRECTION, so
        // while EJECTING (a negative goal) it reads false and hasPiece goes false as soon as the
        // eject starts. That is the safe direction to be wrong in -- the scoring gate closes on a
        // piece the robot is in the middle of throwing away -- and it is what stops the replay
        // above from preserving a stale "piece present" once the eject finishes.
        pieceDetectionSource = "CurrentAndVelocity";
        loaded =
            inputs.statorCurrentAmps > Constants.Indexer.kHasPieceCurrentAmps.get()
                && (goalRpm - inputs.velocityRpm)
                    > Constants.Indexer.kHasPieceVelocityDropRpm.get();
        lastLoadedSample = loaded;
      }

      hasPiece = hasPieceDebouncer.calculate(loaded);
    }

    // ---- Shot counting.
    //
    // The latch records WHERE the piece was last seen, rather than testing systemState at the
    // moment it disappears. That difference is what makes the count robust: the superstructure is
    // free to drop out of FEED the instant hasPiece falls -- which is exactly what a readiness gate
    // built on hasPiece() will do -- and the shot is still counted, because the question being
    // asked is "was this piece being fed when we last saw it", not "are we still feeding now".
    if (hasPiece) {
      sawPieceWhileFeeding = systemState == SystemState.FEEDING;
    }

    // A rising debounce on the LEVEL "a fed piece is gone", not on an instantaneous falling edge. A
    // rising debouncer requires its input to hold true for the whole window, so a beam break
    // chattering as the piece clears the emitter -- clear, blocked, clear, blocked over 30 ms --
    // resets the window each time it flickers back and produces exactly one count when the piece is
    // finally gone. Counting raw edges instead would score that single shot four times, and a shot
    // counter that miscounts is worse than no shot counter at all: it is read by the autos and
    // compared against the scoring table after a match, so a wrong number sends people looking for
    // an aiming problem that does not exist.
    //
    // hasPiece is already debounced kBoth, so this is a second filter on top of a first. That is
    // intentional: the first one protects the scoring gate from flicker, this one protects the
    // count from a piece that leaves and briefly re-triggers detection on its way out.
    if (shotDebouncer.calculate(sawPieceWhileFeeding && !hasPiece)) {
      shotCount++;

      // Disarming here is what makes this a counter rather than a latch that fires once and sticks.
      // The debouncer's input goes false on the same loop, and a kRising debouncer passes a falling
      // edge through immediately, so it is ready for the next piece on the very next loop.
      sawPieceWhileFeeding = false;
    }
  }

  /**
   * Recomputes whether a feed is loaded and whether it has jammed.
   *
   * <p>Added for M10b step 6. Both answers exist only while FEEDING, and only once {@code
   * kDetectionSettleSec} has passed since the machine entered it. That window is the reason the
   * constant survived the retrofit: a wheel spinning up from rest draws high current at low speed,
   * which reads as "loaded" and as "jammed" at once -- the same transient that once made the
   * single-piece detector count shots that never happened. Anywhere else the question has no
   * meaning -- a stopped wheel carries no load, and a reversing one is not trying to feed -- and
   * answering it anyway is what made the old detector so intricate.
   *
   * <p><b>Outputs:</b> updates {@code loaded} and {@code jamDetected}.
   */
  private void updateLoadAndJam() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> {
          loadedDebouncer.setDebounceTime(values[0]);
          jamDebouncer.setDebounceTime(values[1]);
        },
        Constants.Indexer.kLoadedDebounceSec,
        Constants.Indexer.kJamDetectSec);

    boolean measuring =
        systemState == SystemState.FEEDING
            && stateTimer.hasElapsed(Constants.Indexer.kDetectionSettleSec.get());

    // Loaded: working against something, read from current alone. The old piece detector also
    // demanded a velocity shortfall, to reject the current spike of every spin-up; the settle
    // window in `measuring` now rejects that, and a shortfall requirement would only miss a stiff
    // loop that pushes pieces while holding its speed. See Constants.Indexer.kLoadedCurrentAmps.
    boolean loadedSample =
        measuring && inputs.statorCurrentAmps > Constants.Indexer.kLoadedCurrentAmps.get();

    // Jammed: STOPPED by something. The current threshold sits above the loaded one, so a full
    // hopper slowing the wheel is not mistaken for a jam stopping it.
    boolean jamSample =
        measuring
            && inputs.statorCurrentAmps > Constants.Indexer.kJamCurrentAmps.get()
            && inputs.velocityRpm < Constants.Indexer.kJamVelocityRpm.get();

    // Both debouncers are fed every loop, false when not measuring, so neither can carry a
    // half-finished window across a state change: the jam window in particular restarts from zero
    // after every unjam pulse, which is what gives each pulse a fair chance to have worked.
    loaded = loadedDebouncer.calculate(loadedSample);
    jamDetected = jamDebouncer.calculate(jamSample);
  }

  /**
   * Decides which state the machine should be in this loop.
   *
   * @return the state to run this iteration
   */
  private SystemState handleStateTransition() {
    // IMPLEMENTATION (was TODO: "INDEX -> INDEXING until hasPiece(), then HOLDING. FEED -> FEEDING.
    // EJECT -> EJECTING with a state timeout that falls back to IDLE after a tunable eject
    // duration. HOLD -> HOLDING. IDLE -> IDLE. CHARACTERIZE -> CHARACTERIZING.")
    //
    // INDEX is the only guarded arm, and the guard is not one-way: if a piece disappears while
    // INDEX is still being requested, the machine drops back to INDEXING and pulls the next one in.
    // That is what makes INDEX safe to hold down through a multi-piece cycle.
    //
    // HOLD is unguarded on purpose. Holding nothing commands a closed-loop zero on an empty
    // indexer, which costs nothing and does nothing; guarding it on hasPiece() would only add a
    // state nobody can explain when the sensor is momentarily wrong.
    //
    // There is no FAULTED state. This mechanism has no goal it can fail to reach -- it has no
    // limits, no homing, and a stall against a game piece is what indexing NORMALLY looks like for
    // a few tenths of a second. A stall-timeout fault would fire during successful intakes and
    // train the drive team to ignore the alert list. The cross-mechanism interlocks this indexer
    // participates in live in exactly one place, Superstructure.applyState (plan section 2.1).
    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case INDEX -> hasPiece() ? SystemState.HOLDING : SystemState.INDEXING;
      case HOLD -> SystemState.HOLDING;
      case FEED -> feedingState();
      case EJECT -> ejectingState();
      case CHARACTERIZE -> SystemState.CHARACTERIZING;
    };
  }

  /**
   * Resolves the state to run while the indexer has been asked to feed.
   *
   * <p>Added for M10b step 6. A stall while feeding reverses the wheel for {@code kUnjamSec} and
   * then feeds again. After {@code kMaxUnjamAttempts} pulses within one FEED request it stops
   * trying and latches JAMMED, because a jam three reversals cannot clear will not be cleared by a
   * fourth, and an indexer cycling forever presents as "the robot is shooting slowly" rather than
   * as the fault it is. The latch is released by a change of request, the same "latch until intent
   * changes" rule the eject timeout uses; the operator's EJECT is the intended recovery.
   *
   * @return FEEDING, UNJAMMING or JAMMED
   */
  private SystemState feedingState() {
    if (systemState == SystemState.JAMMED) {
      return SystemState.JAMMED;
    }
    if (systemState == SystemState.UNJAMMING) {
      return stateTimer.hasElapsed(Constants.Indexer.kUnjamSec.get())
          ? SystemState.FEEDING
          : SystemState.UNJAMMING;
    }
    if (jamDetected) {
      if (unjamAttempts >= Constants.Indexer.kMaxUnjamAttempts) {
        return SystemState.JAMMED;
      }
      unjamAttempts++;
      return SystemState.UNJAMMING;
    }
    return SystemState.FEEDING;
  }

  /**
   * Resolves the state to run while the indexer has been asked to eject.
   *
   * <p>Split out of {@link #handleStateTransition()} because it is the only arm with a timeout and
   * a latch, and nesting those inside a switch arm makes both hard to read.
   *
   * @return {@link SystemState#EJECTING} while the eject is running, {@link SystemState#IDLE} once
   *     it has run for {@code kEjectDurationSec}
   */
  private SystemState ejectingState() {
    // This method is the one place a transition writes state, which the plan's "pure-ish" wording
    // in section 2.1 allows and which Shooter does differently -- it latches on systemState ==
    // FAULTED, because FAULTED is a state nothing else produces. IDLE is not: the machine boots
    // into it, so "am I in IDLE" cannot distinguish "the eject finished" from "the eject has not
    // started yet", and a separate latch is the only honest way to tell those apart.
    if (ejectComplete) {
      return SystemState.IDLE;
    }

    // Measured from entry into EJECTING, because the state timer restarts on every transition. An
    // eject requested from HOLDING therefore gets its full duration, not whatever was left of some
    // earlier state's.
    if (systemState == SystemState.EJECTING
        && stateTimer.hasElapsed(Constants.Indexer.kEjectDurationSec.get())) {
      ejectComplete = true;
      return SystemState.IDLE;
    }

    return SystemState.EJECTING;
  }

  /**
   * Dispatches to the hardware for the current state.
   *
   * <p><b>Outputs:</b> commands the indexer for this loop, and updates {@code goalRpm}.
   */
  private void applyState() {
    // IMPLEMENTATION (was TODO: "switch (systemState) dispatching io.setVelocity with kIndexRpm for
    // INDEXING, 0 for HOLDING, kFeedRpm for FEEDING, kEjectRpm for EJECTING, or io.setOpenLoop(
    // characterizationVolts) for CHARACTERIZING.")
    //
    // The tunables are read every loop rather than latched on entry, so that a speed edited under
    // /Tuning takes effect while the indexer is already running. That is the whole workflow for
    // finding a feed speed: hold the button, turn the number up until the shots stop arriving late.
    switch (systemState) {
      case IDLE -> {
        // A genuine stop, which with the Talon's Brake neutral mode still holds position. IDLE
        // means "nobody is asking for anything"; it is not the state a piece is held in.
        goalRpm = 0.0;
        io.stop();
      }
      case INDEXING -> commandVelocity(Constants.Indexer.kIndexRpm.get());
      case HOLDING -> {
        // A closed-loop zero rather than io.stop(), and the distinction is the whole point of
        // having a HOLDING state separate from IDLE. Brake mode alone resists a piece that is
        // already moving; a closed-loop zero notices the wheel turning and drives it back, which is
        // what keeps a piece from creeping out of a tilted indexer while the robot is being shoved
        // in a corner. It costs nothing at rest, because with no integrator a zero error is zero
        // volts.
        goalRpm = 0.0;
        io.setVelocity(0.0);
      }
      case FEEDING -> commandVelocity(Constants.Indexer.kFeedRpm.get());
      case UNJAMMING -> commandVelocity(Constants.Indexer.kUnjamRpm.get());
      case JAMMED -> {
        // Stopped, in brake. Holding a stalled wheel against its current limit for the rest of a
        // match is how a motor dies, and a jam that three reversals could not clear needs a person.
        goalRpm = 0.0;
        io.stop();
      }
      case EJECTING -> commandVelocity(Constants.Indexer.kEjectRpm.get());
      case CHARACTERIZING -> {
        // goalRpm is cleared because there is no closed-loop goal in open loop. Leaving the last
        // goal standing would feed a meaningless shortfall to the velocity half of the
        // piece-detection predicate, and a characterization sweep would then report phantom pieces.
        goalRpm = 0.0;
        io.setOpenLoop(characterizationVolts);
      }
    }
  }

  /**
   * Commands a closed-loop speed and records it as the goal.
   *
   * @param rpm the speed to command, in RPM; negative runs the indexer backwards
   *     <p><b>Outputs:</b> writes {@code goalRpm} and commands the indexer.
   */
  private void commandVelocity(double rpm) {
    // Both writes live here so the recorded goal and the commanded speed are set from the same
    // value at the same instant. Splitting them across the switch arms above is how piece detection
    // ends up comparing the measured speed against a goal the hardware was never asked for -- and
    // an invented shortfall of a few hundred RPM is exactly the size of the one it is looking for.
    goalRpm = rpm;
    io.setVelocity(rpm);
  }

  /**
   * Requests a new indexer behavior.
   *
   * @param wanted the behavior to request
   *     <p><b>Outputs:</b> sets the wanted state.
   */
  public void setWantedState(WantedState wanted) {
    // IMPLEMENTATION (was TODO: "wantedState = wanted.")
    //
    // The extra line is the eject latch's only reset path, and it is guarded on the request
    // actually changing rather than on being called: a button bound with whileTrue re-requests the
    // same state every loop, and clearing the latch unconditionally would restart the eject
    // forever, which is the exact oscillation the latch exists to prevent.
    if (wanted != wantedState) {
      ejectComplete = false;
      // Every new request gets a full unjam allowance, and a change of request is what releases a
      // JAMMED latch -- see feedingState().
      unjamAttempts = 0;
    }
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
   * Whether a game piece is currently held.
   *
   * @return true when a piece is detected; the superstructure gates scoring states on this
   */
  @AutoLogOutput(key = "Indexer/HasPiece")
  public boolean hasPiece() {
    // IMPLEMENTATION (was TODO: "return hasPiece.")
    //
    // A plain read of a field that updatePieceDetection() owns. All the debouncing happens there,
    // once per loop, rather than here, so that every caller in a given loop -- the superstructure's
    // scoring gate, this class's own transition step, and the annotation that logs this value --
    // sees the same answer. A getter that filtered on each call would give different callers
    // different answers and make the log disagree with the behavior it is supposed to explain.
    return hasPiece;
  }

  /**
   * Whether a feed is actually pushing against something.
   *
   * <p>Added for M10b step 6 (plan section 2.4). Only ever true while FEEDING, past the spin-up
   * settle window, with the wheel slowed by a load. It gates nothing: the superstructure's
   * "probably empty" belief uses it only to avoid concluding the hopper is empty while the indexer
   * is visibly still moving pieces.
   *
   * @return true while feeding against a load
   */
  @AutoLogOutput(key = "Indexer/Loaded")
  public boolean isLoaded() {
    // The state test here is not redundant with the one in updateLoadAndJam(). That one runs
    // before the transition, so on the loop a FEED request ends it still sees FEEDING and samples a
    // wheel that genuinely was feeding -- and the debounce, two-sided so a momentary dip in a
    // loaded feed does not flicker, then holds the answer true for kLoadedDebounceSec into
    // whatever state came next. Found by the case that watches every loop after leaving FEED.
    return loaded && systemState == SystemState.FEEDING;
  }

  /**
   * Whether the indexer is reversing to clear a jam right now.
   *
   * @return true in {@link SystemState#UNJAMMING}; the superstructure reverses the floor roller in
   *     step with it and pauses its "probably empty" clock
   */
  @AutoLogOutput(key = "Indexer/Unjamming")
  public boolean isUnjamming() {
    return systemState == SystemState.UNJAMMING;
  }

  /**
   * Whether the indexer has given up on a jam.
   *
   * @return true in {@link SystemState#JAMMED}
   */
  @AutoLogOutput(key = "Indexer/Jammed")
  public boolean isJammed() {
    return systemState == SystemState.JAMMED;
  }

  /**
   * How many pieces have been fired since the last reset.
   *
   * @return the shot count
   */
  @AutoLogOutput(key = "Indexer/ShotCount")
  public int getShotCount() {
    // IMPLEMENTATION (was TODO: "return shotCount.")
    return shotCount;
  }

  /**
   * Clears the shot counter.
   *
   * <p><b>Outputs:</b> zeroes {@code shotCount}. Call from {@code autonomousInit} so per-match
   * counts are clean.
   */
  public void resetShotCount() {
    // IMPLEMENTATION (was TODO: "shotCount = 0.")
    //
    // The arming latch is cleared too, which the TODO did not ask for. A pit test that feeds a
    // piece through leaves the latch armed; without this line the first real shot of the match
    // could be counted against that leftover arming, or -- worse -- a piece removed by hand between
    // the test and the match start would count as a shot nobody took.
    shotCount = 0;
    sawPieceWhileFeeding = false;
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
   * The live indexer speed.
   *
   * @return indexer speed, in RPM
   */
  public double getVelocityRpm() {
    // IMPLEMENTATION (was TODO: "return inputs.velocityRpm.")
    return inputs.velocityRpm;
  }

  /**
   * The current state of the indexer machine.
   *
   * <p>Added beyond the plan's method list, matching the accessor {@code Shooter} and {@code
   * FloorRoller} added for the same reason: it lets a bench test or a simulation checkpoint assert
   * that the machine walked the states it should have -- IDLE, INDEXING, HOLDING, FEEDING -- rather
   * than that being readable only off an AdvantageScope graph.
   *
   * @return the state the machine is running this loop
   */
  public SystemState getSystemState() {
    return systemState;
  }

  /**
   * The speed currently being commanded.
   *
   * <p>Added beyond the plan's method list so that a test exercising piece detection can command a
   * speed and then assert against the same goal the detector is comparing to, instead of
   * re-deriving it from the tunables and drifting out of step with {@link #applyState()}.
   *
   * @return goal speed, in RPM; zero while stopped, holding, or characterizing
   */
  public double getGoalRpm() {
    return goalRpm;
  }
}
