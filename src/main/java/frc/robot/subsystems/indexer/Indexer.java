package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
 * The gate between the hopper and the flywheel: stopped in brake it holds the pile back, and
 * feeding it lets pieces through.
 *
 * <p>M10b (plan section 2.4) turned the robot into a hopper robot with no piece sensor, so this
 * class no longer answers "do I have a game piece" or counts shots -- shots are counted at the
 * flywheel, by {@code Shooter}. What a whole hopper pressing on this wheel needs instead is jam
 * recovery ({@link SystemState#UNJAMMING}, {@link SystemState#JAMMED}) and a "loaded" reading
 * ({@link #isLoaded()}) that says whether a feed is actually pushing pieces. The single-piece
 * detector, the beam-break path and the old shot counter were deleted in M10b step 8.
 */
public class Indexer extends SubsystemBase {
  /** What the operator or superstructure has asked the indexer to do. */
  public enum WantedState {
    IDLE,
    FEED,
    EJECT,
    CHARACTERIZE
  }

  /** What the indexer is actually doing right now. */
  public enum SystemState {
    IDLE,
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

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private double characterizationVolts = 0.0;

  /**
   * The speed currently being commanded, in RPM, or zero when the indexer is stopped or open loop.
   *
   * <p>Added beyond the skeleton's field list, for the log and for tests that need to know what the
   * wheel was asked for. {@link #applyState()} is the only writer, so the goal and the command
   * issued to the hardware cannot disagree.
   */
  private double goalRpm = 0.0;

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
    // The motor check is registered with FaultReporter since M14 step 2: an error on the dashboard
    // that does not block scoring, because a dead indexer cannot feed a piece to shoot anyway. The
    // beam-break check went with the beam break itself in M10b step 8: the robot has no piece
    // sensor (plan section 2.4).
    FaultReporter.getInstance()
        .registerDeviceDisconnect(
            FaultReporter.canDevice(
                "Indexer motor", Constants.Can.kIndexerId, Constants.Can.kRioBus),
            FaultReporter.Impact.REPORT_ONLY,
            () -> inputs.connected);
    //
    // The timer is started, not restarted, and it is started here rather than on the first loop so
    // that Indexer/StateTime is a real number from the very first log frame instead of reading zero
    // until something happens to transition.
    stateTimer.start();
  }

  /**
   * Runs one iteration of the state machine. Called by the command scheduler every 20 ms.
   *
   * <p><b>Outputs:</b> refreshes and logs inputs; updates load and jam detection; may change {@code
   * systemState}; commands the hardware; pushes the power budget scalar down to the IO layer.
   */
  @Override
  public void periodic() {
    // IMPLEMENTATION (was TODO: "The standard five-step periodic, but call updatePieceDetection()
    // BETWEEN processInputs and the transition step, so handleStateTransition sees a fresh hasPiece
    // this loop rather than last loop's. Log hasPiece and shotCount.")
    //
    // Piece detection and the shot counter were deleted in M10b step 8 (plan section 2.4); what
    // remains between the inputs and the transition is load and jam detection.

    // ---- 1. Inputs. The key is "Indexer/Inputs" rather than bare "Indexer" so that raw hardware
    // inputs and computed outputs stay in separate subtrees, per the log layout in plan section
    // 9.1. Everything below writes under "Indexer/" directly.
    io.updateInputs(inputs);
    Logger.processInputs("Indexer/Inputs", inputs);

    // ---- 1.5. Load and jam detection read this loop's inputs against LAST loop's state, because
    // the question is what the wheel was doing while it was being driven, and it is answered
    // before the transition so the jam response starts on the same loop the jam is confirmed.
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

    // The error is logged, not just the two series it is made of, so that a feed slowed by the
    // hopper can be read off one trace instead of by subtracting two by hand.
    Logger.recordOutput("Indexer/ErrorRpm", goalRpm - inputs.velocityRpm);

    // Alerts last, so they describe the loop that has just run.
    Logger.recordOutput("Indexer/UnjamAttempts", unjamAttempts);
    Logger.recordOutput("Indexer/JamDetected", jamDetected);

    jammedAlert.set(systemState == SystemState.JAMMED);
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
    // INDEX and HOLD, and the INDEXING and HOLDING states they drove, were deleted in M10b step 8
    // (plan section 2.4): with a hopper there is no single piece to seat in front of the flywheel,
    // and a stopped indexer in brake already holds the pile back.
    //
    // There is no FAULTED state. This mechanism has no goal it can fail to reach -- it has no
    // limits, no homing, and a stall against a game piece is what indexing NORMALLY looks like for
    // a few tenths of a second. A stall-timeout fault would fire during successful intakes and
    // train the drive team to ignore the alert list. The cross-mechanism interlocks this indexer
    // participates in live in exactly one place, Superstructure.applyState (plan section 2.1).
    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
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
    // eject requested mid-feed therefore gets its full duration, not whatever was left of some
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
    // characterizationVolts) for CHARACTERIZING.") INDEXING and HOLDING were deleted in M10b step
    // 8.
    //
    // The tunables are read every loop rather than latched on entry, so that a speed edited under
    // /Tuning takes effect while the indexer is already running. That is the whole workflow for
    // finding a feed speed: hold the button, turn the number up until the shots stop arriving late.
    switch (systemState) {
      case IDLE -> {
        // A genuine stop, which with the Talon's Brake neutral mode still holds position. That is
        // what makes a stopped indexer the gate between the hopper and the flywheel.
        goalRpm = 0.0;
        io.stop();
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
        // goal standing would log a meaningless Indexer/ErrorRpm for the whole sweep.
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
    // value at the same instant. Splitting them across the switch arms above is how a log ends up
    // comparing the measured speed against a goal the hardware was never asked for.
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
   * that the machine walked the states it should have -- IDLE, FEEDING, UNJAMMING, JAMMED -- rather
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
   * <p>Added beyond the plan's method list so that a test can assert against the goal actually
   * commanded -- an unjam pulse reversing the wheel, a jammed indexer stopping it -- instead of
   * re-deriving it from the tunables and drifting out of step with {@link #applyState()}.
   *
   * @return goal speed, in RPM; zero while stopped or characterizing
   */
  public double getGoalRpm() {
    return goalRpm;
  }
}
