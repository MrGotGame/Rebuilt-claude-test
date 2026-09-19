package frc.robot.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Owns the robot's answer to "do I have a game piece," which gates every scoring state, and counts
 * shots by detecting the velocity dip as each piece leaves. Both are debounced.
 *
 * <p>The shot counter is genuinely useful rather than decorative: it gives a per-match "pieces
 * fired" number in the log to compare against the scoring table, which is how you find out whether
 * a bad match was an aiming problem or an intake problem.
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
    EJECTING,
    CHARACTERIZING
  }

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();
  private final Debouncer hasPieceDebouncer = new Debouncer(0.0, Debouncer.DebounceType.kBoth);
  private final Debouncer shotDebouncer = new Debouncer(0.0, Debouncer.DebounceType.kRising);

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private boolean hasPiece = false;
  private int shotCount = 0;
  private double characterizationVolts = 0.0;

  /**
   * Builds the indexer around a hardware implementation.
   *
   * @param io the hardware layer: real, simulated, or empty for log replay
   */
  public Indexer(IndexerIO io) {
    this.io = io;
    // TODO: Start stateTimer, and register FaultReporter checks for the motor and -- only when
    // Constants.Indexer.kHasBeamBreak -- the beam break.
  }

  /**
   * Runs one iteration of the state machine. Called by the command scheduler every 20 ms.
   *
   * <p><b>Outputs:</b> refreshes and logs inputs; updates piece detection; may change {@code
   * systemState}; commands the hardware.
   */
  @Override
  public void periodic() {
    // TODO: The standard five-step periodic, but call updatePieceDetection() BETWEEN processInputs
    // and the transition step, so handleStateTransition sees a fresh hasPiece this loop rather than
    // last loop's. Log hasPiece and shotCount.
  }

  /**
   * Recomputes whether a piece is held, and counts shots as pieces leave.
   *
   * <p><b>Outputs:</b> updates {@code hasPiece} and may increment {@code shotCount}.
   */
  private void updatePieceDetection() {
    // TODO: If kHasBeamBreak and inputs.beamBreakConnected, then
    // hasPiece = hasPieceDebouncer.calculate(inputs.beamBreakTripped).
    // Otherwise use the current-and-velocity fallback: hasPiece = hasPieceDebouncer.calculate(
    // inputs.statorCurrentAmps > kHasPieceCurrentAmps.get()
    //     && (goalRpm - inputs.velocityRpm) > kHasPieceVelocityDropRpm.get()).
    // Then detect a shot: while systemState == FEEDING, a falling edge of hasPiece is a shot;
    // pass it through shotDebouncer and increment shotCount.
    // Remember to set each Debouncer's debounce time from the tunables, since they are constructed
    // with 0.0 above.
  }

  /**
   * Decides which state the machine should be in this loop.
   *
   * @return the state to run this iteration
   */
  private SystemState handleStateTransition() {
    // TODO: INDEX -> INDEXING until hasPiece(), then HOLDING. FEED -> FEEDING. EJECT -> EJECTING
    // with a state timeout that falls back to IDLE after a tunable eject duration. HOLD -> HOLDING.
    // IDLE -> IDLE. CHARACTERIZE -> CHARACTERIZING.
    return SystemState.IDLE;
  }

  /**
   * Dispatches to the hardware for the current state.
   *
   * <p><b>Outputs:</b> commands the indexer for this loop.
   */
  private void applyState() {
    // TODO: switch (systemState) dispatching io.setVelocity with kIndexRpm for INDEXING, 0 for
    // HOLDING, kFeedRpm for FEEDING, kEjectRpm for EJECTING, or io.setOpenLoop(
    // characterizationVolts) for CHARACTERIZING.
  }

  /**
   * Requests a new indexer behavior.
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
   * Whether a game piece is currently held.
   *
   * @return true when a piece is detected; the superstructure gates scoring states on this
   */
  @AutoLogOutput(key = "Indexer/HasPiece")
  public boolean hasPiece() {
    // TODO: return hasPiece.
    return false;
  }

  /**
   * How many pieces have been fired since the last reset.
   *
   * @return the shot count
   */
  @AutoLogOutput(key = "Indexer/ShotCount")
  public int getShotCount() {
    // TODO: return shotCount.
    return 0;
  }

  /**
   * Clears the shot counter.
   *
   * <p><b>Outputs:</b> zeroes {@code shotCount}. Call from {@code autonomousInit} so per-match
   * counts are clean.
   */
  public void resetShotCount() {
    // TODO: shotCount = 0.
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
   * The live indexer speed.
   *
   * @return indexer speed, in RPM
   */
  public double getVelocityRpm() {
    // TODO: return inputs.velocityRpm.
    return 0.0;
  }
}
