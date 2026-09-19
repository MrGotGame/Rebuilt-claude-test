package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shooting.ShootingSolver;
import frc.robot.shooting.ShotParameters;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.floorroller.FloorRoller;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * The cross-mechanism state machine, and the only class allowed to command more than one subsystem.
 *
 * <p>It owns the {@link ShootingSolver}, converts its output into turret and flywheel goals,
 * decides when the indexer may feed, enforces the intake/turret interlock in both directions, and
 * handles the "turret cannot reach" case by asking the drivetrain for a heading.
 *
 * <p>It exposes exactly one mutator to the rest of the robot ({@link #setWantedState} and {@link
 * #setWantedStateCommand}), so that a button binding cannot put the robot into an illegal
 * combination of mechanism states. That single entry point is the whole reason this class exists.
 */
public class Superstructure extends SubsystemBase {
  /** What the operator or an autonomous routine has asked the robot to do. */
  public enum WantedState {
    IDLE,
    DEFAULT,
    HOME,
    INTAKE,
    EJECT,
    SPIN_UP,
    SCORE_ON_MOVE,
    SCORE_STATIONARY,
    SCORE_FORCED,
    PASS,
    AUTO_SCORE_ON_MOVE,
    CHARACTERIZE
  }

  /** What the robot as a whole is actually doing right now. */
  public enum SystemState {
    IDLE,
    HOMING,
    STOWED,
    INTAKING,
    INDEXING,
    EJECTING,
    SPINNING_UP,
    SCORING_ON_THE_MOVE,
    SCORING_STATIONARY,
    SCORING_FORCED,
    PASSING,
    WAITING_FOR_TURRET,
    CHARACTERIZING,
    FAULTED
  }

  private final Drive drive;
  private final Intake intake;
  private final FloorRoller floorRoller;
  private final Indexer indexer;
  private final Turret turret;
  private final Shooter shooter;
  private final Vision vision;
  private final ShootingSolver solver = new ShootingSolver();

  private final Timer stateTimer = new Timer();
  private WantedState wantedState = WantedState.HOME;
  private SystemState systemState = SystemState.HOMING;
  private ShotParameters lastShot = ShotParameters.invalid();
  private boolean ignoreShotTolerances = false;
  private Optional<Rotation2d> requestedDriveHeading = Optional.empty();

  /**
   * Builds the superstructure and wires the mechanism interlocks.
   *
   * @param drive the drivetrain
   * @param intake the pivoting intake
   * @param floorRoller the floor roller
   * @param indexer the indexer
   * @param turret the turret
   * @param shooter the flywheel
   * @param vision the vision subsystem
   */
  public Superstructure(
      Drive drive,
      Intake intake,
      FloorRoller floorRoller,
      Indexer indexer,
      Turret turret,
      Shooter shooter,
      Vision vision) {
    this.drive = drive;
    this.intake = intake;
    this.floorRoller = floorRoller;
    this.indexer = indexer;
    this.turret = turret;
    this.shooter = shooter;
    this.vision = vision;
    // TODO: Start stateTimer, then wire the two interlocks:
    //   intake.setDeployGuard(() -> !turretInIntakeConflictArc());
    //   turret.setIntakeArcGuard(() -> !intake.isStowed());
    // This is the ONE place the interlock exists. Do not duplicate it into the subsystems -- two
    // copies of a safety rule is how one of them gets updated and the other does not.
  }

  /**
   * Runs one iteration of the cross-mechanism state machine, every 20 ms.
   *
   * <p><b>Outputs:</b> recomputes the shot solution once; may change {@code systemState}; commands
   * every mechanism subsystem.
   */
  @Override
  public void periodic() {
    // TODO: The standard five-step periodic, minus the IO steps -- this subsystem has no hardware.
    // BEFORE handleStateTransition(), recompute
    //   lastShot = solver.solve(RobotState.getInstance().getEstimatedPose(),
    //       RobotState.getInstance().getFieldRelativeSpeeds(),
    //       RobotState.getInstance().getFieldRelativeAccel(),
    //       Constants.Field.goalTranslation());
    // exactly ONCE per loop, then solver.logDebug("Superstructure/Shot", lastShot).
    // Solving once and sharing the result is what guarantees the turret, the flywheel and the
    // ready-gate all agree about the same shot; solving per-consumer would let them disagree.
  }

  /**
   * Decides which state the robot should be in this loop, applying every cross-mechanism guard.
   *
   * @return the state to run this iteration
   */
  private SystemState handleStateTransition() {
    // TODO: A guarded switch on wantedState. The key guards:
    //   - Any SCORE_* requires indexer.hasPiece() AND !FaultReporter.getInstance().anyCritical();
    //     otherwise fall through to SPINNING_UP rather than refusing silently.
    //   - SCORE_ON_MOVE additionally requires lastShot.isUsable(), else WAITING_FOR_TURRET.
    //   - turret.isOutOfRange() -> WAITING_FOR_TURRET from any scoring state.
    //   - HOME -> HOMING until intake.isHomed() && turret.isHomed(), with a state timeout to
    //     FAULTED.
    //   - INTAKE -> INTAKING, then INDEXING once indexer.hasPiece().
    return SystemState.IDLE;
  }

  /**
   * Runs one-shot work when a state is entered.
   *
   * @param state the state being entered
   *     <p><b>Outputs:</b> reloads solver tables and switches the intake camera pipeline.
   */
  private void onEnter(SystemState state) {
    // TODO: Any SCORING_* -> solver.reloadTables(), so a shot-table edit takes effect mid-match.
    // Note that indexer.resetShotCount() is deliberately NOT called here -- that belongs to
    // autonomousInit, so the per-match count spans the whole match.
    // INTAKING -> vision.setIntakePipeline(gamePiecePipeline).
  }

  /**
   * Runs one-shot work when a state is left.
   *
   * @param state the state being left
   *     <p><b>Outputs:</b> parks the indexer and clears any requested drive heading.
   */
  private void onExit(SystemState state) {
    // TODO: Any SCORING_* -> indexer.setWantedState(HOLD) and clear requestedDriveHeading.
    // Leaving INTAKING -> vision.setIntakePipeline(aprilTagPipeline).
  }

  /**
   * Dispatches to the handler for the current state.
   *
   * <p><b>Outputs:</b> commands every mechanism for this loop.
   */
  private void applyState() {
    // TODO: switch (systemState) calling the private handlers below.
  }

  /** The resting configuration: everything parked, flywheel idling rather than stopped. */
  private void stowed() {
    // TODO: intake STOW, floorRoller IDLE, indexer HOLD (or IDLE when empty), turret STOW,
    // shooter IDLE -- which means the tunable idle RPM, NOT zero. Spinning up from idle roughly
    // halves recovery time.
  }

  /** Establishes trustworthy zeros on the two position mechanisms. */
  private void homing() {
    // TODO: intake HOME, turret HOME, everything else IDLE.
  }

  /** Deploys and runs the full intake chain. */
  private void intaking() {
    // TODO: intake DEPLOY_AND_INTAKE, floorRoller INTAKE, indexer INDEX, shooter IDLE, and turret
    // STOW -- stowing the turret is what keeps it out of the intake's arc while the intake is down.
  }

  /** Finishes seating a piece after the intake has retracted. */
  private void indexing() {
    // TODO: intake STOW, floorRoller FEED briefly, indexer INDEX until hasPiece() then HOLD.
  }

  /** Runs the whole chain backwards to clear a jam. */
  private void ejecting() {
    // TODO: intake EJECT, floorRoller EJECT, indexer EJECT, shooter COAST.
  }

  /** Pre-spins the flywheel toward the solved speed without feeding. */
  private void spinningUp() {
    // TODO: shooter.setGoal(lastShot.isUsable() ? lastShot.flywheelRpm()
    //     : Constants.Shooter.kIdleRpm.get());
    // Turret aims when the solution is usable, otherwise stows.
  }

  /** The full shooting-on-the-move state. */
  private void scoringOnTheMove() {
    // TODO: turret.setGoal(lastShot.turretFieldAngle(), lastShot.turretAngularVelocityRadPerSec());
    // shooter.setGoal(lastShot.flywheelRpm());
    // indexer.setWantedState(isReadyToScore() ? FEED : HOLD);
    // floorRoller.setWantedState(isReadyToScore() ? FEED : IDLE);
    // intake stays STOW, so nothing can enter the turret's arc mid-shot.
  }

  /** Shooting while deliberately stationary, with a tighter gate. */
  private void scoringStationary() {
    // TODO: As scoringOnTheMove, but solve with solver.solveStationary(...) and additionally
    // require |chassis speed| below a tunable threshold inside isReadyToScore().
  }

  /** Operator override that fires regardless of the readiness gate. */
  private void scoringForced() {
    // TODO: Aim using lastShot but feed UNCONDITIONALLY. Log a loud warning, so that when someone
    // reviews the match log they can tell forced shots apart from gated ones -- otherwise a forced
    // miss looks like a solver bug.
  }

  /** Lobs a piece toward a teammate or a staging area. */
  private void passing() {
    // TODO: Aim at the pass target with a lower, separately-tuned RPM.
    // TODO: Add a pass-target translation to Constants.Field and choose a sensible destination once
    // the REBUILT field layout is known.
  }

  /** The turret cannot reach the aim point, so ask the drivetrain for help. */
  private void waitingForTurret() {
    // TODO: Set requestedDriveHeading to the field angle that would bring the aim point back inside
    // the turret's travel, hold the flywheel at the solved RPM, and keep the indexer in HOLD.
    // DriveCommands reads getRequestedDriveHeading() and blends it into the driver's rotation
    // input, rather than overriding it -- the driver keeps translation authority throughout.
  }

  /** Lets a characterization routine drive exactly one mechanism. */
  private void characterizing() {
    // TODO: Pass through to whichever subsystem CharacterizationCommands selected, and command
    // every other subsystem to IDLE so nothing else moves during a SysId sweep.
  }

  /** Everything stopped after a critical fault. */
  private void faulted() {
    // TODO: Command every subsystem to IDLE. Only a HOME request clears FAULTED.
  }

  /**
   * Whether the turret currently occupies, or would sweep through, the intake's arc.
   *
   * @return true when deploying the intake would collide with the turret
   */
  private boolean turretInIntakeConflictArc() {
    // TODO: TurretUnwrapper.passesThroughArc(turret.getPositionRad(), the turret's commanded
    // position, Constants.Turret.kIntakeConflictMinRad.get(),
    // Constants.Turret.kIntakeConflictMaxRad.get()).
    return false;
  }

  /**
   * The full readiness gate for taking a shot.
   *
   * @return true when every precondition for an accurate shot holds, or unconditionally when
   *     tolerances have been overridden
   */
  @AutoLogOutput(key = "Superstructure/ReadyToScore")
  public boolean isReadyToScore() {
    // TODO: Return true immediately when ignoreShotTolerances. Otherwise AND together:
    //   lastShot.isUsable()
    //   turret.atGoal()
    //   shooter.willBeAtSpeedIn(Constants.getLoopPeriodSec() * 2)
    //   indexer.hasPiece()
    //   RobotState.getInstance().getVisionStalenessSec() < kMaxVisionStalenessSec.get()
    //   RobotState.getInstance().getPoseStdDevEstimate() < kMaxPoseStdDevMeters.get()
    //   RobotState.getInstance().getFieldRelativeAccel().getNorm()
    //       < kMaxChassisAccelForShotMetersPerSec2.get()
    //   !FaultReporter.getInstance().anyCritical()
    //
    // LOG EACH TERM SEPARATELY under Superstructure/ReadyTerms/*. When the robot will not shoot and
    // a match is running, this log is the difference between finding the false term in ten seconds
    // and guessing for three matches.
    return false;
  }

  /**
   * Requests a new robot-level behavior.
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
   * @return an instant command that requests {@code wanted}, deliberately <b>not</b> requiring this
   *     subsystem
   */
  public Command setWantedStateCommand(WantedState wanted) {
    // Note Commands.runOnce, not this.runOnce: requiring the superstructure here would interrupt
    // the default drive command every time a held button re-fires.
    return Commands.runOnce(() -> setWantedState(wanted));
  }

  /**
   * Scores repeatedly until the indexer reports empty. Used by autonomous routines.
   *
   * @return a command that scores on the move, then returns the robot to its default state
   */
  public Command scoreUntilEmptyCommand() {
    // TODO: Replace the 3.0 s literal below with a tunable timeout from Constants.
    return Commands.sequence(
        setWantedStateCommand(WantedState.AUTO_SCORE_ON_MOVE),
        Commands.waitUntil(() -> !indexer.hasPiece()).withTimeout(3.0),
        setWantedStateCommand(WantedState.DEFAULT));
  }

  /**
   * Toggles the operator's "force shoot" override.
   *
   * @param ignore true to bypass every readiness tolerance
   *     <p><b>Outputs:</b> stores the override flag.
   */
  public void setIgnoreShotTolerances(boolean ignore) {
    // TODO: ignoreShotTolerances = ignore.
  }

  /**
   * The heading the superstructure would like the drivetrain to take.
   *
   * @return the requested field heading while waiting for the turret, or empty when the drivetrain
   *     should be left entirely to the driver
   */
  public Optional<Rotation2d> getRequestedDriveHeading() {
    // TODO: return requestedDriveHeading.
    return Optional.empty();
  }

  /**
   * The shot solution computed this loop.
   *
   * @return the most recent solution; autonomous routines use this to decide whether to keep
   *     driving or hold position
   */
  public ShotParameters getLastShot() {
    return lastShot;
  }

  /**
   * The current robot-level state.
   *
   * @return the active state; published as a string that AdvantageScope graphs as a state timeline,
   *     which is the single most useful plot in a post-match review
   */
  @AutoLogOutput(key = "Superstructure/SystemState")
  public SystemState getSystemState() {
    return systemState;
  }
}
