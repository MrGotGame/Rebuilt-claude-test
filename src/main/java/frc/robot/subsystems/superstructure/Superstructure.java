package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingSolver;
import frc.robot.shooting.ShotParameters;
import frc.robot.shooting.TurretUnwrapper;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.floorroller.FloorRoller;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FaultReporter;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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

  /**
   * Which mechanism a characterization routine is currently driving.
   *
   * <p>Added beyond the plan's method list. Section 4.11 asks {@code CHARACTERIZING} to "pass
   * through to whichever subsystem CharacterizationCommands has selected", which presumes a way to
   * say which one; nothing in the skeleton provided it, so {@code CHARACTERIZING} would have had to
   * either move everything or move nothing. Moving everything during a SysId sweep is how a sweep
   * produces a fit of the wrong mechanism, so the selection is made explicit instead.
   *
   * <p>{@link #DRIVE} commands every mechanism to idle and leaves the drivetrain alone, because the
   * drivetrain's own SysId routine already owns it through the command scheduler.
   */
  public enum CharacterizationTarget {
    NONE,
    DRIVE,
    INTAKE_PIVOT,
    FLOOR_ROLLER,
    INDEXER,
    TURRET,
    SHOOTER
  }

  private final Drive drive;
  private final Intake intake;
  private final FloorRoller floorRoller;
  private final Indexer indexer;
  private final Turret turret;
  private final Shooter shooter;
  private final Vision vision;
  private final ShootingSolver solver;

  private final Timer stateTimer = new Timer();
  private WantedState wantedState = WantedState.HOME;
  private SystemState systemState = SystemState.HOMING;
  private ShotParameters lastShot = ShotParameters.invalid();
  private boolean ignoreShotTolerances = false;
  private Optional<Rotation2d> requestedDriveHeading = Optional.empty();

  /**
   * Whether the mechanisms still need to be told to home.
   *
   * <p>This latch exists because both {@code Intake.setWantedState(HOME)} and {@code
   * Turret.setWantedState(HOME)} have side effects -- each clears its homed flag and arms a
   * one-shot home request -- so calling them once per loop from the {@link #homing()} handler, the
   * way every other handler commands its mechanisms, would restart homing on every single loop.
   * Neither mechanism could ever finish, and neither could ever reach its own homing timeout
   * either, because the re-request is consumed ahead of the timeout check. The robot would sit in
   * HOMING forever and then fault on the superstructure's timeout with no mechanism-level alert to
   * say which one was stuck.
   *
   * <p>Set on entry to HOMING and by a fresh {@link WantedState#HOME} request, consumed by {@link
   * #homing()}. That covers boot, a retry after a fault, and an operator pressing HOME again while
   * homing is already running.
   */
  private boolean homeRequestPending = true;

  /**
   * Whether a <b>fresh</b> {@link WantedState#HOME} request is waiting to be acted on.
   *
   * <p>Distinct from {@link #homeRequestPending}, which is about telling the mechanisms to home.
   * This one is about clearing a latched fault, and the two cannot be the same flag.
   *
   * <p>The fault latch cannot be written as "faulted AND the request is not HOME", which is how
   * every mechanism FSM in this project writes it. Those mechanisms boot into IDLE; this one boots
   * into HOMING with HOME already requested, so that test is true at boot and stays true for as
   * long as nobody asks for anything else. A homing timeout would then latch FAULTED for exactly
   * one loop before falling straight back into HOMING, and the robot would cycle HOMING -> FAULTED
   * -> HOMING for the rest of the match -- re-stalling a jammed pivot on every pass, which is the
   * failure the timeout exists to prevent.
   *
   * <p>Consumed by {@link #handleStateTransition()}, so a fault clears on the EDGE of a request
   * rather than while one happens to be standing. The turret's own machine is built the same way,
   * for the same reason.
   */
  private boolean homeRequested = false;

  /** Which mechanism {@link SystemState#CHARACTERIZING} should pass voltage through to. */
  private CharacterizationTarget characterizationTarget = CharacterizationTarget.NONE;

  /**
   * The readiness gate, evaluated once per loop in {@link #periodic()}.
   *
   * <p>Cached rather than recomputed per call, following {@code Indexer.hasPiece()}: the scoring
   * handlers, the {@code @AutoLogOutput} annotation and the per-term log must all see the same
   * answer within one loop, or the log stops explaining the behaviour it exists to explain.
   */
  private boolean readyToScore = false;

  /**
   * The same gate as {@link #readyToScore}, minus the shot-solution term.
   *
   * <p>A pass does not go through the solver -- it is a lob at a fixed speed toward a fixed
   * translation, so there is no {@code ShotParameters} to check and the shot tables are never
   * indexed at a distance nobody measured them at. Reusing readyToScore for it would make passing
   * wait on a solution to the GOAL that has nothing to do with the pass, which on this branch means
   * waiting forever: the tables are zeros, so term 1 is false and the pass could never leave.
   *
   * <p>Every other term still applies. Aiming a pass needs the robot to know where it is just as
   * much as aiming a shot does, and turret.atGoal() is what stops a pass leaving mid-slew.
   */
  private boolean readyToPass = false;

  /** How many times a shot has been taken with the readiness gate bypassed, since boot. */
  private int forcedShotCount = 0;

  /**
   * Raised while any shot is being taken with the gate bypassed.
   *
   * <p>A forced miss and a solver bug look identical in a match video. This is the badge that tells
   * the two apart at the time, and {@code Superstructure/ForcedShot} is how they are told apart
   * afterwards.
   */
  private final Alert forcedShotAlert =
      new Alert("Shot readiness gate BYPASSED (force shoot active)", AlertType.kWarning);

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
    this(drive, intake, floorRoller, indexer, turret, shooter, vision, new ShootingSolver());
  }

  /**
   * Builds the superstructure around an explicitly supplied shooting solver.
   *
   * <p>The robot uses the seven-argument constructor, which builds a solver from {@link
   * Constants.Shot}'s tables. This one exists for the same reason {@code Vision} takes its camera
   * mountings as arguments: those tables are still all zeros awaiting M11, and a solver reading
   * them correctly returns an unusable solution for every shot. That is the right behaviour and
   * there is a permanent test pinning it -- but it also means every scoring state would be
   * unreachable, so an FSM suite built only on the shipped tables could not demonstrate that the
   * machine reaches them at all.
   *
   * @param drive the drivetrain
   * @param intake the pivoting intake
   * @param floorRoller the floor roller
   * @param indexer the indexer
   * @param turret the turret
   * @param shooter the flywheel
   * @param vision the vision subsystem
   * @param solver the shooting solver to own and drive
   */
  public Superstructure(
      Drive drive,
      Intake intake,
      FloorRoller floorRoller,
      Indexer indexer,
      Turret turret,
      Shooter shooter,
      Vision vision,
      ShootingSolver solver) {
    // IMPLEMENTATION (was TODO: "Start stateTimer, then wire the two interlocks: intake
    // .setDeployGuard(() -> !turretInIntakeConflictArc()); turret.setIntakeArcGuard(() ->
    // !intake.isStowed()). This is the ONE place the interlock exists.")
    this.drive = drive;
    this.intake = intake;
    this.floorRoller = floorRoller;
    this.indexer = indexer;
    this.turret = turret;
    this.shooter = shooter;
    this.vision = vision;
    this.solver = solver;

    // Started, not restarted, and here rather than on the first loop, so Superstructure/StateTime
    // is a real number from the very first log frame -- and so the boot homing attempt is timed
    // from boot rather than from whenever something first happened to touch the timer.
    stateTimer.start();

    // THE interlock, both directions, in the one place the plan allows it to exist.
    //
    // They run in opposite directions and both are load-bearing. The first keeps the intake from
    // swinging out into a turret that is sitting in, or about to sweep through, the arc the intake
    // occupies. The second keeps the turret from sweeping through an intake that is already out.
    // Enforcing only one of them leaves the collision reachable from the other side.
    //
    // Both are suppliers, evaluated fresh every loop by the mechanism that consumes them. A pushed
    // boolean would answer a question about where the other mechanism used to be, and would go
    // stale silently the first time a code path forgot to push.
    intake.setDeployGuard(() -> !turretInIntakeConflictArc());
    turret.setIntakeArcGuard(() -> !intake.isStowed());

    // Entry actions for the state the machine BOOTS into. periodic() only fires onEnter on a
    // transition, and systemState is initialised to HOMING at field level, so without this call the
    // boot home would run with none of its entry work done. The concrete loss here is the intake
    // camera's pipeline: a robot rebooted after an intake run would come up with the Limelight
    // still on the game-piece pipeline, contributing no tags at all, and nothing would say so.
    //
    // This has already bitten this project once, on the intake, and it is the failure
    // entryAndExitActionsFireExactlyOnce exists to catch.
    onEnter(systemState);
  }

  /**
   * Runs one iteration of the cross-mechanism state machine, every 20 ms.
   *
   * <p><b>Outputs:</b> recomputes the shot solution once; may change {@code systemState}; commands
   * every mechanism subsystem.
   */
  @Override
  public void periodic() {
    // IMPLEMENTATION (was TODO: "The standard five-step periodic, minus the IO steps ... BEFORE
    // handleStateTransition(), recompute lastShot = solver.solve(...) exactly ONCE per loop, then
    // solver.logDebug("Superstructure/Shot", lastShot). Solving once and sharing the result is what
    // guarantees the turret, the flywheel and the ready-gate all agree about the same shot.")

    // ---- 1. One solve, shared by everything downstream.
    //
    // DEVIATION from the TODO, which names solve() alone while section 4.11 also asks
    // scoringStationary() to use solveStationary(). Doing that inside the handler would be a SECOND
    // solve in the same loop, against different inputs, and the whole reason the TODO insists on
    // solving once is that the turret, the flywheel and the gate must not disagree about which shot
    // they are taking. So the VARIANT is chosen here instead, from the request, and there is still
    // exactly one solution per loop.
    RobotState state = RobotState.getInstance();
    Translation2d goal = Constants.Field.goalTranslation();
    lastShot =
        wantedState == WantedState.SCORE_STATIONARY
            ? solver.solveStationary(state.getEstimatedPose(), goal)
            : solver.solve(
                state.getEstimatedPose(),
                state.getFieldRelativeSpeeds(),
                state.getFieldRelativeAccel(),
                goal);
    solver.logDebug("Superstructure/Shot", lastShot);

    // ---- 2. Transition, with entry and exit actions fired exactly once per change.
    SystemState next = handleStateTransition();
    if (next != systemState) {
      onExit(systemState);
      onEnter(next);
      systemState = next;
      stateTimer.restart();
    }

    // ---- 3. The readiness gate, BEFORE the handlers, so that every handler reading
    // isReadyToScore() this loop reads the same answer, computed against the state it is running
    // in. Evaluating it lazily inside the handlers would let the indexer and the floor roller
    // disagree about whether to feed.
    updateReadyToScore();

    // ---- 4. Command the mechanisms.
    applyState();

    // ---- 5. Log. SystemState and ReadyToScore carry @AutoLogOutput annotations; these are the
    // values that have no getter of their own.
    Logger.recordOutput("Superstructure/WantedState", wantedState);
    Logger.recordOutput("Superstructure/StateTimeSec", stateTimer.get());
    Logger.recordOutput(
        "Superstructure/RequestedDriveHeadingRad",
        requestedDriveHeading.map(Rotation2d::getRadians).orElse(Double.NaN));
    Logger.recordOutput("Superstructure/CharacterizationTarget", characterizationTarget);
  }

  /**
   * Decides which state the robot should be in this loop, applying every cross-mechanism guard.
   *
   * @return the state to run this iteration
   */
  private SystemState handleStateTransition() {
    // IMPLEMENTATION (was TODO: "A guarded switch on wantedState. The key guards: any SCORE_*
    // requires indexer.hasPiece() AND !FaultReporter.getInstance().anyCritical(), otherwise fall
    // through to SPINNING_UP rather than refusing silently. SCORE_ON_MOVE additionally requires
    // lastShot.isUsable(), else WAITING_FOR_TURRET. turret.isOutOfRange() -> WAITING_FOR_TURRET
    // from any scoring state. HOME -> HOMING until intake.isHomed() && turret.isHomed(), with a
    // state timeout to FAULTED. INTAKE -> INTAKING, then INDEXING once indexer.hasPiece().")

    // A fresh home request always wins, and clears a latched fault. This is the operator's
    // deliberate retry: something was wrong, they fixed it, they are asking again.
    //
    // Consumed here rather than tested against wantedState, because wantedState is HOME at boot
    // and stays HOME until somebody asks for something else -- so a wantedState test would leave
    // the fault latch below permanently open. See the homeRequested field.
    if (homeRequested) {
      homeRequested = false;
      return SystemState.HOMING;
    }

    // A fault latches until that retry arrives, matching every mechanism FSM in this project.
    // Without the latch the machine would leave FAULTED the instant the failing condition happened
    // not to hold, and the alert would flicker instead of telling the pit crew there is something
    // to fix.
    if (systemState == SystemState.FAULTED) {
      return SystemState.FAULTED;
    }

    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case DEFAULT -> SystemState.STOWED;
      case HOME -> homingTransition();
      case EJECT -> SystemState.EJECTING;
      case CHARACTERIZE -> SystemState.CHARACTERIZING;
      case SPIN_UP -> SystemState.SPINNING_UP;
      case PASS -> SystemState.PASSING;

        // INDEXING is the seating pass that runs after the piece has been detected: the intake
        // comes in, the floor roller gives it a last nudge, and the indexer parks it. Holding the
        // intake button through a whole cycle therefore walks INTAKING -> INDEXING by itself.
      case INTAKE -> indexer.hasPiece() ? SystemState.INDEXING : SystemState.INTAKING;

      case SCORE_ON_MOVE, AUTO_SCORE_ON_MOVE -> scoringTransition(SystemState.SCORING_ON_THE_MOVE);
      case SCORE_STATIONARY -> scoringTransition(SystemState.SCORING_STATIONARY);
      case SCORE_FORCED -> scoringTransition(SystemState.SCORING_FORCED);
    };
  }

  /**
   * Resolves which state to run while the robot has been asked to home.
   *
   * <p>Split out of {@link #handleStateTransition()} because it carries the only timeout in this
   * machine, and nesting a timeout inside a switch arm makes it hard to see.
   *
   * @return {@link SystemState#STOWED} once both position mechanisms are trustworthy, {@link
   *     SystemState#FAULTED} if they take too long, otherwise {@link SystemState#HOMING}
   */
  private SystemState homingTransition() {
    // Both mechanisms, not either. The turret seeds from its absolute encoder in a single loop
    // while the intake drives into a hard stop over a second or more, so waiting on only one of
    // them would declare the robot ready with an arm of unknown position -- and an unknown arm is
    // exactly what the interlock cannot protect against.
    if (intake.isHomed() && turret.isHomed()) {
      return SystemState.STOWED;
    }

    // Measured from entry into HOMING, because stateTimer restarts on every transition. The
    // timeout is longer than either mechanism's own, deliberately: a mechanism that cannot home
    // should fault first and say WHICH mechanism it is. See Constants.Superstructure.
    if (systemState == SystemState.HOMING
        && stateTimer.hasElapsed(Constants.Superstructure.kHomingTimeoutSec.get())) {
      return SystemState.FAULTED;
    }
    return SystemState.HOMING;
  }

  /**
   * Resolves which state to run while the robot has been asked to score.
   *
   * <p>Split out of {@link #handleStateTransition()} because these four guards are the substance of
   * this FSM and all three scoring requests share them.
   *
   * @param requested the scoring state the operator asked for
   * @return {@code requested}, or the state the guards divert to
   */
  private SystemState scoringTransition(SystemState requested) {
    // Guard 1 and 2: no piece, or a critical fault. Falling through to SPINNING_UP rather than
    // refusing outright is deliberate and it is what the plan asks for: the driver holding the
    // score trigger with an empty robot still gets the flywheel brought up to the solved speed, so
    // that the shot is ready the instant a piece arrives. Refusing silently would present as a
    // trigger that does nothing.
    if (!indexer.hasPiece() || FaultReporter.getInstance().anyCritical()) {
      return SystemState.SPINNING_UP;
    }

    // SCORING_FORCED skips the aim guards as well as the tolerance gate, and that is the whole
    // point of the state: it exists for the case where the operator can see the robot is aimed and
    // the software disagrees. It does NOT skip the two guards above -- forcing a shot out of an
    // empty indexer or through a dead flywheel is not an override of a judgement call, it is a
    // request the robot cannot carry out at all.
    if (requested == SystemState.SCORING_FORCED) {
      return SystemState.SCORING_FORCED;
    }

    // Guard 3: no usable solution. ShotParameters' own contract is that an unusable solution
    // carries meaningless numbers rather than imprecise ones, so acting on one would slew the
    // turret to whatever ShotParameters.invalid() happens to hold -- field angle zero. Refusing to
    // aim at all is the honest answer.
    //
    // With the shot tables still all zeros this is the branch EVERY scoring request takes today.
    // That is correct behaviour on this branch, not a bug to tune around, and
    // scoreOnMoveRequiresUsableShotSolution is the test that pins it.
    //
    // Guard 4: the turret physically cannot get there. Only the turret knows, because only the
    // turret knows which branch it would sweep along and whether the deployed intake is in the way.
    if (!lastShot.isUsable() || turret.isOutOfRange()) {
      return SystemState.WAITING_FOR_TURRET;
    }

    return requested;
  }

  /**
   * Runs one-shot work when a state is entered.
   *
   * @param state the state being entered
   *     <p><b>Outputs:</b> reloads solver tables and switches the intake camera pipeline.
   */
  private void onEnter(SystemState state) {
    // IMPLEMENTATION (was TODO: "Any SCORING_* -> solver.reloadTables(), so a shot-table edit takes
    // effect mid-match. Note that indexer.resetShotCount() is deliberately NOT called here -- that
    // belongs to autonomousInit. INTAKING -> vision.setIntakePipeline(gamePiecePipeline).")
    switch (state) {
      case HOMING -> {
        // Re-arm the home request. Consumed by homing(), once, rather than issued from here,
        // because setWantedState(HOME) on either mechanism is a one-shot with side effects -- see
        // the homeRequestPending field documentation for what re-issuing it every loop does.
        homeRequestPending = true;

        // The intake camera is put back on AprilTags. This is the entry action that makes the
        // constructor's onEnter(HOMING) call worth making: a robot rebooted after an intake run
        // otherwise comes up with the camera still on the game-piece pipeline, silently
        // contributing no tags.
        vision.setIntakePipeline(Constants.Vision.kAprilTagPipelineIndex);
      }

      case INTAKING -> {
        // Game pieces while the intake is out. The switch costs about 100 ms, which is why it is
        // driven from a state change rather than from a per-loop condition that could chatter.
        vision.setIntakePipeline(Constants.Vision.kGamePiecePipelineIndex);
      }

      case SCORING_ON_THE_MOVE, SCORING_STATIONARY, SCORING_FORCED, PASSING -> {
        // Re-read the shot tables on the way into a shot, so that a table edited under /Tuning
        // between matches takes effect without a redeploy. This is the mechanism M11 collects data
        // with.
        solver.reloadTables();

        if (state == SystemState.SCORING_FORCED) {
          forcedShotCount++;
          Logger.recordOutput("Superstructure/ForcedShotCount", forcedShotCount);
        }
      }

      default -> {}
    }
  }

  /**
   * Runs one-shot work when a state is left.
   *
   * @param state the state being left
   *     <p><b>Outputs:</b> parks the indexer and clears any requested drive heading.
   */
  private void onExit(SystemState state) {
    // IMPLEMENTATION (was TODO: "Any SCORING_* -> indexer.setWantedState(HOLD) and clear
    // requestedDriveHeading. Leaving INTAKING -> vision.setIntakePipeline(aprilTagPipeline).")
    switch (state) {
      case INTAKING -> vision.setIntakePipeline(Constants.Vision.kAprilTagPipelineIndex);

      case SCORING_ON_THE_MOVE, SCORING_STATIONARY, SCORING_FORCED, PASSING -> {
        // Park the belt on the way out rather than leaving it feeding. Whatever state runs next
        // will command the indexer again on its own first loop, but "whatever runs next" includes
        // FAULTED and IDLE, and a feed left running through a fault is a piece ejected into the
        // field at a random moment.
        indexer.setWantedState(Indexer.WantedState.HOLD);
        requestedDriveHeading = Optional.empty();
      }

        // The state that ASKS for a heading is also the state that must give it back. Leaving it
        // set
        // would have the drivetrain still nudging towards an aim point the robot stopped caring
        // about several seconds ago.
      case WAITING_FOR_TURRET -> requestedDriveHeading = Optional.empty();

      default -> {}
    }
  }

  /**
   * Dispatches to the handler for the current state.
   *
   * <p><b>Outputs:</b> commands every mechanism for this loop.
   */
  private void applyState() {
    // IMPLEMENTATION (was TODO: "switch (systemState) calling the private handlers below.")
    switch (systemState) {
      case IDLE -> idling();
      case HOMING -> homing();
      case STOWED -> stowed();
      case INTAKING -> intaking();
      case INDEXING -> indexing();
      case EJECTING -> ejecting();
      case SPINNING_UP -> spinningUp();
      case SCORING_ON_THE_MOVE -> scoringOnTheMove();
      case SCORING_STATIONARY -> scoringStationary();
      case SCORING_FORCED -> scoringForced();
      case PASSING -> passing();
      case WAITING_FOR_TURRET -> waitingForTurret();
      case CHARACTERIZING -> characterizing();
      case FAULTED -> faulted();
    }

    // The forced-shot badge, evaluated wherever the machine happens to be: the SCORING_FORCED state
    // and the operator's ignoreShotTolerances toggle are two different routes to the same bypass,
    // and a log reviewer needs to see both.
    boolean gateBypassed = systemState == SystemState.SCORING_FORCED || ignoreShotTolerances;
    forcedShotAlert.set(gateBypassed);
    Logger.recordOutput("Superstructure/ForcedShot", gateBypassed);
  }

  /**
   * The parked configuration: nothing commanded, flywheel genuinely stopped.
   *
   * <p><b>Outputs:</b> idles every mechanism and coasts the flywheel.
   */
  private void idling() {
    // Added beyond the plan's handler list, because WantedState.IDLE had nowhere to dispatch to.
    //
    // The difference from stowed() is the flywheel, and it is the whole reason both states exist.
    // STOWED keeps the wheel spinning at the tunable idle speed, which costs current continuously
    // and buys back about half the spin-up time on the next shot. IDLE is the state for a robot
    // that has been disabled, is sitting in the pit, or is riding the cart: COAST stops the wheel.
    intake.setWantedState(Intake.WantedState.IDLE);
    floorRoller.setWantedState(FloorRoller.WantedState.IDLE);
    indexer.setWantedState(Indexer.WantedState.IDLE);
    turret.setWantedState(Turret.WantedState.IDLE);
    shooter.setWantedState(Shooter.WantedState.COAST);
  }

  /** Establishes trustworthy zeros on the two position mechanisms. */
  private void homing() {
    // IMPLEMENTATION (was TODO: "intake HOME, turret HOME, everything else IDLE.")
    //
    // The two HOME requests are issued ONCE, on the latch, not once per loop as the TODO reads.
    // Both mechanisms treat setWantedState(HOME) as an edge: each clears its homed flag and arms a
    // one-shot request that its transition method consumes ahead of its own timeout check. Issued
    // every loop, neither mechanism can ever finish homing and neither can ever time out either.
    // See the homeRequestPending field for the full failure.
    if (homeRequestPending) {
      intake.setWantedState(Intake.WantedState.HOME);
      turret.setWantedState(Turret.WantedState.HOME);
      homeRequestPending = false;
    }

    floorRoller.setWantedState(FloorRoller.WantedState.IDLE);
    indexer.setWantedState(Indexer.WantedState.IDLE);

    // IDLE rather than COAST: the wheel is welcome to come up to its idle speed while the arm finds
    // its stop, and homing is the one time the robot is guaranteed not to be in a hurry.
    shooter.setWantedState(Shooter.WantedState.IDLE);
  }

  /** The resting configuration: everything parked, flywheel idling rather than stopped. */
  private void stowed() {
    // IMPLEMENTATION (was TODO: "intake STOW, floorRoller IDLE, indexer HOLD (or IDLE when empty),
    // turret STOW, shooter IDLE -- which means the tunable idle RPM, NOT zero. Spinning up from
    // idle roughly halves recovery time.")
    intake.setWantedState(Intake.WantedState.STOW);
    floorRoller.setWantedState(FloorRoller.WantedState.IDLE);

    // HOLD is a closed-loop zero that drives a creeping piece back; IDLE is a genuine stop. Holding
    // nothing would work and cost nothing, but IDLE is the honest description of an empty indexer
    // and it keeps the state timeline readable.
    indexer.setWantedState(
        indexer.hasPiece() ? Indexer.WantedState.HOLD : Indexer.WantedState.IDLE);
    turret.setWantedState(Turret.WantedState.STOW);
    shooter.setWantedState(Shooter.WantedState.IDLE);
  }

  /** Deploys and runs the full intake chain. */
  private void intaking() {
    // IMPLEMENTATION (was TODO: "intake DEPLOY_AND_INTAKE, floorRoller INTAKE, indexer INDEX,
    // shooter IDLE, and turret STOW -- stowing the turret is what keeps it out of the intake's arc
    // while the intake is down.")
    //
    // The turret request is not decoration. The intake's deploy guard asks whether the turret is in
    // or sweeping through the conflict arc, so commanding the turret to stow is what OPENS that
    // guard; leave the turret aiming and the intake would simply refuse to come down, presenting as
    // an intake button that does nothing.
    turret.setWantedState(Turret.WantedState.STOW);
    intake.setWantedState(Intake.WantedState.DEPLOY_AND_INTAKE);
    floorRoller.setWantedState(FloorRoller.WantedState.INTAKE);
    indexer.setWantedState(Indexer.WantedState.INDEX);
    shooter.setWantedState(Shooter.WantedState.IDLE);
  }

  /** Finishes seating a piece after the intake has retracted. */
  private void indexing() {
    // IMPLEMENTATION (was TODO: "intake STOW, floorRoller FEED briefly, indexer INDEX until
    // hasPiece() then HOLD.")
    //
    // "Briefly" is Constants.Superstructure.kIndexFeedSec, measured from entry into this state. A
    // roller left running because the driver is still holding the intake button would grind the
    // next piece against the one already aboard.
    //
    // The indexer is asked for INDEX rather than HOLD even though a piece is already detected:
    // Indexer.handleStateTransition promotes INDEX to HOLDING on hasPiece() by itself, and drops
    // back to INDEXING if the piece is lost again. Deciding it here as well would be a second copy
    // of that rule, free to disagree with the first.
    turret.setWantedState(Turret.WantedState.STOW);
    intake.setWantedState(Intake.WantedState.STOW);
    floorRoller.setWantedState(
        stateTimer.hasElapsed(Constants.Superstructure.kIndexFeedSec.get())
            ? FloorRoller.WantedState.IDLE
            : FloorRoller.WantedState.FEED);
    indexer.setWantedState(Indexer.WantedState.INDEX);
    shooter.setWantedState(Shooter.WantedState.IDLE);
  }

  /** Runs the whole chain backwards to clear a jam. */
  private void ejecting() {
    // IMPLEMENTATION (was TODO: "intake EJECT, floorRoller EJECT, indexer EJECT, shooter COAST.")
    //
    // The turret is stowed for the same reason it is during INTAKING: the eject angle puts the arm
    // outside stow, so the intake's deploy guard applies to EJECT exactly as it does to DEPLOY, and
    // an aiming turret would hold that guard shut and make the eject button do nothing.
    turret.setWantedState(Turret.WantedState.STOW);
    intake.setWantedState(Intake.WantedState.EJECT);
    floorRoller.setWantedState(FloorRoller.WantedState.EJECT);
    indexer.setWantedState(Indexer.WantedState.EJECT);

    // COAST, not IDLE. Clearing a jam is the one time the wheel should be stopped rather than
    // idling: a piece coming backwards through a spinning flywheel is how a jam becomes two jams.
    shooter.setWantedState(Shooter.WantedState.COAST);
  }

  /** Pre-spins the flywheel toward the solved speed without feeding. */
  private void spinningUp() {
    // IMPLEMENTATION (was TODO: "shooter.setGoal(lastShot.isUsable() ? lastShot.flywheelRpm() :
    // Constants.Shooter.kIdleRpm.get()); turret aims when the solution is usable, otherwise
    // stows.")
    intake.setWantedState(Intake.WantedState.STOW);
    floorRoller.setWantedState(FloorRoller.WantedState.IDLE);
    indexer.setWantedState(
        indexer.hasPiece() ? Indexer.WantedState.HOLD : Indexer.WantedState.INDEX);

    if (lastShot.isUsable()) {
      shooter.setGoal(lastShot.flywheelRpm());
      aimAt(lastShot.turretFieldAngle(), lastShot.turretAngularVelocityRadPerSec());
    } else {
      // The idle speed rather than zero, because SPINNING_UP is reached by a driver holding the
      // score trigger while the robot is out of range or still looking for a tag. Dropping the
      // wheel to a stop in that moment costs the whole spin-up the next time it comes into range.
      shooter.setGoal(Constants.Shooter.kIdleRpm.get());
      turret.setWantedState(Turret.WantedState.STOW);
    }
  }

  /** The full shooting-on-the-move state. */
  private void scoringOnTheMove() {
    // IMPLEMENTATION (was TODO: "turret.setGoal(lastShot.turretFieldAngle(), lastShot
    // .turretAngularVelocityRadPerSec()); shooter.setGoal(lastShot.flywheelRpm());
    // indexer.setWantedState(isReadyToScore() ? FEED : HOLD); floorRoller.setWantedState(
    // isReadyToScore() ? FEED : IDLE); intake stays STOW, so nothing can enter the turret's arc
    // mid-shot.")
    runScoringChain();
  }

  /** Shooting while deliberately stationary, with a tighter gate. */
  private void scoringStationary() {
    // IMPLEMENTATION (was TODO: "As scoringOnTheMove, but solve with solver.solveStationary(...)
    // and additionally require |chassis speed| below a tunable threshold inside isReadyToScore().")
    //
    // The solve variant is chosen in periodic() rather than here, so that there is still exactly
    // one solution per loop -- see the note there. The ninth gate term lives in updateReadyToScore,
    // conditioned on this state, so that the term is logged alongside the other eight rather than
    // hidden inside a handler.
    runScoringChain();
  }

  /** Operator override that fires regardless of the readiness gate. */
  private void scoringForced() {
    // IMPLEMENTATION (was TODO: "Aim using lastShot but feed UNCONDITIONALLY. Log a loud warning,
    // so that when someone reviews the match log they can tell forced shots apart from gated ones
    // -- otherwise a forced miss looks like a solver bug.")
    intake.setWantedState(Intake.WantedState.STOW);

    // Still aims. Forcing a shot skips the GATES, not the SOLVER: the operator is overriding a
    // judgement about whether the robot is ready, not asking it to fire in an arbitrary direction.
    // When there is no usable solution there is nothing to aim at, and the turret is left wherever
    // it is rather than being slewed to ShotParameters.invalid()'s field angle of zero.
    if (lastShot.isUsable()) {
      aimAt(lastShot.turretFieldAngle(), lastShot.turretAngularVelocityRadPerSec());
      shooter.setGoal(lastShot.flywheelRpm());
    } else {
      shooter.setGoal(Constants.Shooter.kIdleRpm.get());
    }

    // Unconditional. This is the one place in the robot where the belt runs without the gate
    // agreeing, and the alert and the ForcedShot log raised in applyState() are what make that
    // visible afterwards.
    indexer.setWantedState(Indexer.WantedState.FEED);
    floorRoller.setWantedState(FloorRoller.WantedState.FEED);
  }

  /** Lobs a piece toward a teammate or a staging area. */
  private void passing() {
    // IMPLEMENTATION (was TODO: "Aim at the pass target with a lower, separately-tuned RPM." and
    // "Add a pass-target translation to Constants.Field and choose a sensible destination once the
    // REBUILT field layout is known.")
    //
    // The constants now exist -- Constants.Field.kPassTargetX/Y and
    // Constants.Superstructure.kPassRpm -- and all three are deliberately zero, carrying TODO:
    // MEASURE. A pass destination is a tactical choice about a field nobody has seen, and this
    // project does not guess field-dependent numbers.
    //
    // Zero is safe rather than merely unmeasured: Shooter's zero-goal guard resolves a zero RPM to
    // IDLE instead of spinning up against an unreachable target and faulting, so PASSING aims and
    // declines to fire until somebody fills the numbers in.
    //
    // No solver call. A pass is a lob at a fixed speed, not a ballistic solve: the aim is simply
    // the bearing from the robot to the pass target, and running it through the shot tables would
    // index them at a distance they were never measured at.
    Translation2d target = Constants.Field.passTranslation();
    Translation2d robot = RobotState.getInstance().getEstimatedPose().getTranslation();

    intake.setWantedState(Intake.WantedState.STOW);
    aimAt(target.minus(robot).getAngle(), 0.0);
    shooter.setGoal(Constants.Superstructure.kPassRpm.get());

    // readyToPass, not readyToScore. The difference is term 1: there is no shot solution to check
    // because there was no solve, and gating on one would make a pass wait on a solution to the
    // GOAL -- which on this branch never arrives, so the pass could never leave. Every other term
    // still applies.
    boolean feed = readyToPass;
    indexer.setWantedState(feed ? Indexer.WantedState.FEED : Indexer.WantedState.HOLD);
    floorRoller.setWantedState(feed ? FloorRoller.WantedState.FEED : FloorRoller.WantedState.IDLE);
  }

  /** The turret cannot reach the aim point, so ask the drivetrain for help. */
  private void waitingForTurret() {
    // IMPLEMENTATION (was TODO: "Set requestedDriveHeading to the field angle that would bring the
    // aim point back inside the turret's travel, hold the flywheel at the solved RPM, and keep the
    // indexer in HOLD. DriveCommands reads getRequestedDriveHeading() and blends it into the
    // driver's rotation input, rather than overriding it -- the driver keeps translation authority
    // throughout.")
    intake.setWantedState(Intake.WantedState.STOW);
    floorRoller.setWantedState(FloorRoller.WantedState.IDLE);
    indexer.setWantedState(Indexer.WantedState.HOLD);

    if (!lastShot.isUsable()) {
      // Reached with no solution at all -- today, that is every scoring request, because the shot
      // tables are zeros. There is no aim point, so there is nothing for the drivetrain to turn
      // towards and asking it to hold a heading would take rotation off the driver for no reason.
      // The wheel is kept at its idle speed so the robot is ready the moment a solution appears.
      shooter.setGoal(Constants.Shooter.kIdleRpm.get());
      turret.setWantedState(Turret.WantedState.STOW);
      requestedDriveHeading = Optional.empty();
      return;
    }

    // Hold the solved speed. The flywheel is not the thing that cannot reach, and spinning it down
    // while the chassis rotates would put the spin-up back into the shot.
    shooter.setGoal(lastShot.flywheelRpm());

    // Keep asking for the aim point. The turret refuses on its own -- that refusal is what put the
    // machine in this state -- and re-requesting is what makes it start tracking again the instant
    // the chassis rotation brings the target inside the travel.
    aimAt(lastShot.turretFieldAngle(), lastShot.turretAngularVelocityRadPerSec());
    requestedDriveHeading = headingBringingAimIntoTravel(lastShot.turretFieldAngle());
  }

  /** Lets a characterization routine drive exactly one mechanism. */
  private void characterizing() {
    // IMPLEMENTATION (was TODO: "Pass through to whichever subsystem CharacterizationCommands
    // selected, and command every other subsystem to IDLE so nothing else moves during a SysId
    // sweep.")
    //
    // The selection arrives through setCharacterizationTarget; see CharacterizationTarget for why
    // it had to be invented. The voltages themselves are pushed straight into each mechanism by the
    // routines in CharacterizationCommands, which is why nothing here sets one: this handler's only
    // job is to make sure the other six mechanisms are still.
    intake.setWantedState(
        characterizationTarget == CharacterizationTarget.INTAKE_PIVOT
            ? Intake.WantedState.CHARACTERIZE
            : Intake.WantedState.IDLE);
    floorRoller.setWantedState(
        characterizationTarget == CharacterizationTarget.FLOOR_ROLLER
            ? FloorRoller.WantedState.CHARACTERIZE
            : FloorRoller.WantedState.IDLE);
    indexer.setWantedState(
        characterizationTarget == CharacterizationTarget.INDEXER
            ? Indexer.WantedState.CHARACTERIZE
            : Indexer.WantedState.IDLE);
    turret.setWantedState(
        characterizationTarget == CharacterizationTarget.TURRET
            ? Turret.WantedState.CHARACTERIZE
            : Turret.WantedState.IDLE);
    shooter.setWantedState(
        characterizationTarget == CharacterizationTarget.SHOOTER
            ? Shooter.WantedState.CHARACTERIZE
            : Shooter.WantedState.COAST);

    // The drivetrain is held still for everything except its own sweep. A flywheel kV fitted while
    // the robot rolled across the shop is a fit of the wrong system, and nothing in the resulting
    // numbers looks wrong afterwards.
    //
    // Skipped for DRIVE because the drivetrain's SysId routine owns the subsystem through the
    // command scheduler for the duration of its sweep, and writing a zero here every loop would
    // fight it.
    if (characterizationTarget != CharacterizationTarget.DRIVE) {
      drive.stop();
    }
  }

  /** Everything stopped after a critical fault. */
  private void faulted() {
    // IMPLEMENTATION (was TODO: "Command every subsystem to IDLE. Only a HOME request clears
    // FAULTED.")
    //
    // The latch itself is in handleStateTransition; this handler only parks the mechanisms.
    intake.setWantedState(Intake.WantedState.IDLE);
    floorRoller.setWantedState(FloorRoller.WantedState.IDLE);
    indexer.setWantedState(Indexer.WantedState.IDLE);
    turret.setWantedState(Turret.WantedState.IDLE);

    // COAST rather than IDLE, matching Shooter's own FAULTED handler: if something is mechanically
    // wrong, holding a wheel against a stator limit for the rest of the match is how a motor is
    // destroyed.
    shooter.setWantedState(Shooter.WantedState.COAST);

    // The DRIVETRAIN IS DELIBERATELY NOT TOUCHED. A jammed intake or an unhomed turret is not a
    // reason to strand the robot on the field -- it can still play defence, still cross the line,
    // and still drive back to the wall at the end of the match. Parking the drivetrain here would
    // turn every mechanism fault into a dead robot.
    requestedDriveHeading = Optional.empty();
  }

  /**
   * Commands the turret and the chain behind a gated shot.
   *
   * <p>Shared by {@link #scoringOnTheMove()} and {@link #scoringStationary()}, which differ only in
   * which solve fed {@code lastShot} and in the extra chassis-speed term the gate applies. Written
   * once because two copies of a feed rule are how one of them gets a tolerance change and the
   * other does not.
   *
   * <p><b>Outputs:</b> aims the turret, commands the flywheel, and feeds only when the gate agrees.
   */
  private void runScoringChain() {
    // The intake stays IN for the whole shot, so nothing can enter the turret's arc mid-shot. This
    // is the interlock's quiet half: the loud half refuses a deploy, this one simply never asks.
    intake.setWantedState(Intake.WantedState.STOW);

    aimAt(lastShot.turretFieldAngle(), lastShot.turretAngularVelocityRadPerSec());
    shooter.setGoal(lastShot.flywheelRpm());

    // Read once and used for both, so the belt and the roller cannot disagree within a loop about
    // whether a shot is happening.
    boolean feed = readyToScore;
    indexer.setWantedState(feed ? Indexer.WantedState.FEED : Indexer.WantedState.HOLD);
    floorRoller.setWantedState(feed ? FloorRoller.WantedState.FEED : FloorRoller.WantedState.IDLE);
  }

  /**
   * Points the turret at a field-relative bearing.
   *
   * @param fieldAngle the field-relative azimuth to aim along
   * @param fieldAngularVelocityRadPerSec how fast that azimuth is sweeping, in radians per second
   *     <p><b>Outputs:</b> sets the turret's goal, which also requests {@code AIM}.
   */
  private void aimAt(Rotation2d fieldAngle, double fieldAngularVelocityRadPerSec) {
    // A one-line wrapper on purpose. Turret.setGoal also flips the turret's wanted state to AIM as
    // a side effect, and having every aiming state call one named method keeps that side effect in
    // one place rather than making each caller remember it.
    turret.setGoal(fieldAngle, fieldAngularVelocityRadPerSec);
  }

  /**
   * Finds the chassis heading that brings an aim point inside the turret's travel.
   *
   * @param aimFieldAngle the field-relative bearing the turret has been asked for
   * @return the heading to request, or empty when the current heading already works
   */
  private Optional<Rotation2d> headingBringingAimIntoTravel(Rotation2d aimFieldAngle) {
    // The turret's robot-relative goal is (aim - heading), the same expression Turret.solveAim
    // uses. Rotating the chassis by delta therefore moves the required turret angle by -delta, so
    // the legal headings are heading + delta for every delta satisfying
    //     aim - heading - delta + 2*pi*k  in  [min, max]
    // for some whole number of turns k. The smallest such delta is the least the chassis has to
    // rotate for the turret to be able to do its job.
    //
    // The turns matter and cannot be replaced with an angle-modulus: the travel is +-200 degrees,
    // which is WIDER than a full circle, so one required angle can be legal on one branch and
    // outside the stops on another. Folding everything into (-pi, pi] first would silently pick the
    // wrong branch.
    double min = Constants.Turret.kMinAngleRad.get();
    double max = Constants.Turret.kMaxAngleRad.get();
    double heading = RobotState.getInstance().getRotation().getRadians();
    double required = aimFieldAngle.getRadians() - heading;

    double bestDelta = 0.0;
    double bestMagnitude = Double.POSITIVE_INFINITY;
    for (int turns = -2; turns <= 2; turns++) {
      double branch = required + 2.0 * Math.PI * turns;
      // Rotating by anything in [branch - max, branch - min] puts this branch inside the travel.
      // Clamping zero into that window is the smallest rotation that does it, and is exactly zero
      // when the branch is already legal.
      double delta = MathUtil.clamp(0.0, branch - max, branch - min);
      if (Math.abs(delta) < bestMagnitude) {
        bestMagnitude = Math.abs(delta);
        bestDelta = delta;
      }
    }

    // No rotation needed. The turret is refusing for the other reason it can refuse -- the deployed
    // intake is in the way of the sweep -- and turning the chassis would not help. Returning empty
    // leaves rotation entirely with the driver instead of quietly holding them at their current
    // heading.
    if (bestMagnitude < 1e-9) {
      return Optional.empty();
    }
    return Optional.of(Rotation2d.fromRadians(heading + bestDelta));
  }

  /**
   * Recomputes the readiness gate and publishes every term that feeds it.
   *
   * <p><b>Outputs:</b> writes {@code readyToScore} and logs each term under {@code
   * Superstructure/ReadyTerms/*}.
   */
  private void updateReadyToScore() {
    RobotState state = RobotState.getInstance();

    // The eight terms of plan section 6.5, in the order the plan lists them.
    boolean shotUsable = lastShot.isUsable();
    boolean turretAtGoal = turret.atGoal();
    boolean flywheelReady = shooter.willBeAtSpeedIn(Constants.getLoopPeriodSec() * 2.0);
    boolean hasPiece = indexer.hasPiece();
    boolean visionFresh =
        state.getVisionStalenessSec() < Constants.Shot.kMaxVisionStalenessSec.get();
    boolean poseConfident =
        state.getPoseStdDevEstimate() < Constants.Shot.kMaxPoseStdDevMeters.get();
    boolean accelWithinLimit =
        state.getFieldRelativeAccel().getNorm()
            < Constants.Shot.kMaxChassisAccelForShotMetersPerSec2.get();
    boolean noCriticalFault = !FaultReporter.getInstance().anyCritical();

    // The ninth, which applies to the stationary shot only. Asking for a stationary shot is asking
    // the solver to leave the velocity compensation out of the answer; a robot still creeping into
    // position has put it back in without telling anyone.
    ChassisSpeeds speeds = state.getFieldRelativeSpeeds();
    boolean chassisStationary =
        systemState != SystemState.SCORING_STATIONARY
            || Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                < Constants.Superstructure.kMaxChassisSpeedForStationaryShotMetersPerSec.get();

    // EVERY term logged separately, and logged unconditionally -- including while the override is
    // active. The question asked at 11 p.m. on a practice field is not "is it ready" but "WHICH
    // term is false", and the question asked after a match is "was the gate too tight, or did the
    // operator override a correct refusal". Short-circuiting the computation when
    // ignoreShotTolerances is set would answer the first and destroy the second.
    Logger.recordOutput("Superstructure/ReadyTerms/ShotUsable", shotUsable);
    Logger.recordOutput("Superstructure/ReadyTerms/TurretAtGoal", turretAtGoal);
    Logger.recordOutput("Superstructure/ReadyTerms/FlywheelReady", flywheelReady);
    Logger.recordOutput("Superstructure/ReadyTerms/HasPiece", hasPiece);
    Logger.recordOutput("Superstructure/ReadyTerms/VisionFresh", visionFresh);
    Logger.recordOutput("Superstructure/ReadyTerms/PoseConfident", poseConfident);
    Logger.recordOutput("Superstructure/ReadyTerms/AccelWithinLimit", accelWithinLimit);
    Logger.recordOutput("Superstructure/ReadyTerms/NoCriticalFault", noCriticalFault);
    Logger.recordOutput("Superstructure/ReadyTerms/ChassisStationary", chassisStationary);
    Logger.recordOutput("Superstructure/ReadyTerms/TolerancesIgnored", ignoreShotTolerances);

    // Split so the pass gate is the same computation minus one term, rather than a second copy of
    // seven conditions free to drift out of step with these.
    boolean everythingButTheSolution =
        turretAtGoal
            && flywheelReady
            && hasPiece
            && visionFresh
            && poseConfident
            && accelWithinLimit
            && noCriticalFault
            && chassisStationary;

    readyToScore = ignoreShotTolerances || (shotUsable && everythingButTheSolution);
    readyToPass = ignoreShotTolerances || everythingButTheSolution;
    Logger.recordOutput("Superstructure/ReadyToPass", readyToPass);
  }

  /**
   * Whether the turret currently occupies, or would sweep through, the intake's arc.
   *
   * @return true when deploying the intake would collide with the turret
   */
  private boolean turretInIntakeConflictArc() {
    // IMPLEMENTATION (was TODO: "TurretUnwrapper.passesThroughArc(turret.getPositionRad(), the
    // turret's commanded position, Constants.Turret.kIntakeConflictMinRad.get(),
    // Constants.Turret.kIntakeConflictMaxRad.get()).")
    //
    // Where it IS and where it is GOING, not just where it is. A turret currently clear of the arc
    // but part-way through a sweep across it would pass a position-only test, and the intake would
    // deploy into its path -- the arm takes most of a second to swing and the turret crosses the
    // arc in a fraction of that.
    //
    // passesThroughArcWrapped rather than passesThroughArc: the arc is physical space and this
    // turret's travel is wider than a full circle, so -180 degrees and +180 degrees are the same
    // direction and only one of them is inside an arc written as 150-210 degrees. The raw test
    // opened this guard for a turret standing in the intake's path.
    return TurretUnwrapper.passesThroughArcWrapped(
        turret.getPositionRad(),
        turret.getCommandedPositionRad(),
        Constants.Turret.kIntakeConflictMinRad.get(),
        Constants.Turret.kIntakeConflictMaxRad.get());
  }

  /**
   * The full readiness gate for taking a shot.
   *
   * @return true when every precondition for an accurate shot holds, or unconditionally when
   *     tolerances have been overridden
   */
  @AutoLogOutput(key = "Superstructure/ReadyToScore")
  public boolean isReadyToScore() {
    // IMPLEMENTATION (was TODO: the eight-term AND, "Return true immediately when
    // ignoreShotTolerances", and "LOG EACH TERM SEPARATELY under Superstructure/ReadyTerms/*".)
    //
    // The computation lives in updateReadyToScore, which periodic() runs once per loop before the
    // handlers, and this returns what it decided. Same reasoning as Indexer.hasPiece(): the two
    // scoring handlers, this annotation, and the per-term log must agree within a loop, and a
    // getter that recomputed would give different callers different answers the moment any input
    // moved mid-loop -- which is precisely what would make the log stop explaining the behaviour.
    return readyToScore;
  }

  /**
   * Requests a new robot-level behavior.
   *
   * @param wanted the behavior to request
   *     <p><b>Outputs:</b> sets the wanted state.
   */
  public void setWantedState(WantedState wanted) {
    // IMPLEMENTATION (was TODO: "wantedState = wanted.")
    wantedState = wanted;

    // Beyond the TODO, and necessary: a HOME request must re-arm the mechanism home requests even
    // when the machine is ALREADY in HOMING. Without this, an operator pressing HOME again after a
    // failed attempt would see nothing happen -- the state does not change, so no onEnter fires,
    // so nothing re-issues the request. Both mechanism FSMs carry the same latch for the same
    // reason.
    if (wanted == WantedState.HOME) {
      homeRequested = true;
      homeRequestPending = true;
    }
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
    // IMPLEMENTATION (was TODO: "Replace the 3.0 s literal below with a tunable timeout from
    // Constants.")
    //
    // Constants.Auto.kScoreTimeoutSec already existed for exactly this, so no new tunable was
    // added. It is read inside Commands.defer rather than when the command is built, because a
    // command built at robot init would otherwise capture the boot value and go on using it after
    // somebody retuned the number between matches -- a tunable that silently is not one.
    //
    // The timeout is the point of this command, not a safety net: the indexer reports empty from a
    // load predicate, and a piece that jams on the way out never produces the falling edge this
    // waits for. Without a bound, one jam would hold the auto in a scoring state for the rest of
    // the fifteen seconds.
    return Commands.sequence(
        setWantedStateCommand(WantedState.AUTO_SCORE_ON_MOVE),
        Commands.defer(
            () ->
                Commands.waitUntil(() -> !indexer.hasPiece())
                    .withTimeout(Constants.Auto.kScoreTimeoutSec.get()),
            Set.of()),
        setWantedStateCommand(WantedState.DEFAULT));
  }

  /**
   * Toggles the operator's "force shoot" override.
   *
   * @param ignore true to bypass every readiness tolerance
   *     <p><b>Outputs:</b> stores the override flag.
   */
  public void setIgnoreShotTolerances(boolean ignore) {
    // IMPLEMENTATION (was TODO: "ignoreShotTolerances = ignore.")
    ignoreShotTolerances = ignore;
  }

  /**
   * Selects which mechanism {@link SystemState#CHARACTERIZING} passes through to.
   *
   * @param target the mechanism a characterization routine is about to drive
   *     <p><b>Outputs:</b> stores the selection; everything else is commanded still while
   *     characterizing.
   */
  public void setCharacterizationTarget(CharacterizationTarget target) {
    characterizationTarget = target;
  }

  /**
   * Parks the robot when it is disabled.
   *
   * <p>Added beyond the plan's method list, and called from {@code RobotContainer.onDisable()}.
   * Three things have to be cleared here rather than left to the next enable.
   *
   * <p><b>Outputs:</b> requests {@link WantedState#IDLE}, drops the force-shoot override, forgets
   * the shot solution, and releases any requested drive heading.
   */
  public void onDisable() {
    // The force-shoot override is the important one, and it is a genuine safety issue rather than
    // tidiness. It is a toggle an operator holds on during a scoring run; if it survives a disable,
    // the robot comes back at the start of the next match with the readiness gate already bypassed
    // and nothing on the dashboard that the drive team was looking at when they last saw it.
    ignoreShotTolerances = false;

    // A stale solution would otherwise be acted on for the first loop after re-enable, before
    // periodic() has re-solved against wherever the robot has been carried to in the meantime.
    lastShot = ShotParameters.invalid();

    requestedDriveHeading = Optional.empty();

    // IDLE rather than DEFAULT: DEFAULT holds the flywheel at its idle speed, and a robot on a cart
    // with a spinning flywheel is how somebody loses a finger.
    setWantedState(WantedState.IDLE);
  }

  /**
   * The heading the superstructure would like the drivetrain to take.
   *
   * @return the requested field heading while waiting for the turret, or empty when the drivetrain
   *     should be left entirely to the driver
   */
  public Optional<Rotation2d> getRequestedDriveHeading() {
    // IMPLEMENTATION (was TODO: "return requestedDriveHeading.")
    return requestedDriveHeading;
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
   * How many shots have been taken with the readiness gate bypassed since boot.
   *
   * <p>Added beyond the plan's method list so that {@code forceShootBypassesGatesButStillAims} can
   * assert the bypass happened rather than infer it, and so a post-match review has a number rather
   * than a shape on a timeline.
   *
   * @return the forced-shot count
   */
  public int getForcedShotCount() {
    return forcedShotCount;
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
