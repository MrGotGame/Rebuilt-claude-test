package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.CharacterizationCommands;
import frc.robot.commands.CharacterizationCommands.ShotTableSample;
import frc.robot.shooting.ShootingSolver;
import frc.robot.shooting.ShotFlightModel;
import frc.robot.shooting.TurretUnwrapper;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.floorroller.FloorRoller;
import frc.robot.subsystems.floorroller.FloorRollerIO;
import frc.robot.subsystems.floorroller.FloorRollerIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.IdealPivotIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.QuestNavIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.FaultReporter;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Transition tests for the cross-mechanism state machine.
 *
 * <p>These tests build the superstructure on top of simulated IO implementations and step it loop
 * by loop, asserting which states it reaches and, just as importantly, which it refuses to reach.
 *
 * <p>Unlike the two pure-math test classes, these need the WPILib HAL because the subsystems use
 * {@code Timer} and the command scheduler.
 *
 * <p><b>Two things about this suite are worth reading before changing it.</b>
 *
 * <p><b>The readiness gate cannot open here, and that is correct.</b> {@code
 * Constants.Shot.kRpmPerMeterPerSec} and {@code kRpmIntercept} are both still zero awaiting M11, so
 * every solved shot carries a flywheel speed of zero, and {@code Shooter.willBeAtSpeedIn} refuses a
 * zero goal by design. Term 3 of the gate is therefore false in every case below. {@link
 * #theReadinessGateCannotOpenUntilTheShotTablesAreMeasured} records that as a named limit with its
 * cause rather than leaving it as a mystery, and {@link #forceShootBypassesGatesButStillAims} is
 * what exercises the feed path meanwhile. The gate failing closed is the safe direction to be stuck
 * in; a gate stuck open would be the bug.
 *
 * <p><b>{@code RobotState} and {@code FaultReporter} are process-wide singletons</b> and this suite
 * runs in the same JVM as every other. The pose is reset in {@link #setUp()} so nothing is
 * inherited, and the one fault this suite registers is a class-level registration driven by a flag
 * that {@link #tearDown()} clears, so it cannot leak into another class as an active fault.
 */
public class SuperstructureFsmTest {
  private static final double LOOP_SEC = 0.02;

  /**
   * Whether this suite's injected critical fault is currently active.
   *
   * <p>Static, and registered exactly once, because {@code FaultReporter} is a process-wide
   * singleton with no way to unregister. Registering per test would pile up one dead fault per
   * case; registering a live one and forgetting to clear it would make every later test class in
   * the JVM see a robot with a critical fault. Neither is acceptable, so there is one registration
   * whose supplier reads this flag, and {@link #tearDown()} always clears it.
   */
  private static boolean injectedFaultActive = false;

  /**
   * Whether this suite's injected <b>warning</b>-severity fault is currently active.
   *
   * <p>Separate from the error above because the severity distinction is the thing under test:
   * {@code anyCritical()} must count one and ignore the other.
   */
  private static boolean injectedWarningActive = false;

  private static boolean faultRegistered = false;

  private Drive drive;
  private Intake intake;
  private FloorRoller floorRoller;
  private IndexerIOSim indexerSim;
  private Indexer indexer;
  private Turret turret;
  private Shooter shooter;
  private RecordingVisionIO intakeCameraIO;
  private Vision vision;
  private Superstructure superstructure;

  /**
   * Geometry for the intake {@link #buildRobot} constructs, or null for the robot's own.
   *
   * <p>Added for M10b step 5. The robot's own geometry is still all 0.0 placeholders, on which the
   * turret-conflict band fails closed -- so the turret can never enter its arc, and every interlock
   * assertion would pass whether the interlock worked or not. The interlock cases set this to
   * {@code IdealPivotIO.GEOMETRY}; every other case leaves it null and builds exactly as before.
   */
  private Intake.PivotGeometry intakeGeometry = null;

  /** A camera stand-in that remembers every pipeline it was switched to. */
  private static final class RecordingVisionIO implements VisionIO {
    private final List<Integer> pipelines = new ArrayList<>();

    @Override
    public void setPipeline(int index) {
      pipelines.add(index);
    }
  }

  /** Builds the whole robot over simulated IO, with the HAL up and the clock paused. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();

    // Every mechanism reads RobotState, so the singleton must exist before the first periodic.
    RobotState.initialize(new SwerveDriveKinematics(Drive.getModuleTranslations()));

    // Reset the shared pose rather than inheriting whatever the previous test class left in it.
    // See the class comment: one estimator is shared by the whole suite.
    placeRobotAt(0.0, 0.0);

    if (!faultRegistered) {
      FaultReporter.getInstance()
          .register(
              "SuperstructureFsmTest injected fault", AlertType.kError, () -> injectedFaultActive);
      FaultReporter.getInstance()
          .register(
              "SuperstructureFsmTest injected warning",
              AlertType.kWarning,
              () -> injectedWarningActive);
      faultRegistered = true;
    }
    injectedFaultActive = false;
    injectedWarningActive = false;

    buildRobot(
        new ShootingSolver(),
        new IntakeIOSim(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
  }

  /** Releases the HAL and unregisters every subsystem so cases cannot leak into each other. */
  @AfterEach
  void tearDown() {
    injectedFaultActive = false;
    injectedWarningActive = false;
    unregisterAll();
    CommandScheduler.getInstance().cancelAll();
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  // ------------------------------------------------------------------------------- harness ----

  /**
   * Builds every subsystem and the superstructure over the supplied IO layers.
   *
   * @param solver the shooting solver the superstructure should own
   * @param intakeIo the intake hardware layer
   * @param indexerIo the indexer hardware layer
   * @param turretIo the turret hardware layer
   * @param shooterIo the flywheel hardware layer
   * @param floorRollerIo the floor roller hardware layer
   *     <p><b>Outputs:</b> replaces every field on this test instance.
   */
  private void buildRobot(
      ShootingSolver solver,
      IntakeIO intakeIo,
      IndexerIO indexerIo,
      TurretIO turretIo,
      ShooterIO shooterIo,
      FloorRollerIO floorRollerIo) {
    unregisterAll();

    // Empty module and gyro IO on purpose. The superstructure never reads the drivetrain -- it only
    // publishes a requested heading that DriveCommands blends in -- and an empty ModuleIO registers
    // no odometry signals, which is what keeps PhoenixOdometryThread from starting a background
    // thread inside a unit test.
    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});

    intake = intakeGeometry == null ? new Intake(intakeIo) : new Intake(intakeIo, intakeGeometry);
    floorRoller = new FloorRoller(floorRollerIo);
    indexerSim = indexerIo instanceof IndexerIOSim sim ? sim : null;
    indexer = new Indexer(indexerIo);
    turret = new Turret(turretIo);
    shooter = new Shooter(shooterIo);

    intakeCameraIO = new RecordingVisionIO();
    vision =
        new Vision(
            new VisionIO() {},
            intakeCameraIO,
            new QuestNavIO() {},
            turret::getPositionRad,
            turret::getVelocityRadPerSec,
            intake::isAtIntakingAngle);

    superstructure =
        new Superstructure(drive, intake, floorRoller, indexer, turret, shooter, vision, solver);
  }

  /** Drops every subsystem this test built out of the scheduler. */
  private void unregisterAll() {
    for (Subsystem subsystem :
        new Subsystem[] {drive, intake, floorRoller, indexer, turret, shooter, superstructure}) {
      if (subsystem != null) {
        CommandScheduler.getInstance().unregisterSubsystem(subsystem);
      }
    }
  }

  /**
   * Advances the whole robot by a number of loops.
   *
   * @param loops how many 20 ms iterations to run
   *     <p><b>Outputs:</b> runs each subsystem's periodic that many times and advances the
   *     simulated clock with it.
   */
  private void step(int loops) {
    for (int i = 0; i < loops; i++) {
      // Mechanisms first, superstructure last, matching the command scheduler on the robot: it
      // runs subsystem periodics in registration order and RobotContainer constructs the
      // superstructure after everything it commands. A superstructure running FIRST would see this
      // loop's requests take effect in the same loop, which is a full loop faster than the real
      // robot and would hide any one-loop race.
      intake.periodic();
      floorRoller.periodic();
      indexer.periodic();
      turret.periodic();
      shooter.periodic();
      superstructure.periodic();
      SimHooks.stepTiming(LOOP_SEC);
    }
  }

  /** Steps while feeding a fresh, tight Limelight fix every loop, so the vision terms are true. */
  private void stepWithVision(int loops) {
    for (int i = 0; i < loops; i++) {
      feedFreshVisionFix();
      step(1);
    }
  }

  /**
   * Hands {@code RobotState} a clean Limelight observation at the current instant.
   *
   * <p>The observed pose is the current estimate, so the pose-jump rule accepts it, and the
   * standard deviation is tighter than {@code kMaxPoseStdDevMeters}. That makes gate terms 5 and 6
   * true; every other term is left to the robot.
   */
  private void feedFreshVisionFix() {
    RobotState state = RobotState.getInstance();
    state.addVisionObservation(
        new RobotState.PoseObservation(
            state.getEstimatedPose(),
            Timer.getFPGATimestamp(),
            VecBuilder.fill(0.02, 0.02, 9999.0),
            RobotState.PoseSource.LIMELIGHT_TURRET));
  }

  /** Puts the robot at a field position with zero heading. */
  private void placeRobotAt(double x, double y) {
    RobotState.getInstance()
        .resetPose(
            new Pose2d(x, y, Rotation2d.kZero),
            Rotation2d.kZero,
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            });
  }

  /**
   * A solver with tables that produce a usable solution.
   *
   * <p>The shipped tables are all zeros, which correctly yields an unusable shot for every request
   * -- so a suite using only those could never demonstrate that the machine reaches a scoring state
   * at all. These numbers are plausible rather than measured; nothing here asserts on the values
   * they produce, only on the fact that a solution exists.
   *
   * @return a solver over a non-zero time-of-flight table
   */
  private static ShootingSolver usableSolver() {
    return new ShootingSolver(new double[] {1.0, 4.0, 7.0}, new double[] {0.50, 0.80, 1.10});
  }

  /**
   * A solver that produces a shot the flywheel will actually spin up for.
   *
   * <p>Added for M10b step 7. {@link #usableSolver()} solves, but its RPM fit is still {@code
   * Constants.Shot}'s all-zero one, so every shot asks for 0 RPM and the gate can never open -- see
   * {@link #theReadinessGateCannotOpenUntilTheShotTablesAreMeasured}. The hopper cases need to
   * watch the gate open, close and hold, so this adds a plausible fit through the solver's
   * explicit-RPM constructor. At 3 m it asks for roughly 2,700 RPM.
   *
   * @return a solver over the same tables as {@link #usableSolver()}, with a non-zero RPM fit
   */
  private static ShootingSolver firingSolver() {
    return new ShootingSolver(
        new double[] {1.0, 4.0, 7.0},
        new double[] {0.50, 0.80, 1.10},
        () -> FIRING_RPM_PER_MPS,
        () -> 0.0);
  }

  /**
   * The firing solver's RPM fit: flywheel RPM per meter per second of muzzle speed, zero offset.
   */
  private static final double FIRING_RPM_PER_MPS = 250.0;

  /** The solver the robot {@link #buildFiringRobotUnhomed} built, for its iteration count. */
  private ShootingSolver firingShotSolver;

  /** The simulated flywheel under the robot {@link #buildFiringRobot()} built. */
  private ShooterIOSim firingShooter;

  /**
   * Builds a robot whose gate can open: the firing solver, the ideal intake over the realistic test
   * geometry, and a simulated flywheel the case can inject pieces into. Places it 3 m in front of
   * the goal, where the shot points straight ahead (clear of the turret's conflict arc), and homes
   * it.
   *
   * <p><b>Outputs:</b> replaces every subsystem; sets {@link #firingShooter}.
   */
  private void buildFiringRobot() {
    buildFiringRobotUnhomed(-3.0, IdealPivotIO.GEOMETRY);
    homeAndSettle();
  }

  /** The ideal pivot under the robot {@link #buildFiringRobotUnhomed} built. */
  private IdealPivotIO firingPivot;

  /**
   * Robot x at which the goal lies directly BEHIND it, so the shot needs the turret at 180 degrees
   * -- squarely inside its intake-conflict arc. {@link #buildFiringRobot()}'s -3 m is the mirror
   * image, with the shot straight ahead and clear of the arc.
   */
  private static final double GOAL_BEHIND_X = 3.0;

  /**
   * Builds a firing robot 3 m from the goal but does not home it, so a case can watch boot.
   *
   * <p>Added for M10b step 10, whose cases need the turret aiming through its conflict arc and, for
   * the misconfiguration guard, an intake geometry other than the test default.
   *
   * @param xMeters where to stand on the goal's x axis: -3 faces the goal, +3 has it behind
   * @param geometry the intake pivot geometry
   *     <p><b>Outputs:</b> replaces every subsystem; sets {@link #firingShooter} and {@link
   *     #firingPivot}.
   */
  private void buildFiringRobotUnhomed(double xMeters, Intake.PivotGeometry geometry) {
    intakeGeometry = geometry;
    firingShooter = new ShooterIOSim();
    firingPivot = new IdealPivotIO();
    firingShotSolver = firingSolver();
    buildRobot(
        firingShotSolver,
        firingPivot,
        new IndexerIOSim(),
        new TurretIOSim(),
        firingShooter,
        new FloorRollerIOSim());
    placeRobotAt(xMeters, 0.0);
  }

  /**
   * Whether the turret's commanded angle is inside its intake-conflict arc, on any branch.
   *
   * @return true when the turret is being sent into the arc
   */
  private boolean turretCommandedIntoArc() {
    double commanded = turret.getCommandedPositionRad();
    return TurretUnwrapper.passesThroughArcWrapped(
        commanded,
        commanded,
        Constants.Turret.kIntakeConflictMinRad.get(),
        Constants.Turret.kIntakeConflictMaxRad.get());
  }

  /**
   * Loops allowed for the simulated turret to settle after a large move, about twelve seconds.
   *
   * <p>On its placeholder gains the simulated turret rings about a 180-degree goal for roughly
   * eight seconds (measured while writing the step 10 checkpoint) before it holds inside tolerance.
   * That is M6's to fix by characterization; until then a case that needs the gate open after a big
   * turret move has to wait it out.
   */
  private static final int TURRET_SETTLE_LOOPS = 600;

  /** Loops in one feed-assist rock period. */
  private static int rockPeriodLoops() {
    return (int) Math.round(Constants.Intake.kFeedAssistPeriodSec.get() / LOOP_SEC);
  }

  /**
   * Requests a shot on the move and steps, with fresh vision, until the readiness gate opens.
   *
   * <p><b>Outputs:</b> leaves the robot in SCORING_ON_THE_MOVE with the gate open on the last loop.
   */
  private void openTheGate() {
    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    for (int i = 0; i < 600 && !superstructure.isReadyToScore(); i++) {
      stepWithVision(1);
    }
    assertTrue(
        superstructure.isReadyToScore(),
        "the gate never opened on the firing robot; flywheel at "
            + shooter.getVelocityRpm()
            + " of "
            + shooter.getGoalRpm()
            + " RPM, turret at goal "
            + turret.atGoal());
  }

  /** Loops in {@code Constants.Superstructure.kEmptyConfirmSec}. */
  private static int emptyConfirmLoops() {
    return (int) Math.round(Constants.Superstructure.kEmptyConfirmSec.get() / LOOP_SEC);
  }

  /** Loops that comfortably outlast the robot-level homing timeout. */
  private static int loopsPastHomingTimeout() {
    return (int) (Constants.Superstructure.kHomingTimeoutSec.get() / LOOP_SEC) + 30;
  }

  /** Homes the robot and waits for it to settle into RESTING. */
  private void homeAndSettle() {
    step(loopsPastHomingTimeout());
    assertEquals(
        Superstructure.SystemState.RESTING,
        superstructure.getSystemState(),
        "the robot never finished homing; intake homed="
            + intake.isHomed()
            + " turret homed="
            + turret.isHomed());
  }

  // --------------------------------------------------------------------------------- tests ----

  /**
   * The robot should home itself and settle into RESTING with no input, the intake still stowed.
   *
   * <p>The robot must start with the intake stowed (a rule), and nothing but the first enable or an
   * intake request deploys it -- so at rest after homing, with no enable, it is still stowed.
   */
  @Test
  void startsInHomingAndReachesResting() {
    assertEquals(
        Superstructure.SystemState.HOMING,
        superstructure.getSystemState(),
        "the machine must boot into HOMING; nothing is known about the arm or the turret yet");
    assertFalse(intake.isHomed(), "the intake cannot be homed before it has run");
    assertFalse(turret.isHomed(), "the turret cannot be homed before it has run");

    homeAndSettle();

    assertTrue(intake.isHomed(), "reaching RESTING without a homed intake would be a lie");
    assertTrue(turret.isHomed(), "reaching RESTING without a homed turret would be a lie");
    assertEquals(
        Superstructure.IntakeMode.STOWED,
        superstructure.getIntakeMode(),
        "nothing has enabled the robot or asked for the intake, so it must still be stowed");

    // The robot is asked for nothing and is therefore doing nothing dangerous, but the flywheel is
    // deliberately NOT stopped: RESTING idles it so the next shot does not pay the whole spin-up.
    assertNotEquals(
        Shooter.SystemState.COASTING,
        shooter.getSystemState(),
        "RESTING must idle the flywheel rather than stop it");
  }

  /**
   * Homing must be requested as an edge, not re-issued every loop.
   *
   * <p>This is the specific failure the superstructure's home latch exists to prevent, and it is
   * invisible from the outside: both mechanisms treat {@code setWantedState(HOME)} as a one-shot
   * that clears their homed flag and re-arms a request their transition step consumes ahead of
   * their own timeout. A handler that commanded HOME once per loop -- which is how every other
   * handler in this class commands its mechanisms -- would restart homing on every loop, so neither
   * mechanism could ever finish and neither could ever fault either. The robot would sit in HOMING
   * until the robot-level timeout with no mechanism alert to say which one was stuck.
   */
  @Test
  void homingIsRequestedOnceAndNotReissuedEveryLoop() {
    step(loopsPastHomingTimeout());

    assertTrue(
        intake.isHomed(),
        "the intake never homed, which is what a re-issued HOME request looks like from here");
    assertTrue(turret.isHomed(), "the turret never homed");
    assertNotEquals(
        Superstructure.SystemState.HOMING,
        superstructure.getSystemState(),
        "still homing long after both mechanisms should have finished");
  }

  /**
   * A score request reaches its scoring state with nothing known about what is aboard.
   *
   * <p>Replaces M10's {@code scoreRequestWithoutPieceNeverReachesScoring}, which pinned the piece
   * guard the hopper replan removed (plan section 2.4). The robot has no piece sensor, so "has a
   * piece" cannot be answered; a guard on it would divert every scoring request forever. This case
   * pins the decision so nobody quietly re-adds one.
   */
  @Test
  void scoringDoesNotRequireAPiece() {
    buildRobot(
        usableSolver(),
        new IntakeIOSim(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    stepWithVision(60);

    assertEquals(
        Superstructure.SystemState.SCORING_ON_THE_MOVE,
        superstructure.getSystemState(),
        "a score request with a usable solution must score, with no piece guard in the way");
  }

  /** Scoring on the move must require a usable shot solution. */
  @Test
  void scoreOnMoveRequiresUsableShotSolution() {
    // The SHIPPED solver, reading Constants.Shot's all-zero tables. This is the robot as it exists
    // on this branch, and the behaviour asserted here is the behaviour on a real field until M11
    // measures the tables.
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    stepWithVision(40);

    assertFalse(
        superstructure.getLastShot().isUsable(),
        "with the shot tables all zeros the solver must report an unusable shot");
    assertEquals(
        Superstructure.SystemState.WAITING_FOR_TURRET,
        superstructure.getSystemState(),
        "a scoring request with no usable solution must not enter a scoring state");
    assertNotEquals(
        Indexer.SystemState.FEEDING, indexer.getSystemState(), "fed without a shot solution");
  }

  /** A mechanism that cannot home must fault rather than stall forever. */
  @Test
  void homingTimesOutToFaulted() {
    // A pivot that never settles, so the hard stop is never found. Everything else is healthy, so
    // the fault can only come from the intake.
    buildRobot(
        new ShootingSolver(),
        new IntakeIO() {
          @Override
          public void updateInputs(IntakeIOInputs inputs) {
            inputs.pivotConnected = true;
            inputs.rollerConnected = true;
            inputs.absEncoderConnected = true;
            inputs.pivotVelocityRadPerSec = 100.0;
          }
        },
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());

    // Watched loop by loop from boot rather than sampled at the end, because the defect that hides
    // here is a COUNT and not a state. A jammed pivot that re-enters HOMING after faulting stalls
    // its motor for another full timeout, and by the time any end-of-run sample is taken the
    // machine has settled back into FAULTED and looks perfectly well behaved.
    //
    // Mutation testing is what forced this shape. Reverting the intake's home-latch fix survived
    // BOTH a single end-of-run sample and a window that only started once the robot had already
    // faulted: the extra homing attempt happens early, finishes before either check begins, and
    // leaves no trace in the final state.
    int intakeHomingEntries = intake.getSystemState() == Intake.SystemState.HOMING ? 1 : 0;
    Intake.SystemState previousIntakeState = intake.getSystemState();
    boolean robotHasFaulted = false;

    int watchLoops =
        loopsPastHomingTimeout()
            + (int) (2.5 * Constants.Intake.kHomingTimeoutSec.get() / LOOP_SEC);
    for (int i = 0; i < watchLoops; i++) {
      step(1);

      Intake.SystemState now = intake.getSystemState();
      if (now == Intake.SystemState.HOMING && previousIntakeState != Intake.SystemState.HOMING) {
        intakeHomingEntries++;
      }
      previousIntakeState = now;

      // Once the robot has faulted it must stay faulted, checked every loop rather than once, for
      // the same reason: a machine flickering out of FAULTED passes a single sample half the time,
      // which is worse than no assertion because it still reads as coverage.
      robotHasFaulted |= superstructure.getSystemState() == Superstructure.SystemState.FAULTED;
      if (robotHasFaulted) {
        assertEquals(
            Superstructure.SystemState.FAULTED,
            superstructure.getSystemState(),
            "the robot left FAULTED without being asked to re-home, after " + i + " loops");
      }
    }

    assertEquals(
        Superstructure.SystemState.FAULTED,
        superstructure.getSystemState(),
        "a robot that cannot home must fault, not sit in HOMING for the rest of the match");
    assertFalse(intake.isHomed(), "a timed-out home must not mark the pivot trustworthy");

    // The mechanism must have faulted FIRST, and its alert names the intake; the robot-level fault
    // names nothing.
    assertEquals(
        Intake.SystemState.FAULTED,
        intake.getSystemState(),
        "the intake must latch FAULTED rather than cycle back into HOMING");

    // ONE homing attempt, the one it booted into. Every extra entry is another full kHomingTimeout
    // of driving a jammed pivot into a stop it cannot find, which is precisely what the timeout
    // exists to stop.
    assertEquals(
        1,
        intakeHomingEntries,
        "the intake re-homed after faulting; each retry is another full-timeout stall against a"
            + " jam, and nothing asked for it");
  }

  /** Only an explicit home request may clear a fault. */
  @Test
  void onlyHomeClearsFaulted() {
    buildRobot(
        new ShootingSolver(),
        new IntakeIO() {
          @Override
          public void updateInputs(IntakeIOInputs inputs) {
            inputs.pivotConnected = true;
            inputs.pivotVelocityRadPerSec = 100.0;
          }
        },
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    step(loopsPastHomingTimeout());
    assertEquals(Superstructure.SystemState.FAULTED, superstructure.getSystemState());

    for (Superstructure.WantedState wanted : Superstructure.WantedState.values()) {
      if (wanted == Superstructure.WantedState.HOME) {
        continue;
      }
      superstructure.setWantedState(wanted);
      step(10);
      assertEquals(
          Superstructure.SystemState.FAULTED,
          superstructure.getSystemState(),
          wanted + " cleared a latched fault; only HOME may do that");
    }

    superstructure.setWantedState(Superstructure.WantedState.HOME);
    step(3);
    assertEquals(
        Superstructure.SystemState.HOMING,
        superstructure.getSystemState(),
        "HOME must clear the fault and retry");
  }

  /**
   * The intake and turret must not be able to occupy each other's space -- and, since M10b step 5,
   * the space in question is the STOWED intake, not the deployed one.
   *
   * <p>Rewritten for the pivot-band interlock (plan section 2.4). On this robot the arm is in the
   * turret's sweep only in the few degrees of travel nearest stow, so each half now guards motion
   * into or beside that band, and there is a third direction the M10 version could not have: a
   * DEPLOYED intake must NOT block the turret. That negative case is what stops the interlock ever
   * being "fixed" back into a blanket refusal, which would keep the turret out of its arc for most
   * of a match now that the intake rests deployed.
   *
   * <p>Built on the ideal pivot and the realistic test geometry, because the robot's own geometry
   * is all 0.0 placeholders on which the band fails closed: the turret could never enter its arc,
   * and both directions would pass whether the interlock worked or not.
   */
  @Test
  void intakeAndTurretInterlockHoldsBothWays() {
    double arcMin = Constants.Turret.kIntakeConflictMinRad.get();
    double arcMax = Constants.Turret.kIntakeConflictMaxRad.get();
    double insideArc = 0.5 * (arcMin + arcMax);
    intakeGeometry = IdealPivotIO.GEOMETRY;

    // ---- Direction 1: the intake must not move INTO the band while the turret is in its arc.
    //
    // The turret is a stand-in whose reported angle the case moves by hand, for the reason the M10
    // version recorded: a turret driven there for real is stopped short by its own guard, which
    // tests the other direction. The intake is first taken OUT of the band -- deploying is always
    // allowed -- then the turret is put in the arc, and only then is the intake asked back in.
    double[] turretAt = {0.0};
    buildRobot(
        new ShootingSolver(),
        new IdealPivotIO(),
        new IndexerIOSim(),
        new TurretIO() {
          @Override
          public void updateInputs(TurretIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.absEncoderConnected = true;
            inputs.positionRad = turretAt[0];
            inputs.absolutePositionRad = turretAt[0];
          }
        },
        new ShooterIOSim(),
        new FloorRollerIOSim());
    homeAndSettle();
    assertTrue(intake.isInTurretConflictBand(), "the stowed intake should be in the turret's way");

    superstructure.setIntakeRequested(true);
    step(10);
    assertFalse(
        intake.isInTurretConflictBand(),
        "the intake never left the band, so the stow below would test nothing");

    turretAt[0] = insideArc;
    step(2);
    assertTrue(
        TurretUnwrapper.passesThroughArcWrapped(
            turret.getPositionRad(), turret.getPositionRad(), arcMin, arcMax),
        "the turret is not in the conflict arc, so nothing is being tested; it is at "
            + turret.getPositionRad());

    // STOW, not DEFAULT: since M10b step 7 releasing the intake and resting leaves it DOWN, and
    // STOW is the only request that raises it.
    superstructure.setIntakeRequested(false);
    superstructure.setWantedState(Superstructure.WantedState.STOW);
    boolean sawWaiting = false;
    for (int i = 0; i < 200; i++) {
      step(1);
      assertFalse(
          intake.isInTurretConflictBand(),
          "the intake moved, or was commanded, into the band beside a turret in its arc");
      sawWaiting |= intake.getSystemState() == Intake.SystemState.WAITING_FOR_TURRET;
    }
    assertTrue(sawWaiting, "the refused stow never reported WAITING_FOR_TURRET");

    // Once the turret leaves, the stow the robot asked for completes on its own.
    turretAt[0] = 0.0;
    step(20);
    assertEquals(
        Intake.SystemState.STOWED,
        intake.getSystemState(),
        "the stow did not resume once the turret cleared its arc");

    // ---- Direction 2: the turret must not enter its arc while the intake is in the band.
    buildRobot(
        new ShootingSolver(),
        new IdealPivotIO(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    homeAndSettle();
    assertTrue(
        intake.isInTurretConflictBand(),
        "the intake is not in the band, so the turret guard is not under pressure");

    boolean sawRefusal = false;
    for (int i = 0; i < 300; i++) {
      step(1);
      turret.setGoal(Rotation2d.fromRadians(insideArc), 0.0);

      double commanded = turret.getCommandedPositionRad();
      assertFalse(
          TurretUnwrapper.passesThroughArcWrapped(commanded, commanded, arcMin, arcMax),
          "the turret was commanded into its arc beside a stowed intake, at " + commanded);
      sawRefusal |= turret.getSystemState() == Turret.SystemState.OUT_OF_RANGE;
    }
    assertTrue(
        sawRefusal,
        "the turret never reported OUT_OF_RANGE, so it may have been held back by something other"
            + " than the interlock");

    // ---- Direction 3, the negative case: a DEPLOYED intake does not block the turret.
    superstructure.setIntakeRequested(true);
    step(10);
    assertFalse(intake.isInTurretConflictBand(), "the intake never left the band");

    boolean reachedArc = false;
    for (int i = 0; i < 300 && !reachedArc; i++) {
      step(1);
      turret.setGoal(Rotation2d.fromRadians(insideArc), 0.0);
      double commanded = turret.getCommandedPositionRad();
      reachedArc = TurretUnwrapper.passesThroughArcWrapped(commanded, commanded, arcMin, arcMax);
    }
    assertTrue(
        reachedArc,
        "the turret was kept out of its arc while the intake was deployed -- the M10 interlock's"
            + " mistake, which would block it for most of a match now that the intake rests down");
  }

  /** Entry and exit actions must fire exactly once per transition. */
  @Test
  void entryAndExitActionsFireExactlyOnce() {
    // The observable entry and exit action is the intake camera's pipeline, which the plan assigns
    // to the superstructure: game pieces while the intake is out, AprilTags whenever it is not.
    // Counting the switches is a side-effect count rather than a test hook bolted onto production
    // code, and a pipeline switch costs about 100 ms, so a repeated one is a real cost and not just
    // untidy.

    // ONE call already, from the constructor. onEnter never fires for the state a machine boots
    // into, because no transition occurs, so a boot entry action has to be run explicitly. Without
    // it, a robot rebooted after an intake run comes up with the camera still on the game-piece
    // pipeline, contributing no tags at all, and nothing says so.
    assertEquals(
        List.of(Constants.Vision.kAprilTagPipelineIndex),
        intakeCameraIO.pipelines,
        "the boot state's entry action did not run in the constructor");

    homeAndSettle();
    assertEquals(
        1,
        intakeCameraIO.pipelines.size(),
        "homing and stowing must not touch the camera pipeline");

    // Since M10b step 7 intaking is the intake REGION, not a SystemState, and the pipeline switches
    // on the edges of IntakeMode.INTAKING. Held for 200 loops to prove the switch is an edge and
    // not a per-loop reassertion.
    superstructure.setIntakeRequested(true);
    step(200);
    assertEquals(
        Superstructure.IntakeMode.INTAKING,
        superstructure.getIntakeMode(),
        "this case needs the intake region to sit in INTAKING");
    assertEquals(
        List.of(Constants.Vision.kAprilTagPipelineIndex, Constants.Vision.kGamePiecePipelineIndex),
        intakeCameraIO.pipelines,
        "starting to intake must switch to the game-piece pipeline exactly once, however long it is"
            + " held");

    superstructure.setIntakeRequested(false);
    step(200);
    assertEquals(
        List.of(
            Constants.Vision.kAprilTagPipelineIndex,
            Constants.Vision.kGamePiecePipelineIndex,
            Constants.Vision.kAprilTagPipelineIndex),
        intakeCameraIO.pipelines,
        "stopping intaking must switch back to AprilTags exactly once -- the intake stays down, but"
            + " it is no longer looking for pieces");
  }

  /** Force-shoot must bypass the readiness gate but still aim. */
  @Test
  void forceShootBypassesGatesButStillAims() {
    buildRobot(
        usableSolver(),
        new IntakeIOSim(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    step(40);

    // No vision has ever been fed in this case, so the pose is as stale as it can be and terms 5
    // and 6 are false on top of the flywheel term. The gate is correctly shut.
    assertEquals(
        Superstructure.SystemState.SCORING_ON_THE_MOVE,
        superstructure.getSystemState(),
        "with a usable solution the machine must enter the scoring state");
    assertFalse(superstructure.isReadyToScore(), "the gate must be shut on a stale pose");
    assertNotEquals(
        Indexer.SystemState.FEEDING,
        indexer.getSystemState(),
        "the belt fed with the gate shut and no override");

    superstructure.setIgnoreShotTolerances(true);
    step(5);

    assertTrue(
        superstructure.isReadyToScore(),
        "the override must open the gate regardless of how stale the pose is");
    assertEquals(
        Indexer.SystemState.FEEDING,
        indexer.getSystemState(),
        "the override opened the gate but nothing fed");

    // ... and it still AIMS. Forcing a shot skips the gates, not the solver: an override that also
    // abandoned the aim would fire in whatever direction the turret happened to be pointing, which
    // is not what an operator overriding a tolerance is asking for.
    assertTrue(
        turret.getSystemState() == Turret.SystemState.AIMING
            || turret.getSystemState() == Turret.SystemState.UNWRAPPING,
        "the turret stopped aiming under the override; it was " + turret.getSystemState());
    assertTrue(
        superstructure.getLastShot().isUsable(),
        "this case is only meaningful while a usable solution exists to aim at");
  }

  /**
   * The dedicated forced-scoring state feeds unconditionally and is counted.
   *
   * <p>Separate from the override toggle above because they are two different routes to the same
   * bypass and a log reviewer has to be able to see either one. The count is what turns "the robot
   * missed" into "the robot missed four shots that the gate had refused".
   */
  @Test
  void forcedScoringStateFeedsAndIsCounted() {
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();

    assertEquals(0, superstructure.getForcedShotCount(), "nothing forced anything yet");

    superstructure.setWantedState(Superstructure.WantedState.SCORE_FORCED);
    step(40);

    assertEquals(
        Superstructure.SystemState.SCORING_FORCED,
        superstructure.getSystemState(),
        "SCORE_FORCED must reach SCORING_FORCED even with no usable solution");
    assertEquals(
        Indexer.SystemState.FEEDING,
        indexer.getSystemState(),
        "the forced state must feed unconditionally");
    assertEquals(
        1,
        superstructure.getForcedShotCount(),
        "entering the forced state once must count exactly one forced shot");

    // Held, not re-entered: the count is per entry, not per loop.
    step(100);
    assertEquals(1, superstructure.getForcedShotCount(), "the forced-shot count drifted");
  }

  /** Disabling must leave the machine in a safe state. */
  @Test
  void disablingClearsWantedStateSafely() {
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();

    superstructure.setIgnoreShotTolerances(true);
    superstructure.setWantedState(Superstructure.WantedState.SCORE_FORCED);
    step(20);
    assertEquals(Indexer.SystemState.FEEDING, indexer.getSystemState(), "precondition: feeding");

    superstructure.onDisable();
    step(10);

    assertEquals(
        Superstructure.SystemState.IDLE,
        superstructure.getSystemState(),
        "a disabled robot must park, not hold whatever it was doing");

    // Nothing is left commanded. The flywheel is the one that matters: IDLE on this machine means
    // COAST on the shooter, because a robot on a cart with a spinning flywheel is how somebody
    // loses a finger. DEFAULT would have left it at the idle speed.
    assertEquals(Shooter.SystemState.COASTING, shooter.getSystemState(), "flywheel left spinning");
    assertEquals(Indexer.SystemState.IDLE, indexer.getSystemState(), "indexer left commanded");
    assertEquals(
        FloorRoller.SystemState.IDLE, floorRoller.getSystemState(), "floor roller left commanded");
    assertEquals(Turret.SystemState.IDLE, turret.getSystemState(), "turret left commanded");
    assertEquals(Intake.SystemState.IDLE, intake.getSystemState(), "intake left commanded");

    // And no goal leaks into the next enable. The force-shoot override is the dangerous one: it is
    // a toggle an operator holds on during a scoring run, and if it survived a disable the robot
    // would come back for the next match with the readiness gate already bypassed and nothing on
    // the dashboard that anybody was looking at when they last saw it.
    assertFalse(
        superstructure.isReadyToScore(),
        "the force-shoot override survived a disable and the gate is still open");
    assertTrue(
        superstructure.getRequestedDriveHeading().isEmpty(),
        "a requested drive heading survived a disable");
  }

  /**
   * Every {@code SystemState} is reachable. Clause 2 of the per-mechanism checkpoint, applied to
   * the machine that coordinates them.
   *
   * <p>FAULTED is deliberately not visited here: reaching it needs hardware that cannot home, so it
   * is covered by {@link #homingTimesOutToFaulted} and {@link
   * #deadMechanismsStillWalkTheStatesButNothingReportsReady} instead. Every other state is reached
   * in one run.
   *
   * <p>This case is the tenth FSM test the M10 checkpoint asks for and the skeleton did not ship.
   * The other nine are named placeholders; this one is required by the project's own testing
   * standard for every state machine and was simply missing.
   */
  @Test
  void everySystemStateIsReachable() {
    buildRobot(
        usableSolver(),
        new IntakeIOSim(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    placeRobotAt(-3.0, 0.0);

    assertEquals(Superstructure.SystemState.HOMING, superstructure.getSystemState());
    homeAndSettle();
    assertEquals(Superstructure.SystemState.RESTING, superstructure.getSystemState());

    // STOW is also a route to RESTING. (INTAKING and INDEXING are gone since M10b step 7: intaking
    // is the other region of the machine, covered by its own cases.)
    superstructure.setWantedState(Superstructure.WantedState.STOW);
    step(10);
    assertEquals(Superstructure.SystemState.RESTING, superstructure.getSystemState());

    superstructure.setWantedState(Superstructure.WantedState.EJECT);
    step(10);
    assertEquals(Superstructure.SystemState.EJECTING, superstructure.getSystemState());

    superstructure.setWantedState(Superstructure.WantedState.SPIN_UP);
    step(10);
    assertEquals(Superstructure.SystemState.SPINNING_UP, superstructure.getSystemState());

    superstructure.setWantedState(Superstructure.WantedState.PASS);
    step(10);
    assertEquals(Superstructure.SystemState.PASSING, superstructure.getSystemState());

    superstructure.setWantedState(Superstructure.WantedState.CHARACTERIZE);
    step(10);
    assertEquals(Superstructure.SystemState.CHARACTERIZING, superstructure.getSystemState());

    // Back to a scoring posture.
    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    step(30);
    assertEquals(Superstructure.SystemState.SCORING_ON_THE_MOVE, superstructure.getSystemState());

    superstructure.setWantedState(Superstructure.WantedState.SCORE_STATIONARY);
    step(30);
    assertEquals(Superstructure.SystemState.SCORING_STATIONARY, superstructure.getSystemState());

    superstructure.setWantedState(Superstructure.WantedState.SCORE_FORCED);
    step(10);
    assertEquals(Superstructure.SystemState.SCORING_FORCED, superstructure.getSystemState());

    // Out of range: the shot stops being usable, so there is nothing to aim at and the machine
    // holds rather than scoring. This is the same branch the shipped all-zero tables take, reached
    // here through distance instead.
    placeRobotAt(-40.0, 0.0);
    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    step(30);
    assertFalse(superstructure.getLastShot().isUsable(), "40 m must be out of range");
    assertEquals(Superstructure.SystemState.WAITING_FOR_TURRET, superstructure.getSystemState());

    superstructure.setWantedState(Superstructure.WantedState.IDLE);
    step(10);
    assertEquals(Superstructure.SystemState.IDLE, superstructure.getSystemState());

    // One forced shot in that whole tour, and exactly one. The tour passed through four scoring
    // states and only one of them bypassed the gate; a counter that rose on any of the others
    // would turn the post-match question "how many shots did the operator force" into noise.
    // Mutation testing found this: incrementing on entry to ANY scoring state broke nothing,
    // because no other case in this suite visits more than one.
    assertEquals(
        1,
        superstructure.getForcedShotCount(),
        "only the forced state may raise the forced-shot count");
  }

  /**
   * A turret that cannot reach the aim point must divert a scoring request too.
   *
   * <p>Added to close a surviving mutation: deleting {@code turret.isOutOfRange()} from the scoring
   * guard broke nothing, because every other case in this suite aims at a direction the turret can
   * always reach. The guard was real code that no test could reach, which is the same thing as no
   * guard at all until somebody deploys the intake at the wrong moment.
   *
   * <p>The block comes from the intake being STOWED -- in the turret-conflict band -- while the
   * shot wants the turret pointed straight backwards, through its arc. Here the intake is stowed
   * because nothing has enabled the robot yet: the robot starts stowed by rule. Until M10b step 5
   * the block came from the intake being left OUT, which on this robot cannot reach the turret; in
   * step 5 it came from M10's INDEXING stowing the intake after a pickup, which step 7 removed. The
   * case needs the real geometry to mean anything: on the 0.0 placeholders the band fails closed
   * and the turret is blocked regardless.
   */
  @Test
  void aBlockedTurretDivertsScoringToWaitingForTurret() {
    intakeGeometry = IdealPivotIO.GEOMETRY;
    buildRobot(
        usableSolver(),
        new IdealPivotIO(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());

    // +3 m in front of a goal at the origin, facing along +X, so the shot wants the turret at 180
    // degrees -- squarely inside the 150-210 degree conflict arc.
    placeRobotAt(3.0, 0.0);
    homeAndSettle();
    assertTrue(
        intake.isInTurretConflictBand(),
        "this case needs the intake in the turret's way, stowed as the robot starts");

    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    boolean sawWaiting = false;
    for (int i = 0; i < 200 && !sawWaiting; i++) {
      step(1);
      sawWaiting = superstructure.getSystemState() == Superstructure.SystemState.WAITING_FOR_TURRET;
    }

    assertTrue(
        superstructure.getLastShot().isUsable(),
        "this case is only meaningful while the solution itself is fine -- otherwise it would be"
            + " testing the usable-solution guard instead");
    assertTrue(
        sawWaiting,
        "a scoring request the turret cannot carry out must divert, not sit in a scoring state"
            + " with the turret refusing underneath it");
    assertTrue(turret.isOutOfRange(), "the turret should be the thing refusing");
    assertNotEquals(
        Indexer.SystemState.FEEDING, indexer.getSystemState(), "fed while the turret was blocked");

    // A LIMIT, recorded rather than asserted away. WAITING_FOR_TURRET is also the state that asks
    // the drivetrain to rotate, and here it correctly asks for nothing: the aim point is reachable,
    // it is the PATH that is blocked, and rotating the chassis would not retract the intake.
    //
    // The other reason to be in this state -- an aim point outside the travel entirely -- cannot be
    // reached at all with the shipped placeholder limits of plus or minus 200 degrees, because a
    // travel wider than a full circle can reach every direction on some branch. So the requested
    // heading is unreachable code on this branch, and will stop being so the moment M6 measures the
    // real hard stops, which will almost certainly be narrower. Do not delete it as dead.
    assertTrue(
        superstructure.getRequestedDriveHeading().isEmpty(),
        "an intake-blocked turret is not fixed by rotating the chassis, so no heading should be"
            + " requested");
  }

  /**
   * A warning-severity fault must not stop the robot shooting.
   *
   * <p>Added to close a surviving mutation: dropping the {@code kError} test from {@code
   * anyCritical()} broke nothing, because this suite only ever registered an error. The distinction
   * is load-bearing and deliberate -- {@code Vision} registers both QuestNav checks as WARNINGS
   * precisely because the robot shoots perfectly well without the headset, and an error badge for a
   * degraded-but-working robot is how a drive team learns to ignore red.
   */
  @Test
  void aWarningSeverityFaultDoesNotStopScoring() {
    buildRobot(
        usableSolver(),
        new IntakeIOSim(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    step(30);
    assertEquals(
        Superstructure.SystemState.SCORING_ON_THE_MOVE,
        superstructure.getSystemState(),
        "precondition: a healthy robot with a solution scores");

    injectedWarningActive = true;
    step(10);

    assertFalse(
        FaultReporter.getInstance().anyCritical(),
        "a warning is not critical; treating it as one would ground the robot for a low headset"
            + " battery");
    assertEquals(
        Superstructure.SystemState.SCORING_ON_THE_MOVE,
        superstructure.getSystemState(),
        "a warning-severity fault stopped the robot scoring");

    // ... and it is still reported, which is the other half of the contract. A severity that
    // silences a fault entirely would be worse than one that over-reports it.
    assertTrue(
        FaultReporter.getInstance().getActiveFaultNames().stream()
            .anyMatch(name -> name.contains("injected warning")),
        "the warning was not reported at all");
  }

  /**
   * A critical fault must divert a scoring request, not merely refuse to open the gate.
   *
   * <p>This is term 8 of the readiness gate doing its other job. A dead flywheel follower or a
   * disconnected turret encoder means the robot cannot aim or fire accurately, and better not to
   * shoot than to shoot into a partner's robot.
   */
  @Test
  void aCriticalFaultDivertsScoringToSpinUp() {
    buildRobot(
        usableSolver(),
        new IntakeIOSim(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    step(30);
    assertEquals(
        Superstructure.SystemState.SCORING_ON_THE_MOVE,
        superstructure.getSystemState(),
        "precondition: a healthy robot with a solution scores");

    injectedFaultActive = true;
    step(10);
    assertEquals(
        Superstructure.SystemState.SPINNING_UP,
        superstructure.getSystemState(),
        "a critical fault must divert a scoring request");

    injectedFaultActive = false;
    step(10);
    assertEquals(
        Superstructure.SystemState.SCORING_ON_THE_MOVE,
        superstructure.getSystemState(),
        "clearing the fault must let scoring resume");
  }

  /**
   * A pass must not wait on a shot solution it never asked for.
   *
   * <p>PASSING does not go through the solver: it is a lob at a fixed speed toward a fixed
   * translation, and running it through the shot tables would index them at a distance nobody
   * measured them at. Gating the feed on {@code readyToScore} would therefore make a pass wait on a
   * solution to the GOAL, which has nothing to do with where the piece is going -- and on this
   * branch, with the tables all zeros, that solution never arrives at all, so the pass could never
   * leave.
   *
   * <p>The observable difference is the gate, so that is what this asserts. The pass itself still
   * does not fire, because kPassRpm is zero awaiting a measurement and the flywheel term is
   * therefore false, exactly as for a scored shot -- see {@link
   * #theReadinessGateCannotOpenUntilTheShotTablesAreMeasured}.
   *
   * <p><b>A KNOWN EQUIVALENT MUTANT LIVES HERE, and it is not this case's fault.</b> Replacing the
   * whole pass gate with the shot gate survives mutation testing, because the two differ only when
   * every non-solution term is true while the solution term is false -- and "every non-solution
   * term is true" needs the flywheel at a non-zero goal, which needs kPassRpm. With it at zero the
   * two gates agree in every reachable state, so no test can tell them apart. Measuring kPassRpm
   * makes the distinction observable; when that happens, add a case asserting a pass feeds while a
   * shot solution is unusable, and this note can go.
   */
  @Test
  void passingDoesNotWaitOnAShotSolution() {
    // The SHIPPED solver, so every shot solution is unusable -- which is the whole point.
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();

    superstructure.setWantedState(Superstructure.WantedState.PASS);
    stepWithVision(40);

    assertEquals(Superstructure.SystemState.PASSING, superstructure.getSystemState());
    assertFalse(
        superstructure.getLastShot().isUsable(),
        "this case is only meaningful while the shot solution is unusable");

    // The override reaches the pass gate as well as the shot gate. If the pass were gated on
    // readyToScore's solution term, forcing it would be the only way to pass at all -- and the
    // forced path exists to override a judgement, not to work around a term that does not apply.
    superstructure.setIgnoreShotTolerances(true);
    step(5);
    assertEquals(
        Indexer.SystemState.FEEDING,
        indexer.getSystemState(),
        "a forced pass must feed even though no shot solution exists");
    assertEquals(
        FloorRoller.SystemState.FEEDING,
        floorRoller.getSystemState(),
        "the floor roller must feed with the indexer, or the pass runs the belt against a stopped"
            + " roller");
  }

  /**
   * A limit, recorded rather than tuned away: the readiness gate cannot open on this branch.
   *
   * <p>{@code Constants.Shot.kRpmPerMeterPerSec} and {@code kRpmIntercept} are both zero awaiting
   * M11, so every solved shot carries a flywheel speed of zero, {@code Shooter} resolves a zero
   * goal to IDLE rather than faulting, and {@code willBeAtSpeedIn} refuses a zero goal outright.
   * Term 3 is therefore false no matter how good every other input is.
   *
   * <p>This is written down as a test rather than left as a surprise because the shape is exactly
   * what {@code PROJECT_OVERVIEW.md} warns about from M9: a failure that is a limitation of the
   * inputs rather than a defect in the code. When M11 fills the tables in, this case will start
   * failing, and the right response is to delete it and assert the gate opening instead.
   */
  @Test
  void theReadinessGateCannotOpenUntilTheShotTablesAreMeasured() {
    buildRobot(
        usableSolver(),
        new IntakeIOSim(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_STATIONARY);
    stepWithVision(200);

    // Everything the test CAN make true is true: a usable solution, a stationary robot, fresh tight
    // vision, and no faults. (Term 4, a piece aboard, no longer exists -- M10b step 7.)
    assertTrue(superstructure.getLastShot().isUsable(), "term 1 should be satisfiable");
    assertTrue(
        RobotState.getInstance().getVisionStalenessSec()
            < Constants.Shot.kMaxVisionStalenessSec.get(),
        "term 5 should be satisfiable with a fix fed every loop");
    assertTrue(
        RobotState.getInstance().getPoseStdDevEstimate()
            < Constants.Shot.kMaxPoseStdDevMeters.get(),
        "term 6 should be satisfiable with a 2 cm fix");
    assertFalse(FaultReporter.getInstance().anyCritical(), "term 8 should be satisfiable");

    // And the gate is still shut, for exactly one reason.
    assertEquals(
        0.0,
        shooter.getGoalRpm(),
        "the RPM fit constants are zero, so the solver's flywheel speed is zero");
    assertFalse(
        shooter.willBeAtSpeedIn(Constants.getLoopPeriodSec() * 2.0),
        "a zero goal must never read as at speed; that guard is what makes this a limit and not a"
            + " silent vacuous pass");
    assertFalse(
        superstructure.isReadyToScore(),
        "the gate opened with a zero flywheel goal, which would fire a piece at nothing");
  }

  /**
   * With hardware that never moves, the machine still walks its states and nothing reports ready.
   *
   * <p>The deliberate-failure case the testing standard requires. State is a software concept;
   * homed, at-goal and at-speed are physical ones, and a suite that cannot tell them apart passes
   * against a robot that does not move.
   */
  @Test
  void deadMechanismsStillWalkTheStatesButNothingReportsReady() {
    buildRobot(
        usableSolver(),
        new IntakeIO() {},
        new IndexerIO() {},
        new TurretIO() {},
        new ShooterIO() {},
        new FloorRollerIO() {});
    placeRobotAt(-3.0, 0.0);

    assertEquals(Superstructure.SystemState.HOMING, superstructure.getSystemState());
    step(loopsPastHomingTimeout());

    // Nothing homed, because nothing moved, so the robot says so instead of proceeding on an
    // imaginary zero. Both mechanisms have stall detection that requires current as well as a lack
    // of velocity, precisely so a dead motor is not mistaken for a mechanism resting on its stop.
    assertFalse(intake.isHomed(), "a dead pivot must not report itself homed");
    assertFalse(turret.isHomed(), "a dead turret must not report itself homed");
    assertEquals(
        Superstructure.SystemState.FAULTED,
        superstructure.getSystemState(),
        "a robot whose mechanisms cannot home must fault");

    // The machine still transitions -- FAULTED is a state it walked to, not a crash -- and no
    // success predicate is fooled.
    assertFalse(superstructure.isReadyToScore(), "a dead robot reported itself ready to shoot");
    assertFalse(turret.atGoal(), "a dead turret reported itself on target");
    assertFalse(
        shooter.atSpeed(), "a dead flywheel reported itself at speed; both readings are zero");

    // And FAULTED still clears on request, so a dead robot is recoverable rather than bricked.
    superstructure.setWantedState(Superstructure.WantedState.HOME);
    step(3);
    assertEquals(Superstructure.SystemState.HOMING, superstructure.getSystemState());
  }

  // --------------------------------------------------------------- M10b step 7: the hopper ----

  /**
   * Releasing the intake stops the roller and leaves the intake DOWN, resting through DEFAULT; only
   * STOW raises it. And the floor roller never runs for intaking alone.
   *
   * <p>Raising the intake lets pieces fall out of the robot, so nothing but an explicit STOW may do
   * it (plan section 2.4). M10's DEFAULT stowed it, which on a hopper robot dumps the hopper every
   * time the driver lets go of a button.
   */
  @Test
  void releasingIntakeLeavesItDeployed() {
    intakeGeometry = IdealPivotIO.GEOMETRY;
    buildRobot(
        new ShootingSolver(),
        new IdealPivotIO(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    homeAndSettle();

    superstructure.setIntakeRequested(true);
    step(10);
    assertEquals(Superstructure.IntakeMode.INTAKING, superstructure.getIntakeMode());
    assertEquals(Intake.SystemState.INTAKING, intake.getSystemState());
    assertEquals(
        FloorRoller.SystemState.IDLE,
        floorRoller.getSystemState(),
        "the floor roller ran for intaking alone; it only ever feeds the indexer");

    superstructure.setIntakeRequested(false);
    for (int i = 0; i < 30; i++) {
      step(1);
      assertNotEquals(
          Intake.SystemState.STOWING, intake.getSystemState(), "releasing intake raised it");
    }
    assertEquals(Superstructure.IntakeMode.DEPLOYED, superstructure.getIntakeMode());
    assertEquals(Intake.SystemState.DEPLOYED, intake.getSystemState());

    superstructure.setWantedState(Superstructure.WantedState.DEFAULT);
    step(30);
    assertEquals(
        Intake.SystemState.DEPLOYED, intake.getSystemState(), "DEFAULT must not raise the intake");

    superstructure.setWantedState(Superstructure.WantedState.STOW);
    step(10);
    assertEquals(Superstructure.IntakeMode.STOWED, superstructure.getIntakeMode());
    assertEquals(Intake.SystemState.STOWED, intake.getSystemState(), "STOW must raise it");
  }

  /**
   * The first enable deploys the intake, even when it arrives during the boot home; and an enable
   * after an explicit STOW deploys it again.
   *
   * <p>The robot must start stowed (a rule) and pieces fall out of a stowed intake, so the first
   * enable deploys it with no button press. The enable can easily arrive while the boot home is
   * still running, and must not be lost to it.
   */
  @Test
  void firstEnableDeploysTheIntake() {
    intakeGeometry = IdealPivotIO.GEOMETRY;
    buildRobot(
        new ShootingSolver(),
        new IdealPivotIO(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    assertEquals(Superstructure.SystemState.HOMING, superstructure.getSystemState());
    superstructure.onEnable();

    homeAndSettle();
    step(10);
    assertEquals(
        Superstructure.IntakeMode.DEPLOYED,
        superstructure.getIntakeMode(),
        "an enable during the boot home was lost");
    assertTrue(intake.isRetainingPieces(), "the enabled robot is not holding its preload");

    superstructure.setWantedState(Superstructure.WantedState.STOW);
    step(10);
    assertEquals(Intake.SystemState.STOWED, intake.getSystemState());
    superstructure.onEnable();
    superstructure.setWantedState(Superstructure.WantedState.DEFAULT);
    step(10);
    assertEquals(
        Intake.SystemState.DEPLOYED,
        intake.getSystemState(),
        "an enable after a STOW must deploy the intake again");
  }

  /**
   * The floor roller LEADS and the indexer GATES: a shot request starts the floor roller at once
   * while the indexer stays stopped as the block, and the indexer feeds only once the gate opens.
   * Neither intaking alone nor a bare pre-spin runs the floor roller.
   */
  @Test
  void floorRollerLeadsAndIndexerGates() {
    buildFiringRobot();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    step(3);
    assertFalse(
        superstructure.isReadyToScore(), "precondition: no vision yet, so the gate is shut");
    assertEquals(
        FloorRoller.SystemState.FEEDING,
        floorRoller.getSystemState(),
        "the floor roller must start the moment a shot is requested, before the shooter is ready");
    assertEquals(
        Indexer.SystemState.IDLE,
        indexer.getSystemState(),
        "the indexer is the block: it must stay stopped until the gate opens");

    openTheGate();
    stepWithVision(3);
    assertEquals(
        Indexer.SystemState.FEEDING, indexer.getSystemState(), "the open gate did not feed");

    superstructure.setWantedState(Superstructure.WantedState.DEFAULT);
    superstructure.setIntakeRequested(true);
    step(10);
    assertEquals(
        FloorRoller.SystemState.IDLE,
        floorRoller.getSystemState(),
        "intaking ran the floor roller");

    superstructure.setIntakeRequested(false);
    superstructure.setWantedState(Superstructure.WantedState.SPIN_UP);
    step(10);
    assertEquals(
        FloorRoller.SystemState.IDLE,
        floorRoller.getSystemState(),
        "a bare pre-spin ran the floor roller; nothing has asked to shoot");
  }

  /**
   * The robot intakes and shoots at once: the intake keeps collecting, the turret keeps aiming, and
   * the gate still feeds.
   *
   * <p>The point of the hopper replan's orthogonal regions (plan section 2.4). M10's intaking()
   * stowed the turret, and INTAKE and SCORE were one enum, so this combination could not exist.
   */
  @Test
  void intakeAndScoreRunTogether() {
    buildFiringRobot();
    superstructure.setIntakeRequested(true);
    openTheGate();
    stepWithVision(5);

    assertEquals(Superstructure.SystemState.SCORING_ON_THE_MOVE, superstructure.getSystemState());
    assertEquals(Superstructure.IntakeMode.INTAKING, superstructure.getIntakeMode());
    assertEquals(Intake.SystemState.INTAKING, intake.getSystemState(), "stopped collecting");
    assertTrue(
        turret.getSystemState() == Turret.SystemState.AIMING
            || turret.getSystemState() == Turret.SystemState.UNWRAPPING,
        "the turret stopped aiming while the robot intakes; it was " + turret.getSystemState());
    assertEquals(Indexer.SystemState.FEEDING, indexer.getSystemState(), "stopped feeding");
    assertEquals(FloorRoller.SystemState.FEEDING, floorRoller.getSystemState());
  }

  /**
   * Feed-assist rocks only while the feed is open and the intake is not being held; holding intake
   * wins, and letting go of the shot returns the intake to rest.
   */
  @Test
  void feedAssistOnlyWhileGateOpenAndNotIntaking() {
    buildFiringRobot();
    superstructure.onEnable();
    step(10);

    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    step(1);
    assertFalse(superstructure.isReadyToScore(), "precondition: gate shut");
    assertEquals(
        Superstructure.IntakeMode.DEPLOYED,
        superstructure.getIntakeMode(),
        "feed-assist rocked with the gate shut");

    openTheGate();
    assertEquals(Superstructure.IntakeMode.FEED_ASSIST, superstructure.getIntakeMode());
    stepWithVision(3);
    assertEquals(Intake.SystemState.FEED_ASSISTING, intake.getSystemState());

    superstructure.setIntakeRequested(true);
    stepWithVision(2);
    assertEquals(
        Superstructure.IntakeMode.INTAKING,
        superstructure.getIntakeMode(),
        "intaking must win over feed-assist: the mouth has to be down to collect");

    superstructure.setIntakeRequested(false);
    stepWithVision(2);
    assertEquals(Superstructure.IntakeMode.FEED_ASSIST, superstructure.getIntakeMode());

    superstructure.setWantedState(Superstructure.WantedState.DEFAULT);
    step(2);
    assertEquals(Superstructure.IntakeMode.DEPLOYED, superstructure.getIntakeMode());
  }

  /**
   * Once a burst is feeding, a wheel recovering from each piece keeps the feed open instead of
   * stopping and restarting the indexer; a dip beyond even the burst band does close it; and losing
   * another term clears the latch.
   *
   * <p>The dips are sized so the TIGHT band rejects the recovering wheel -- proved, not assumed --
   * while the burst band accepts it. The feed is counted loop by loop: "it ended up feeding" would
   * pass a gate that stuttered every piece.
   */
  @Test
  void burstLatchKeepsFeedingThroughRecovery() {
    buildFiringRobot();
    openTheGate();
    stepWithVision(5);
    assertTrue(superstructure.isBurstLatched(), "the latch did not set when the gate opened");

    double horizon = Constants.getLoopPeriodSec() * 2.0;
    boolean tightBandRejected = false;
    for (int piece = 0; piece < 8; piece++) {
      firingShooter.injectShot(0.4 * Constants.Shooter.kBurstToleranceRpm.get());
      for (int i = 0; i < 10; i++) {
        stepWithVision(1);
        tightBandRejected |= !shooter.willBeAtSpeedIn(horizon);
        assertTrue(
            superstructure.isReadyToScore(),
            "the gate closed while the wheel recovered within the burst band, piece " + piece);
        assertEquals(
            Indexer.SystemState.FEEDING,
            indexer.getSystemState(),
            "the indexer stopped mid-burst, piece " + piece);
      }
    }
    assertTrue(
        tightBandRejected, "the dips never left the tight band, so the latch was never needed");

    firingShooter.injectShot(3.0 * Constants.Shooter.kBurstToleranceRpm.get());
    boolean closed = false;
    for (int i = 0; i < 5; i++) {
      stepWithVision(1);
      closed |= !superstructure.isReadyToScore();
    }
    assertTrue(closed, "a dip beyond the burst band did not close the gate");

    // Starve vision: the staleness term fails, and it is not the flywheel term, so the latch goes.
    step(60);
    assertFalse(superstructure.isBurstLatched(), "a non-flywheel term failed and the latch held");
  }

  /**
   * The empty belief accumulates only with the gate OPEN: hours of gate-closed "trying" say nothing
   * about the hopper, while gate-open feeding with nothing coming out reaches the belief in about
   * {@code kEmptyConfirmSec}.
   *
   * <p>The first half is the property the retrofit order names as the mutation target: counting
   * gate-closed time would end an auto's scoring leg while the hopper is still full, waiting on a
   * turret.
   */
  @Test
  void likelyEmptyNeedsTheGateOpen() {
    buildRobot(
        usableSolver(),
        new IntakeIOSim(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();
    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    stepWithVision(10 * emptyConfirmLoops());
    assertFalse(superstructure.isReadyToScore(), "precondition: this robot's gate cannot open");
    assertEquals(
        0.0,
        superstructure.getUnproductiveFeedSec(),
        "gate-closed time was counted as evidence the hopper is empty");
    assertFalse(superstructure.likelyEmpty());

    buildFiringRobot();
    openTheGate();
    int loops = 0;
    while (!superstructure.likelyEmpty() && loops < 5 * emptyConfirmLoops()) {
      stepWithVision(1);
      loops++;
    }
    assertTrue(superstructure.likelyEmpty(), "gate-open feeding with no shots never read empty");
    assertEquals(
        emptyConfirmLoops(),
        loops,
        5,
        "the belief did not take about kEmptyConfirmSec of gate-open feeding");
  }

  /**
   * The empty belief is held off by the three things that say pieces are still moving: the indexer
   * working against a load, a shot coming out, and the intake collecting.
   */
  @Test
  void likelyEmptyIsHeldOffByLoadShotsAndIntaking() {
    // A loaded feed: the clock stops once the load registers, and never reaches the belief.
    buildFiringRobot();
    indexerSim.setSimulatedLoad(true);
    openTheGate();
    stepWithVision(3 * emptyConfirmLoops());
    assertFalse(superstructure.likelyEmpty(), "a loaded feed was believed empty");
    assertTrue(
        superstructure.getUnproductiveFeedSec() < Constants.Superstructure.kEmptyConfirmSec.get());

    // A shot resets the clock.
    buildFiringRobot();
    openTheGate();
    stepWithVision(emptyConfirmLoops() / 2);
    firingShooter.injectShot();
    stepWithVision(2);
    assertTrue(
        superstructure.getUnproductiveFeedSec() < 3 * LOOP_SEC,
        "a shot did not reset the clock; it read " + superstructure.getUnproductiveFeedSec());
    stepWithVision(emptyConfirmLoops() / 2 + 2);
    assertFalse(
        superstructure.likelyEmpty(), "the belief counted feeding from before the last shot");

    // Intaking resets it too: new pieces may be arriving.
    superstructure.setIntakeRequested(true);
    stepWithVision(2);
    assertEquals(0.0, superstructure.getUnproductiveFeedSec(), "intaking did not reset the clock");
  }

  /**
   * The auto scoring command ends on the empty belief when nothing comes out, and on its timeout
   * when shots keep coming -- a hopper that never seems to empty must not hold the auto forever.
   */
  @Test
  void scoreUntilEmptyEndsOnBeliefOrTimeout() {
    buildFiringRobot();
    int timeoutLoops = (int) Math.round(Constants.Auto.kScoreTimeoutSec.get() / LOOP_SEC);

    Command byBelief = superstructure.scoreUntilEmptyCommand();
    byBelief.initialize();
    int loops = 0;
    boolean finished = false;
    while (!finished && loops < 3 * timeoutLoops) {
      stepWithVision(1);
      byBelief.execute();
      finished = byBelief.isFinished();
      loops++;
    }
    byBelief.end(false);
    assertTrue(finished, "the command never ended");
    assertTrue(superstructure.likelyEmpty(), "it ended, but not because the hopper read empty");
    assertTrue(loops < timeoutLoops, "it ran to the timeout instead of ending on the belief");
    step(2);
    assertEquals(
        Superstructure.SystemState.RESTING,
        superstructure.getSystemState(),
        "the command did not hand the robot back to DEFAULT");

    buildFiringRobot();
    Command byTimeout = superstructure.scoreUntilEmptyCommand();
    byTimeout.initialize();
    loops = 0;
    finished = false;
    while (!finished && loops < 3 * timeoutLoops) {
      stepWithVision(1);
      if (loops % 10 == 0) {
        firingShooter.injectShot();
      }
      byTimeout.execute();
      finished = byTimeout.isFinished();
      loops++;
    }
    byTimeout.end(false);
    assertTrue(finished, "a hopper that kept firing held the command past its timeout");
    assertFalse(superstructure.likelyEmpty(), "the shots should have kept the belief off");
    assertEquals(
        timeoutLoops, loops, 5, "it ended, but not at kScoreTimeoutSec after the scoring began");
  }

  /**
   * While the indexer reverses to clear a jam, the floor roller reverses with it and the empty
   * clock does not advance; a latched jam freezes the clock too.
   *
   * <p>The clock condition that matters here is "the indexer is FEEDING". An unjam pulse runs with
   * the gate still open and the indexer not loaded -- it is reversing -- so without that condition
   * every pulse would count toward "the hopper is empty".
   */
  @Test
  void jamReversesTheFloorRollerAndPausesTheEmptyClock() {
    buildFiringRobot();
    openTheGate();
    stepWithVision(5);
    indexerSim.setSimulatedJam(true);

    boolean sawUnjam = false;
    boolean sawRollerReverse = false;
    double clockBefore = superstructure.getUnproductiveFeedSec();
    for (int i = 0; i < 400 && indexer.getSystemState() != Indexer.SystemState.JAMMED; i++) {
      double clock = superstructure.getUnproductiveFeedSec();
      stepWithVision(1);
      if (indexer.getSystemState() == Indexer.SystemState.UNJAMMING) {
        sawUnjam = true;
        assertEquals(
            clock,
            superstructure.getUnproductiveFeedSec(),
            1e-9,
            "the empty clock advanced during an unjam pulse");
      }
      sawRollerReverse |= floorRoller.getSystemState() == FloorRoller.SystemState.EJECTING;
    }
    assertTrue(sawUnjam, "the jam never produced an unjam pulse");
    assertTrue(sawRollerReverse, "the floor roller never reversed with the indexer");
    assertEquals(Indexer.SystemState.JAMMED, indexer.getSystemState());

    double frozen = superstructure.getUnproductiveFeedSec();
    stepWithVision(50);
    assertEquals(
        frozen, superstructure.getUnproductiveFeedSec(), 1e-9, "the clock ran on a jammed indexer");
    assertTrue(
        superstructure.getUnproductiveFeedSec() >= clockBefore, "sanity: the clock went backwards");
  }

  // ------------------------------------------------------ M10b step 10: verify, then record ----

  /**
   * While scoring with the turret in its conflict arc, the resting intake rocks between its two
   * angles, never enters the conflict band, and never makes the turret yield.
   *
   * <p>Plan section 4.14's {@code feedAssistRocksAndNeverBlocksTheTurret}, added in step 10. The
   * intake suite proves the rock alternates on the half period, but only this case puts it beside a
   * turret that needs the arc -- the one place a rock that strayed into the band would matter. The
   * low-power fallback half of the plan's case cannot be reached here: {@code PowerBudget} is still
   * the M14 stub whose scalar is always 1.0, so the fallback is tested on the pure function in
   * {@code IntakeFsmTest} instead.
   */
  @Test
  void feedAssistRocksAndNeverBlocksTheTurret() {
    buildFiringRobotUnhomed(GOAL_BEHIND_X, IdealPivotIO.GEOMETRY);
    homeAndSettle();
    superstructure.onEnable();
    step(10);
    openTheGate();
    assertTrue(turretCommandedIntoArc(), "the shot does not need the arc, so nothing is tested");

    boolean sawDeployGoal = false;
    boolean sawAssistGoal = false;
    for (int i = 0; i < 3 * rockPeriodLoops(); i++) {
      stepWithVision(1);
      assertFalse(intake.isInTurretConflictBand(), "the rock entered the band, loop " + i);
      assertFalse(superstructure.isFeedAssistBlocked(), "a correctly placed rock was refused");
      assertTrue(turretCommandedIntoArc(), "the turret yielded its arc to the rock, loop " + i);
      if (intake.getSystemState() == Intake.SystemState.FEED_ASSISTING) {
        sawDeployGoal |= firingPivot.lastGoalRad() == IdealPivotIO.DEPLOYED;
        sawAssistGoal |= firingPivot.lastGoalRad() == IdealPivotIO.FEED_ASSIST;
      }
    }
    assertTrue(sawDeployGoal && sawAssistGoal, "the intake held still instead of rocking");
  }

  /**
   * A feed-assist angle tuned into the conflict band is refused beside a turret in its arc: the
   * intake stays out of the band, the turret keeps the arc, and the warning is raised.
   *
   * <p>Plan section 4.14's {@code misconfiguredFeedAssistYieldsToTurret}, added in step 10. On this
   * robot the rock stays well below the band, so the guard never fires on correct constants --
   * which is exactly why it needs a case: nothing else would notice it had stopped working. The
   * geometry puts the raised angle halfway into the band. {@link
   * #feedAssistRocksAndNeverBlocksTheTurret} is the negative control, on correct geometry.
   */
  @Test
  void misconfiguredFeedAssistYieldsToTurret() {
    Intake.PivotGeometry raisedIntoTheBand =
        new Intake.PivotGeometry(
            () -> IdealPivotIO.STOWED,
            () -> IdealPivotIO.DEPLOYED,
            () -> IdealPivotIO.EJECT,
            () -> IdealPivotIO.STOWED + 0.5 * IdealPivotIO.BAND,
            () -> IdealPivotIO.BAND);
    buildFiringRobotUnhomed(GOAL_BEHIND_X, raisedIntoTheBand);
    homeAndSettle();
    superstructure.onEnable();
    step(10);
    openTheGate();
    assertTrue(turretCommandedIntoArc(), "the shot does not need the arc, so nothing is tested");

    boolean sawBlocked = false;
    for (int i = 0; i < 3 * rockPeriodLoops(); i++) {
      stepWithVision(1);
      assertFalse(
          intake.isInTurretConflictBand(),
          "the mis-tuned rock swung into the band beside a turret in its arc, loop " + i);
      assertTrue(turretCommandedIntoArc(), "the turret yielded; while scoring it wins");
      sawBlocked |= superstructure.isFeedAssistBlocked();
    }
    assertTrue(sawBlocked, "the refused feed-assist raised no warning");
  }

  /**
   * Plan section 10.2's M10b simulation checkpoint, walked as one continuous match rather than as
   * separate cases, so the clauses are shown to hold together and in order.
   *
   * <p>From boot: the intake starts STOWED, and a shot that needs the arc is refused the turret
   * while it is. The floor roller leads from the first loop of the request. The first enable
   * deploys the intake, and the turret gets its arc only once the pivot has left the band. Then
   * intake, score and drive together: the intake region reads INTAKING while the turret aims and
   * the indexer feeds. Releasing the intake leaves it down and rocking, still clear of the band.
   *
   * <p>The plan asks for this in simulation, from the log. AdvantageScope cannot be driven from the
   * agent's sandbox, so the checkpoint is discharged here over the same simulated IO the sim uses,
   * and it can fail on every build instead of once. The one clause with no assertion is "M10's
   * eight separately-logged booleans becomes seven": Logger records nothing under test, so that one
   * is checked by reading updateReadyToScore(), which logs the seven terms, plus the
   * stationary-only term and the override, each under its own key.
   */
  @Test
  void theHopperCheckpointHoldsAcrossOneMatch() {
    buildFiringRobotUnhomed(GOAL_BEHIND_X, IdealPivotIO.GEOMETRY);
    assertEquals(Superstructure.SystemState.HOMING, superstructure.getSystemState());
    homeAndSettle();
    assertEquals(Superstructure.IntakeMode.STOWED, superstructure.getIntakeMode());
    assertTrue(intake.isInTurretConflictBand(), "the robot must start stowed, in the band");

    // ---- A shot that needs the arc, before the enable. The turret is refused; the floor roller
    // leads anyway, with the indexer shut.
    int loop = 0;
    int firstRollerFeed = -1;
    int firstIndexerFeed = -1;
    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    for (int i = 0; i < 50; i++, loop++) {
      stepWithVision(1);
      assertFalse(
          turretCommandedIntoArc(), "the turret was sent into its arc beside a stowed intake");
      if (firstRollerFeed < 0 && floorRoller.getSystemState() == FloorRoller.SystemState.FEEDING) {
        firstRollerFeed = loop;
      }
      if (firstIndexerFeed < 0 && indexer.getSystemState() == Indexer.SystemState.FEEDING) {
        firstIndexerFeed = loop;
      }
    }
    assertTrue(firstRollerFeed >= 0 && firstRollerFeed <= 2, "the floor roller did not lead");
    assertEquals(-1, firstIndexerFeed, "the indexer fed with the turret refused its aim");

    // ---- The first enable deploys the intake; the turret gets its arc only once it is out.
    //
    // The window is long because the simulated turret, on its placeholder gains, rings about a
    // 180-degree goal for roughly eight seconds before it settles inside tolerance and the gate can
    // open -- the same unmeasured-gains ringing the intake pivot shows, and M6's to fix, not this
    // milestone's. It is the time to the gate that is long, not the time to the arc.
    superstructure.onEnable();
    int leftBand = -1;
    int enteredArc = -1;
    for (int i = 0;
        i < TURRET_SETTLE_LOOPS && (enteredArc < 0 || firstIndexerFeed < 0);
        i++, loop++) {
      stepWithVision(1);
      if (leftBand < 0 && !intake.isInTurretConflictBand()) {
        leftBand = loop;
      }
      if (enteredArc < 0 && turretCommandedIntoArc()) {
        enteredArc = loop;
      }
      if (firstIndexerFeed < 0 && indexer.getSystemState() == Indexer.SystemState.FEEDING) {
        firstIndexerFeed = loop;
      }
    }
    assertTrue(leftBand >= 0, "the enable never deployed the intake out of the band");
    assertTrue(enteredArc >= leftBand, "the turret entered its arc before the pivot left the band");
    assertTrue(
        firstIndexerFeed >= 0,
        "the gate never opened after the enable; superstructure "
            + superstructure.getSystemState()
            + ", turret "
            + turret.getSystemState()
            + " at "
            + turret.getPositionRad()
            + " commanded "
            + turret.getCommandedPositionRad()
            + ", turret at goal "
            + turret.atGoal()
            + ", flywheel "
            + shooter.getVelocityRpm()
            + " of "
            + shooter.getGoalRpm());
    assertTrue(firstIndexerFeed > firstRollerFeed, "the indexer fed before the floor roller ran");
    assertTrue(intake.isRetainingPieces(), "the enabled robot is not holding its preload");

    // ---- Intake, score and drive together. The chassis speed goes to RobotState the way the
    // drivetrain reports it, sideways at 1 m/s, so the solver has a moving shot to compensate.
    superstructure.setIntakeRequested(true);
    ChassisSpeeds driving = new ChassisSpeeds(0.0, 1.0, 0.0);
    try {
      boolean sawAllAtOnce = false;
      // The moving shot swings the aim point by roughly 15 degrees, so the turret rings again; the
      // same long window applies.
      for (int i = 0; i < TURRET_SETTLE_LOOPS && !sawAllAtOnce; i++) {
        stepDriving(driving);
        sawAllAtOnce =
            superstructure.getIntakeMode() == Superstructure.IntakeMode.INTAKING
                && intake.getSystemState() == Intake.SystemState.INTAKING
                && turret.getSystemState() == Turret.SystemState.AIMING
                && indexer.getSystemState() == Indexer.SystemState.FEEDING
                && RobotState.getInstance().getFieldRelativeSpeeds().vyMetersPerSecond > 0.5;
      }
      assertTrue(
          sawAllAtOnce,
          "never intaking, aiming and feeding at once while driving; intake mode "
              + superstructure.getIntakeMode()
              + ", turret "
              + turret.getSystemState()
              + ", indexer "
              + indexer.getSystemState());

      // ---- Release the intake: it stays down, and with the feed open it rocks, clear of the band.
      // The gate first opened while the turret was still ringing near the edge of its tolerance,
      // and it rocks only while the gate is open, so the rock is watched for as long as the turret
      // may take to settle; the invariants are asserted on every one of those loops.
      superstructure.setIntakeRequested(false);
      boolean sawDeployGoal = false;
      boolean sawAssistGoal = false;
      for (int i = 0; i < TURRET_SETTLE_LOOPS && !(sawDeployGoal && sawAssistGoal); i++) {
        stepDriving(driving);
        assertNotEquals(Intake.SystemState.STOWING, intake.getSystemState(), "releasing raised it");
        assertFalse(intake.isInTurretConflictBand(), "the released intake entered the band");
        assertTrue(intake.isRetainingPieces(), "the released intake stopped holding the hopper");
        if (intake.getSystemState() == Intake.SystemState.FEED_ASSISTING) {
          sawDeployGoal |= firingPivot.lastGoalRad() == IdealPivotIO.DEPLOYED;
          sawAssistGoal |= firingPivot.lastGoalRad() == IdealPivotIO.FEED_ASSIST;
        }
      }
      assertTrue(sawDeployGoal && sawAssistGoal, "the released intake did not rock while scoring");
    } finally {
      // RobotState is a process-wide singleton, and resetting the pose does not reset the speeds.
      // Stop the chassis, and hold it stopped long enough for the acceleration filter to drain,
      // so no later case inherits a moving robot.
      for (int i = 0; i < 50; i++) {
        stepDriving(new ChassisSpeeds());
      }
    }
  }

  /**
   * The burst latch belongs to one burst: it is clear until the gate first opens, so the first
   * piece meets the tight band; it sets when the gate opens; and it clears when the robot stops
   * scoring, so the next burst starts tight again -- including when it stops scoring into a
   * pre-spin, which keeps every other term true.
   *
   * <p>Added by step 10's mutation pass. {@link #burstLatchKeepsFeedingThroughRecovery} proves the
   * latch holds the feed open once set and clears on a lost vision term, but no case asserted
   * either end of its life: a latch that set as soon as every other term held, or that survived a
   * return to rest, both passed every test while letting a burst's first piece leave a wheel that
   * was only inside the wide band.
   *
   * <p>Each burst ends in SPIN_UP, not DEFAULT, and that is the point. The first version returned
   * to rest and the mutant survived it: RESTING stows the turret, the turret term fails, and that
   * clears the latch whether or not the latch's own state check works. SPINNING_UP keeps the turret
   * aimed and the wheel up, so the state check is the only thing left to clear it -- and a pre-spin
   * between two scoring legs is exactly what an auto's spinUpWhile does.
   */
  @Test
  void theBurstLatchBelongsToOneBurst() {
    buildFiringRobot();
    double horizon = Constants.getLoopPeriodSec() * 2.0;

    for (int burst = 0; burst < 2; burst++) {
      superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
      for (int i = 0; i < TURRET_SETTLE_LOOPS && !superstructure.isReadyToScore(); i++) {
        assertFalse(
            superstructure.isBurstLatched(),
            "burst " + burst + ": the latch set before the gate had opened, loop " + i);
        stepWithVision(1);
      }
      assertTrue(superstructure.isReadyToScore(), "burst " + burst + ": the gate never opened");
      assertTrue(
          shooter.willBeAtSpeedIn(horizon),
          "burst " + burst + ": the gate opened on a wheel outside the tight band");
      stepWithVision(3);
      assertTrue(
          superstructure.isBurstLatched(), "burst " + burst + ": the open gate did not latch");

      superstructure.setWantedState(Superstructure.WantedState.SPIN_UP);
      stepWithVision(5);
      assertEquals(Superstructure.SystemState.SPINNING_UP, superstructure.getSystemState());
      assertFalse(
          superstructure.isBurstLatched(),
          "burst " + burst + ": the latch survived into a pre-spin between bursts");
    }

    superstructure.setWantedState(Superstructure.WantedState.DEFAULT);
    stepWithVision(5);
    assertFalse(superstructure.isBurstLatched(), "the latch survived the robot going back to rest");
  }

  /**
   * Every request that shoots runs the floor roller, including the autos' and a request the
   * superstructure has diverted because the shot is not ready.
   *
   * <p>Added by step 10's mutation pass. Plan section 2.4: "shot requested" means the WANTED state,
   * so the floor roller also runs while a score request waits in WAITING_FOR_TURRET or SPINNING_UP,
   * with the pile already packed against the stopped indexer. {@link
   * #floorRollerLeadsAndIndexerGates} requests only SCORE_ON_MOVE, and on a robot where it is not
   * diverted, so a roller keyed on the system state, or one that forgot AUTO_SCORE_ON_MOVE and so
   * never led in autonomous, passed it. This robot runs the shipped all-zero shot tables, so every
   * aimed request is diverted.
   */
  @Test
  void everyShotRequestRunsTheFloorRollerEvenWhenDiverted() {
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();

    Superstructure.WantedState[] shooting = {
      Superstructure.WantedState.SCORE_ON_MOVE,
      Superstructure.WantedState.SCORE_STATIONARY,
      Superstructure.WantedState.SCORE_FORCED,
      Superstructure.WantedState.AUTO_SCORE_ON_MOVE,
      Superstructure.WantedState.PASS,
    };
    boolean sawDiverted = false;
    for (Superstructure.WantedState wanted : shooting) {
      superstructure.setWantedState(wanted);
      stepWithVision(5);
      assertEquals(
          FloorRoller.SystemState.FEEDING,
          floorRoller.getSystemState(),
          wanted
              + " did not run the floor roller (superstructure "
              + superstructure.getSystemState()
              + ")");
      sawDiverted |=
          superstructure.getSystemState() == Superstructure.SystemState.WAITING_FOR_TURRET;

      superstructure.setWantedState(Superstructure.WantedState.DEFAULT);
      step(5);
      assertEquals(
          FloorRoller.SystemState.IDLE, floorRoller.getSystemState(), "rest ran the roller");
    }
    assertTrue(sawDiverted, "no request was diverted, so the WANTED-state rule was not tested");
  }

  // ------------------------------------------------------- M11 step 2: the shot-table override
  // ----

  /** A measurement speed no solver fit in this suite would produce by accident. */
  private static final double TABLE_RPM = 3210.0;

  /**
   * With the shipped, unmeasured tables, SCORE_STATIONARY cannot shoot; with the shot-table
   * override it aims at the goal, spins to the given speed, opens the gate and feeds a burst.
   *
   * <p>The negative half is the reason M11 needed the override at all: until the tables are
   * measured the solver refuses every shot, so without it nothing could collect them. The positive
   * half runs the ordinary stationary chain end to end -- the gate, then the burst latch -- rather
   * than asserting on the solution alone, because a measurement the gate never lets fire measures
   * nothing.
   */
  @Test
  void theShotTableOverrideLetsAnUnmeasuredRobotFireABurst() {
    intakeGeometry = IdealPivotIO.GEOMETRY;
    buildRobot(
        new ShootingSolver(),
        new IdealPivotIO(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_STATIONARY);
    for (int i = 0; i < 100; i++) {
      stepWithVision(1);
      assertNotEquals(
          Indexer.SystemState.FEEDING,
          indexer.getSystemState(),
          "fed with no table and no override");
    }
    assertFalse(superstructure.getLastShot().isUsable(), "precondition: the tables are unmeasured");

    superstructure.setShotTableRpm(TABLE_RPM);
    for (int i = 0; i < TURRET_SETTLE_LOOPS && !superstructure.isReadyToScore(); i++) {
      stepWithVision(1);
    }
    assertTrue(superstructure.isReadyToScore(), "the override never let the gate open");
    assertEquals(TABLE_RPM, superstructure.getLastShot().flywheelRpm(), 1e-9);
    assertEquals(TABLE_RPM, shooter.getGoalRpm(), 1e-9, "the flywheel was not sent to the speed");
    assertEquals(
        0.0,
        superstructure.getLastShot().turretFieldAngle().getRadians(),
        1e-9,
        "the goal is straight ahead, so the aim must be too");

    // The gate opens on a PREDICTION, mid spin-up. Until M12 step 1 it then dropped for a loop
    // while the wheel was still accelerating hard, because Shooter.willBeAtSpeedIn's projection
    // overshot even the burst band, and the indexer stopped mid-burst; this case had to wait up to
    // ten loops for the feed. With the projection saturated at the goal, the gate holds from the
    // loop it opens, and the feed starts on the next loop and never stops.
    int fedAt = -1;
    for (int i = 0; i < 40; i++) {
      stepWithVision(1);
      assertTrue(superstructure.isReadyToScore(), "the gate dropped during the spin-up, loop " + i);
      if (indexer.getSystemState() == Indexer.SystemState.FEEDING) {
        fedAt = fedAt < 0 ? i : fedAt;
      } else {
        assertTrue(fedAt < 0, "the indexer stopped mid-burst, loop " + i);
      }
    }
    assertTrue(fedAt >= 0 && fedAt <= 2, "the open gate did not feed promptly: " + fedAt);
    assertTrue(superstructure.isBurstLatched(), "a measurement burst must latch like any other");
  }

  /**
   * The override belongs to one stationary measurement: another request clears it, a disable clears
   * it, and while it is set it touches no shot but the stationary one.
   *
   * <p>A leaked override is the failure that matters. It would fire every later stationary shot at
   * the last test speed, with nothing in a match to show why the robot was missing. Each clear is
   * checked by going BACK to SCORE_STATIONARY afterwards and confirming the shot uses the tables
   * again, since "the flag is false" alone would pass a clear that the solve step ignored.
   */
  @Test
  void theShotTableOverrideCannotLeakIntoAnotherShot() {
    buildFiringRobot();
    superstructure.setWantedState(Superstructure.WantedState.SCORE_STATIONARY);
    stepWithVision(3);
    double tableRpm = superstructure.getLastShot().flywheelRpm();
    assertTrue(superstructure.getLastShot().isUsable(), "precondition: the tables can shoot");
    assertNotEquals(TABLE_RPM, tableRpm, 1.0, "precondition: the two speeds must differ");

    // While set, it applies to the stationary shot...
    superstructure.setShotTableRpm(TABLE_RPM);
    stepWithVision(3);
    assertEquals(TABLE_RPM, superstructure.getLastShot().flywheelRpm(), 1e-9);

    // ...and to nothing else. A shot on the move is solved from the tables, and asking for it ends
    // the measurement.
    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    stepWithVision(3);
    assertNotEquals(TABLE_RPM, superstructure.getLastShot().flywheelRpm(), 1.0);
    assertFalse(superstructure.isShotTableOverrideActive(), "another request did not clear it");
    superstructure.setWantedState(Superstructure.WantedState.SCORE_STATIONARY);
    stepWithVision(3);
    assertEquals(
        tableRpm,
        superstructure.getLastShot().flywheelRpm(),
        1e-6,
        "after another request, the stationary shot still fired at the test speed");

    // A disable clears it too.
    superstructure.setShotTableRpm(TABLE_RPM);
    stepWithVision(3);
    assertEquals(TABLE_RPM, superstructure.getLastShot().flywheelRpm(), 1e-9);
    superstructure.onDisable();
    assertFalse(superstructure.isShotTableOverrideActive(), "a disable did not clear it");
    superstructure.setWantedState(Superstructure.WantedState.SCORE_STATIONARY);
    stepWithVision(3);
    assertEquals(
        tableRpm,
        superstructure.getLastShot().flywheelRpm(),
        1e-6,
        "after a disable, the stationary shot still fired at the test speed");

    // And setting it while another shot is already requested -- no request change to clear it --
    // still leaves that shot alone. It applies only while SCORE_STATIONARY is the request.
    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    stepWithVision(3);
    superstructure.setShotTableRpm(TABLE_RPM);
    stepWithVision(3);
    assertNotEquals(
        TABLE_RPM,
        superstructure.getLastShot().flywheelRpm(),
        1.0,
        "an override set during a shot on the move took over the moving shot");
    superstructure.setWantedState(Superstructure.WantedState.DEFAULT);
  }

  // ------------------------------------------------ M11 step 3: the shot-table measurement burst
  // ----

  /**
   * Builds the shipped, unmeasured robot -- all-zero tables -- facing the goal from 3 m, and homes
   * it. The robot M11 is actually measured on.
   *
   * <p><b>Outputs:</b> replaces every subsystem; sets {@link #firingShooter}.
   */
  private void buildUnmeasuredRobot() {
    intakeGeometry = IdealPivotIO.GEOMETRY;
    firingShooter = new ShooterIOSim();
    buildRobot(
        new ShootingSolver(),
        new IdealPivotIO(),
        new IndexerIOSim(),
        new TurretIOSim(),
        firingShooter,
        new FloorRollerIOSim());
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();
  }

  /**
   * Runs a started command by hand, with fresh vision, until it finishes or a limit is reached;
   * while the indexer feeds, a piece leaves every ten loops until {@code pieces} have gone.
   *
   * @param command a command that has already been initialised
   * @param pieces how many pieces the hopper holds
   * @param maxLoops how many loops to allow
   * @return the loops it took to finish, or {@code maxLoops + 1} if it never did
   */
  private int runBurst(Command command, int pieces, int maxLoops) {
    int fired = 0;
    int feedingLoops = 0;
    for (int loop = 1; loop <= maxLoops; loop++) {
      stepWithVision(1);
      if (indexer.getSystemState() == Indexer.SystemState.FEEDING && fired < pieces) {
        if (++feedingLoops % 10 == 0) {
          firingShooter.injectShot();
          fired++;
        }
      }
      command.execute();
      if (command.isFinished()) {
        return loop;
      }
    }
    return maxLoops + 1;
  }

  /**
   * A measurement burst on the unmeasured robot fires the whole hopper, ends when it reads empty,
   * reports what it fired, and leaves the robot at rest with the override cleared.
   *
   * <p>This is the M11 practice-field loop, end to end in simulation: set the speed and the taped
   * distance, trigger, and read one row of data back. It is also the case that proves the override
   * really is the thing letting an unmeasured robot fire: the report's speed is the one asked for.
   */
  @Test
  void aShotTableBurstFiresTheHopperAndReportsIt() {
    buildUnmeasuredRobot();
    List<ShotTableSample> samples = new ArrayList<>();
    double holdSec = 20.0;
    Command burst =
        CharacterizationCommands.measureShotTablePoint(
            superstructure, shooter, () -> TABLE_RPM, () -> 3.05, holdSec, samples::add);

    burst.initialize();
    int loops = runBurst(burst, 8, (int) (holdSec / LOOP_SEC) + 50);
    burst.end(false);

    assertTrue(loops < holdSec / LOOP_SEC, "the burst ran to its time limit instead of emptying");
    assertEquals(1, samples.size(), "one burst must report exactly once");
    ShotTableSample sample = samples.get(0);
    assertEquals(8, sample.shotsFired(), "the burst did not count the eight pieces it fired");
    assertEquals(TABLE_RPM, sample.rpm(), 1e-9);
    assertEquals(3.05, sample.tapedDistanceMeters(), 1e-9);
    assertEquals(3.0, sample.estimatedDistanceMeters(), 1e-6, "the estimated distance is wrong");
    assertFalse(sample.timedOut());

    assertFalse(superstructure.isShotTableOverrideActive(), "the burst left its override behind");
    step(3);
    assertEquals(
        Superstructure.SystemState.RESTING,
        superstructure.getSystemState(),
        "the burst did not hand the robot back to rest");
  }

  /**
   * Bursts after the first count only their own pieces, wait for their own pieces even though the
   * hopper is already believed empty, and an empty hopper ends on the time limit and says so.
   *
   * <p>Burst A fires two pieces and ends on the belief, leaving it set. Burst B is a hopper
   * reloaded by hand, which resets nothing, with three pieces: ending on the belief alone would end
   * it on its first loop, having measured nothing, and on the practice field that is every burst
   * after the first. It must also report three shots, not the five fired since boot. Burst C has
   * nothing in it, so no shot of its own ever arrives and only the time limit can end it.
   */
  @Test
  void laterShotTableBurstsMeasureOnlyThemselves() {
    buildUnmeasuredRobot();
    List<ShotTableSample> samples = new ArrayList<>();
    double holdSec = 20.0;
    int limitLoops = (int) (holdSec / LOOP_SEC) + 50;

    Command a =
        CharacterizationCommands.measureShotTablePoint(
            superstructure, shooter, () -> TABLE_RPM, () -> 3.0, holdSec, samples::add);
    a.initialize();
    runBurst(a, 2, limitLoops);
    a.end(false);
    assertEquals(2, samples.get(0).shotsFired());
    assertFalse(samples.get(0).timedOut(), "burst A should have ended on its belief");
    assertTrue(superstructure.likelyEmpty(), "precondition: burst A leaves the belief set");

    Command b =
        CharacterizationCommands.measureShotTablePoint(
            superstructure, shooter, () -> TABLE_RPM, () -> 3.0, holdSec, samples::add);
    b.initialize();
    runBurst(b, 3, limitLoops);
    b.end(false);
    assertEquals(3, samples.get(1).shotsFired(), "burst B ended early, or counted A's pieces too");
    assertFalse(samples.get(1).timedOut(), "burst B did not end on its own belief");

    double shortHoldSec = 4.0;
    Command c =
        CharacterizationCommands.measureShotTablePoint(
            superstructure, shooter, () -> TABLE_RPM, () -> 3.0, shortHoldSec, samples::add);
    c.initialize();
    int loops = runBurst(c, 0, TURRET_SETTLE_LOOPS);
    c.end(false);
    assertEquals(
        (int) Math.round(shortHoldSec / LOOP_SEC),
        loops,
        2,
        "an empty hopper did not end on the time limit");
    assertTrue(samples.get(2).timedOut(), "an empty burst must report that it timed out");
    assertEquals(0, samples.get(2).shotsFired());
  }

  /**
   * A cancelled burst still clears its override, rests the robot and reports.
   *
   * <p>Cancelling is how a practice-field operator stops a burst that is going wrong, and it is the
   * path most likely to be forgotten: a cancelled command skips its own finish condition.
   */
  @Test
  void aCancelledShotTableBurstStillCleansUp() {
    buildUnmeasuredRobot();
    List<ShotTableSample> samples = new ArrayList<>();
    Command burst =
        CharacterizationCommands.measureShotTablePoint(
            superstructure, shooter, () -> TABLE_RPM, () -> 3.0, 20.0, samples::add);
    burst.initialize();
    stepWithVision(10);
    burst.execute();
    assertTrue(superstructure.isShotTableOverrideActive(), "precondition: the burst is running");

    burst.end(true);
    assertFalse(superstructure.isShotTableOverrideActive(), "a cancel left the override set");
    assertEquals(1, samples.size(), "a cancelled burst must still report");
    step(3);
    assertEquals(Superstructure.SystemState.RESTING, superstructure.getSystemState());
  }

  // -------------------------------------------------- M12 step 3: the checkpoint, in simulation
  // ----

  /** Radius of the orbit the M12 checkpoint strafes around the goal on, meters. */
  private static final double ORBIT_RADIUS_METERS = 3.0;

  /** The plan's checkpoint speed: strafing at 2 m/s or more. */
  private static final double ORBIT_SPEED_METERS_PER_SEC = 2.0;

  /** Where on the orbit the robot is, as the angle of the robot seen from the goal. */
  private double orbitAngleRad;

  /**
   * The robot's field velocity on the orbit at the current angle.
   *
   * @return tangential velocity, counter-clockwise around the goal, in meters per second
   */
  private Translation2d orbitVelocity() {
    return new Translation2d(
        ORBIT_SPEED_METERS_PER_SEC, new Rotation2d(orbitAngleRad + Math.PI / 2));
  }

  /**
   * Advances one loop along a circle around the goal while facing it: a strafe that stays
   * perpendicular to the line of sight, at a constant range, for as long as the case likes.
   *
   * <p>The pose is placed rather than integrated from odometry, and the chassis speed is fed to
   * RobotState the way the drivetrain reports it, robot-relative, with the rotation that keeps the
   * robot facing the goal. Both are exactly consistent with the motion, which is all the solver,
   * the gate and the turret read.
   *
   * <p><b>Outputs:</b> moves the robot, feeds RobotState a speed sample and a vision fix, and
   * steps.
   */
  private void stepOrbiting() {
    double omega = ORBIT_SPEED_METERS_PER_SEC / ORBIT_RADIUS_METERS;
    orbitAngleRad += omega * LOOP_SEC;
    Translation2d goal = Constants.Field.goalTranslation();
    Rotation2d heading = new Rotation2d(orbitAngleRad + Math.PI);
    Translation2d position =
        goal.plus(new Translation2d(ORBIT_RADIUS_METERS, new Rotation2d(orbitAngleRad)));
    RobotState.getInstance()
        .resetPose(
            new Pose2d(position, heading),
            Rotation2d.kZero,
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            });
    Translation2d velocity = orbitVelocity();
    RobotState.getInstance()
        .addDriveSpeedsObservation(
            Timer.getFPGATimestamp(),
            ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.getX(), velocity.getY(), omega, heading));
    feedFreshVisionFix();
    step(1);
  }

  /**
   * Where a piece released now would land, judged from what the robot is actually DOING.
   *
   * <p>The release point is the robot's position, the muzzle direction is the turret's MEASURED
   * angle plus the heading, and the muzzle speed is the flywheel's MEASURED speed back through the
   * fit -- not the solver's commands. {@link ShotFlightModel} then follows the piece. The solver is
   * proved right on its own in {@code ShootingSolverTest}, so any miss here is a mechanism that did
   * not do what it was told: the turret behind its aim, or the wheel off its speed.
   *
   * @param robotVelocity the robot's field velocity at release, in meters per second
   * @return the landing point, in field coordinates
   */
  private Translation2d landingPoint(Translation2d robotVelocity) {
    Pose2d pose = RobotState.getInstance().getEstimatedPose();
    return ShotFlightModel.landingPoint(
        firingShotSolver,
        pose.getTranslation(),
        robotVelocity,
        new Rotation2d(turret.getPositionRad()).plus(pose.getRotation()),
        shooter.getVelocityRpm() / FIRING_RPM_PER_MPS);
  }

  /**
   * The worst miss a shot inside the gate's own tolerances can have: the commanded shot with the
   * turret off by its full {@code kToleranceRad} and the wheel off by its full tight band, in every
   * combination of sign.
   *
   * <p>This is the pass mark for the checkpoint, rather than a number chosen to make it pass. A
   * piece that lands inside it did what the gate promised; one that lands outside it was released
   * by a gate that let through more error than its tolerances allow.
   *
   * @param robotVelocity the robot's field velocity at release, in meters per second
   * @return the largest landing miss inside the tolerances, in meters
   */
  private double gateErrorBudget(Translation2d robotVelocity) {
    Pose2d pose = RobotState.getInstance().getEstimatedPose();
    var shot = superstructure.getLastShot();
    double aimTolerance = Constants.Turret.kToleranceRad.get();
    double speedTolerance = Constants.Shooter.kToleranceRpm.get() / FIRING_RPM_PER_MPS;
    double worst = 0.0;
    for (double aimSign : new double[] {-1.0, 1.0}) {
      for (double speedSign : new double[] {-1.0, 1.0}) {
        Translation2d landing =
            ShotFlightModel.landingPoint(
                firingShotSolver,
                pose.getTranslation(),
                robotVelocity,
                shot.turretFieldAngle().plus(new Rotation2d(aimSign * aimTolerance)),
                shot.flywheelRpm() / FIRING_RPM_PER_MPS + speedSign * speedTolerance);
        worst = Math.max(worst, landing.getDistance(Constants.Field.goalTranslation()));
      }
    }
    return worst;
  }

  /**
   * Plan section 10.2's M12 checkpoint, in simulation: strafing perpendicular to the goal at 2 m/s
   * with SCORE_ON_MOVE, every piece the gate lets go lands on the goal.
   *
   * <p>The robot orbits the goal at 3 m while facing it (see {@link #stepOrbiting()}), settles in a
   * pre-spin, then scores on the move for ten seconds. A piece is released whenever the indexer is
   * feeding and the wheel meets the tight band, at most one every ten loops so the wheel recovers
   * between them. The checkpoint's two log clauses are asserted too: the virtual goal sits UPSTREAM
   * of the motion by the robot's velocity times the flight time, and the solver stays under its
   * iteration cap.
   */
  @Test
  void theMovingShotLandsWhileStrafingAtTwoMetersPerSecond() {
    buildFiringRobotUnhomed(-ORBIT_RADIUS_METERS, IdealPivotIO.GEOMETRY);
    homeAndSettle();
    orbitAngleRad = Math.PI;

    List<Double> misses = new ArrayList<>();
    try {
      superstructure.setWantedState(Superstructure.WantedState.SPIN_UP);
      for (int i = 0; i < TURRET_SETTLE_LOOPS; i++) {
        stepOrbiting();
      }

      superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
      Translation2d goal = Constants.Field.goalTranslation();
      double horizon = Constants.getLoopPeriodSec() * 2.0;
      int sinceLastShot = 1000;
      for (int i = 0; i < 500; i++) {
        stepOrbiting();
        sinceLastShot++;
        var shot = superstructure.getLastShot();
        if (shot.isUsable()) {
          Translation2d lead = shot.virtualGoal().minus(goal);
          Translation2d expected = orbitVelocity().times(-shot.timeOfFlightSec());
          assertEquals(0.0, lead.minus(expected).getNorm(), 0.02, "the virtual goal is misplaced");
          assertTrue(
              firingShotSolver.getLastIterationCount() < Constants.Shot.kMaxSolverIterations,
              "the solver hit its iteration cap");
        }
        if (indexer.getSystemState() == Indexer.SystemState.FEEDING
            && shooter.willBeAtSpeedIn(horizon)
            && sinceLastShot >= 10) {
          double miss = landingPoint(orbitVelocity()).getDistance(goal);
          double budget = gateErrorBudget(orbitVelocity());
          assertTrue(
              miss <= budget + 0.01,
              "piece "
                  + misses.size()
                  + " missed by "
                  + miss
                  + " m, outside the gate's budget of "
                  + budget
                  + " m");
          misses.add(miss);
          firingShooter.injectShot();
          sinceLastShot = 0;
        }
      }
    } finally {
      for (int i = 0; i < 50; i++) {
        RobotState.getInstance()
            .addDriveSpeedsObservation(Timer.getFPGATimestamp(), new ChassisSpeeds());
        step(1);
      }
    }
    // The plan's bar is 7 of 10 on the robot. Two checks here, and they catch different failures.
    // The per-piece budget above is built around the COMMANDED shot, so it judges the mechanisms --
    // did the turret and the wheel do what they were told -- and moves with the solver. This one
    // judges the whole chain against the real goal: the mean miss must be well inside what the
    // robot would have missed by with no compensation at all, its drift over the flight. With the
    // velocity counted twice (the bug M12 step 3 fixed), the budget still passes and this fails.
    assertTrue(misses.size() >= 20, "only " + misses.size() + " pieces in ten seconds of strafing");
    double mean = misses.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);
    double drift =
        ORBIT_SPEED_METERS_PER_SEC * firingShotSolver.getTimeOfFlight(ORBIT_RADIUS_METERS);
    assertTrue(
        mean < 0.2 * drift,
        "the mean miss, "
            + mean
            + " m, is not an order of magnitude inside the uncompensated "
            + "drift of "
            + drift
            + " m");
  }

  /**
   * Steps one loop with a fresh vision fix while RobotState measures the given chassis speed.
   *
   * @param robotRelative the chassis speed to report, robot-relative
   *     <p><b>Outputs:</b> feeds RobotState a speed sample and a vision fix, then steps the robot.
   */
  private void stepDriving(ChassisSpeeds robotRelative) {
    RobotState.getInstance().addDriveSpeedsObservation(Timer.getFPGATimestamp(), robotRelative);
    stepWithVision(1);
  }
}
