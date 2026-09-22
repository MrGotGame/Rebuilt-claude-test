package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.shooting.ShootingSolver;
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

    intake = new Intake(intakeIo);
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
            intake::isIntaking);

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
   * @return a solver over non-zero time-of-flight and launch-speed tables
   */
  private static ShootingSolver usableSolver() {
    return new ShootingSolver(
        new double[] {1.0, 4.0, 7.0},
        new double[] {0.50, 0.80, 1.10},
        new double[] {8.0, 12.0, 16.0});
  }

  /** Loops that comfortably outlast the robot-level homing timeout. */
  private static int loopsPastHomingTimeout() {
    return (int) (Constants.Superstructure.kHomingTimeoutSec.get() / LOOP_SEC) + 30;
  }

  /** Loops the indexer needs to settle and then debounce a newly-arrived piece. */
  private static int detectionLoops() {
    return (int)
            ((Constants.Indexer.kDetectionSettleSec.get()
                    + Constants.Indexer.kHasPieceDebounceSec.get())
                / LOOP_SEC)
        + 15;
  }

  /** Homes the robot and waits for it to settle into STOWED. */
  private void homeAndSettle() {
    step(loopsPastHomingTimeout());
    assertEquals(
        Superstructure.SystemState.STOWED,
        superstructure.getSystemState(),
        "the robot never finished homing; intake homed="
            + intake.isHomed()
            + " turret homed="
            + turret.isHomed());
  }

  /**
   * Runs an intake cycle until the indexer reports a piece aboard.
   *
   * <p>Piece detection runs off the load on a <b>turning</b> wheel, so a piece cannot be detected
   * while the indexer is stopped. Driving the real intake state is what makes the detection path
   * the same one the robot uses, rather than poking a flag.
   */
  private void acquirePiece() {
    indexerSim.setSimulatedPiecePresent(true);
    superstructure.setWantedState(Superstructure.WantedState.INTAKE);
    step(detectionLoops() + 20);
    assertTrue(indexer.hasPiece(), "the injected piece was never detected by the indexer");
  }

  // --------------------------------------------------------------------------------- tests ----

  /** The robot should home itself and settle into stowed with no input. */
  @Test
  void startsInHomingAndReachesStowed() {
    assertEquals(
        Superstructure.SystemState.HOMING,
        superstructure.getSystemState(),
        "the machine must boot into HOMING; nothing is known about the arm or the turret yet");
    assertFalse(intake.isHomed(), "the intake cannot be homed before it has run");
    assertFalse(turret.isHomed(), "the turret cannot be homed before it has run");

    homeAndSettle();

    assertTrue(intake.isHomed(), "reaching STOWED without a homed intake would be a lie");
    assertTrue(turret.isHomed(), "reaching STOWED without a homed turret would be a lie");

    // The robot is asked for nothing and is therefore doing nothing dangerous, but the flywheel is
    // deliberately NOT stopped: STOWED idles it so the next shot does not pay the whole spin-up.
    assertNotEquals(
        Shooter.SystemState.COASTING,
        shooter.getSystemState(),
        "STOWED must idle the flywheel rather than stop it");
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

  /** A score request with no game piece must never reach a scoring state. */
  @Test
  void scoreRequestWithoutPieceNeverReachesScoring() {
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

    assertFalse(indexer.hasPiece(), "this case is only meaningful with an empty indexer");
    assertEquals(
        Superstructure.SystemState.SPINNING_UP,
        superstructure.getSystemState(),
        "an empty robot asked to score must pre-spin, not score");

    // Pre-spinning rather than refusing is the point: the driver holding the trigger with an empty
    // robot still gets the wheel brought up, so the shot is ready the instant a piece arrives.
    assertNotEquals(
        Indexer.SystemState.FEEDING, indexer.getSystemState(), "the belt ran with nothing aboard");
  }

  /** Scoring on the move must require a usable shot solution. */
  @Test
  void scoreOnMoveRequiresUsableShotSolution() {
    // The SHIPPED solver, reading Constants.Shot's all-zero tables. This is the robot as it exists
    // on this branch, and the behaviour asserted here is the behaviour on a real field until M11
    // measures the tables.
    placeRobotAt(-3.0, 0.0);
    homeAndSettle();
    acquirePiece();

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

  /** The intake and turret must not be able to occupy each other's space. */
  @Test
  void intakeAndTurretInterlockHoldsBothWays() {
    homeAndSettle();

    double arcMin = Constants.Turret.kIntakeConflictMinRad.get();
    double arcMax = Constants.Turret.kIntakeConflictMaxRad.get();
    double insideArc = 0.5 * (arcMin + arcMax);

    // ---- Direction 1: the intake must refuse to deploy into a turret that is in the arc.
    //
    // The turret is a stand-in that reports itself parked inside the arc and ignores commands --
    // which is the checkpoint's manual test written down: park the turret at 180 degrees, then ask
    // for the intake. A turret driven there with TurretIOSim was tried first and is not usable
    // here: the placeholder pivot gains leave the intake ringing about its stow setpoint, so
    // Intake.isStowed() flickers, the turret's own guard fires mid-sweep and it stops short of the
    // arc. That is the interlock working, but it tests the other direction, and it would make this
    // case depend on gains M6 has not measured yet.
    //
    // The order matters, and the first version of this case got it wrong. Requesting the intake and
    // the turret together is not a test of this direction at all: the intake is free to deploy
    // while the turret is still at stow, so it does, and the refusal that follows belongs to
    // direction 2. The turret has to be established in the arc first.
    double parkedInArc = insideArc;
    buildRobot(
        new ShootingSolver(),
        new IntakeIOSim(),
        new IndexerIOSim(),
        new TurretIO() {
          @Override
          public void updateInputs(TurretIOInputs inputs) {
            inputs.motorConnected = true;
            inputs.absEncoderConnected = true;
            inputs.positionRad = parkedInArc;
            inputs.absolutePositionRad = parkedInArc;
          }
        },
        new ShooterIOSim(),
        new FloorRollerIOSim());
    homeAndSettle();

    assertTrue(
        TurretUnwrapper.passesThroughArcWrapped(
            turret.getPositionRad(), turret.getPositionRad(), arcMin, arcMax),
        "the turret is not in the conflict arc, so nothing is being tested; it is at "
            + turret.getPositionRad());

    superstructure.setWantedState(Superstructure.WantedState.INTAKE);
    for (int i = 0; i < 300; i++) {
      step(1);
      assertNotEquals(
          Intake.SystemState.DEPLOYING,
          intake.getSystemState(),
          "the intake started deploying into a turret sitting in its arc");
      assertNotEquals(
          Intake.SystemState.INTAKING,
          intake.getSystemState(),
          "the intake deployed into a turret sitting in its arc");
    }

    // EJECT is covered by the same guard, not just the two requests named DEPLOY. The eject angle
    // puts the arm outside stow exactly as the deploy angle does, so a guard that covered only
    // DEPLOY would leave the collision reachable through a different button.
    superstructure.setWantedState(Superstructure.WantedState.EJECT);
    for (int i = 0; i < 200; i++) {
      step(1);
      assertNotEquals(
          Intake.SystemState.EJECTING,
          intake.getSystemState(),
          "the intake reached the eject angle past a closed guard");
    }

    // ---- Direction 2: the turret must refuse to sweep through a deployed intake.
    //
    // Rebuilt rather than continued, because direction 1 deliberately left the turret parked in the
    // arc, and the intake cannot come out at all from there.
    buildRobot(
        new ShootingSolver(),
        new IntakeIOSim(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());
    homeAndSettle();

    superstructure.setWantedState(Superstructure.WantedState.INTAKE);
    step(120);
    assertFalse(
        intake.isStowed(),
        "the intake never left stow, so the turret guard was never put under pressure");

    boolean sawRefusal = false;
    for (int i = 0; i < 300; i++) {
      step(1);
      turret.setGoal(Rotation2d.fromRadians(insideArc), 0.0);

      double commanded = turret.getCommandedPositionRad();
      assertFalse(
          TurretUnwrapper.passesThroughArcWrapped(commanded, commanded, arcMin, arcMax),
          "the turret was commanded into the deployed intake, at " + commanded);
      sawRefusal |= turret.getSystemState() == Turret.SystemState.OUT_OF_RANGE;
    }
    assertTrue(
        sawRefusal,
        "the turret never reported OUT_OF_RANGE, so it may have been held back by something other"
            + " than the interlock");
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

    // The piece stays out of the indexer so the machine stays in INTAKING rather than walking on
    // to INDEXING, which would fire onExit(INTAKING) and confuse the count with a real transition.
    superstructure.setWantedState(Superstructure.WantedState.INTAKE);
    step(200);
    assertEquals(
        Superstructure.SystemState.INTAKING,
        superstructure.getSystemState(),
        "this case needs the machine to sit in INTAKING");
    assertEquals(
        List.of(Constants.Vision.kAprilTagPipelineIndex, Constants.Vision.kGamePiecePipelineIndex),
        intakeCameraIO.pipelines,
        "entering INTAKING must switch to the game-piece pipeline exactly once, however long the"
            + " state is held");

    superstructure.setWantedState(Superstructure.WantedState.DEFAULT);
    step(200);
    assertEquals(
        List.of(
            Constants.Vision.kAprilTagPipelineIndex,
            Constants.Vision.kGamePiecePipelineIndex,
            Constants.Vision.kAprilTagPipelineIndex),
        intakeCameraIO.pipelines,
        "leaving INTAKING must switch back to AprilTags exactly once");
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
    acquirePiece();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    step(40);

    // No vision has ever been fed in this case, so the pose is as stale as it can be and terms 5
    // and 6 are false on top of the flywheel term. The gate is correctly shut.
    assertEquals(
        Superstructure.SystemState.SCORING_ON_THE_MOVE,
        superstructure.getSystemState(),
        "with a piece and a usable solution the machine must enter the scoring state");
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
    acquirePiece();

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
    acquirePiece();

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
    assertEquals(Superstructure.SystemState.STOWED, superstructure.getSystemState());

    superstructure.setWantedState(Superstructure.WantedState.INTAKE);
    step(20);
    assertEquals(Superstructure.SystemState.INTAKING, superstructure.getSystemState());

    acquirePiece();
    assertEquals(
        Superstructure.SystemState.INDEXING,
        superstructure.getSystemState(),
        "a piece arriving during INTAKE must promote the machine to INDEXING");

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

    // Back to a scoring posture. The piece has to still be aboard for any of the three to be
    // reachable, which acquirePiece guaranteed and EJECT did not undo -- the simulated piece is a
    // flag on the IO, not something the eject can physically remove.
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
   * <p>The block is produced the way it happens on a field: a piece is collected, which leaves the
   * intake out, and the shot that follows wants the turret pointed straight backwards -- through
   * the arc the intake is standing in.
   */
  @Test
  void aBlockedTurretDivertsScoringToWaitingForTurret() {
    buildRobot(
        usableSolver(),
        new IntakeIOSim(),
        new IndexerIOSim(),
        new TurretIOSim(),
        new ShooterIOSim(),
        new FloorRollerIOSim());

    // +3 m in front of a goal at the origin, facing along +X, so the shot wants the turret at 180
    // degrees -- squarely inside the 150-210 degree conflict arc.
    placeRobotAt(3.0, 0.0);
    homeAndSettle();
    acquirePiece();
    assertFalse(intake.isStowed(), "this case needs the intake still out after the pickup");

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
    acquirePiece();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    step(30);
    assertEquals(
        Superstructure.SystemState.SCORING_ON_THE_MOVE,
        superstructure.getSystemState(),
        "precondition: a healthy robot with a piece and a solution scores");

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
    acquirePiece();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_ON_MOVE);
    step(30);
    assertEquals(
        Superstructure.SystemState.SCORING_ON_THE_MOVE,
        superstructure.getSystemState(),
        "precondition: a healthy robot with a piece and a solution scores");

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
    acquirePiece();

    superstructure.setWantedState(Superstructure.WantedState.SCORE_STATIONARY);
    stepWithVision(200);

    // Everything the test CAN make true is true: a usable solution, a piece aboard, a stationary
    // robot, fresh tight vision, and no faults.
    assertTrue(superstructure.getLastShot().isUsable(), "term 1 should be satisfiable");
    assertTrue(indexer.hasPiece(), "term 4 should be satisfiable");
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
    assertFalse(indexer.hasPiece(), "a dead indexer invented a game piece");
    assertFalse(turret.atGoal(), "a dead turret reported itself on target");
    assertFalse(
        shooter.atSpeed(), "a dead flywheel reported itself at speed; both readings are zero");

    // And FAULTED still clears on request, so a dead robot is recoverable rather than bricked.
    superstructure.setWantedState(Superstructure.WantedState.HOME);
    step(3);
    assertEquals(Superstructure.SystemState.HOMING, superstructure.getSystemState());
  }
}
