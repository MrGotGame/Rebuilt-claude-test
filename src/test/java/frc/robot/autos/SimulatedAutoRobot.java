package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.shooting.ShootingSolver;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.floorroller.FloorRoller;
import frc.robot.subsystems.floorroller.FloorRollerIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.IdealPivotIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.QuestNavIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.AllianceFlipUtil;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * The whole robot over simulated IO, stepped by the real command scheduler, for the M13 suites.
 *
 * <p>Added for M13. The superstructure suite steps each subsystem's periodic by hand and never
 * moves the drivetrain; an auto needs the opposite. So this builds {@code Drive} over {@code
 * ModuleIOSim}, so PathPlanner's commands really move the simulated chassis and {@code RobotState}
 * really measures it, and it runs {@link CommandScheduler#run()} each loop, exactly as {@code
 * Robot.robotPeriodic} does, so compositions are scheduled, interrupted and ended by the same code
 * that will run them on the robot.
 *
 * <p>Three things stand in for the world:
 *
 * <ul>
 *   <li><b>The goal is at the field origin</b> ({@code Constants.Field} is still all zeros), and
 *       the solver is the superstructure suite's firing solver, whose explicit RPM fit lets the
 *       readiness gate open. With the shipped all-zero tables nothing could ever fire.
 *   <li><b>A fresh, tight vision fix</b> arrives every loop at the current estimate, so the gate's
 *       vision terms hold. The fix never moves the pose, so it cannot help or hurt path following.
 *   <li><b>A hopper model.</b> While the indexer feeds, one piece leaves every ten loops (the
 *       cadence of the superstructure suite's bursts) and knocks the simulated flywheel. While the
 *       intake collects, one piece comes aboard every {@link #INTAKE_LOOPS_PER_PIECE} loops from a
 *       finite supply on the field.
 * </ul>
 *
 * <p>It also watches, every loop, the one thing M13's checkpoint forbids outright: the robot moving
 * while the intake is above its piece-retaining range ({@code Superstructure/StowedWhileMoving},
 * computed here the same way, since the logger records nothing under test).
 */
final class SimulatedAutoRobot {
  /** One robot loop, seconds. */
  static final double LOOP_SEC = 0.02;

  /** Loops of intaking per piece collected, while the field supply lasts. */
  static final int INTAKE_LOOPS_PER_PIECE = 15;

  /** Loops of feeding per piece fired, as in the superstructure suite's bursts. */
  static final int FEED_LOOPS_PER_PIECE = 10;

  /** The pieces the rules put in the robot at the start of a match. */
  static final int PRELOAD = 8;

  /** Flywheel RPM per m/s of muzzle speed, zero offset: the superstructure suite's firing fit. */
  private static final double FIRING_RPM_PER_MPS = 250.0;

  final Drive drive;
  final Intake intake;
  final SlowPivotIO pivotIo;
  final FloorRoller floorRoller;
  final Indexer indexer;
  final Turret turret;
  final ShooterIOSim shooterIo;
  final Shooter shooter;
  final Vision vision;
  final Superstructure superstructure;
  final AutoFactory factory;
  final AutoRoutines routines;

  /** Pieces in the hopper right now. */
  int piecesAboard = PRELOAD;

  /** Pieces still on the field for the intake to collect. */
  int fieldPieces = 0;

  /** Pieces the hopper model has fired into the flywheel. */
  int piecesFired = 0;

  /** Loops, since the routine began, in which the robot moved with the intake not retaining. */
  int stowedWhileMovingLoops = 0;

  /** Loops since the routine began. */
  int loops = 0;

  /**
   * The routine's loop on which the chassis first moved, or -1 while it has not.
   *
   * <p>Moving is judged as the superstructure judges it for {@code StowedWhileMoving}: faster than
   * {@code kMaxChassisSpeedForStationaryShotMetersPerSec}.
   */
  int firstMovingLoop = -1;

  /** Whether the intake was retaining pieces on {@link #firstMovingLoop}. */
  boolean retainingWhenFirstMoved = false;

  /** Whether the intake has retained pieces at any point since the routine began. */
  boolean hasRetained = false;

  /**
   * Loops, before the intake first retained, in which the robot did anything: moved, or had the
   * shooter side out of rest.
   *
   * <p>The checkpoint's clause is that every routine STARTS with the wait for the intake. Judged
   * only by movement, a routine that opens with a stationary preload would pass without the wait,
   * because the burst takes long enough for the intake to come down anyway. This is the clause
   * itself: until the intake retains, nothing at all is asked of the robot.
   */
  int actedBeforeRetainingLoops = 0;

  private int feedingLoops = 0;
  private int intakingLoops = 0;

  /**
   * Builds the robot, places it, homes it, and enables it in autonomous on the given alliance.
   *
   * <p>The caller must already have initialised the HAL and paused the clock.
   *
   * @param blueStart where the robot stands, drawn blue: on red it stands on the flip of this
   * @param paths the paths the routines may follow, in place of the deploy folder
   * @param red true for the red alliance
   */
  SimulatedAutoRobot(Pose2d blueStart, AutoFactory.PathSource paths, boolean red) {
    DriverStationSim.setAllianceStationId(red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    RobotState.initialize(new SwerveDriveKinematics(Drive.getModuleTranslations()));

    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
    pivotIo = new SlowPivotIO();
    intake = new Intake(pivotIo, IdealPivotIO.GEOMETRY);
    floorRoller = new FloorRoller(new FloorRollerIOSim());
    indexer = new Indexer(new IndexerIOSim());
    turret = new Turret(new TurretIOSim());
    shooterIo = new ShooterIOSim();
    shooter = new Shooter(shooterIo);
    vision =
        new Vision(
            new VisionIO() {},
            new VisionIO() {},
            new QuestNavIO() {},
            turret::getPositionRad,
            turret::getVelocityRadPerSec,
            intake::isAtIntakingAngle);
    superstructure =
        new Superstructure(
            drive,
            intake,
            floorRoller,
            indexer,
            turret,
            shooter,
            vision,
            new ShootingSolver(
                new double[] {1.0, 4.0, 7.0},
                new double[] {0.50, 0.80, 1.10},
                () -> FIRING_RPM_PER_MPS,
                () -> 0.0));
    factory = new AutoFactory(drive, superstructure, intake, shooter, vision, paths);
    routines = new AutoRoutines(factory);

    // Flipped HERE, after the driver station has been told the alliance. AllianceFlipUtil reads the
    // alliance when it is called, and the first version of this harness left the flip to its
    // callers, who called it before the constructor had set red. Every "red" robot stood on the
    // blue start; the routines' own pose resets hid it, and a path case started 8 m from its path.
    placeAt(AllianceFlipUtil.apply(blueStart));
    homeAndSettle();
  }

  /**
   * Puts the pose estimate at a field pose, as a pose reset would.
   *
   * @param pose the pose to adopt, blue-origin field coordinates
   *     <p><b>Outputs:</b> resets {@code RobotState}.
   */
  void placeAt(Pose2d pose) {
    RobotState.getInstance()
        .resetPose(pose, drive.getRawGyroRotation(), drive.getModulePositions());
  }

  /**
   * Steps until the robot has homed and is resting, as it would be sitting on the field.
   *
   * <p><b>Outputs:</b> runs the robot; resets the hopper model's counters afterwards.
   */
  private void homeAndSettle() {
    int limit = (int) (Constants.Superstructure.kHomingTimeoutSec.get() / LOOP_SEC) + 30;
    for (int i = 0; i < limit; i++) {
      step();
    }
    assertEquals(
        Superstructure.SystemState.RESTING,
        superstructure.getSystemState(),
        "the simulated robot never finished homing");
    assertEquals(
        Superstructure.IntakeMode.STOWED,
        superstructure.getIntakeMode(),
        "the robot must start the match with the intake stowed");
    // From here the pivot takes real time to move, so "nothing moves before the intake retains"
    // is a claim a routine can fail. See SlowPivotIO.
    pivotIo.slow = true;
    loops = 0;
    stowedWhileMovingLoops = 0;
    firstMovingLoop = -1;
    hasRetained = false;
    actedBeforeRetainingLoops = 0;
  }

  /**
   * Does what {@code Robot.autonomousInit} does, then schedules a routine.
   *
   * @param routine the routine to run
   *     <p><b>Outputs:</b> zeroes the shot count, deploys the intake, and schedules the routine.
   */
  void startAuto(Command routine) {
    shooter.resetShotCount();
    superstructure.onEnable();
    CommandScheduler.getInstance().schedule(routine);
  }

  /**
   * Runs the robot until a command finishes or a limit is reached.
   *
   * @param command the command to watch; must already be scheduled
   * @param maxLoops how many loops to allow
   * @return the loops it took, or {@code maxLoops + 1} if it never finished
   */
  int runUntilFinished(Command command, int maxLoops) {
    for (int i = 1; i <= maxLoops; i++) {
      step();
      if (!command.isScheduled()) {
        return i;
      }
    }
    return maxLoops + 1;
  }

  /**
   * Advances the robot a number of loops.
   *
   * @param count how many loops
   *     <p><b>Outputs:</b> as {@link #step()}.
   */
  void step(int count) {
    for (int i = 0; i < count; i++) {
      step();
    }
  }

  /**
   * One robot loop: the world, then the scheduler, then the clock.
   *
   * <p><b>Outputs:</b> moves pieces, feeds a vision fix, runs every subsystem and command, and
   * records the stowed-while-moving watch.
   */
  void step() {
    RobotState state = RobotState.getInstance();
    state.addVisionObservation(
        new RobotState.PoseObservation(
            state.getEstimatedPose(),
            Timer.getFPGATimestamp(),
            VecBuilder.fill(0.02, 0.02, 9999.0),
            RobotState.PoseSource.LIMELIGHT_TURRET));

    if (indexer.getSystemState() == Indexer.SystemState.FEEDING && piecesAboard > 0) {
      if (++feedingLoops % FEED_LOOPS_PER_PIECE == 0) {
        shooterIo.injectShot();
        piecesAboard--;
        piecesFired++;
      }
    }
    if (superstructure.getIntakeMode() == Superstructure.IntakeMode.INTAKING && fieldPieces > 0) {
      if (++intakingLoops % INTAKE_LOOPS_PER_PIECE == 0) {
        fieldPieces--;
        piecesAboard++;
      }
    }

    CommandScheduler.getInstance().run();

    boolean moving =
        speedMetersPerSec()
            > Constants.Superstructure.kMaxChassisSpeedForStationaryShotMetersPerSec.get();
    if (moving && !intake.isRetainingPieces()) {
      stowedWhileMovingLoops++;
    }
    hasRetained |= intake.isRetainingPieces();
    if (!hasRetained
        && (moving || superstructure.getSystemState() != Superstructure.SystemState.RESTING)) {
      actedBeforeRetainingLoops++;
    }
    if (moving && firstMovingLoop < 0) {
      firstMovingLoop = loops;
      retainingWhenFirstMoved = intake.isRetainingPieces();
    }
    loops++;
    SimHooks.stepTiming(LOOP_SEC);
  }

  /**
   * The chassis speed over the ground.
   *
   * @return the measured field-relative speed, m/s
   */
  double speedMetersPerSec() {
    ChassisSpeeds speeds = RobotState.getInstance().getFieldRelativeSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /**
   * The pose estimate.
   *
   * @return where RobotState thinks the robot is
   */
  Pose2d pose() {
    return RobotState.getInstance().getEstimatedPose();
  }

  /**
   * Stops everything this robot started, so the next case begins clean.
   *
   * <p><b>Outputs:</b> cancels all commands, unregisters every subsystem, and disables the driver
   * station.
   */
  void close() {
    CommandScheduler.getInstance().cancelAll();
    for (Subsystem subsystem :
        new Subsystem[] {
          drive, intake, floorRoller, indexer, turret, shooter, vision, superstructure
        }) {
      CommandScheduler.getInstance().unregisterSubsystem(subsystem);
    }
    DriverStationSim.resetData();
  }

  /**
   * A path source over paths built in a test.
   *
   * @param paths the paths, each answering to its {@code name}
   * @return a source that serves them, and fails like a missing file for any other name; an auto
   *     name is served as the one path of that name
   */
  static AutoFactory.PathSource pathsOf(PathPlannerPath... paths) {
    Map<String, PathPlannerPath> byName = new HashMap<>();
    for (PathPlannerPath path : paths) {
      byName.put(path.name, path);
    }
    return new AutoFactory.PathSource() {
      @Override
      public PathPlannerPath path(String name) throws Exception {
        PathPlannerPath path = byName.get(name);
        if (path == null) {
          throw new FileNotFoundException(name);
        }
        return path;
      }

      @Override
      public List<PathPlannerPath> autoPaths(String autoName) throws Exception {
        return List.of(path(autoName));
      }
    };
  }

  /**
   * A straight path between two points, facing a fixed heading the whole way.
   *
   * @param name the name the path answers to
   * @param from start position, blue-origin field coordinates; its rotation is the chassis heading
   * @param to end position; its rotation is ignored
   * @param maxVelMetersPerSec the path's speed limit
   * @param endVelMetersPerSec the speed at which it ends; nonzero for a scoring path
   * @return the path
   */
  static PathPlannerPath straightPath(
      String name, Pose2d from, Pose2d to, double maxVelMetersPerSec, double endVelMetersPerSec) {
    Rotation2d travel = to.getTranslation().minus(from.getTranslation()).getAngle();
    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                List.of(
                    new Pose2d(from.getTranslation(), travel),
                    new Pose2d(to.getTranslation(), travel))),
            new PathConstraints(maxVelMetersPerSec, 3.0, 6.0, 12.0),
            new IdealStartingState(0.0, from.getRotation()),
            new GoalEndState(endVelMetersPerSec, from.getRotation()));
    path.name = name;
    return path;
  }
}
