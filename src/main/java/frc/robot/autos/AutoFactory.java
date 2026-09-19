package frc.robot.autos;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;

/**
 * Owns everything PathPlanner needs to exist, in one place, so that {@code Drive} stays a
 * drivetrain rather than becoming a path-follower.
 *
 * <p>It holds the single {@link RobotConfig} instance that both {@code AutoBuilder} and {@code
 * Drive}'s setpoint generator share. If those two ever disagree about robot mass or wheel
 * coefficient of friction, the setpoint generator silently clips the speeds PathPlanner asks for
 * and every path undershoots -- a failure that looks like a tuning problem and is not.
 *
 * <p>It wires {@code AutoBuilder} to {@code RobotState} for the pose supplier (not to {@code
 * Drive}, which no longer owns a pose estimator), registers the {@code NamedCommands} vocabulary
 * that {@code .auto} files may use, installs {@code LocalADStarAK} so pathfinding is replayable,
 * and hooks {@code PathPlannerLogging} so the active path lands in the log.
 *
 * <p>The concrete capability bought: a student can add an auto in the PathPlanner GUI, drop in a
 * marker named {@code ScoreOnMove}, and it works with no Java change at all.
 */
public class AutoFactory {
  /**
   * The one {@link RobotConfig} for the whole robot.
   *
   * <p>{@code Drive}'s setpoint generator and {@code AutoBuilder} must both use THIS instance.
   */
  public static RobotConfig PP_CONFIG = null;

  private final Drive drive;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Indexer indexer;
  private final Turret turret;
  private final Vision vision;

  /**
   * Builds the factory and performs all PathPlanner configuration.
   *
   * <p>Construct this exactly once, from {@code RobotContainer}, after {@code
   * RobotState.initialize()}.
   *
   * @param drive the drivetrain
   * @param superstructure the cross-mechanism state machine
   * @param intake the pivoting intake
   * @param indexer the indexer
   * @param turret the turret
   * @param vision the vision subsystem
   */
  public AutoFactory(
      Drive drive,
      Superstructure superstructure,
      Intake intake,
      Indexer indexer,
      Turret turret,
      Vision vision) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.intake = intake;
    this.indexer = indexer;
    this.turret = turret;
    this.vision = vision;
    // TODO: Call loadRobotConfig(), configureAutoBuilder() and registerNamedCommands(), in that
    // order. The order matters: AutoBuilder needs PP_CONFIG, and building any PathPlannerAuto
    // needs the named commands already registered.
  }

  /**
   * Loads the shared robot configuration, preferring the PathPlanner GUI settings.
   *
   * <p><b>Outputs:</b> assigns {@link #PP_CONFIG}; raises a fault alert if it had to fall back.
   */
  private void loadRobotConfig() {
    // TODO: Try PP_CONFIG = RobotConfig.fromGUISettings(), which reads
    // src/main/deploy/pathplanner/settings.json and throws IOException / ParseException.
    // On failure, fall back to a hand-built RobotConfig from Constants.Drivetrain and
    // Drive.getModuleTranslations(), AND raise a FaultReporter ERROR alert -- a stale or missing
    // settings.json must be loud, because the symptom is merely "autos are a bit off".
    // TODO: VERIFY the RobotConfig and ModuleConfig constructor argument order against
    // PathplannerLib 2026.1.2 before relying on the fallback.
  }

  /**
   * Wires {@code AutoBuilder} to the robot's pose, velocity and output plumbing.
   *
   * <p><b>Outputs:</b> configures AutoBuilder, the pathfinder, and the PathPlanner logging hooks.
   */
  private void configureAutoBuilder() {
    // TODO: AutoBuilder.configure(
    //     () -> RobotState.getInstance().getEstimatedPose(),
    //     pose -> RobotState.getInstance().resetPose(pose, drive.getRawGyroRotation(),
    //         drive.getModulePositions()),
    //     () -> RobotState.getInstance().getRobotRelativeSpeeds(),
    //     (speeds, feedforwards) -> drive.runVelocity(speeds, feedforwards),
    //     new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0),
    //         new PIDConstants(5.0, 0.0, 0.0)),
    //     PP_CONFIG, AllianceFlipUtil::shouldFlip, drive);
    // Then Pathfinding.setPathfinder(new LocalADStarAK()) and the two PathPlannerLogging callbacks
    // from the stock template: setLogActivePathCallback -> "Odometry/Trajectory" and
    // setLogTargetPoseCallback -> "Odometry/TrajectorySetpoint".
  }

  /**
   * Registers the marker vocabulary that {@code .auto} files may use.
   *
   * <p><b>Outputs:</b> populates PathPlanner's global NamedCommands registry.
   */
  private void registerNamedCommands() {
    // TODO: Register each name below EXACTLY as spelled. These strings are typed by hand into the
    // PathPlanner GUI, and a typo produces a silently-skipped marker rather than an error:
    //   "Intake"            -> superstructure.setWantedStateCommand(WantedState.INTAKE)
    //   "StopIntake"        -> superstructure.setWantedStateCommand(WantedState.DEFAULT)
    //   "SpinUp"            -> superstructure.setWantedStateCommand(WantedState.SPIN_UP)
    //   "ScoreOnMove"       -> superstructure.scoreUntilEmptyCommand()
    //   "ScoreStationary"   -> scoreStationaryCommand()
    //   "WaitForPiece"      -> waitForPieceCommand(2.0)
    //   "Eject"             -> superstructure.setWantedStateCommand(WantedState.EJECT)
    //   "Home"              -> superstructure.setWantedStateCommand(WantedState.HOME)
    //   "StowTurret"        -> turret.setWantedStateCommand(Turret.WantedState.STOW)
    //   "GamePiecePipeline" -> Commands.runOnce(() -> vision.setIntakePipeline(1))
    //   "TagPipeline"       -> Commands.runOnce(() -> vision.setIntakePipeline(0))
  }

  /**
   * Builds a routine from a {@code .auto} file drawn in the PathPlanner GUI.
   *
   * @param autoName the auto file's name, without extension
   * @return the routine, or a do-nothing command if the file is missing
   */
  public Command buildAuto(String autoName) {
    // TODO: return new PathPlannerAuto(autoName), wrapped in try/catch. On failure return
    // Commands.none() AND fire a FaultReporter alert, so a renamed file cannot crash robotInit at
    // an event.
    return Commands.none();
  }

  /**
   * Builds a command that follows one path file.
   *
   * @param pathName the path file's name, without extension
   * @return the follow command, or a do-nothing command if the file is missing
   */
  public Command followPath(String pathName) {
    // TODO: PathPlannerPath.fromPathFile(pathName) inside try/catch for IOException and
    // org.json.simple.parser.ParseException, then AutoBuilder.followPath(path). Return
    // Commands.none() plus an alert on failure.
    return Commands.none();
  }

  /**
   * Scores from a standstill until the indexer is empty.
   *
   * @return the scoring command
   */
  public Command scoreStationaryCommand() {
    // TODO: superstructure.setWantedStateCommand(SCORE_STATIONARY)
    //     .andThen(Commands.waitUntil(() -> !indexer.hasPiece()))
    //     .withTimeout(Constants.Auto.kScoreTimeoutSec.get())
    //     .andThen(superstructure.setWantedStateCommand(DEFAULT));
    return Commands.none();
  }

  /**
   * Waits for the indexer to report a piece.
   *
   * @param timeoutSec how long to wait before giving up, in seconds
   * @return the wait command
   */
  public Command waitForPieceCommand(double timeoutSec) {
    // TODO: Commands.waitUntil(indexer::hasPiece).withTimeout(timeoutSec). Use this after an intake
    // marker so a routine does not drive away from a piece it never actually collected.
    return Commands.none();
  }

  /**
   * Snaps the pose estimate to a path's starting pose.
   *
   * @param pathName the path whose start pose to adopt
   * @return a command that performs the reset
   */
  public Command resetPoseToPathStart(String pathName) {
    // TODO: Commands.runOnce(() -> { load the path, take
    // AllianceFlipUtil.shouldFlip() ? p.flipPath().getStartingHolonomicPose().orElseThrow()
    //                               : p.getStartingHolonomicPose().orElseThrow();
    // then RobotState.getInstance().resetPose(start, drive.getRawGyroRotation(),
    // drive.getModulePositions()); }).
    // TODO: VERIFY getStartingHolonomicPose()'s Optional return type against PathplannerLib
    // 2026.1.2.
    return Commands.none();
  }

  /**
   * The expected starting pose of a named auto, for the pre-match preview.
   *
   * @param autoName the auto file's name, without extension
   * @return the alliance-flipped starting pose, or empty if it cannot be determined
   */
  public Optional<Pose2d> getStartingPose(String autoName) {
    // TODO: PathPlannerAuto.getStartingPoseFromAutoFile(autoName), alliance-flipped, wrapped in
    // Optional and try/catch. TODO: VERIFY that static helper's exact name in PathplannerLib
    // 2026.1.2.
    return Optional.empty();
  }

  /**
   * The constraints used for on-the-fly pathfinding.
   *
   * @return shared pathfinding constraints, from {@code Constants.Auto}
   */
  public static PathConstraints defaultPathfindConstraints() {
    // Implemented rather than stubbed: returning null here would NPE inside PathPlanner.
    return new PathConstraints(
        Constants.Auto.kPathfindMaxVel.get(),
        Constants.Auto.kPathfindMaxAccel.get(),
        Constants.Auto.kPathfindMaxAngVel.get(),
        Constants.Auto.kPathfindMaxAngAccel.get());
  }

  /**
   * The drivetrain, so {@code AutoRoutines} need not hold its own reference.
   *
   * @return the drivetrain
   */
  public Drive getDrive() {
    return drive;
  }

  /**
   * The superstructure, so {@code AutoRoutines} need not hold its own reference.
   *
   * @return the superstructure
   */
  public Superstructure getSuperstructure() {
    return superstructure;
  }

  /**
   * The indexer, used by routines that branch on whether a piece was actually collected.
   *
   * @return the indexer
   */
  public Indexer getIndexer() {
    return indexer;
  }

  /**
   * The turret, used by routines that pre-position it during a long drive.
   *
   * @return the turret
   */
  public Turret getTurret() {
    return turret;
  }
}
