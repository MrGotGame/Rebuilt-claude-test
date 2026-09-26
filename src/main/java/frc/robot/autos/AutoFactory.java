package frc.robot.autos;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

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
 *
 * <p>M10b step 9 (plan section 2.4) moved the autos onto the hopper model. The robot has no piece
 * sensor, so nothing here waits for a piece or branches on one: the intake is an independent
 * request that a path can hold while it scores, a scoring leg ends on the superstructure's "likely
 * empty" belief or a timeout, and the only piece count available is the flywheel's shot count. That
 * is why this factory holds the shooter and no longer holds the indexer.
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
  private final Shooter shooter;
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
   * @param shooter the flywheel, whose shot count is the only piece count the robot has
   * @param turret the turret
   * @param vision the vision subsystem
   */
  public AutoFactory(
      Drive drive,
      Superstructure superstructure,
      Intake intake,
      Shooter shooter,
      Turret turret,
      Vision vision) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.intake = intake;
    this.shooter = shooter;
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
    // PathPlanner GUI, and a typo produces a silently-skipped marker rather than an error. The
    // table in plan section 8.2 is the contract; change both in the same commit.
    //   "Intake"            -> Commands.runOnce(() -> superstructure.setIntakeRequested(true))
    //   "StopIntake"        -> Commands.runOnce(() -> superstructure.setIntakeRequested(false))
    //                          (the roller stops; the intake stays DEPLOYED, so the hopper keeps
    //                          its pieces)
    //   "StowIntake"        -> superstructure.setWantedStateCommand(WantedState.STOW)
    //                          (rarely: raising the intake can dump the hopper)
    //   "SpinUp"            -> superstructure.setWantedStateCommand(WantedState.SPIN_UP)
    //   "ScoreOnMove"       -> superstructure.scoreUntilEmptyCommand()
    //   "ScoreStationary"   -> scoreStationaryCommand()
    //   "Eject"             -> superstructure.setWantedStateCommand(WantedState.EJECT)
    //   "Home"              -> superstructure.setWantedStateCommand(WantedState.HOME)
    //   "StowTurret"        -> turret.setWantedStateCommand(Turret.WantedState.STOW)
    //   "GamePiecePipeline" -> Commands.runOnce(() -> vision.setIntakePipeline(1))
    //   "TagPipeline"       -> Commands.runOnce(() -> vision.setIntakePipeline(0))
    //
    // "Intake" and "StopIntake" set the intake request, which is independent of the shooter-side
    // wanted state, so one path can intake and score at once. They used to request
    // WantedState.INTAKE, which M10b step 7 removed, and DEFAULT, which would cancel any scoring in
    // progress. "WaitForPiece" is gone for good: with no piece sensor there is nothing to wait
    // for, and a marker that always times out only costs the auto time.
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
   * Scores from a standstill until the hopper is probably empty, or the timeout.
   *
   * @return the scoring command
   */
  public Command scoreStationaryCommand() {
    // TODO: superstructure.setWantedStateCommand(SCORE_STATIONARY)
    //     .andThen(Commands.waitUntil(superstructure::likelyEmpty))
    //     .withTimeout(Constants.Auto.kScoreTimeoutSec.get())
    //     .andThen(superstructure.setWantedStateCommand(DEFAULT));
    // Read the timeout inside Commands.defer, as Superstructure.scoreUntilEmptyCommand() does, so
    // a retuned kScoreTimeoutSec takes effect without a reboot. (It used to wait for
    // !indexer.hasPiece(); M10b deleted the piece model, and likelyEmpty() is the belief that
    // replaced it.)
    return Commands.none();
  }

  /**
   * Logs how many pieces the flywheel has counted since autonomous began, as {@code
   * Auto/PreloadFired}.
   *
   * <p>Added for M10b step 9 (plan section 2.4). The rules put exactly 8 pieces in the robot at the
   * start of a match and the shot count is zeroed at {@code autonomousInit}, so logged at the end
   * of the preload burst this is a free calibration of the shot detector in every match: anything
   * other than 8 means pieces merged inside one refractory window, a shot was missed, or pieces
   * stayed in the hopper. It gates nothing.
   *
   * @return an instant command that logs the shooter's current shot count
   */
  public Command logPreloadFiredCommand() {
    // Read when the command RUNS, not when it is built: routines are built once, when the chooser
    // is populated at robot init, and a count captured then would always be zero.
    return Commands.runOnce(() -> Logger.recordOutput("Auto/PreloadFired", shooter.getShotCount()));
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
   * The intake, so {@code AutoRoutines} can wait for it to reach a piece-retaining angle.
   *
   * <p>Replaces {@code getIndexer()}, whose only use was branching on whether a piece had been
   * collected -- a question the robot can no longer answer (M10b step 9).
   *
   * @return the intake
   */
  public Intake getIntake() {
    return intake;
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
