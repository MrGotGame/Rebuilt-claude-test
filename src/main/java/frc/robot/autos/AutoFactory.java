package frc.robot.autos;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FaultReporter;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.Callable;
import org.littletonrobotics.junction.Logger;

/**
 * Owns everything PathPlanner needs to exist, in one place, so that {@code Drive} stays a
 * drivetrain rather than becoming a path-follower.
 *
 * <p>It hands {@code AutoBuilder} the single {@link RobotConfig} instance, {@link Drive#PP_CONFIG},
 * that {@code Drive}'s setpoint generator already uses. If those two ever disagreed about robot
 * mass or wheel coefficient of friction, the setpoint generator would silently clip the speeds
 * PathPlanner asks for and every path would undershoot -- a failure that looks like a tuning
 * problem and is not.
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
   * Relative difference above which a {@code settings.json} value is reported as disagreeing with
   * the robot's config. Half a percent: well above rounding in a GUI text box, well below any
   * difference that matters, and the gearing, wheel and speed mismatches this check exists for are
   * all tens of percent.
   */
  private static final double GUI_CONFIG_RELATIVE_TOLERANCE = 0.005;

  /** How far a GUI module position may sit from the robot's before it is reported, meters. */
  private static final double GUI_MODULE_POSITION_TOLERANCE_METERS = 0.005;

  /**
   * Whether {@code settings.json} has already been compared this process. The file cannot change
   * while the code runs, and every comparison registers a fault, so it is done once.
   */
  private static boolean guiSettingsChecked = false;

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
    checkGuiSettings();
    // TODO: Call configureAutoBuilder() and registerNamedCommands(), in that order. Building any
    // PathPlannerAuto needs the named commands already registered.
  }

  /**
   * Compares PathPlanner's {@code settings.json} with the robot's own config, once per process.
   *
   * <p>The GUI draws and previews paths with {@code settings.json}; the robot follows them with
   * {@link Drive#PP_CONFIG}. A difference between the two makes the preview lie about what the
   * robot will do, so it is reported by name rather than tolerated.
   *
   * <p><b>Outputs:</b> registers a warning with the fault reporter, and logs {@code
   * Auto/GuiConfigMismatches}, when the file disagrees with the robot or cannot be read.
   */
  private static void checkGuiSettings() {
    // IMPLEMENTATION (was TODO: "Try PP_CONFIG = RobotConfig.fromGUISettings(), which reads
    // src/main/deploy/pathplanner/settings.json ... On failure, fall back to a hand-built
    // RobotConfig from Constants.Drivetrain and Drive.getModuleTranslations(), AND raise a
    // FaultReporter ERROR alert").
    //
    // Inverted on purpose (M13 step 1). The TODO made settings.json the config and a hand-built
    // one the fallback. But Drive.PP_CONFIG already exists, built from TunerConstants -- the only
    // authority on this drivetrain, regenerated by Tuner X whenever a ratio or offset is confirmed
    // -- and the setpoint generator was already using it. Loading the GUI file as well would put a
    // second RobotConfig beside the first, which is exactly the failure the plan warns about. And
    // when M13 began, the shipped settings.json disagreed with TunerConstants on the wheel radius,
    // the gearing, the free speed, the current limit and the module positions, so it would have
    // been the WRONG one. The file is now read only to be checked.
    //
    // A warning, not the error the TODO asked for: gate term 8 refuses to score on any active
    // error, and a GUI preview being out of date is no reason to stop the robot shooting.
    if (guiSettingsChecked) {
      return;
    }
    guiSettingsChecked = true;

    List<String> mismatches = guiSettingsMismatches(RobotConfig::fromGUISettings);
    Logger.recordOutput("Auto/GuiConfigMismatches", mismatches.toArray(new String[0]));
    if (!mismatches.isEmpty()) {
      FaultReporter.getInstance()
          .register(
              "PathPlanner settings.json disagrees with the robot: "
                  + String.join(", ", mismatches),
              AlertType.kWarning,
              () -> true);
    }
  }

  /**
   * Reads the GUI settings through a loader and compares them with {@link Drive#PP_CONFIG}.
   *
   * <p>Added for M13 step 1, with the loader as a parameter so a test can hand it a file that
   * cannot be parsed.
   *
   * @param loader reads {@code settings.json}; in production {@code RobotConfig::fromGUISettings}
   * @return the mismatches, or a single entry saying the file could not be read; never throws
   */
  static List<String> guiSettingsMismatches(Callable<RobotConfig> loader) {
    try {
      return describeConfigMismatches(Drive.PP_CONFIG, loader.call());
    } catch (Exception e) {
      // Broader than the IOException and ParseException that fromGUISettings declares, and it has
      // to be. The settings.json this project shipped with when M13 began is in an older format
      // with no per-module keys, and PathplannerLib 2026 fails on it with a NullPointerException
      // from inside its parser. Catching only the declared two would crash robot init on it.
      return List.of("settings.json unreadable (" + e.getClass().getSimpleName() + ")");
    }
  }

  /**
   * Names every value on which PathPlanner's GUI settings disagree with the robot's config.
   *
   * <p>Added for M13 step 1. Package-private so the comparison can be tested on its own, since the
   * file it is normally handed is whatever happens to be in the deploy folder.
   *
   * @param robot the config the robot actually follows paths with
   * @param gui the config read from {@code settings.json}
   * @return one entry per disagreeing value, named by its {@code settings.json} key and showing
   *     both numbers; empty when the two agree
   */
  static List<String> describeConfigMismatches(RobotConfig robot, RobotConfig gui) {
    List<String> mismatches = new ArrayList<>();
    compare(mismatches, "robotMass", robot.massKG, gui.massKG);
    compare(mismatches, "robotMOI", robot.MOI, gui.MOI);
    compare(
        mismatches,
        "driveWheelRadius",
        robot.moduleConfig.wheelRadiusMeters,
        gui.moduleConfig.wheelRadiusMeters);
    compare(
        mismatches,
        "maxDriveSpeed",
        robot.moduleConfig.maxDriveVelocityMPS,
        gui.moduleConfig.maxDriveVelocityMPS);
    compare(mismatches, "wheelCOF", robot.moduleConfig.wheelCOF, gui.moduleConfig.wheelCOF);
    compare(
        mismatches,
        "driveCurrentLimit",
        robot.moduleConfig.driveCurrentLimit,
        gui.moduleConfig.driveCurrentLimit);
    // The gearing is folded into the drive motor model by DCMotor.withReduction, which divides
    // the free speed by the ratio. So a free-speed difference at the wheel is a gearing difference,
    // or a different motor type, and the entry says both.
    compare(
        mismatches,
        "driveGearing/driveMotorType",
        robot.moduleConfig.driveMotor.freeSpeedRadPerSec,
        gui.moduleConfig.driveMotor.freeSpeedRadPerSec);
    if (robot.moduleLocations.length != gui.moduleLocations.length) {
      mismatches.add("module count " + gui.moduleLocations.length);
    } else {
      for (int i = 0; i < robot.moduleLocations.length; i++) {
        if (robot.moduleLocations[i].getDistance(gui.moduleLocations[i])
            > GUI_MODULE_POSITION_TOLERANCE_METERS) {
          mismatches.add(
              MODULE_KEYS[i]
                  + " "
                  + formatTranslation(gui.moduleLocations[i])
                  + " (robot "
                  + formatTranslation(robot.moduleLocations[i])
                  + ")");
        }
      }
    }
    return mismatches;
  }

  /**
   * The {@code settings.json} key prefix for each module, in PathPlanner's FL, FR, BL, BR order.
   */
  private static final String[] MODULE_KEYS = {"flModule", "frModule", "blModule", "brModule"};

  /**
   * Appends a mismatch entry when two values differ by more than the relative tolerance.
   *
   * @param mismatches the list to append to
   * @param key the {@code settings.json} key the value lives under
   * @param robotValue the robot's value
   * @param guiValue the GUI's value
   *     <p><b>Outputs:</b> may append one entry to {@code mismatches}.
   */
  private static void compare(
      List<String> mismatches, String key, double robotValue, double guiValue) {
    double scale = Math.max(Math.abs(robotValue), Math.abs(guiValue));
    if (Math.abs(robotValue - guiValue) > GUI_CONFIG_RELATIVE_TOLERANCE * scale) {
      mismatches.add(String.format("%s %.4g (robot %.4g)", key, guiValue, robotValue));
    }
  }

  /**
   * Formats a module position for a mismatch entry.
   *
   * @param translation the position, meters
   * @return the position as "(x, y)" to the millimetre
   */
  private static String formatTranslation(Translation2d translation) {
    return String.format("(%.3f, %.3f)", translation.getX(), translation.getY());
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
    //     Drive.PP_CONFIG, AllianceFlipUtil::shouldFlip, drive);
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
