package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FaultReporter;
import frc.robot.util.LocalADStarAK;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
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
  private final Vision vision;
  private final PathSource paths;

  /**
   * Where paths and autos are read from.
   *
   * <p>Added for M13 step 3, so the routines can be tested against paths built in the test rather
   * than against whatever files happen to be in the deploy folder. The robot uses {@link #DEPLOY}.
   */
  public interface PathSource {
    /**
     * Reads one path.
     *
     * @param name the path file's name, without extension
     * @return the path, drawn for the blue alliance
     * @throws Exception if it is missing or unreadable
     */
    PathPlannerPath path(String name) throws Exception;

    /**
     * Reads the paths an auto file follows, in order.
     *
     * @param autoName the auto file's name, without extension
     * @return its paths, drawn for the blue alliance
     * @throws Exception if it is missing or unreadable
     */
    List<PathPlannerPath> autoPaths(String autoName) throws Exception;
  }

  /** The deploy folder, {@code src/main/deploy/pathplanner}: what the robot reads. */
  public static final PathSource DEPLOY =
      new PathSource() {
        @Override
        public PathPlannerPath path(String name) throws Exception {
          return PathPlannerPath.fromPathFile(name);
        }

        @Override
        public List<PathPlannerPath> autoPaths(String autoName) throws Exception {
          return PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        }
      };

  /**
   * Every file reported missing this process, so each is reported once however often it is asked
   * for. The fault reporter is process-wide, so this is too.
   */
  private static final Set<String> reportedMissing = new HashSet<>();

  /**
   * Builds the factory and performs all PathPlanner configuration.
   *
   * <p>Construct this exactly once, from {@code RobotContainer}, after {@code
   * RobotState.initialize()}.
   *
   * <p>It no longer takes the turret (M13 step 5). Its only turret users were a marker and a
   * routine helper that commanded the turret directly, and the superstructure overwrites every
   * turret request on the next loop, so both did nothing; both now go through the superstructure.
   *
   * @param drive the drivetrain
   * @param superstructure the cross-mechanism state machine
   * @param intake the pivoting intake
   * @param shooter the flywheel, whose shot count is the only piece count the robot has
   * @param vision the vision subsystem
   */
  public AutoFactory(
      Drive drive, Superstructure superstructure, Intake intake, Shooter shooter, Vision vision) {
    this(drive, superstructure, intake, shooter, vision, DEPLOY);
  }

  /**
   * Builds the factory over a given source of paths.
   *
   * <p>Added for M13 step 3. The robot reads the deploy folder; the tests hand in paths they built.
   *
   * @param drive the drivetrain
   * @param superstructure the cross-mechanism state machine
   * @param intake the pivoting intake
   * @param shooter the flywheel, whose shot count is the only piece count the robot has
   * @param vision the vision subsystem
   * @param paths where to read paths and autos from
   */
  public AutoFactory(
      Drive drive,
      Superstructure superstructure,
      Intake intake,
      Shooter shooter,
      Vision vision,
      PathSource paths) {
    this.paths = paths;
    this.drive = drive;
    this.superstructure = superstructure;
    this.intake = intake;
    this.shooter = shooter;
    this.vision = vision;
    checkGuiSettings();
    configureAutoBuilder();
    // Last, because building any PathPlannerAuto needs the named commands already registered.
    registerNamedCommands();
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
    // IMPLEMENTATION (was TODO: "AutoBuilder.configure(RobotState pose, RobotState.resetPose,
    // RobotState robot-relative speeds, drive.runVelocity(speeds, feedforwards),
    // PPHolonomicDriveController(5, 0, 0 / 5, 0, 0), Drive.PP_CONFIG, AllianceFlipUtil::shouldFlip,
    // drive). Then Pathfinding.setPathfinder(new LocalADStarAK()) and the two PathPlannerLogging
    // callbacks.")
    //
    // Pose, reset and speeds all go through RobotState, never Drive: RobotState holds the only,
    // vision-fused estimate, so a path drawn to a scoring position ends there and not wherever
    // odometry has drifted to. The reset also clears RobotState's QuestNav seeding, so the headset
    // re-seeds in the new frame instead of pulling the estimate back to the old one.
    //
    // The output is an explicit two-argument lambda, not drive::runVelocity, because Drive has both
    // runVelocity overloads and the method reference is ambiguous between configure's Consumer and
    // BiConsumer forms. The two-argument one is wanted: it carries PathPlanner's per-module
    // feedforwards (logged; applied once the drive motors run torque-current, see Drive).
    //
    // The flip callback is shouldFlipPaths(), not AllianceFlipUtil::shouldFlip -- see there.
    AutoBuilder.configure(
        () -> RobotState.getInstance().getEstimatedPose(),
        pose ->
            RobotState.getInstance()
                .resetPose(pose, drive.getRawGyroRotation(), drive.getModulePositions()),
        () -> RobotState.getInstance().getRobotRelativeSpeeds(),
        (speeds, feedforwards) -> drive.runVelocity(speeds, feedforwards),
        new PPHolonomicDriveController(
            new PIDConstants(Constants.Auto.kTranslationKp, 0.0, Constants.Auto.kTranslationKd),
            new PIDConstants(Constants.Auto.kRotationKp, 0.0, Constants.Auto.kRotationKd)),
        Drive.PP_CONFIG,
        AutoFactory::shouldFlipPaths,
        drive);

    // The pathfinder is global to PathPlanner and needs no reference to this factory, so it is
    // installed once per process. LocalADStarAK is AdvantageKit's replay-safe wrapper: without it
    // AD* plans on a background thread whose timing differs on replay, and the replayed robot
    // drives a different route from the one in the log.
    if (!pathfinderInstalled) {
      Pathfinding.setPathfinder(new LocalADStarAK());
      pathfinderInstalled = true;
    }
    PathPlannerLogging.setLogActivePathCallback(
        path -> Logger.recordOutput("Odometry/Trajectory", path.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        target -> Logger.recordOutput("Odometry/TrajectorySetpoint", target));
  }

  /** Whether the AD* pathfinder has been installed in this process. */
  private static boolean pathfinderInstalled = false;

  /**
   * Whether PathPlanner should flip paths to the red alliance, with its flip made to match ours.
   *
   * <p>Added for M13 step 2, and it is the plan's risk #4. PathPlanner does not flip with {@code
   * AllianceFlipUtil}. It flips with its own {@link FlippingUtil}, whose field size defaults to
   * 16.54 by 8.07 m -- a field from an earlier season -- while the goal the robot aims at is
   * flipped over {@code Constants.Field}. Left alone, a red auto drives to positions computed on
   * one field and aims at a goal placed on another, and it works perfectly on blue. So before
   * PathPlanner reads the answer, its flip is set to rotational symmetry (REBUILT's, and the only
   * case {@code AllianceFlipUtil} implements) over {@code Constants.Field}'s size.
   *
   * <p>The sync happens here, on every read, because this is the moment PathPlanner is about to
   * flip: a path command asks when it starts. The field size is a tunable, so a value set once at
   * construction could go stale.
   *
   * @return true on the red alliance, exactly when {@link AllianceFlipUtil#shouldFlip()} is
   *     <p><b>Outputs:</b> sets {@link FlippingUtil}'s symmetry and field size.
   */
  static boolean shouldFlipPaths() {
    FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;
    FlippingUtil.fieldSizeX = Constants.Field.kFieldLengthMeters.get();
    FlippingUtil.fieldSizeY = Constants.Field.kFieldWidthMeters.get();
    return AllianceFlipUtil.shouldFlip();
  }

  /**
   * Registers the marker vocabulary that {@code .auto} files may use.
   *
   * <p><b>Outputs:</b> populates PathPlanner's global NamedCommands registry.
   */
  private void registerNamedCommands() {
    // IMPLEMENTATION (was TODO: "Register each name below EXACTLY as spelled ... The table in plan
    // section 8.2 is the contract; change both in the same commit.")
    //
    // Registered from one map, markerVocabulary(), so the names the robot registers and the names
    // AutoFactoryTest checks every deploy file against are the same list by construction.
    markerVocabulary().forEach(NamedCommands::registerCommand);
  }

  /**
   * The marker names a PathPlanner file may use, each with a fresh command.
   *
   * <p>Added for M13 step 4. The names are typed by hand into the GUI, and PathPlanner silently
   * skips any it does not know, so this map is a contract with every path and auto file. Plan
   * section 8.2 is the table of it; change both in the same commit.
   *
   * @return marker name to command, in the order of the section 8.2 table
   */
  public Map<String, Command> markerVocabulary() {
    Map<String, Command> markers = new LinkedHashMap<>();
    // The intake is the superstructure's second, independent request, so a path can intake and
    // score at once. StopIntake stops the roller and leaves the intake DOWN, so the hopper keeps
    // its pieces; it must not request DEFAULT, which would cancel any scoring in progress.
    markers.put("Intake", Commands.runOnce(() -> superstructure.setIntakeRequested(true)));
    markers.put("StopIntake", Commands.runOnce(() -> superstructure.setIntakeRequested(false)));
    // Rarely: raising the intake can dump the hopper.
    markers.put(
        "StowIntake", superstructure.setWantedStateCommand(Superstructure.WantedState.STOW));
    markers.put("SpinUp", superstructure.setWantedStateCommand(Superstructure.WantedState.SPIN_UP));
    markers.put("ScoreOnMove", superstructure.scoreUntilEmptyCommand());
    markers.put("ScoreStationary", scoreStationaryCommand());
    markers.put("Eject", superstructure.setWantedStateCommand(Superstructure.WantedState.EJECT));
    markers.put("Home", superstructure.setWantedStateCommand(Superstructure.WantedState.HOME));
    // Through the superstructure, not turret.setWantedStateCommand(STOW) as the plan's table had
    // it. The superstructure sets the turret's wanted state in every handler, every loop, so a
    // marker that commanded the turret directly was overwritten on the very next loop and did
    // nothing at all -- and during a scoring leg, stowing the turret is not what anyone wants
    // anyway. DEFAULT rests the shooter side, and resting stows the turret.
    markers.put(
        "StowTurret", superstructure.setWantedStateCommand(Superstructure.WantedState.DEFAULT));
    // GamePiecePipeline and TagPipeline are NOT registered, though the plan's table listed them.
    // Since M10b the superstructure switches the intake camera itself, on the edge into and out of
    // intaking, so a marker could only disagree with it: GamePiecePipeline outside an intake leg
    // would leave the camera off AprilTags, contributing nothing, with nothing to switch it back.
    // A file that still uses either name fails AutoFactoryTest's deploy scan.
    return markers;
  }

  /**
   * Builds a routine from a {@code .auto} file drawn in the PathPlanner GUI.
   *
   * @param autoName the auto file's name, without extension
   * @return the routine, or a do-nothing command if the file is missing
   */
  public Command buildAuto(String autoName) {
    // IMPLEMENTATION (was TODO: "return new PathPlannerAuto(autoName), wrapped in try/catch. On
    // failure return Commands.none() AND fire a FaultReporter alert, so a renamed file cannot crash
    // robotInit at an event.")
    //
    // The try/catch the TODO asked for is not enough on its own, and a test found out why: in
    // 2026.1.2 the PathPlannerAuto constructor catches FileNotFoundException, IOException and
    // ParseException ITSELF and quietly builds an empty auto. So the file is read first through
    // getPathGroupFromAutoFile (the path source's autoPaths), which does throw, and the auto is
    // built only once that has succeeded. This is the route for an auto drawn entirely in the GUI;
    // the routines in AutoRoutines are built from paths instead.
    try {
      paths.autoPaths(autoName);
      return new PathPlannerAuto(autoName);
    } catch (Exception e) {
      reportMissing("auto", autoName, e);
      return Commands.none();
    }
  }

  /**
   * Builds a command that follows one path file.
   *
   * @param pathName the path file's name, without extension
   * @return the follow command, or a do-nothing command if the file is missing
   */
  public Command followPath(String pathName) {
    // IMPLEMENTATION (was TODO: "PathPlannerPath.fromPathFile(pathName) inside try/catch for
    // IOException and org.json.simple.parser.ParseException, then AutoBuilder.followPath(path).
    // Return Commands.none() plus an alert on failure.")
    //
    // The file is read when the routine is BUILT, which is when the chooser is populated at boot,
    // so a missing path is on the dashboard in the queue line rather than discovered mid-auto. The
    // alliance flip is not decided here: AutoBuilder asks shouldFlipPaths() when the command
    // starts, by which time the driver station has reported the alliance.
    //
    // Catches Exception rather than the two the TODO named: fromPathFile also throws
    // FileVersionException in 2026.1.2, and a malformed file fails however its parser happens to.
    Optional<PathPlannerPath> path = loadPath(pathName);
    return path.isPresent() ? AutoBuilder.followPath(path.get()) : Commands.none();
  }

  /**
   * Reads a path through the path source, reporting it if it cannot be read.
   *
   * @param pathName the path file's name, without extension
   * @return the path, drawn blue, or empty if it could not be read
   */
  private Optional<PathPlannerPath> loadPath(String pathName) {
    try {
      return Optional.of(paths.path(pathName));
    } catch (Exception e) {
      reportMissing("path", pathName, e);
      return Optional.empty();
    }
  }

  /**
   * Registers a warning that a path or auto file could not be read, once per file per process.
   *
   * <p>A WARNING rather than the error the skeleton's TODOs imply: readiness-gate term 8 refuses to
   * score while any error is active, and a missing auto file must not stop the robot shooting in
   * teleop. It is still on the dashboard, by name, from boot.
   *
   * @param kind "path" or "auto"
   * @param name the file's name
   * @param cause why it could not be read
   *     <p><b>Outputs:</b> may register a fault and print a driver station warning.
   */
  private static void reportMissing(String kind, String name, Exception cause) {
    String message =
        "PathPlanner "
            + kind
            + " unreadable: "
            + name
            + " ("
            + cause.getClass().getSimpleName()
            + ")";
    if (reportedMissing.add(message)) {
      FaultReporter.getInstance().register(message, AlertType.kWarning, () -> true);
      DriverStation.reportWarning(message, false);
    }
  }

  /**
   * Scores from a standstill until the hopper is probably empty, or the timeout.
   *
   * @return the scoring command
   */
  public Command scoreStationaryCommand() {
    // IMPLEMENTATION (was TODO: "superstructure.setWantedStateCommand(SCORE_STATIONARY)
    // .andThen(Commands.waitUntil(superstructure::likelyEmpty))
    // .withTimeout(Constants.Auto.kScoreTimeoutSec.get())
    // .andThen(superstructure.setWantedStateCommand(DEFAULT)); Read the timeout inside
    // Commands.defer").
    //
    // The request is HELD for the wait and handed back as DEFAULT when the wait ends, rather than
    // set once and followed by a DEFAULT in sequence. The sequence leaks the request if anything
    // cancels it partway -- a routine's own timeout around it, for one -- and the robot then goes
    // on
    // requesting a stationary shot through the next leg of the auto, which is one it is driving.
    return Commands.deadline(
        Commands.defer(
            () ->
                Commands.waitUntil(superstructure::likelyEmpty)
                    .withTimeout(Constants.Auto.kScoreTimeoutSec.get()),
            Set.of()),
        holdWantedState(Superstructure.WantedState.SCORE_STATIONARY));
  }

  /**
   * Holds a shooter-side request for as long as the command runs, then hands back {@code DEFAULT}.
   *
   * <p>Added for M13 step 5. The superstructure's own {@code setWantedStateCommand} is a one-shot,
   * which is right for a button but wrong for an auto leg: a request set once outlives the leg it
   * was set for and carries into the next one. Every auto building block that asks the shooter side
   * for something asks through this, so the request lasts exactly as long as the leg, however the
   * leg ends -- finished, timed out or cancelled.
   *
   * @param wanted the request to hold
   * @return a command that never ends by itself; run it beside something that does
   */
  public Command holdWantedState(Superstructure.WantedState wanted) {
    return Commands.startEnd(
        () -> superstructure.setWantedState(wanted),
        () -> superstructure.setWantedState(Superstructure.WantedState.DEFAULT));
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
    // IMPLEMENTATION (was TODO: "Commands.runOnce(() -> { load the path, take
    // AllianceFlipUtil.shouldFlip() ? p.flipPath().getStartingHolonomicPose().orElseThrow() :
    // p.getStartingHolonomicPose().orElseThrow(); then RobotState.getInstance().resetPose(start,
    // drive.getRawGyroRotation(), drive.getModulePositions()); })").
    //
    // Verified against 2026.1.2: getStartingHolonomicPose() returns Optional<Pose2d>, empty for a
    // path with no ideal starting state. orElseThrow() inside a command would crash the auto, so
    // an empty start skips the reset and the robot keeps the estimate it already has.
    //
    // The flip is AllianceFlipUtil.apply on the blue start, not p.flipPath(). Step 2 made the two
    // agree, and applying ours keeps this pose in the same frame as the goal by construction.
    //
    // The path is read when the routine is built, but the alliance is read when the reset RUNS:
    // routines are built at boot, before the driver station has said which alliance this is.
    Optional<Pose2d> blueStart =
        loadPath(pathName).flatMap(PathPlannerPath::getStartingHolonomicPose);
    return Commands.runOnce(
        () ->
            blueStart.ifPresent(
                start ->
                    RobotState.getInstance()
                        .resetPose(
                            AllianceFlipUtil.apply(start),
                            drive.getRawGyroRotation(),
                            drive.getModulePositions())));
  }

  /**
   * The expected starting pose of a named auto, for the pre-match preview.
   *
   * @param autoName the auto file's name, without extension
   * @return the alliance-flipped starting pose, or empty if it cannot be determined
   */
  public Optional<Pose2d> getStartingPose(String autoName) {
    // IMPLEMENTATION (was TODO: "PathPlannerAuto.getStartingPoseFromAutoFile(autoName),
    // alliance-flipped, wrapped in Optional and try/catch. TODO: VERIFY that static helper's exact
    // name in PathplannerLib 2026.1.2.")
    //
    // Verified, and it does not exist in 2026.1.2. The nearest static helper is
    // getPathGroupFromAutoFile, and an auto starts where its first path does, so that is used.
    try {
      List<PathPlannerPath> group = paths.autoPaths(autoName);
      return group.isEmpty()
          ? Optional.empty()
          : group.get(0).getStartingHolonomicPose().map(AllianceFlipUtil::apply);
    } catch (Exception e) {
      reportMissing("auto", autoName, e);
      return Optional.empty();
    }
  }

  /**
   * The alliance-flipped starting pose of a path, for the pre-match preview.
   *
   * <p>Added for M13 step 3. A routine built in {@code AutoRoutines} starts where its first path
   * does, so this is its starting pose with no companion {@code .auto} file needed.
   *
   * @param pathName the path file's name, without extension
   * @return where the path starts on the current alliance, or empty if it cannot be read
   */
  public Optional<Pose2d> getPathStartingPose(String pathName) {
    return loadPath(pathName)
        .flatMap(PathPlannerPath::getStartingHolonomicPose)
        .map(AllianceFlipUtil::apply);
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
}
