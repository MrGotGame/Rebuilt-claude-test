package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FaultReporter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for {@link AutoFactory}: the robot's PathPlanner configuration and wiring.
 *
 * <p>Added for M13. Step 1 is the comparison of PathPlanner's {@code settings.json} against the
 * config the robot actually follows paths with. The comparison is tested on hand-built configs
 * rather than on the shipped file, because the shipped file is whatever the GUI last saved.
 */
public class AutoFactoryTest {
  /** The robot a case built, or null for the cases that need none. */
  private SimulatedAutoRobot robot;

  /** Initialises the HAL and pauses the clock, as every suite in this project does. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();
  }

  /** Stops whatever a case built and releases the HAL. */
  @AfterEach
  void tearDown() {
    if (robot != null) {
      robot.close();
    }
    DriverStationSim.resetData();
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  // ------------------------------------------------------ step 1: the GUI settings check ----

  /** Mass, kg, of the reference config. */
  private static final double MASS = 60.0;

  /** Moment of inertia, kg*m^2, of the reference config. */
  private static final double MOI = 6.0;

  /** Wheel radius, meters. */
  private static final double RADIUS = 0.05;

  /** Free speed at the wheel, m/s. */
  private static final double SPEED = 4.5;

  /** Wheel coefficient of friction. */
  private static final double COF = 1.2;

  /** Drive current limit, amps. */
  private static final double CURRENT = 80.0;

  /** Drive gear ratio. */
  private static final double GEARING = 6.75;

  /** Module offset from the robot centre in each axis, meters. */
  private static final double HALF_BASE = 0.3;

  /**
   * Builds a config, varying whichever values a case needs to.
   *
   * @param mass robot mass, kg
   * @param moi moment of inertia, kg*m^2
   * @param radius wheel radius, meters
   * @param speed free speed at the wheel, m/s
   * @param cof wheel coefficient of friction
   * @param current drive current limit, amps
   * @param gearing drive gear ratio
   * @param backLeftY the back-left module's y offset, meters; {@link #HALF_BASE} when square
   * @return a four-module config
   */
  private static RobotConfig config(
      double mass,
      double moi,
      double radius,
      double speed,
      double cof,
      double current,
      double gearing,
      double backLeftY) {
    return new RobotConfig(
        mass,
        moi,
        new ModuleConfig(
            radius, speed, cof, DCMotor.getKrakenX60Foc(1).withReduction(gearing), current, 1),
        new Translation2d(HALF_BASE, HALF_BASE),
        new Translation2d(HALF_BASE, -HALF_BASE),
        new Translation2d(-HALF_BASE, backLeftY),
        new Translation2d(-HALF_BASE, -HALF_BASE));
  }

  /**
   * The reference config every case compares against.
   *
   * @return a fresh copy of it
   */
  private static RobotConfig reference() {
    return config(MASS, MOI, RADIUS, SPEED, COF, CURRENT, GEARING, HALF_BASE);
  }

  /**
   * Asserts a comparison found exactly one mismatch, under the expected key.
   *
   * @param gui the config standing in for {@code settings.json}
   * @param key the key the single entry must name
   */
  private static void assertOnlyMismatch(RobotConfig gui, String key) {
    List<String> mismatches = AutoFactory.describeConfigMismatches(reference(), gui);
    assertEquals(1, mismatches.size(), "expected only " + key + ", got " + mismatches);
    assertTrue(mismatches.get(0).startsWith(key), "expected " + key + ", got " + mismatches);
  }

  /** Two identical configs agree, so a correct settings.json raises nothing. */
  @Test
  void identicalConfigsAgree() {
    assertEquals(List.of(), AutoFactory.describeConfigMismatches(reference(), reference()));
  }

  /**
   * Rounding in the GUI's text boxes is not a mismatch.
   *
   * <p>Every value is off by 0.1 percent. A comparison with no tolerance would fire the warning on
   * every robot, and a warning that is always on is one nobody reads.
   */
  @Test
  void roundingIsNotAMismatch() {
    double r = 1.001;
    RobotConfig rounded =
        config(
            MASS * r,
            MOI * r,
            RADIUS * r,
            SPEED * r,
            COF * r,
            CURRENT * r,
            GEARING * r,
            HALF_BASE + 0.001);
    assertEquals(List.of(), AutoFactory.describeConfigMismatches(reference(), rounded));
  }

  /** A wrong mass is named. */
  @Test
  void aWrongMassIsNamed() {
    assertOnlyMismatch(
        config(MASS + 5, MOI, RADIUS, SPEED, COF, CURRENT, GEARING, HALF_BASE), "robotMass");
  }

  /** A wrong moment of inertia is named. */
  @Test
  void aWrongMoiIsNamed() {
    assertOnlyMismatch(
        config(MASS, MOI + 1, RADIUS, SPEED, COF, CURRENT, GEARING, HALF_BASE), "robotMOI");
  }

  /**
   * A wrong wheel radius is named.
   *
   * <p>The shipped settings.json had 0.048 m against TunerConstants' 2.167 in (0.055 m) when M13
   * began, a 13 percent difference.
   */
  @Test
  void aWrongWheelRadiusIsNamed() {
    assertOnlyMismatch(
        config(MASS, MOI, RADIUS * 0.87, SPEED, COF, CURRENT, GEARING, HALF_BASE),
        "driveWheelRadius");
  }

  /** A wrong free speed is named. */
  @Test
  void aWrongFreeSpeedIsNamed() {
    assertOnlyMismatch(
        config(MASS, MOI, RADIUS, SPEED + 1, COF, CURRENT, GEARING, HALF_BASE), "maxDriveSpeed");
  }

  /** A wrong wheel friction is named. */
  @Test
  void aWrongWheelFrictionIsNamed() {
    assertOnlyMismatch(
        config(MASS, MOI, RADIUS, SPEED, COF - 0.3, CURRENT, GEARING, HALF_BASE), "wheelCOF");
  }

  /** A wrong current limit is named. */
  @Test
  void aWrongCurrentLimitIsNamed() {
    assertOnlyMismatch(
        config(MASS, MOI, RADIUS, SPEED, COF, CURRENT / 2, GEARING, HALF_BASE),
        "driveCurrentLimit");
  }

  /**
   * A wrong gear ratio is named, although the config holds it only inside the motor model.
   *
   * <p>The shipped settings.json had 5.143 against TunerConstants' 7.36 when M13 began.
   */
  @Test
  void aWrongGearingIsNamed() {
    assertOnlyMismatch(
        config(MASS, MOI, RADIUS, SPEED, COF, CURRENT, 5.143, HALF_BASE),
        "driveGearing/driveMotorType");
  }

  /**
   * A settings.json the parser chokes on is reported, not thrown.
   *
   * <p>The file this project shipped with when M13 began is in an older format, and PathplannerLib
   * 2026 fails on it with a NullPointerException rather than the IOException or ParseException it
   * declares. The check runs in the factory's constructor, so an exception escaping it would stop
   * the robot code from starting.
   */
  @Test
  void anUnparseableSettingsFileIsReportedNotThrown() {
    List<String> mismatches =
        AutoFactory.guiSettingsMismatches(
            () -> {
              throw new NullPointerException("no flModuleX");
            });
    assertEquals(List.of("settings.json unreadable (NullPointerException)"), mismatches);
  }

  /** A readable settings file goes through the same comparison. */
  @Test
  void aReadableSettingsFileIsCompared() {
    assertEquals(List.of(), AutoFactory.guiSettingsMismatches(() -> Drive.PP_CONFIG));
  }

  /** A module in the wrong place is named by which module it is. */
  @Test
  void aMisplacedModuleIsNamed() {
    assertOnlyMismatch(
        config(MASS, MOI, RADIUS, SPEED, COF, CURRENT, GEARING, HALF_BASE + 0.02), "blModule");
  }

  // ------------------------------------------------ step 2: AutoBuilder and the one flip ----

  /**
   * Sets the driver station's alliance.
   *
   * @param red true for red, false for blue
   *     <p><b>Outputs:</b> updates the simulated driver station.
   */
  private static void setAlliance(boolean red) {
    DriverStationSim.setAllianceStationId(red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  /**
   * On red, PathPlanner flips a path onto exactly the pose {@code AllianceFlipUtil} would.
   *
   * <p>The plan's risk #4. PathPlanner's flip is set up the way it ships -- mirrored, on a 16.54 by
   * 8.07 m field -- and the case first confirms that flip DISAGREES with ours, so it cannot pass
   * merely because the two happen to agree. Then it asks the flip callback, which is what {@code
   * AutoBuilder} does as a path starts, and the two must agree in position and heading. The start
   * heading is 30 degrees because a mirrored flip and a rotational one agree on a heading of zero.
   */
  @Test
  void aRedPathStartsWhereAllianceFlipUtilPutsIt() {
    setAlliance(true);
    Pose2d blueStart = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(30.0));
    PathPlannerPath path =
        SimulatedAutoRobot.straightPath(
            "Flip", blueStart, new Pose2d(4.0, 1.5, blueStart.getRotation()), 2.0, 0.0);
    Pose2d ours = AllianceFlipUtil.apply(blueStart);

    FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kMirrored;
    FlippingUtil.fieldSizeX = 16.54;
    FlippingUtil.fieldSizeY = 8.07;
    Pose2d unsynced = path.flipPath().getStartingHolonomicPose().orElseThrow();
    assertTrue(
        unsynced.getTranslation().getDistance(ours.getTranslation()) > 1.0,
        "PathPlanner's own flip already agreed with ours, so this case would test nothing");

    assertTrue(AutoFactory.shouldFlipPaths(), "red must flip");
    Pose2d theirs = path.flipPath().getStartingHolonomicPose().orElseThrow();
    assertEquals(0.0, theirs.getTranslation().getDistance(ours.getTranslation()), 1e-9);
    assertEquals(0.0, theirs.getRotation().minus(ours.getRotation()).getRadians(), 1e-9, "heading");
  }

  /** Blue never flips. */
  @Test
  void blueDoesNotFlip() {
    setAlliance(false);
    assertFalse(AutoFactory.shouldFlipPaths());
  }

  /**
   * Runs a PathPlanner path-following command on a simulated robot until it ends.
   *
   * @param start where the robot stands, blue-origin
   * @param path the path, drawn blue
   * @param red true for the red alliance
   * @return the fastest the robot moved during the path, m/s
   */
  private double followOnRobot(Pose2d start, PathPlannerPath path, boolean red) {
    robot =
        new SimulatedAutoRobot(
            red ? AllianceFlipUtil.apply(start) : start, SimulatedAutoRobot.pathsOf(), red);
    Command follow = AutoBuilder.followPath(path);
    CommandScheduler.getInstance().schedule(follow);
    double fastest = 0.0;
    for (int i = 0; i < 500 && follow.isScheduled(); i++) {
      robot.step();
      fastest = Math.max(fastest, robot.speedMetersPerSec());
    }
    assertFalse(follow.isScheduled(), "the path never finished");
    return fastest;
  }

  /**
   * A path drives the simulated robot to where it ends, as measured by {@code RobotState}.
   *
   * <p>This proves the wiring rather than the controller: the pose PathPlanner reads, the speeds it
   * reads, and the drivetrain it commands all have to be the right ones for the robot to arrive.
   * The fastest-speed check stops it passing for a robot that was simply placed at the end.
   */
  @Test
  void aPathDrivesTheRobotToItsEnd() {
    Pose2d start = new Pose2d(-4.0, 0.0, Rotation2d.kZero);
    Pose2d end = new Pose2d(-2.0, 0.0, Rotation2d.kZero);
    double fastest =
        followOnRobot(start, SimulatedAutoRobot.straightPath("Blue", start, end, 2.0, 0.0), false);
    assertTrue(fastest > 1.0, "the robot never really moved; fastest " + fastest + " m/s");
    assertEquals(0.0, robot.pose().getTranslation().getDistance(end.getTranslation()), 0.10);
  }

  /**
   * On red, the same blue-drawn path drives the robot to the flipped end.
   *
   * <p>Section 8.4's first drawing rule is to draw everything blue and let the flip handle red.
   * This is that rule working end to end: the robot starts on the flipped start and must finish on
   * {@code AllianceFlipUtil}'s flip of the blue end, which is where the goal-relative code will
   * also think it is.
   */
  @Test
  void onRedTheSamePathEndsOnTheFlippedEnd() {
    Pose2d start = new Pose2d(-4.0, 0.0, Rotation2d.kZero);
    Pose2d end = new Pose2d(-2.0, 0.0, Rotation2d.kZero);
    followOnRobot(start, SimulatedAutoRobot.straightPath("Red", start, end, 2.0, 0.0), true);
    assertEquals(
        0.0,
        robot.pose().getTranslation().getDistance(AllianceFlipUtil.apply(end.getTranslation())),
        0.10);
  }

  // --------------------------------------------------- step 3: loading paths, loudly ----

  /** Where the step 3 cases stand, blue-origin. */
  private static final Pose2d START = new Pose2d(-4.0, 1.0, Rotation2d.fromDegrees(30.0));

  /** A path from {@link #START} to two metres further along x. */
  private static PathPlannerPath knownPath() {
    return SimulatedAutoRobot.straightPath(
        "Known", START, new Pose2d(-2.0, 1.0, START.getRotation()), 2.0, 0.0);
  }

  /**
   * Builds a robot at {@link #START} whose path source holds only {@link #knownPath()}.
   *
   * @param red true for the red alliance
   *     <p><b>Outputs:</b> sets {@link #robot}.
   */
  private void buildRobotWithKnownPath(boolean red) {
    robot =
        new SimulatedAutoRobot(
            red ? AllianceFlipUtil.apply(START) : START,
            SimulatedAutoRobot.pathsOf(knownPath()),
            red);
  }

  /**
   * Whether a fault with exactly this name is active.
   *
   * @param name the fault's name
   * @return true if the fault reporter lists it as active
   */
  private static boolean faultActive(String name) {
    return FaultReporter.getInstance().getActiveFaultNames().contains(name);
  }

  /**
   * A missing path gives a command that does nothing, and a warning that names it.
   *
   * <p>Section 8.2's warning is that PathPlanner's failures are silent. A missing path must neither
   * crash the robot code at boot, which is when the chooser builds every routine, nor disappear
   * without a word.
   */
  @Test
  void aMissingPathDoesNothingAndIsNamed() {
    buildRobotWithKnownPath(false);
    Command follow = robot.factory.followPath("Step3_NoSuchPath");
    CommandScheduler.getInstance().schedule(follow);
    robot.step(2);
    assertFalse(follow.isScheduled(), "a missing path must end at once");
    assertEquals(0.0, robot.pose().getTranslation().getDistance(START.getTranslation()), 0.01);
    assertTrue(
        faultActive("PathPlanner path unreadable: Step3_NoSuchPath (FileNotFoundException)"),
        "no warning named the missing path; active: "
            + FaultReporter.getInstance().getActiveFaultNames());
  }

  /**
   * The missing-file warning is a warning, not an error.
   *
   * <p>Readiness-gate term 8 refuses to score while any error is active, so an error here would
   * stop the robot shooting in teleop because a path file was renamed.
   */
  @Test
  void aMissingPathDoesNotBlockScoring() {
    buildRobotWithKnownPath(false);
    robot.factory.followPath("Step3_NotAnError");
    assertTrue(
        faultActive("PathPlanner path unreadable: Step3_NotAnError (FileNotFoundException)"));
    assertFalse(FaultReporter.getInstance().anyCritical(), "a missing path raised an error");
  }

  /** A path from the source is followed to its end. */
  @Test
  void aKnownPathIsFollowed() {
    buildRobotWithKnownPath(false);
    Command follow = robot.factory.followPath("Known");
    CommandScheduler.getInstance().schedule(follow);
    robot.step(300);
    assertFalse(follow.isScheduled(), "the path never finished");
    assertEquals(0.0, robot.pose().getTranslation().getDistance(new Translation2d(-2.0, 1.0)), 0.1);
  }

  /** The pose reset puts the estimate on the path's start, heading included. */
  @Test
  void theResetAdoptsThePathStart() {
    buildRobotWithKnownPath(false);
    robot.placeAt(new Pose2d(1.0, -2.0, Rotation2d.kPi));
    Command reset = robot.factory.resetPoseToPathStart("Known");
    CommandScheduler.getInstance().schedule(reset);
    robot.step(1);
    assertEquals(0.0, robot.pose().getTranslation().getDistance(START.getTranslation()), 1e-3);
    assertEquals(
        START.getRotation().getDegrees(), robot.pose().getRotation().getDegrees(), 0.5, "heading");
  }

  /**
   * The reset reads the alliance when it RUNS, not when the routine is built.
   *
   * <p>Routines are built when the chooser is populated at boot, before the driver station has said
   * which alliance this is -- on the field it reads blue, the default, at that moment. A reset that
   * captured the flip then would put every red robot on the blue start.
   */
  @Test
  void theResetReadsTheAllianceWhenItRuns() {
    buildRobotWithKnownPath(false);
    Command reset = robot.factory.resetPoseToPathStart("Known");
    setAlliance(true);
    CommandScheduler.getInstance().schedule(reset);
    robot.step(1);
    Pose2d red = AllianceFlipUtil.apply(START);
    assertEquals(0.0, robot.pose().getTranslation().getDistance(red.getTranslation()), 1e-3);
    assertEquals(red.getRotation().getDegrees(), robot.pose().getRotation().getDegrees(), 0.5);
  }

  /** A reset to a missing path leaves the estimate alone rather than crashing the auto. */
  @Test
  void aResetToAMissingPathLeavesThePoseAlone() {
    buildRobotWithKnownPath(false);
    Command reset = robot.factory.resetPoseToPathStart("Step3_NoSuchReset");
    CommandScheduler.getInstance().schedule(reset);
    robot.step(1);
    assertEquals(0.0, robot.pose().getTranslation().getDistance(START.getTranslation()), 0.01);
  }

  /** An auto starts where its first path does, flipped on red. */
  @Test
  void anAutoStartsWhereItsFirstPathDoes() {
    buildRobotWithKnownPath(false);
    assertEquals(
        0.0,
        robot
            .factory
            .getStartingPose("Known")
            .orElseThrow()
            .getTranslation()
            .getDistance(START.getTranslation()),
        1e-6);
    assertEquals(
        robot.factory.getStartingPose("Known"), robot.factory.getPathStartingPose("Known"));
    setAlliance(true);
    assertEquals(
        0.0,
        robot
            .factory
            .getStartingPose("Known")
            .orElseThrow()
            .getTranslation()
            .getDistance(AllianceFlipUtil.apply(START.getTranslation())),
        1e-6);
    assertTrue(robot.factory.getStartingPose("Step3_NoSuchAuto").isEmpty());
    assertTrue(robot.factory.getPathStartingPose("Step3_NoSuchAuto").isEmpty());
  }

  /** A missing auto file gives a do-nothing command and a warning, not a crash at boot. */
  @Test
  void aMissingAutoFileDoesNothingAndIsNamed() {
    buildRobotWithKnownPath(false);
    Command auto = robot.factory.buildAuto("Step3_NoSuchAutoFile");
    CommandScheduler.getInstance().schedule(auto);
    robot.step(2);
    assertFalse(auto.isScheduled());
    assertTrue(
        FaultReporter.getInstance().getActiveFaultNames().stream()
            .anyMatch(name -> name.startsWith("PathPlanner auto unreadable: Step3_NoSuchAutoFile")),
        "no warning named the missing auto");
  }

  // ------------------------------------------------------- step 4: the marker vocabulary ----

  /** The names plan section 8.2 promises a path file may use, as corrected by M13 step 4. */
  private static final List<String> VOCABULARY =
      List.of(
          "Intake",
          "StopIntake",
          "StowIntake",
          "SpinUp",
          "ScoreOnMove",
          "ScoreStationary",
          "Eject",
          "Home",
          "StowTurret");

  /**
   * Builds a robot 3 m in front of the goal, where it can score.
   *
   * <p><b>Outputs:</b> sets {@link #robot}.
   */
  private void buildScoringRobot() {
    robot =
        new SimulatedAutoRobot(
            new Pose2d(-3.0, 0.0, Rotation2d.kZero), SimulatedAutoRobot.pathsOf(), false);
  }

  /**
   * Runs one registered marker, the way PathPlanner runs it, then steps the robot.
   *
   * @param name the marker name
   * @param loops how many loops to step afterwards
   *     <p><b>Outputs:</b> schedules the marker's command and runs the robot.
   */
  private void runMarker(String name, int loops) {
    assertTrue(NamedCommands.hasCommand(name), name + " is not registered");
    CommandScheduler.getInstance().schedule(NamedCommands.getCommand(name));
    robot.step(loops);
  }

  /** Every name in the vocabulary is registered, and the robot's map holds nothing else. */
  @Test
  void everyMarkerIsRegistered() {
    buildScoringRobot();
    assertEquals(VOCABULARY, List.copyOf(robot.factory.markerVocabulary().keySet()));
    for (String name : VOCABULARY) {
      assertTrue(NamedCommands.hasCommand(name), name + " is not registered");
    }
  }

  /**
   * Intake collects, and StopIntake stops the roller but leaves the intake down.
   *
   * <p>StopIntake must neither raise the intake, which would dump the hopper, nor send DEFAULT,
   * which would cancel a scoring leg the same path is running.
   */
  @Test
  void intakeAndStopIntakeLeaveTheIntakeDown() {
    buildScoringRobot();
    runMarker("SpinUp", 2);
    runMarker("Intake", 5);
    assertEquals(Superstructure.IntakeMode.INTAKING, robot.superstructure.getIntakeMode());
    runMarker("StopIntake", 5);
    assertEquals(Superstructure.IntakeMode.DEPLOYED, robot.superstructure.getIntakeMode());
    assertEquals(
        Superstructure.SystemState.SPINNING_UP,
        robot.superstructure.getSystemState(),
        "StopIntake cancelled the shooter side's request");
  }

  /** StowIntake raises the intake. */
  @Test
  void stowIntakeRaisesTheIntake() {
    buildScoringRobot();
    runMarker("Intake", 5);
    runMarker("StopIntake", 1);
    runMarker("StowIntake", 5);
    assertEquals(Superstructure.IntakeMode.STOWED, robot.superstructure.getIntakeMode());
  }

  /**
   * Each shooter-side marker puts the superstructure in its state.
   *
   * <p>StowTurret included: it rests the shooter side, which stows the turret. Registered as the
   * plan had it, a direct turret request, it was overwritten by the superstructure on the next
   * loop.
   */
  @Test
  void shooterSideMarkersReachTheirStates() {
    buildScoringRobot();
    runMarker("SpinUp", 3);
    assertEquals(Superstructure.SystemState.SPINNING_UP, robot.superstructure.getSystemState());
    runMarker("ScoreOnMove", 3);
    assertEquals(
        Superstructure.SystemState.SCORING_ON_THE_MOVE, robot.superstructure.getSystemState());
    CommandScheduler.getInstance().cancelAll();
    runMarker("StowTurret", 3);
    assertEquals(Superstructure.SystemState.RESTING, robot.superstructure.getSystemState());
    assertEquals(Turret.SystemState.STOWING, robot.turret.getSystemState());
    runMarker("Eject", 3);
    assertEquals(Superstructure.SystemState.EJECTING, robot.superstructure.getSystemState());
    runMarker("Home", 3);
    assertEquals(Superstructure.SystemState.HOMING, robot.superstructure.getSystemState());
  }

  /**
   * Collects every named-command reference in a PathPlanner file's JSON.
   *
   * <p>PathPlanner nests commands -- sequential and parallel groups inside an auto, a command
   * inside each event marker of a path -- and names a registered command as {@code {"type":
   * "named", "data": {"name": ...}}}. This walks the whole tree for that shape.
   *
   * @param json a parsed JSON value
   * @param names where to add each name found
   *     <p><b>Outputs:</b> appends to {@code names}.
   */
  private static void collectNamedCommands(Object json, List<String> names) {
    if (json instanceof JSONObject object) {
      if ("named".equals(object.get("type")) && object.get("data") instanceof JSONObject data) {
        names.add(String.valueOf(data.get("name")));
      }
      for (Object value : object.values()) {
        collectNamedCommands(value, names);
      }
    } else if (json instanceof JSONArray array) {
      for (Object value : array) {
        collectNamedCommands(value, names);
      }
    }
  }

  /**
   * The scanner finds a named command however deeply PathPlanner nests it.
   *
   * <p>The deploy scan below passes trivially while no drawn file uses a marker, which is the state
   * M13 leaves the deploy folder in. This case is what shows the scan can see one when it is there.
   */
  @Test
  void theMarkerScannerFindsNestedNames() throws Exception {
    String auto =
        "{\"command\": {\"type\": \"sequential\", \"data\": {\"commands\": ["
            + "{\"type\": \"named\", \"data\": {\"name\": \"SpinUp\"}},"
            + "{\"type\": \"parallel\", \"data\": {\"commands\": ["
            + "{\"type\": \"path\", \"data\": {\"pathName\": \"Leave\"}},"
            + "{\"type\": \"named\", \"data\": {\"name\": \"TagPipeline\"}}]}}]}},"
            + "\"eventMarkers\": [{\"name\": \"m\", \"command\":"
            + " {\"type\": \"named\", \"data\": {\"name\": \"Intake\"}}}]}";
    List<String> names = new ArrayList<>();
    collectNamedCommands(new JSONParser().parse(auto), names);
    // JSONObject is a HashMap, so the order found is not the order written.
    assertEquals(Set.of("SpinUp", "TagPipeline", "Intake"), Set.copyOf(names));
    assertEquals(3, names.size());
  }

  /**
   * Every marker any drawn path or auto uses is registered.
   *
   * <p>Section 8.2: a marker PathPlanner does not know is skipped with no error and no warning, and
   * it is the most common "works in the GUI, not on the robot" failure. This case is the only guard
   * that failure can have. It reads every file under the deploy folder's {@code pathplanner}
   * directory, so it goes red the moment someone draws a marker with a typo, or one of the two
   * names M13 retired.
   */
  @Test
  void everyMarkerInTheDeployFolderIsRegistered() throws Exception {
    buildScoringRobot();
    Path root = Filesystem.getDeployDirectory().toPath().resolve("pathplanner");
    List<Path> files;
    try (Stream<Path> walk = Files.walk(root)) {
      files =
          walk.filter(f -> f.toString().endsWith(".path") || f.toString().endsWith(".auto"))
              .toList();
    }
    assertFalse(files.isEmpty(), "found no path or auto files under " + root);
    List<String> unregistered = new ArrayList<>();
    for (Path file : files) {
      List<String> names = new ArrayList<>();
      collectNamedCommands(new JSONParser().parse(Files.readString(file)), names);
      for (String name : names) {
        if (!NamedCommands.hasCommand(name)) {
          unregistered.add(root.relativize(file) + ": " + name);
        }
      }
    }
    assertEquals(List.of(), unregistered, "markers PathPlanner would silently skip");
  }
}
