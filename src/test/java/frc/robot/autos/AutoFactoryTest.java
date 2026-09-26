package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
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
  /** Initialises the HAL, as every suite in this project does. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
  }

  /** Releases the HAL. */
  @AfterEach
  void tearDown() {
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
}
