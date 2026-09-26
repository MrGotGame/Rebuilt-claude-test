package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.util.FaultReporter;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for {@link AutoSelector}: the chooser, and the pre-match placement preview.
 *
 * <p>Added for M13 step 7. The M13 checkpoint asks that {@code AutoSelector/StartPoseError} read
 * under 5 cm with the robot placed correctly and that the warning fire when it is not. Selections
 * are made the way the dashboard makes them, through the chooser's NetworkTables entry.
 */
public class AutoSelectorTest {
  private SimulatedAutoRobot robot;
  private AutoSelector selector;

  /** Initialises the HAL and pauses the clock. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();
  }

  /** Stops the robot and releases the HAL. */
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

  /**
   * Builds the robot on the shooting position, over the checkpoint's synthetic field, with a
   * selector.
   *
   * @param red true for the red alliance
   *     <p><b>Outputs:</b> sets {@link #robot} and {@link #selector}.
   */
  private void build(boolean red) {
    robot = new SimulatedAutoRobot(AutoCheckpointTest.SHOOT, AutoCheckpointTest.field(), red);
    selector = new AutoSelector(robot.factory, robot.routines);
  }

  /**
   * Picks a routine on the dashboard, and lets the chooser read it.
   *
   * @param name the option's display name
   *     <p><b>Outputs:</b> writes the chooser's selected entry and refreshes the selector.
   */
  private void select(String name) {
    NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getSubTable(AutoSelector.CHOOSER_KEY)
        .getEntry("selected")
        .setString(name);
    SmartDashboard.updateValues();
    selector.refreshChooser();
  }

  /** The chooser offers the eight routines of section 8.3 after the Do Nothing default. */
  @Test
  void theChooserOffersEveryRoutine() {
    build(false);
    List<String> expected =
        List.of(
            "Do Nothing",
            "Leave",
            "Preload Only",
            "Center 3",
            "Center 5",
            "Amp 3",
            "Source 3",
            "Source Steal",
            "Scavenge");
    List<String> names = selector.getOptionNames();
    assertEquals(expected, names.subList(0, expected.size()));
    assertEquals(
        Constants.kTuningMode, names.contains("Center 3 (stationary)"), "the diagnostic's gating");
  }

  /**
   * With nothing chosen, the robot does nothing, and never gets a null routine.
   *
   * <p>A null here would make autonomousInit silently schedule nothing; Do Nothing does the same on
   * purpose, and says so in the log.
   */
  @Test
  void nothingChosenMeansDoNothing() {
    build(false);
    selector.refreshChooser();
    assertEquals(AutoSelector.DO_NOTHING, selector.getSelectedName());
    Command command = selector.get();
    assertNotNull(command);
    CommandScheduler.getInstance().schedule(command);
    robot.step(2);
    assertFalse(command.isScheduled(), "Do Nothing did not end at once");
  }

  /**
   * A dashboard selection is what get() builds, and each call builds a fresh routine.
   *
   * <p>A command may belong to only one composition, and a practice auto is often run twice without
   * a reboot, so the same instance must never be handed out twice.
   */
  @Test
  void aSelectionBuildsAFreshRoutineEachTime() {
    build(false);
    select("Center 3");
    assertEquals("Center 3", selector.getSelectedName());
    assertNotSame(selector.get(), selector.get());
  }

  /** The selected routine is the one that runs: Leave ends where Leave does. */
  @Test
  void theSelectedRoutineIsTheOneThatRuns() {
    build(false);
    select("Leave");
    Command routine = selector.get();
    robot.startAuto(routine);
    robot.runUntilFinished(routine, 1000);
    assertEquals(-5.0, robot.pose().getX(), 0.25, "the robot did not run Leave");
  }

  /**
   * On its start, the placement error is under 5 cm and the warning stays quiet.
   *
   * <p>The checkpoint's first half. Center 3 starts on the shooting position, where the robot is.
   */
  @Test
  void aCorrectlyPlacedRobotReadsUnderFiveCentimetres() {
    build(false);
    select("Center 3");
    selector.logStartingPose();
    assertTrue(
        selector.getStartPoseErrorMeters() < 0.05,
        "placed on the start, the error read " + selector.getStartPoseErrorMeters());
    assertFalse(selector.isMisplaced());
  }

  /**
   * Off its start, the error reads the offset and the warning fires.
   *
   * <p>The checkpoint's second half: 30 cm off the tape, the single most common reason an auto that
   * worked in practice fails in a match.
   */
  @Test
  void aMisplacedRobotRaisesTheWarning() {
    build(false);
    select("Center 3");
    robot.placeAt(AutoCheckpointTest.SHOOT.plus(new Transform2d(0.0, 0.30, Rotation2d.kZero)));
    selector.logStartingPose();
    assertEquals(0.30, selector.getStartPoseErrorMeters(), 0.01);
    assertTrue(selector.isMisplaced(), "30 cm off the start did not warn");
    assertTrue(
        FaultReporter.getInstance().getActiveFaultNames().stream()
            .anyMatch(name -> name.startsWith("Robot is not on the selected auto's start")),
        "the warning is not on the dashboard");
  }

  /**
   * The preview follows the selection: the same placement is right for one routine, wrong for
   * another.
   */
  @Test
  void thePreviewFollowsTheSelection() {
    build(false);
    select("Amp 3");
    selector.logStartingPose();
    assertTrue(selector.isMisplaced(), "the shooting position is 2 m from Amp 3's start");
    select("Center 3");
    selector.logStartingPose();
    assertFalse(selector.isMisplaced());
  }

  /**
   * On red, the preview is the flipped start: a robot correctly placed on the red side reads true.
   *
   * <p>Previewing the blue start on red would warn every red match, and a warning that is always on
   * is one nobody reads.
   */
  @Test
  void onRedThePreviewIsTheFlippedStart() {
    build(true);
    assertTrue(robot.pose().getX() > 0.0, "the red robot is not on the red side; the case is void");
    select("Center 3");
    selector.logStartingPose();
    assertTrue(
        selector.getStartPoseErrorMeters() < 0.05,
        "a correctly placed red robot read " + selector.getStartPoseErrorMeters());
  }

  /** A routine with no start never warns: Do Nothing does not care where it stands. */
  @Test
  void doNothingNeverWarns() {
    build(false);
    robot.placeAt(new Pose2d(5.0, 5.0, AutoCheckpointTest.SHOOT.getRotation()));
    select(AutoSelector.DO_NOTHING);
    selector.logStartingPose();
    assertTrue(Double.isNaN(selector.getStartPoseErrorMeters()));
    assertFalse(selector.isMisplaced());
  }

  /** selectionChanged reports each change once. */
  @Test
  void aChangeIsReportedOnce() {
    build(false);
    select("Leave");
    assertTrue(selector.selectionChanged());
    assertFalse(selector.selectionChanged());
    select("Scavenge");
    assertTrue(selector.selectionChanged());
  }
}
