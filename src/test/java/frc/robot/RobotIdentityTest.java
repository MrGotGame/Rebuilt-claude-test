package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Tests for the competition/practice identity of plan section 5.5.
 *
 * <p>Added for M14 step 5. The test JVM is never a real roboRIO, so these exercise the pure
 * resolver {@link Constants#identify} and {@link Constants#forRobot(RobotType, double, double)},
 * which {@link Constants#getRobot()} and the two offset readers are built on.
 */
public class RobotIdentityTest {
  private static final String COMP = "COMP-1234";
  private static final String PRACTICE = "PRAC-5678";

  /** The serials passed to the unknown-robot callback. */
  private final List<String> unknown = new ArrayList<>();

  /**
   * Resolves a serial on a real robot.
   *
   * @param serial the roboRIO serial
   * @return the robot it resolves to
   */
  private RobotType onRealRobot(String serial) {
    return Constants.identify(Mode.REAL, serial, COMP, PRACTICE, unknown::add);
  }

  /** Each known serial resolves to its robot, and raises nothing. */
  @Test
  void eachKnownSerialIsItsRobot() {
    assertEquals(RobotType.COMP, onRealRobot(COMP));
    assertEquals(RobotType.PRACTICE, onRealRobot(PRACTICE));
    assertEquals(List.of(), unknown, "a known robot was reported unknown");
  }

  /**
   * An unknown serial runs the competition robot's constants, and says so.
   *
   * <p>Section 5.5: loud, never silent. The classic failure is the practice robot's offsets on the
   * competition robot on a Friday morning, and it produces no error at all.
   */
  @Test
  void anUnknownSerialIsCompAndLoud() {
    assertEquals(RobotType.COMP, onRealRobot("SPARE-0001"));
    assertEquals(List.of("SPARE-0001"), unknown);
  }

  /** Simulation and replay are the simulated robot, whatever the serial, and raise nothing. */
  @Test
  void simulationIsTheSimbot() {
    assertEquals(
        RobotType.SIMBOT, Constants.identify(Mode.SIM, COMP, COMP, PRACTICE, unknown::add));
    assertEquals(
        RobotType.SIMBOT, Constants.identify(Mode.REPLAY, "", COMP, PRACTICE, unknown::add));
    assertEquals(List.of(), unknown);
  }

  /** The practice robot reads its own value; every other robot reads the competition value. */
  @Test
  void onlyThePracticeRobotReadsThePracticeValue() {
    assertEquals(2.0, Constants.forRobot(RobotType.PRACTICE, 1.0, 2.0));
    assertEquals(1.0, Constants.forRobot(RobotType.COMP, 1.0, 2.0));
    assertEquals(1.0, Constants.forRobot(RobotType.SIMBOT, 1.0, 2.0));
  }
}
