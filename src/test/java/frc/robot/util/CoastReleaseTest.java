package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for {@link CoastRelease}: the drivetrain drops to coast once, only after the delay, and
 * only when the robot is still.
 *
 * <p>Added for M14 step 4. The clock is paused and stepped by hand, so the delay is measured in
 * simulated seconds rather than in however long the test machine takes.
 */
public class CoastReleaseTest {
  private static final double LOOP_SEC = 0.02;

  private final CoastRelease release = new CoastRelease();

  /** Initialises the HAL and pauses the clock. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();
  }

  /** Releases the HAL. */
  @AfterEach
  void tearDown() {
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  /** Loops in {@code kCoastDelaySec}. */
  private static int delayLoops() {
    return (int) Math.round(Constants.Drivetrain.kCoastDelaySec.get() / LOOP_SEC);
  }

  /**
   * Runs a disabled period at a speed, counting the loops on which the release fired.
   *
   * @param loops how many loops
   * @param speed the chassis speed, m/s
   * @return the loop on which it first fired, or -1 if it never did
   */
  private int runDisabled(int loops, double speed) {
    int firstFired = -1;
    for (int i = 0; i < loops; i++) {
      if (release.shouldReleaseNow(speed) && firstFired < 0) {
        firstFired = i;
      }
      SimHooks.stepTiming(LOOP_SEC);
    }
    return firstFired;
  }

  /**
   * A still robot is released once the delay has passed, and not before.
   *
   * <p>Not before, because a robot that coasts the instant it is disabled rolls away at the end of
   * autonomous or during a pause. Not much after, or the crew waits at the end of every match.
   */
  @Test
  void aStillRobotIsReleasedAfterTheDelay() {
    release.onDisabled();
    int fired = runDisabled(delayLoops() * 2, 0.0);
    assertEquals(delayLoops(), fired, 2, "released at the wrong time");
    assertTrue(release.isReleased());
  }

  /** It fires once per disabled period, not on every loop after the delay. */
  @Test
  void itFiresOnce() {
    release.onDisabled();
    runDisabled(delayLoops() + 5, 0.0);
    assertEquals(-1, runDisabled(50, 0.0), "the release fired again");
  }

  /**
   * A robot still rolling is not released until it stops.
   *
   * <p>Dropping to coast while it rolls means it keeps rolling, into whatever is in front of it.
   */
  @Test
  void aRollingRobotWaitsUntilItStops() {
    release.onDisabled();
    double rolling = Constants.Drivetrain.kCoastReleaseMaxSpeedMetersPerSec * 5.0;
    assertEquals(-1, runDisabled(delayLoops() * 2, rolling), "released while rolling");
    assertEquals(0, runDisabled(5, 0.0), "not released as soon as it stopped");
  }

  /** A new disabled period starts the delay and the latch afresh. */
  @Test
  void eachDisabledPeriodStartsAfresh() {
    release.onDisabled();
    runDisabled(delayLoops() + 5, 0.0);
    release.onDisabled();
    assertFalse(release.isReleased(), "the latch survived a new disabled period");
    assertEquals(delayLoops(), runDisabled(delayLoops() * 2, 0.0), 2);
  }
}
