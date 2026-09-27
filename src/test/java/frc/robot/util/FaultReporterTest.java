package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.util.FaultReporter.Impact;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for {@link FaultReporter}: how loud a fault is versus whether it blocks scoring, and one
 * entry per name.
 *
 * <p>Added for M14 step 1. The reporter is a process-wide singleton that every suite registers
 * with, so each case uses names of its own and asserts only about those, never about the registry
 * as a whole.
 */
public class FaultReporterTest {
  private final FaultReporter reporter = FaultReporter.getInstance();

  /** Initialises the HAL, which the dashboard alerts need. */
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

  /**
   * How many times a name appears among the active faults.
   *
   * @param name the fault name
   * @return its count among the active faults
   */
  private long activeCount(String name) {
    return reporter.getActiveFaultNames().stream().filter(name::equals).count();
  }

  /**
   * Registering a name again replaces the old entry rather than adding a second.
   *
   * <p>On the robot a device registers once. Under test hundreds of robots are built in one
   * process, and without replacement a fault from a dead-hardware case would stay active for every
   * case after it.
   */
  @Test
  void registeringANameAgainReplacesIt() {
    String name = "FaultReporterTest replaced";
    reporter.register(name, AlertType.kWarning, () -> true);
    reporter.register(name, AlertType.kWarning, () -> true);
    assertEquals(1, activeCount(name), "the second registration was added, not swapped in");
    reporter.register(name, AlertType.kWarning, () -> false);
    assertEquals(0, activeCount(name), "the old, active entry survived its replacement");
  }

  /** The pre-M14 form keeps its meaning: an error blocks scoring and a warning does not. */
  @Test
  void anErrorBlocksAndAWarningDoesNot() {
    reporter.register("FaultReporterTest error", AlertType.kError, () -> true);
    reporter.register("FaultReporterTest warning", AlertType.kWarning, () -> true);
    assertTrue(reporter.getBlockingFaultNames().contains("FaultReporterTest error"));
    assertFalse(reporter.getBlockingFaultNames().contains("FaultReporterTest warning"));
    assertTrue(reporter.anyCritical());
    reporter.register("FaultReporterTest error", AlertType.kError, () -> false);
  }

  /**
   * An error that is report-only is shown, but does not block scoring.
   *
   * <p>The point of step 1: a disconnected intake roller is as red on the dashboard as a dead
   * flywheel, but only one of them makes a shot unsafe.
   */
  @Test
  void aReportOnlyErrorIsShownButDoesNotBlock() {
    String name = "FaultReporterTest report-only error";
    reporter.register(name, AlertType.kError, Impact.REPORT_ONLY, () -> true);
    assertEquals(1, activeCount(name), "a report-only fault must still be shown");
    assertFalse(reporter.getBlockingFaultNames().contains(name), "a report-only fault blocked");
  }

  /** A device disconnect blocks by default, and names the device. */
  @Test
  void aDeviceDisconnectIsNamedAndBlocksByDefault() {
    boolean[] connected = {true};
    reporter.registerDeviceDisconnect("FaultReporterTest motor (CAN 99, rio)", () -> connected[0]);
    String alert = "FaultReporterTest motor (CAN 99, rio) disconnected";
    assertEquals(0, activeCount(alert), "a connected device was reported");
    connected[0] = false;
    assertEquals(1, activeCount(alert));
    assertTrue(reporter.getBlockingFaultNames().contains(alert));
    connected[0] = true;
  }

  /** A device disconnect can be registered as report-only. */
  @Test
  void aDeviceDisconnectCanBeReportOnly() {
    reporter.registerDeviceDisconnect(
        "FaultReporterTest roller (CAN 98, rio)", Impact.REPORT_ONLY, () -> false);
    String alert = "FaultReporterTest roller (CAN 98, rio) disconnected";
    assertEquals(1, activeCount(alert));
    assertFalse(reporter.getBlockingFaultNames().contains(alert));
  }
}
