package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * A central registry of WPILib {@link Alert} objects, so device health is discoverable in one place
 * rather than scattered across seven subsystems.
 *
 * <p>Subsystems register a named check with a {@link BooleanSupplier} ("is this Talon connected").
 * This class polls every check once per loop, drives the backing {@link Alert}, tracks how long
 * each fault has been active, and publishes a single summary boolean that a driver dashboard can
 * render as one red light.
 *
 * <p>The concrete competitive value: a CANcoder that browns out between matches shows up on the
 * dashboard while the robot is still in the queue line, rather than at the first shot.
 */
public class FaultReporter {
  /**
   * One registered health check.
   *
   * @param name human-readable fault name, shown on the dashboard
   * @param isFaulted returns true while the fault condition holds
   * @param alert the backing WPILib alert
   * @param type severity, used by {@link #anyCritical()}
   * @param activeSince single-element mutable box holding the FPGA timestamp the fault began, or
   *     NaN while healthy
   */
  private record Fault(
      String name, BooleanSupplier isFaulted, Alert alert, AlertType type, double[] activeSince) {}

  private static FaultReporter instance = null;
  private final List<Fault> faults = new ArrayList<>();

  /**
   * Returns the process-wide reporter, creating it on first use.
   *
   * @return the singleton instance
   */
  public static FaultReporter getInstance() {
    // TODO: Lazy singleton -- if (instance == null) instance = new FaultReporter(); return
    // instance.
    return instance;
  }

  /**
   * Registers a health check to be polled every loop.
   *
   * <p>Call this from a subsystem constructor, once per device.
   *
   * @param name human-readable fault name, for example {@code "Turret motor disconnected"}
   * @param type severity; {@code kError} faults participate in {@link #anyCritical()}
   * @param isFaulted returns true while the fault condition holds
   *     <p><b>Outputs:</b> appends a new {@link Fault} to the registry.
   */
  public void register(String name, AlertType type, BooleanSupplier isFaulted) {
    // TODO: Create a new Alert(name, type) and append a Fault record with a fresh
    // new double[] {Double.NaN} as the activeSince box.
  }

  /**
   * Convenience wrapper for the most common check, a device connection flag.
   *
   * @param deviceName device name; {@code " disconnected"} is appended to form the alert text
   * @param connected returns true while the device is connected
   *     <p><b>Outputs:</b> registers a {@code kError} fault that fires when {@code connected} is
   *     false.
   */
  public void registerDeviceDisconnect(String deviceName, BooleanSupplier connected) {
    // TODO: register(deviceName + " disconnected", AlertType.kError,
    // () -> !connected.getAsBoolean()).
  }

  /**
   * Polls every registered check and updates alerts and logs. Call once per robot loop, after
   * subsystem inputs have been refreshed.
   *
   * <p><b>Outputs:</b> drives every {@link Alert}; updates each fault's active-since timestamp;
   * logs {@code Faults/<name>}, {@code Faults/AnyCritical} and {@code Faults/Count}.
   */
  public void periodic() {
    // TODO: For each fault: read isFaulted, call alert.set(active), set activeSince[0] to
    // Timer.getFPGATimestamp() on a rising edge and Double.NaN on a falling edge, then
    // Logger.recordOutput("Faults/" + name, active). Finally log "Faults/AnyCritical" and
    // "Faults/Count".
  }

  /**
   * Reports whether any error-severity fault is currently active.
   *
   * <p>The superstructure uses this to refuse to enter scoring states when a shooter motor is dead,
   * rather than spinning up against a mechanism that cannot respond.
   *
   * @return true if at least one registered {@code kError} fault is active right now
   */
  public boolean anyCritical() {
    // TODO: Stream the registry for type == AlertType.kError && isFaulted.getAsBoolean().
    return false;
  }

  /**
   * Lists the currently-active faults, for display as a single dashboard string.
   *
   * @return names of all active faults; empty when everything is healthy
   */
  public List<String> getActiveFaultNames() {
    // TODO: Filter the registry on isFaulted and map to name.
    return List.of();
  }
}
