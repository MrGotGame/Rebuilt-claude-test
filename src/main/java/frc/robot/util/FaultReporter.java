package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

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
    // IMPLEMENTATION (was TODO: "Lazy singleton -- if (instance == null) instance = new
    // FaultReporter(); return instance.")
    //
    // Implemented as part of M10 rather than M14. Until now every subsystem worked around this
    // method returning null by raising plain WPILib Alerts instead of registering here, and each
    // one of them says so in a comment pointing at "once that class is implemented". M10 is the
    // milestone that forced the issue: term 8 of the readiness gate is
    // !FaultReporter.getInstance().anyCritical(), so leaving the singleton null would have made
    // the superstructure throw on its first loop. The per-device REGISTRATIONS those comments
    // promise are still M14's; this is only the class they will register with.
    if (instance == null) {
      instance = new FaultReporter();
    }
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
    // IMPLEMENTATION (was TODO: "Create a new Alert(name, type) and append a Fault record with
    // a fresh new double[] {Double.NaN} as the activeSince box.")
    //
    // The supplier is stored and polled, never sampled here. A fault registered with a boolean
    // would report whatever was true at construction time for the rest of the match.
    faults.add(new Fault(name, isFaulted, new Alert(name, type), type, new double[] {Double.NaN}));
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
    // IMPLEMENTATION (was TODO: "register(deviceName + \" disconnected\", AlertType.kError,
    // () -> !connected.getAsBoolean()).")
    //
    // Note the inversion: callers hand over a "connected" signal because that is what an IO layer
    // publishes, while a Fault holds a "faulted" predicate. Doing the negation here, once, is what
    // stops every caller from having to remember which polarity this class wants -- and a polarity
    // mistake is silent, because an alert that never fires looks exactly like healthy hardware.
    register(deviceName + " disconnected", AlertType.kError, () -> !connected.getAsBoolean());
  }

  /**
   * Polls every registered check and updates alerts and logs. Call once per robot loop, after
   * subsystem inputs have been refreshed.
   *
   * <p><b>Outputs:</b> drives every {@link Alert}; updates each fault's active-since timestamp;
   * logs {@code Faults/<name>}, {@code Faults/AnyCritical} and {@code Faults/Count}.
   */
  public void periodic() {
    // IMPLEMENTATION (was TODO: "For each fault: read isFaulted, call alert.set(active), set
    // activeSince[0] to Timer.getFPGATimestamp() on a rising edge and Double.NaN on a falling
    // edge, then Logger.recordOutput(\"Faults/\" + name, active). Finally log
    // \"Faults/AnyCritical\" and \"Faults/Count\".")
    //
    // Each supplier is read exactly once per loop and the result reused for the alert, the log and
    // the counters, so the dashboard and the log cannot disagree about a fault that flickered
    // between two reads.
    int activeCount = 0;
    boolean anyCriticalActive = false;
    for (Fault fault : faults) {
      boolean active = fault.isFaulted().getAsBoolean();
      fault.alert().set(active);

      // The edge handling is what makes activeSince a fault DURATION rather than a fault time.
      // Writing the timestamp every loop while the fault holds would report "started just now" for
      // a fault that has been live all match, which is the opposite of what a pit crew triaging an
      // intermittent connection needs to know.
      if (active && Double.isNaN(fault.activeSince()[0])) {
        fault.activeSince()[0] = Timer.getFPGATimestamp();
      } else if (!active) {
        fault.activeSince()[0] = Double.NaN;
      }

      Logger.recordOutput("Faults/" + fault.name(), active);
      if (active) {
        activeCount++;
        anyCriticalActive |= fault.type() == AlertType.kError;
      }
    }
    Logger.recordOutput("Faults/AnyCritical", anyCriticalActive);
    Logger.recordOutput("Faults/Count", activeCount);
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
    // IMPLEMENTATION (was TODO: "Stream the registry for type == AlertType.kError &&
    // isFaulted.getAsBoolean().")
    //
    // The suppliers are polled live rather than read from whatever periodic() last cached,
    // deliberately. This is a safety gate -- the superstructure refuses to enter a scoring state
    // while it is true -- and a gate answering with last loop's data is a gate that lets one more
    // shot through after the flywheel follower dies. It costs a handful of boolean reads.
    //
    // NOTE for M14: this returns false on the robot today because nothing has registered yet. Each
    // subsystem raises its device alerts as plain WPILib Alerts and says at the field declaration
    // that it will move to registerDeviceDisconnect once this class exists. It now does.
    return faults.stream()
        .anyMatch(fault -> fault.type() == AlertType.kError && fault.isFaulted().getAsBoolean());
  }

  /**
   * Lists the currently-active faults, for display as a single dashboard string.
   *
   * @return names of all active faults; empty when everything is healthy
   */
  public List<String> getActiveFaultNames() {
    // IMPLEMENTATION (was TODO: "Filter the registry on isFaulted and map to name.")
    return faults.stream()
        .filter(fault -> fault.isFaulted().getAsBoolean())
        .map(Fault::name)
        .toList();
  }
}
