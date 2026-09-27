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
 *
 * <p>M14 step 1 separated two things the first version fused. A fault's {@link AlertType} is how
 * LOUD it is on the dashboard; its {@link Impact} is whether it stops the robot scoring, which is
 * what readiness-gate term 8 reads through {@link #anyCritical()}. Every disconnected device is a
 * red error, but only a device whose failure makes a shot unsafe -- a dead flywheel follower, a
 * turret that does not know where it points -- closes the gate. The plan had wanted both: term 8's
 * row blocks scoring on "a dead flywheel follower or a disconnected turret encoder", while its
 * degradation table lets the robot keep shooting with one Limelight down.
 *
 * <p>It also holds ONE entry per name. Registering a name that is already registered replaces the
 * old entry and closes its alert. On the robot each device registers once and nothing changes;
 * under test, where hundreds of robots are built in one process, some on dead hardware on purpose,
 * it is what stops one case's dead turret from blocking scoring in every case after it.
 */
public class FaultReporter {
  /** Whether an active fault stops the robot scoring. */
  public enum Impact {
    /** Readiness-gate term 8 refuses to score while this fault is active. */
    BLOCKS_SCORING,
    /** Shown on the dashboard and logged, but the robot keeps scoring. */
    REPORT_ONLY
  }

  /**
   * One registered health check.
   *
   * @param name human-readable fault name, shown on the dashboard
   * @param isFaulted returns true while the fault condition holds
   * @param alert the backing WPILib alert
   * @param type how loudly it is shown
   * @param impact whether it blocks scoring, which is what {@link #anyCritical()} reads
   * @param activeSince single-element mutable box holding the FPGA timestamp the fault began, or
   *     NaN while healthy
   */
  private record Fault(
      String name,
      BooleanSupplier isFaulted,
      Alert alert,
      AlertType type,
      Impact impact,
      double[] activeSince) {}

  private static FaultReporter instance = null;
  private final List<Fault> faults = new ArrayList<>();

  /**
   * Builds an empty reporter.
   *
   * <p>Package-private for tests, added in M14 step 1: the robot uses {@link #getInstance()}, but a
   * test of the registry's own rules needs one that nothing else has registered with, or it can
   * neither assert that nothing blocks nor avoid leaving faults behind for other suites.
   */
  FaultReporter() {}

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
   * <p>Kept from before M14 with its meaning: an error blocks scoring and anything quieter does
   * not. Use {@link #register(String, AlertType, Impact, BooleanSupplier)} to say otherwise.
   *
   * @param name human-readable fault name, for example {@code "Turret motor disconnected"}
   * @param type severity; a {@code kError} fault also blocks scoring
   * @param isFaulted returns true while the fault condition holds
   *     <p><b>Outputs:</b> registers the fault, replacing any fault of the same name.
   */
  public void register(String name, AlertType type, BooleanSupplier isFaulted) {
    register(
        name,
        type,
        type == AlertType.kError ? Impact.BLOCKS_SCORING : Impact.REPORT_ONLY,
        isFaulted);
  }

  /**
   * Registers a health check with an explicit effect on scoring.
   *
   * <p>Added for M14 step 1.
   *
   * @param name human-readable fault name; registering a name again replaces the old entry
   * @param type how loudly it is shown
   * @param impact whether it blocks scoring while active
   * @param isFaulted returns true while the fault condition holds
   *     <p><b>Outputs:</b> closes the alert of any fault of the same name and registers this one.
   */
  public void register(String name, AlertType type, Impact impact, BooleanSupplier isFaulted) {
    // IMPLEMENTATION (was TODO: "Create a new Alert(name, type) and append a Fault record with
    // a fresh new double[] {Double.NaN} as the activeSince box.")
    //
    // The supplier is stored and polled, never sampled here. A fault registered with a boolean
    // would report whatever was true at construction time for the rest of the match.
    //
    // M14: replace, never duplicate. See the class comment for why.
    faults.removeIf(
        fault -> {
          if (fault.name().equals(name)) {
            fault.alert().close();
            return true;
          }
          return false;
        });
    faults.add(
        new Fault(name, isFaulted, new Alert(name, type), type, impact, new double[] {Double.NaN}));
  }

  /**
   * Convenience wrapper for the most common check, a device connection flag.
   *
   * @param deviceName device name; {@code " disconnected"} is appended to form the alert text
   * @param connected returns true while the device is connected
   *     <p><b>Outputs:</b> registers a {@code kError} fault, blocking scoring, that fires when
   *     {@code connected} is false.
   */
  public void registerDeviceDisconnect(String deviceName, BooleanSupplier connected) {
    registerDeviceDisconnect(deviceName, Impact.BLOCKS_SCORING, connected);
  }

  /**
   * The name a CAN device is registered under: what it is, its ID, and its bus.
   *
   * <p>Added for M14 step 2. The M14 checkpoint is "unplug any single CAN device and the robot
   * tells you which one, by name". A description alone does not find the plug; the ID and bus do.
   *
   * @param description what the device is, for example {@code "Turret motor"}
   * @param canId its CAN ID
   * @param bus the CAN bus it is on
   * @return for example {@code "Turret motor (CAN 24, rio)"}
   */
  public static String canDevice(String description, int canId, String bus) {
    return description + " (CAN " + canId + ", " + bus + ")";
  }

  /**
   * Registers a device's connection flag, as an error, with an explicit effect on scoring.
   *
   * <p>Added for M14 step 1. Every device disconnect is an ERROR on the dashboard -- red, because
   * something is unplugged -- but only the ones whose loss makes a shot unsafe block scoring.
   *
   * @param deviceName device name, ideally with its CAN ID and bus; {@code " disconnected"} is
   *     appended
   * @param impact whether its loss blocks scoring
   * @param connected returns true while the device is connected
   *     <p><b>Outputs:</b> registers the fault, replacing any of the same name.
   */
  public void registerDeviceDisconnect(
      String deviceName, Impact impact, BooleanSupplier connected) {
    // IMPLEMENTATION (was TODO: "register(deviceName + \" disconnected\", AlertType.kError,
    // () -> !connected.getAsBoolean()).")
    //
    // Note the inversion: callers hand over a "connected" signal because that is what an IO layer
    // publishes, while a Fault holds a "faulted" predicate. Doing the negation here, once, is what
    // stops every caller from having to remember which polarity this class wants -- and a polarity
    // mistake is silent, because an alert that never fires looks exactly like healthy hardware.
    register(
        deviceName + " disconnected", AlertType.kError, impact, () -> !connected.getAsBoolean());
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
        anyCriticalActive |= fault.impact() == Impact.BLOCKS_SCORING;
      }
    }
    Logger.recordOutput("Faults/AnyCritical", anyCriticalActive);
    Logger.recordOutput("Faults/Blocking", getBlockingFaultNames().toArray(new String[0]));
    Logger.recordOutput("Faults/Count", activeCount);
  }

  /**
   * Reports whether any fault that blocks scoring is currently active.
   *
   * <p>The superstructure uses this to refuse to enter scoring states when a shooter motor is dead,
   * rather than spinning up against a mechanism that cannot respond.
   *
   * <p>Since M14 step 1 this reads each fault's {@link Impact}, not its alert type. A fault
   * registered through {@link #register(String, AlertType, BooleanSupplier)} as an error still
   * blocks, so the meaning is unchanged for every caller that existed before.
   *
   * @return true if at least one registered fault that blocks scoring is active right now
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
        .anyMatch(
            fault -> fault.impact() == Impact.BLOCKS_SCORING && fault.isFaulted().getAsBoolean());
  }

  /**
   * Lists the currently-active faults, for display as a single dashboard string.
   *
   * @return names of all active faults; empty when everything is healthy
   */
  public List<String> getActiveFaultNames() {
    return activeNames(false);
  }

  /**
   * Lists the active faults that block scoring: the answer to "why won't it shoot".
   *
   * <p>Added for M14 step 1, and logged as {@code Faults/Blocking} every loop.
   *
   * @return names of the active faults whose impact is {@link Impact#BLOCKS_SCORING}
   */
  public List<String> getBlockingFaultNames() {
    return activeNames(true);
  }

  /**
   * The active faults' names.
   *
   * @param blockingOnly true to list only faults that block scoring
   * @return the names, in registration order
   */
  private List<String> activeNames(boolean blockingOnly) {
    // IMPLEMENTATION (was TODO: "Filter the registry on isFaulted and map to name.")
    return faults.stream()
        .filter(fault -> !blockingOnly || fault.impact() == Impact.BLOCKS_SCORING)
        .filter(fault -> fault.isFaulted().getAsBoolean())
        .map(Fault::name)
        .toList();
  }
}
