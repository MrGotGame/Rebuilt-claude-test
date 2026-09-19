package frc.robot.util;

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A constant whose value can be edited live from a dashboard while the robot is enabled, then
 * frozen to its compiled default for competition.
 *
 * <p>In {@link Constants#kTuningMode} this publishes a {@link LoggedNetworkNumber} under {@code
 * /Tuning/<key>}. AdvantageScope's tuning tab writes there, and because the value flows through the
 * AdvantageKit logger, a log replay reproduces whatever number you typed mid-match. Outside tuning
 * mode it returns the compiled default and generates zero NetworkTables traffic.
 *
 * <p>{@link #hasChanged(int)} and the static {@code ifChanged} helpers exist so a subsystem can
 * reapply Phoenix configuration only when a gain actually moved. Reapplying every loop floods the
 * CAN bus and is a common cause of mystery latency.
 *
 * <p>Adapted from the tunable-constant pattern used by FRC teams 6328 and 2910; the AdvantageKit
 * template does not ship this class.
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Creates a tunable number with no default value yet. {@link #get()} returns 0.0 until {@link
   * #initDefault(double)} is called.
   *
   * @param dashboardKey key this number appears under, relative to {@code /Tuning} (for example
   *     {@code "Turret/kP"})
   */
  public LoggedTunableNumber(String dashboardKey) {
    // IMPLEMENTATION (was TODO: "this.key = tableKey + "/" + dashboardKey. Do not touch
    // NetworkTables here.")
    //
    // Only the key string is built here. Deliberately no NetworkTables work: Constants.java
    // constructs well over a hundred of these as static final fields, and that happens during class
    // initialization -- potentially before the HAL exists, and always before we know whether tuning
    // mode is on. Publishing here would make merely *loading* Constants cost a hundred NT entries.
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Creates a tunable number and immediately latches its default value.
   *
   * @param dashboardKey key this number appears under, relative to {@code /Tuning}
   * @param defaultValue value returned outside tuning mode, and the value the dashboard entry
   *     starts at inside tuning mode
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    // IMPLEMENTATION (was TODO: "this(dashboardKey) then initDefault(defaultValue).")
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Latches the default value and, when tuning is enabled, creates the backing dashboard entry.
   * Calling this more than once has no effect after the first call.
   *
   * @param defaultValue value returned outside tuning mode
   *     <p><b>Outputs:</b> sets {@code hasDefault} and {@code defaultValue}; may construct {@code
   *     dashboardNumber}.
   */
  public void initDefault(double defaultValue) {
    // IMPLEMENTATION (was TODO: "If !hasDefault, latch the default and, when Constants.kTuningMode
    // && !Constants.disableHal, construct new LoggedNetworkNumber(key, defaultValue).")
    //
    // The hasDefault guard makes this idempotent, which is what lets LoggedTunableGains call
    // initDefault fluently from withGravity() without clobbering a value already set.
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;

      // The dashboard entry is created only in tuning mode. The disableHal check is what lets unit
      // tests construct Constants without a HAL: LoggedNetworkNumber reaches NetworkTables, which
      // needs the HAL initialized, and a JUnit run has none.
      if (Constants.kTuningMode && !Constants.disableHal) {
        dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
      }
    }
  }

  /**
   * Reads the current value of this constant.
   *
   * @return the dashboard value when tuning mode is active and the HAL is available, otherwise the
   *     compiled default; 0.0 if no default has been set yet
   */
  public double get() {
    // IMPLEMENTATION (was TODO: "Return 0.0 if no default. Otherwise return dashboardNumber.get()
    // in tuning mode with HAL available, else defaultValue.")
    //
    // The null check on dashboardNumber is belt-and-braces: kTuningMode is final so it cannot flip
    // at runtime, but disableHal is not, and a test that toggles it after construction would
    // otherwise dereference null here.
    if (!hasDefault) {
      return 0.0;
    }
    if (Constants.kTuningMode && !Constants.disableHal && dashboardNumber != null) {
      return dashboardNumber.get();
    }
    return defaultValue;
  }

  /**
   * Reports whether this value has changed since the last time the same caller asked.
   *
   * <p>Each caller passes its own id so that two subsystems polling the same tunable do not consume
   * each other's change events.
   *
   * @param id caller identity; pass {@code hashCode()} of the owning object
   * @return true if the value differs from the value seen by this id on its previous call, and true
   *     on the very first call for a given id
   */
  public boolean hasChanged(int id) {
    // IMPLEMENTATION (was TODO: "Compare get() against lastHasChangedValues.get(id); store and
    // return true on change.")
    //
    // Returning true on the FIRST call for an id is intentional and load-bearing: it makes a
    // subsystem push its compiled-in gains to the motor controller on the very first loop, so there
    // is exactly one code path for "apply gains" rather than a separate startup path that can drift
    // out of sync with the retune path.
    double current = get();
    Double last = lastHasChangedValues.get(id);
    if (last == null || last != current) {
      lastHasChangedValues.put(id, current);
      return true;
    }
    return false;
  }

  /**
   * {@link DoubleSupplier} implementation so a tunable can be passed anywhere a supplier is wanted.
   *
   * @return the same value as {@link #get()}
   */
  @Override
  public double getAsDouble() {
    // IMPLEMENTATION (was TODO: "return get().")
    return get();
  }

  /**
   * Runs an action only when at least one of the supplied tunables has changed for this caller,
   * handing the action the current values.
   *
   * @param id caller identity; pass {@code hashCode()} of the owning object
   * @param action consumer invoked with the current values, in the same order as {@code
   *     tunableNumbers}
   * @param tunableNumbers the tunables to watch
   *     <p><b>Outputs:</b> invokes {@code action} at most once per call.
   */
  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    // IMPLEMENTATION (was TODO: "If any tunable hasChanged(id), call action with the current values
    // as a double[].")
    //
    // Note the loop does NOT short-circuit. hasChanged has a side effect -- it records the value it
    // saw -- so bailing out early would leave the remaining tunables still holding their old
    // recorded value, and they would each fire a spurious change on some later loop. Every tunable
    // must be polled every call, and only then is the result acted on.
    boolean anyChanged = false;
    for (LoggedTunableNumber tunable : tunableNumbers) {
      anyChanged |= tunable.hasChanged(id);
    }

    if (anyChanged) {
      double[] values = new double[tunableNumbers.length];
      for (int i = 0; i < tunableNumbers.length; i++) {
        values[i] = tunableNumbers[i].get();
      }
      action.accept(values);
    }
  }

  /**
   * Runs an action only when at least one of the supplied tunables has changed for this caller,
   * ignoring the values themselves.
   *
   * @param id caller identity; pass {@code hashCode()} of the owning object
   * @param action runnable invoked when any watched tunable changed
   * @param tunableNumbers the tunables to watch
   *     <p><b>Outputs:</b> invokes {@code action} at most once per call.
   */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    // IMPLEMENTATION (was TODO: "Delegate to the Consumer overload, ignoring the values.")
    ifChanged(id, values -> action.run(), tunableNumbers);
  }
}
