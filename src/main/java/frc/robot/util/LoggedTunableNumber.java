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
    // TODO: this.key = tableKey + "/" + dashboardKey. Do not touch NetworkTables here -- the
    // dashboard entry is created lazily in initDefault so that constructing a tunable is free.
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
    // TODO: Call this(dashboardKey), then initDefault(defaultValue).
    this(dashboardKey);
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
    // TODO: If !hasDefault, latch hasDefault = true and this.defaultValue = defaultValue. Then,
    // when
    // Constants.kTuningMode && !Constants.disableHal, construct
    // dashboardNumber = new LoggedNetworkNumber(key, defaultValue).
  }

  /**
   * Reads the current value of this constant.
   *
   * @return the dashboard value when tuning mode is active and the HAL is available, otherwise the
   *     compiled default; 0.0 if no default has been set yet
   */
  public double get() {
    // TODO: Return 0.0 if !hasDefault. Otherwise return dashboardNumber.get() when
    // Constants.kTuningMode && !Constants.disableHal, else return defaultValue.
    return 0.0;
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
    // TODO: Compare get() against lastHasChangedValues.get(id). If absent or different, put the new
    // value and return true; otherwise return false.
    return false;
  }

  /**
   * {@link DoubleSupplier} implementation so a tunable can be passed anywhere a supplier is wanted.
   *
   * @return the same value as {@link #get()}
   */
  @Override
  public double getAsDouble() {
    // TODO: return get().
    return 0.0;
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
    // TODO: If any tunable hasChanged(id) -- evaluate them ALL, do not short-circuit, or the
    // unevaluated ones will fire on the next loop -- call action.accept() with the current values
    // mapped into a double[].
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
    // TODO: Delegate to the Consumer overload, discarding the values: ifChanged(id, values ->
    // action.run(), tunableNumbers).
  }
}
