// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Shared helpers for talking to Phoenix 6 devices.
 *
 * <p>Beyond the template's retry helper, this holds a batched signal-refresh registry. Every
 * non-swerve Talon on the RIO bus registers its status signals here, and {@link #refreshAll()}
 * refreshes them with a single {@link BaseStatusSignal#refreshAll} call per loop instead of one
 * call per subsystem. That is measurably cheaper on the bus, and it gives every subsystem the same
 * timestamp for its measurements, which matters when the shooting solver correlates turret angle
 * with pose.
 */
public class PhoenixUtil {
  private static final List<BaseStatusSignal> registeredSignals = new ArrayList<>();
  private static BaseStatusSignal[] signalArray = new BaseStatusSignal[0];

  /**
   * Fires when a Phoenix configuration never applied after every retry.
   *
   * <p>A configuration that silently failed is how a robot ends up running with default PID gains
   * and nobody knowing why it behaves oddly, so this is surfaced as a driver-station error rather
   * than only a log line.
   */
  private static final Alert configFailedAlert =
      new Alert(
          "A Phoenix device configuration failed to apply. Check CAN wiring.", AlertType.kError);

  private PhoenixUtil() {}

  /**
   * Runs a Phoenix configuration command, retrying until it reports success.
   *
   * @param maxAttempts how many times to try before giving up
   * @param command the configuration call to run; returns the Phoenix status code
   *     <p><b>Outputs:</b> applies the configuration to hardware as a side effect.
   */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    // IMPLEMENTATION (was TODO: "KEEP the existing template body (loop until error.isOK()). Add a
    // FaultReporter alert when all attempts fail, so a config that never applied is visible instead
    // of silent.")
    //
    // The template loop is kept exactly. The addition is the failure path: `lastError` survives the
    // loop so that, if every attempt failed, we can both raise the alert and name the status code.
    //
    // A WPILib Alert is used directly rather than FaultReporter, because FaultReporter is still
    // stubbed at this point in the build-out and its getInstance() returns null. Swap this for
    // FaultReporter.getInstance().register(...) once that class is implemented.
    StatusCode lastError = StatusCode.OK;
    for (int i = 0; i < maxAttempts; i++) {
      lastError = command.get();
      if (lastError.isOK()) {
        return;
      }
    }

    // Every attempt failed. Make it loud.
    configFailedAlert.set(true);
    Logger.recordOutput("Phoenix/LastConfigFailure", lastError.toString());
  }

  /**
   * Adds status signals to the batch that {@link #refreshAll()} refreshes each loop.
   *
   * <p>Call this from every IO implementation's constructor for its non-odometry signals. Odometry
   * signals are owned by {@code PhoenixOdometryThread} and must not be registered here.
   *
   * @param signals the signals to refresh as part of the shared batch
   *     <p><b>Outputs:</b> appends to the registry and rebuilds the backing array.
   */
  public static void registerSignals(BaseStatusSignal... signals) {
    // IMPLEMENTATION (was TODO: "Append to registeredSignals and rebuild signalArray.")
    //
    // The List is the source of truth and the array is a cache of it. Rebuilding the array here --
    // at construction time, a handful of times total -- is what keeps refreshAll() allocation-free
    // on the 50 Hz path. BaseStatusSignal.refreshAll takes a varargs array, so without this cache
    // every loop would allocate one.
    //
    // Synchronized because IO constructors can in principle run on a different thread from the one
    // that later calls refreshAll, and an ArrayList resize racing an array read is a real hazard.
    synchronized (registeredSignals) {
      for (BaseStatusSignal signal : signals) {
        registeredSignals.add(signal);
      }
      signalArray = registeredSignals.toArray(new BaseStatusSignal[0]);
    }
  }

  /**
   * Refreshes every registered signal in one batched call. Call once per {@code robotPeriodic},
   * before any subsystem periodic runs.
   *
   * <p><b>Outputs:</b> updates all registered signals in place; logs the batch status under {@code
   * Phoenix/RefreshStatus}.
   */
  public static void refreshAll() {
    // IMPLEMENTATION (was TODO: "BaseStatusSignal.refreshAll(signalArray) once per robotPeriodic,
    // before subsystems run. Log the returned StatusCode under "Phoenix/RefreshStatus".")
    //
    // The empty-array guard matters: refreshAll on an empty varargs array is not meaningfully
    // defined, and early in the build-out -- or in a replay run where no IO registered anything --
    // the registry legitimately is empty.
    BaseStatusSignal[] signals = signalArray;
    if (signals.length == 0) {
      return;
    }

    StatusCode status = BaseStatusSignal.refreshAll(signals);
    Logger.recordOutput("Phoenix/RefreshStatus", status.toString());
    Logger.recordOutput("Phoenix/RegisteredSignalCount", signals.length);
  }

  /**
   * Reduces bus traffic from devices whose signals are not all needed.
   *
   * @param devices the Phoenix devices to optimize
   *     <p><b>Outputs:</b> disables unrequested status frames on each device.
   */
  public static void optimizeBus(ParentDevice... devices) {
    // IMPLEMENTATION (was TODO: "ParentDevice.optimizeBusUtilizationForAll(devices) wrapped in
    // tryUntilOk. Every Talon in this project must be optimized or the RIO bus will saturate with 9
    // devices.")
    //
    // optimizeBusUtilizationForAll disables every status frame that nobody has explicitly asked
    // for. That is safe ONLY because each IO calls setUpdateFrequency on the signals it needs
    // first -- optimize runs last and turns off the remainder. Calling these in the wrong order
    // silently disables the signals you actually wanted.
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(devices));
  }

  /**
   * Converts a signal's Phoenix timestamp into the FPGA time base used by pose estimation.
   *
   * @param signal the signal whose capture time is wanted
   * @return the capture time in the FPGA time base, in seconds
   */
  public static double getLatencyCompensatedSeconds(BaseStatusSignal signal) {
    // IMPLEMENTATION (was TODO: "Return signal.getTimestamp().getTime() converted to the FPGA time
    // base the same way PhoenixOdometryThread does (FPGA now minus signal latency).")
    //
    // Phoenix timestamps and WPILib FPGA timestamps are DIFFERENT TIME BASES. Mixing them silently
    // poisons every vision measurement correlated against them, and the symptom -- a pose estimate
    // that is subtly wrong only while moving -- is very hard to trace back to here.
    //
    // Rather than trust a cross-base conversion, this computes the age of the sample from Phoenix's
    // own latency figure and subtracts it from the current FPGA time. Both terms are then in the
    // FPGA base by construction.
    return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - signal.getTimestamp().getLatency();
  }
}
