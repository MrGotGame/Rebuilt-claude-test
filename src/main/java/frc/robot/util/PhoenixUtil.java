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
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

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

  private PhoenixUtil() {}

  /**
   * Runs a Phoenix configuration command, retrying until it reports success.
   *
   * @param maxAttempts how many times to try before giving up
   * @param command the configuration call to run; returns the Phoenix status code
   *     <p><b>Outputs:</b> applies the configuration to hardware as a side effect.
   */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
    // TODO: After the loop, if the last attempt still failed, raise a FaultReporter alert naming
    // the
    // device. A configuration that never applied is currently silent, which is how a robot ends up
    // running with default PID gains and nobody knowing why it behaves oddly.
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
    // TODO: Add all signals to registeredSignals, then rebuild signalArray via
    // registeredSignals.toArray(new BaseStatusSignal[0]). Rebuilding once here keeps refreshAll
    // allocation-free.
  }

  /**
   * Refreshes every registered signal in one batched call. Call once per {@code robotPeriodic},
   * before any subsystem periodic runs.
   *
   * <p><b>Outputs:</b> updates all registered signals in place; logs the batch status under {@code
   * Phoenix/RefreshStatus}.
   */
  public static void refreshAll() {
    // TODO: Call BaseStatusSignal.refreshAll(signalArray) and record the returned StatusCode with
    // Logger.recordOutput("Phoenix/RefreshStatus", code.toString()). Guard against an empty array.
  }

  /**
   * Reduces bus traffic from devices whose signals are not all needed.
   *
   * @param devices the Phoenix devices to optimize
   *     <p><b>Outputs:</b> disables unrequested status frames on each device.
   */
  public static void optimizeBus(ParentDevice... devices) {
    // TODO: Wrap ParentDevice.optimizeBusUtilizationForAll(devices) in tryUntilOk. Every Talon in
    // this project must be optimized: with nine devices on the RIO bus, leaving default frame rates
    // enabled will saturate it and show up as erratic loop overruns.
    return;
  }

  /**
   * Converts a signal's Phoenix timestamp into the FPGA time base used by pose estimation.
   *
   * @param signal the signal whose capture time is wanted
   * @return the capture time in the FPGA time base, in seconds
   */
  public static double getLatencyCompensatedSeconds(BaseStatusSignal signal) {
    // TODO: Convert the same way PhoenixOdometryThread already does -- FPGA now minus the signal's
    // reported latency. Verify the exact accessor against Phoenix6 26.1.2; mixing time bases here
    // silently poisons every vision measurement that is correlated against it.
    return 0.0;
  }
}
