package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Predicts and mitigates roboRIO brownouts by scaling every subsystem's current limit.
 *
 * <p>Six Kraken X60s and two X44s can momentarily demand far more current than a battery at 40%
 * state of charge can supply. The result is a roboRIO brownout mid-match, which drops every closed
 * loop on the robot simultaneously. This class reads bus voltage through a low-pass filter and the
 * PDH total current, estimates the voltage slope to predict an impending brownout, and publishes a
 * scalar in {@code [kMinCurrentScalar, 1.0]} that each subsystem multiplies into its stator current
 * limit.
 *
 * <p>Non-critical mechanisms give up their amps first: the intake and floor rollers are throttled
 * before the flywheel, and the drivetrain is throttled last. The capability bought is simply that
 * you finish the match still able to drive and shoot, instead of rebooting at 30 seconds.
 */
public class PowerBudget {
  /** Priority tiers, throttled from lowest to highest as electrical headroom shrinks. */
  public enum Priority {
    /** Intake and floor rollers: sacrificed first. */
    LOW,
    /**
     * Indexer: sacrificed second. (The plan listed the intake pivot here too; the intake applies
     * LOW to both its motors, so that rocking is the first thing given up -- see {@code Intake}.)
     */
    MEDIUM,
    /** Flywheel and turret: sacrificed only under real distress. */
    HIGH,
    /** Drivetrain: never throttled, so the robot can always be driven off the field. */
    CRITICAL
  }

  /**
   * Where along the brownout severity axis each tier starts giving up current.
   *
   * <p>Severity runs from 0 at {@code kBrownoutWarningVolts} to 1 at {@code
   * kBrownoutCriticalVolts}. LOW starts at once, MEDIUM a third of the way down, HIGH two thirds of
   * the way, and every tier reaches {@code kMinCurrentScalar} together at the critical voltage. So
   * at any severity, a lower tier has given up at least as much as every higher one.
   */
  private static final double MEDIUM_STARTS_AT = 1.0 / 3.0;

  private static final double HIGH_STARTS_AT = 2.0 / 3.0;

  /** Time constant of the battery-voltage low-pass filter, seconds. */
  private static final double FILTER_TIME_CONSTANT_SEC = 0.1;

  private static PowerBudget instance = null;
  private final DoubleSupplier batteryVolts;
  private PowerDistribution pdh;
  private double filteredVoltage = Double.NaN;
  private double voltageSlopeVoltsPerSec = 0.0;

  /** How far into the brownout band the filtered voltage has fallen: 0 healthy, 1 critical. */
  private double severity = 0.0;

  /**
   * Builds a budget over a given battery-voltage source.
   *
   * <p>Package-private for tests, added in M14 step 3; the robot uses {@link #getInstance()}.
   *
   * @param batteryVolts reads the battery voltage, in volts
   */
  PowerBudget(DoubleSupplier batteryVolts) {
    this.batteryVolts = batteryVolts;
  }

  /**
   * Returns the process-wide power budget, creating it on first use.
   *
   * @return the singleton instance
   */
  public static PowerBudget getInstance() {
    // IMPLEMENTATION (was TODO: "Lazy singleton; construct PowerDistribution(Constants.Can.kPdhId,
    // ModuleType.kRev) only when Constants.currentMode == REAL so sim and replay do not touch the
    // HAL.")
    //
    // Only the lazy-singleton half is implemented here, because the drivetrain needs a non-null
    // instance to ask for its current-limit scalar. periodic() is still a stub, so getScalar()
    // returns a constant 1.0 and no throttling actually happens yet -- which is the correct, safe
    // default. The PowerDistribution object is deliberately NOT constructed until periodic() is
    // written, so nothing touches the HAL in simulation or replay.
    //
    // M14 step 3 implemented the rest. The voltage is RobotController's, which in simulation is
    // RoboRioSim's settable input, so a test can sag the battery. The PDH is opened only on a real
    // robot, as the TODO asked, so simulation and replay never touch it.
    if (instance == null) {
      instance = new PowerBudget(RobotController::getBatteryVoltage);
      if (Constants.currentMode == Constants.Mode.REAL) {
        instance.pdh = new PowerDistribution(Constants.Can.kPdhId, ModuleType.kRev);
      }
    }
    return instance;
  }

  /**
   * Samples battery voltage and current and recomputes the throttle scalar. Call once per robot
   * loop.
   *
   * <p><b>Outputs:</b> updates {@code filteredVoltage}, {@code voltageSlopeVoltsPerSec} and {@code
   * scalar}; logs bus voltage, PDH total current, brownout state and the scalar.
   */
  public void periodic() {
    // IMPLEMENTATION (was TODO: "Feed RobotController.getBatteryVoltage() through a
    // LinearFilter.singlePoleIIR (about a 0.1 s time constant at 0.02 s period), estimate the slope
    // as (new - old) / 0.02, and compute scalar: 1.0 at or above kBrownoutWarningVolts, ramping
    // linearly down to kMinCurrentScalar at kBrownoutCriticalVolts. Log pdh.getTotalCurrent(),
    // RobotController.isBrownedOut() and scalar.")
    //
    // The single-pole filter is written out rather than taken from LinearFilter, for one reason:
    // it starts from the FIRST sample. LinearFilter.singlePoleIIR starts from zero, so for its
    // first
    // few hundred milliseconds it reads a 0 V battery, and every mechanism would boot throttled.
    //
    // One ramp became a severity plus a start point per tier (scalarFor). The TODO's single linear
    // scalar gave every tier the same cut; the plan's point is that the intake gives up its amps
    // well before the flywheel does.
    double dt = Constants.getLoopPeriodSec();
    double raw = batteryVolts.getAsDouble();
    double previous = filteredVoltage;
    if (Double.isNaN(filteredVoltage)) {
      filteredVoltage = raw;
      previous = raw;
    } else {
      filteredVoltage += (dt / (FILTER_TIME_CONSTANT_SEC + dt)) * (raw - filteredVoltage);
    }
    voltageSlopeVoltsPerSec = (filteredVoltage - previous) / dt;

    double warn = Constants.Power.kBrownoutWarningVolts.get();
    double critical = Constants.Power.kBrownoutCriticalVolts.get();
    severity = MathUtil.clamp((warn - filteredVoltage) / (warn - critical), 0.0, 1.0);

    Logger.recordOutput("Power/FilteredVoltage", filteredVoltage);
    Logger.recordOutput("Power/VoltageSlopeVoltsPerSec", voltageSlopeVoltsPerSec);
    Logger.recordOutput("Power/Severity", severity);
    for (Priority priority : Priority.values()) {
      Logger.recordOutput("Power/Scalar/" + priority.name(), getScalar(priority));
    }
    Logger.recordOutput("Power/BrownoutImminent", isBrownoutImminent());
    Logger.recordOutput("Power/BrownedOut", RobotController.isBrownedOut());
    if (pdh != null) {
      Logger.recordOutput("Power/TotalCurrent", pdh.getTotalCurrent());
    }
  }

  /**
   * Returns the current-limit multiplier a subsystem of the given priority should apply.
   *
   * @param priority how essential this mechanism is to finishing the match
   * @return a multiplier in {@code [kMinCurrentScalar, 1.0]}; always 1.0 for {@link
   *     Priority#CRITICAL}
   */
  public double getScalar(Priority priority) {
    // IMPLEMENTATION (was TODO: "Return 1.0 for CRITICAL. For HIGH/MEDIUM/LOW return progressively
    // more aggressive fractions of `scalar`, so the intake gives up its amps well before the
    // drivetrain does.")
    return scalarFor(priority, severity, Constants.Power.kMinCurrentScalar.get());
  }

  /**
   * The current-limit multiplier for a tier at a given brownout severity.
   *
   * <p>Added for M14 step 3, pure so the tier ordering can be tested on its own. Each tier holds
   * 1.0 until the severity reaches its start point, then ramps linearly to the minimum at severity
   * 1. CRITICAL, the drivetrain, never moves: a robot that cannot drive off the field is worse than
   * one that browns out.
   *
   * @param priority the tier
   * @param severity how far into the brownout band the battery is, from 0 to 1
   * @param minScalar the multiplier every non-critical tier reaches at severity 1
   * @return the multiplier, in {@code [minScalar, 1.0]}
   */
  static double scalarFor(Priority priority, double severity, double minScalar) {
    double start =
        switch (priority) {
          case LOW -> 0.0;
          case MEDIUM -> MEDIUM_STARTS_AT;
          case HIGH -> HIGH_STARTS_AT;
          case CRITICAL -> 1.0;
        };
    if (severity <= start) {
      return 1.0;
    }
    double fraction = (severity - start) / (1.0 - start);
    return 1.0 - (1.0 - minScalar) * fraction;
  }

  /**
   * Reads the filtered battery voltage.
   *
   * @return low-pass-filtered bus voltage, in volts
   */
  public double getFilteredVoltage() {
    // IMPLEMENTATION (was TODO: "return filteredVoltage."), with the battery's raw reading before
    // the first sample rather than NaN.
    return Double.isNaN(filteredVoltage) ? batteryVolts.getAsDouble() : filteredVoltage;
  }

  /**
   * Reports whether a brownout looks imminent.
   *
   * <p>The superstructure uses this to refuse a flywheel spin-up it will not be able to finish.
   *
   * @return true when voltage is below the warning threshold and still falling
   */
  public boolean isBrownoutImminent() {
    // IMPLEMENTATION (was TODO: "return filteredVoltage < kBrownoutWarningVolts &&
    // voltageSlopeVoltsPerSec < 0.0.")
    //
    // Nothing reads this yet. The plan's risk register has the superstructure refuse a spin-up it
    // cannot finish; that changes what the robot does mid-match, and is left as a decision rather
    // than folded into M14.
    return filteredVoltage < Constants.Power.kBrownoutWarningVolts.get()
        && voltageSlopeVoltsPerSec < 0.0;
  }
}
