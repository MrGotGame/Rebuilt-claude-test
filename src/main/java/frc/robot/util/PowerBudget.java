package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;

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
    /** Indexer and pivot: sacrificed second. */
    MEDIUM,
    /** Flywheel and turret: sacrificed only under real distress. */
    HIGH,
    /** Drivetrain: never throttled, so the robot can always be driven off the field. */
    CRITICAL
  }

  private static PowerBudget instance = null;
  private PowerDistribution pdh;
  private double filteredVoltage = 12.0;
  private double voltageSlopeVoltsPerSec = 0.0;
  private double scalar = 1.0;

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
    if (instance == null) {
      instance = new PowerBudget();
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
    // TODO: Feed RobotController.getBatteryVoltage() through a LinearFilter.singlePoleIIR (about a
    // 0.1 s time constant at 0.02 s period), estimate the slope as (new - old) / 0.02, and compute
    // scalar: 1.0 at or above kBrownoutWarningVolts, ramping linearly down to kMinCurrentScalar at
    // kBrownoutCriticalVolts. Log pdh.getTotalCurrent(), RobotController.isBrownedOut() and scalar.
  }

  /**
   * Returns the current-limit multiplier a subsystem of the given priority should apply.
   *
   * @param priority how essential this mechanism is to finishing the match
   * @return a multiplier in {@code [kMinCurrentScalar, 1.0]}; always 1.0 for {@link
   *     Priority#CRITICAL}
   */
  public double getScalar(Priority priority) {
    // TODO: Return 1.0 for CRITICAL. For HIGH/MEDIUM/LOW return progressively more aggressive
    // fractions of `scalar`, so the intake gives up its amps well before the drivetrain does.
    return 1.0;
  }

  /**
   * Reads the filtered battery voltage.
   *
   * @return low-pass-filtered bus voltage, in volts
   */
  public double getFilteredVoltage() {
    // TODO: return filteredVoltage.
    return 12.0;
  }

  /**
   * Reports whether a brownout looks imminent.
   *
   * <p>The superstructure uses this to refuse a flywheel spin-up it will not be able to finish.
   *
   * @return true when voltage is below the warning threshold and still falling
   */
  public boolean isBrownoutImminent() {
    // TODO: return filteredVoltage < kBrownoutWarningVolts && voltageSlopeVoltsPerSec < 0.0.
    return false;
  }
}
