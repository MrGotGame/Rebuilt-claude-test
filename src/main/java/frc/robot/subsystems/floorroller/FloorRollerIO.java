package frc.robot.subsystems.floorroller;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware contract for the floor roller: a single Kraken X60 under closed-loop velocity control.
 */
public interface FloorRollerIO {
  /** Everything the floor roller hardware reports, captured once per loop and logged for replay. */
  @AutoLog
  public static class FloorRollerIOInputs {
    public boolean connected = false;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /**
   * Samples all hardware signals into the input struct.
   *
   * @param inputs the struct to populate; every field must be written every call
   */
  public default void updateInputs(FloorRollerIOInputs inputs) {}

  /**
   * Commands a closed-loop roller speed.
   *
   * @param velocityRpm target speed at the roller shaft, in RPM; negative ejects
   */
  public default void setVelocity(double velocityRpm) {}

  /**
   * Drives the roller open loop.
   *
   * @param volts commanded voltage
   */
  public default void setOpenLoop(double volts) {}

  /**
   * Pushes new closed-loop gains to the motor.
   *
   * @param gains the gain set to apply
   */
  public default void setGains(Slot0Configs gains) {}

  /**
   * Scales this mechanism's current limits for brownout mitigation.
   *
   * @param scalar multiplier in {@code [0, 1]} applied to the configured stator limit
   */
  public default void setCurrentLimitScalar(double scalar) {}

  /** Stops the motor immediately. */
  public default void stop() {}
}
