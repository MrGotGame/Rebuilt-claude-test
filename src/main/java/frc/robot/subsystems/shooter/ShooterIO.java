package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware contract for the flywheel: two Kraken X60s in leader/follower.
 *
 * <p>Both motors' telemetry is logged <b>separately</b>, deliberately. A follower that has lost its
 * {@code Follower} request draws almost no current while the leader draws double to compensate.
 * That failure is completely invisible unless both motors are logged, and it presents as "the shots
 * got weak" rather than as an obvious fault.
 */
public interface ShooterIO {
  /** Everything the flywheel hardware reports, captured once per loop and logged for replay. */
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public double velocityRpm = 0.0;
    public double goalRpm = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double followerAppliedVolts = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;
    public double followerTempCelsius = 0.0;
    public double closedLoopErrorRpm = 0.0;
  }

  /**
   * Samples all hardware signals into the input struct.
   *
   * @param inputs the struct to populate; every field must be written every call
   */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * Commands a closed-loop flywheel speed.
   *
   * @param velocityRpm target speed at the flywheel shaft, in RPM
   */
  public default void setVelocity(double velocityRpm) {}

  /**
   * Drives the flywheel open loop, for characterization.
   *
   * @param volts commanded voltage
   */
  public default void setOpenLoop(double volts) {}

  /**
   * Pushes new closed-loop gains to the leader.
   *
   * @param gains the gain set to apply
   */
  public default void setGains(Slot0Configs gains) {}

  /**
   * Scales this mechanism's current limits for brownout mitigation.
   *
   * @param scalar multiplier in {@code [0, 1]} applied to the configured limits on both motors
   */
  public default void setCurrentLimitScalar(double scalar) {}

  /**
   * Sets the neutral behavior of both motors.
   *
   * @param brake true for brake; normally always false, since braking a flywheel wastes the stored
   *     energy that makes the next shot fast. Brake exists only for a pit-safety mode
   */
  public default void setNeutralMode(boolean brake) {}

  /** Stops the leader; the follower coasts with it. */
  public default void stop() {}
}
