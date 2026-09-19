package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware contract for the turret: a Kraken X44 with a WCP ThroughBore on the gearbox output.
 *
 * <p>This is a <b>finite-travel</b> mechanism, not a continuously-rotating azimuth. Position is
 * reported and commanded as a continuous (unwrapped) angle, and it is the caller's job to keep that
 * angle inside the mechanical limits.
 */
public interface TurretIO {
  /** Everything the turret hardware reports, captured once per loop and logged for replay. */
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public boolean absEncoderConnected = false;

    /** Continuous (unwrapped) turret position in the robot frame, radians. */
    public double positionRad = 0.0;

    /** Raw absolute encoder reading mapped into [-pi, pi), radians. */
    public double absolutePositionRad = 0.0;

    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double closedLoopErrorRad = 0.0;
    public double closedLoopReferenceRad = 0.0;
  }

  /**
   * Samples all hardware signals into the input struct.
   *
   * @param inputs the struct to populate; every field must be written every call
   */
  public default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Motion Magic position control with an added velocity feedforward.
   *
   * @param positionRad continuous target position in the turret's own frame, in radians; must
   *     already be inside the mechanical limits
   * @param velocityFeedforwardRadPerSec expected angular rate of the target, used to cancel chassis
   *     rotation so the turret holds its aim while the drivetrain spins
   */
  public default void setPositionSetpoint(
      double positionRad, double velocityFeedforwardRadPerSec) {}

  /**
   * Drives the turret open loop, for homing and manual jogging.
   *
   * @param volts commanded voltage
   */
  public default void setOpenLoop(double volts) {}

  /**
   * Seeds the motor's internal position from a known true angle.
   *
   * @param positionRad the true current angle at the gearbox output, in radians
   */
  public default void seedPosition(double positionRad) {}

  /**
   * Pushes new closed-loop gains to the motor.
   *
   * @param gains the gain set to apply
   */
  public default void setGains(Slot0Configs gains) {}

  /**
   * Pushes the mechanical travel limits down into the motor controller's own firmware guard.
   *
   * @param minRad lower travel limit, in radians
   * @param maxRad upper travel limit, in radians
   */
  public default void setSoftLimits(double minRad, double maxRad) {}

  /**
   * Sets the turret's neutral behavior.
   *
   * @param brake true for brake, false for coast
   */
  public default void setBrakeMode(boolean brake) {}

  /**
   * Scales this mechanism's current limits for brownout mitigation.
   *
   * @param scalar multiplier in {@code [0, 1]} applied to the configured stator limit
   */
  public default void setCurrentLimitScalar(double scalar) {}

  /** Stops the motor immediately. */
  public default void stop() {}
}
