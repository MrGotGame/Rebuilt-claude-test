package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

/**
 * The hardware contract for the pivoting intake: one Kraken X44 on the pivot, one Kraken X60 on the
 * roller, and a WCP ThroughBore absolute encoder reading the <i>gearbox output</i>.
 *
 * <p>Because the encoder is on the output rather than the motor, the pivot angle is directly
 * observable and no homing sequence is strictly required once the offset is calibrated. A homing
 * routine is kept anyway as the fallback for an unplugged or dead encoder.
 *
 * <p>Every input field is a public primitive so AdvantageKit's {@link AutoLog} processor can
 * generate the replayable input struct.
 */
public interface IntakeIO {
  /** Everything the intake hardware reports, captured once per loop and logged for replay. */
  @AutoLog
  public static class IntakeIOInputs {
    public boolean pivotConnected = false;
    public boolean absEncoderConnected = false;
    public double pivotPositionRad = 0.0;
    public double pivotAbsolutePositionRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotStatorCurrentAmps = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotTempCelsius = 0.0;

    public boolean rollerConnected = false;
    public double rollerVelocityRpm = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
    public double rollerTempCelsius = 0.0;
  }

  /**
   * Samples all hardware signals into the input struct. Called once per loop, before any control
   * decision.
   *
   * @param inputs the struct to populate; every field must be written every call
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Commands the pivot to an angle under Motion Magic profiled position control.
   *
   * @param positionRad target angle at the gearbox output, in radians
   */
  public default void setPivotPosition(double positionRad) {}

  /**
   * Drives the pivot open loop, for homing and manual jogging.
   *
   * @param volts commanded voltage, positive toward the deployed direction
   */
  public default void setPivotOpenLoop(double volts) {}

  /**
   * Commands the intake roller to a closed-loop speed.
   *
   * @param velocityRpm target speed at the roller shaft, in RPM; negative ejects
   */
  public default void setRollerVelocity(double velocityRpm) {}

  /**
   * Drives the intake roller open loop.
   *
   * @param volts commanded voltage
   */
  public default void setRollerOpenLoop(double volts) {}

  /**
   * Seeds the pivot motor's internal rotor position, so closed loop runs off the Talon's high-rate
   * sensor rather than the slow DIO encoder.
   *
   * @param positionRad the true current angle at the gearbox output, in radians
   */
  public default void seedPivotPosition(double positionRad) {}

  /**
   * Sets the pivot's neutral behavior.
   *
   * @param brake true for brake, false for coast; coast is wanted when disabled so the pivot can be
   *     moved by hand
   */
  public default void setPivotBrakeMode(boolean brake) {}

  /**
   * Pushes new closed-loop gains to the pivot motor.
   *
   * @param gains the gain set to apply
   */
  public default void setPivotGains(Slot0Configs gains) {}

  /**
   * Scales this mechanism's current limits, so the power budget can throttle it during a brownout.
   *
   * @param scalar multiplier in {@code [0, 1]} applied to the configured stator limits
   */
  public default void setCurrentLimitScalar(double scalar) {}

  /** Stops both motors immediately. */
  public default void stop() {}
}
