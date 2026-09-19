package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware contract for the indexer: a single Kraken X60, plus an optional beam break.
 *
 * <p>The beam break is optional hardware. With {@code Constants.Indexer.kHasBeamBreak == false} the
 * subsystem falls back to detecting a game piece from a current spike plus a velocity dip. Having
 * both paths behind one interface means deciding to add a sensor in week 5 is a one-line constant
 * change rather than a refactor.
 */
public interface IndexerIO {
  /** Everything the indexer hardware reports, captured once per loop and logged for replay. */
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected = false;
    public double positionRotations = 0.0;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public boolean beamBreakTripped = false;
    public boolean beamBreakConnected = false;
  }

  /**
   * Samples all hardware signals into the input struct.
   *
   * @param inputs the struct to populate; every field must be written every call
   */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /**
   * Commands a closed-loop indexer speed.
   *
   * @param velocityRpm target speed at the indexer shaft, in RPM; negative ejects
   */
  public default void setVelocity(double velocityRpm) {}

  /**
   * Drives the indexer open loop.
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
