package frc.robot.subsystems.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

/**
 * Real-hardware turret: a Kraken X44 with a WCP ThroughBore on the gearbox output, read on a
 * roboRIO DIO.
 *
 * <p>The Talon's software limit switches are configured from the same tunable limits the unwrapper
 * uses, so that even a software bug upstream cannot drive the turret into its hard stops.
 *
 * <p><b>ContinuousWrap must stay OFF.</b> This is a finite-travel turret, not a swerve azimuth.
 * Letting Phoenix wrap the setpoint would silently defeat the entire unwrap design and let the
 * turret take the short way around straight through a hard stop.
 */
public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;
  private final DutyCycleEncoder absEncoder;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Double> closedLoopError;
  private final StatusSignal<Double> closedLoopReference;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * Constructs the Talon, the absolute encoder, and the status signals.
   *
   * <p>NOTE: device and signal construction is pre-filled because these fields are final. Your work
   * is the configuration, in the TODO.
   */
  public TurretIOTalonFX() {
    motor = new TalonFX(Constants.Can.kTurretId, Constants.Can.kRioCanBus);
    absEncoder = new DutyCycleEncoder(Constants.Dio.kTurretAbsEncoder);
    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();
    // TODO: VERIFY these two accessors and their generic types against Phoenix6 26.1.2 -- the
    // closed-loop signal getters have moved between releases.
    closedLoopError = motor.getClosedLoopError();
    closedLoopReference = motor.getClosedLoopReference();

    // TODO: Apply a TalonFXConfiguration via PhoenixUtil.tryUntilOk with:
    //   - Feedback.SensorToMechanismRatio = Constants.Turret.kGearRatio
    //   - Slot0 from the Constants.Turret tunable gains
    //   - MotionMagic cruise and acceleration from kCruiseRadPerSec / kAccelRadPerSec2
    //   - ClosedLoopGeneral.ContinuousWrap = FALSE  <-- critical, see the class docs
    //   - SoftwareLimitSwitch forward and reverse ENABLED, at kMaxAngleRad - kLimitMarginRad and
    //     kMinAngleRad + kLimitMarginRad, converted to rotations
    //   - MotorOutput NeutralMode = Brake, Inverted per kInverted
    //   - CurrentLimits from kStatorLimitAmps
    // TODO: Register signals at 100 Hz, not 50 -- the shot solver wants the freshest possible
    // turret
    // angle. Then PhoenixUtil.optimizeBus(motor) and seed position from the absolute encoder.
  }

  /**
   * Samples the motor and the absolute encoder.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // TODO: refreshAll on all eight signals, feed isOK() through connectedDebounce, fill every
    // field. For the absolute reading:
    //   inputs.absolutePositionRad = MathUtil.angleModulus(
    //       Units.rotationsToRadians(absEncoder.get() - kAbsEncoderOffsetRot.get()));
    // Convert closedLoopError and closedLoopReference from rotations to radians.
  }

  /**
   * Commands a position with velocity feedforward.
   *
   * @param positionRad continuous target position, in radians
   * @param velocityFeedforwardRadPerSec target angular rate, in radians per second
   */
  @Override
  public void setPositionSetpoint(double positionRad, double velocityFeedforwardRadPerSec) {
    // TODO: motor.setControl(positionRequest
    //     .withPosition(Units.radiansToRotations(positionRad))
    //     .withFeedForward(kKs.get() * Math.signum(velocityFeedforwardRadPerSec)
    //         + kKv.get() * Units.radiansToRotations(velocityFeedforwardRadPerSec)));
    // TODO: VERIFY the withFeedForward unit convention against Phoenix6 26.1.2. It is VOLTS for the
    // *Voltage requests and AMPS for the *TorqueCurrentFOC requests; mixing them up produces a
    // feedforward that is wrong by roughly an order of magnitude and looks like a tuning problem.
  }

  /**
   * Drives the turret open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setOpenLoop(double volts) {
    // TODO: motor.setControl(voltageRequest.withOutput(volts)).
  }

  /**
   * Seeds the motor position from a known true angle.
   *
   * @param positionRad the true current angle, in radians
   */
  @Override
  public void seedPosition(double positionRad) {
    // TODO: PhoenixUtil.tryUntilOk(5,
    //     () -> motor.setPosition(Units.radiansToRotations(positionRad), 0.25));
  }

  /**
   * Pushes new closed-loop gains.
   *
   * @param gains the gain set to apply
   */
  @Override
  public void setGains(Slot0Configs gains) {
    // TODO: PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(gains, 0.25)).
  }

  /**
   * Re-applies the firmware travel guard.
   *
   * @param minRad lower travel limit, in radians
   * @param maxRad upper travel limit, in radians
   */
  @Override
  public void setSoftLimits(double minRad, double maxRad) {
    // TODO: Apply a SoftwareLimitSwitchConfigs with BOTH limits enabled, converted to rotations.
    // Call this whenever the limit tunables change, so that re-tuning a limit at a competition
    // cannot leave the firmware guard behind at the old value.
  }

  /**
   * Sets the turret's neutral behavior.
   *
   * @param brake true for brake, false for coast
   */
  @Override
  public void setBrakeMode(boolean brake) {
    // TODO: Apply a MotorOutputConfigs carrying the requested NeutralModeValue, via tryUntilOk.
  }

  /**
   * Scales the configured current limit.
   *
   * @param scalar multiplier in {@code [0, 1]}
   */
  @Override
  public void setCurrentLimitScalar(double scalar) {
    // TODO: Scale kStatorLimitAmps. The turret is PowerBudget.Priority.HIGH -- nearly as important
    // as the drivetrain, because a turret that cannot move cannot score.
  }

  /** Stops the motor immediately. */
  @Override
  public void stop() {
    // TODO: motor.stopMotor().
  }
}
