package frc.robot.subsystems.floorroller;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/** Real-hardware floor roller: one Kraken X60 on the RIO bus, closed-loop velocity. */
public class FloorRollerIOTalonFX implements FloorRollerIO {
  private final TalonFX motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * Constructs the Talon and its status signals.
   *
   * <p>NOTE: device and signal construction is pre-filled because these fields are final. Your work
   * is the configuration, in the TODO.
   */
  public FloorRollerIOTalonFX() {
    motor = new TalonFX(Constants.Can.kFloorRollerId, Constants.Can.kRioCanBus);
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();

    // TODO: Apply a TalonFXConfiguration via PhoenixUtil.tryUntilOk with:
    //   - Feedback.SensorToMechanismRatio = Constants.FloorRoller.kGearRatio
    //   - Slot0 from the Constants.FloorRoller gains
    //   - MotorOutput NeutralMode = COAST. A roller should not fight a game piece when disabled.
    //   - CurrentLimits from kStatorLimitAmps, and Inverted per kInverted
    // TODO: Set signal update frequency to 50 Hz, PhoenixUtil.registerSignals(...), then
    // PhoenixUtil.optimizeBus(motor).
  }

  /**
   * Samples the motor.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(FloorRollerIOInputs inputs) {
    // TODO: BaseStatusSignal.refreshAll on the five signals, feed isOK() through connectedDebounce
    // into inputs.connected, then fill every field. velocityRpm = velocity.getValueAsDouble() * 60.
  }

  /**
   * Commands a closed-loop roller speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: motor.setControl(velocityRequest.withVelocity(velocityRpm / 60.0)).
  }

  /**
   * Drives the roller open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setOpenLoop(double volts) {
    // TODO: motor.setControl(voltageRequest.withOutput(volts)).
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
   * Scales the configured current limit.
   *
   * @param scalar multiplier in {@code [0, 1]}
   */
  @Override
  public void setCurrentLimitScalar(double scalar) {
    // TODO: Scale kStatorLimitAmps. This roller is PowerBudget.Priority.LOW, so it is the first
    // mechanism to give up amps when the battery sags.
  }

  /** Stops the motor immediately. */
  @Override
  public void stop() {
    // TODO: motor.stopMotor().
  }
}
