package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/** Real-hardware indexer: one Kraken X60 on the RIO bus, with an optional beam break on DIO. */
public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX motor;
  private final DigitalInput beamBreak;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * Constructs the Talon, the optional beam break, and the status signals.
   *
   * <p>NOTE: device and signal construction is pre-filled because these fields are final. Your work
   * is the configuration, in the TODO.
   */
  public IndexerIOTalonFX() {
    motor = new TalonFX(Constants.Can.kIndexerId, Constants.Can.kRioCanBus);
    beamBreak =
        Constants.Indexer.kHasBeamBreak ? new DigitalInput(Constants.Dio.kIndexerBeamBreak) : null;
    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();

    // TODO: Same configuration pattern as FloorRollerIOTalonFX, using Constants.Indexer values, but
    // set MotorOutput NeutralMode = BRAKE so a held piece does not roll back out when disabled.
    // TODO: Set signal update frequency to 50 Hz, PhoenixUtil.registerSignals(...), then
    // PhoenixUtil.optimizeBus(motor).
  }

  /**
   * Samples the motor and the beam break.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // TODO: refreshAll on the six signals, feed isOK() through connectedDebounce, fill every field.
    // For the beam break: guard on beamBreak != null, then
    // inputs.beamBreakTripped = !beamBreak.get(). TODO: VERIFY your sensor's polarity on the bench
    // before trusting that inversion -- most beam breaks are active-low, but not all are, and
    // getting it backwards means the robot believes it always has a piece.
  }

  /**
   * Commands a closed-loop indexer speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: motor.setControl(velocityRequest.withVelocity(velocityRpm / 60.0)).
  }

  /**
   * Drives the indexer open loop.
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
    // TODO: Scale kStatorLimitAmps. The indexer is PowerBudget.Priority.MEDIUM.
  }

  /** Stops the motor immediately. */
  @Override
  public void stop() {
    // TODO: motor.stopMotor().
  }
}
