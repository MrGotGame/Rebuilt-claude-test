package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
 * Real-hardware intake IO.
 *
 * <p>The subtle piece is the absolute encoder. The WCP ThroughBore is wired to a roboRIO DIO and
 * read with {@link DutyCycleEncoder}, which lives on the RIO rather than the CAN bus, so it has no
 * Phoenix status signal and no latency compensation. We read it at boot -- and again whenever the
 * pivot is at rest and the seed has drifted -- to {@code setPosition()} the Talon's internal rotor
 * sensor, then run closed loop off the Talon's own high-rate sensor.
 *
 * <p>UPGRADE PATH: replacing the DIO encoder with a CTRE CANdi lets you set {@code
 * Feedback.FeedbackSensorSource = FusedCANdiPWM1} and delete the seeding logic entirely.
 */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX pivot;
  private final TalonFX roller;
  private final DutyCycleEncoder pivotAbsEncoder;

  private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotVolts;
  private final StatusSignal<Current> pivotStator;
  private final StatusSignal<Current> pivotSupply;
  private final StatusSignal<Temperature> pivotTemp;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerVolts;
  private final StatusSignal<Current> rollerStator;
  private final StatusSignal<Temperature> rollerTemp;

  private final Debouncer pivotConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rollerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private double currentLimitScalar = 1.0;

  /**
   * Constructs both Talons, the absolute encoder, and the status signals.
   *
   * <p>NOTE: the device and signal construction below is pre-filled because these fields are final
   * and Java requires them to be assigned here. Your work is the CONFIGURATION, in the TODO.
   */
  public IntakeIOTalonFX() {
    pivot = new TalonFX(Constants.Can.kIntakePivotId, Constants.Can.kRioCanBus);
    roller = new TalonFX(Constants.Can.kIntakeRollerId, Constants.Can.kRioCanBus);
    pivotAbsEncoder = new DutyCycleEncoder(Constants.Dio.kIntakePivotAbsEncoder);

    pivotPosition = pivot.getPosition();
    pivotVelocity = pivot.getVelocity();
    pivotVolts = pivot.getMotorVoltage();
    pivotStator = pivot.getStatorCurrent();
    pivotSupply = pivot.getSupplyCurrent();
    pivotTemp = pivot.getDeviceTemp();
    rollerVelocity = roller.getVelocity();
    rollerVolts = roller.getMotorVoltage();
    rollerStator = roller.getStatorCurrent();
    rollerTemp = roller.getDeviceTemp();

    // TODO: Apply a TalonFXConfiguration to each motor, wrapped in PhoenixUtil.tryUntilOk:
    //   - Feedback.SensorToMechanismRatio = Constants.Intake.kPivotGearRatio
    //   - Slot0 from the Constants.Intake pivot gains
    //   - MotionMagic cruise velocity and acceleration from kPivotCruiseRadPerSec / kPivotAccel
    //   - SoftwareLimitSwitch forward and reverse at the stow and deploy limits
    //   - MotorOutput NeutralMode = Brake, Inverted per kPivotInverted / kRollerInverted
    //   - CurrentLimits from kPivotStatorLimitAmps / kRollerStatorLimitAmps
    // TODO: Set the signal update frequency to 50 Hz, register them with
    // PhoenixUtil.registerSignals(...), then PhoenixUtil.optimizeBus(pivot, roller).
    // TODO: Finally seed the pivot from the absolute encoder so closed loop starts from truth.
  }

  /**
   * Samples both motors and the absolute encoder.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO: Refresh the pivot signals and the roller signals as two SEPARATE BaseStatusSignal
    // .refreshAll groups, so each group's isOK() feeds its own Debouncer into pivotConnected /
    // rollerConnected -- a single combined refresh would report both dead when only one is.
    // Convert rotations to radians with Units.rotationsToRadians, and rotations/sec to RPM with
    // * 60. Read pivotAbsEncoder.get() and pivotAbsEncoder.isConnected() for the absolute fields,
    // subtracting Constants.Intake.kPivotAbsEncoderOffsetRot before converting.
  }

  /**
   * Commands the pivot to an angle with gravity feedforward.
   *
   * @param positionRad target angle at the gearbox output, in radians
   */
  @Override
  public void setPivotPosition(double positionRad) {
    // TODO: pivot.setControl(pivotPositionRequest
    //     .withPosition(Units.radiansToRotations(positionRad))
    //     .withFeedForward(Constants.Intake.kPivotKg.get() * Math.cos(positionRad)));
    // The cosine term is the gravity feedforward for a pivoting arm; drop it if yours is
    // counterbalanced. Note the cosine is of the MECHANISM angle, so kStowedAngleRad must be
    // measured from horizontal for this to be correct.
  }

  /**
   * Drives the pivot open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setPivotOpenLoop(double volts) {
    // TODO: pivot.setControl(voltageRequest.withOutput(volts)).
  }

  /**
   * Commands the roller to a closed-loop speed.
   *
   * @param velocityRpm target speed at the roller shaft, in RPM
   */
  @Override
  public void setRollerVelocity(double velocityRpm) {
    // TODO: roller.setControl(rollerVelocityRequest.withVelocity(velocityRpm / 60.0)). Phoenix
    // velocity units are rotations per second, not RPM.
  }

  /**
   * Drives the roller open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setRollerOpenLoop(double volts) {
    // TODO: roller.setControl(voltageRequest.withOutput(volts)).
  }

  /**
   * Seeds the pivot's rotor position from a known true angle.
   *
   * @param positionRad the true current angle at the gearbox output, in radians
   */
  @Override
  public void seedPivotPosition(double positionRad) {
    // TODO: PhoenixUtil.tryUntilOk(5,
    //     () -> pivot.setPosition(Units.radiansToRotations(positionRad), 0.25));
  }

  /**
   * Sets the pivot's neutral behavior.
   *
   * @param brake true for brake, false for coast
   */
  @Override
  public void setPivotBrakeMode(boolean brake) {
    // TODO: Apply a MotorOutputConfigs carrying the requested NeutralModeValue, via tryUntilOk.
  }

  /**
   * Pushes new closed-loop gains to the pivot.
   *
   * @param gains the gain set to apply
   */
  @Override
  public void setPivotGains(Slot0Configs gains) {
    // TODO: PhoenixUtil.tryUntilOk(5, () -> pivot.getConfigurator().apply(gains, 0.25)).
  }

  /**
   * Scales the configured current limits.
   *
   * @param scalar multiplier in {@code [0, 1]}
   */
  @Override
  public void setCurrentLimitScalar(double scalar) {
    // TODO: Store it, and only when it has moved more than about 5% from the last applied value,
    // apply a CurrentLimitsConfigs with kPivotStatorLimitAmps * scalar and
    // kRollerStatorLimitAmps * scalar. Reapplying every loop would flood the CAN bus.
  }

  /** Stops both motors immediately. */
  @Override
  public void stop() {
    // TODO: pivot.stopMotor(); roller.stopMotor().
  }
}
