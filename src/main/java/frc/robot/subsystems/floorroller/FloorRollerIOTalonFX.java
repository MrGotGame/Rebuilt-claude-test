package frc.robot.subsystems.floorroller;

import static frc.robot.util.PhoenixUtil.optimizeBus;
import static frc.robot.util.PhoenixUtil.registerSignals;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
   * Last current-limit scalar actually pushed to hardware, for the 5% change guard in {@link
   * #setCurrentLimitScalar(double)}.
   */
  private double lastAppliedCurrentScalar = 1.0;

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

    // IMPLEMENTATION (was TODO: "Apply a TalonFXConfiguration via PhoenixUtil.tryUntilOk with
    // Feedback.SensorToMechanismRatio = kGearRatio, Slot0 from the Constants.FloorRoller gains,
    // MotorOutput NeutralMode = COAST ... CurrentLimits from kStatorLimitAmps, and Inverted per
    // kInverted" and "Set signal update frequency to 50 Hz, PhoenixUtil.registerSignals(...), then
    // PhoenixUtil.optimizeBus(motor).")
    //
    // SensorToMechanismRatio is the load-bearing line, exactly as in ShooterIOTalonFX. With it set,
    // every velocity the Talon reports and every velocity commanded is at the ROLLER shaft rather
    // than the rotor, which is what confines the RPM/rot-per-sec conversion to this one file (plan
    // section 5.4) and lets the state machine above speak only in roller RPM.
    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = Constants.FloorRoller.kGearRatio;
    config.Slot0 = currentGains();

    // COAST, and for a different reason than the flywheel's. A braked roller clamps whatever game
    // piece is between its wheels the instant the robot is disabled, so a piece half-way through
    // the floor path cannot be pulled out by hand in the queue line without fighting the gearbox.
    // Coasting also means a piece being pushed into a disabled robot back-drives the roller instead
    // of jamming against it.
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        Constants.FloorRoller.kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    applyCurrentLimits(config.CurrentLimits, 1.0);
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // 50 Hz, not the flywheel's 100 Hz. The floor roller has no predictive readiness gate to feed
    // -- nothing differentiates its velocity -- so one fresh sample per 50 Hz robot loop is all any
    // consumer can use, and nine devices share the roboRIO bus. Temperature is the exception at
    // 4 Hz: a motor takes tens of seconds to heat up, so paying any more for it is bus budget spent
    // on a number that cannot change between frames.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, appliedVolts, statorCurrent, supplyCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, temp);

    // Registering puts these signals in the shared batch that PhoenixUtil.refreshAll() will refresh
    // once per robotPeriodic, so that when Robot.robotPeriodic is filled in every mechanism's
    // measurements carry the same timestamp. This IO still refreshes locally in updateInputs as
    // well -- see the comment there for why that is not redundant.
    registerSignals(velocity, appliedVolts, statorCurrent, supplyCurrent, temp);

    // Must run LAST. optimizeBus disables every status frame nobody explicitly asked for, so the
    // setUpdateFrequencyForAll calls above are what "asking" means. Reversing these two silently
    // turns off the signals just requested, and the symptom is inputs frozen at zero.
    optimizeBus(motor);
  }

  /**
   * Samples the motor.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(FloorRollerIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "BaseStatusSignal.refreshAll on the five signals, feed isOK()
    // through connectedDebounce into inputs.connected, then fill every field. velocityRpm =
    // velocity.getValueAsDouble() * 60.")
    //
    // The local refresh is kept even though these signals are also in PhoenixUtil's shared batch,
    // for two reasons. First, the StatusCode is the measurement that drives inputs.connected: the
    // shared batch returns ONE status covering every device on the bus, so using it here would make
    // a dead device anywhere in the robot report the floor roller as disconnected. Second,
    // Robot.robotPeriodic does not call PhoenixUtil.refreshAll() yet -- it is still a TODO -- and
    // an IO that depended on it would silently report zeros for every field with nothing flagging
    // the fault. Refreshing twice costs nothing on the bus: refresh reads values the Phoenix driver
    // has already received, and bus traffic is governed by the update frequencies set above.
    //
    // Temperature is refreshed separately and its status deliberately ignored, because at 4 Hz it
    // is stale by design and folding it into the connection check would make a healthy motor look
    // intermittently disconnected.
    var status = BaseStatusSignal.refreshAll(velocity, appliedVolts, statorCurrent, supplyCurrent);
    BaseStatusSignal.refreshAll(temp);

    // A falling-edge debounce reports a disconnect only after 0.5 s of continuous failure but
    // reports a reconnect instantly. A single dropped frame on a busy bus is normal and must not
    // raise an alert; half a second of silence is a real fault.
    inputs.connected = connectedDebounce.calculate(status.isOK());

    // SensorToMechanismRatio is configured, so this is already roller rotations per second.
    inputs.velocityRpm = velocity.getValueAsDouble() * 60.0;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  /**
   * Commands a closed-loop roller speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // IMPLEMENTATION (was TODO: "motor.setControl(velocityRequest.withVelocity(velocityRpm /
    // 60.0)).")
    //
    // Negative values are passed straight through rather than clamped: EJECT is a negative RPM
    // goal, and this is the first mechanism in the project that genuinely runs backwards in normal
    // operation. Slot0's kS term is signed by the caller for the same reason.
    motor.setControl(velocityRequest.withVelocity(velocityRpm / 60.0));
  }

  /**
   * Drives the roller open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setOpenLoop(double volts) {
    // IMPLEMENTATION (was TODO: "motor.setControl(voltageRequest.withOutput(volts)).")
    //
    // A VoltageOut request replaces the VelocityVoltage one outright, so there is no stale closed
    // loop left running underneath a characterization sweep.
    motor.setControl(voltageRequest.withOutput(volts));
  }

  /**
   * Pushes new closed-loop gains.
   *
   * @param gains the gain set to apply
   */
  @Override
  public void setGains(Slot0Configs gains) {
    // IMPLEMENTATION (was TODO: "PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(
    // gains, 0.25)).")
    //
    // A bare Slot0Configs is applied rather than a whole TalonFXConfiguration. Phoenix merges a
    // partial configuration group in place; applying a full configuration object here would reset
    // every other group -- current limits and neutral mode included -- for at least one loop, every
    // time somebody nudged a gain on the dashboard.
    tryUntilOk(5, () -> motor.getConfigurator().apply(gains, 0.25));
  }

  /**
   * Scales the configured current limit.
   *
   * @param scalar multiplier in {@code [0, 1]}
   */
  @Override
  public void setCurrentLimitScalar(double scalar) {
    // IMPLEMENTATION (was TODO: "Scale kStatorLimitAmps. This roller is PowerBudget.Priority.LOW,
    // so it is the first mechanism to give up amps when the battery sags.")
    //
    // The 5% deadband is what makes this safe to call every loop from FloorRoller.periodic(). It
    // matters more here than anywhere else in the robot: LOW priority means this scalar is the
    // first to move and the one that moves most often, so without the guard a sagging battery would
    // put a configuration frame on the bus every 20 ms precisely when the bus is busiest.
    if (Math.abs(scalar - lastAppliedCurrentScalar) < 0.05) {
      return;
    }
    lastAppliedCurrentScalar = scalar;

    var limits = new CurrentLimitsConfigs();
    applyCurrentLimits(limits, scalar);
    tryUntilOk(5, () -> motor.getConfigurator().apply(limits, 0.25));
  }

  /** Stops the motor immediately. */
  @Override
  public void stop() {
    // IMPLEMENTATION (was TODO: "motor.stopMotor().")
    //
    // stopMotor() issues a neutral request, which with NeutralModeValue.Coast lets the roller spin
    // down freely. Deliberately NOT setVelocity(0.0): a closed-loop zero would actively brake the
    // roller and hold it there, which fights a game piece somebody is trying to pull out and burns
    // stator current doing nothing useful.
    motor.stopMotor();
  }

  /**
   * Builds a {@link Slot0Configs} from the current values of the tunable floor-roller gains.
   *
   * <p>Used only to seed the initial configuration. Retuning at runtime goes through {@link
   * #setGains(Slot0Configs)}, driven by {@code FloorRoller.periodic()}.
   *
   * @return the gain set described by {@code Constants.FloorRoller}
   */
  private static Slot0Configs currentGains() {
    // kI and kD are left at the Slot0Configs default of zero and have no tunable of their own, in
    // Constants or here. An integrator on a roller winds up while a game piece is stalling it and
    // then dumps that accumulated output the moment the piece clears, which throws the piece; a
    // derivative term amplifies the velocity quantisation a single-encoder roller reports. kP plus
    // the kS/kV feedforward is the whole controller this mechanism wants.
    var gains = new Slot0Configs();
    gains.kP = Constants.FloorRoller.kKp.get();
    gains.kS = Constants.FloorRoller.kKs.get();
    gains.kV = Constants.FloorRoller.kKv.get();
    return gains;
  }

  /**
   * Fills a current-limit configuration group with this mechanism's limit, scaled.
   *
   * <p>One place builds these so that the boot configuration and every later {@link
   * #setCurrentLimitScalar(double)} call cannot drift apart -- a mismatch there would mean the
   * limit silently changes the first time the power budget throttles anything.
   *
   * @param limits the configuration group to fill
   * @param scalar multiplier in {@code [0, 1]} applied to the stator limit
   *     <p><b>Outputs:</b> mutates {@code limits} in place.
   */
  private static void applyCurrentLimits(CurrentLimitsConfigs limits, double scalar) {
    limits.StatorCurrentLimit = Constants.FloorRoller.kStatorLimitAmps * scalar;
    limits.StatorCurrentLimitEnable = true;

    // Stator only, unlike the flywheel's two-stage supply limit. Stator limiting already bounds the
    // torque this roller can put into a jammed game piece, which is the failure this mechanism
    // actually has, and Constants.FloorRoller carries no supply figure. Inventing one here would be
    // a guessed electrical limit with nothing to check it against, which is exactly what the
    // conventions forbid; add it alongside a measured number if the PDH channel ever trips.
  }
}
