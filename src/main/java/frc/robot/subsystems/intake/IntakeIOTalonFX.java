package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.optimizeBus;
import static frc.robot.util.PhoenixUtil.registerSignals;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
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

    // IMPLEMENTATION (was TODO: "Apply a TalonFXConfiguration to each motor, wrapped in
    // PhoenixUtil.tryUntilOk: Feedback.SensorToMechanismRatio, Slot0, MotionMagic cruise and
    // acceleration, SoftwareLimitSwitch forward and reverse at the stow and deploy limits,
    // MotorOutput NeutralMode = Brake, Inverted, CurrentLimits ... Set the signal update frequency
    // to 50 Hz, register them with PhoenixUtil.registerSignals(...), then
    // PhoenixUtil.optimizeBus(pivot, roller) ... Finally seed the pivot from the absolute encoder
    // so closed loop starts from truth.")
    //
    // SensorToMechanismRatio is the load-bearing line, exactly as in ShooterIOTalonFX. With it set,
    // every position and velocity the Talon reports, and every position commanded, is at the
    // GEARBOX OUTPUT rather than the rotor. That is what lets the state machine, the soft limits
    // below and the gravity feedforward all speak in mechanism radians, and it confines the
    // rotations/radians conversion to this one file (plan section 5.4).
    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.Feedback.SensorToMechanismRatio = Constants.Intake.kPivotGearRatio;
    pivotConfig.Slot0 = currentPivotGains();

    // Motion Magic speaks in mechanism rotations per second and per second squared, while the rest
    // of the project speaks in radians. Converting here rather than storing rotations in Constants
    // keeps every tunable in the same units as the numbers a student reads on the dashboard.
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        Units.radiansToRotations(Constants.Intake.kPivotCruiseRadPerSec.get());
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(Constants.Intake.kPivotAccelRadPerSec2.get());

    applySoftLimits(pivotConfig.SoftwareLimitSwitch, true);

    // BRAKE, unlike every roller in this robot. A coasting pivot falls under its own weight the
    // instant the robot is disabled, which at best drops the intake on the floor in the queue line
    // and at worst drops it onto the turret. Coast is still reachable deliberately, through
    // setPivotBrakeMode(false), which is what RobotContainer's disabled-coast passthrough uses so
    // the mechanism can be moved by hand in the pit.
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted =
        Constants.Intake.kPivotInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    applyPivotCurrentLimits(pivotConfig.CurrentLimits, 1.0);
    tryUntilOk(5, () -> pivot.getConfigurator().apply(pivotConfig, 0.25));

    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.Feedback.SensorToMechanismRatio = Constants.Intake.kRollerGearRatio;
    rollerConfig.Slot0 = currentRollerGains();

    // COAST on the roller, for the reason the floor roller coasts: a braked roller clamps whatever
    // game piece is between its wheels the moment the robot is disabled, so a piece stuck in the
    // intake cannot be pulled out by hand without fighting the gearbox.
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted =
        Constants.Intake.kRollerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    applyRollerCurrentLimits(rollerConfig.CurrentLimits, 1.0);
    tryUntilOk(5, () -> roller.getConfigurator().apply(rollerConfig, 0.25));

    // 50 Hz, matching the floor roller rather than the flywheel's 100 Hz: nothing differentiates
    // these signals, so one fresh sample per robot loop is all any consumer can use, and nine
    // devices share the roboRIO bus. Temperature is the exception at 4 Hz, because a motor takes
    // tens of seconds to heat up and anything faster is bus budget spent on a number that cannot
    // change between frames.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotVelocity,
        pivotVolts,
        pivotStator,
        pivotSupply,
        rollerVelocity,
        rollerVolts,
        rollerStator);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, pivotTemp, rollerTemp);

    // Registering puts these in the shared batch PhoenixUtil.refreshAll() drains once per
    // robotPeriodic, so that when Robot.robotPeriodic is filled in, every mechanism's measurements
    // carry the same timestamp. This IO still refreshes locally in updateInputs -- see the comment
    // there for why that is not redundant.
    registerSignals(
        pivotPosition,
        pivotVelocity,
        pivotVolts,
        pivotStator,
        pivotSupply,
        pivotTemp,
        rollerVelocity,
        rollerVolts,
        rollerStator,
        rollerTemp);

    // Must run LAST. optimizeBus disables every status frame nobody explicitly asked for, so the
    // setUpdateFrequencyForAll calls above are what "asking" means. Reversing these two silently
    // turns off the signals just requested, and the symptom is inputs frozen at zero.
    optimizeBus(pivot, roller);

    // Seeding is conditional on the encoder actually being there. A DutyCycleEncoder with nothing
    // plugged into its DIO reports 0.0 rather than failing, so seeding unconditionally would write
    // a confident, wrong zero into the Talon -- and the pivot would then drive "to the stow angle"
    // from a position off by the whole travel of the mechanism, through whatever is in the way.
    // With the encoder absent we leave the Talon's position alone; Intake starts unhomed, sees
    // absEncoderConnected false, and runs the stall-homing fallback, which is what it is for.
    //
    // The call is inlined rather than routed through seedPivotPosition() because calling an
    // overridable method from a constructor runs it before this object is fully built.
    if (pivotAbsEncoder.isConnected()) {
      double seedRad = absolutePositionRad();
      tryUntilOk(5, () -> pivot.setPosition(Units.radiansToRotations(seedRad), 0.25));
    }
  }

  /**
   * Samples both motors and the absolute encoder.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "Refresh the pivot signals and the roller signals as two SEPARATE
    // BaseStatusSignal.refreshAll groups, so each group's isOK() feeds its own Debouncer into
    // pivotConnected / rollerConnected -- a single combined refresh would report both dead when
    // only one is. Convert rotations to radians with Units.rotationsToRadians, and rotations/sec to
    // RPM with * 60. Read pivotAbsEncoder.get() and pivotAbsEncoder.isConnected() for the absolute
    // fields, subtracting Constants.Intake.kPivotAbsEncoderOffsetRot before converting.")
    //
    // Three refresh calls rather than one, because the STATUS CODE is the measurement here. The
    // failure the separation prevents is specific: a dead pivot and a dead roller demand completely
    // different responses -- a dead roller means the intake cannot pick a piece up, while a dead
    // pivot means nobody knows where the arm is and the turret interlock is running blind -- and a
    // single combined refresh would report both whenever either one failed.
    //
    // The local refresh is kept even though these signals are also in PhoenixUtil's shared batch.
    // That batch returns ONE status covering every device on the bus, so using it here would make a
    // dead device anywhere in the robot report the intake as disconnected; and Robot.robotPeriodic
    // does not call PhoenixUtil.refreshAll() yet, so an IO depending on it would silently report
    // zeros for every field. Refreshing twice costs no bus traffic: refresh reads values the
    // Phoenix driver has already received.
    var pivotStatus =
        BaseStatusSignal.refreshAll(
            pivotPosition, pivotVelocity, pivotVolts, pivotStator, pivotSupply);
    var rollerStatus = BaseStatusSignal.refreshAll(rollerVelocity, rollerVolts, rollerStator);

    // Temperature is refreshed separately and its status deliberately ignored: at 4 Hz it is stale
    // by design, and folding it into either connection check would make a healthy motor look
    // intermittently disconnected.
    BaseStatusSignal.refreshAll(pivotTemp, rollerTemp);

    // A falling-edge debounce reports a disconnect only after 0.5 s of continuous failure but
    // reports a reconnect instantly. A single dropped frame on a busy bus is normal and must not
    // raise an alert; half a second of silence is a real fault.
    inputs.pivotConnected = pivotConnectedDebounce.calculate(pivotStatus.isOK());
    inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());

    // SensorToMechanismRatio is configured, so these are already gearbox-output rotations and
    // gearbox-output rotations per second.
    inputs.pivotPositionRad = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
    inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
    inputs.pivotAppliedVolts = pivotVolts.getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivotStator.getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivotSupply.getValueAsDouble();
    inputs.pivotTempCelsius = pivotTemp.getValueAsDouble();

    // isConnected() on a DutyCycleEncoder is a frequency check on the PWM signal, which is the only
    // way the RIO can tell "encoder reading zero" apart from "encoder unplugged". Intake treats
    // this as the switch between trusting position directly and having to stall-home for it, so it
    // carries rather more weight than the usual connection boolean.
    inputs.absEncoderConnected = pivotAbsEncoder.isConnected();
    inputs.pivotAbsolutePositionRad = absolutePositionRad();

    inputs.rollerVelocityRpm = rollerVelocity.getValueAsDouble() * 60.0;
    inputs.rollerAppliedVolts = rollerVolts.getValueAsDouble();
    inputs.rollerStatorCurrentAmps = rollerStator.getValueAsDouble();
    inputs.rollerTempCelsius = rollerTemp.getValueAsDouble();
  }

  /**
   * Commands the pivot to an angle with gravity feedforward.
   *
   * @param positionRad target angle at the gearbox output, in radians
   */
  @Override
  public void setPivotPosition(double positionRad) {
    // IMPLEMENTATION (was TODO: "pivot.setControl(pivotPositionRequest.withPosition(
    // Units.radiansToRotations(positionRad)).withFeedForward(Constants.Intake.kPivotKg.get() *
    // Math.cos(positionRad))). The cosine term is the gravity feedforward for a pivoting arm; drop
    // it if yours is counterbalanced. Note the cosine is of the MECHANISM angle, so kStowedAngleRad
    // must be measured from horizontal for this to be correct.")
    //
    // THE GRAVITY FEEDFORWARD AND ITS SIGN. Gravity's torque on a pivoting arm is proportional to
    // the cosine of the angle from horizontal: largest with the arm level, zero with it straight up
    // or straight down. A constant volts-per-cosine term therefore cancels gravity across the whole
    // travel, and the closed loop is left with friction and inertia instead of a disturbance that
    // reverses sign halfway through the stroke.
    //
    // The sign convention is written out in full on Constants.Intake.kPivotKg, and summarised here
    // because this is the line that acts on it: angles are measured from HORIZONTAL with POSITIVE
    // RAISING the arm, positive motor output must raise the arm (that is what kPivotInverted is
    // for), and kPivotKg is then POSITIVE. Get it backwards and the feedforward ADDS to gravity
    // instead of opposing it: the pivot accelerates into whichever hard stop is downhill at roughly
    // twice the voltage the controller intended, which is how a gearbox is stripped on the first
    // deploy. Simulation is where to catch that -- IntakeIOSim applies this same term to a
    // SingleJointedArmSim, so a wrong sign slams the simulated arm the way it would slam the real
    // one (plan section 9.2).
    //
    // DEVIATION FROM THE TODO, deliberate: the cosine is taken of the MEASURED angle, not of the
    // commanded one. Gravity acts on where the arm IS, and Motion Magic can spend most of a second
    // profiling toward a target most of a right angle away -- for all of which the TODO's version
    // would apply the feedforward the arm will need when it ARRIVES. Near the ends of travel that
    // is the difference between full gravity compensation and none. Phoenix's own built-in
    // GravityTypeValue.Arm_Cosine does exactly this off the measured position; it is not used here
    // only because it measures from the sensor's own zero, and this pivot's zero is wherever the
    // absolute encoder offset puts it rather than at horizontal.
    double measuredRad = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
    pivot.setControl(
        pivotPositionRequest
            .withPosition(Units.radiansToRotations(positionRad))
            .withFeedForward(Constants.Intake.kPivotKg.get() * Math.cos(measuredRad)));
  }

  /**
   * Drives the pivot open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setPivotOpenLoop(double volts) {
    // IMPLEMENTATION (was TODO: "pivot.setControl(voltageRequest.withOutput(volts)).")
    //
    // No gravity feedforward on this path, on purpose. It serves homing and characterization, and
    // both need to know what a KNOWN voltage does to the mechanism. Quietly adding a position-
    // dependent term would bias a characterization fit, and would change the force homing presses
    // into the hard stop with depending on where the arm happened to start from.
    pivot.setControl(voltageRequest.withOutput(volts));
  }

  /**
   * Commands the roller to a closed-loop speed.
   *
   * @param velocityRpm target speed at the roller shaft, in RPM
   */
  @Override
  public void setRollerVelocity(double velocityRpm) {
    // IMPLEMENTATION (was TODO: "roller.setControl(rollerVelocityRequest.withVelocity(velocityRpm /
    // 60.0)). Phoenix velocity units are rotations per second, not RPM.")
    //
    // Negative values pass straight through rather than being clamped: EJECT is a negative RPM
    // goal. Slot0's kS is signed by whoever sets it for the same reason -- see currentRollerGains.
    roller.setControl(rollerVelocityRequest.withVelocity(velocityRpm / 60.0));
  }

  /**
   * Drives the roller open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setRollerOpenLoop(double volts) {
    // IMPLEMENTATION (was TODO: "roller.setControl(voltageRequest.withOutput(volts)).")
    //
    // A VoltageOut request replaces the VelocityVoltage one outright, so there is no stale closed
    // loop left running underneath.
    roller.setControl(voltageRequest.withOutput(volts));
  }

  /**
   * Seeds the pivot's rotor position from a known true angle.
   *
   * @param positionRad the true current angle at the gearbox output, in radians
   */
  @Override
  public void seedPivotPosition(double positionRad) {
    // IMPLEMENTATION (was TODO: "PhoenixUtil.tryUntilOk(5, () -> pivot.setPosition(
    // Units.radiansToRotations(positionRad), 0.25)).")
    //
    // tryUntilOk matters more here than on an ordinary configuration call. Every angle this
    // mechanism commands afterwards is relative to this one write, so a setPosition that silently
    // failed once -- a single dropped frame on a busy bus at boot -- leaves the pivot confidently
    // driving to angles that are all wrong by the same offset, with nothing anywhere reporting a
    // fault.
    tryUntilOk(5, () -> pivot.setPosition(Units.radiansToRotations(positionRad), 0.25));
  }

  /**
   * Sets the pivot's neutral behavior.
   *
   * @param brake true for brake, false for coast
   */
  @Override
  public void setPivotBrakeMode(boolean brake) {
    // IMPLEMENTATION (was TODO: "Apply a MotorOutputConfigs carrying the requested
    // NeutralModeValue, via tryUntilOk.")
    //
    // Inversion is re-stated alongside the neutral mode. MotorOutputConfigs is a whole
    // configuration GROUP, so applying one that sets only NeutralMode resets Inverted to its
    // default -- which on this mechanism reverses the pivot, and the very next thing that happens
    // is the gravity feedforward pushing the arm downhill instead of holding it up.
    var output = new MotorOutputConfigs();
    output.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    output.Inverted =
        Constants.Intake.kPivotInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> pivot.getConfigurator().apply(output, 0.25));
  }

  /**
   * Enables or disables the pivot's software travel limits.
   *
   * @param enabled true to enforce the configured forward and reverse soft limits
   *     <p><b>Outputs:</b> reconfigures the pivot Talon's software limit switches.
   */
  @Override
  public void setPivotSoftLimitsEnabled(boolean enabled) {
    // IMPLEMENTATION (a method added beyond the plan's list; IntakeIO documents why it exists).
    // Called only on state transitions -- Intake turns the limits off on entering HOMING and back
    // on when it leaves -- so this needs none of the change deadband setCurrentLimitScalar carries.
    //
    // The thresholds are recomputed from the angle tunables on every call rather than latched at
    // boot, so re-tuning the stow or deploy angle on the dashboard and then re-homing moves the
    // limits to where the new angles say they belong, with no redeploy.
    var limits = new SoftwareLimitSwitchConfigs();
    applySoftLimits(limits, enabled);
    tryUntilOk(5, () -> pivot.getConfigurator().apply(limits, 0.25));
  }

  /**
   * Pushes new closed-loop gains to the pivot.
   *
   * @param gains the gain set to apply
   */
  @Override
  public void setPivotGains(Slot0Configs gains) {
    // IMPLEMENTATION (was TODO: "PhoenixUtil.tryUntilOk(5, () -> pivot.getConfigurator().apply(
    // gains, 0.25)).")
    //
    // A bare Slot0Configs is applied rather than a whole TalonFXConfiguration. Phoenix merges a
    // partial configuration group in place; applying a full configuration object here would reset
    // every other group -- the soft limits and the current limits included -- for at least one
    // loop, every time somebody nudged a gain on the dashboard.
    tryUntilOk(5, () -> pivot.getConfigurator().apply(gains, 0.25));
  }

  /**
   * Scales the configured current limits.
   *
   * @param scalar multiplier in {@code [0, 1]}
   */
  @Override
  public void setCurrentLimitScalar(double scalar) {
    // IMPLEMENTATION (was TODO: "Store it, and only when it has moved more than about 5% from the
    // last applied value, apply a CurrentLimitsConfigs with kPivotStatorLimitAmps * scalar and
    // kRollerStatorLimitAmps * scalar. Reapplying every loop would flood the CAN bus.")
    //
    // The 5% deadband is what makes this safe to call every loop from Intake.periodic(). This
    // mechanism is PowerBudget.Priority.LOW, so its scalar is among the first to move and the one
    // that moves most often; without the guard a sagging battery would put two configuration frames
    // on the bus every 20 ms, precisely when the bus is busiest.
    if (Math.abs(scalar - currentLimitScalar) < 0.05) {
      return;
    }
    currentLimitScalar = scalar;

    var pivotLimits = new CurrentLimitsConfigs();
    applyPivotCurrentLimits(pivotLimits, scalar);
    tryUntilOk(5, () -> pivot.getConfigurator().apply(pivotLimits, 0.25));

    var rollerLimits = new CurrentLimitsConfigs();
    applyRollerCurrentLimits(rollerLimits, scalar);
    tryUntilOk(5, () -> roller.getConfigurator().apply(rollerLimits, 0.25));
  }

  /** Stops both motors immediately. */
  @Override
  public void stop() {
    // IMPLEMENTATION (was TODO: "pivot.stopMotor(); roller.stopMotor().")
    //
    // stopMotor() issues a neutral request, so each motor falls to its own configured neutral mode:
    // the pivot brakes and holds roughly where it is, the roller coasts down. That asymmetry is the
    // entire point of configuring them differently. It is deliberately NOT a closed-loop hold at
    // the current angle -- this is what FAULTED calls, and holding a jammed pivot against a stator
    // limit for the rest of the match is how a motor is destroyed.
    pivot.stopMotor();
    roller.stopMotor();
  }

  /**
   * Reads the absolute encoder and converts it to a mechanism angle.
   *
   * <p>The offset is subtracted in the encoder's own units (rotations), and the result is wrapped
   * into plus or minus half a turn before conversion. Without the wrap, a pivot sitting just the
   * other side of the ThroughBore's 0-to-1 rollover reads a full rotation away from where it truly
   * is, and the mechanism would drive its entire travel to "correct" that. The wrap assumes the
   * travel does not itself span the rollover: choose {@code kPivotAbsEncoderOffsetRot} so that the
   * rollover sits outside the arc the intake actually sweeps.
   *
   * <p>CALIBRATION: set {@code kPivotAbsEncoderOffsetRot} to 0, put the pivot on its stow hard
   * stop, read {@code Intake/Inputs/pivotAbsolutePositionRad}, and store {@code
   * Units.radiansToRotations(that reading) - Units.radiansToRotations(kStowedAngleRad)}.
   *
   * @return the pivot angle the absolute encoder reports, in radians at the gearbox output
   */
  private double absolutePositionRad() {
    // Per robot since M14 step 5: the competition robot's offset, or the practice robot's.
    double offsetRot =
        pivotAbsEncoder.get()
            - Constants.forRobot(
                Constants.Intake.kPivotAbsEncoderOffsetRot.get(),
                Constants.PracticeRobot.kIntakePivotAbsEncoderOffsetRot.get());
    return Units.rotationsToRadians(MathUtil.inputModulus(offsetRot, -0.5, 0.5));
  }

  /**
   * Builds a {@link Slot0Configs} from the current values of the tunable pivot gains.
   *
   * <p>Used only to seed the initial configuration. Retuning at runtime goes through {@link
   * #setPivotGains(Slot0Configs)}, driven by {@code Intake.periodic()}.
   *
   * @return the pivot gain set described by {@code Constants.Intake}
   */
  private static Slot0Configs currentPivotGains() {
    // kG is deliberately NOT set here even though Slot0Configs has a field for it. Phoenix applies
    // its kG only when GravityType is configured to match the mechanism, and its Arm_Cosine mode
    // measures the angle from the SENSOR's zero -- which on this pivot is wherever the absolute
    // encoder offset puts it, not horizontal. The gravity term is applied explicitly instead, in
    // setPivotPosition, where the angle it uses is under our control.
    //
    // kI has no tunable in Constants and stays at the Slot0Configs default of zero: an integrator
    // on an arm winds up while the mechanism sits against a hard stop -- which this one does
    // through every homing sequence -- and then dumps all of it the moment the arm comes free.
    var gains = new Slot0Configs();
    gains.kP = Constants.Intake.kPivotKp.get();
    gains.kD = Constants.Intake.kPivotKd.get();
    gains.kS = Constants.Intake.kPivotKs.get();
    gains.kV = Constants.Intake.kPivotKv.get();
    return gains;
  }

  /**
   * Builds a {@link Slot0Configs} from the current values of the tunable roller gains.
   *
   * @return the roller gain set described by {@code Constants.Intake}
   */
  private static Slot0Configs currentRollerGains() {
    // kS is signed by whoever sets it, and this roller genuinely runs both ways: EJECT is a
    // negative RPM. A positive static-friction term on a backwards command pushes against the very
    // motion being asked for, which is a bug that hides completely on a flywheel.
    var gains = new Slot0Configs();
    gains.kP = Constants.Intake.kRollerKp.get();
    gains.kS = Constants.Intake.kRollerKs.get();
    gains.kV = Constants.Intake.kRollerKv.get();
    return gains;
  }

  /**
   * Fills a software-limit configuration group from the pivot's travel, with a margin.
   *
   * <p>The limits bracket the outermost angle the state machine can command, widened by {@code
   * kPivotSoftLimitMarginRad} so that homing can still drive into a hard stop. Taking the minimum
   * and maximum across all three commanded angles, rather than assuming stow is the low end, keeps
   * this correct whichever way round the mechanism turns out to be built -- which matters, because
   * none of those three angles has been measured yet.
   *
   * @param limits the configuration group to fill
   * @param enabled whether the limits should be enforced
   *     <p><b>Outputs:</b> mutates {@code limits} in place.
   */
  private static void applySoftLimits(SoftwareLimitSwitchConfigs limits, boolean enabled) {
    double stowRad = Constants.Intake.kStowedAngleRad.get();
    double deployRad = Constants.Intake.kDeployedAngleRad.get();
    double ejectRad = Constants.Intake.kEjectAngleRad.get();
    double marginRad = Constants.Intake.kPivotSoftLimitMarginRad;

    double lowestRad = Math.min(stowRad, Math.min(deployRad, ejectRad)) - marginRad;
    double highestRad = Math.max(stowRad, Math.max(deployRad, ejectRad)) + marginRad;

    // All four fields are written on every call. SoftwareLimitSwitchConfigs is a configuration
    // GROUP, so applying one with only the enable flags set resets both thresholds to zero -- and a
    // soft limit at zero in both directions freezes the pivot solidly, with no error anywhere.
    //
    // NOTE for whoever first powers this mechanism up: while the three angle constants are all
    // still 0.0, these limits collapse to a band one margin wide, and the pivot will barely move.
    // That is the intended failure direction. Measure the angles first.
    limits.ForwardSoftLimitThreshold = Units.radiansToRotations(highestRad);
    limits.ForwardSoftLimitEnable = enabled;
    limits.ReverseSoftLimitThreshold = Units.radiansToRotations(lowestRad);
    limits.ReverseSoftLimitEnable = enabled;
  }

  /**
   * Fills a current-limit configuration group with the pivot's limit, scaled.
   *
   * <p>One place builds these so that the boot configuration and every later {@link
   * #setCurrentLimitScalar(double)} call cannot drift apart -- a mismatch would mean the limit
   * silently changes the first time the power budget throttles anything.
   *
   * @param limits the configuration group to fill
   * @param scalar multiplier in {@code [0, 1]} applied to the stator limit
   *     <p><b>Outputs:</b> mutates {@code limits} in place.
   */
  private static void applyPivotCurrentLimits(CurrentLimitsConfigs limits, double scalar) {
    // Stator only, as on the floor roller. Stator limiting already bounds the torque the pivot can
    // put into a hard stop, which is the failure this mechanism actually has -- it spends part of
    // every homing sequence deliberately stalled -- and Constants.Intake carries no supply figure.
    // Inventing one here would be a guessed electrical limit with nothing to check it against,
    // which is exactly what the project's conventions forbid.
    limits.StatorCurrentLimit = Constants.Intake.kPivotStatorLimitAmps * scalar;
    limits.StatorCurrentLimitEnable = true;
  }

  /**
   * Fills a current-limit configuration group with the roller's limit, scaled.
   *
   * @param limits the configuration group to fill
   * @param scalar multiplier in {@code [0, 1]} applied to the stator limit
   *     <p><b>Outputs:</b> mutates {@code limits} in place.
   */
  private static void applyRollerCurrentLimits(CurrentLimitsConfigs limits, double scalar) {
    limits.StatorCurrentLimit = Constants.Intake.kRollerStatorLimitAmps * scalar;
    limits.StatorCurrentLimitEnable = true;
  }
}
