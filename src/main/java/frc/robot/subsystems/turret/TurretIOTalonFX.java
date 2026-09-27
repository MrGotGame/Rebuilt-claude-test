package frc.robot.subsystems.turret;

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
   * Last current-limit scalar actually pushed to hardware, for the 5% change guard in {@link
   * #setCurrentLimitScalar(double)}.
   */
  private double lastAppliedCurrentScalar = 1.0;

  /** Lower travel limit last pushed to hardware, for the change guard in setSoftLimits. */
  private double lastAppliedMinRad = Double.NaN;

  /** Upper travel limit last pushed to hardware, for the change guard in setSoftLimits. */
  private double lastAppliedMaxRad = Double.NaN;

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

    // IMPLEMENTATION (was TODO: "Apply a TalonFXConfiguration via PhoenixUtil.tryUntilOk with
    // Feedback.SensorToMechanismRatio = Constants.Turret.kGearRatio; Slot0 from the
    // Constants.Turret tunable gains; MotionMagic cruise and acceleration from kCruiseRadPerSec /
    // kAccelRadPerSec2; ClosedLoopGeneral.ContinuousWrap = FALSE; SoftwareLimitSwitch forward and
    // reverse ENABLED, at kMaxAngleRad - kLimitMarginRad and kMinAngleRad + kLimitMarginRad,
    // converted to rotations; MotorOutput NeutralMode = Brake, Inverted per kInverted;
    // CurrentLimits from kStatorLimitAmps.")
    //
    // SensorToMechanismRatio is the load-bearing line, exactly as in ShooterIOTalonFX. With it set,
    // every position and velocity the Talon reports, and every setpoint commanded, is at the TURRET
    // shaft rather than the rotor. That is what lets Turret, TurretUnwrapper and RobotState all
    // speak in turret radians and never think about the gearbox, and it confines the
    // rotations/radians conversion to this one file (plan section 5.4).
    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = Constants.Turret.kGearRatio;
    config.Slot0 = currentGains();

    // Motion Magic's units are mechanism rotations per second and per second squared, because
    // SensorToMechanismRatio is set above. The constants are named in radians, so the conversion
    // has to happen here; leaving it out would make the turret cruise at 12 ROTATIONS per second,
    // roughly six times the intended rate, which on a mechanism with hard stops is a broken belt
    // rather than a tuning complaint.
    config.MotionMagic.MotionMagicCruiseVelocity =
        Units.radiansToRotations(Constants.Turret.kCruiseRadPerSec.get());
    config.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(Constants.Turret.kAccelRadPerSec2.get());

    // THE critical line in this file, and the reason the class carries a warning of its own.
    // ContinuousWrap tells Phoenix the mechanism is circular, so a setpoint 359 degrees away should
    // be reached by turning one degree the other way. Correct for a swerve azimuth; catastrophic
    // here, because the "other way" runs straight through a hard stop. Making that decision in
    // software, where the travel limits are known, is the entire purpose of TurretUnwrapper.
    config.ClosedLoopGeneral.ContinuousWrap = false;

    // The firmware guard. It duplicates limits TurretUnwrapper already respects, deliberately: the
    // unwrapper protects against an unreachable AIM goal, and this protects against everything else
    // -- a bad manual jog, a mis-seeded position, a future caller that bypasses the unwrapper. A
    // mechanism whose only protection is the code that commands it has no protection at all.
    applySoftLimits(config.SoftwareLimitSwitch, softLimitMinRad(), softLimitMaxRad());

    // BRAKE, unlike every roller in this robot. A coasting turret keeps slewing after a disable,
    // and a turret that drifts while disabled arrives at the next enable somewhere other than where
    // its seeded position claims. Brake also stops a disabled robot being pushed across the field
    // from swinging its turret into a field element.
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        Constants.Turret.kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    applyCurrentLimits(config.CurrentLimits, 1.0);
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    lastAppliedMinRad = softLimitMinRad();
    lastAppliedMaxRad = softLimitMaxRad();

    // IMPLEMENTATION (was TODO: "Register signals at 100 Hz, not 50 -- the shot solver wants the
    // freshest possible turret angle. Then PhoenixUtil.optimizeBus(motor) and seed position from
    // the absolute encoder.")
    //
    // 100 Hz, not the 50 Hz the rollers use. This is the one mechanism whose position is consumed
    // by something other than its own control loop: RobotState's turret-angle buffer is what M8's
    // moving-camera transform interpolates against, and a stale turret angle there becomes a
    // bearing error on every vision measurement taken while the turret is slewing. Temperature
    // stays at 4 Hz -- a motor takes tens of seconds to heat up, and nine devices share this bus.
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        position,
        velocity,
        appliedVolts,
        statorCurrent,
        supplyCurrent,
        closedLoopError,
        closedLoopReference);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, temp);

    // Registering puts these signals in the shared batch that PhoenixUtil.refreshAll() will refresh
    // once per robotPeriodic, so every mechanism's measurements carry the same timestamp. This IO
    // still refreshes locally in updateInputs as well -- see the comment there for why that is not
    // redundant.
    registerSignals(
        position,
        velocity,
        appliedVolts,
        statorCurrent,
        supplyCurrent,
        closedLoopError,
        closedLoopReference,
        temp);

    // Must run LAST. optimizeBus disables every status frame nobody explicitly asked for, so the
    // setUpdateFrequencyForAll calls above are what "asking" means. Reversing these two silently
    // turns off the signals just requested, and the symptom is inputs frozen at zero.
    optimizeBus(motor);

    // A boot seed, not the authoritative one. Turret's HOMING state re-seeds from the same encoder
    // on the first loop anything is asked of the turret that needs a trustworthy zero, and it
    // resolves the branch ambiguity this call cannot (see Turret.resolveAbsoluteSeed). This exists
    // so Turret/Inputs/positionRad is a plausible number from the very first log frame rather than
    // reading zero, and so the firmware soft limits above guard something real before homing.
    //
    // A DutyCycleEncoder that has not yet seen a full PWM frame reads 0.0, so this seed can be
    // wrong at boot. That is tolerable precisely because it is not the authoritative seed.
    seedPosition(readAbsoluteRad());
  }

  /**
   * Samples the motor and the absolute encoder.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "refreshAll on all eight signals, feed isOK() through
    // connectedDebounce, fill every field. For the absolute reading: inputs.absolutePositionRad =
    // MathUtil.angleModulus(Units.rotationsToRadians(absEncoder.get() -
    // kAbsEncoderOffsetRot.get())). Convert closedLoopError and closedLoopReference from rotations
    // to radians.")
    //
    // DEVIATION FROM THE TODO, deliberate: the signals are refreshed in TWO groups, and only the
    // first group's status decides motorConnected. The closed-loop signals are published by the
    // Talon only while it is actually running a closed-loop control mode; during HOMING, MANUAL and
    // CHARACTERIZING the turret is open loop, so folding those two into the health check would
    // report the motor disconnected for the whole of every homing routine -- and homing is exactly
    // when somebody is watching the alert list. Temperature is excluded for the opposite reason: at
    // 4 Hz it is stale by design.
    //
    // The local refresh is kept even though these signals are also in PhoenixUtil's shared batch.
    // That batch returns ONE status covering every device on the bus, so using it here would make a
    // dead device anywhere in the robot report the turret as disconnected; and Robot.robotPeriodic
    // does not call PhoenixUtil.refreshAll() yet.
    var status =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, statorCurrent, supplyCurrent);
    BaseStatusSignal.refreshAll(closedLoopError, closedLoopReference, temp);

    // A falling-edge debounce reports a disconnect only after 0.5 s of continuous failure, but
    // reports a reconnect instantly. A single dropped frame on a busy bus is normal and must not
    // raise an alert; half a second of silence is a real fault.
    inputs.motorConnected = connectedDebounce.calculate(status.isOK());

    // No debounce on the encoder. DutyCycleEncoder.isConnected() tests whether a PWM frame arrived
    // on the RIO's own DIO pin rather than completing a CAN round trip, so it does not suffer the
    // occasional dropped frame the debounce above exists to absorb.
    inputs.absEncoderConnected = absEncoder.isConnected();

    // SensorToMechanismRatio is configured, so these are already turret radians and radians per
    // second. The rotor never appears above this line.
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.absolutePositionRad = readAbsoluteRad();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();

    // These are measured against Motion Magic's INSTANTANEOUS PROFILE REFERENCE, not against the
    // final goal -- which is why Turret.atGoal() tests the distance to the commanded goal as well.
    // A profile being tracked perfectly reports a closed-loop error near zero for the entire sweep,
    // so a position-only readiness gate built on this field alone would fire mid-slew.
    inputs.closedLoopErrorRad = Units.rotationsToRadians(closedLoopError.getValueAsDouble());
    inputs.closedLoopReferenceRad =
        Units.rotationsToRadians(closedLoopReference.getValueAsDouble());
  }

  /**
   * Commands a position with velocity feedforward.
   *
   * @param positionRad continuous target position, in radians
   * @param velocityFeedforwardRadPerSec target angular rate, in radians per second
   *     <p><b>Outputs:</b> issues a Motion Magic position request to the turret Talon.
   */
  @Override
  public void setPositionSetpoint(double positionRad, double velocityFeedforwardRadPerSec) {
    // IMPLEMENTATION (was TODO: "motor.setControl(positionRequest
    // .withPosition(Units.radiansToRotations(positionRad))
    // .withFeedForward(kKs.get() * Math.signum(velocityFeedforwardRadPerSec)
    // + kKv.get() * Units.radiansToRotations(velocityFeedforwardRadPerSec)))." plus "VERIFY the
    // withFeedForward unit convention against Phoenix6 26.1.2.")
    //
    // The request is MotionMagicVoltage, so withFeedForward is in VOLTS and is added to the
    // controller's own output. kS and kV are therefore the usual voltage feedforward pair, and kV
    // multiplies ROTATIONS per second, because that is the unit Slot0 gains are expressed in once
    // SensorToMechanismRatio is configured.
    //
    // TODO: VERIFY against Phoenix6 26.1.2 that withFeedForward on a *Voltage request is volts. It
    // is amps on the *TorqueCurrentFOC requests, and mixing the two up produces a feedforward wrong
    // by roughly an order of magnitude that presents as a tuning problem rather than a unit bug.
    //
    // Math.signum on the kS term is load-bearing here in a way it is not on a flywheel: this
    // mechanism runs in BOTH directions, and a fixed +kS would push the turret one way whenever it
    // was asked to track the other. At zero feedforward signum returns zero, so a turret holding a
    // stationary aim gets no static-friction term at all -- correct, since there is no motion to
    // break friction for.
    double feedforwardVolts =
        Constants.Turret.kKs.get() * Math.signum(velocityFeedforwardRadPerSec)
            + Constants.Turret.kKv.get() * Units.radiansToRotations(velocityFeedforwardRadPerSec);
    motor.setControl(
        positionRequest
            .withPosition(Units.radiansToRotations(positionRad))
            .withFeedForward(feedforwardVolts));
  }

  /**
   * Drives the turret open loop.
   *
   * @param volts commanded voltage
   *     <p><b>Outputs:</b> issues a voltage request to the turret Talon.
   */
  @Override
  public void setOpenLoop(double volts) {
    // IMPLEMENTATION (was TODO: "motor.setControl(voltageRequest.withOutput(volts)).")
    //
    // The firmware soft limits apply to a VoltageOut request too, which is what makes exposing an
    // open-loop path on a mechanism with hard stops safe at all. Note that stall-homing
    // (Turret.homing) works only because the position is untrustworthy before homing, so the
    // firmware guard is at that moment guarding a fiction and the mechanism itself is what stops
    // the motion. That is why kHomingVolts is small.
    motor.setControl(voltageRequest.withOutput(volts));
  }

  /**
   * Seeds the motor position from a known true angle.
   *
   * @param positionRad the true current angle, in radians
   *     <p><b>Outputs:</b> overwrites the Talon's internal position.
   */
  @Override
  public void seedPosition(double positionRad) {
    // IMPLEMENTATION (was TODO: "PhoenixUtil.tryUntilOk(5,
    // () -> motor.setPosition(Units.radiansToRotations(positionRad), 0.25)).")
    //
    // tryUntilOk rather than a bare call, because this is the one write whose failure is completely
    // silent: a dropped setPosition leaves the Talon believing its old position, the soft limits
    // then guard the wrong window, and the first AIM request drives into a hard stop.
    tryUntilOk(5, () -> motor.setPosition(Units.radiansToRotations(positionRad), 0.25));
  }

  /**
   * Pushes new closed-loop gains.
   *
   * @param gains the gain set to apply
   *     <p><b>Outputs:</b> writes the Slot0 configuration group on the Talon.
   */
  @Override
  public void setGains(Slot0Configs gains) {
    // IMPLEMENTATION (was TODO: "PhoenixUtil.tryUntilOk(5,
    // () -> motor.getConfigurator().apply(gains, 0.25)).")
    //
    // A bare Slot0Configs is applied rather than a whole TalonFXConfiguration. Phoenix merges a
    // partial configuration group in place; applying a full configuration object here would reset
    // every other group -- including, on this mechanism, the software limit switches -- for at
    // least one loop. On a turret that window is long enough to reach a stop.
    tryUntilOk(5, () -> motor.getConfigurator().apply(gains, 0.25));
  }

  /**
   * Re-applies the firmware travel guard.
   *
   * @param minRad lower travel limit, in radians
   * @param maxRad upper travel limit, in radians
   *     <p><b>Outputs:</b> writes the SoftwareLimitSwitch configuration group on the Talon.
   */
  @Override
  public void setSoftLimits(double minRad, double maxRad) {
    // IMPLEMENTATION (was TODO: "Apply a SoftwareLimitSwitchConfigs with BOTH limits enabled,
    // converted to rotations. Call this whenever the limit tunables change, so that re-tuning a
    // limit at a competition cannot leave the firmware guard behind at the old value.")
    //
    // The equality guard makes this safe to call unconditionally. Turret.periodic() already gates
    // it behind LoggedTunableNumber.ifChanged, but a configuration write costs a round trip on a
    // bus carrying nine devices, and this is the second line of defence against a future caller
    // that pushes limits every loop.
    if (minRad == lastAppliedMinRad && maxRad == lastAppliedMaxRad) {
      return;
    }
    lastAppliedMinRad = minRad;
    lastAppliedMaxRad = maxRad;

    var limits = new SoftwareLimitSwitchConfigs();
    applySoftLimits(limits, minRad, maxRad);
    tryUntilOk(5, () -> motor.getConfigurator().apply(limits, 0.25));
  }

  /**
   * Sets the turret's neutral behavior.
   *
   * @param brake true for brake, false for coast
   *     <p><b>Outputs:</b> writes the MotorOutput configuration group on the Talon.
   */
  @Override
  public void setBrakeMode(boolean brake) {
    // IMPLEMENTATION (was TODO: "Apply a MotorOutputConfigs carrying the requested
    // NeutralModeValue, via tryUntilOk.")
    //
    // Inversion is re-stated. MotorOutputConfigs is a whole configuration GROUP, so applying one
    // that sets only NeutralMode resets Inverted to its default -- which on this mechanism reverses
    // the direction of the position loop and sends the turret to the opposite hard stop at full
    // Motion Magic cruise.
    var output = new MotorOutputConfigs();
    output.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    output.Inverted =
        Constants.Turret.kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> motor.getConfigurator().apply(output, 0.25));
  }

  /**
   * Scales the configured current limit.
   *
   * @param scalar multiplier in {@code [0, 1]}
   *     <p><b>Outputs:</b> writes the CurrentLimits configuration group on the Talon.
   */
  @Override
  public void setCurrentLimitScalar(double scalar) {
    // IMPLEMENTATION (was TODO: "Scale kStatorLimitAmps. The turret is PowerBudget.Priority.HIGH --
    // nearly as important as the drivetrain, because a turret that cannot move cannot score.")
    //
    // The 5% deadband is what makes this safe to call every loop from Turret.periodic(). Without
    // it, a configuration frame per loop at 50 Hz would be spent re-applying a limit that had not
    // changed, on a bus that already carries nine devices.
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
    // stopMotor() replaces the active control request with a neutral one, which with the configured
    // Brake neutral mode holds the turret where it is. That is the wanted behaviour in IDLE and in
    // FAULTED: a turret left in a closed-loop hold against a jam would sit at the stator limit for
    // the rest of the match.
    motor.stopMotor();
  }

  /**
   * Reads the absolute encoder and converts it to a wrapped turret angle.
   *
   * <p>The result is in {@code [-pi, pi)} because that is all a single-turn encoder on the output
   * shaft can honestly report. Resolving that into a position inside the travel limits is {@code
   * Turret}'s job, not this one's -- see {@code Turret.resolveAbsoluteSeed}.
   *
   * @return the absolute turret angle, wrapped into {@code [-pi, pi)}, in radians
   */
  private double readAbsoluteRad() {
    // One reader, used by both the boot seed and updateInputs, so the offset convention cannot
    // drift between them. A boot seed computed with a different sign from the runtime reading is
    // the kind of bug that only shows up as "the turret is fine until you re-home it".
    return MathUtil.angleModulus(
        Units.rotationsToRadians(
            absEncoder.get()
                - Constants.forRobot(
                    Constants.Turret.kAbsEncoderOffsetRot.get(),
                    Constants.PracticeRobot.kTurretAbsEncoderOffsetRot.get())));
  }

  /**
   * The lower firmware travel limit, held back from the hard stop by the safety margin.
   *
   * @return lower soft limit, in radians
   */
  private static double softLimitMinRad() {
    return Constants.Turret.kMinAngleRad.get() + Constants.Turret.kLimitMarginRad.get();
  }

  /**
   * The upper firmware travel limit, held back from the hard stop by the safety margin.
   *
   * @return upper soft limit, in radians
   */
  private static double softLimitMaxRad() {
    return Constants.Turret.kMaxAngleRad.get() - Constants.Turret.kLimitMarginRad.get();
  }

  /**
   * Builds a {@link Slot0Configs} from the current values of the tunable turret gains.
   *
   * <p>Used only to seed the initial configuration. Retuning at runtime goes through {@link
   * #setGains(Slot0Configs)}, driven by {@code Turret.periodic()}.
   *
   * @return the gain set described by {@code Constants.Turret}
   */
  private static Slot0Configs currentGains() {
    var gains = new Slot0Configs();
    gains.kP = Constants.Turret.kKp.get();

    // No kI, and Constants.Turret deliberately does not define one. An integrator buys nothing on a
    // mechanism that holds a near-zero load, and it winds up whenever the turret is held against a
    // soft limit by an out-of-range aim request -- then delivers the accumulated term as a lurch
    // the moment the goal becomes reachable again. OUT_OF_RANGE is a normal state here, not an
    // exceptional one, so that would happen in most matches.
    gains.kD = Constants.Turret.kKd.get();
    gains.kS = Constants.Turret.kKs.get();
    gains.kV = Constants.Turret.kKv.get();
    gains.kA = Constants.Turret.kKa.get();
    return gains;
  }

  /**
   * Fills a software-limit configuration group from travel limits in radians.
   *
   * <p>One place builds these so that the boot configuration and every later {@link
   * #setSoftLimits(double, double)} call cannot drift apart.
   *
   * @param limits the configuration group to fill
   * @param minRad lower travel limit, in radians
   * @param maxRad upper travel limit, in radians
   *     <p><b>Outputs:</b> mutates {@code limits} in place.
   */
  private static void applySoftLimits(
      SoftwareLimitSwitchConfigs limits, double minRad, double maxRad) {
    // Both limits are ENABLED, not merely set. A threshold with its enable flag left false is the
    // most common way this configuration group silently does nothing, and the symptom -- a turret
    // that drives straight past a limit visibly configured in Tuner X -- sends people looking at
    // the software instead of at the checkbox.
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = Units.radiansToRotations(maxRad);
    limits.ReverseSoftLimitEnable = true;
    limits.ReverseSoftLimitThreshold = Units.radiansToRotations(minRad);
  }

  /**
   * Fills a current-limit configuration group with this mechanism's limit, scaled.
   *
   * @param limits the configuration group to fill
   * @param scalar multiplier in {@code [0, 1]} applied to the limit
   *     <p><b>Outputs:</b> mutates {@code limits} in place.
   */
  private static void applyCurrentLimits(CurrentLimitsConfigs limits, double scalar) {
    // Stator only, unlike the flywheel's two-stage supply limit. A turret's draw is a short burst
    // while slewing and almost nothing while holding, so there is no sustained-draw phase for a
    // supply limit to protect against -- and a supply limit low enough to matter would throttle
    // the one part of the motion that has to be fast.
    limits.StatorCurrentLimit = Constants.Turret.kStatorLimitAmps * scalar;
    limits.StatorCurrentLimitEnable = true;
  }
}
