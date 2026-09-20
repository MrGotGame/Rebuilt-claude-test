package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.optimizeBus;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

/** Real-hardware flywheel: two Kraken X60s on the RIO bus in leader/follower. */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leader;
  private final TalonFX follower;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  /**
   * The follower linkage, held as a field rather than constructed per call.
   *
   * <p>This request is re-issued on every output command (see {@link #setVelocity(double)}). A
   * {@code Follower} request is not persistent across a device reset: a follower that briefly
   * browns out or is power-cycled comes back with no control request at all and simply coasts for
   * the rest of the match. The only symptom is that shots get weak, because the leader silently
   * draws double current to make up the difference.
   */
  private final Follower followerRequest =
      new Follower(Constants.Can.kFlywheelLeaderId, Constants.Shooter.kFollowerOpposesLeader);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> leaderVolts;
  private final StatusSignal<Voltage> followerVolts;
  private final StatusSignal<Current> leaderStator;
  private final StatusSignal<Current> followerStator;
  private final StatusSignal<Current> leaderSupply;
  private final StatusSignal<Current> followerSupply;
  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<Temperature> followerTemp;
  private final Debouncer leaderDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private double goalRpm = 0.0;

  /** Last current-limit scalar actually pushed to hardware, for the 5% change guard. */
  private double lastAppliedCurrentScalar = 1.0;

  /**
   * Constructs both Talons and the status signals.
   *
   * <p>NOTE: device and signal construction is pre-filled because these fields are final. Your work
   * is the configuration and the follower linkage, in the TODO.
   */
  public ShooterIOTalonFX() {
    leader = new TalonFX(Constants.Can.kFlywheelLeaderId, Constants.Can.kRioCanBus);
    follower = new TalonFX(Constants.Can.kFlywheelFollowerId, Constants.Can.kRioCanBus);
    velocity = leader.getVelocity();
    leaderVolts = leader.getMotorVoltage();
    followerVolts = follower.getMotorVoltage();
    leaderStator = leader.getStatorCurrent();
    followerStator = follower.getStatorCurrent();
    leaderSupply = leader.getSupplyCurrent();
    followerSupply = follower.getSupplyCurrent();
    leaderTemp = leader.getDeviceTemp();
    followerTemp = follower.getDeviceTemp();

    // IMPLEMENTATION (was TODO: "Configure the LEADER via PhoenixUtil.tryUntilOk with
    // SensorToMechanismRatio, Slot0, Coast, current limits and inversion; then link the follower;
    // then register signals at 100 Hz and optimize the bus.")
    //
    // SensorToMechanismRatio is the single most load-bearing line here. With it set, every velocity
    // the Talon reports and every velocity we command is at the FLYWHEEL shaft, not the rotor. That
    // is what lets the rest of the project speak in flywheel RPM and never think about the gearbox
    // again -- and it is why the RPM/rot-per-sec conversion appears in this file and nowhere else
    // (plan section 5.4).
    var leaderConfig = new TalonFXConfiguration();
    leaderConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.kGearRatio;
    leaderConfig.Slot0 = currentGains();

    // COAST, always. Braking a flywheel throws away the stored energy that makes the next shot
    // fast: the wheel is the only place in this robot where coasting down slowly is the desirable
    // behaviour. Shooter.SystemState.IDLE exists for the same reason.
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderConfig.MotorOutput.Inverted =
        Constants.Shooter.kLeaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    applyCurrentLimits(leaderConfig.CurrentLimits, 1.0);
    tryUntilOk(5, () -> leader.getConfigurator().apply(leaderConfig, 0.25));

    // The follower gets the same mechanical and electrical configuration but NOT the gains: a
    // follower mirrors the leader's output directly and never runs a control loop of its own, so
    // Slot0 on it would be dead configuration that looks meaningful in Tuner X.
    //
    // Its Inverted value is deliberately left at the default rather than mirrored from
    // kLeaderInverted. Opposition between the two motors is expressed in exactly one place, the
    // Follower request's opposeMasterDirection flag. Expressing it twice is how a pair ends up
    // fighting itself at full current with no error anywhere.
    var followerConfig = new TalonFXConfiguration();
    followerConfig.Feedback.SensorToMechanismRatio = Constants.Shooter.kGearRatio;
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    applyCurrentLimits(followerConfig.CurrentLimits, 1.0);
    tryUntilOk(5, () -> follower.getConfigurator().apply(followerConfig, 0.25));

    follower.setControl(followerRequest);

    // 100 Hz on everything the control loop and the health checks read, which is twice the 50 Hz
    // robot loop so a sample is never older than one loop period. Temperature is the exception at
    // 4 Hz: a motor takes tens of seconds to heat up, and nine devices share the roboRIO bus, so
    // paying 100 Hz for it would spend bus budget the drivetrain handoff needs.
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, velocity, leaderVolts, followerVolts, leaderStator, followerStator, leaderSupply,
        followerSupply);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, leaderTemp, followerTemp);

    // Must run LAST. optimizeBus disables every status frame nobody asked for, so the
    // setUpdateFrequencyForAll calls above are what "asking" means. Reversing these two silently
    // turns off the signals we just requested.
    optimizeBus(leader, follower);
  }

  /**
   * Samples both motors.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "refreshAll, then fill every field ... Use the two SEPARATE
    // Debouncers so leaderConnected and followerConnected are genuinely independent.")
    //
    // Three refresh calls rather than one, because the STATUS CODE is the measurement here. One
    // batched refresh over all nine signals would return a single status, and a dead follower would
    // then read as "the whole shooter is disconnected" -- which is exactly the conflation this IO
    // exists to prevent. Splitting by motor is what makes the two connection booleans independent.
    //
    // Temperature is refreshed separately and its status deliberately ignored: it updates at 4 Hz,
    // so folding it into either connection group would make that group's health depend on a signal
    // that is stale by design.
    var leaderStatus =
        BaseStatusSignal.refreshAll(velocity, leaderVolts, leaderStator, leaderSupply);
    var followerStatus = BaseStatusSignal.refreshAll(followerVolts, followerStator, followerSupply);
    BaseStatusSignal.refreshAll(leaderTemp, followerTemp);

    // A falling-edge debounce reports a disconnect only after 0.5 s of continuous failure, but
    // reports a reconnect instantly. A single dropped frame on a busy bus is normal and must not
    // raise an alert; half a second of silence is a real fault.
    inputs.leaderConnected = leaderDebounce.calculate(leaderStatus.isOK());
    inputs.followerConnected = followerDebounce.calculate(followerStatus.isOK());

    // SensorToMechanismRatio is configured, so this is already flywheel rotations per second.
    inputs.velocityRpm = velocity.getValueAsDouble() * 60.0;
    inputs.goalRpm = goalRpm;
    inputs.leaderAppliedVolts = leaderVolts.getValueAsDouble();
    inputs.followerAppliedVolts = followerVolts.getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leaderStator.getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerStator.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupply.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupply.getValueAsDouble();
    inputs.leaderTempCelsius = leaderTemp.getValueAsDouble();
    inputs.followerTempCelsius = followerTemp.getValueAsDouble();

    // Signed, not absolute: a positive error means "below the goal, still spinning up" and a
    // negative one means "above it". Shooter.willBeAtSpeedIn depends on being able to tell those
    // apart, and a plot of this value is how you recognise the one-shot-low failure.
    inputs.closedLoopErrorRpm = goalRpm - inputs.velocityRpm;
  }

  /**
   * Commands a closed-loop flywheel speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // IMPLEMENTATION (was TODO: "goalRpm = velocityRpm; leader.setControl(velocityRequest
    // .withVelocity(velocityRpm / 60.0)); AND re-assert the Follower request on the follower.")
    goalRpm = velocityRpm;
    leader.setControl(velocityRequest.withVelocity(velocityRpm / 60.0));
    follower.setControl(followerRequest);
  }

  /**
   * Drives the flywheel open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setOpenLoop(double volts) {
    // IMPLEMENTATION (was TODO: "leader.setControl(voltageRequest.withOutput(volts)), plus the
    // Follower re-assert.")
    //
    // goalRpm is zeroed because there is no closed-loop goal in open loop. Leaving the last goal
    // standing would make closedLoopErrorRpm -- and therefore Shooter.atSpeed() -- report against a
    // setpoint that is no longer being commanded, which would let a characterization run look like
    // a successful spin-up.
    goalRpm = 0.0;
    leader.setControl(voltageRequest.withOutput(volts));
    follower.setControl(followerRequest);
  }

  /**
   * Pushes new closed-loop gains to the leader.
   *
   * @param gains the gain set to apply
   */
  @Override
  public void setGains(Slot0Configs gains) {
    // IMPLEMENTATION (was TODO: "tryUntilOk apply to the LEADER ONLY. The follower mirrors output,
    // not control, so gains on it do nothing.")
    //
    // A bare Slot0Configs is applied rather than a whole TalonFXConfiguration. Phoenix merges a
    // partial configuration group in place; applying a full configuration object here would reset
    // every other group, including the current limits and the neutral mode, for at least one loop.
    tryUntilOk(5, () -> leader.getConfigurator().apply(gains, 0.25));
  }

  /**
   * Scales the configured current limits on both motors.
   *
   * @param scalar multiplier in {@code [0, 1]}
   */
  @Override
  public void setCurrentLimitScalar(double scalar) {
    // IMPLEMENTATION (was TODO: "Scale both the stator and supply limits on BOTH motors. The
    // flywheel is PowerBudget.Priority.HIGH.")
    //
    // The 5% deadband is what makes this safe to call every loop from Shooter.periodic(). Without
    // it, two configuration frames per loop at 50 Hz would be spent re-applying a limit that had
    // not changed, on a bus that already carries nine devices.
    if (Math.abs(scalar - lastAppliedCurrentScalar) < 0.05) {
      return;
    }
    lastAppliedCurrentScalar = scalar;

    var limits = new CurrentLimitsConfigs();
    applyCurrentLimits(limits, scalar);
    tryUntilOk(5, () -> leader.getConfigurator().apply(limits, 0.25));
    tryUntilOk(5, () -> follower.getConfigurator().apply(limits, 0.25));
  }

  /**
   * Sets the neutral behavior of both motors.
   *
   * @param brake true for brake; normally always false
   */
  @Override
  public void setNeutralMode(boolean brake) {
    // IMPLEMENTATION (was TODO: "Apply a MotorOutputConfigs to both motors via tryUntilOk.")
    //
    // Inversion is re-stated on both motors. MotorOutputConfigs is a whole configuration GROUP, so
    // applying one that sets only NeutralMode resets Inverted to its default -- on the leader that
    // would reverse the flywheel, and it is the kind of change that shows up as "the robot shoots
    // backwards now" long after whoever called this has moved on.
    NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    var leaderOut = new MotorOutputConfigs();
    leaderOut.NeutralMode = mode;
    leaderOut.Inverted =
        Constants.Shooter.kLeaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    var followerOut = new MotorOutputConfigs();
    followerOut.NeutralMode = mode;
    followerOut.Inverted = InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> leader.getConfigurator().apply(leaderOut, 0.25));
    tryUntilOk(5, () -> follower.getConfigurator().apply(followerOut, 0.25));
  }

  /** Stops the leader; the follower coasts with it. */
  @Override
  public void stop() {
    // IMPLEMENTATION (was TODO: "leader.stopMotor(). Do not also stop the follower -- it follows.")
    //
    // The follower is left on its Follower request on purpose. Calling stopMotor() on it would
    // replace that request with a neutral one, and the next setVelocity would then be spinning a
    // single motor until the re-assert landed.
    goalRpm = 0.0;
    leader.stopMotor();
  }

  /**
   * Builds a {@link Slot0Configs} from the current values of the tunable shooter gains.
   *
   * <p>Used only to seed the initial configuration. Retuning at runtime goes through {@link
   * #setGains(Slot0Configs)}, driven by {@code Shooter.periodic()}.
   *
   * @return the gain set described by {@code Constants.Shooter}
   */
  private static Slot0Configs currentGains() {
    var gains = new Slot0Configs();
    gains.kP = Constants.Shooter.kKp.get();
    gains.kI = Constants.Shooter.kKi.get();
    gains.kD = Constants.Shooter.kKd.get();
    gains.kS = Constants.Shooter.kKs.get();
    gains.kV = Constants.Shooter.kKv.get();
    gains.kA = Constants.Shooter.kKa.get();
    return gains;
  }

  /**
   * Fills a current-limit configuration group with this mechanism's limits, scaled.
   *
   * <p>One place builds these so that the boot configuration and every later {@link
   * #setCurrentLimitScalar(double)} call cannot drift apart -- a mismatch there would mean the
   * limits silently change the first time the power budget throttles anything.
   *
   * @param limits the configuration group to fill
   * @param scalar multiplier in {@code [0, 1]} applied to both limits
   *     <p><b>Outputs:</b> mutates {@code limits} in place.
   */
  private static void applyCurrentLimits(CurrentLimitsConfigs limits, double scalar) {
    limits.StatorCurrentLimit = Constants.Shooter.kStatorLimitAmps * scalar;
    limits.StatorCurrentLimitEnable = true;

    // Supply limiting is two-stage in Phoenix 6: kSupplyLimitAmps may be drawn for
    // kSupplyLowerTimeSec, after which the limit drops to kSupplyLowerLimitAmps. That shape is
    // exactly what a flywheel wants -- spin-up is a brief, large draw and cruising is a small one
    // -- and a single flat limit would have to be set high enough for spin-up, which then leaves no
    // protection at all during the rest of the match.
    limits.SupplyCurrentLimit = Constants.Shooter.kSupplyLimitAmps * scalar;
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLowerLimit = Constants.Shooter.kSupplyLowerLimitAmps * scalar;
    limits.SupplyCurrentLowerTime = Constants.Shooter.kSupplyLowerTimeSec;
  }
}
