package frc.robot.subsystems.indexer;

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
   * Last current-limit scalar actually pushed to hardware, for the 5% change guard in {@link
   * #setCurrentLimitScalar(double)}.
   */
  private double lastAppliedCurrentScalar = 1.0;

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

    // IMPLEMENTATION (was TODO: "Same configuration pattern as FloorRollerIOTalonFX, using
    // Constants.Indexer values, but set MotorOutput NeutralMode = BRAKE so a held piece does not
    // roll back out when disabled." and "Set signal update frequency to 50 Hz,
    // PhoenixUtil.registerSignals(...), then PhoenixUtil.optimizeBus(motor).")
    //
    // SensorToMechanismRatio is the load-bearing line, exactly as in FloorRollerIOTalonFX and
    // ShooterIOTalonFX. With it set, every velocity the Talon reports and every velocity commanded
    // is at the INDEXER shaft rather than the rotor, which confines the RPM/rot-per-sec conversion
    // to this one file (plan section 5.4) and lets the state machine speak only in indexer RPM.
    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = Constants.Indexer.kGearRatio;
    config.Slot0 = currentGains();

    // BRAKE, and this is the one configuration line where the indexer deliberately differs from the
    // floor roller, which coasts. This mechanism is the last thing holding a game piece before the
    // flywheel, often on a slight downhill into the shooter, and the failure it prevents is a piece
    // walking out of the robot between auto and teleop, or during the pause after a match-ending
    // disable, when nothing is driving the motor at all. The cost is that a jammed piece cannot be
    // pulled out by hand without back-driving the gearbox; that is the right trade for a mechanism
    // that is supposed to retain exactly one piece.
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        Constants.Indexer.kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    applyCurrentLimits(config.CurrentLimits, 1.0);
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // 50 Hz on everything the state machine reads, matching the floor roller: one fresh sample per
    // robot loop is all any consumer can use, and nine devices share the roboRIO bus. Stator
    // current is in that set rather than at a lower rate because it is half of the piece-detection
    // predicate when no beam break is installed -- a stale current sample there would delay the
    // INDEXING -> HOLDING transition and let the indexer keep pulling on a piece it already has.
    //
    // Temperature is the exception at 4 Hz: a motor takes tens of seconds to heat up, so paying
    // more for it is bus budget spent on a number that cannot change between frames.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, statorCurrent, supplyCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, temp);

    // Registering puts these signals in the shared batch that PhoenixUtil.refreshAll() will refresh
    // once per robotPeriodic, so that when Robot.robotPeriodic is filled in every mechanism's
    // measurements carry the same timestamp. This IO still refreshes locally in updateInputs as
    // well -- see the comment there for why that is not redundant.
    registerSignals(position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);

    // Must run LAST. optimizeBus disables every status frame nobody explicitly asked for, so the
    // setUpdateFrequencyForAll calls above are what "asking" means. Reversing these two silently
    // turns off the signals just requested, and the symptom is inputs frozen at zero.
    optimizeBus(motor);
  }

  /**
   * Samples the motor and the beam break.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "refreshAll on the six signals, feed isOK() through
    // connectedDebounce, fill every field. For the beam break: guard on beamBreak != null, then
    // inputs.beamBreakTripped = !beamBreak.get().")
    //
    // The local refresh is kept even though these signals are also in PhoenixUtil's shared batch,
    // for two reasons. First, the StatusCode is the measurement that drives inputs.connected: the
    // shared batch returns ONE status covering every device on the bus, so using it here would make
    // a dead device anywhere in the robot report the indexer as disconnected. Second,
    // Robot.robotPeriodic does not call PhoenixUtil.refreshAll() yet -- it is still a TODO -- and
    // an IO that depended on it would silently report zeros for every field with nothing flagging
    // the fault. Refreshing twice costs nothing on the bus: refresh reads values the Phoenix driver
    // has already received, and bus traffic is governed by the update frequencies set above.
    //
    // Temperature is refreshed separately and its status deliberately ignored, because at 4 Hz it
    // is stale by design and folding it into the connection check would make a healthy motor look
    // intermittently disconnected.
    var status =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, statorCurrent, supplyCurrent);
    BaseStatusSignal.refreshAll(temp);

    // A falling-edge debounce reports a disconnect only after 0.5 s of continuous failure but
    // reports a reconnect instantly. A single dropped frame on a busy bus is normal and must not
    // raise an alert; half a second of silence is a real fault.
    inputs.connected = connectedDebounce.calculate(status.isOK());

    // SensorToMechanismRatio is configured, so both of these are already at the indexer shaft.
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRpm = velocity.getValueAsDouble() * 60.0;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();

    if (beamBreak == null) {
      // Both fields are written on every path, per the IndexerIO contract. Reporting tripped =
      // false here rather than leaving the previous value is what makes Indexer fall through to the
      // current-and-velocity path instead of latching onto a stale "piece present".
      inputs.beamBreakConnected = false;
      inputs.beamBreakTripped = false;
    } else {
      // beamBreakConnected means "configured and constructed", NOT "electrically healthy". A
      // three-wire beam break whose signal lead falls off reads as a constant level, and no DIO
      // channel can tell that apart from a sensor honestly reporting that level, so this flag
      // cannot be made to mean more than it does. What it does buy is the case that actually
      // happens: kHasBeamBreak flipped true in code while no sensor is installed, which Indexer
      // turns into an alert instead of a robot that silently believes it is always loaded.
      inputs.beamBreakConnected = true;

      // TODO: VERIFY your sensor's polarity on the bench before trusting this inversion. Most beam
      // breaks are active-low -- the output is pulled low while the beam is BROKEN, so a piece
      // present reads false at the DIO pin and this negation turns it into "tripped" -- but not all
      // are, and getting it backwards means the robot believes it always has a piece, which pins
      // hasPiece() true and opens every scoring gate on an empty indexer. Check it by putting a
      // piece in the indexer with the robot enabled and reading Indexer/Inputs/beamBreakTripped.
      inputs.beamBreakTripped = !beamBreak.get();
    }
  }

  /**
   * Commands a closed-loop indexer speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // IMPLEMENTATION (was TODO: "motor.setControl(velocityRequest.withVelocity(velocityRpm /
    // 60.0)).")
    //
    // Negative values are passed straight through rather than clamped: EJECT is a negative RPM
    // goal. A zero request is passed through as well, and is meaningful here in a way it is not on
    // the floor roller -- Indexer's HOLDING state commands a closed-loop zero so that the P term
    // actively resists a piece trying to roll out, which is weaker than a stall but stronger than
    // the brake alone.
    motor.setControl(velocityRequest.withVelocity(velocityRpm / 60.0));
  }

  /**
   * Drives the indexer open loop.
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
    // every other group -- current limits and the brake neutral mode included -- for at least one
    // loop, every time somebody nudged a gain on the dashboard. Losing brake mode for a loop while
    // holding a piece is exactly the failure this mechanism exists to prevent.
    tryUntilOk(5, () -> motor.getConfigurator().apply(gains, 0.25));
  }

  /**
   * Scales the configured current limit.
   *
   * @param scalar multiplier in {@code [0, 1]}
   */
  @Override
  public void setCurrentLimitScalar(double scalar) {
    // IMPLEMENTATION (was TODO: "Scale kStatorLimitAmps. The indexer is PowerBudget.Priority
    // .MEDIUM.")
    //
    // MEDIUM means this scalar moves after the intake and floor roller have already given up amps
    // but before the flywheel and turret do. The ordering is deliberate: a robot that cannot pick a
    // piece up can still shoot the one it is holding, so the indexer keeps its current until the
    // pickup mechanisms have already been throttled.
    //
    // The 5% deadband is what makes this safe to call every loop from Indexer.periodic(). Without
    // it a sagging battery would put a configuration frame on the bus every 20 ms, precisely when
    // the bus is busiest.
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
    // stopMotor() issues a neutral request, which with NeutralModeValue.Brake configured above
    // leaves the indexer braked rather than coasting. That is the desired behavior here and the
    // reason Indexer's IDLE state can use it: a piece resting in a braked indexer stays there.
    // Deliberately NOT setVelocity(0.0) -- that is a closed-loop hold, which Indexer uses for its
    // separate HOLDING state when it wants the P term actively fighting a piece that is moving.
    motor.stopMotor();
  }

  /**
   * Builds a {@link Slot0Configs} from the current values of the tunable indexer gains.
   *
   * <p>Used only to seed the initial configuration. Retuning at runtime goes through {@link
   * #setGains(Slot0Configs)}, driven by {@code Indexer.periodic()}.
   *
   * @return the gain set described by {@code Constants.Indexer}
   */
  private static Slot0Configs currentGains() {
    // kP and kV only. Constants.Indexer carries no kS, kI or kD, and none is invented here: an
    // integrator winds up while a game piece is stalling the indexer and then dumps that
    // accumulated output the moment the piece clears, which is how a feed turns into a throw, and a
    // derivative term amplifies the velocity quantisation a single-encoder mechanism reports.
    //
    // The missing kS is the one worth flagging. A static-friction term would help the indexer break
    // away from rest with a piece loaded, and if a measured one is ever wanted, add kKs to
    // Constants.Indexer and set it here with Math.signum applied by the caller -- the floor roller
    // does exactly that, because EJECT commands a negative RPM and an unsigned kS would push
    // against the motion being asked for.
    var gains = new Slot0Configs();
    gains.kP = Constants.Indexer.kKp.get();
    gains.kV = Constants.Indexer.kKv.get();
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
    limits.StatorCurrentLimit = Constants.Indexer.kStatorLimitAmps * scalar;
    limits.StatorCurrentLimitEnable = true;

    // Stator only, as on the floor roller. Stator limiting already bounds the torque this indexer
    // can put into a jammed piece, and Constants.Indexer carries no supply figure; inventing one
    // here would be a guessed electrical limit with nothing to check it against. Add it alongside a
    // measured number if the PDH channel ever trips.
    //
    // Note that the stator limit is also what keeps current-spike piece detection meaningful. If
    // this limit were ever raised far above kHasPieceCurrentAmps the spike would still be detected,
    // but the indexer would be free to keep pulling hard on a piece it has already detected; the
    // two numbers should be retuned together.
  }
}
