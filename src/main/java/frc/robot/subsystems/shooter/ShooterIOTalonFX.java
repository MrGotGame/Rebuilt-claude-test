package frc.robot.subsystems.shooter;

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

/** Real-hardware flywheel: two Kraken X60s on the RIO bus in leader/follower. */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leader;
  private final TalonFX follower;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
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

    // TODO: Configure the LEADER via PhoenixUtil.tryUntilOk with:
    //   - Feedback.SensorToMechanismRatio = Constants.Shooter.kGearRatio
    //   - Slot0 from the Constants.Shooter gains
    //   - MotorOutput NeutralMode = COAST. Never brake a flywheel.
    //   - CurrentLimits: stator kStatorLimitAmps, supply kSupplyLimitAmps with a supply-time
    //     threshold so spin-up is allowed to draw hard briefly
    //   - Inverted per kLeaderInverted
    // TODO: Then link the follower:
    //   follower.setControl(new Follower(Constants.Can.kFlywheelLeaderId,
    //       Constants.Shooter.kFollowerOpposesLeader));
    // TODO: Register signals at 100 Hz and PhoenixUtil.optimizeBus(leader, follower).
  }

  /**
   * Samples both motors.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // TODO: refreshAll, then fill every field. velocityRpm = velocity.getValueAsDouble() * 60, and
    // closedLoopErrorRpm = goalRpm - velocityRpm. Use the two SEPARATE Debouncers so
    // leaderConnected and followerConnected are genuinely independent -- that independence is the
    // whole reason this IO logs both motors.
  }

  /**
   * Commands a closed-loop flywheel speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: goalRpm = velocityRpm;
    // leader.setControl(velocityRequest.withVelocity(velocityRpm / 60.0));
    // AND re-assert the Follower request on the follower. Re-asserting every call is deliberate: a
    // follower that briefly power-cycles otherwise sits idle for the rest of the match, and the
    // only symptom is that shots get weak.
  }

  /**
   * Drives the flywheel open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setOpenLoop(double volts) {
    // TODO: leader.setControl(voltageRequest.withOutput(volts)), plus the Follower re-assert.
  }

  /**
   * Pushes new closed-loop gains to the leader.
   *
   * @param gains the gain set to apply
   */
  @Override
  public void setGains(Slot0Configs gains) {
    // TODO: tryUntilOk apply to the LEADER ONLY. The follower mirrors output, not control, so gains
    // on it do nothing.
  }

  /**
   * Scales the configured current limits on both motors.
   *
   * @param scalar multiplier in {@code [0, 1]}
   */
  @Override
  public void setCurrentLimitScalar(double scalar) {
    // TODO: Scale both the stator and supply limits on BOTH motors. The flywheel is
    // PowerBudget.Priority.HIGH.
  }

  /**
   * Sets the neutral behavior of both motors.
   *
   * @param brake true for brake; normally always false
   */
  @Override
  public void setNeutralMode(boolean brake) {
    // TODO: Apply a MotorOutputConfigs to both motors via tryUntilOk.
  }

  /** Stops the leader; the follower coasts with it. */
  @Override
  public void stop() {
    // TODO: leader.stopMotor(). Do not also stop the follower -- it follows.
  }
}
