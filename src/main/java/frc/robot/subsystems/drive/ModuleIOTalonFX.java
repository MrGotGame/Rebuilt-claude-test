// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.PowerBudget;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /**
   * Live-tunable drive and steer gains, shared by all four modules.
   *
   * <p>Deliberately {@code static}: a swerve drivetrain wants one gain set applied to every module,
   * so four independent copies would be four chances to tune three of them and forget the fourth.
   * Each module still passes its own {@code hashCode()} as the change-detection id, so each one
   * independently notices the change and pushes it to its own Talon.
   *
   * <p>Defaults are seeded from the CTRE-generated TunerConstants so that, with tuning mode off,
   * behaviour is byte-identical to the stock template.
   */
  private static final LoggedTunableGains driveGains =
      new LoggedTunableGains(
          "Drive/DriveMotor",
          TunerConstants.FrontLeft.DriveMotorGains.kP,
          TunerConstants.FrontLeft.DriveMotorGains.kI,
          TunerConstants.FrontLeft.DriveMotorGains.kD,
          TunerConstants.FrontLeft.DriveMotorGains.kS,
          TunerConstants.FrontLeft.DriveMotorGains.kV,
          TunerConstants.FrontLeft.DriveMotorGains.kA);

  /** Live-tunable steer gains, shared by all four modules. See {@link #driveGains}. */
  private static final LoggedTunableGains turnGains =
      new LoggedTunableGains(
          "Drive/SteerMotor",
          TunerConstants.FrontLeft.SteerMotorGains.kP,
          TunerConstants.FrontLeft.SteerMotorGains.kI,
          TunerConstants.FrontLeft.SteerMotorGains.kD,
          TunerConstants.FrontLeft.SteerMotorGains.kS,
          TunerConstants.FrontLeft.SteerMotorGains.kV,
          TunerConstants.FrontLeft.SteerMotorGains.kA);

  /** Last current-limit scalar actually pushed to hardware, for the 5% change guard. */
  private double lastAppliedCurrentScalar = 1.0;

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;
    driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus);
    turnTalon = new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus);
    cancoder = new CANcoder(constants.EncoderId, TunerConstants.kCANBus);

    // Configure drive motor
    var driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> throw new RuntimeException(
              "You have selected a turn feedback source that is not supported by the default implementation of ModuleIOTalonFX. Please check the AdvantageKit documentation for more information on alternative configurations: https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");
        };
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Create timestamp queue
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition.clone());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Create turn status signals
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnPosition.clone());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
  }

  /**
   * Samples all hardware signals into the input struct.
   *
   * @param inputs the struct to populate; every field is written every call
   */
  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

    // Tier-3 hooks. Both are no-ops on the overwhelming majority of loops -- they only touch the
    // CAN bus when a tunable or the power budget has actually moved -- so running them from the
    // per-loop input path costs effectively nothing.
    updateTunableGains();
    updateCurrentLimit();
  }

  /**
   * Runs the drive motor open loop.
   *
   * @param output commanded output, in volts
   */
  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  /**
   * Runs the turn motor open loop.
   *
   * @param output commanded output, in volts
   */
  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  /**
   * Runs the drive motor at a closed-loop velocity.
   *
   * @param velocityRadPerSec target wheel velocity, in radians per second
   */
  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  /**
   * Turns the module azimuth to a closed-loop position.
   *
   * @param rotation target azimuth angle
   */
  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
              rotation.getRotations());
        });
  }

  // ------------------------------------------------------------ tier-3 additions start here ----

  /**
   * Sets brake or coast on both of this module's motors.
   *
   * @param brake true for brake, false for coast
   *     <p><b>Outputs:</b> reconfigures the neutral mode of both Talons.
   */
  @Override
  public void setBrakeMode(boolean brake) {
    // IMPLEMENTATION (was TODO: "Build a MotorOutputConfigs with NeutralModeValue.Brake/Coast and
    // apply it to both driveTalon and turnTalon via tryUntilOk. Do NOT re-apply the whole
    // TalonFXConfiguration here; that drops the Slot0 gains for a loop.")
    //
    // A bare MotorOutputConfigs is applied rather than a full TalonFXConfiguration, exactly as the
    // TODO warns. Applying a whole configuration object overwrites EVERY config group, including
    // Slot0 -- so for one loop the module would run with default gains and visibly lurch. Phoenix
    // merges a partial config group in place, leaving everything else untouched.
    NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Inversion must be carried along. MotorOutputConfigs is a whole config GROUP, so applying one
    // that only sets NeutralMode would reset Inverted to its default and flip the module.
    var driveOut = new MotorOutputConfigs();
    driveOut.NeutralMode = mode;
    driveOut.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    var turnOut = new MotorOutputConfigs();
    turnOut.NeutralMode = mode;
    turnOut.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveOut, 0.25));
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnOut, 0.25));
  }

  /**
   * Re-applies swerve gains when a tunable changes, so gains can be retuned without a redeploy.
   *
   * <p><b>Outputs:</b> may reconfigure both Talons. Call at the end of {@code updateInputs}.
   */
  private void updateTunableGains() {
    // IMPLEMENTATION (was TODO: "driveGains.ifChanged(hashCode(), cfg -> tryUntilOk(5, () ->
    // driveTalon.getConfigurator().apply(cfg, 0.25))) and the same for steer.")
    //
    // hashCode() identifies THIS module to the shared tunable. Because LoggedTunableNumber reports
    // a change on the first call for a given id, all four modules push their gains once on the
    // first loop after boot and then stay quiet until someone edits a value.
    driveGains.ifChanged(
        hashCode(), cfg -> tryUntilOk(5, () -> driveTalon.getConfigurator().apply(cfg, 0.25)));
    turnGains.ifChanged(
        hashCode(), cfg -> tryUntilOk(5, () -> turnTalon.getConfigurator().apply(cfg, 0.25)));
  }

  /**
   * Scales the stator current limit from the power budget.
   *
   * <p><b>Outputs:</b> may reconfigure both Talons.
   */
  private void updateCurrentLimit() {
    // IMPLEMENTATION (was TODO: "double scalar = PowerBudget.getInstance().getScalar(
    // Priority.CRITICAL). If it differs from the last applied value by >5%, apply a
    // CurrentLimitsConfigs with StatorCurrentLimit = constants.SlipCurrent * scalar.")
    //
    // The drivetrain is CRITICAL priority, so in practice this scalar stays at 1.0 and this method
    // never touches the bus -- the drivetrain is the last thing to be throttled, because a robot
    // that cannot drive cannot even leave the field. The plumbing exists so that if the budget ever
    // does decide to protect the battery, the limit follows.
    double scalar = PowerBudget.getInstance().getScalar(PowerBudget.Priority.CRITICAL);

    // The 5% deadband is the whole point. Without it this would re-apply a configuration every
    // single loop at 50 Hz on four modules -- 200 config frames per second -- which measurably
    // degrades bus utilisation and shows up as erratic odometry.
    if (Math.abs(scalar - lastAppliedCurrentScalar) < 0.05) {
      return;
    }
    lastAppliedCurrentScalar = scalar;

    var limits = new CurrentLimitsConfigs();
    limits.StatorCurrentLimit = constants.SlipCurrent * scalar;
    limits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(limits, 0.25));
  }
}
