package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/**
 * Physics simulation of the intake, good enough to develop and debug the state machine with no
 * hardware present.
 *
 * <p>The pivot is a {@link SingleJointedArmSim} so that gravity, the hard stops, and the gravity
 * feedforward all behave the way they will on the real robot. The roller is a {@link DCMotorSim}.
 * Both are driven by a WPILib {@link ProfiledPIDController} standing in for the Talon's Motion
 * Magic.
 */
public class IntakeIOSim implements IntakeIO {
  private final SingleJointedArmSim pivotSim;
  private final DCMotorSim rollerSim;
  private final ProfiledPIDController pivotController;
  private boolean pivotClosedLoop = false;
  private double pivotAppliedVolts = 0.0;
  private double rollerAppliedVolts = 0.0;
  private double rollerSetpointRpm = 0.0;

  /**
   * Builds the two simulation models and the stand-in profiled controller.
   *
   * <p>NOTE: these fields are final, so construction is pre-filled. The angle bounds below are
   * deliberate placeholders rather than the Constants values, because every Intake angle constant
   * currently returns 0.0 and {@link SingleJointedArmSim} rejects a zero-width range.
   */
  public IntakeIOSim() {
    // TODO: Once kStowedAngleRad and kDeployedAngleRad are MEASURED, replace the two literals below
    // with Constants.Intake.kStowedAngleRad.get() and kDeployedAngleRad.get() so the sim enforces
    // the same travel as the real mechanism.
    pivotSim =
        new SingleJointedArmSim(
            // TODO: VERIFY whether WPILib 2026 exposes a getKrakenX44() factory. If it does, use
            // it.
            // If not, build a DCMotor from the X44 datasheet stall torque and free speed rather
            // than
            // leaving the X60 model here -- the X44 is a materially weaker motor and the sim will
            // otherwise flatter your gains.
            DCMotor.getKrakenX60(1),
            Constants.Intake.kPivotGearRatio,
            Constants.Intake.kPivotMoiKgM2,
            Constants.Intake.kPivotLengthMeters,
            -Math.PI / 2.0,
            Math.PI / 2.0,
            true,
            0.0);
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, Constants.FloorRoller.kGearRatio),
            DCMotor.getKrakenX60Foc(1));
    pivotController =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(8.0, 30.0));
    // TODO: Seed the controller constraints from kPivotCruiseRadPerSec / kPivotAccelRadPerSec2
    // rather than the literals above, so tuning the real robot also tunes the sim.
  }

  /**
   * Advances both simulation models one loop period and fills the input struct.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO: When pivotClosedLoop, run pivotController.calculate(pivotSim.getAngleRads()) and use it
    // as pivotAppliedVolts. Clamp both applied voltages to +-12 V, call setInputVoltage on each sim
    // and update(Constants.getLoopPeriodSec()), then fill EVERY input field. Set pivotConnected,
    // rollerConnected and absEncoderConnected to true, and set pivotAbsolutePositionRad equal to
    // pivotPositionRad -- the simulated encoder is perfect, which is exactly why sim will not catch
    // a bad kPivotAbsEncoderOffsetRot.
  }

  /**
   * Commands the simulated pivot to an angle.
   *
   * @param positionRad target angle, in radians
   */
  @Override
  public void setPivotPosition(double positionRad) {
    // TODO: pivotClosedLoop = true; pivotController.setGoal(positionRad).
  }

  /**
   * Drives the simulated pivot open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setPivotOpenLoop(double volts) {
    // TODO: pivotClosedLoop = false; pivotAppliedVolts = volts.
  }

  /**
   * Commands the simulated roller to a speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setRollerVelocity(double velocityRpm) {
    // TODO: Store the setpoint, then drive rollerAppliedVolts with a simple kV feedforward plus a
    // proportional term on the RPM error.
  }

  /**
   * Drives the simulated roller open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setRollerOpenLoop(double volts) {
    // TODO: rollerAppliedVolts = volts.
  }

  /**
   * Teleports the simulated pivot to a known angle.
   *
   * @param positionRad the angle to assume, in radians
   */
  @Override
  public void seedPivotPosition(double positionRad) {
    // TODO: pivotSim.setState(positionRad, 0.0).
  }

  /**
   * Applies gains to the stand-in controller.
   *
   * @param gains the gain set; only kP, kI and kD are meaningful in simulation
   */
  @Override
  public void setPivotGains(Slot0Configs gains) {
    // TODO: pivotController.setPID(gains.kP, gains.kI, gains.kD). Note that sim gains are NOT real
    // gains: this controller is a WPILib profiled PID in volts, while the robot runs Motion Magic
    // on the Talon. Tuning kP here does not transfer.
  }

  /** Zeroes both applied voltages and leaves closed loop. */
  @Override
  public void stop() {
    // TODO: pivotAppliedVolts = 0; rollerAppliedVolts = 0; pivotClosedLoop = false.
  }
}
