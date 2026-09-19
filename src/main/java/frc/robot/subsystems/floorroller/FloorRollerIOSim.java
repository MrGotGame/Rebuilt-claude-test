package frc.robot.subsystems.floorroller;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Simulated floor roller: a {@link DCMotorSim} driven by a voltage feedforward plus a PID term. */
public class FloorRollerIOSim implements FloorRollerIO {
  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;

  /** Builds the motor model. NOTE: pre-filled because {@code sim} is final. */
  public FloorRollerIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, Constants.FloorRoller.kGearRatio),
            DCMotor.getKrakenX60Foc(1));
    // TODO: The 0.001 kg*m^2 above is a placeholder inertia. Replace it with the real roller
    // inertia from CAD if you want spin-up time in sim to match the robot.
  }

  /**
   * Advances the model one loop period and fills the input struct.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(FloorRollerIOInputs inputs) {
    // TODO: When closedLoop, appliedVolts = ffVolts + controller.calculate(sim.getAngularVelocity
    // RadPerSec()). Clamp to +-12 V, sim.setInputVoltage(appliedVolts), sim.update(
    // Constants.getLoopPeriodSec()), then fill every field with connected = true.
  }

  /**
   * Commands a closed-loop roller speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: closedLoop = true; ffVolts = kKs * Math.signum(rpm) + kKv * rpm / 60.0;
    // controller.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm)).
  }

  /**
   * Drives the roller open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setOpenLoop(double volts) {
    // TODO: closedLoop = false; appliedVolts = volts.
  }

  /**
   * Applies gains to the stand-in controller.
   *
   * @param gains the gain set; only kP, kI and kD are meaningful in simulation
   */
  @Override
  public void setGains(Slot0Configs gains) {
    // TODO: controller.setPID(gains.kP, gains.kI, gains.kD).
  }

  /** Zeroes the applied voltage and leaves closed loop. */
  @Override
  public void stop() {
    // TODO: closedLoop = false; appliedVolts = 0; ffVolts = 0.
  }
}
