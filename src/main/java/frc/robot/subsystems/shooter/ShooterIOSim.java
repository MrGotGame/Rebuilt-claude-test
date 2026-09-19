package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Simulated flywheel: a {@link FlywheelSim} driven by a voltage feedforward plus a PID term. */
public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim sim;
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;
  private double goalRpm = 0.0;

  /** Builds the flywheel model: two motors driving one shared inertia. NOTE: pre-filled, final. */
  public ShooterIOSim() {
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(2),
                Constants.Shooter.kMoiKgM2,
                Constants.Shooter.kGearRatio),
            DCMotor.getKrakenX60Foc(2));
  }

  /**
   * Advances the model one loop period and fills the input struct.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // TODO: appliedVolts = ffVolts + (closedLoop ?
    // controller.calculate(sim.getAngularVelocityRPM())
    // : 0.0). Clamp to +-12 V, sim.setInputVoltage, sim.update(Constants.getLoopPeriodSec()).
    // Then mirror the same values into BOTH the leader and follower input fields, so a sim log has
    // exactly the same shape as a real log and the same AdvantageScope layout works on both.
  }

  /**
   * Commands a closed-loop flywheel speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: closedLoop = true; goalRpm = velocityRpm;
    // ffVolts = kKs.get() * Math.signum(velocityRpm) + kKv.get() * velocityRpm / 60.0;
    // controller.setSetpoint(velocityRpm).
  }

  /**
   * Drives the flywheel open loop.
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
    // TODO: closedLoop = false; appliedVolts = 0; ffVolts = 0; goalRpm = 0.
  }
}
