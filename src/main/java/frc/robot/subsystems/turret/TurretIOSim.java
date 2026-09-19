package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Simulated turret.
 *
 * <p>The important detail is that this sim <b>reproduces the hard stops</b>. Without them the
 * unwrap logic is never exercised before the code reaches the real robot, and the first time anyone
 * finds out that the turret takes the long way around is when a belt jumps.
 */
public class TurretIOSim implements TurretIO {
  private final DCMotorSim sim;
  private final ProfiledPIDController controller;
  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;
  private double minRad = -Math.PI;
  private double maxRad = Math.PI;

  /**
   * Builds the motor model and the stand-in profiled controller. NOTE: pre-filled, final fields.
   */
  public TurretIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                // TODO: See the note in IntakeIOSim about the Kraken X44 DCMotor factory. The X44
                // is
                // materially weaker than the X60 modelled here.
                DCMotor.getKrakenX60Foc(1), Constants.Turret.kMoiKgM2, Constants.Turret.kGearRatio),
            DCMotor.getKrakenX60Foc(1));
    controller =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(12.0, 60.0));
    // TODO: Seed the gains and constraints from Constants.Turret rather than the literals above.
  }

  /**
   * Advances the model one loop period, enforces the hard stops, and fills the input struct.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // TODO: When closedLoop, appliedVolts = ffVolts + controller.calculate(sim.getAngularPosition
    // Rad()). Clamp to +-12 V, sim.setInputVoltage, sim.update(Constants.getLoopPeriodSec()).
    // THEN clamp the simulated position into [minRad, maxRad] with sim.setState(clamped, 0.0) when
    // it would exceed a limit -- zeroing velocity at the stop is what makes the sim behave like a
    // real hard stop instead of an infinite-travel azimuth.
    // Fill every field; set motorConnected and absEncoderConnected true, and absolutePositionRad
    // equal to the wrapped positionRad.
  }

  /**
   * Commands a position with velocity feedforward.
   *
   * @param positionRad continuous target position, in radians
   * @param velocityFeedforwardRadPerSec target angular rate, in radians per second
   */
  @Override
  public void setPositionSetpoint(double positionRad, double velocityFeedforwardRadPerSec) {
    // TODO: closedLoop = true; controller.setGoal(positionRad);
    // ffVolts = Constants.Turret.kKv.get() * velocityFeedforwardRadPerSec.
  }

  /**
   * Drives the turret open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setOpenLoop(double volts) {
    // TODO: closedLoop = false; appliedVolts = volts.
  }

  /**
   * Teleports the simulated turret to a known angle.
   *
   * @param positionRad the angle to assume, in radians
   */
  @Override
  public void seedPosition(double positionRad) {
    // TODO: sim.setState(positionRad, 0.0).
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

  /**
   * Stores the travel limits used by the clamp in {@link #updateInputs}.
   *
   * @param minRad lower travel limit, in radians
   * @param maxRad upper travel limit, in radians
   */
  @Override
  public void setSoftLimits(double minRad, double maxRad) {
    // TODO: this.minRad = minRad; this.maxRad = maxRad.
  }

  /** Zeroes the applied voltage and leaves closed loop. */
  @Override
  public void stop() {
    // TODO: closedLoop = false; appliedVolts = 0; ffVolts = 0.
  }
}
