package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
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
    // IMPLEMENTATION (was TODO: "appliedVolts = ffVolts + (closedLoop ? controller.calculate(...)
    // : 0.0). Clamp to +-12 V, sim.setInputVoltage, sim.update(Constants.getLoopPeriodSec()). Then
    // mirror the same values into BOTH the leader and follower input fields.")
    //
    // DEVIATION FROM THE TODO, deliberate: the TODO says to run the controller on RPM
    // (`controller.calculate(sim.getAngularVelocityRPM())`), but the feedforward in setVelocity is
    // written in rotations per second, and Slot0Configs.kP on the real Talon is volts per
    // rotation-per-second of error. Running the simulated loop on RPM would make the same numeric
    // kP sixty times stronger in simulation than on hardware, so a gain tuned in sim would send the
    // real flywheel into violent oscillation. Everything below is therefore in rot/s, and the
    // RPM conversion happens only at the input-struct boundary.
    if (closedLoop) {
      appliedVolts = ffVolts + controller.calculate(sim.getAngularVelocityRPM() / 60.0);
    } else {
      // Resetting while open loop clears the integrator. Without this, a characterization run
      // followed by a closed-loop request would start with whatever integral had accumulated before
      // the mode switch and deliver a step to the wheel.
      controller.reset();
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.getLoopPeriodSec());

    double velocityRpm = sim.getAngularVelocityRPM();

    // Both motors are reported, with the same values, so that a simulation log has exactly the same
    // shape as a real log and one AdvantageScope layout works on both. It also means the follower
    // health check has something to read in sim rather than a permanently-false field that would
    // look like a dead motor.
    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.velocityRpm = velocityRpm;
    inputs.goalRpm = goalRpm;
    inputs.leaderAppliedVolts = appliedVolts;
    inputs.followerAppliedVolts = appliedVolts;

    // FlywheelSim models the pair as one plant, so getCurrentDrawAmps() is the current for BOTH
    // motors. Halving it is what makes the per-motor numbers comparable with the real IO, where
    // each Talon reports only its own draw.
    double perMotorStatorAmps = Math.abs(sim.getCurrentDrawAmps()) / 2.0;
    inputs.leaderStatorCurrentAmps = perMotorStatorAmps;
    inputs.followerStatorCurrentAmps = perMotorStatorAmps;

    // A motor controller is a buck converter: supply current is roughly stator current scaled by
    // duty cycle. The approximation is crude, but reporting supply == stator would overstate the
    // battery load by up to 6x at low speed and make any sim-side power budgeting meaningless.
    double dutyCycle = Math.abs(MathUtil.clamp(appliedVolts, -12.0, 12.0)) / 12.0;
    inputs.leaderSupplyCurrentAmps = perMotorStatorAmps * dutyCycle;
    inputs.followerSupplyCurrentAmps = perMotorStatorAmps * dutyCycle;

    // FlywheelSim has no thermal model. A fixed room temperature is reported rather than 0.0 so
    // that a plot of this field in simulation is obviously constant rather than looking like a
    // sensor reading zero -- which is what a genuinely disconnected Talon reports.
    inputs.leaderTempCelsius = 25.0;
    inputs.followerTempCelsius = 25.0;

    inputs.closedLoopErrorRpm = goalRpm - velocityRpm;
  }

  /**
   * Commands a closed-loop flywheel speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // IMPLEMENTATION (was TODO: "closedLoop = true; goalRpm = velocityRpm; ffVolts = kKs.get() *
    // Math.signum(velocityRpm) + kKv.get() * velocityRpm / 60.0;
    // controller.setSetpoint(velocityRpm).")
    //
    // The setpoint is in rot/s rather than RPM, for the unit reason explained in updateInputs. The
    // feedforward formula is unchanged from the TODO and was already in rot/s.
    closedLoop = true;
    goalRpm = velocityRpm;
    ffVolts =
        Constants.Shooter.kKs.get() * Math.signum(velocityRpm)
            + Constants.Shooter.kKv.get() * velocityRpm / 60.0;
    controller.setSetpoint(velocityRpm / 60.0);
  }

  /**
   * Drives the flywheel open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setOpenLoop(double volts) {
    // IMPLEMENTATION (was TODO: "closedLoop = false; appliedVolts = volts.")
    //
    // ffVolts is cleared as well. It is added to appliedVolts unconditionally in updateInputs, so
    // leaving a stale feedforward standing would silently offset every characterization point by
    // whatever the last closed-loop request happened to need -- and a characterization run with a
    // constant voltage offset produces a plausible-looking but wrong kS.
    closedLoop = false;
    ffVolts = 0.0;
    goalRpm = 0.0;
    appliedVolts = volts;
  }

  /**
   * Applies gains to the stand-in controller.
   *
   * @param gains the gain set; only kP, kI and kD are meaningful in simulation
   */
  @Override
  public void setGains(Slot0Configs gains) {
    // IMPLEMENTATION (was TODO: "controller.setPID(gains.kP, gains.kI, gains.kD).")
    //
    // kS, kV and kA are ignored here on purpose: this IO reads them straight from Constants in
    // setVelocity, because WPILib's PIDController has no feedforward slot the way Slot0 does.
    controller.setPID(gains.kP, gains.kI, gains.kD);
  }

  /**
   * Simulates one game piece passing through the wheels, at the standard simulated size.
   *
   * <p>Added for M10b (plan section 2.4). Game pieces are not simulated without maple-sim, so this
   * is how the shot detector, burst hysteresis and the superstructure's "probably empty" belief are
   * exercised with no hopper model at all: tests and a sim keyboard binding call it at whatever
   * cadence they want to replay.
   *
   * <p><b>Outputs:</b> removes {@code kShotDipRpm * kSimShotDipMargin} RPM from the wheel at once.
   */
  public void injectShot() {
    injectShot(Constants.Shooter.kShotDipRpm.get() * Constants.Shooter.kSimShotDipMargin);
  }

  /**
   * Simulates one game piece passing through the wheels, taking a chosen amount of speed.
   *
   * <p>The explicit-size form exists for the negative cases: a dip smaller than the detection
   * threshold must NOT count, and proving that needs a way to make one.
   *
   * @param dipRpm speed removed in one step, in RPM; the wheel never goes below zero
   *     <p><b>Outputs:</b> sets the simulated wheel speed. The controller sees the drop on the next
   *     {@link #updateInputs(ShooterIOInputs)} and recovers from it exactly as it would from a real
   *     piece.
   */
  public void injectShot(double dipRpm) {
    // A speed step rather than an energy model. A piece passing through takes a bite out of the
    // wheel's kinetic energy over a few milliseconds, far shorter than a 20 ms loop, so at loop
    // resolution it IS a step; modelling the contact would add parameters nobody can measure yet
    // without changing what the detector sees.
    double newRpm = Math.max(0.0, sim.getAngularVelocityRPM() - dipRpm);
    sim.setAngularVelocity(newRpm * 2.0 * Math.PI / 60.0);
  }

  /** Zeroes the applied voltage and leaves closed loop. */
  @Override
  public void stop() {
    // IMPLEMENTATION (was TODO: "closedLoop = false; appliedVolts = 0; ffVolts = 0; goalRpm = 0.")
    //
    // Zero volts is a coast, not a brake: FlywheelSim keeps integrating the plant, so the wheel
    // spins down against its own back-EMF exactly as the real one does with NeutralModeValue.Coast.
    closedLoop = false;
    appliedVolts = 0.0;
    ffVolts = 0.0;
    goalRpm = 0.0;
  }
}
