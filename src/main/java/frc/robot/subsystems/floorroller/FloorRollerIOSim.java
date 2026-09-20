package frc.robot.subsystems.floorroller;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
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
    // The placeholder inertia the skeleton hard-coded here now lives in
    // Constants.FloorRoller.kMoiKgM2, carrying its "TODO: from CAD" with it. The number is still a
    // guess; moving it keeps every tunable-but-unmeasured value in one file, which is where anyone
    // sweeping the project for guesses before a competition will look.
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                Constants.FloorRoller.kMoiKgM2,
                Constants.FloorRoller.kGearRatio),
            DCMotor.getKrakenX60Foc(1));
  }

  /**
   * Advances the model one loop period and fills the input struct.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(FloorRollerIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "When closedLoop, appliedVolts = ffVolts + controller.calculate(
    // sim.getAngularVelocityRadPerSec()). Clamp to +-12 V, sim.setInputVoltage(appliedVolts),
    // sim.update(Constants.getLoopPeriodSec()), then fill every field with connected = true.")
    //
    // DEVIATION FROM THE TODO, deliberate, and the same one ShooterIOSim makes: the TODO runs the
    // simulated controller on RADIANS PER SECOND. Slot0Configs.kP on the real Talon is volts per
    // ROTATION-per-second of error, and the feedforward below is likewise in rot/s. Feeding rad/s
    // to the controller would make the same numeric kP 2*pi times stronger in simulation than on
    // hardware, so a gain tuned in sim -- which is the whole point of having this class -- would
    // arrive on the robot six times too hot. The loop below is therefore in rot/s throughout and
    // the RPM conversion happens only at the input-struct boundary.
    if (closedLoop) {
      double velocityRotPerSec =
          Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec()) / 60.0;
      appliedVolts = ffVolts + controller.calculate(velocityRotPerSec);
    } else {
      // Resetting while open loop clears the integrator. Without this, a characterization run
      // followed by a closed-loop request would start with whatever integral had accumulated before
      // the mode switch and deliver a step to the roller.
      controller.reset();
    }

    // The clamped value is what the model is driven with AND what gets reported, so that saturation
    // is visible in the log. Reporting the raw request instead would show a 20 V trace that no
    // Talon can produce, and the first thing anyone debugging a sluggish simulated roller wants to
    // know is whether the controller is simply asking for more than 12 V.
    double outputVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(outputVolts);
    sim.update(Constants.getLoopPeriodSec());

    inputs.connected = true;
    inputs.velocityRpm =
        Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = outputVolts;

    // DCMotorSim reports the draw of the single modelled motor, so no halving is needed here (the
    // flywheel's sim models two motors as one plant and does halve it).
    double statorAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.statorCurrentAmps = statorAmps;

    // A motor controller is a buck converter: supply current is roughly stator current scaled by
    // duty cycle. The approximation is crude, but reporting supply == stator would overstate the
    // battery load by up to 6x at low speed and make any sim-side power budgeting meaningless --
    // and this mechanism is the first one PowerBudget throttles, so that number matters here.
    inputs.supplyCurrentAmps = statorAmps * Math.abs(outputVolts) / 12.0;

    // DCMotorSim has no thermal model. A fixed room temperature is reported rather than 0.0 so that
    // a plot of this field in simulation is obviously constant rather than looking like a sensor
    // reading zero -- which is what a genuinely disconnected Talon reports.
    inputs.tempCelsius = 25.0;
  }

  /**
   * Commands a closed-loop roller speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // IMPLEMENTATION (was TODO: "closedLoop = true; ffVolts = kKs * Math.signum(rpm) + kKv * rpm /
    // 60.0; controller.setSetpoint(Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm)).")
    //
    // The setpoint is in rot/s rather than rad/s, for the unit reason explained in updateInputs.
    // The feedforward formula is unchanged from the TODO and was already in rot/s.
    //
    // Math.signum on the kS term is not decoration on this mechanism. EJECT commands a NEGATIVE
    // RPM, and a fixed +kS would then push against the very motion being asked for -- a bug that
    // hides completely on a flywheel, which only ever spins one way.
    closedLoop = true;
    ffVolts =
        Constants.FloorRoller.kKs.get() * Math.signum(velocityRpm)
            + Constants.FloorRoller.kKv.get() * velocityRpm / 60.0;
    controller.setSetpoint(velocityRpm / 60.0);
  }

  /**
   * Drives the roller open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setOpenLoop(double volts) {
    // IMPLEMENTATION (was TODO: "closedLoop = false; appliedVolts = volts.")
    //
    // ffVolts is cleared as well. It is added to appliedVolts unconditionally above, so leaving a
    // stale feedforward standing would silently offset every characterization point by whatever the
    // last closed-loop request happened to need -- and a staircase run with a constant voltage
    // offset produces a plausible-looking but wrong kS.
    closedLoop = false;
    ffVolts = 0.0;
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
    // kS and kV are ignored here on purpose: this IO reads them straight from Constants in
    // setVelocity, because WPILib's PIDController has no feedforward slot the way Slot0 does. In
    // practice kI and kD arrive as zero, since FloorRollerIOTalonFX.currentGains() never sets them.
    controller.setPID(gains.kP, gains.kI, gains.kD);
  }

  /** Zeroes the applied voltage and leaves closed loop. */
  @Override
  public void stop() {
    // IMPLEMENTATION (was TODO: "closedLoop = false; appliedVolts = 0; ffVolts = 0.")
    //
    // Zero volts is a coast, not a brake: DCMotorSim keeps integrating the plant, so the roller
    // spins down against its own back-EMF exactly as the real one does with NeutralModeValue.Coast.
    closedLoop = false;
    appliedVolts = 0.0;
    ffVolts = 0.0;
  }
}
