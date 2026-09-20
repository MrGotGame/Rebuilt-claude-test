package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Simulated turret.
 *
 * <p>The important detail is that this sim <b>reproduces the hard stops</b>. Without them the
 * unwrap logic is never exercised before the code reaches the real robot, and the first time anyone
 * finds out that the turret takes the long way around is when a belt jumps.
 *
 * <p>Plan section 9.2 rates this the highest-value simulation in the project: it is what validates
 * unwrap branch selection, limit behaviour, the sign of the velocity feedforward, and the whole aim
 * loop against a simulated pose, before a turret physically exists. Everything below is therefore
 * written to behave like {@link TurretIOTalonFX} rather than merely to move plausibly -- same
 * units, same feedforward formula, same closed-loop-error semantics -- because a simulation that
 * disagrees with the hardware in any of those is worse than none at all: it validates the wrong
 * thing and does it convincingly.
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

    // IMPLEMENTATION (was TODO: "Seed the gains and constraints from Constants.Turret rather than
    // the literals above.")
    //
    // Everything in this controller is in ROTATIONS, not radians, and that is the single most
    // important decision in this file. On the real Talon, SensorToMechanismRatio makes the
    // mechanism unit one turret rotation, so Slot0Configs.kP is volts per ROTATION of error and the
    // Motion Magic constraints are rotations per second. Running the simulated loop in radians
    // would make the same numeric kP 2*pi times weaker in simulation than on hardware -- so a gain
    // tuned in sim, which is the entire purpose of this class, would arrive on the robot six times
    // too hot and slam a finite-travel mechanism into its stops. The radian conversion therefore
    // happens only at the setpoint and input-struct boundaries.
    //
    // The same reasoning applies to the constraints: they mirror the MotionMagic cruise and
    // acceleration that TurretIOTalonFX configures, converted identically, so a sweep takes the
    // same wall-clock time in both.
    controller =
        new ProfiledPIDController(
            Constants.Turret.kKp.get(),
            0.0,
            Constants.Turret.kKd.get(),
            new TrapezoidProfile.Constraints(
                Units.radiansToRotations(Constants.Turret.kCruiseRadPerSec.get()),
                Units.radiansToRotations(Constants.Turret.kAccelRadPerSec2.get())));
  }

  /**
   * Advances the model one loop period, enforces the hard stops, and fills the input struct.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "When closedLoop, appliedVolts = ffVolts +
    // controller.calculate(sim.getAngularPositionRad()). Clamp to +-12 V, sim.setInputVoltage,
    // sim.update(Constants.getLoopPeriodSec()). THEN clamp the simulated position into
    // [minRad, maxRad] with sim.setState(clamped, 0.0) when it would exceed a limit -- zeroing
    // velocity at the stop is what makes the sim behave like a real hard stop instead of an
    // infinite-travel azimuth. Fill every field; set motorConnected and absEncoderConnected true,
    // and absolutePositionRad equal to the wrapped positionRad.")
    //
    // DEVIATION FROM THE TODO, deliberate: the controller is fed ROTATIONS, not
    // sim.getAngularPositionRad() directly. See the unit argument in the constructor.
    if (closedLoop) {
      appliedVolts =
          ffVolts + controller.calculate(Units.radiansToRotations(sim.getAngularPositionRad()));
    } else {
      // Resetting while open loop clears the integrator AND re-anchors the trapezoid profile to the
      // current position. Without it, a homing or characterization run followed by a closed-loop
      // request would resume the profile from wherever it had marched to during the open-loop
      // period -- which on this mechanism means a step command of up to the full travel.
      controller.reset(Units.radiansToRotations(sim.getAngularPositionRad()));
    }

    // The clamped value is what the model is driven with AND what gets reported, so saturation is
    // visible in the log. Reporting the raw request instead would show a trace no Talon can
    // produce, and the first question anyone asks about a sluggish simulated turret is whether the
    // controller is simply asking for more than 12 V.
    double outputVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(outputVolts);
    sim.update(Constants.getLoopPeriodSec());

    // THE hard stop. DCMotorSim models an infinite-travel shaft, so without this the simulated
    // turret would happily wind past 400 degrees and every unwrap decision would look correct
    // because nothing ever pushed back. Zeroing the velocity as well as the position is what makes
    // it a stop rather than a wall the shaft bounces off: a real turret against its hard stop is
    // stationary, drawing stall current, not reversing.
    //
    // The controller is deliberately NOT reset here. A real closed loop jammed against a stop keeps
    // pushing and keeps drawing current, and reproducing that is how the simulation earns the right
    // to be trusted about the OUT_OF_RANGE path -- which exists precisely so this never happens.
    double positionRad = sim.getAngularPositionRad();
    if (positionRad < minRad) {
      positionRad = minRad;
      sim.setState(minRad, 0.0);
    } else if (positionRad > maxRad) {
      positionRad = maxRad;
      sim.setState(maxRad, 0.0);
    }

    inputs.motorConnected = true;
    inputs.absEncoderConnected = true;
    inputs.positionRad = positionRad;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();

    // Wrapped, exactly like a real single-turn ThroughBore on the output shaft -- which means the
    // simulation faithfully reproduces the ambiguity that Turret.resolveAbsoluteSeed exists to
    // handle: with travel wider than one turn, two distinct true positions produce the same
    // absolute reading. Reporting the unwrapped position here instead would make the simulated
    // turret home perfectly every time and hide the one homing failure that can actually break the
    // mechanism.
    inputs.absolutePositionRad = MathUtil.angleModulus(positionRad);
    inputs.appliedVolts = outputVolts;

    double statorAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.statorCurrentAmps = statorAmps;

    // A motor controller is a buck converter: supply current is roughly stator current scaled by
    // duty cycle. Crude, but reporting supply == stator would overstate the battery load by up to
    // 6x at low output and make any sim-side power budgeting meaningless.
    inputs.supplyCurrentAmps = statorAmps * Math.abs(outputVolts) / 12.0;

    // DCMotorSim has no thermal model. A fixed room temperature is reported rather than 0.0 so that
    // a plot of this field in simulation is obviously constant rather than looking like a sensor
    // reading zero -- which is what a genuinely disconnected Talon reports.
    inputs.tempCelsius = 25.0;

    // Reported against the PROFILE's instantaneous reference, matching what the real Talon's
    // getClosedLoopError/getClosedLoopReference return under a Motion Magic request. Reporting the
    // error against the final goal instead would be more flattering and would silently break
    // Turret.atGoal(), which is written to the hardware semantics.
    double referenceRad = Units.rotationsToRadians(controller.getSetpoint().position);
    inputs.closedLoopReferenceRad = closedLoop ? referenceRad : 0.0;
    inputs.closedLoopErrorRad = closedLoop ? referenceRad - positionRad : 0.0;
  }

  /**
   * Commands a position with velocity feedforward.
   *
   * @param positionRad continuous target position, in radians
   * @param velocityFeedforwardRadPerSec target angular rate, in radians per second
   *     <p><b>Outputs:</b> enters closed loop and moves the profiled controller's goal.
   */
  @Override
  public void setPositionSetpoint(double positionRad, double velocityFeedforwardRadPerSec) {
    // IMPLEMENTATION (was TODO: "closedLoop = true; controller.setGoal(positionRad); ffVolts =
    // Constants.Turret.kKv.get() * velocityFeedforwardRadPerSec.")
    //
    // TWO DELIBERATE DEVIATIONS from the TODO, both so that this class validates the thing it is
    // here to validate:
    //
    // 1. The goal is converted to rotations, for the unit reason argued in the constructor.
    //
    // 2. The feedforward is the FULL kS + kV expression that TurretIOTalonFX uses, not kV alone.
    //    Plan section 9.2 makes this simulation responsible for catching a wrong feedforward SIGN,
    //    and the sign lives in the Math.signum(kS) term as much as in kV. A sim that modelled only
    //    the kV half would run a materially different feedforward from the robot and would pass
    //    happily with the kS term inverted -- which on hardware is a turret that fights itself
    //    whenever it tracks counter-clockwise.
    closedLoop = true;
    controller.setGoal(Units.radiansToRotations(positionRad));
    ffVolts =
        Constants.Turret.kKs.get() * Math.signum(velocityFeedforwardRadPerSec)
            + Constants.Turret.kKv.get() * Units.radiansToRotations(velocityFeedforwardRadPerSec);
  }

  /**
   * Drives the turret open loop.
   *
   * @param volts commanded voltage
   *     <p><b>Outputs:</b> leaves closed loop and sets the applied voltage.
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
    appliedVolts = volts;
  }

  /**
   * Teleports the simulated turret to a known angle.
   *
   * @param positionRad the angle to assume, in radians
   *     <p><b>Outputs:</b> overwrites the model's state.
   */
  @Override
  public void seedPosition(double positionRad) {
    // IMPLEMENTATION (was TODO: "sim.setState(positionRad, 0.0).")
    //
    // This is the sim's counterpart to writing the Talon's internal position, so like the real one
    // it must NOT move the mechanism -- it redefines where the mechanism believes it is. Zeroing
    // the velocity is part of that: a seed happens when the turret is known to be stationary.
    //
    // The profiled controller is re-anchored too. Without it, the next closed-loop command would
    // profile from the pre-seed position and command a step equal to the seed correction, which is
    // exactly the moment -- immediately after homing -- when the turret can least afford one.
    sim.setState(positionRad, 0.0);
    controller.reset(Units.radiansToRotations(positionRad));
  }

  /**
   * Applies gains to the stand-in controller.
   *
   * @param gains the gain set; only kP, kI and kD are meaningful in simulation
   *     <p><b>Outputs:</b> retunes the stand-in controller.
   */
  @Override
  public void setGains(Slot0Configs gains) {
    // IMPLEMENTATION (was TODO: "controller.setPID(gains.kP, gains.kI, gains.kD).")
    //
    // kS, kV and kA are ignored here on purpose: this IO reads them straight from Constants in
    // setPositionSetpoint, because WPILib's ProfiledPIDController has no feedforward slot the way
    // Slot0Configs does. In practice kI arrives as zero, since TurretIOTalonFX.currentGains()
    // deliberately never sets one.
    controller.setPID(gains.kP, gains.kI, gains.kD);
  }

  /**
   * Stores the travel limits used by the clamp in {@link #updateInputs}.
   *
   * @param minRad lower travel limit, in radians
   * @param maxRad upper travel limit, in radians
   *     <p><b>Outputs:</b> stores both limits.
   */
  @Override
  public void setSoftLimits(double minRad, double maxRad) {
    // IMPLEMENTATION (was TODO: "this.minRad = minRad; this.maxRad = maxRad.")
    //
    // These stand in for the Talon's firmware limit switches AND for the physical hard stops at
    // once. The real mechanism has both, a few degrees apart; modelling only one of them is enough
    // to exercise every code path that cares, and modelling the gap would require a number nobody
    // has measured yet.
    this.minRad = minRad;
    this.maxRad = maxRad;
  }

  /** Zeroes the applied voltage and leaves closed loop. */
  @Override
  public void stop() {
    // IMPLEMENTATION (was TODO: "closedLoop = false; appliedVolts = 0; ffVolts = 0.")
    //
    // Zero volts is a coast here, not the brake the real Talon applies: DCMotorSim keeps
    // integrating the plant, so the simulated turret drifts to a halt against its own back-EMF
    // rather than stopping dead. On a low-inertia, high-ratio turret the difference is a fraction
    // of a degree, and modelling brake properly would need a friction term nobody has measured.
    closedLoop = false;
    appliedVolts = 0.0;
    ffVolts = 0.0;
  }
}
