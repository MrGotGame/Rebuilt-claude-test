package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Simulated indexer, including a synthetic hopper load and jam.
 *
 * <p>The load and jam simulation matters: it lets jam recovery and the load reading be exercised on
 * a laptop exactly as they will run on the robot, which is the only way to debug that logic before
 * the mechanism exists.
 */
public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean closedLoop = false;
  private boolean simulatedLoad = false;
  private boolean simulatedJam = false;
  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;

  /** Builds the motor model. NOTE: pre-filled because {@code sim} is final. */
  public IndexerIOSim() {
    // IMPLEMENTATION (was TODO: "Replace the placeholder 0.001 kg*m^2 inertia with the real value
    // from CAD.")
    //
    // The number is still a guess, and the TODO it carries has moved with it into
    // Constants.Indexer.kMoiKgM2 rather than being resolved here. That follows the precedent set by
    // FloorRollerIOSim: every tunable-but-unmeasured value lives in Constants, which is where
    // anyone sweeping the project for guesses before a competition will look -- an inertia buried
    // in a sim constructor is one nobody finds.
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                Constants.Indexer.kMoiKgM2,
                Constants.Indexer.kGearRatio),
            DCMotor.getKrakenX60Foc(1));
  }

  /**
   * Advances the model one loop period and fills the input struct, including the synthetic load and
   * jam.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "Standard sim update, as in FloorRollerIOSim. Additionally, when
    // simulatedPiecePresent, add a synthetic current offset that pushes statorCurrentAmps above
    // kHasPieceCurrentAmps AND a velocity droop of more than kHasPieceVelocityDropRpm, so BOTH
    // halves of the detection predicate are exercised. Set beamBreakTripped =
    // simulatedPiecePresent and beamBreakConnected = Constants.Indexer.kHasBeamBreak.") The
    // synthetic piece and the beam break were deleted in M10b step 8; the hopper's load and a jam
    // replace them below.
    //
    // DEVIATION FROM THE TODO, deliberate, and the same one FloorRollerIOSim and ShooterIOSim make:
    // the TODO's "standard sim update" runs the simulated controller on RADIANS PER SECOND.
    // Slot0Configs.kP on the real Talon is volts per ROTATION-per-second of error, and the
    // feedforward below is likewise in rot/s. Feeding rad/s to the controller would make the same
    // numeric kP 2*pi times stronger in simulation than on hardware, so a gain tuned in sim -- the
    // whole point of having this class -- would arrive on the robot six times too hot. The loop
    // below is therefore in rot/s throughout and the RPM conversion happens only at the
    // input-struct boundary.
    if (closedLoop) {
      double velocityRotPerSec =
          Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec()) / 60.0;
      appliedVolts = ffVolts + controller.calculate(velocityRotPerSec);
    } else {
      // Resetting while open loop clears the integrator. Without this, a characterization run
      // followed by a closed-loop request would start with whatever integral had accumulated before
      // the mode switch and deliver a step to the indexer.
      controller.reset();
    }

    // The clamped value is what the model is driven with AND what gets reported, so that saturation
    // is visible in the log. Reporting the raw request instead would show a 20 V trace that no
    // Talon can produce, and the first thing anyone debugging a sluggish simulated indexer wants to
    // know is whether the controller is simply asking for more than 12 V.
    double outputVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(outputVolts);
    sim.update(Constants.getLoopPeriodSec());

    inputs.connected = true;
    inputs.positionRotations = sim.getAngularPositionRotations();
    inputs.appliedVolts = outputVolts;

    // The two values the synthetic load and jam perturb, taken from the model first.
    // DCMotorSim reports the draw of the single modelled motor, so no halving is needed here (the
    // flywheel's sim models two motors as one plant and does halve it).
    double velocityRpm =
        Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec());
    double statorAmps = Math.abs(sim.getCurrentDrawAmps());

    // M10b step 6: the hopper's load and a jam, perturbed at the reporting boundary rather than as
    // a load torque on the plant, because DCMotorSim has no external-torque input. Plan section 9.2
    // is explicit that game pieces are not simulated without maple-sim; this is that limit.
    // Both act only while the wheel is driven FORWARD, toward the flywheel, which is the only
    // direction the hopper pushes against. A reverse unjam pulse therefore reads normally, and a
    // jam is cleared only when the case clears it, never by the pulse itself: that is what lets a
    // test prove the indexer retries and then gives up.
    boolean drivenForward = outputVolts > 0.0;
    double loadMargin = Constants.Indexer.kSimLoadMargin;
    if (simulatedJam && drivenForward) {
      // Stopped against the jam: well above the jam current, speed held under the jam threshold.
      statorAmps = Math.max(statorAmps, Constants.Indexer.kJamCurrentAmps.get() * loadMargin);
      velocityRpm = 0.0;
    } else if (simulatedLoad && drivenForward) {
      // Working against the pile at full speed: above the loaded current, below the jam current,
      // and the speed left alone. That is the stiff-loop case -- a well-tuned velocity loop holds
      // its speed and shows the load only as current -- which is exactly the case the load
      // reading must NOT miss, so it is the one simulated.
      statorAmps = Math.max(statorAmps, Constants.Indexer.kLoadedCurrentAmps.get() * loadMargin);
    }

    inputs.velocityRpm = velocityRpm;
    inputs.statorCurrentAmps = statorAmps;

    // A motor controller is a buck converter: supply current is roughly stator current scaled by
    // duty cycle. The approximation is crude, but reporting supply == stator would overstate the
    // battery load by up to 6x at low speed and make any sim-side power budgeting meaningless.
    inputs.supplyCurrentAmps = statorAmps * Math.abs(outputVolts) / 12.0;

    // DCMotorSim has no thermal model. A fixed room temperature is reported rather than 0.0 so that
    // a plot of this field in simulation is obviously constant rather than looking like a sensor
    // reading zero -- which is what a genuinely disconnected Talon reports.
    inputs.tempCelsius = 25.0;
  }

  /**
   * Test hook: the hopper is pressing pieces into the indexer.
   *
   * <p>Added for M10b step 6. While set and driven forward, the indexer reports the current of a
   * wheel working against a load, which is what {@code Indexer.isLoaded()} looks for. It is the
   * hopper-era counterpart of the single-piece model's {@code setSimulatedPiecePresent}, deleted in
   * step 8: there is no longer one piece to be present or absent, only a pile that is or is not
   * pushing.
   *
   * @param loaded true to simulate a loaded feed
   *     <p><b>Outputs:</b> sets the synthetic load flag read by {@link #updateInputs}.
   */
  public void setSimulatedLoad(boolean loaded) {
    simulatedLoad = loaded;
  }

  /**
   * Test hook: something is jammed in the indexer.
   *
   * <p>Added for M10b step 6. While set and driven forward, the wheel reports stopped against a
   * current above the jam threshold. Reversing does not clear it; only clearing this flag does,
   * which is what lets a test watch the indexer retry and then give up.
   *
   * @param jammed true to simulate a jam
   *     <p><b>Outputs:</b> sets the synthetic jam flag read by {@link #updateInputs}.
   */
  public void setSimulatedJam(boolean jammed) {
    simulatedJam = jammed;
  }

  /**
   * Commands a closed-loop indexer speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // IMPLEMENTATION (was TODO: "closedLoop = true; set ffVolts from kKv and the setpoint on the
    // controller.")
    //
    // The setpoint is in rot/s rather than rad/s, for the unit reason explained in updateInputs.
    //
    // There is no kS term, because Constants.Indexer has no kKs tunable and one is not invented
    // here -- see IndexerIOTalonFX.currentGains() for the reasoning and for what to do if a
    // measured one is ever wanted. kV alone is already correctly signed for EJECT: a negative RPM
    // produces a negative feedforward voltage, which is the direction being asked for.
    closedLoop = true;
    ffVolts = Constants.Indexer.kKv.get() * velocityRpm / 60.0;
    controller.setSetpoint(velocityRpm / 60.0);
  }

  /**
   * Drives the indexer open loop.
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
    // kV is ignored here on purpose: this IO reads it straight from Constants in setVelocity,
    // because WPILib's PIDController has no feedforward slot the way Slot0 does. In practice kI and
    // kD arrive as zero, since IndexerIOTalonFX.currentGains() never sets them.
    controller.setPID(gains.kP, gains.kI, gains.kD);
  }

  /** Zeroes the applied voltage and leaves closed loop. */
  @Override
  public void stop() {
    // IMPLEMENTATION (was TODO: "closedLoop = false; appliedVolts = 0; ffVolts = 0.")
    //
    // DEVIATION IN FIDELITY, stated rather than hidden: zero volts here is a coast, because
    // DCMotorSim keeps integrating the plant and the wheel spins down against its own back-EMF. The
    // real indexer is configured NeutralModeValue.Brake, so it stops considerably faster. The
    // difference is not worth modelling -- WPILib's sim classes have no brake mode -- but it does
    // mean a hopper-retention problem seen on the robot will not reproduce in simulation, so do not
    // conclude from a clean sim run that brake mode is working.
    closedLoop = false;
    appliedVolts = 0.0;
    ffVolts = 0.0;
  }
}
