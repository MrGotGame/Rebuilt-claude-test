package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Simulated indexer, including a synthetic game piece.
 *
 * <p>The piece simulation matters: it lets the current-spike detection path be exercised on a
 * laptop exactly as it will run on the robot, which is the only way to debug that logic before the
 * mechanism exists.
 */
public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean closedLoop = false;
  private boolean simulatedPiecePresent = false;
  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;

  /** Builds the motor model. NOTE: pre-filled because {@code sim} is final. */
  public IndexerIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, Constants.Indexer.kGearRatio),
            DCMotor.getKrakenX60Foc(1));
    // TODO: Replace the placeholder 0.001 kg*m^2 inertia with the real value from CAD.
  }

  /**
   * Advances the model one loop period and fills the input struct, including the synthetic piece.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // TODO: Standard sim update, as in FloorRollerIOSim. Additionally, when simulatedPiecePresent,
    // add a synthetic current offset that pushes statorCurrentAmps above kHasPieceCurrentAmps AND a
    // velocity droop of more than kHasPieceVelocityDropRpm, so BOTH halves of the detection
    // predicate are exercised. Set beamBreakTripped = simulatedPiecePresent and
    // beamBreakConnected = Constants.Indexer.kHasBeamBreak.
  }

  /**
   * Test hook that places or removes a simulated game piece.
   *
   * @param present true to simulate a held piece
   *     <p><b>Outputs:</b> sets the synthetic piece flag read by {@link #updateInputs}.
   */
  public void setSimulatedPiecePresent(boolean present) {
    // TODO: simulatedPiecePresent = present. Called by the simulation keyboard bindings, or by
    // maple-sim's intake callback, so a student can drive a full intake-to-shoot cycle on a laptop.
  }

  /**
   * Commands a closed-loop indexer speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: closedLoop = true; set ffVolts from kKv and the setpoint on the controller.
  }

  /**
   * Drives the indexer open loop.
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
