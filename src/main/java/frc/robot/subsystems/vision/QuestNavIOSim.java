package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Supplier;

/**
 * Simulated QuestNav headset, with injectable tracking loss and drift so the fallback and re-seed
 * logic can be exercised on a laptop.
 */
public class QuestNavIOSim implements QuestNavIO {
  private final Supplier<Pose2d> groundTruthPoseSupplier;
  private Pose3d seedOffset = new Pose3d();
  private boolean trackingForced = true;
  private double driftMetersPerSec = 0.0;

  /**
   * Builds a simulated headset.
   *
   * @param groundTruthPoseSupplier source of truth for where the robot really is
   */
  public QuestNavIOSim(Supplier<Pose2d> groundTruthPoseSupplier) {
    this.groundTruthPoseSupplier = groundTruthPoseSupplier;
  }

  /**
   * Test hook that drops or restores tracking on demand.
   *
   * @param tracking false to simulate lost tracking
   *     <p><b>Outputs:</b> stores the flag, so the degradation path can be watched engaging.
   */
  public void setTracking(boolean tracking) {
    // TODO: trackingForced = tracking.
  }

  /**
   * Test hook that injects slow positional drift.
   *
   * @param metersPerSec drift rate, in meters per second
   *     <p><b>Outputs:</b> stores the drift rate, so the re-seed logic is exercised in simulation.
   */
  public void setDrift(double metersPerSec) {
    // TODO: driftMetersPerSec = metersPerSec.
  }

  /**
   * Emits one simulated pose frame per loop.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    // TODO: Emit one pose at Timer.getFPGATimestamp() - 0.01, equal to the ground-truth pose
    // composed with Constants.Vision.kRobotToQuest and seedOffset, plus accumulated drift.
    // Set connected = true, tracking = trackingForced, batteryPercent = 100.
  }

  /**
   * Re-seeds the simulated headset origin.
   *
   * @param fieldToQuest the pose the next report should equal
   *     <p><b>Outputs:</b> recomputes {@code seedOffset}.
   */
  @Override
  public void setPose(Pose3d fieldToQuest) {
    // TODO: Recompute seedOffset so the next reported pose equals fieldToQuest exactly, and zero
    // the accumulated drift.
  }

  /** No-op in simulation. */
  @Override
  public void commandPeriodic() {
    // Intentionally empty.
  }
}
