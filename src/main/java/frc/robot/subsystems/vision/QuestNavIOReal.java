package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.QuestNav;

/**
 * Thin wrapper over {@link QuestNav} from questnavlib 2026-3.0.0-beta.
 *
 * <p>Expected API surface: {@code commandPeriodic()}, {@code getAllUnreadPoseFrames()} returning
 * frames whose components are {@code questPose3d()}, {@code dataTimestamp()}, {@code
 * appTimestamp()}, {@code frameCount()} and {@code isTracking()}; plus {@code isConnected()},
 * {@code isTracking()}, {@code getBatteryPercent()} and {@code getTrackingLostCounter()} returning
 * {@code Optional<Integer>}, and {@code setPose(Pose3d)}.
 *
 * <p>TODO: VERIFY that surface against the installed beta before relying on it. This is a beta
 * dependency and its method names are the most likely thing in this project to have moved.
 */
public class QuestNavIOReal implements QuestNavIO {
  private final QuestNav questNav = new QuestNav();

  /** Registers the headset event callbacks. */
  public QuestNavIOReal() {
    // TODO: Register the event callbacks so tracking transitions land in the log with no polling:
    //   questNav.onTrackingLost(() -> Logger.recordOutput("QuestNav/Event", "TrackingLost"))
    //   questNav.onTrackingAcquired(...), onConnected(...), onDisconnected(...)
    //   questNav.onLowBattery(Constants.Vision.kQuestMinBatteryPercent.get(), pct -> ...)
    // Callbacks only fire from commandPeriodic(), so that call is not optional.
  }

  /**
   * Drains all unread pose frames and headset health.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    // TODO: inputs.connected = questNav.isConnected(); inputs.tracking = questNav.isTracking();
    // batteryPercent and trackingLostCounter from the Optional<Integer> getters with .orElse(-1).
    // Drain questNav.getAllUnreadPoseFrames() into the PARALLEL questPoses and questPoseTimestamps
    // arrays, using frame.questPose3d() and frame.dataTimestamp().
    // Drain EVERY frame each loop. Leaving frames unread is the direct cause of the classic
    // "QuestNav pose is 200 ms behind" complaint, because the backlog never clears.
  }

  /**
   * Re-seeds the headset origin.
   *
   * @param fieldToQuest the true field pose of the headset
   */
  @Override
  public void setPose(Pose3d fieldToQuest) {
    // TODO: questNav.setPose(fieldToQuest). This sets the HEADSET pose, not the robot pose, so the
    // caller must already have composed Constants.Vision.kRobotToQuest.
  }

  /** Runs the questnavlib command and response machinery. */
  @Override
  public void commandPeriodic() {
    // TODO: questNav.commandPeriodic(). Call this first thing in Vision.periodic().
  }
}
