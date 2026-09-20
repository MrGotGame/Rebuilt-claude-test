package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

/**
 * Thin wrapper over {@link QuestNav} from questnavlib 2026-3.0.0-beta.
 *
 * <p><b>API verified</b> against the installed beta with {@code javap}, which the skeleton's TODO
 * asked for and which was worth doing: the plan documents {@code getBatteryPercent()} and {@code
 * getTrackingLostCounter()} as returning {@code Optional<Integer>}, and they actually return {@code
 * OptionalInt}. Those two have no useful supertype in common, so that particular error would have
 * been caught by the compiler -- but a beta dependency is exactly where a signature moves again,
 * and the next change may not be so loud. The surface this class uses is listed here so it can be
 * re-checked in one pass:
 *
 * <pre>
 *   void          commandPeriodic()
 *   boolean       isConnected()
 *   boolean       isTracking()
 *   OptionalInt   getBatteryPercent()
 *   OptionalInt   getTrackingLostCounter()
 *   OptionalInt   getFrameCount()
 *   PoseFrame[]   getAllUnreadPoseFrames()
 *   void          setPose(Pose3d)
 *   void          onConnected / onDisconnected / onTrackingAcquired / onTrackingLost(Runnable)
 *   void          onLowBattery(int, IntConsumer)
 *
 *   PoseFrame     questPose3d(), dataTimestamp(), appTimestamp(), frameCount(), isTracking()
 * </pre>
 */
public class QuestNavIOReal implements QuestNavIO {
  private final QuestNav questNav = new QuestNav();

  /**
   * Sentinel for a health value the headset did not report.
   *
   * <p>Negative on purpose, so it can never be mistaken for a healthy reading. The trust gate
   * compares the battery against a positive threshold, so an absent value fails that comparison and
   * QuestNav is distrusted -- the correct response to a headset that will not say how it is doing.
   */
  private static final int kUnknown = -1;

  /** Registers the headset event callbacks. */
  public QuestNavIOReal() {
    // IMPLEMENTATION (was TODO: "Register the event callbacks so tracking transitions land in the
    // log with no polling.")
    //
    // These are edge-triggered and the polled inputs are level-triggered, and both are wanted. A
    // 50 Hz poll of isTracking() cannot distinguish "tracking was lost and reacquired between two
    // loops" from "nothing happened", and a headset jarred by a defender does exactly that. The
    // event lands in the log with its own timestamp; the poll drives the trust gate.
    questNav.onConnected(() -> Logger.recordOutput("QuestNav/Event", "Connected"));
    questNav.onDisconnected(() -> Logger.recordOutput("QuestNav/Event", "Disconnected"));
    questNav.onTrackingAcquired(() -> Logger.recordOutput("QuestNav/Event", "TrackingAcquired"));
    questNav.onTrackingLost(() -> Logger.recordOutput("QuestNav/Event", "TrackingLost"));

    // The threshold is read ONCE, here, unlike every other use of a LoggedTunableNumber in this
    // project. questnavlib takes the trip point by value at registration and offers no way to
    // re-arm it, so editing Vision/QuestMinBattery at a competition moves the trust gate -- which
    // reads the tunable live in Vision.updateQuestNavTrust -- without moving this callback. That
    // is tolerable only because the callback is a log annotation and the gate is what protects the
    // pose. It would be a real defect if anything depended on the two agreeing.
    questNav.onLowBattery(
        (int) Constants.Vision.kQuestMinBatteryPercent.get(),
        percent -> Logger.recordOutput("QuestNav/Event", "LowBattery:" + percent));
  }

  /**
   * Drains all unread pose frames and headset health.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "connected, tracking, battery and trackingLostCounter from the
    // getters with .orElse(-1); drain getAllUnreadPoseFrames() into the parallel arrays.")
    inputs.connected = questNav.isConnected();
    inputs.tracking = questNav.isTracking();
    inputs.batteryPercent = questNav.getBatteryPercent().orElse(kUnknown);
    inputs.trackingLostCounter = questNav.getTrackingLostCounter().orElse(kUnknown);
    inputs.frameCount = questNav.getFrameCount().orElse(kUnknown);

    // EVERY frame, every loop. The headset publishes at about 100 Hz against a 50 Hz robot loop, so
    // there are normally two waiting; reading one and leaving the rest builds a backlog that never
    // clears, and each frame handed to the estimator is then older than the last. That is the
    // "QuestNav pose is 200 ms behind" complaint, and it presents as a pose that lags during
    // motion and looks perfect at rest -- which sends people to tune covariances instead.
    PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
    inputs.questPoses = new Pose3d[frames.length];
    inputs.questPoseTimestamps = new double[frames.length];
    for (int i = 0; i < frames.length; i++) {
      inputs.questPoses[i] = frames[i].questPose3d();

      // dataTimestamp, not appTimestamp. The former is when the headset's tracker produced the
      // pose, expressed on the robot's clock; the latter runs on the headset app's own clock,
      // which has no fixed relationship to the FPGA timebase. Feeding the estimator a timestamp
      // from a different clock does not fail -- it silently correlates the measurement against the
      // wrong moment of the robot's history, which is indistinguishable from a bad mounting
      // transform.
      inputs.questPoseTimestamps[i] = frames[i].dataTimestamp();
    }
  }

  /**
   * Re-seeds the headset origin.
   *
   * @param fieldToQuest the true field pose of the headset
   */
  @Override
  public void setPose(Pose3d fieldToQuest) {
    // IMPLEMENTATION (was TODO: "questNav.setPose(fieldToQuest). This sets the HEADSET pose, not
    // the robot pose, so the caller must already have composed kRobotToQuest.")
    //
    // No composition here, deliberately. Applying it in both the caller and the IO is how the
    // transform ends up counted twice, and a doubled mounting offset is a constant bias that looks
    // exactly like a mis-measured kRobotToQuest -- so it would be hunted in CAD rather than code.
    questNav.setPose(fieldToQuest);
  }

  /** Runs the questnavlib command and response machinery. */
  @Override
  public void commandPeriodic() {
    // IMPLEMENTATION (was TODO: "questNav.commandPeriodic(). Call this first thing in
    // Vision.periodic().")
    //
    // Not optional, and not merely bookkeeping: the callbacks registered in the constructor fire
    // from inside this call, and a setPose command is delivered and acknowledged through it. Skip
    // it and seeding silently never completes, which presents as a headset stuck in its own
    // arbitrary frame forever while the code cheerfully reports it as seeded.
    questNav.commandPeriodic();
  }
}
