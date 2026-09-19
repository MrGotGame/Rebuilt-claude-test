package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware contract for the Meta Quest 3S headset running QuestNav. */
public interface QuestNavIO {
  /** Everything the headset reports, captured once per loop and logged for replay. */
  @AutoLog
  public static class QuestNavIOInputs {
    public boolean connected = false;
    public boolean tracking = false;
    public int batteryPercent = 0;
    public int trackingLostCounter = 0;
    public int frameCount = 0;

    /** Headset poses in the Quest's own frame, oldest first, drained each loop. */
    public Pose3d[] questPoses = new Pose3d[0];

    /** FPGA capture times matching {@link #questPoses}, index for index, in seconds. */
    public double[] questPoseTimestamps = new double[0];
  }

  /**
   * Samples the headset into the input struct.
   *
   * @param inputs the struct to populate; every field must be written every call
   */
  public default void updateInputs(QuestNavIOInputs inputs) {}

  /**
   * Re-seeds the headset's origin so its reported pose matches a known field pose.
   *
   * @param fieldToQuest the true field pose <b>of the headset</b>, not of the robot; the caller
   *     must pre-compose {@code Constants.Vision.kRobotToQuest}
   */
  public default void setPose(Pose3d fieldToQuest) {}

  /**
   * Runs the questnavlib command and response machinery. Must be called once per loop, first thing
   * in {@code Vision.periodic()}; the library's event callbacks only fire from here.
   */
  public default void commandPeriodic() {}
}
