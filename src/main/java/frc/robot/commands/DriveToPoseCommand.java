package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/**
 * Auto-align with a pathfinding handoff.
 *
 * <p>Far from the goal -- beyond {@code Constants.Drivetrain.kPathfindHandoffMeters} -- this
 * delegates to PathPlanner's {@code pathfindToPose}, which runs the AD* pathfinder and routes
 * around field obstacles taken from {@code navgrid.json}. Inside that radius it switches to two
 * profiled controllers on X and Y plus one on heading, because pathfinding's terminal accuracy is
 * worse than a direct profiled controller's and its replanning near the goal produces visible
 * twitch.
 *
 * <p>The concrete capability: the driver taps a button anywhere on the field and the robot arrives
 * at a scoring pose, square, at a repeatable few centimetres -- which is what makes a scripted
 * teleop cycle possible at all.
 */
public class DriveToPoseCommand extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> targetSupplier;
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private Pose2d target = Pose2d.kZero;
  private Command pathfindCommand = null;
  private boolean usingPathfind = false;

  /**
   * Builds an auto-align toward a pose that may change between schedulings.
   *
   * <p>NOTE: the three controllers are final, so their construction is pre-filled with placeholder
   * gains. Replace the literals with the {@code Constants.Drivetrain.kAutoAlign*} tunables.
   *
   * @param drive the drivetrain
   * @param targetSupplier supplier of the goal pose, in blue-origin field coordinates
   */
  public DriveToPoseCommand(Drive drive, Supplier<Pose2d> targetSupplier) {
    this.drive = drive;
    this.targetSupplier = targetSupplier;
    this.xController =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3.0, 3.0));
    this.yController =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3.0, 3.0));
    this.thetaController =
        new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(8.0, 20.0));
    // TODO: Seed all three controllers from Constants.Drivetrain.kAutoAlign* tunables instead of
    // the literals above, and call thetaController.enableContinuousInput(-Math.PI, Math.PI) --
    // without that call the robot will take the long way around to any heading near +/-180 degrees.
    addRequirements(drive);
  }

  /**
   * Builds an auto-align toward a fixed pose.
   *
   * @param drive the drivetrain
   * @param target the goal pose, in blue-origin field coordinates
   */
  public DriveToPoseCommand(Drive drive, Pose2d target) {
    this(drive, () -> target);
  }

  /**
   * Captures the goal and prepares the controllers.
   *
   * <p><b>Outputs:</b> resets all three profiled controllers to the robot's current motion, and may
   * schedule a pathfinding command.
   */
  @Override
  public void initialize() {
    // TODO: target = targetSupplier.get(). Reset all three controllers to the CURRENT measurement
    // AND the CURRENT velocity -- for example xController.reset(pose.getX(),
    // speeds.vxMetersPerSecond) -- so the profile starts from the robot's real motion rather than
    // from zero. That is what removes the characteristic lurch at the start of an auto-align.
    // Then decide usingPathfind from the distance to target, and if true schedule
    // AutoBuilder.pathfindToPose(target, AutoFactory.defaultPathfindConstraints(), 0.0).
  }

  /**
   * Runs one control iteration.
   *
   * <p><b>Outputs:</b> commands chassis speeds, or defers to the scheduled pathfinding command.
   */
  @Override
  public void execute() {
    // TODO: While usingPathfind and still outside kPathfindHandoffMeters, let the scheduled
    // pathfind command run and do nothing here. Otherwise cancel it, set usingPathfind = false, and
    // compute field-relative vx/vy/omega from the three controllers evaluated against
    // RobotState.getInstance().getEstimatedPose(), then
    // drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega,
    //     RobotState.getInstance().getRotation()));
    // Log target, error and which mode is active every loop.
  }

  /**
   * Whether the command has settled on the goal.
   *
   * @return true once translation, heading and speed are all inside tolerance
   */
  @Override
  public boolean isFinished() {
    // TODO: Return atGoal(), but require it to hold for TWO CONSECUTIVE LOOPS via a Debouncer, so
    // that a single noisy vision frame cannot end the command early and leave the robot short.
    return false;
  }

  /**
   * Stops the drivetrain and cleans up any pathfinding command.
   *
   * @param interrupted whether the command was interrupted rather than finishing normally
   *     <p><b>Outputs:</b> cancels the pathfind command and stops the drive.
   */
  @Override
  public void end(boolean interrupted) {
    // TODO: Cancel pathfindCommand when non-null, then drive.stop().
  }

  /**
   * Public form of the completion test, so the superstructure can gate a shot on being aligned.
   *
   * @return true when translation error, heading error and chassis speed are all inside tolerance
   */
  public boolean atGoal() {
    // TODO: translation error < kAutoAlignToleranceMeters
    //     && heading error < kAutoAlignHeadingToleranceRad
    //     && chassis speed < kAutoAlignSettledSpeedMetersPerSec.
    return false;
  }

  /**
   * How far the robot still has to go.
   *
   * @return distance from the current estimated pose to the target, in meters
   */
  public double getDistanceToTargetMeters() {
    // TODO: RobotState.getInstance().getEstimatedPose().getTranslation()
    //     .getDistance(target.getTranslation());
    // Log it as "DriveToPose/Distance".
    return 0.0;
  }
}
