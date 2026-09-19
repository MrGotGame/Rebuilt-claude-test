package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Wraps AdvantageKit's {@link LoggedDashboardChooser} -- logged, and therefore replayable, so you
 * can tell from a log which routine the drive team actually selected -- and adds the one thing the
 * stock chooser lacks: a <b>preview</b>.
 *
 * <p>Every disabled loop it publishes the selected routine's expected starting pose, so
 * AdvantageScope's 2D field shows a ghost robot where the routine thinks you are standing.
 * Comparing that ghost against the live estimated pose in the queue line catches a robot placed 30
 * cm off the tape, which is the single most common cause of an auto that "worked in practice."
 */
public class AutoSelector {
  private final LoggedDashboardChooser<Command> chooser =
      new LoggedDashboardChooser<>("Auto Choice");
  private final LoggedDashboardChooser<String> startPoseNameChooser =
      new LoggedDashboardChooser<>("Auto Start Path");
  private final AutoFactory factory;
  private final AutoRoutines routines;
  private String lastSelectedName = "";

  /**
   * Builds the chooser and populates it.
   *
   * <p>Construct this after {@link AutoFactory}, so the named commands are already registered --
   * building a routine whose markers are not yet registered throws.
   *
   * @param factory the configured PathPlanner factory
   * @param routines the routine library
   */
  public AutoSelector(AutoFactory factory, AutoRoutines routines) {
    this.factory = factory;
    this.routines = routines;
    // TODO: Call populate().
  }

  /**
   * Fills both choosers.
   *
   * <p><b>Outputs:</b> adds one option per routine, plus a parallel start-path option for the
   * preview.
   */
  private void populate() {
    // TODO: chooser.addDefaultOption("Do Nothing", routines.doNothing()), then addOption for each
    // of the eight routines, plus a matching startPoseNameChooser option per routine naming the
    // path whose start pose to preview.
    // Keep the two choosers' option ORDER identical, so the preview always matches the selection.
  }

  /**
   * Appends an extra option, for example a characterization routine.
   *
   * @param name the display name
   * @param command the command to run
   *     <p><b>Outputs:</b> adds the option to the chooser.
   */
  public void addOption(String name, Command command) {
    // TODO: chooser.addOption(name, command). Public so RobotContainer can append the
    // CharacterizationCommands entries only when Constants.kTuningMode is true.
  }

  /**
   * The currently-selected routine.
   *
   * @return the selected command; never null
   */
  public Command get() {
    // TODO: Command selected = chooser.get(); return selected == null ? Commands.none() : selected.
    // NEVER return null here -- Robot.autonomousInit would then silently do nothing, and the first
    // anyone would know about it is fifteen wasted seconds of a real match.
    return Commands.none();
  }

  /**
   * The display name of the current selection.
   *
   * @return the selected routine's name, or an empty string if nothing is selected
   */
  public String getSelectedName() {
    // TODO: Note that chooser.getSendableChooser().getSelected() is NOT the display name. Track the
    // name through the parallel startPoseNameChooser instead, and log it as
    // "AutoSelector/Selected".
    return "";
  }

  /**
   * Where the selected routine expects the robot to start.
   *
   * @return the alliance-flipped starting pose, or empty if it cannot be determined
   */
  public Optional<Pose2d> getSelectedStartingPose() {
    // TODO: factory.getStartingPose(getSelectedName()) when the selection maps to a .auto file,
    // otherwise derive it from the first path in the routine. Already alliance-flipped by
    // AutoFactory.
    return Optional.empty();
  }

  /**
   * Publishes the starting-pose preview and placement error.
   *
   * <p><b>Outputs:</b> logs {@code AutoSelector/StartingPose} and {@code
   * AutoSelector/StartPoseError}; raises a warning alert when the robot is misplaced. Called every
   * {@code disabledPeriodic}.
   */
  public void logStartingPose() {
    // TODO: Logger.recordOutput("AutoSelector/StartingPose",
    //     getSelectedStartingPose().orElse(Pose2d.kZero));
    // and Logger.recordOutput("AutoSelector/StartPoseError", the distance from
    // RobotState.getInstance().getEstimatedPose() to that starting pose).
    // Fire a FaultReporter WARNING when that error exceeds Constants.Auto.kStartPoseWarnMeters --
    // a warning in the queue line is free, and a blown auto is not.
  }

  /**
   * Whether the selection changed since the last check.
   *
   * @return true if the selected routine is different from the previous call
   */
  public boolean selectionChanged() {
    // TODO: Compare getSelectedName() against lastSelectedName, update it, and return whether it
    // moved. Robot uses this to avoid re-logging the preview at 50 Hz when nothing has changed.
    return false;
  }
}
