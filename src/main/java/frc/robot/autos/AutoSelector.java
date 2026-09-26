package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.FaultReporter;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
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
 *
 * <p>M13 step 7 changed one thing about the skeleton's design. It kept the display name in a SECOND
 * dashboard chooser with the same option order. Two widgets cannot keep their selections in step:
 * the drive team would have had to pick the routine twice, and a mismatch would have previewed one
 * routine's start while running another. There is one chooser now, of routine NAMES. The name is
 * the logged selection, and it keys both the routine and the path whose start the preview shows.
 */
public class AutoSelector {
  /** The chooser's dashboard key. */
  static final String CHOOSER_KEY = "Auto Choice";

  /** The default, and the name {@link #get()} falls back to. */
  static final String DO_NOTHING = "Do Nothing";

  private final LoggedDashboardChooser<String> chooser = new LoggedDashboardChooser<>(CHOOSER_KEY);
  private final AutoFactory factory;
  private final AutoRoutines routines;

  /** Each option's builder, by name, in chooser order. */
  private final Map<String, Supplier<Command>> builders = new LinkedHashMap<>();

  /** The path each routine starts on, by name; absent for a routine with no start. */
  private final Map<String, String> startPaths = new LinkedHashMap<>();

  private String lastSelectedName = "";

  /** The placement error the preview last computed, meters; NaN when there is no start to check. */
  private double startPoseErrorMeters = Double.NaN;

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
    // IMPLEMENTATION (was TODO: "Call populate().")
    populate();
    // Registered once, polled every loop by the fault reporter. A warning, not an error: gate term
    // 8 refuses to score on any error, and a robot standing off its tape can still shoot.
    FaultReporter.getInstance()
        .register(
            "Robot is not on the selected auto's start (see AutoSelector/StartPoseError)",
            AlertType.kWarning,
            this::isMisplaced);
  }

  /**
   * Fills the chooser.
   *
   * <p><b>Outputs:</b> adds one option per routine, and records the path each one starts on.
   */
  private void populate() {
    // IMPLEMENTATION (was TODO: "chooser.addDefaultOption(\"Do Nothing\", routines.doNothing()),
    // then addOption for each of the eight routines, plus a matching startPoseNameChooser option
    // per routine naming the path whose start pose to preview.")
    //
    // One chooser rather than two; see the class comment. Do Nothing is the DEFAULT: a
    // mis-selected auto that does nothing loses points, one that drives into a partner can lose
    // the match.
    builders.put(DO_NOTHING, routines::doNothing);
    chooser.addDefaultOption(DO_NOTHING, DO_NOTHING);
    addRoutine("Leave", "Leave", routines::driveForward);
    addRoutine("Preload Only", "Center_Preload", routines::preloadOnly);
    addRoutine("Center 3", "Center_ToPiece1", routines::centerThreePiece);
    addRoutine("Center 5", "Center_ToPiece1", routines::centerFivePiece);
    addRoutine("Amp 3", "Amp_ToPiece1", routines::ampSideThreePiece);
    addRoutine("Source 3", "Source_ToPiece1", routines::sourceSideThreePiece);
    addRoutine("Source Steal", "Source_Steal", routines::sourceSideSteal);
    addRoutine("Scavenge", "Center_ToPiece1", routines::dynamicScavenge);
    // Section 8.6 step 3's diagnostic. Tuning mode only, like the characterization entries: it is
    // for the practice field, and it cannot be picked by mistake at an event.
    if (Constants.kTuningMode) {
      addRoutine("Center 3 (stationary)", "Center_ToPiece1", routines::centerThreePieceStationary);
    }
  }

  /**
   * Adds a routine to the chooser, and builds it once so a missing path is reported at boot.
   *
   * <p>The build is thrown away. Each selection gets a fresh command from {@link #get()}; this one
   * exists only so that a renamed path file puts a warning on the dashboard in the queue line,
   * rather than at the start of a match.
   *
   * @param name the chooser's display name
   * @param startPath the path the routine starts on, for the preview
   * @param builder builds a fresh instance of the routine
   *     <p><b>Outputs:</b> adds the option and records its start path.
   */
  private void addRoutine(String name, String startPath, Supplier<Command> builder) {
    builder.get();
    builders.put(name, builder);
    startPaths.put(name, startPath);
    chooser.addOption(name, name);
  }

  /**
   * Appends an extra option, for example a characterization routine.
   *
   * @param name the display name
   * @param command the command to run
   *     <p><b>Outputs:</b> adds the option to the chooser.
   */
  public void addOption(String name, Command command) {
    // IMPLEMENTATION (was TODO: "chooser.addOption(name, command). Public so RobotContainer can
    // append the CharacterizationCommands entries only when Constants.kTuningMode is true.")
    //
    // Scheduled on its own, never composed, so the one instance can be run any number of times.
    builders.put(name, () -> command);
    chooser.addOption(name, name);
  }

  /**
   * The currently-selected routine, freshly built.
   *
   * @return the selected command; never null
   */
  public Command get() {
    // IMPLEMENTATION (was TODO: "Command selected = chooser.get(); return selected == null ?
    // Commands.none() : selected. NEVER return null here".)
    //
    // Built here, when autonomous starts, so every run gets a fresh instance: a command may belong
    // to only one composition, and re-running a practice auto must not throw.
    Supplier<Command> builder = builders.get(getSelectedName());
    return builder == null ? Commands.none() : builder.get();
  }

  /**
   * The display name of the current selection.
   *
   * @return the selected routine's name; {@code "Do Nothing"} if nothing is selected
   */
  public String getSelectedName() {
    // IMPLEMENTATION (was TODO: "Track the name through the parallel startPoseNameChooser
    // instead, and log it as \"AutoSelector/Selected\".")
    //
    // The chooser's value IS the name, so there is nothing to track.
    String selected = chooser.get();
    String name = selected == null ? DO_NOTHING : selected;
    Logger.recordOutput("AutoSelector/Selected", name);
    return name;
  }

  /**
   * Where the selected routine expects the robot to start.
   *
   * @return the alliance-flipped starting pose, or empty for a routine that has none, or whose
   *     start path cannot be read
   */
  public Optional<Pose2d> getSelectedStartingPose() {
    // IMPLEMENTATION (was TODO: "factory.getStartingPose(getSelectedName()) when the selection
    // maps to a .auto file, otherwise derive it from the first path in the routine.")
    //
    // Always from the first path: every routine here is built from paths, and one starts where its
    // first path does, so no companion .auto file is needed just to declare a start.
    String startPath = startPaths.get(getSelectedName());
    return startPath == null ? Optional.empty() : factory.getPathStartingPose(startPath);
  }

  /**
   * Publishes the starting-pose preview and the placement error.
   *
   * <p><b>Outputs:</b> logs {@code AutoSelector/StartingPose} and {@code
   * AutoSelector/StartPoseError}, and updates the error the misplacement warning reads. Called
   * every {@code disabledPeriodic}.
   */
  public void logStartingPose() {
    // IMPLEMENTATION (was TODO: "Logger.recordOutput(\"AutoSelector/StartingPose\", ...) and
    // Logger.recordOutput(\"AutoSelector/StartPoseError\", the distance from
    // RobotState.getInstance().getEstimatedPose() to that starting pose). Fire a FaultReporter
    // WARNING when that error exceeds Constants.Auto.kStartPoseWarnMeters".)
    //
    // With no start to compare against -- Do Nothing, or an unreadable path -- the error is NaN,
    // which never exceeds the bound, so the warning stays quiet rather than nagging about a routine
    // that does not care where it stands.
    Optional<Pose2d> start = getSelectedStartingPose();
    startPoseErrorMeters =
        start
            .map(
                pose ->
                    RobotState.getInstance()
                        .getEstimatedPose()
                        .getTranslation()
                        .getDistance(pose.getTranslation()))
            .orElse(Double.NaN);
    Logger.recordOutput("AutoSelector/StartingPose", start.orElse(Pose2d.kZero));
    Logger.recordOutput("AutoSelector/StartPoseError", startPoseErrorMeters);
  }

  /**
   * The placement error the preview last computed.
   *
   * <p>Added for M13 step 7, so the checkpoint's clause on {@code AutoSelector/StartPoseError} can
   * be asserted: the logger records nothing under test.
   *
   * @return meters between the estimated pose and the selected routine's start, or NaN when there
   *     is no start to compare against
   */
  public double getStartPoseErrorMeters() {
    return startPoseErrorMeters;
  }

  /**
   * Whether the robot stands further from the selected start than the warning allows.
   *
   * @return true when the last preview's error exceeds {@code Constants.Auto.kStartPoseWarnMeters}
   */
  public boolean isMisplaced() {
    return startPoseErrorMeters > Constants.Auto.kStartPoseWarnMeters.get();
  }

  /**
   * The chooser's option names, in order.
   *
   * <p>Added for M13 step 7, for the tests: a {@code SendableChooser} does not list its options.
   *
   * @return every selectable name, the default first
   */
  List<String> getOptionNames() {
    return List.copyOf(builders.keySet());
  }

  /**
   * Refreshes the chooser from the dashboard, as AdvantageKit's logger does every loop.
   *
   * <p>Added for M13 step 7, for the tests: the logger is not running under test, so nothing else
   * would read a selection made on the dashboard.
   *
   * <p><b>Outputs:</b> updates the chooser's selected value.
   */
  void refreshChooser() {
    chooser.periodic();
  }

  /**
   * Whether the selection changed since the last check.
   *
   * @return true if the selected routine is different from the previous call
   */
  public boolean selectionChanged() {
    // IMPLEMENTATION (was TODO: "Compare getSelectedName() against lastSelectedName, update it,
    // and return whether it moved.")
    String name = getSelectedName();
    boolean changed = !name.equals(lastSelectedName);
    lastSelectedName = name;
    return changed;
  }
}
