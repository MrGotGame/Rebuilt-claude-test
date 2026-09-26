package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * The M13 simulation checkpoint: every routine, run end to end on the whole simulated robot.
 *
 * <p>Plan section 10.2's M13 checkpoint asks that all eight routines run in simulation, that none
 * moves before the intake is deployed ({@code Superstructure/StowedWhileMoving} never true), and
 * that {@code Auto/PreloadFired} reads 8 at the end of {@code Preload Only}. AdvantageScope cannot
 * be driven from the agent's sandbox, so, as M10b's and M12's checkpoints did, it runs as JUnit.
 *
 * <p>The paths are synthetic. The real ones are drawn in the PathPlanner GUI with the REBUILT field
 * loaded, and nothing in the plan says where a piece is staged, so these are only shaped like the
 * real ones: the goal is at the field origin, the shooting position is 3 m in front of it, pickups
 * end slow and scoring paths end moving (section 8.4's second drawing rule). What the checkpoint
 * proves is that every routine is wired -- each leg runs, in order, with the right requests, to the
 * right place -- not that the geometry is any good.
 *
 * <p>Every routine also runs on the red alliance. The plan's risk #4 says to force red in
 * simulation and re-run every auto in M13, not at the event.
 */
public class AutoCheckpointTest {
  private static final double LOOP_SEC = SimulatedAutoRobot.LOOP_SEC;

  /** The shooting position: 3 m in front of the goal, facing it. */
  static final Pose2d SHOOT = new Pose2d(-3.0, 0.0, Rotation2d.kZero);

  /**
   * The amp-side start, facing the goal.
   *
   * <p>Every start faces the goal because the preload is a stationary shot from a pre-aimed start
   * (section 4.12's shootInPlace). It matters in simulation too: on its placeholder gains the
   * simulated turret rings for about eight seconds after a large move, twice the preload's bound.
   */
  private static final Pose2d AMP = facingGoal(-3.0, 2.0);

  /** The source-side start, facing the goal. */
  private static final Pose2d SOURCE = facingGoal(-3.0, -2.0);

  /**
   * A pose at a point, facing the goal at the field origin.
   *
   * @param x field x, meters
   * @param y field y, meters
   * @return the pose
   */
  private static Pose2d facingGoal(double x, double y) {
    return new Pose2d(x, y, new Rotation2d(-x, -y));
  }

  /** Speed limit on a pickup path, m/s. */
  private static final double PICKUP_SPEED = 2.0;

  /** Speed at which a pickup path ends, m/s: slow, so the intake can pull pieces in. */
  private static final double PICKUP_END_SPEED = 0.5;

  /** Speed at which a scoring path ends, m/s: still moving, which is the point of a turret. */
  private static final double SCORE_END_SPEED = 1.0;

  /** The autonomous period, seconds. */
  private static final double AUTO_PERIOD_SEC = 15.0;

  /** The robot a case built. */
  private SimulatedAutoRobot robot;

  /** Initialises the HAL and pauses the clock. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();
  }

  /** Stops the robot and releases the HAL. */
  @AfterEach
  void tearDown() {
    if (robot != null) {
      robot.close();
    }
    DriverStationSim.resetData();
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  /**
   * A pickup path: slow at its end.
   *
   * @param name the path name
   * @param from where it starts
   * @param to where it ends
   * @return the path
   */
  private static PathPlannerPath pickup(String name, Pose2d from, Translation2d to) {
    return SimulatedAutoRobot.straightPath(
        name, from, new Pose2d(to, from.getRotation()), PICKUP_SPEED, PICKUP_END_SPEED);
  }

  /**
   * A scoring path back to a shooting position: still moving at its end.
   *
   * @param name the path name
   * @param from where it starts
   * @param to where it ends
   * @return the path
   */
  private static PathPlannerPath score(String name, Translation2d from, Pose2d to) {
    return SimulatedAutoRobot.straightPath(
        name, new Pose2d(from, to.getRotation()), to, PICKUP_SPEED, SCORE_END_SPEED);
  }

  /** Every path the routines name, shaped like section 8.4's, drawn blue. */
  static AutoFactory.PathSource field() {
    Translation2d piece1 = new Translation2d(-4.5, 1.5);
    Translation2d piece2 = new Translation2d(-4.5, -1.5);
    Translation2d piece3 = new Translation2d(-5.0, 0.0);
    Translation2d ampPiece1 = new Translation2d(-4.5, 3.0);
    Translation2d ampPiece2 = new Translation2d(-5.0, 2.0);
    Translation2d sourcePiece1 = new Translation2d(-4.5, -3.0);
    Translation2d sourcePiece2 = new Translation2d(-5.0, -2.0);
    return SimulatedAutoRobot.pathsOf(
        SimulatedAutoRobot.straightPath(
            "Leave", SHOOT, new Pose2d(-5.0, 0.0, Rotation2d.kZero), PICKUP_SPEED, 0.0),
        SimulatedAutoRobot.straightPath(
            "Center_Preload", SHOOT, new Pose2d(-4.5, 0.0, Rotation2d.kZero), PICKUP_SPEED, 0.0),
        pickup("Center_ToPiece1", SHOOT, piece1),
        score("Center_Score1", piece1, SHOOT),
        pickup("Center_ToPiece2", SHOOT, piece2),
        score("Center_Score2", piece2, SHOOT),
        pickup("Center_ToPiece3", SHOOT, piece3),
        score("Center_Score3", piece3, SHOOT),
        pickup("Amp_ToPiece1", AMP, ampPiece1),
        score("Amp_Score1", ampPiece1, AMP),
        pickup("Amp_ToPiece2", AMP, ampPiece2),
        score("Amp_Score2", ampPiece2, AMP),
        pickup("Source_ToPiece1", SOURCE, sourcePiece1),
        score("Source_Score1", sourcePiece1, SOURCE),
        pickup("Source_ToPiece2", SOURCE, sourcePiece2),
        score("Source_Score2", sourcePiece2, SOURCE),
        SimulatedAutoRobot.straightPath(
            "Source_Steal", SOURCE, new Pose2d(-6.0, -2.0, Rotation2d.kZero), 3.0, 0.0));
  }

  /**
   * A stand-in for PathPlanner's pathfinder: a straight path from wherever the robot is.
   *
   * <p>The real pathfinder plans on a background thread against real time, which a paused clock
   * cannot drive. The target is already on the current alliance, so the path is marked not to flip.
   *
   * @return a pathfinder for {@link AutoRoutines}
   */
  private Function<Pose2d, Command> straightPathfinder() {
    return target ->
        Commands.defer(
            () -> {
              PathPlannerPath path =
                  SimulatedAutoRobot.straightPath(
                      "Pathfind", robot.pose(), target, PICKUP_SPEED, 0.0);
              path.preventFlipping = true;
              return AutoBuilder.followPath(path);
            },
            Set.of(robot.drive));
  }

  /** One routine: its chooser name, how to build it, where it starts and where it must end. */
  private record Routine(
      String name,
      Function<AutoRoutines, Command> build,
      Pose2d start,
      Translation2d end,
      boolean firesPreload) {
    @Override
    public String toString() {
      return name;
    }
  }

  /** The eight routines of section 8.3, plus section 8.6 step 3's stationary Center 3. */
  static Stream<Arguments> routines() {
    Translation2d shoot = SHOOT.getTranslation();
    Routine[] all = {
      new Routine("Leave", AutoRoutines::driveForward, SHOOT, new Translation2d(-5.0, 0.0), false),
      new Routine(
          "Preload Only", AutoRoutines::preloadOnly, SHOOT, new Translation2d(-4.5, 0.0), true),
      new Routine("Center 3", AutoRoutines::centerThreePiece, SHOOT, shoot, true),
      new Routine(
          "Center 3 (stationary)", AutoRoutines::centerThreePieceStationary, SHOOT, shoot, true),
      new Routine("Center 5", AutoRoutines::centerFivePiece, SHOOT, shoot, true),
      new Routine("Amp 3", AutoRoutines::ampSideThreePiece, AMP, AMP.getTranslation(), true),
      new Routine(
          "Source 3", AutoRoutines::sourceSideThreePiece, SOURCE, SOURCE.getTranslation(), true),
      new Routine(
          "Source Steal",
          AutoRoutines::sourceSideSteal,
          SOURCE,
          new Translation2d(-6.0, -2.0),
          true),
      // No piece in view, so the scavenge falls back to its static waypoints, the ends of the
      // three Center pickups, and finishes on the third.
      new Routine(
          "Scavenge", AutoRoutines::dynamicScavenge, SHOOT, new Translation2d(-5.0, 0.0), true),
    };
    return Stream.of(all)
        .flatMap(routine -> Stream.of(Arguments.of(routine, false), Arguments.of(routine, true)));
  }

  /**
   * Builds the robot on a routine's start, as the drive team would place it, and runs the routine
   * the way {@code Robot.autonomousInit} does.
   *
   * @param routine the routine
   * @param red true for the red alliance
   * @param gamePieces what the intake camera reports
   * @return the loops the routine took
   */
  private int run(Routine routine, boolean red, Supplier<Optional<Pose2d>> gamePieces) {
    robot = new SimulatedAutoRobot(routine.start(), field(), red);
    robot.fieldPieces = 20;
    AutoRoutines routines = new AutoRoutines(robot.factory, straightPathfinder(), gamePieces);
    Command command = routine.build().apply(routines);
    robot.startAuto(command);
    int limit = (int) (60.0 / LOOP_SEC);
    int loops = robot.runUntilFinished(command, limit);
    assertTrue(loops <= limit, routine + " never finished");
    return loops;
  }

  /**
   * Each routine runs to its end, starts by waiting for the intake, never moves with it up, and
   * ends where its last path does.
   *
   * <p>The end position is what shows every leg ran: a leg that silently did nothing would leave
   * the robot somewhere else. {@code Preload Only} and every routine that opens with a preload must
   * also read 8 for {@code Auto/PreloadFired}, which checks the flywheel's shot detector against
   * the one piece count the robot knows for certain.
   *
   * @param routine the routine
   * @param red true for the red alliance
   */
  @ParameterizedTest(name = "{0}, red={1}")
  @MethodSource("routines")
  void everyRoutineRunsToItsEnd(Routine routine, boolean red) {
    int loops = run(routine, red, Optional::empty);
    System.out.printf(
        "M13 checkpoint: %-22s %-4s %5.1f s, %d pieces fired%n",
        routine, red ? "red" : "blue", loops * LOOP_SEC, robot.piecesFired);

    // The stationary Center 3 is a diagnostic for section 8.6 step 3, not a match routine, and in
    // simulation each of its stops waits out the turret's placeholder-gain ringing.
    if (!routine.name().equals("Center 3 (stationary)")) {
      assertTrue(
          loops * LOOP_SEC <= AUTO_PERIOD_SEC,
          routine + " took " + loops * LOOP_SEC + " s, longer than the autonomous period");
    }
    assertEquals(
        0,
        robot.stowedWhileMovingLoops,
        "moved with the intake above its retaining range (Superstructure/StowedWhileMoving)");
    assertEquals(
        0,
        robot.actedBeforeRetainingLoops,
        routine + " did something before the intake retained; it must start by waiting for it");
    assertTrue(robot.firstMovingLoop >= 0, routine + " never moved");
    assertTrue(robot.retainingWhenFirstMoved, routine + " moved before the intake retained");

    Translation2d end = red ? AllianceFlipUtil.apply(routine.end()) : routine.end();
    assertEquals(
        0.0,
        robot.pose().getTranslation().getDistance(end),
        0.25,
        routine + " ended at " + robot.pose().getTranslation() + ", not " + end);

    if (routine.firesPreload()) {
      assertEquals(
          Optional.of(SimulatedAutoRobot.PRELOAD),
          robot.factory.getPreloadFired().stream().boxed().findFirst(),
          "Auto/PreloadFired");
    } else {
      assertTrue(robot.factory.getPreloadFired().isEmpty(), "logged a preload it never fired");
    }

    robot.step(3);
    assertEquals(
        Superstructure.SystemState.RESTING,
        robot.superstructure.getSystemState(),
        "the routine left a shooter-side request standing");
  }

  /**
   * Do Nothing does nothing: it never moves and never fires.
   *
   * <p>It is the chooser's default, and a mis-selected auto must cost points, not a collision.
   */
  @Test
  void doNothingDoesNothing() {
    robot = new SimulatedAutoRobot(SHOOT, field(), false);
    Command command = robot.routines.doNothing();
    robot.startAuto(command);
    robot.step(250);
    assertEquals(-1, robot.firstMovingLoop, "Do Nothing moved");
    assertEquals(0, robot.piecesFired, "Do Nothing fired");
  }

  /**
   * With a game piece in view, the scavenge goes to it rather than to its static waypoint.
   *
   * <p>The routine's whole reason to exist. The camera reports a piece well off every waypoint, and
   * each cycle must drive there.
   */
  @Test
  void theScavengeChasesAVisiblePiece() {
    Pose2d seen = new Pose2d(-4.0, -2.5, Rotation2d.kZero);
    Routine scavenge =
        new Routine("Scavenge", AutoRoutines::dynamicScavenge, SHOOT, seen.getTranslation(), true);
    run(scavenge, false, () -> Optional.of(seen));
    assertEquals(0.0, robot.pose().getTranslation().getDistance(seen.getTranslation()), 0.25);
  }

  /**
   * The stationary Center 3 takes its scoring shots standing still, and the moving one on the move.
   *
   * <p>Section 8.6 step 3 is only worth running if the two really differ in the one respect the
   * step isolates.
   */
  @Test
  void theStationaryCenterThreeNeverScoresOnTheMove() {
    robot = new SimulatedAutoRobot(SHOOT, field(), false);
    AutoRoutines routines = robot.routines;
    Command stationary = routines.centerThreePieceStationary();
    robot.startAuto(stationary);
    int onTheMove = 0;
    int standing = 0;
    for (int i = 0; i < 3000 && stationary.isScheduled(); i++) {
      robot.step();
      Superstructure.SystemState state = robot.superstructure.getSystemState();
      onTheMove += state == Superstructure.SystemState.SCORING_ON_THE_MOVE ? 1 : 0;
      standing += state == Superstructure.SystemState.SCORING_STATIONARY ? 1 : 0;
    }
    assertFalse(stationary.isScheduled(), "the stationary Center 3 never finished");
    assertEquals(0, onTheMove, "the stationary variant scored on the move");
    assertTrue(standing > 0, "the stationary variant never scored");
  }
}
