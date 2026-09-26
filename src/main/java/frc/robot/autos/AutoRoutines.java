package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * The composable routine builder.
 *
 * <p>A {@code .auto} file drawn in the PathPlanner GUI is the right tool for <i>geometry</i>. It is
 * the wrong tool for <i>logic</i> such as loops, a sequence built from a list of path names, or a
 * timeout read from a tunable when the routine starts. So every routine here is a small Java
 * composition of {@link AutoFactory#followPath} segments and superstructure commands, with the GUI
 * owning only path shapes.
 *
 * <p>Each routine is a {@code public Command} that returns a <b>fresh</b> instance on every call,
 * never a cached one. A {@code Command} may only belong to one composition at a time, so
 * re-selecting an auto in the chooser would otherwise throw.
 *
 * <p>Three building blocks do the heavy lifting. {@link #driveAndIntake} follows a path while the
 * superstructure intakes; {@link #driveAndShoot} follows a path while the superstructure scores on
 * the move -- which is where shoot-while-moving in autonomous actually happens; and {@link
 * #driveIntakeAndShoot} does both at once. The turret and the solver do not care that the chassis
 * velocity came from PathPlanner rather than a joystick.
 *
 * <p>M13 step 5 fixed one rule for all of them: a block HOLDS whatever it asks of the shooter side
 * for exactly the length of its leg and hands back {@code DEFAULT} when the leg ends, however it
 * ends. The skeleton's one-shot requests outlived their legs. There is no {@code aimTurretAt}: the
 * superstructure overwrites any direct turret request on the next loop, and {@link #spinUpWhile}
 * already pre-aims, because spinning up aims the turret at the solved shot.
 *
 * <p>M10b step 9 (plan section 2.4) put these on the hopper model. The robot starts with the intake
 * stowed and 8 pieces aboard, and pieces fall out of a stowed intake, so every routine opens with
 * {@link #waitUntilIntakeRetaining()}. There is no piece sensor, so no routine waits for a piece or
 * branches on one: a missed pickup cannot be detected, and a moving scoring leg over an empty
 * hopper costs no stop.
 */
public class AutoRoutines {
  private final AutoFactory factory;
  private final Superstructure superstructure;

  /**
   * Builds the routine library.
   *
   * @param factory the configured PathPlanner factory
   */
  public AutoRoutines(AutoFactory factory) {
    this.factory = factory;
    this.superstructure = factory.getSuperstructure();
    // Deliberately builds NO Command here. Commands are built lazily in the accessors so that each
    // chooser selection gets a fresh instance.
  }

  // ------------------------------------------------------------------------- building blocks ----

  /**
   * Follows a path with the intake running, then keeps it running for a short fixed dwell.
   *
   * @param pathName the path file's name, without extension
   * @return the composed command
   */
  public Command driveAndIntake(String pathName) {
    // IMPLEMENTATION (was TODO: "Commands.deadline(factory.followPath(pathName),
    // superstructure.intakeWhileHeldCommand()).andThen(Commands.deadline(Commands.waitSeconds(
    // Constants.Auto.kIntakeSettleSec.get()), superstructure.intakeWhileHeldCommand())).")
    //
    // One intake request spanning the path AND the dwell, rather than the TODO's two back to back:
    // the same effect, with no instant between them in which the request is released. The dwell is
    // read when it starts, so a retune takes effect without a reboot. Releasing the request at the
    // end stops the roller and leaves the intake DOWN (section 2.4); it asks nothing of the shooter
    // side, so a spin-up around this leg carries straight through it.
    return Commands.deadline(
        Commands.sequence(factory.followPath(pathName), intakeSettle()),
        superstructure.intakeWhileHeldCommand());
  }

  /**
   * The fixed dwell at the end of a pickup leg.
   *
   * @return a wait of {@code Constants.Auto.kIntakeSettleSec}, read when it starts
   */
  private Command intakeSettle() {
    return Commands.defer(
        () -> Commands.waitSeconds(Constants.Auto.kIntakeSettleSec.get()), Set.of());
  }

  /**
   * Follows a path with the intake running while scoring on the move: sweep and fire at once.
   *
   * <p>Added for M10b step 9. This is the hopper robot's natural unit: drive through a group of
   * pieces with the intake down and fire whatever is aboard as it goes, with no stop and no
   * separate pickup leg. It relies on the superstructure's two orthogonal requests -- the intake
   * and the shooter side -- so it needs no auto-specific code.
   *
   * @param pathName the path file's name, without extension
   * @return the composed command
   */
  public Command driveIntakeAndShoot(String pathName) {
    // IMPLEMENTATION (was TODO: "Commands.deadline(factory.followPath(pathName),
    // superstructure.intakeWhileHeldCommand(), superstructure.setWantedStateCommand(
    // AUTO_SCORE_ON_MOVE)).andThen(superstructure.setWantedStateCommand(DEFAULT)).")
    //
    // holdWantedState in place of the one-shot and the trailing DEFAULT: the same outcome when the
    // path finishes, and still DEFAULT when something cancels the leg partway.
    return Commands.deadline(
        factory.followPath(pathName),
        superstructure.intakeWhileHeldCommand(),
        factory.holdWantedState(Superstructure.WantedState.AUTO_SCORE_ON_MOVE));
  }

  /**
   * Follows a path while scoring on the move, with no stop inserted.
   *
   * @param pathName the path file's name, without extension
   * @return the composed command
   */
  public Command driveAndShoot(String pathName) {
    // IMPLEMENTATION (was TODO: "Commands.deadline(factory.followPath(pathName),
    // superstructure.setWantedStateCommand(Superstructure.WantedState.AUTO_SCORE_ON_MOVE));")
    //
    // Held, not set once. The TODO's one-shot left AUTO_SCORE_ON_MOVE standing after the path, so
    // the robot kept shooting into the next pickup leg, and any spin-up wrapped around that leg was
    // overwritten. The leg now scores for exactly as long as its path. This is shoot-on-the-move in
    // autonomous, with nothing auto-specific in it: the path supplies the chassis velocity,
    // RobotState measures it, and the solver's virtual goal does the rest (section 8.5).
    return Commands.deadline(
        factory.followPath(pathName),
        factory.holdWantedState(Superstructure.WantedState.AUTO_SCORE_ON_MOVE));
  }

  /**
   * Stops and takes a stationary shot. Used for the preload, from a pre-aimed start.
   *
   * @param timeoutSec how long to allow for the shot, seconds, read when the shot starts
   * @return the composed command
   */
  public Command shootInPlace(DoubleSupplier timeoutSec) {
    // IMPLEMENTATION (was TODO: "Commands.runOnce(drive::stop) followed by
    // factory.scoreStationaryCommand().withTimeout(timeoutSec).")
    //
    // The timeout is a supplier read when the shot starts, not a number fixed when the routine is
    // built at boot, so a retuned bound takes effect without a reboot. The stop requires the
    // drivetrain, so it also ends whatever else was driving it; a stationary shot needs the chassis
    // still, and the gate will not open until it is.
    return Commands.sequence(
        Commands.runOnce(factory.getDrive()::stop, factory.getDrive()),
        Commands.defer(
            () -> factory.scoreStationaryCommand().withTimeout(timeoutSec.getAsDouble()),
            Set.of()));
  }

  /**
   * Runs an inner command with the flywheel already spinning up.
   *
   * @param inner the command to run
   * @return the composed command
   */
  public Command spinUpWhile(Command inner) {
    // IMPLEMENTATION (was TODO: "Commands.deadline(inner,
    // superstructure.setWantedStateCommand(Superstructure.WantedState.SPIN_UP));")
    //
    // The TODO's form had two faults. Its SPIN_UP was a one-shot, so it outlived the leg: the
    // flywheel went on spinning, and the turret aiming, after the pickup ended. And a deadline
    // group starts its deadline FIRST, so a leg whose first action is a shooter-side request --
    // spinUpWhile(driveAndShoot(...)), say -- had that request overwritten, in the same instant, by
    // the outer SPIN_UP, and never scored. The plan's own preloadOnly escaped the second fault only
    // by accident: shootInPlace begins with a drive stop, which puts its SCORE_STATIONARY one step
    // later. (M13's planning notes said preloadOnly never fired; a mutation run showed it did.)
    //
    // So SPIN_UP is set BEFORE the inner command starts, not beside it. Anything the inner leg asks
    // of the shooter side then simply replaces it, whatever order the leg does things in, and a
    // pickup leg, which asks nothing, runs spinning. Whatever the inner leg left standing is handed
    // back as DEFAULT when it ends.
    return Commands.sequence(
            superstructure.setWantedStateCommand(Superstructure.WantedState.SPIN_UP), inner)
        .finallyDo(() -> superstructure.setWantedState(Superstructure.WantedState.DEFAULT));
  }

  /**
   * Folds a list of paths into an alternating drive-and-intake, drive-and-shoot sequence.
   *
   * @param pathNames the path files to chain, in order
   * @param shootBetween true to score between pickups, false to simply follow each path
   * @return the composed command
   */
  public Command sequenceOf(List<String> pathNames, boolean shootBetween) {
    // IMPLEMENTATION (was TODO: "Fold pathNames into a sequence alternating driveAndIntake and
    // driveAndShoot, or plain followPath when shootBetween is false.")
    //
    // With shootBetween, the paths alternate pickup, score, pickup, score, starting with a pickup,
    // and each pickup runs spinning up (section 8.5), so the flywheel is at speed as the scoring
    // path begins. Built into a fresh array each call, for the same reason every routine is fresh.
    Command[] legs = new Command[pathNames.size()];
    for (int i = 0; i < legs.length; i++) {
      String path = pathNames.get(i);
      if (!shootBetween) {
        legs[i] = factory.followPath(path);
      } else if (i % 2 == 0) {
        legs[i] = spinUpWhile(driveAndIntake(path));
      } else {
        legs[i] = driveAndShoot(path);
      }
    }
    return Commands.sequence(legs);
  }

  /**
   * Waits until the intake is low enough to keep pieces aboard, or a short timeout.
   *
   * <p>Added for M10b step 9 (plan section 2.4). Every routine starts with this, before its first
   * path. The rules require the robot to start with the intake STOWED and 8 pieces aboard, pieces
   * fall out of a stowed intake, and {@code Superstructure.onEnable()} deploys it at {@code
   * autonomousInit} -- so a routine that drove off at once would shed its preload on the way.
   *
   * @return a command that ends when {@code Intake.isRetainingPieces()} is true, or after {@code
   *     Constants.Auto.kIntakeRetainTimeoutSec}, whichever is first
   */
  public Command waitUntilIntakeRetaining() {
    // The timeout exists so that a dead pivot costs the auto a second, not the whole period. It is
    // read inside Commands.defer, when the routine reaches this step, rather than when the routine
    // is built: routines are built once, when the chooser is populated at robot init, and a timeout
    // captured then would ignore a retune between matches. The same reason, and the same shape, as
    // Superstructure.scoreUntilEmptyCommand().
    //
    // isRetainingPieces() rather than "the intake is DEPLOYED": the feed-assist angle lies between
    // stow and deploy and already retains, so the wait ends as the arm passes it on the way down
    // instead of waiting for the rest of the travel.
    return waitUntilRetaining(factory.getIntake());
  }

  /**
   * The wait {@link #waitUntilIntakeRetaining()} builds, as a function of the intake alone.
   *
   * <p>Split out in M13 step 4, when the factory began configuring PathPlanner and registering its
   * markers in its constructor. The wait reads nothing but the intake, so it can be tested on an
   * intake alone, which is how its three cases in {@code AutoRoutinesTest} have always run.
   *
   * @param intake the intake to wait on
   * @return a command that ends when the intake retains pieces, or after {@code
   *     Constants.Auto.kIntakeRetainTimeoutSec}, whichever is first
   */
  static Command waitUntilRetaining(Intake intake) {
    return Commands.defer(
        () ->
            Commands.waitUntil(intake::isRetainingPieces)
                .withTimeout(Constants.Auto.kIntakeRetainTimeoutSec.get()),
        Set.of());
  }

  // ------------------------------------------------------------------------------- routines ----

  /**
   * The safe default.
   *
   * @return a command that does nothing
   */
  public Command doNothing() {
    // MUST remain the chooser's default. A mis-selected auto that does nothing loses 15 points; one
    // that drives into an alliance partner can lose the match.
    return Commands.none();
  }

  /**
   * The zero-risk mobility auto, and the smoke test for the whole PathPlanner wiring.
   *
   * @return the routine
   */
  public Command driveForward() {
    // TODO: waitUntilIntakeRetaining() -> factory.followPath("Leave"). If this one works,
    // AutoBuilder, RobotConfig and the pose reset are all wired correctly; if it does not, nothing
    // else in this file will.
    return Commands.none();
  }

  /**
   * Fires the 8-piece preload, then leaves. The fallback routine when vision is down.
   *
   * @return the routine
   */
  public Command preloadOnly() {
    // TODO: resetPoseToPathStart("Center_Preload") -> waitUntilIntakeRetaining() ->
    // spinUpWhile(shootInPlace(<long enough for all 8 pieces -- the old 2.0 s was a one-piece
    // timeout>)) ->
    // factory.logPreloadFiredCommand() -> followPath("Center_Preload"). This routine depends on
    // nothing but odometry, which is exactly why it is the one to select when the cameras are
    // misbehaving. Auto/PreloadFired should read 8; see logPreloadFiredCommand() for what it means
    // when it does not.
    return Commands.none();
  }

  /**
   * Center start, preload plus two floor pieces.
   *
   * @return the routine
   */
  public Command centerThreePiece() {
    // TODO: resetPose -> waitUntilIntakeRetaining() -> shootInPlace ->
    // factory.logPreloadFiredCommand() -> driveAndIntake("Center_ToPiece1") ->
    // driveAndShoot("Center_Score1") -> driveAndIntake("Center_ToPiece2") ->
    // driveAndShoot("Center_Score2"). There is no bailout after a pickup: a missed pickup cannot be
    // detected without a piece sensor, and a moving scoring leg over an empty hopper costs no stop.
    // Once the field layout is known, consider driveIntakeAndShoot sweeps in place of the
    // pickup/score pairs (plan section 8.3).
    return Commands.none();
  }

  /**
   * The showcase routine: five pieces, every scoring leg taken on the move.
   *
   * @return the routine
   */
  public Command centerFivePiece() {
    // TODO: centerThreePiece() extended with driveAndIntake("Center_ToPiece3") ->
    // driveAndShoot("Center_Score3"), and never a stop on a scoring leg.
    // Expect to need kAccelCompensationScalar above 0 for this one: the path's acceleration
    // segments are precisely where the moving shots go wide.
    return Commands.none();
  }

  /**
   * Amp-side mirror of the three-piece routine.
   *
   * @return the routine
   */
  public Command ampSideThreePiece() {
    // TODO: The left/amp-side mirror of centerThreePiece using the Amp_* paths, opening with
    // waitUntilIntakeRetaining() like every routine. Write it BLUE-ONLY; AllianceFlipUtil and
    // AutoBuilder's shouldFlip callback handle red automatically.
    return Commands.none();
  }

  /**
   * Source-side mirror of the three-piece routine.
   *
   * @return the routine
   */
  public Command sourceSideThreePiece() {
    // TODO: The right/source-side mirror using the Source_* paths, opening with
    // waitUntilIntakeRetaining() like every routine.
    return Commands.none();
  }

  /**
   * Crosses to the opponent's staging area to deny pieces. High risk, high reward.
   *
   * @return the routine
   */
  public Command sourceSideSteal() {
    // TODO: waitUntilIntakeRetaining(), then drive across on "Source_Steal" with the intake
    // requested, and return to shoot -- a natural driveIntakeAndShoot sweep on the way back. Only
    // run this when the alliance has agreed to it beforehand.
    return Commands.none();
  }

  /**
   * The vision-driven routine: chase whatever game pieces the intake camera can actually see.
   *
   * @return the routine
   */
  public Command dynamicScavenge() {
    // TODO: waitUntilIntakeRetaining(), then loop up to Constants.Auto.kMaxScavengeCycles: if
    // vision.getGamePieceFieldPose() is present, AutoBuilder.pathfindToPose(that pose,
    // AutoFactory.defaultPathfindConstraints(), 0.0) under superstructure.intakeWhileHeldCommand();
    // otherwise drive to the next static waypoint. Then driveAndShoot back.
    // This depends on the intake Limelight's game-piece pipeline being properly tuned, so keep it
    // behind its own chooser entry that students can simply avoid at an early event.
    return Commands.none();
  }

  /**
   * Placeholder slot that {@code RobotContainer} replaces with characterization entries when tuning
   * mode is enabled.
   *
   * @return a command that does nothing
   */
  public Command characterizationPlaceholder() {
    return Commands.none();
  }
}
