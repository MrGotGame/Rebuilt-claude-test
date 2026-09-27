package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutoFactory;
import frc.robot.autos.AutoRoutines;
import frc.robot.autos.AutoSelector;
import frc.robot.commands.CharacterizationCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.floorroller.FloorRoller;
import frc.robot.subsystems.floorroller.FloorRollerIO;
import frc.robot.subsystems.floorroller.FloorRollerIOSim;
import frc.robot.subsystems.floorroller.FloorRollerIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.QuestNavIO;
import frc.robot.subsystems.vision.QuestNavIOReal;
import frc.robot.subsystems.vision.QuestNavIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * The wiring harness for the whole robot.
 *
 * <p>The {@code switch} on {@link Constants#currentMode} selects real, simulated or empty IO
 * implementations for all seven subsystems. {@code RobotState.initialize()} runs before anything
 * can read it, {@link AutoFactory} registers the named commands, and the two-controller binding
 * scheme routes every driver action through {@link Superstructure#setWantedStateCommand} rather
 * than touching subsystems directly -- which is what makes an illegal mechanism combination
 * unrepresentable from a button.
 *
 * <p>SysId and characterization bindings are guarded by {@link Constants#kTuningMode} so they
 * cannot fire at an event.
 */
public class RobotContainer {
  private final Drive drive;
  private final Intake intake;
  private final FloorRoller floorRoller;
  private final Indexer indexer;
  private final Turret turret;
  private final Shooter shooter;
  private final Vision vision;
  private final Superstructure superstructure;

  private final CommandXboxController driver =
      new CommandXboxController(Constants.Controls.kDriverPort);
  private final CommandXboxController operator =
      new CommandXboxController(Constants.Controls.kOperatorPort);

  /**
   * Dashboard toggle that drives the flywheel bench harness, or null outside tuning mode.
   *
   * <p>Not final, and deliberately left unassigned when {@link Constants#kTuningMode} is false, so
   * that a competition robot never publishes this key at all. Nothing reads it except the trigger
   * built in {@link #configureShooterTuningBindings()}.
   */
  private LoggedNetworkBoolean benchSpinUp;

  /** Tuning-mode trigger for one shot-table measurement burst (M11); null outside tuning mode. */
  private LoggedNetworkBoolean measureShotTable;

  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoSelector autoSelector;

  /**
   * Builds every subsystem for the current runtime mode and wires the robot together.
   *
   * <p>NOTE: the IO selection below is pre-filled, because all of these fields are final and Java
   * requires them to be assigned here. Your work is in {@link #configureButtonBindings()} and the
   * subsystems themselves.
   */
  public RobotContainer() {
    // RobotState is initialized FIRST, before any subsystem exists, so that nothing can read a null
    // singleton during construction. getModuleTranslations() is static, so this is safe here.
    RobotState.initialize(new SwerveDriveKinematics(Drive.getModuleTranslations()));

    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOTalonFX());
        floorRoller = new FloorRoller(new FloorRollerIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        turret = new Turret(new TurretIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        vision =
            new Vision(
                new VisionIOLimelight(Constants.Vision.kTurretCameraName),
                new VisionIOLimelight(Constants.Vision.kIntakeCameraName),
                new QuestNavIOReal(),
                turret::getPositionRad,
                turret::getVelocityRadPerSec,
                intake::isAtIntakingAngle);
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOSim());
        floorRoller = new FloorRoller(new FloorRollerIOSim());
        indexer = new Indexer(new IndexerIOSim());
        turret = new Turret(new TurretIOSim());
        shooter = new Shooter(new ShooterIOSim());
        vision =
            new Vision(
                new VisionIOSim(
                    Constants.Vision.kTurretCameraName,
                    () -> RobotState.getInstance().getEstimatedPose()),
                new VisionIOSim(
                    Constants.Vision.kIntakeCameraName,
                    () -> RobotState.getInstance().getEstimatedPose()),
                new QuestNavIOSim(() -> RobotState.getInstance().getEstimatedPose()),
                turret::getPositionRad,
                turret::getVelocityRadPerSec,
                intake::isAtIntakingAngle);
        break;

      default:
        // REPLAY: every IO is an empty implementation, so all values come from the log.
        // WARNING: if you add a subsystem and forget its empty IO here, replay silently produces
        // zeros for it rather than failing -- and zeros look like a real, plausible measurement.
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIO() {});
        floorRoller = new FloorRoller(new FloorRollerIO() {});
        indexer = new Indexer(new IndexerIO() {});
        turret = new Turret(new TurretIO() {});
        shooter = new Shooter(new ShooterIO() {});
        vision =
            new Vision(
                new VisionIO() {},
                new VisionIO() {},
                new QuestNavIO() {},
                turret::getPositionRad,
                turret::getVelocityRadPerSec,
                intake::isAtIntakingAngle);
        break;
    }

    superstructure =
        new Superstructure(drive, intake, floorRoller, indexer, turret, shooter, vision);
    autoFactory = new AutoFactory(drive, superstructure, intake, shooter, vision);
    autoRoutines = new AutoRoutines(autoFactory);
    autoSelector = new AutoSelector(autoFactory, autoRoutines);

    configureButtonBindings();
    configureCharacterizationBindings();
    configureShooterTuningBindings();
    configureShotTableBindings();
  }

  /**
   * Binds every driver and operator control.
   *
   * <p><b>Outputs:</b> sets the drivetrain default command and registers all button triggers.
   */
  private void configureButtonBindings() {
    // ---------------------------------------------------------------- driver: base driving ----

    // IMPLEMENTATION (was TODO: "drive.setDefaultCommand(DriveCommands.joystickDrive(...)) reading
    // RobotState for heading.")
    //
    // The default command runs whenever nothing else has claimed the drivetrain, which is what
    // makes the robot driveable the instant it is enabled in teleop.
    //
    // SIGN CONVENTION -- all three axes are negated, and each for its own reason:
    //   getLeftY()  An Xbox stick reports NEGATIVE when pushed forward, while WPILib field
    //               coordinates put +X forward. Negating maps "stick forward" to "robot forward".
    //   getLeftX()  The stick reports positive to the right; WPILib puts +Y to the LEFT.
    //   getRightX() The stick reports positive to the right, but rotation is
    //               counter-clockwise-positive, so pushing right must produce negative omega.
    // Getting any one of these backwards produces a robot that drives mirrored on that axis, which
    // is obvious on a practice field and very confusing if only one axis is wrong.
    //
    // joystickDriveWithOverride as of M10. The note that used to stand here said this was a
    // one-line change once the superstructure existed, and it was: getRequestedDriveHeading() now
    // returns a heading while the machine is in WAITING_FOR_TURRET, which is how the drivetrain
    // helps a turret reach an aim point outside its travel.
    //
    // The override is an additive nudge and never a takeover -- it applies only while the rotation
    // stick is inside its deadband, so the driver takes rotation back by touching it, with no mode
    // and no button. An assist a driver cannot override is an assist they will not use.
    drive.setDefaultCommand(
        DriveCommands.joystickDriveWithOverride(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            superstructure::getRequestedDriveHeading));

    // Reset the field-relative heading to zero. Held on a button rather than automatic because it
    // redefines which way "forward" means for the driver, and .ignoringDisable(true) lets the team
    // square the robot up on the field before the match starts.
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    Rotation2d.kZero),
                                drive.getRawGyroRotation(),
                                drive.getModulePositions()))
                .ignoringDisable(true));

    // Cross the wheels into an X to resist being pushed. Bound to X for the mnemonic.
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // ------------------------------------------------------------- driver: the mechanisms ----
    //
    // EVERY mechanism action routes through superstructure.setWantedStateCommand(...). Binding a
    // subsystem directly is what lets a button put the robot into a combination the state machine
    // never intended, and it is exactly what this architecture exists to prevent.
    //
    // Every one is held-to-run with an explicit release back to DEFAULT rather than a toggle. A
    // toggle leaves the robot in a scoring state after the driver has moved on to something else,
    // and the state it is left in is invisible from behind the glass.
    //
    // setWantedStateCommand does not require the superstructure, so a held button re-firing cannot
    // interrupt the default drive command.
    // Intake is the one exception to "held with a release back to DEFAULT". Since M10b step 7 it
    // is the superstructure's SECOND mutator, independent of the scoring buttons, so the driver can
    // hold intake and score together -- shoot, move and intake at once (plan section 2.4).
    // Releasing
    // it stops the roller and leaves the intake DOWN, because raising it would drop the hopper's
    // pieces; it must not send DEFAULT, which would cancel a scoring request still being held.
    driver
        .rightTrigger(Constants.Controls.kTriggerThreshold.get())
        .whileTrue(superstructure.intakeWhileHeldCommand());

    driver
        .rightBumper()
        .whileTrue(superstructure.setWantedStateCommand(Superstructure.WantedState.SCORE_ON_MOVE))
        .onFalse(superstructure.setWantedStateCommand(Superstructure.WantedState.DEFAULT));

    driver
        .leftBumper()
        .whileTrue(
            superstructure.setWantedStateCommand(Superstructure.WantedState.SCORE_STATIONARY))
        .onFalse(superstructure.setWantedStateCommand(Superstructure.WantedState.DEFAULT));

    driver
        .leftTrigger(Constants.Controls.kTriggerThreshold.get())
        .whileTrue(superstructure.setWantedStateCommand(Superstructure.WantedState.PASS))
        .onFalse(superstructure.setWantedStateCommand(Superstructure.WantedState.DEFAULT));

    // HOME is onTrue rather than whileTrue: it is a request to re-establish the zeros, not a state
    // to hold, and the superstructure walks itself out to STOWED when both mechanisms report homed.
    // It is also the only thing that clears a latched fault.
    driver.y().onTrue(superstructure.setWantedStateCommand(Superstructure.WantedState.HOME));

    // --------------------------------------------------------------- operator: overrides ----
    //
    // Force shoot, held rather than latched. Holding it means the override cannot be left on by
    // accident, which matters because it bypasses every readiness tolerance: a forced shot that
    // misses and a solver bug look identical in a match video. The superstructure raises an alert
    // and counts forced shots so the two can be told apart afterwards.
    operator
        .rightBumper()
        .onTrue(Commands.runOnce(() -> superstructure.setIgnoreShotTolerances(true)))
        .onFalse(Commands.runOnce(() -> superstructure.setIgnoreShotTolerances(false)));

    // EJECT is on the operator rather than the driver: it is a jam-clearing action somebody
    // watching the mechanism asks for, not something the driver reaches for mid-cycle.
    operator
        .b()
        .whileTrue(superstructure.setWantedStateCommand(Superstructure.WantedState.EJECT))
        .onFalse(superstructure.setWantedStateCommand(Superstructure.WantedState.DEFAULT));

    // STOW is the only request that raises the intake (M10b step 7). onTrue, not whileTrue: the
    // posture it sets persists, and the robot rests with the intake stowed until something deploys
    // it again. It is on the operator because raising the intake drops any pieces in the hopper --
    // a deliberate act, for defence or an endgame, not something to reach for mid-cycle.
    operator.x().onTrue(superstructure.setWantedStateCommand(Superstructure.WantedState.STOW));

    // The driver's cue that the hopper is probably empty: a rumble while holding a scoring button
    // and the superstructure believes nothing is left to fire. A belief, not a measurement -- the
    // robot has no piece sensor -- so it only ever informs; it never stops a shot.
    new Trigger(superstructure::likelyEmpty)
        .and(driver.rightBumper().or(driver.leftBumper()))
        .whileTrue(
            Commands.startEnd(
                () ->
                    driver
                        .getHID()
                        .setRumble(
                            GenericHID.RumbleType.kBothRumble,
                            Constants.Controls.kEmptyRumbleStrength.get()),
                () -> driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));

    // STILL NOT BOUND, and deliberately: the plan's operator layout also lists a manual turret
    // nudge on a stick and a flywheel RPM trim on the D-pad. Neither has a route through the
    // superstructure -- there is no WantedState for "jog the turret by hand", and adding one would
    // mean a fifteenth SystemState that section 4.11 does not have. Binding Turret.setManualVolts
    // or Shooter.setGoal straight from a button is precisely the thing the paragraph above forbids,
    // so they wait for a decision about what the state machine should call them rather than being
    // smuggled in as a special case.
  }

  /**
   * Adds the characterization routines to the auto chooser, but only while tuning.
   *
   * <p><b>Outputs:</b> appends chooser options when {@link Constants#kTuningMode} is true.
   */
  private void configureCharacterizationBindings() {
    // TODO: if (!Constants.kTuningMode) return; then add every routine from
    // CharacterizationCommands to autoSelector as a named option, exactly as the stock template
    // does for the drivetrain. Guarding on kTuningMode is what keeps them off the field.
    //
    // Left for M6 by M13 step 7, deliberately. autoSelector.addOption is ready, but every routine
    // in CharacterizationCommands is still a skeleton returning Commands.none(), so wiring them now
    // would put eight chooser entries in front of the drive team that silently do nothing. Add each
    // one here in the same commit that implements it.
  }

  /**
   * Binds the flywheel bench harness that makes the M4 checkpoint demonstrable, while tuning only.
   *
   * <p>This is a bench harness, not competition control. Every mechanism action in a match routes
   * through {@link Superstructure#setWantedStateCommand}, which is what makes an illegal mechanism
   * combination unrepresentable from a button -- but the superstructure's state machine is a
   * skeleton until M10, so there is currently no route through it that can spin the flywheel. This
   * fills that gap so the milestone can be verified when it is built rather than months later.
   *
   * <p>The trigger is a NetworkTables boolean rather than a controller button, so the checkpoint
   * needs no joystick on either the simulator or the robot. That is not just convenience: polling a
   * controller button on an unpopulated Driver Station port makes WPILib log a "Joystick Button 4
   * on port 1 not available" warning on every loop, which buries the log you are trying to read. It
   * also means the entire checkpoint -- trigger, target speed, and gains -- is driven from one
   * AdvantageScope pane with nothing plugged in.
   *
   * <p>Driving a subsystem directly is acceptable here for exactly one reason: the flywheel has no
   * mechanical interlock with anything. It cannot collide with the intake or the turret and it has
   * no travel limits to crash into, which is why the plan picked it as the first mechanism. Do not
   * copy this pattern for the turret or the intake.
   *
   * <p><b>Outputs:</b> when tuning mode is on, publishes {@code /Tuning/Shooter/BenchSpinUp} and
   * registers one trigger against it.
   */
  private void configureShooterTuningBindings() {
    if (!Constants.kTuningMode) {
      return;
    }

    // Defaults false, and NetworkTables does not persist it across a restart, so every boot starts
    // with the flywheel un-commanded no matter how the last session was left.
    benchSpinUp = new LoggedNetworkBoolean("/Tuning/Shooter/BenchSpinUp", false);

    // LoggedNetworkBoolean implements BooleanSupplier, so it can back a Trigger directly. The
    // command scheduler polls it every loop, which is what turns a dashboard checkbox into edges.
    new Trigger(benchSpinUp)
        // Rising edge: read kBenchTestRpm fresh rather than capturing it once, so editing the
        // target speed and re-toggling is the whole retune loop, with no redeploy. That is half of
        // what the M4 checkpoint asks you to demonstrate; the other half is editing Shooter/Kp in
        // the same pane and watching the response change.
        .onTrue(
            Commands.runOnce(() -> shooter.setGoal(Constants.Shooter.kBenchTestRpm.get()), shooter))
        // Falling edge: back to the idle hold rather than a stop, because IDLE is what the
        // superstructure will actually command between shots and it is the state the checkpoint
        // walks out of. To stop the wheel completely, disable the robot. COAST is deliberately not
        // reachable from here -- it is a pit and brownout state that M10 will command.
        .onFalse(shooter.setWantedStateCommand(Shooter.WantedState.IDLE));
  }

  /**
   * Binds the shot-table measurement burst to a dashboard toggle, while tuning only (M11 step 3).
   *
   * <p>Follows the flywheel bench harness above: a NetworkTables boolean rather than a controller
   * button, so a practice-field session needs nothing plugged into the Driver Station, and the
   * whole loop -- speed, taped distance, trigger -- lives in one AdvantageScope pane. Unlike the
   * bench harness it goes through the superstructure, because a burst moves the turret, the
   * flywheel, the indexer and the floor roller together.
   *
   * <p><b>Outputs:</b> when tuning mode is on, publishes {@code /Tuning/ShotTable/Measure} and
   * registers one trigger against it.
   */
  private void configureShotTableBindings() {
    if (!Constants.kTuningMode) {
      return;
    }
    measureShotTable = new LoggedNetworkBoolean("/Tuning/ShotTable/Measure", false);

    // Rising edge only: one toggle is one burst, and the burst ends on its own when the hopper
    // reads empty. Leaving the toggle on does not repeat it; toggle off and on for the next one.
    // The time limit is the auto scoring bound, which is sized to empty a full 50-piece hopper.
    new Trigger(measureShotTable)
        .onTrue(
            CharacterizationCommands.measureShotTablePoint(
                superstructure,
                shooter,
                Constants.Shot.kMeasureRpm,
                Constants.Shot.kMeasureTapedDistanceMeters,
                Constants.Auto.kScoreTimeoutSec.get()));
  }

  /**
   * The routine to run in autonomous.
   *
   * @return the selected autonomous command; never null
   */
  public Command getAutonomousCommand() {
    // IMPLEMENTATION (was TODO: "return autoSelector.get(). It already guarantees a non-null
    // command.") The selector builds a fresh instance of the selected routine on every call.
    return autoSelector.get();
  }

  /**
   * Sets drivetrain brake or coast.
   *
   * @param brake true for brake, false for coast
   *     <p><b>Outputs:</b> reconfigures the drivetrain, and the turret and intake neutral modes.
   */
  public void setDriveBrakeMode(boolean brake) {
    // IMPLEMENTATION, M14 step 4 (was TODO: "drive.setBrakeMode(brake), plus a turret and intake
    // neutral-mode passthrough -- a disabled robot should be pushable, but a disabled turret should
    // not flop around.")
    //
    // The drivetrain only. The turret and the intake pivot are never passed through: both are
    // configured in brake, and both stay there. A coasting pivot falls under its own weight -- at
    // best onto the floor in the queue line, at worst onto the turret -- and a coasting turret
    // flops. Moving either by hand in the pit is a deliberate act, through their IO's own brake
    // setters, not something a post-match timer should do.
    drive.setBrakeMode(brake);
  }

  /**
   * Called when the robot is disabled.
   *
   * <p><b>Outputs:</b> parks the superstructure and resets subsystem profiles, so that re-enabling
   * does not deliver a step input to a mechanism.
   */
  public void onDisable() {
    // IMPLEMENTATION (was TODO: "superstructure.setWantedState(Superstructure.WantedState.IDLE)
    // and tell every subsystem to reset its motion profiles.")
    //
    // Delegated to Superstructure.onDisable rather than setting the wanted state from here, because
    // parking the robot is more than one assignment: the force-shoot override has to be dropped,
    // the shot solution forgotten, and any requested drive heading released. Doing that here would
    // put three pieces of the superstructure's own safety contract in a different file, where the
    // next person to add a fourth would have no reason to look.
    //
    // The override is the one that matters. It is a toggle an operator holds during a scoring run,
    // and if it survived a disable the robot would come back for the next match with the readiness
    // gate already bypassed.
    superstructure.onDisable();

    // The per-subsystem motion-profile reset the TODO also asks for is NOT done here, and that is a
    // deferral rather than an oversight: no mechanism exposes a reset entry point yet, and each
    // one's profile lives inside its Phoenix configuration rather than in this code. Commanding
    // IDLE, which onDisable does through the superstructure, already stops every mechanism, so
    // nothing is left driving; what is outstanding is only the re-enable transient. Left for M14's
    // brake-mode and competition-hardening pass, which is where the disable path gets its real
    // treatment.
  }

  /**
   * Called when autonomous starts.
   *
   * <p>Added for M10b step 7. The first enable of a match deploys the intake -- the robot must
   * start stowed, and pieces fall out of a stowed intake -- and the flywheel's shot count is zeroed
   * so the match's count starts clean, which also makes the 8-piece preload a known check of the
   * shot detector.
   *
   * <p><b>Outputs:</b> resets the shot count and tells the superstructure the robot is enabled.
   */
  public void onAutonomousInit() {
    shooter.resetShotCount();
    superstructure.onEnable();
  }

  /**
   * Called when teleop starts.
   *
   * <p>Added for M10b step 7. Auto must never leak a goal into teleop, so the shooter side goes
   * back to DEFAULT and any intake request an auto left standing is released. The intake posture is
   * NOT reset: an intake deployed in auto stays deployed, because the hopper is presumably not
   * empty. And onEnable deploys it if nothing has yet -- a practice session that starts in teleop
   * still begins with the intake stowed.
   *
   * <p><b>Outputs:</b> resets the superstructure's requests and tells it the robot is enabled.
   */
  public void onTeleopInit() {
    superstructure.setIntakeRequested(false);
    superstructure.setWantedState(Superstructure.WantedState.DEFAULT);
    superstructure.onEnable();
  }

  /**
   * Refreshes the autonomous starting-pose preview.
   *
   * <p><b>Outputs:</b> logs the selected routine's expected start pose. Called every {@code
   * disabledPeriodic}.
   */
  public void updateAutoPreview() {
    // IMPLEMENTATION (was TODO: "autoSelector.logStartingPose(). Catching a misplaced robot before
    // the match starts is worth the twenty lines this costs.")
    //
    // Every disabled loop, not only when the selection changes: the error it logs is the thing
    // that changes while the robot is being placed on the tape.
    autoSelector.logStartingPose();
  }

  /**
   * Records how autonomous finished, so routines can be scored from the log afterwards.
   *
   * <p>Added for M13 step 7, for {@code Robot.autonomousExit}.
   *
   * <p><b>Outputs:</b> logs {@code Auto/FinalPose} and {@code Auto/ShotCount}.
   */
  public void logAutonomousExit() {
    // The FLYWHEEL's count, which is zeroed at autonomousInit: the only piece count the robot has
    // (M10b). Set against Auto/PreloadFired it says how many pieces the routine fired after the
    // preload.
    Logger.recordOutput("Auto/FinalPose", RobotState.getInstance().getEstimatedPose());
    Logger.recordOutput("Auto/ShotCount", shooter.getShotCount());
  }
}
