package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutoFactory;
import frc.robot.autos.AutoRoutines;
import frc.robot.autos.AutoSelector;
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
                intake::isIntaking);
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
                intake::isIntaking);
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
                intake::isIntaking);
        break;
    }

    superstructure =
        new Superstructure(drive, intake, floorRoller, indexer, turret, shooter, vision);
    autoFactory = new AutoFactory(drive, superstructure, intake, indexer, turret, vision);
    autoRoutines = new AutoRoutines(autoFactory);
    autoSelector = new AutoSelector(autoFactory, autoRoutines);

    configureButtonBindings();
    configureCharacterizationBindings();
    configureShooterTuningBindings();
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
    // joystickDrive is used rather than joystickDriveWithOverride deliberately: it is the
    // template's proven implementation, and the override path is inert anyway while
    // Superstructure.getRequestedDriveHeading() still returns Optional.empty(). Switching to the
    // override version is a one-line change once the superstructure is implemented:
    //
    //   DriveCommands.joystickDriveWithOverride(
    //       drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX(),
    //       superstructure::getRequestedDriveHeading)
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

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

    // ------------------------------------------------------- not yet bound: awaiting mechanisms
    // ----
    //
    // The bindings below are intentionally NOT wired yet, because every one of them routes through
    // Superstructure, whose state machine is still stubbed. Binding them now would give a driver
    // buttons that silently do nothing.
    //
    //   rightBumper   whileTrue -> SCORE_ON_MOVE,      onFalse -> DEFAULT
    //   rightTrigger  whileTrue -> INTAKE,             onFalse -> DEFAULT
    //   leftBumper    whileTrue -> SCORE_STATIONARY,   onFalse -> DEFAULT
    //   leftTrigger   whileTrue -> PASS,               onFalse -> DEFAULT
    //   Y             onTrue    -> HOME
    //
    // Operator: manual turret nudge on a stick, flywheel RPM trim on the D-pad, and a "force shoot"
    // toggle calling superstructure.setIgnoreShotTolerances(true).
    //
    // Route EVERY mechanism action through superstructure.setWantedStateCommand(...). Binding a
    // subsystem directly is what lets a button put the robot into a combination the state machine
    // never intended, and it is exactly what this architecture exists to prevent.
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
   * The routine to run in autonomous.
   *
   * @return the selected autonomous command; never null
   */
  public Command getAutonomousCommand() {
    // TODO: return autoSelector.get(). It already guarantees a non-null command.
    return Commands.none();
  }

  /**
   * Sets drivetrain brake or coast.
   *
   * @param brake true for brake, false for coast
   *     <p><b>Outputs:</b> reconfigures the drivetrain, and the turret and intake neutral modes.
   */
  public void setDriveBrakeMode(boolean brake) {
    // TODO: drive.setBrakeMode(brake), plus a turret and intake neutral-mode passthrough -- a
    // disabled robot should be pushable, but a disabled turret should not flop around.
  }

  /**
   * Called when the robot is disabled.
   *
   * <p><b>Outputs:</b> parks the superstructure and resets subsystem profiles, so that re-enabling
   * does not deliver a step input to a mechanism.
   */
  public void onDisable() {
    // TODO: superstructure.setWantedState(Superstructure.WantedState.IDLE) and tell every subsystem
    // to reset its motion profiles.
  }

  /**
   * Refreshes the autonomous starting-pose preview.
   *
   * <p><b>Outputs:</b> logs the selected routine's expected start pose. Called every {@code
   * disabledPeriodic}.
   */
  public void updateAutoPreview() {
    // TODO: autoSelector.logStartingPose(). Catching a misplaced robot before the match starts is
    // worth the twenty lines this costs.
  }
}
