package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoFactory;
import frc.robot.autos.AutoRoutines;
import frc.robot.autos.AutoSelector;
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
                turret::getVelocityRadPerSec);
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
                turret::getVelocityRadPerSec);
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
                turret::getVelocityRadPerSec);
        break;
    }

    superstructure =
        new Superstructure(drive, intake, floorRoller, indexer, turret, shooter, vision);
    autoFactory = new AutoFactory(drive, superstructure, intake, indexer, turret, vision);
    autoRoutines = new AutoRoutines(autoFactory);
    autoSelector = new AutoSelector(autoFactory, autoRoutines);

    configureButtonBindings();
    configureCharacterizationBindings();
  }

  /**
   * Binds every driver and operator control.
   *
   * <p><b>Outputs:</b> sets the drivetrain default command and registers all button triggers.
   */
  private void configureButtonBindings() {
    // TODO: drive.setDefaultCommand(DriveCommands.joystickDriveWithOverride(drive,
    //     () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX(),
    //     superstructure::getRequestedDriveHeading));
    //
    // Driver:
    //   rightBumper   whileTrue -> SCORE_ON_MOVE,      onFalse -> DEFAULT
    //   rightTrigger  whileTrue -> INTAKE,             onFalse -> DEFAULT
    //   leftBumper    whileTrue -> SCORE_STATIONARY,   onFalse -> DEFAULT
    //   leftTrigger   whileTrue -> PASS,               onFalse -> DEFAULT
    //   X             whileTrue -> EJECT
    //   Y             onTrue    -> HOME
    //   B             onTrue    -> reset pose to the vision estimate, .ignoringDisable(true)
    //   POV up/down/left/right  -> DriveCommands.snapToHeading(...)
    //   A             whileTrue -> DriveToPoseCommand to the nearest scoring pose
    //
    // Operator: manual turret nudge on a stick, flywheel RPM trim on the D-pad, and a "force shoot"
    // toggle calling superstructure.setIgnoreShotTolerances(true).
    //
    // Route EVERY action through superstructure.setWantedStateCommand(...). Binding a subsystem
    // directly is what lets a button put the robot into a combination the state machine never
    // intended, and it is exactly what this architecture exists to prevent.
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
