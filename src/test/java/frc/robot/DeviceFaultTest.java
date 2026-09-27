package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.floorroller.FloorRoller;
import frc.robot.subsystems.floorroller.FloorRollerIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.QuestNavIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.FaultReporter;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * The first clause of the M14 checkpoint, in simulation: unplug any single device and the robot
 * says which one, by name -- and says nothing about any other.
 *
 * <p>Added for M14 step 2. Every device's simulated hardware layer is wrapped so a case can unplug
 * exactly one of them. The case then checks two things from the fault reporter:
 *
 * <ul>
 *   <li>the unplugged device, and only it, is named among the active faults; and
 *   <li>it blocks scoring exactly when it is one of the three devices whose loss makes a shot
 *       unsafe -- the flywheel leader and follower and the turret motor.
 * </ul>
 *
 * <p>The fault reporter is shared by every suite, so the checks look only at the names this robot
 * registers. The Pigeon2 is the one device not covered: its check is deliberately off in
 * simulation, where there is no gyro and odometry integrates the modules.
 */
public class DeviceFaultTest {
  /** The device a case unplugs, by the name it is registered under; empty for none. */
  private String unplugged = "";

  private final List<Subsystem> built = new ArrayList<>();

  private static final String RIO = Constants.Can.kRioBus;
  private static final String CANIVORE = TunerConstants.kCANBus.getName();

  /** Every device name the robot registers, in the order they are built, and whether it blocks. */
  private static Map<String, Boolean> devices() {
    Map<String, Boolean> devices = new LinkedHashMap<>();
    devices.put(disconnect("Intake pivot motor", Constants.Can.kIntakePivotId, RIO), false);
    devices.put(disconnect("Intake roller motor", Constants.Can.kIntakeRollerId, RIO), false);
    devices.put("Intake pivot absolute encoder disconnected", false);
    devices.put(disconnect("Floor roller motor", Constants.Can.kFloorRollerId, RIO), false);
    devices.put(disconnect("Indexer motor", Constants.Can.kIndexerId, RIO), false);
    devices.put(disconnect("Turret motor", Constants.Can.kTurretId, RIO), true);
    devices.put(
        "Turret absolute encoder disconnected -- homing will fall back to finding a hard stop",
        false);
    devices.put(disconnect("Flywheel leader motor", Constants.Can.kFlywheelLeaderId, RIO), true);
    devices.put(
        disconnect("Flywheel follower motor", Constants.Can.kFlywheelFollowerId, RIO), true);
    String[] corners = {"Front-left", "Front-right", "Back-left", "Back-right"};
    var modules =
        List.of(
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    for (int i = 0; i < 4; i++) {
      devices.put(
          disconnect(corners[i] + " drive motor", modules.get(i).DriveMotorId, CANIVORE), false);
      devices.put(
          disconnect(corners[i] + " steer motor", modules.get(i).SteerMotorId, CANIVORE), false);
      devices.put(disconnect(corners[i] + " CANcoder", modules.get(i).EncoderId, CANIVORE), false);
    }
    devices.put(
        "Intake Limelight (" + Constants.Vision.kIntakeCameraName + ") disconnected", false);
    devices.put(
        "Turret Limelight (" + Constants.Vision.kTurretCameraName + ") disconnected", false);
    return devices;
  }

  /**
   * The alert a CAN device raises when it is unplugged.
   *
   * @param description what the device is
   * @param id its CAN ID
   * @param bus its bus
   * @return the fault name
   */
  private static String disconnect(String description, int id, String bus) {
    return FaultReporter.canDevice(description, id, bus) + " disconnected";
  }

  /** Every device, as a case: its fault name. */
  static List<String> deviceNames() {
    return List.copyOf(devices().keySet());
  }

  /** Initialises the HAL, pauses the clock, and makes sure the pose estimator exists. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();
    RobotState.initialize(new SwerveDriveKinematics(Drive.getModuleTranslations()));
  }

  /** Drops every subsystem this case built and releases the HAL. */
  @AfterEach
  void tearDown() {
    for (Subsystem subsystem : built) {
      CommandScheduler.getInstance().unregisterSubsystem(subsystem);
    }
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  /**
   * Whether the device registered under a name is the one this case unplugged.
   *
   * @param name the device's fault name
   * @return true if it should read disconnected
   */
  private boolean isUnplugged(String name) {
    return unplugged.equals(name);
  }

  /**
   * Builds every subsystem over simulated hardware, with {@link #unplugged} reading disconnected,
   * and runs each one loop.
   *
   * <p><b>Outputs:</b> registers every device with the fault reporter; fills {@link #built}.
   */
  private void buildAndStep() {
    String pivot = disconnect("Intake pivot motor", Constants.Can.kIntakePivotId, RIO);
    String roller = disconnect("Intake roller motor", Constants.Can.kIntakeRollerId, RIO);
    Intake intake =
        new Intake(
            new IntakeIOSim() {
              @Override
              public void updateInputs(IntakeIOInputs inputs) {
                super.updateInputs(inputs);
                inputs.pivotConnected &= !isUnplugged(pivot);
                inputs.rollerConnected &= !isUnplugged(roller);
                inputs.absEncoderConnected &=
                    !isUnplugged("Intake pivot absolute encoder disconnected");
              }
            });
    FloorRoller floorRoller =
        new FloorRoller(
            new FloorRollerIOSim() {
              @Override
              public void updateInputs(FloorRollerIOInputs inputs) {
                super.updateInputs(inputs);
                inputs.connected &=
                    !isUnplugged(
                        disconnect("Floor roller motor", Constants.Can.kFloorRollerId, RIO));
              }
            });
    Indexer indexer =
        new Indexer(
            new IndexerIOSim() {
              @Override
              public void updateInputs(IndexerIOInputs inputs) {
                super.updateInputs(inputs);
                inputs.connected &=
                    !isUnplugged(disconnect("Indexer motor", Constants.Can.kIndexerId, RIO));
              }
            });
    Turret turret =
        new Turret(
            new TurretIOSim() {
              @Override
              public void updateInputs(TurretIOInputs inputs) {
                super.updateInputs(inputs);
                inputs.motorConnected &=
                    !isUnplugged(disconnect("Turret motor", Constants.Can.kTurretId, RIO));
                inputs.absEncoderConnected &=
                    !isUnplugged(
                        "Turret absolute encoder disconnected -- homing will fall back to"
                            + " finding a hard stop");
              }
            });
    Shooter shooter =
        new Shooter(
            new ShooterIOSim() {
              @Override
              public void updateInputs(ShooterIOInputs inputs) {
                super.updateInputs(inputs);
                inputs.leaderConnected &=
                    !isUnplugged(
                        disconnect("Flywheel leader motor", Constants.Can.kFlywheelLeaderId, RIO));
                inputs.followerConnected &=
                    !isUnplugged(
                        disconnect(
                            "Flywheel follower motor", Constants.Can.kFlywheelFollowerId, RIO));
              }
            });
    String[] corners = {"Front-left", "Front-right", "Back-left", "Back-right"};
    var constants =
        List.of(
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    ModuleIOSim[] modules = new ModuleIOSim[4];
    for (int i = 0; i < 4; i++) {
      String drive =
          disconnect(corners[i] + " drive motor", constants.get(i).DriveMotorId, CANIVORE);
      String steer =
          disconnect(corners[i] + " steer motor", constants.get(i).SteerMotorId, CANIVORE);
      String encoder = disconnect(corners[i] + " CANcoder", constants.get(i).EncoderId, CANIVORE);
      modules[i] =
          new ModuleIOSim(constants.get(i)) {
            @Override
            public void updateInputs(ModuleIOInputs inputs) {
              super.updateInputs(inputs);
              inputs.driveConnected &= !isUnplugged(drive);
              inputs.turnConnected &= !isUnplugged(steer);
              inputs.turnEncoderConnected &= !isUnplugged(encoder);
            }
          };
    }
    Drive drive = new Drive(new GyroIO() {}, modules[0], modules[1], modules[2], modules[3]);
    String intakeCamera =
        "Intake Limelight (" + Constants.Vision.kIntakeCameraName + ") disconnected";
    String turretCamera =
        "Turret Limelight (" + Constants.Vision.kTurretCameraName + ") disconnected";
    Vision vision =
        new Vision(
            new VisionIO() {
              @Override
              public void updateInputs(VisionIOInputs inputs) {
                inputs.connected = !isUnplugged(turretCamera);
              }
            },
            new VisionIO() {
              @Override
              public void updateInputs(VisionIOInputs inputs) {
                inputs.connected = !isUnplugged(intakeCamera);
              }
            },
            new QuestNavIO() {},
            turret::getPositionRad,
            turret::getVelocityRadPerSec,
            intake::isAtIntakingAngle);

    built.addAll(List.of(drive, intake, floorRoller, indexer, turret, shooter, vision));
    for (Subsystem subsystem : built) {
      subsystem.periodic();
    }
  }

  /**
   * The active faults among this robot's devices.
   *
   * @param blockingOnly true to list only those that block scoring
   * @return their names
   */
  private static Set<String> activeDevices(boolean blockingOnly) {
    FaultReporter reporter = FaultReporter.getInstance();
    List<String> active =
        blockingOnly ? reporter.getBlockingFaultNames() : reporter.getActiveFaultNames();
    Set<String> ours = devices().keySet();
    return Set.copyOf(active.stream().filter(ours::contains).toList());
  }

  /**
   * With every device plugged in, nothing is reported.
   *
   * <p>The case the others are measured against. A device reported disconnected on a healthy robot
   * is how a drive team learns to ignore the alerts widget, which is the one place the M14
   * checkpoint needs them to look.
   */
  @org.junit.jupiter.api.Test
  void aHealthyRobotReportsNothing() {
    buildAndStep();
    assertEquals(Set.of(), activeDevices(false));
  }

  /**
   * Unplugged alone, each device is named, alone, and blocks scoring only if its loss makes a shot
   * unsafe.
   *
   * @param device the fault name of the device to unplug
   */
  @ParameterizedTest(name = "{0}")
  @MethodSource("deviceNames")
  void eachDeviceUnpluggedAloneIsNamedAlone(String device) {
    unplugged = device;
    buildAndStep();
    assertEquals(Set.of(device), activeDevices(false), "not exactly the unplugged device");
    boolean blocks = devices().get(device);
    assertEquals(
        blocks ? Set.of(device) : Set.of(),
        activeDevices(true),
        blocks ? "a device that makes a shot unsafe did not block" : "a device blocked scoring");
  }
}
