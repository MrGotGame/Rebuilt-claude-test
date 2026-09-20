package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;

/**
 * The single entry point for every number in this codebase.
 *
 * <p>Three rules the whole team follows:
 *
 * <ol>
 *   <li>No numeric literal appears in any file other than this one, {@code TunerConstants.java},
 *       and unit tests.
 *   <li>Anything you might want to change at a competition is a {@link LoggedTunableNumber}.
 *       Anything physically fixed -- a gear ratio, a CAN ID -- is a plain {@code static final}.
 *   <li>Every field whose value depends on the REBUILT field or game piece carries a {@code TODO}
 *       saying where to measure it. None of them are guesses.
 * </ol>
 *
 * <p>Units throughout this project are <b>meters, radians, and seconds</b>, except motor speeds
 * expressed as RPM where that is the natural tuning unit. Any field whose unit is not obvious from
 * its name carries the unit in its name or its comment.
 *
 * <p>This file also owns the competition-versus-practice robot identity switch, so a single {@link
 * #getRobot()} call selects the right offsets and gains for whichever chassis is on the cart.
 */
public final class Constants {
  private Constants() {}

  // ---------------------------------------------------------------- runtime mode & identity ----

  /** How the robot code is currently executing. */
  public static enum Mode {
    /** Running on a real roboRIO. */
    REAL,
    /** Running a physics simulation. */
    SIM,
    /** Replaying a recorded log. */
    REPLAY
  }

  /** Which physical chassis this code is running on. */
  public static enum RobotType {
    /** The competition robot. */
    COMP,
    /** The practice robot, which may have different offsets and gains. */
    PRACTICE,
    /** No physical robot; simulation or replay. */
    SIMBOT
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /**
   * roboRIO serial number of the competition robot.
   *
   * <p>TODO: MEASURE. Read this off the RIO with {@link RobotController#getSerialNumber()}, or from
   * the roboRIO web dashboard, and paste it here.
   */
  public static final String kCompRoboRioSerial = "TODO-COMP-SERIAL";

  /** roboRIO serial number of the practice robot. TODO: MEASURE, same method as above. */
  public static final String kPracticeRoboRioSerial = "TODO-PRACTICE-SERIAL";

  /** Master switch for tunable numbers and SysId bindings. MUST be false at competition. */
  public static final boolean kTuningMode = true;

  /** Set true only by unit tests, so {@link LoggedTunableNumber} does not touch NetworkTables. */
  public static boolean disableHal = false;

  /**
   * Identifies which physical robot this code is running on, so identity-dependent constants can
   * branch.
   *
   * @return {@link RobotType#SIMBOT} in simulation or replay; otherwise the robot whose roboRIO
   *     serial number matches, defaulting to {@link RobotType#COMP}
   */
  public static RobotType getRobot() {
    // TODO: In SIM/REPLAY return SIMBOT. On a real RIO, compare RobotController.getSerialNumber()
    // against kCompRoboRioSerial and kPracticeRoboRioSerial and return the match. Default to COMP
    // but raise a FaultReporter alert when neither matches, so an unknown RIO is loud rather than
    // silently running the wrong encoder offsets.
    return RobotType.COMP;
  }

  /**
   * The main robot loop period. Exists so the shooting solver and the swerve setpoint generator
   * never hardcode 0.02.
   *
   * @return loop period, in seconds
   */
  public static double getLoopPeriodSec() {
    // TODO: return edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod.
    return 0.02;
  }

  // ----------------------------------------------------------------------------- CAN & ports ----

  /** CAN bus names and device IDs. See the CAN map in section 5 of the plan. */
  public static final class Can {
    private Can() {}

    /**
     * CANivore carrying the drivetrain only, so swerve never competes with mechanisms for bus time.
     */
    public static final String kDriveBus = "canivore";

    /** The roboRIO's own CAN bus, carrying the seven mechanism Talons. */
    public static final String kRioBus = "rio";

    /**
     * The drivetrain bus as a {@link CANBus} object.
     *
     * <p>Phoenix 6 has deprecated the {@code TalonFX(int, String)} constructor for removal, so
     * every device must be built against a CANBus rather than a bus name string.
     */
    public static final CANBus kDriveCanBus = new CANBus(kDriveBus);

    /** The roboRIO bus as a {@link CANBus} object. */
    public static final CANBus kRioCanBus = new CANBus(kRioBus);

    public static final int kPigeonId = 1;
    public static final int kIntakePivotId = 20;
    public static final int kIntakeRollerId = 21;
    public static final int kFloorRollerId = 22;
    public static final int kIndexerId = 23;
    public static final int kTurretId = 24;
    public static final int kFlywheelLeaderId = 25;
    public static final int kFlywheelFollowerId = 26;
    public static final int kPdhId = 1;
  }

  /** roboRIO digital input channels. */
  public static final class Dio {
    private Dio() {}

    /** WCP ThroughBore on the intake pivot gearbox output. */
    public static final int kIntakePivotAbsEncoder = 0;

    /** WCP ThroughBore on the turret gearbox output. */
    public static final int kTurretAbsEncoder = 1;

    /**
     * TODO: optional beam break; delete this and set Indexer.kHasBeamBreak false if not installed.
     */
    public static final int kIndexerBeamBreak = 2;
  }

  // -------------------------------------------------------------------------------- the field ----

  /**
   * Field geometry, in blue-origin WPILib coordinates.
   *
   * <p>Every value here is currently a placeholder. The robot will not shoot correctly until they
   * are measured -- see each field's TODO for where the number comes from.
   */
  public static final class Field {
    private Field() {}

    /**
     * Blue-alliance goal center on the field floor plane, in meters, blue-origin coordinates.
     *
     * <p>TODO: MEASURE. Take X and Y from the official 2026 REBUILT field drawings (the FIRST
     * "Field Layout and Marking Diagram" PDF), or by driving the robot to the goal and reading
     * Odometry/Robot in AdvantageScope. Do not guess.
     */
    public static final LoggedTunableNumber kGoalXMeters =
        new LoggedTunableNumber("Field/GoalX", 0.0);

    /** Blue-alliance goal center Y, meters. TODO: MEASURE, same source as kGoalXMeters. */
    public static final LoggedTunableNumber kGoalYMeters =
        new LoggedTunableNumber("Field/GoalY", 0.0);

    /**
     * Height of the goal opening above the carpet, meters. TODO: MEASURE from the field drawings.
     */
    public static final LoggedTunableNumber kGoalHeightMeters =
        new LoggedTunableNumber("Field/GoalHeight", 0.0);

    /**
     * Field length, meters. Used only by AllianceFlipUtil. TODO: take from the AprilTag layout
     * JSON.
     */
    public static final LoggedTunableNumber kFieldLengthMeters =
        new LoggedTunableNumber("Field/Length", 0.0);

    /** Field width, meters. TODO: take from the AprilTag layout JSON. */
    public static final LoggedTunableNumber kFieldWidthMeters =
        new LoggedTunableNumber("Field/Width", 0.0);

    /**
     * Mass of one REBUILT game piece, kg. TODO: MEASURE on a kitchen scale; used only by maple-sim.
     */
    public static final LoggedTunableNumber kGamePieceMassKg =
        new LoggedTunableNumber("Field/GamePieceMass", 0.0);

    /** Game-piece diameter, meters. TODO: MEASURE with calipers; used by intake-camera ranging. */
    public static final LoggedTunableNumber kGamePieceDiameterMeters =
        new LoggedTunableNumber("Field/GamePieceDiameter", 0.0);

    /**
     * The goal position to aim at, already flipped for the current alliance.
     *
     * @return goal center on the floor plane, in meters, in the current alliance's frame
     */
    public static Translation2d goalTranslation() {
      // IMPLEMENTATION (was TODO: "Build a Translation2d from kGoalX/kGoalY, then pass it through
      // AllianceFlipUtil.apply(...) so red-alliance matches aim at the mirrored goal.")
      //
      // Flipped on every call rather than cached, because the alliance is not known until the
      // driver station connects -- which is always after this class has been initialized.
      return AllianceFlipUtil.apply(new Translation2d(kGoalXMeters.get(), kGoalYMeters.get()));
    }

    /**
     * The goal position including its height, for ballistic math and the AdvantageScope 3D overlay.
     *
     * @return goal center in three dimensions, in meters, in the current alliance's frame
     */
    public static Translation3d goalTranslation3d() {
      // IMPLEMENTATION (was TODO: "Same as goalTranslation() but with kGoalHeightMeters as Z.")
      //
      // Height is NOT flipped: the goal is the same distance off the carpet on both alliances.
      Translation2d flipped = goalTranslation();
      return new Translation3d(flipped.getX(), flipped.getY(), kGoalHeightMeters.get());
    }
  }

  // -------------------------------------------------------------------------------- the drive ----

  /** Drivetrain constants not owned by the CTRE-generated TunerConstants. */
  public static final class Drivetrain {
    private Drivetrain() {}

    /** Robot mass, kg. Feeds PathPlanner's RobotConfig. TODO: MEASURE by weighing the robot. */
    public static final double kRobotMassKg = 74.088;

    /** Robot moment of inertia, kg*m^2. TODO: from CAD, or from a spin test. */
    public static final double kRobotMoiKgM2 = 6.883;

    /** Wheel coefficient of friction on carpet. TODO: tune if PathPlanner overshoots corners. */
    public static final double kWheelCoefficientOfFriction = 1.2;

    public static final LoggedTunableNumber kJoystickDeadband =
        new LoggedTunableNumber("Drive/Deadband", 0.08);

    public static final LoggedTunableNumber kHeadingKp =
        new LoggedTunableNumber("Drive/HeadingKp", 5.0);
    public static final LoggedTunableNumber kHeadingKd =
        new LoggedTunableNumber("Drive/HeadingKd", 0.4);
    public static final LoggedTunableNumber kHeadingMaxVelRadPerSec =
        new LoggedTunableNumber("Drive/HeadingMaxVel", 8.0);
    public static final LoggedTunableNumber kHeadingMaxAccelRadPerSec2 =
        new LoggedTunableNumber("Drive/HeadingMaxAccel", 20.0);

    public static final LoggedTunableNumber kAutoAlignTranslationKp =
        new LoggedTunableNumber("Drive/AutoAlignKp", 4.0);
    public static final LoggedTunableNumber kAutoAlignMaxVelMetersPerSec =
        new LoggedTunableNumber("Drive/AutoAlignMaxVel", 3.0);
    public static final LoggedTunableNumber kAutoAlignMaxAccelMetersPerSec2 =
        new LoggedTunableNumber("Drive/AutoAlignMaxAccel", 3.0);
    public static final LoggedTunableNumber kAutoAlignToleranceMeters =
        new LoggedTunableNumber("Drive/AutoAlignTolerance", 0.03);

    /** Beyond this distance DriveToPoseCommand delegates to PathPlanner's pathfinder. */
    public static final LoggedTunableNumber kPathfindHandoffMeters =
        new LoggedTunableNumber("Drive/PathfindHandoff", 1.5);

    /** Heading tolerance for auto-align completion, radians. */
    public static final LoggedTunableNumber kAutoAlignHeadingToleranceRad =
        new LoggedTunableNumber("Drive/AutoAlignHeadingTolerance", 0.035);

    /** Chassis speed below which auto-align is considered settled, m/s. */
    public static final LoggedTunableNumber kAutoAlignSettledSpeedMetersPerSec =
        new LoggedTunableNumber("Drive/AutoAlignSettledSpeed", 0.05);

    /** Max module azimuth rate for PathPlanner's SwerveSetpointGenerator, rad/s. */
    public static final double kMaxModuleAzimuthRadPerSec = 20.0;

    /**
     * How long after disabling before the drivetrain drops to coast so it can be pushed, seconds.
     */
    public static final LoggedTunableNumber kCoastDelaySec =
        new LoggedTunableNumber("Drive/CoastDelay", 5.0);
  }

  // ------------------------------------------------------------------------------- the intake ----

  /** Pivoting floor intake: Kraken X44 pivot with a ThroughBore, Kraken X60 roller. */
  public static final class Intake {
    private Intake() {}

    /** Motor rotations per gearbox-output rotation. TODO: MEASURE by counting gear teeth. */
    public static final double kPivotGearRatio = 1.0;

    public static final boolean kPivotInverted = false;
    public static final boolean kRollerInverted = false;

    /**
     * ThroughBore reading, in rotations, when the pivot sits exactly at {@link #kStowedAngleRad}.
     *
     * <p>TODO: MEASURE. Put the pivot on its stow hard stop, read Intake/Inputs/pivotAbsRot in
     * AdvantageScope, and paste that number here.
     */
    public static final LoggedTunableNumber kPivotAbsEncoderOffsetRot =
        new LoggedTunableNumber("Intake/PivotAbsOffset", 0.0);

    public static final LoggedTunableNumber kStowedAngleRad =
        new LoggedTunableNumber("Intake/StowedAngle", 0.0);
    public static final LoggedTunableNumber kDeployedAngleRad =
        new LoggedTunableNumber("Intake/DeployedAngle", 0.0);
    public static final LoggedTunableNumber kEjectAngleRad =
        new LoggedTunableNumber("Intake/EjectAngle", 0.0);
    public static final LoggedTunableNumber kPivotToleranceRad =
        new LoggedTunableNumber("Intake/PivotTolerance", 0.05);

    public static final LoggedTunableNumber kPivotKp =
        new LoggedTunableNumber("Intake/PivotKp", 40.0);
    public static final LoggedTunableNumber kPivotKd =
        new LoggedTunableNumber("Intake/PivotKd", 0.0);
    public static final LoggedTunableNumber kPivotKs =
        new LoggedTunableNumber("Intake/PivotKs", 0.0);
    public static final LoggedTunableNumber kPivotKv =
        new LoggedTunableNumber("Intake/PivotKv", 0.0);
    public static final LoggedTunableNumber kPivotKg =
        new LoggedTunableNumber("Intake/PivotKg", 0.0);
    public static final LoggedTunableNumber kPivotCruiseRadPerSec =
        new LoggedTunableNumber("Intake/PivotCruise", 8.0);
    public static final LoggedTunableNumber kPivotAccelRadPerSec2 =
        new LoggedTunableNumber("Intake/PivotAccel", 30.0);

    public static final LoggedTunableNumber kIntakeRollerRpm =
        new LoggedTunableNumber("Intake/RollerRpm", 3000.0);
    public static final LoggedTunableNumber kEjectRollerRpm =
        new LoggedTunableNumber("Intake/EjectRollerRpm", -3000.0);

    public static final LoggedTunableNumber kHomingDutyCycle =
        new LoggedTunableNumber("Intake/HomingDutyCycle", -0.15);
    public static final LoggedTunableNumber kHomingVelocityThresholdRadPerSec =
        new LoggedTunableNumber("Intake/HomingVelThreshold", 0.15);
    public static final LoggedTunableNumber kHomingSettleSec =
        new LoggedTunableNumber("Intake/HomingSettle", 0.25);
    public static final LoggedTunableNumber kHomingTimeoutSec =
        new LoggedTunableNumber("Intake/HomingTimeout", 3.0);

    public static final double kPivotStatorLimitAmps = 60.0;
    public static final double kRollerStatorLimitAmps = 80.0;

    /** Pivot moment of inertia for SingleJointedArmSim, kg*m^2. TODO: pull from CAD. */
    public static final double kPivotMoiKgM2 = 0.05;

    /** Pivot arm length for SingleJointedArmSim, meters. TODO: pull from CAD. */
    public static final double kPivotLengthMeters = 0.35;
  }

  /** The floor roller that moves game pieces from the intake toward the indexer. */
  public static final class FloorRoller {
    private FloorRoller() {}

    /** Motor rotations per roller rotation. TODO: MEASURE. */
    public static final double kGearRatio = 1.0;

    public static final boolean kInverted = false;
    public static final double kStatorLimitAmps = 60.0;

    public static final LoggedTunableNumber kIntakeRpm =
        new LoggedTunableNumber("FloorRoller/IntakeRpm", 2500.0);
    public static final LoggedTunableNumber kFeedRpm =
        new LoggedTunableNumber("FloorRoller/FeedRpm", 3000.0);
    public static final LoggedTunableNumber kEjectRpm =
        new LoggedTunableNumber("FloorRoller/EjectRpm", -2500.0);
    public static final LoggedTunableNumber kKp = new LoggedTunableNumber("FloorRoller/Kp", 0.1);
    public static final LoggedTunableNumber kKv = new LoggedTunableNumber("FloorRoller/Kv", 0.12);
    public static final LoggedTunableNumber kKs = new LoggedTunableNumber("FloorRoller/Ks", 0.0);

    /**
     * Speed band inside which {@code FloorRoller.atSpeed()} reports the roller as up to speed.
     *
     * <p>Wider than the flywheel's 75 RPM on purpose. Nothing downstream gates a shot on this
     * roller, so the only cost of a loose band is an optimistic log line, whereas a tight band on a
     * roller that is pushing a game piece around would read as "never at speed" for the whole
     * intake cycle. TODO: tune once the roller has run with a piece in it.
     */
    public static final LoggedTunableNumber kToleranceRpm =
        new LoggedTunableNumber("FloorRoller/ToleranceRpm", 200.0);

    /** DCMotorSim moment of inertia, kg*m^2, roller plus shaft. TODO: from CAD. */
    public static final double kMoiKgM2 = 0.001;
  }

  /** The indexer that holds a piece and feeds it into the flywheel on command. */
  public static final class Indexer {
    private Indexer() {}

    /** Motor rotations per indexer rotation. TODO: MEASURE. */
    public static final double kGearRatio = 1.0;

    public static final boolean kInverted = false;
    public static final double kStatorLimitAmps = 50.0;

    /** TODO: flip to true if you install a beam break; piece detection gets far more reliable. */
    public static final boolean kHasBeamBreak = false;

    public static final LoggedTunableNumber kIndexRpm =
        new LoggedTunableNumber("Indexer/IndexRpm", 1200.0);
    public static final LoggedTunableNumber kFeedRpm =
        new LoggedTunableNumber("Indexer/FeedRpm", 3500.0);
    public static final LoggedTunableNumber kEjectRpm =
        new LoggedTunableNumber("Indexer/EjectRpm", -1500.0);
    public static final LoggedTunableNumber kKp = new LoggedTunableNumber("Indexer/Kp", 0.1);
    public static final LoggedTunableNumber kKv = new LoggedTunableNumber("Indexer/Kv", 0.12);

    /**
     * Current-spike piece detection threshold, amps. Used only when {@link #kHasBeamBreak} is
     * false.
     *
     * <p>TODO: tune on the robot by watching Indexer/Inputs/currentAmps while feeding a piece.
     */
    public static final LoggedTunableNumber kHasPieceCurrentAmps =
        new LoggedTunableNumber("Indexer/HasPieceCurrent", 25.0);

    public static final LoggedTunableNumber kHasPieceVelocityDropRpm =
        new LoggedTunableNumber("Indexer/HasPieceVelDrop", 400.0);
    public static final LoggedTunableNumber kHasPieceDebounceSec =
        new LoggedTunableNumber("Indexer/HasPieceDebounce", 0.06);
    public static final LoggedTunableNumber kShotDetectDebounceSec =
        new LoggedTunableNumber("Indexer/ShotDebounce", 0.08);
  }

  // ------------------------------------------------------------------------------- the turret ----

  /** The turret: Kraken X44 with a WCP ThroughBore on the gearbox output, finite travel. */
  public static final class Turret {
    private Turret() {}

    /** Motor rotations per turret rotation. TODO: MEASURE by counting gear teeth. */
    public static final double kGearRatio = 1.0;

    public static final boolean kInverted = false;
    public static final double kStatorLimitAmps = 40.0;

    /**
     * ThroughBore reading, in rotations, when the turret points straight forward along robot +X.
     *
     * <p>TODO: MEASURE the same way as the intake pivot offset.
     */
    public static final LoggedTunableNumber kAbsEncoderOffsetRot =
        new LoggedTunableNumber("Turret/AbsOffset", 0.0);

    /**
     * Minimum mechanical travel limit in the turret's own frame, robot-relative radians.
     *
     * <p>TODO: MEASURE by driving the turret slowly to each hard stop with kTuningMode on, reading
     * Turret/Inputs/positionRad, then backing off by {@link #kLimitMarginRad}.
     */
    public static final LoggedTunableNumber kMinAngleRad =
        new LoggedTunableNumber("Turret/MinAngle", -Math.toRadians(200.0));

    /** Maximum mechanical travel limit, robot-relative radians. TODO: MEASURE, see kMinAngleRad. */
    public static final LoggedTunableNumber kMaxAngleRad =
        new LoggedTunableNumber("Turret/MaxAngle", Math.toRadians(200.0));

    /** Safety margin held back from each hard stop, radians. */
    public static final LoggedTunableNumber kLimitMarginRad =
        new LoggedTunableNumber("Turret/LimitMargin", Math.toRadians(3.0));

    public static final LoggedTunableNumber kKp = new LoggedTunableNumber("Turret/Kp", 50.0);
    public static final LoggedTunableNumber kKd = new LoggedTunableNumber("Turret/Kd", 0.5);
    public static final LoggedTunableNumber kKs = new LoggedTunableNumber("Turret/Ks", 0.15);
    public static final LoggedTunableNumber kKv = new LoggedTunableNumber("Turret/Kv", 0.12);
    public static final LoggedTunableNumber kKa = new LoggedTunableNumber("Turret/Ka", 0.0);
    public static final LoggedTunableNumber kCruiseRadPerSec =
        new LoggedTunableNumber("Turret/Cruise", 12.0);
    public static final LoggedTunableNumber kAccelRadPerSec2 =
        new LoggedTunableNumber("Turret/Accel", 60.0);

    public static final LoggedTunableNumber kToleranceRad =
        new LoggedTunableNumber("Turret/Tolerance", Math.toRadians(1.5));
    public static final LoggedTunableNumber kVelocityToleranceRadPerSec =
        new LoggedTunableNumber("Turret/VelTolerance", 0.5);

    /** Below this angular error the unwrapper refuses to switch branches, to stop chatter. */
    public static final LoggedTunableNumber kUnwrapHysteresisRad =
        new LoggedTunableNumber("Turret/UnwrapHysteresis", Math.toRadians(10.0));

    public static final LoggedTunableNumber kStowAngleRad =
        new LoggedTunableNumber("Turret/StowAngle", 0.0);

    /** Lower bound of turret angles that would sweep through the deployed intake, radians. */
    public static final LoggedTunableNumber kIntakeConflictMinRad =
        new LoggedTunableNumber("Turret/IntakeConflictMin", Math.toRadians(150.0));

    /** Upper bound of the intake conflict zone, radians. */
    public static final LoggedTunableNumber kIntakeConflictMaxRad =
        new LoggedTunableNumber("Turret/IntakeConflictMax", Math.toRadians(210.0));

    /** Turret moment of inertia for TurretIOSim, kg*m^2. TODO: from CAD. */
    public static final double kMoiKgM2 = 0.02;

    /**
     * How close to the latched branch target a long-way-around sweep must get before the turret is
     * treated as aiming again, radians.
     *
     * <p>Deliberately much looser than {@link #kToleranceRad}. This threshold decides when to stop
     * suppressing the chassis-cancelling feedforward, not when a shot is allowed -- {@code
     * Turret.atGoal()} still owns that. Making it as tight as the shot tolerance would hold the
     * feedforward off through the settling tail of every sweep, which is exactly the part of the
     * motion where the chassis may already be rotating again.
     */
    public static final LoggedTunableNumber kUnwrapExitToleranceRad =
        new LoggedTunableNumber("Turret/UnwrapExitTolerance", Math.toRadians(10.0));

    /**
     * How long the turret may stay in {@code HOMING} before giving up and faulting, seconds.
     *
     * <p>Generous, because the fallback stall-homing path may have to traverse the whole travel at
     * {@link #kHomingVolts}. The absolute-encoder path finishes in one loop and never approaches
     * this.
     */
    public static final LoggedTunableNumber kHomingTimeoutSec =
        new LoggedTunableNumber("Turret/HomingTimeout", 5.0);

    /**
     * Open-loop voltage used by the fallback stall-homing path, volts.
     *
     * <p><b>Negative drives toward {@link #kMinAngleRad}</b>, and the angle seeded when the turret
     * stalls is {@code kMinAngleRad} for exactly that reason. Flipping this sign without also
     * flipping the seed zeroes the turret against the wrong hard stop, which presents as a turret
     * that is a full travel-width wrong and aims into the driver station.
     *
     * <p>Small on purpose: this path ends by deliberately stalling the mechanism against a hard
     * stop, so the voltage must be low enough that doing so is harmless.
     */
    public static final LoggedTunableNumber kHomingVolts =
        new LoggedTunableNumber("Turret/HomingVolts", -1.0);

    /** Below this speed the stall-homing path believes the turret is against a stop, rad/s. */
    public static final LoggedTunableNumber kHomingVelocityThresholdRadPerSec =
        new LoggedTunableNumber("Turret/HomingVelThreshold", 0.15);

    /**
     * Stator current the turret must be drawing for a stall to count as the hard stop.
     *
     * <p>Stall detection on velocity alone cannot tell a hard stop from a dead motor: both report
     * zero. A stop pulls current, a disconnected motor pulls none, so this is what stops an
     * unpowered turret from seeding a confidently wrong zero and then driving at full authority
     * from it. TODO: MEASURE -- drive into the stop at kHomingVolts and read the steady draw.
     */
    public static final LoggedTunableNumber kHomingCurrentAmps =
        new LoggedTunableNumber("Turret/HomingCurrent", 10.0);

    /**
     * How long stall-homing drives before it is allowed to believe a low velocity, seconds.
     *
     * <p>Without this grace period the turret is stationary on the very first homing loop, so the
     * velocity test would fire immediately and seed the current position as the hard stop. Homing
     * would then always "succeed" and always be wrong.
     */
    public static final LoggedTunableNumber kHomingMinMotionSec =
        new LoggedTunableNumber("Turret/HomingMinMotion", 0.25);

    /** Largest voltage an operator manual jog may command, volts. */
    public static final LoggedTunableNumber kManualMaxVolts =
        new LoggedTunableNumber("Turret/ManualMaxVolts", 4.0);
  }

  // ------------------------------------------------------------------------------ the shooter ----

  /** The flywheel: two Kraken X60s in leader-follower. */
  public static final class Shooter {
    private Shooter() {}

    /** Motor rotations per flywheel rotation. TODO: MEASURE. */
    public static final double kGearRatio = 1.0;

    public static final boolean kLeaderInverted = false;

    /**
     * TODO: VERIFY against the gearbox before first spin-up; wrong here means the pair fights
     * itself.
     */
    public static final boolean kFollowerOpposesLeader = true;

    public static final double kStatorLimitAmps = 80.0;
    public static final double kSupplyLimitAmps = 60.0;

    public static final LoggedTunableNumber kKp = new LoggedTunableNumber("Shooter/Kp", 0.2);
    public static final LoggedTunableNumber kKi = new LoggedTunableNumber("Shooter/Ki", 0.0);
    public static final LoggedTunableNumber kKd = new LoggedTunableNumber("Shooter/Kd", 0.0);
    public static final LoggedTunableNumber kKs = new LoggedTunableNumber("Shooter/Ks", 0.0);
    public static final LoggedTunableNumber kKv = new LoggedTunableNumber("Shooter/Kv", 0.118);
    public static final LoggedTunableNumber kKa = new LoggedTunableNumber("Shooter/Ka", 0.0);

    public static final LoggedTunableNumber kIdleRpm =
        new LoggedTunableNumber("Shooter/IdleRpm", 1500.0);
    public static final LoggedTunableNumber kToleranceRpm =
        new LoggedTunableNumber("Shooter/ToleranceRpm", 75.0);
    public static final LoggedTunableNumber kMaxRpm =
        new LoggedTunableNumber("Shooter/MaxRpm", 6000.0);
    public static final LoggedTunableNumber kSpinUpTimeoutSec =
        new LoggedTunableNumber("Shooter/SpinUpTimeout", 3.0);

    /** FlywheelSim moment of inertia, kg*m^2, wheels plus shafts. TODO: from CAD. */
    public static final double kMoiKgM2 = 0.004;
  }

  // ------------------------------------------------------------------- shooting-on-the-move ----

  /**
   * The shot table and solver tuning. Every array here must be measured before the robot scores.
   */
  public static final class Shot {
    private Shot() {}

    /**
     * Distances, meters, at which the shot table is sampled. Must be ascending and the same length
     * as {@link #kLaunchSpeedMetersPerSec} and {@link #kTimeOfFlightSec}.
     *
     * <p>TODO: MEASURE at 8-10 distances spanning your usable range.
     */
    public static final double[] kDistanceBreakpointsMeters = {1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0};

    /**
     * Ground-frame launch speed, m/s, needed to reach the goal from each breakpoint distance.
     *
     * <p>TODO: MEASURE. Stand still at each distance, sweep flywheel RPM until you score reliably,
     * then convert that RPM to surface speed. All zeros means the solver has no idea how to shoot.
     */
    public static final double[] kLaunchSpeedMetersPerSec = {0, 0, 0, 0, 0, 0, 0};

    /**
     * Time of flight, seconds, at each breakpoint distance.
     *
     * <p>TODO: MEASURE from the same test, using video at 120 fps and counting frames from release
     * to arrival. This is what makes shooting on the move work; a wrong TOF aims at the wrong
     * place.
     */
    public static final double[] kTimeOfFlightSec = {0, 0, 0, 0, 0, 0, 0};

    /** Slope of the muzzle-speed to flywheel-RPM fit. TODO: fit from your measured shot data. */
    public static final LoggedTunableNumber kRpmPerMeterPerSec =
        new LoggedTunableNumber("Shot/RpmPerMps", 0.0);

    /** Intercept of the muzzle-speed to flywheel-RPM fit. TODO: fit from your shot data. */
    public static final LoggedTunableNumber kRpmIntercept =
        new LoggedTunableNumber("Shot/RpmIntercept", 0.0);

    /** Hard cap on solver iterations, so a non-converging solve cannot stall the loop. */
    public static final int kMaxSolverIterations = 6;

    /** Time-of-flight change below which the solver declares convergence, seconds. */
    public static final LoggedTunableNumber kSolverToleranceSec =
        new LoggedTunableNumber("Shot/SolverTolerance", 0.002);

    /**
     * 0 disables acceleration compensation, 1 applies it fully. Start at 0; raise once velocity
     * works.
     */
    public static final LoggedTunableNumber kAccelCompensationScalar =
        new LoggedTunableNumber("Shot/AccelComp", 0.0);

    /** Above this chassis acceleration the shot is refused, m/s^2. */
    public static final LoggedTunableNumber kMaxChassisAccelForShotMetersPerSec2 =
        new LoggedTunableNumber("Shot/MaxAccel", 3.0);

    public static final LoggedTunableNumber kMaxVisionStalenessSec =
        new LoggedTunableNumber("Shot/MaxVisionStaleness", 0.6);
    public static final LoggedTunableNumber kMaxPoseStdDevMeters =
        new LoggedTunableNumber("Shot/MaxPoseStdDev", 0.20);
    public static final LoggedTunableNumber kMinDistanceMeters =
        new LoggedTunableNumber("Shot/MinDistance", 1.0);
    public static final LoggedTunableNumber kMaxDistanceMeters =
        new LoggedTunableNumber("Shot/MaxDistance", 7.0);
  }

  // ------------------------------------------------------------------------------- the vision ----

  /** Camera names, mounting transforms, and vision trust policy. */
  public static final class Vision {
    private Vision() {}

    public static final String kTurretCameraName = "limelight-turret";
    public static final String kIntakeCameraName = "limelight-intake";

    /**
     * Robot origin to the turret's rotation axis. Only the translation and any fixed tilt live
     * here; the turret's own rotation is composed in at runtime.
     *
     * <p>TODO: MEASURE from CAD.
     */
    public static final Transform3d kRobotToTurretAxis =
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

    /**
     * Turret frame at 0 rad to the turret camera lens. TODO: MEASURE from CAD, verify with a tag.
     */
    public static final Transform3d kTurretToCamera =
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

    /**
     * Robot origin to the intake camera lens; fixed, since the camera rides the frame. TODO:
     * MEASURE.
     */
    public static final Transform3d kRobotToIntakeCamera =
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

    /**
     * Robot origin to the Quest 3S headset origin.
     *
     * <p>TODO: MEASURE carefully. This is the single biggest source of QuestNav pose error, and an
     * error here looks exactly like drift, which sends people hunting in the wrong place.
     */
    public static final Transform3d kRobotToQuest =
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

    /** Base MegaTag2 XY standard deviation at 1 m with 1 tag. Scaled by distance^2 / tagCount. */
    public static final LoggedTunableNumber kMegaTag2XyStdDevBase =
        new LoggedTunableNumber("Vision/MT2XyStdDev", 0.04);

    /** Deliberately enormous: MegaTag2 heading is an input to the solve, not an output of it. */
    public static final LoggedTunableNumber kMegaTag2ThetaStdDev =
        new LoggedTunableNumber("Vision/MT2ThetaStdDev", 9999.0);

    public static final LoggedTunableNumber kQuestXyStdDev =
        new LoggedTunableNumber("Vision/QuestXyStdDev", 0.02);
    public static final LoggedTunableNumber kQuestThetaStdDev =
        new LoggedTunableNumber("Vision/QuestThetaStdDev", 0.035);

    /** Reject any vision pose farther than this from the current estimate, meters. */
    public static final LoggedTunableNumber kMaxAcceptedPoseJumpMeters =
        new LoggedTunableNumber("Vision/MaxJump", 1.0);

    public static final LoggedTunableNumber kMaxAmbiguity =
        new LoggedTunableNumber("Vision/MaxAmbiguity", 0.3);
    public static final LoggedTunableNumber kMaxZErrorMeters =
        new LoggedTunableNumber("Vision/MaxZError", 0.4);

    /** Reject turret-camera frames captured while the turret is slewing faster than this, rad/s. */
    public static final LoggedTunableNumber kMaxTurretRateForVisionRadPerSec =
        new LoggedTunableNumber("Vision/MaxTurretRate", 1.5);

    /** Below this headset battery percentage, stop trusting QuestNav. */
    public static final LoggedTunableNumber kQuestMinBatteryPercent =
        new LoggedTunableNumber("Vision/QuestMinBattery", 15.0);

    public static final LoggedTunableNumber kQuestReseedResidualMeters =
        new LoggedTunableNumber("Vision/QuestReseedResidual", 0.30);
    public static final LoggedTunableNumber kQuestReseedHoldSec =
        new LoggedTunableNumber("Vision/QuestReseedHold", 0.5);

    /**
     * How fast the pose-confidence estimate degrades while no camera is contributing, in metres of
     * effective standard deviation per second of staleness.
     *
     * <p>0.5 means that one second after the last accepted vision fix, a 4 cm fix is treated as
     * being worth about 54 cm -- which will fail the shot gate, as it should. Raise it to become
     * more conservative about shooting on stale vision.
     */
    public static final LoggedTunableNumber kStalenessGrowthMetersPerSec =
        new LoggedTunableNumber("Vision/StalenessGrowth", 0.5);
  }

  // --------------------------------------------------------------------------- power & faults ----

  /** Brownout prediction thresholds for {@link frc.robot.util.PowerBudget}. */
  public static final class Power {
    private Power() {}

    public static final LoggedTunableNumber kBrownoutWarningVolts =
        new LoggedTunableNumber("Power/BrownoutWarnVolts", 7.5);
    public static final LoggedTunableNumber kBrownoutCriticalVolts =
        new LoggedTunableNumber("Power/BrownoutCritVolts", 6.9);
    public static final LoggedTunableNumber kTotalCurrentBudgetAmps =
        new LoggedTunableNumber("Power/TotalBudget", 250.0);
    public static final LoggedTunableNumber kMinCurrentScalar =
        new LoggedTunableNumber("Power/MinScalar", 0.5);
  }

  // ------------------------------------------------------------------------------ autonomous ----

  /** Autonomous routine tuning, consumed by the {@code frc.robot.autos} package. */
  public static final class Auto {
    private Auto() {}

    public static final LoggedTunableNumber kScoreTimeoutSec =
        new LoggedTunableNumber("Auto/ScoreTimeout", 2.0);
    public static final LoggedTunableNumber kIntakeSettleSec =
        new LoggedTunableNumber("Auto/IntakeSettle", 0.25);

    /** Warn in the queue line when the robot is placed further than this from the auto's start. */
    public static final LoggedTunableNumber kStartPoseWarnMeters =
        new LoggedTunableNumber("Auto/StartPoseWarn", 0.15);

    public static final LoggedTunableNumber kPathfindMaxVel =
        new LoggedTunableNumber("Auto/PathfindMaxVel", 3.0);
    public static final LoggedTunableNumber kPathfindMaxAccel =
        new LoggedTunableNumber("Auto/PathfindMaxAccel", 3.0);
    public static final LoggedTunableNumber kPathfindMaxAngVel =
        new LoggedTunableNumber("Auto/PathfindMaxAngVel", 6.0);
    public static final LoggedTunableNumber kPathfindMaxAngAccel =
        new LoggedTunableNumber("Auto/PathfindMaxAngAccel", 12.0);

    /** How many pickup attempts the vision-driven scavenge routine will make. */
    public static final int kMaxScavengeCycles = 3;
  }

  // --------------------------------------------------------------------------------- controls ----

  /** Driver and operator controller layout and feel. */
  public static final class Controls {
    private Controls() {}

    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final LoggedTunableNumber kTriggerThreshold =
        new LoggedTunableNumber("Controls/TriggerThreshold", 0.25);
    public static final LoggedTunableNumber kTurretNudgeRadPerSec =
        new LoggedTunableNumber("Controls/TurretNudgeRate", 0.5);
    public static final LoggedTunableNumber kRpmTrimStep =
        new LoggedTunableNumber("Controls/RpmTrimStep", 50.0);
    public static final LoggedTunableNumber kReadyRumbleStrength =
        new LoggedTunableNumber("Controls/ReadyRumble", 0.4);
  }
}
