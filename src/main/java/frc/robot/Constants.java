package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Quaternion;
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
     * Blue-alliance pass destination X, meters, blue-origin coordinates.
     *
     * <p>TODO: MEASURE. A pass is a lob to a teammate or to a staging area, so this is a tactical
     * choice about the REBUILT field rather than a dimension that can be read off a drawing -- pick
     * it once the field layout is known and the team has decided where pieces should be dropped.
     * Left at the origin it is not a guess, it is an admission: {@code Superstructure}'s PASSING
     * state aims at it and commands {@link Superstructure#kPassRpm}, which is also zero, so passing
     * is inert rather than wrong until both are chosen.
     */
    public static final LoggedTunableNumber kPassTargetXMeters =
        new LoggedTunableNumber("Field/PassTargetX", 0.0);

    /** Blue-alliance pass destination Y, meters. TODO: MEASURE, same decision as above. */
    public static final LoggedTunableNumber kPassTargetYMeters =
        new LoggedTunableNumber("Field/PassTargetY", 0.0);

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

    /**
     * The pass destination, already flipped for the current alliance.
     *
     * @return the pass target in meters, in the current alliance's frame
     */
    public static Translation2d passTranslation() {
      // Flipped on every call for the same reason goalTranslation() is: the alliance is not known
      // until the driver station connects, which is always after this class has initialized.
      return AllianceFlipUtil.apply(
          new Translation2d(kPassTargetXMeters.get(), kPassTargetYMeters.get()));
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

    /**
     * Motor rotations per intake-roller rotation. TODO: MEASURE by counting gear teeth.
     *
     * <p>Added for M5. The roller needs its own ratio so that {@code SensorToMechanismRatio} can be
     * configured on the roller Talon as well as the pivot one, which is what lets {@link
     * #kIntakeRollerRpm} mean RPM at the roller shaft rather than at the rotor.
     */
    public static final double kRollerGearRatio = 1.0;

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

    /**
     * The partial-raise angle the intake rocks up to while scoring. Added for M10b (plan section
     * 2.4).
     *
     * <p>Between {@link #kDeployedAngleRad} and {@link #kStowedAngleRad}. Two properties of this
     * robot make it the upper end of the rock: pieces stay in the hopper at this height (they can
     * fall out of a stowed intake), and it is clear of the turret, outside {@link
     * #kTurretConflictBandRad}. It is also the upper bound of the range in which {@code
     * Intake.isRetainingPieces()} reports true, which autos wait on before their first path.
     *
     * <p>TODO: MEASURE. Raise the intake slowly over a full hopper until it just starts pushing the
     * pile toward the indexer, and read Intake/Inputs/pivotPositionRad.
     */
    public static final LoggedTunableNumber kFeedAssistAngleRad =
        new LoggedTunableNumber("Intake/FeedAssistAngle", 0.0);

    /**
     * Intake-roller speed while feed-assisting, RPM, inward. Added for M10b.
     *
     * <p>Inward so that pieces sitting at the mouth are pulled onto the hopper floor rather than
     * pushed back out as the arm rises. Lower than {@link #kIntakeRollerRpm} because nothing new is
     * being collected; it only has to keep what is already at the mouth moving the right way. TODO:
     * tune on the robot.
     */
    public static final LoggedTunableNumber kFeedAssistRollerRpm =
        new LoggedTunableNumber("Intake/FeedAssistRollerRpm", 1000.0);

    /**
     * Full period of the feed-assist rock between {@link #kDeployedAngleRad} and {@link
     * #kFeedAssistAngleRad}, seconds. Added for M10b.
     *
     * <p>Rocking is the intended behaviour: it lets pieces flow off the intake onto the hopper
     * floor and into the indexer better than a static raise. Zero holds the intake still at {@link
     * #kFeedAssistAngleRad}, which is kept as a diagnostic and is also what the intake falls back
     * to on its own when {@code PowerBudget} starts cutting LOW-priority current, since rocking is
     * pure pivot current.
     *
     * <p>TODO: tune on the robot. Too short and the pivot never reaches either end before it
     * reverses (watch Intake/AtGoal); too long and the flow stalls between strokes.
     */
    public static final LoggedTunableNumber kFeedAssistPeriodSec =
        new LoggedTunableNumber("Intake/FeedAssistPeriod", 0.5);

    /**
     * Width of the turret-conflict band, radians, measured from {@link #kStowedAngleRad} toward
     * {@link #kDeployedAngleRad}. Added for M10b.
     *
     * <p>While any part of the pivot's position or commanded motion lies inside this band, the arm
     * is in the turret's sweep. On this robot that is roughly the top 15 degrees of travel: the
     * STOWED intake conflicts with the turret, while the deployed and feed-assist positions, and
     * everything between them, do not. That replaces the M10 interlock's assumption, which had it
     * backwards and treated a deployed intake as the conflict.
     *
     * <p>A WIDTH FROM STOW rather than two absolute angles, for two reasons. The band's direction
     * then follows whatever sign convention the pivot ends up with, and the placeholder below is
     * already the real shape. Two absolute placeholders of 0.0 would describe an empty band and
     * leave the interlock silently inert -- the identity-transform trap M8 hit, in a new costume.
     *
     * <p>TODO: MEASURE the exact edge from the Onshape model (tools/onshape_measurements.py reads
     * named mate connectors; add one at the band edge) and confirm it on the robot with the turret
     * parked just outside its conflict arc. Add margin rather than trimming it: too wide costs a
     * fraction of a second of turret travel, too narrow costs the arm.
     */
    public static final LoggedTunableNumber kTurretConflictBandRad =
        new LoggedTunableNumber("Intake/TurretConflictBand", Math.toRadians(15.0));

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
    /**
     * Gravity feedforward for the pivot, in volts, applied as {@code kPivotKg * cos(angle)}.
     *
     * <p>THE SIGN CONVENTION, written down once, because getting it wrong makes the pivot slam into
     * a hard stop at full output. The convention every other Intake angle constant is measured in:
     *
     * <ul>
     *   <li>The angle is at the gearbox output, measured from HORIZONTAL, so {@code cos} is 1 where
     *       gravity's torque about the pivot is largest and 0 where the arm points straight up or
     *       straight down. A datum measured from anywhere else makes the cosine term peak in the
     *       wrong place, which reads as "the feedforward helps at one end of travel and fights at
     *       the other".
     *   <li>Positive angle RAISES the arm, matching {@code SingleJointedArmSim}'s convention so the
     *       simulation and the robot agree.
     *   <li>Positive motor output must therefore also raise the arm. That is what {@link
     *       #kPivotInverted} is for; set it so this holds before tuning anything else.
     *   <li>With all of the above true, kPivotKg is POSITIVE: holding the arm level against gravity
     *       takes positive volts.
     * </ul>
     *
     * <p>TODO: MEASURE. Put the pivot at horizontal, in brake, and jog open-loop voltage until the
     * arm just holds itself; paste that voltage, with its sign. If the value that holds it is
     * negative, positive output is lowering the arm and {@link #kPivotInverted} is wrong -- fix
     * that rather than storing a negative kG, or the closed loop will be fighting its own
     * feedforward everywhere except horizontal.
     */
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

    /**
     * Intake-roller velocity gains. Added for M5: the skeleton gave the pivot a full gain set but
     * left the roller with none, and {@link #kIntakeRollerRpm} is a closed-loop goal.
     *
     * <p>kP plus a kS/kV feedforward, with no integrator and no derivative, for the reason every
     * roller in this project runs that way: an integrator winds up while a game piece stalls the
     * roller and then dumps that output the instant the piece clears, which throws the piece.
     */
    public static final LoggedTunableNumber kRollerKp =
        new LoggedTunableNumber("Intake/RollerKp", 0.1);

    public static final LoggedTunableNumber kRollerKs =
        new LoggedTunableNumber("Intake/RollerKs", 0.0);
    public static final LoggedTunableNumber kRollerKv =
        new LoggedTunableNumber("Intake/RollerKv", 0.12);

    /**
     * Open-loop duty cycle used to drive the pivot into a hard stop while homing.
     *
     * <p>The SIGN chooses which hard stop is searched for, and it must point at whichever stop
     * {@link #kStowedAngleRad} names -- because {@code Intake.onExit(HOMING)} seeds the pivot to
     * that angle on arrival. Negative, as shipped, means the stow stop is assumed to be the one
     * reached by LOWERING the arm (see the angle convention on {@link #kPivotKg}). TODO: MEASURE --
     * if this intake stows upward, flip this sign, or homing will seed the stow angle at the deploy
     * stop and every commanded position will be wrong by the full travel of the mechanism.
     *
     * <p>The MAGNITUDE wants to be just enough to reach the stop and stall there. Too much and the
     * pivot arrives hard enough to bounce, which reads as velocity and restarts the settle timer.
     */
    public static final LoggedTunableNumber kHomingDutyCycle =
        new LoggedTunableNumber("Intake/HomingDutyCycle", -0.15);

    public static final LoggedTunableNumber kHomingVelocityThresholdRadPerSec =
        new LoggedTunableNumber("Intake/HomingVelThreshold", 0.15);
    public static final LoggedTunableNumber kHomingSettleSec =
        new LoggedTunableNumber("Intake/HomingSettle", 0.25);
    public static final LoggedTunableNumber kHomingTimeoutSec =
        new LoggedTunableNumber("Intake/HomingTimeout", 3.0);

    /**
     * Pivot stator current above which the mechanism counts as pressed against its hard stop.
     *
     * <p>Added for M5. Velocity alone cannot tell "stalled against the stop" apart from "not moving
     * because nothing is driving it", and the second case happens for real: a software limit, a
     * disabled robot, or a snapped belt all read as zero velocity while the homing routine happily
     * declares success and seeds a position that is wrong by the whole travel of the mechanism.
     * Requiring current as well as stillness makes the stall the thing being detected.
     *
     * <p>TODO: MEASURE. Drive the pivot into its stop at {@link #kHomingDutyCycle} and read
     * Intake/Inputs/pivotStatorCurrentAmps; take roughly half of what you see, and keep it well
     * under {@link #kPivotStatorLimitAmps}. The default of 0.0 deliberately disables this half of
     * the check, leaving the plan's velocity-only detection, so an unmeasured robot behaves exactly
     * as specified rather than failing to home for a reason nobody has written down yet.
     */
    public static final LoggedTunableNumber kHomingCurrentAmps =
        new LoggedTunableNumber("Intake/HomingCurrent", 10.0);

    public static final double kPivotStatorLimitAmps = 60.0;
    public static final double kRollerStatorLimitAmps = 80.0;

    /**
     * How far beyond the outermost commanded angle the Talon's soft limits sit, in radians.
     *
     * <p>Added for M5. The soft limits exist to stop a runaway closed loop, not to define travel --
     * the hard stops do that. Setting them exactly at the stow and deploy angles would make homing
     * impossible, because homing works by driving INTO a hard stop and the soft limit would cut the
     * output just short of it every time.
     *
     * <p>Roughly 6 degrees. Widen it if homing stalls short of the stop; narrow it if a runaway
     * gets far enough to do damage before the limit catches it.
     */
    public static final double kPivotSoftLimitMarginRad = 0.10;

    /**
     * What {@code Intake} assumes about the turret interlock when no deploy guard has been
     * installed. Added for M5.
     *
     * <p>{@code Superstructure} injects the real guard (see plan section 4.11), and {@code
     * Superstructure} is a skeleton until M10. Until then this is the answer the intake gets, and
     * it is FALSE: with nothing tracking where the turret is, the honest answer to "is it safe to
     * swing the intake out" is "unknown", and unknown must not mean yes for the one interaction the
     * plan calls the most likely way to destroy this robot.
     *
     * <p>The visible consequence in the interim is that DEPLOY_AND_INTAKE, DEPLOY_ONLY and EJECT
     * all resolve to STOWING, which {@code Intake/DeployAllowed} and the "no deploy guard
     * installed" alert both make obvious rather than mysterious. Flip this to true ONLY to bench
     * test the pivot with the turret physically removed or known parked, and flip it back.
     */
    public static final boolean kDeployAllowedWithoutGuard = false;

    /** Pivot moment of inertia for SingleJointedArmSim, kg*m^2. TODO: pull from CAD. */
    public static final double kPivotMoiKgM2 = 0.05;

    /** Pivot arm length for SingleJointedArmSim, meters. TODO: pull from CAD. */
    public static final double kPivotLengthMeters = 0.35;

    /**
     * Intake-roller moment of inertia for DCMotorSim, kg*m^2. TODO: pull from CAD.
     *
     * <p>The value the IntakeIOSim skeleton hard-coded, moved here so that every guessed-but-
     * tunable number in the project lives in one file, which is where anyone sweeping for guesses
     * before a competition will look.
     */
    public static final double kRollerMoiKgM2 = 0.001;
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

    /**
     * How long after a speed change the load predicate is ignored.
     *
     * <p>Spin-up from rest satisfies both halves of the load test at once -- high current AND a
     * large shortfall against the goal -- so an empty indexer told to FEED reported a phantom piece
     * and then "lost" it on reaching speed, which the shot counter recorded as a shot. Debouncing
     * was relied on to outlast that transient and did not. Ignoring the predicate until the wheel
     * has had time to settle removes the failure by construction rather than by timing margin.
     *
     * <p>Survives the M10b hopper retrofit, which deletes the piece detector it was written for.
     * The same spin-up transient would fool both of M10b's replacements -- high current at low
     * speed reads as {@link #kLoadedCurrentAmps} "loaded" AND as {@link #kJamCurrentAmps} "jammed"
     * -- so {@code isLoaded()} and jam detection also ignore the wheel for this long after every
     * goal change.
     *
     * <p>TODO: MEASURE -- read the spin-up duration from Indexer/Inputs and set this comfortably
     * above it.
     */
    public static final LoggedTunableNumber kDetectionSettleSec =
        new LoggedTunableNumber("Indexer/DetectionSettleSec", 0.4);

    public static final LoggedTunableNumber kHasPieceDebounceSec =
        new LoggedTunableNumber("Indexer/HasPieceDebounce", 0.06);
    public static final LoggedTunableNumber kShotDetectDebounceSec =
        new LoggedTunableNumber("Indexer/ShotDebounce", 0.08);

    /**
     * Stator current above which the feeding indexer counts as pushing against something, amps.
     * Added for M10b (plan section 2.4).
     *
     * <p>"Loaded" is the hopper-era replacement for "has a piece". It is evaluated only while
     * FEEDING forward, where it can mean something, and it gates nothing: it only stops {@code
     * Superstructure.likelyEmpty()} from concluding the hopper is empty while the indexer is
     * visibly still moving pieces. Both halves are required, as with the old detector, because a
     * current spike alone fires on every acceleration.
     *
     * <p>Starts at the same values as {@link #kHasPieceCurrentAmps} and {@link
     * #kHasPieceVelocityDropRpm}, which it supersedes; those are deleted in M10b's final step.
     * TODO: tune from Indexer/Inputs while feeding a full hopper and then an empty one.
     */
    public static final LoggedTunableNumber kLoadedCurrentAmps =
        new LoggedTunableNumber("Indexer/LoadedCurrent", 25.0);

    /** Velocity shortfall below goal that, with {@link #kLoadedCurrentAmps}, means loaded, RPM. */
    public static final LoggedTunableNumber kLoadedVelocityDropRpm =
        new LoggedTunableNumber("Indexer/LoadedVelDrop", 400.0);

    /** Debounce on the loaded predicate, both edges, seconds. Added for M10b. */
    public static final LoggedTunableNumber kLoadedDebounceSec =
        new LoggedTunableNumber("Indexer/LoadedDebounce", 0.06);

    /**
     * Stator current above which a feeding indexer that has also stopped turning is jammed, amps.
     * Added for M10b (plan section 2.4).
     *
     * <p>With a whole hopper pressing on the indexer, a jam is a matter of when, not if. A jam is
     * this current AND velocity below {@link #kJamVelocityRpm}, sustained for {@link
     * #kJamDetectSec}. It must sit clearly above the loaded threshold: a loaded wheel is slowed, a
     * jammed one is stopped, and the gap between the two is what keeps a full hopper from being
     * mistaken for a jam. TODO: MEASURE by stalling the indexer on purpose and reading the current.
     */
    public static final LoggedTunableNumber kJamCurrentAmps =
        new LoggedTunableNumber("Indexer/JamCurrent", 40.0);

    /** Speed below which a high-current indexer counts as stopped for jam detection, RPM. */
    public static final LoggedTunableNumber kJamVelocityRpm =
        new LoggedTunableNumber("Indexer/JamVelocity", 200.0);

    /**
     * How long the jam condition must hold before the indexer reverses, seconds. Long enough that a
     * piece briefly catching on its way through is not treated as a jam. TODO: tune on the robot.
     */
    public static final LoggedTunableNumber kJamDetectSec =
        new LoggedTunableNumber("Indexer/JamDetect", 0.15);

    /** Reverse speed of an unjam pulse, RPM. Negative: backwards, away from the flywheel. */
    public static final LoggedTunableNumber kUnjamRpm =
        new LoggedTunableNumber("Indexer/UnjamRpm", -1500.0);

    /** Duration of one unjam pulse, seconds. TODO: tune on the robot. */
    public static final LoggedTunableNumber kUnjamSec =
        new LoggedTunableNumber("Indexer/Unjam", 0.2);

    /**
     * Unjam pulses allowed within one FEED request before the indexer gives up, latches JAMMED and
     * raises an alert. Added for M10b.
     *
     * <p>Bounded because a jam that three reversals cannot clear will not be cleared by a fourth,
     * and an indexer cycling forever presents as "the robot is shooting slowly" rather than as the
     * fault it is. The operator's EJECT is the recovery from there. A plain int rather than a
     * tunable because it is a policy, not a measurement.
     */
    public static final int kMaxUnjamAttempts = 3;

    /**
     * How long an EJECT request runs before the machine parks itself back in IDLE, seconds.
     *
     * <p>A duration rather than a sensor test on purpose: the whole reason to eject is that the
     * piece is jammed or the wrong piece, which is exactly the situation in which piece detection
     * cannot be trusted to tell you when it has gone. A bounded run also stops a held EJECT button
     * from grinding the indexer against a stuck piece for the rest of the match.
     *
     * <p>TODO: tune on the robot. Watch Indexer/Inputs/velocityRpm while ejecting a piece and take
     * the time from the reversal to the piece clearing, plus a margin.
     */
    public static final LoggedTunableNumber kEjectDurationSec =
        new LoggedTunableNumber("Indexer/EjectDuration", 0.75);

    /** Indexer moment of inertia for IndexerIOSim, kg*m^2. TODO: pull from CAD. */
    public static final double kMoiKgM2 = 0.001;

    /**
     * How far past the piece-detection thresholds IndexerIOSim's synthetic piece pushes,
     * dimensionless.
     *
     * <p>Expressed as a multiple of {@link #kHasPieceCurrentAmps} and {@link
     * #kHasPieceVelocityDropRpm} rather than as absolute amps and RPM, so that retuning either
     * threshold on the robot cannot silently stop the simulated piece from being detected -- which
     * would present as "the sim stopped working" long after the change that caused it.
     */
    public static final double kSimPieceDetectionMargin = 1.5;
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

    /**
     * Sustained supply limit, applied after {@link #kSupplyLowerTimeSec} of drawing {@link
     * #kSupplyLimitAmps}.
     *
     * <p>Phoenix 6 supply limiting is two-stage, and a flywheel is the mechanism that most wants
     * that shape: spin-up is a brief large draw, cruising is a small one. A single flat limit would
     * have to be set high enough for spin-up and would then protect nothing for the rest of the
     * match. TODO: MEASURE from a spin-up log once the flywheel exists -- the right value is the
     * steady-state draw at kMaxRpm plus headroom.
     */
    public static final double kSupplyLowerLimitAmps = 40.0;

    /** How long {@link #kSupplyLimitAmps} may be drawn before the lower limit takes over. */
    public static final double kSupplyLowerTimeSec = 1.0;

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

    /**
     * The wider at-speed tolerance used once a burst is already feeding, RPM. Added for M10b (plan
     * section 2.4).
     *
     * <p>Every piece knocks the wheel down. Judged against {@link #kToleranceRpm}, the readiness
     * gate would close after each piece and reopen as the wheel recovered, stopping and restarting
     * the indexer several times a second. Once the feed has opened within a scoring state, the gate
     * judges the wheel against this band instead, until any OTHER gate term closes it. The first
     * piece of every burst still meets the tight band.
     *
     * <p>TODO: tune during M11 against the hit rate of pieces 2..N in a burst. Too wide and the
     * late pieces fall short; too narrow and the burst stutters.
     */
    public static final LoggedTunableNumber kBurstToleranceRpm =
        new LoggedTunableNumber("Shooter/BurstToleranceRpm", 300.0);

    /**
     * Loop-to-loop speed drop that counts as one piece passing through the wheels, RPM. Added for
     * M10b (plan section 2.4).
     *
     * <p>With a hopper and no piece sensor, the flywheel is the most reliable piece detector on the
     * robot: every scored piece passes through it and takes a sharp bite out of wheel speed. A drop
     * this large, while the wheel was within {@link #kShotArmToleranceRpm} of its goal, is a shot.
     *
     * <p>TODO: MEASURE from a logged burst. The dip must sit well above loop-to-loop noise at speed
     * and well below the dip a real piece causes. The 8-piece preload is a known count, so firing
     * it is the easiest check.
     */
    public static final LoggedTunableNumber kShotDipRpm =
        new LoggedTunableNumber("Shooter/ShotDipRpm", 150.0);

    /**
     * How close to its goal the wheel must be for a speed drop to count as a shot, RPM. Added for
     * M10b.
     *
     * <p>The arming condition is what stops a lowered goal or a spin-up transient from registering
     * as a shot. Wider than {@link #kBurstToleranceRpm}, because in a fast burst the wheel may
     * still be recovering from the previous piece when the next one arrives, and that piece must
     * count.
     */
    public static final LoggedTunableNumber kShotArmToleranceRpm =
        new LoggedTunableNumber("Shooter/ShotArmTolerance", 400.0);

    /**
     * After a shot is counted, further speed drops are ignored for this long, seconds. Added for
     * M10b.
     *
     * <p>Stops one piece's dip, which can span more than one 20 ms loop, from counting twice. TODO:
     * MEASURE the dip width in a logged burst first. If back-to-back pieces merge into one dip,
     * shrinking this is the wrong fix; count dips in the high-rate velocity samples instead.
     */
    public static final LoggedTunableNumber kShotRefractorySec =
        new LoggedTunableNumber("Shooter/ShotRefractory", 0.04);

    /**
     * How far past {@link #kShotDipRpm} a simulated piece knocks the wheel, dimensionless. Added
     * for M10b.
     *
     * <p>{@code ShooterIOSim.injectShot()} removes {@code kShotDipRpm * kSimShotDipMargin} RPM in a
     * single step. Expressed as a multiple rather than as absolute RPM, for the same reason the
     * single-piece indexer's {@code kSimPieceDetectionMargin} was: retuning the detector on the
     * robot must not silently stop the simulated piece from being detected, which would present as
     * "the sim stopped counting shots" long after the change that caused it. Below 1.0 the injected
     * piece would be invisible to the detector by construction.
     */
    public static final double kSimShotDipMargin = 2.0;

    /**
     * Speed commanded by the tuning-mode bench harness in {@code RobotContainer}.
     *
     * <p>This exists so the M4 checkpoint can be run end to end from AdvantageScope with no code
     * change and no controller: tick {@code /Tuning/Shooter/BenchSpinUp} to spin the wheel, then
     * edit {@code Shooter/Kp} and this value in the same pane and watch the response change.
     *
     * <p>Note that this value is only read on the rising edge of that toggle, so changing it while
     * the wheel is already spinning does nothing until you toggle off and back on. That is
     * deliberate -- a live-following bench speed would make the flywheel chase every keystroke
     * while somebody is typing a four-digit number.
     *
     * <p>The harness is reachable only while {@link Constants#kTuningMode} is true, so it cannot be
     * triggered at an event.
     */
    public static final LoggedTunableNumber kBenchTestRpm =
        new LoggedTunableNumber("Shooter/BenchTestRpm", 3000.0);

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

  // ----------------------------------------------------------------------- the superstructure ----

  /**
   * Cross-mechanism timing and thresholds owned by {@code Superstructure}.
   *
   * <p>Everything here describes a decision no single mechanism can make on its own. The
   * per-mechanism tolerances that feed the readiness gate live with their mechanisms, and the six
   * shot-quality thresholds live in {@link Shot}; this class holds only what is genuinely about the
   * robot as a whole.
   */
  public static final class Superstructure {
    private Superstructure() {}

    /**
     * How long the robot-level homing sequence may run before it gives up and faults, seconds.
     *
     * <p>Deliberately longer than <b>both</b> mechanism homing timeouts ({@link
     * Intake#kHomingTimeoutSec} at 3 s and {@link Turret#kHomingTimeoutSec} at 5 s). A mechanism
     * that cannot home must fault first, because its own fault names the mechanism and raises the
     * alert a pit crew can act on. If this timeout fired first the only thing anyone would see is
     * "the superstructure faulted", which is true of both mechanisms and therefore useless.
     */
    public static final LoggedTunableNumber kHomingTimeoutSec =
        new LoggedTunableNumber("Superstructure/HomingTimeout", 7.0);

    /**
     * How fast the chassis may be translating and still count as stationary for {@code
     * SCORING_STATIONARY}, in meters per second.
     *
     * <p>This is the ninth term the plan adds to the readiness gate for the stationary shot only.
     * 0.15 m/s is a robot that has stopped and is settling on its wheels rather than one still
     * creeping into position; the point of asking for a stationary shot is that the velocity
     * compensation is taken out of the answer, and a robot that is still moving has put it back in
     * without telling the solver.
     *
     * <p>TODO: tune during M11 alongside the shot tables. Start loose and tighten while watching
     * the hit rate, exactly as plan section 6.5 prescribes for every other term.
     */
    public static final LoggedTunableNumber kMaxChassisSpeedForStationaryShotMetersPerSec =
        new LoggedTunableNumber("Superstructure/MaxStationaryShotSpeed", 0.15);

    /**
     * Flywheel speed used for a pass, in RPM.
     *
     * <p>TODO: MEASURE. A pass is a lob over a known distance to {@link Field#passTranslation()},
     * so this depends on both the REBUILT field and the chosen pass destination, and the project's
     * standing rule is that field-dependent numbers are never guessed. Zero is not a placeholder
     * that will quietly misbehave: {@code Shooter}'s zero-goal guard resolves a zero goal to IDLE
     * rather than faulting, so PASSING aims correctly and simply declines to fire until somebody
     * measures this.
     */
    public static final LoggedTunableNumber kPassRpm =
        new LoggedTunableNumber("Superstructure/PassRpm", 0.0);

    /**
     * Gate-open, unloaded feeding with no shot detected before {@code likelyEmpty()} turns true,
     * seconds. Added for M10b (plan section 2.4).
     *
     * <p>With no piece sensor, "the hopper is empty" can only be a belief: the robot has been
     * trying to fire for this long and nothing has come out. Autos use it to end a scoring leg
     * early, and the driver controller rumbles on it; it NEVER gates a teleop shot. Only time spent
     * with the readiness gate open and the indexer feeding unloaded counts. Time with the gate
     * closed says nothing about the hopper, and counting it would end an auto's scoring leg while
     * the hopper is still full.
     *
     * <p>TODO: MEASURE. It must be longer than the longest gap between Shooter/ShotDetected pulses
     * in a real burst, or an auto will drive off with pieces aboard. Read the gap distribution off
     * a logged burst and add margin.
     */
    public static final LoggedTunableNumber kEmptyConfirmSec =
        new LoggedTunableNumber("Superstructure/EmptyConfirm", 0.75);

    /**
     * How long the floor roller keeps feeding after a piece has been detected, seconds.
     *
     * <p>The plan asks the INDEXING state to run the floor roller "briefly", and this is that word
     * made measurable. It exists so the roller cannot be left running indefinitely by an operator
     * holding the intake button down with a piece already aboard, which would grind the next piece
     * against the first one.
     */
    public static final LoggedTunableNumber kIndexFeedSec =
        new LoggedTunableNumber("Superstructure/IndexFeedSec", 0.5);
  }

  // ------------------------------------------------------------------------------- the vision ----

  /** Camera names, mounting transforms, and vision trust policy. */
  public static final class Vision {
    private Vision() {}

    public static final String kTurretCameraName = "limelight-turret";
    public static final String kIntakeCameraName = "limelight-intake";

    /**
     * Intake-camera pipeline that runs AprilTag localisation.
     *
     * <p>Plain {@code static final int}s rather than tunables, because these index pipelines
     * configured in the Limelight's own web UI. Changing the number here without also changing the
     * pipeline in the camera does not retune anything; it selects the wrong pipeline, which
     * presents as a camera that has silently stopped seeing tags.
     *
     * <p>{@code Superstructure} owns the switching: game pieces while the intake is deployed,
     * AprilTags whenever it is not. The switch costs roughly 100 ms, which is why it is driven from
     * state entry and exit rather than from a per-loop condition that could chatter.
     */
    public static final int kAprilTagPipelineIndex = 0;

    /**
     * Intake-camera pipeline that runs game-piece detection. See {@link #kAprilTagPipelineIndex}.
     */
    public static final int kGamePiecePipelineIndex = 1;

    /**
     * Robot origin to the turret's rotation axis, with the axis frame taken at floor level.
     *
     * <p>MEASURED from {@code Robot.step} via {@code tools/step_inspect.py}: the turret gear's
     * circular extent gives the axis, and the assembly origin was shown to coincide with the FRC
     * robot origin by three independent routes (frame bounding box centred within 1 mm, the four
     * swerve modules symmetric about it, and module hardware bottoming out at exactly Z = 0).
     *
     * <p>The Z is deliberately 0 rather than the gear height. Only the rotation matters here --
     * {@code GeomUtil.rotateAboutZ} spins about the axis's own Z and the height does not affect
     * that -- so putting the frame on the floor keeps {@code kTurretToCamera} readable as "how far
     * up and out the camera sits", instead of splitting the height across two constants.
     *
     * <p>Rotation is identity: turret zero points ROBOT RIGHT, and that offset is carried in {@code
     * kTurretToCamera} rather than here, so this constant stays a pure position.
     */
    public static final Transform3d kRobotToTurretAxis =
        new Transform3d(new Translation3d(0.167638, -0.187325, 0.0), new Rotation3d());

    /**
     * Turret frame at 0 rad to the turret camera lens.
     *
     * <p>MEASURED from CAD. The lens was located from the geometry rather than guessed: a
     * Limelight's model carries a concentric barrel of circles stepping along the optical axis, and
     * its centre sits at the exact lateral centre of the housing, which is a free check that the
     * right feature was found. Using the housing's bounding-box centre instead would have put the
     * camera about 19 mm behind the lens along its own view direction.
     *
     * <p>The CAD is posed with the turret at +90 degrees -- pointing robot-forward -- while turret
     * zero points robot RIGHT, so the extraction was un-rotated by 90 degrees to reach the frame
     * this constant is defined in. Get that wrong and every turret-camera pose is off by the posed
     * angle: exactly the failure M8's checkpoint hunts, arriving from the CAD rather than the code.
     *
     * <p>The rotation is a quaternion because {@code Rotation3d(roll, pitch, yaw)} builds Rz.Ry.Rx,
     * and that order is easy to get backwards and wrong only away from zero. It encodes a camera
     * tilted 20 degrees up and mounted in PORTRAIT -- rolled 90 degrees, so image-up points
     * robot-left. Sanity check: at turret zero this points bearing -90 degrees (right), 20 degrees
     * up.
     *
     * <p>TODO: VERIFY with a tag. CAD does not know how the thing was actually bolted on, and a
     * lens's entrance pupil sits a little behind its front element.
     */
    public static final Transform3d kTurretToCamera =
        new Transform3d(
            new Translation3d(-0.133756, -0.038737, 0.435000),
            new Rotation3d(new Quaternion(-0.405580, 0.579228, -0.405580, 0.579228)));

    /**
     * Robot origin to the intake camera lens, <b>valid only while the intake is at its intaking
     * position</b>.
     *
     * <p>This is NOT a fixed camera, and section 7.2 of the plan is wrong about it: "the intake
     * camera needs none of this: its transform is the constant kRobotToIntakeCamera, pushed once at
     * construction" was written from an assumption the hardware did not honour. The camera rides
     * the pivoting intake, roughly 0.4 m from the pivot axis -- nearly three times the turret
     * camera's 0.139 m lever arm -- so a 60 degree deploy sweeps the lens through about 0.4 m.
     *
     * <p>Rather than give the intake camera the full moving-transform treatment M8 built for the
     * turret, the fusion is GATED: observations are only accepted when the intake was in its
     * intaking position at the moment of capture, which is the one pose this constant describes.
     * See {@code Vision}'s INTAKE_NOT_DEPLOYED rejection. The cost is that the camera contributes
     * nothing while stowed, ejecting, rocking to feed the hopper, or in transit; the benefit is
     * that what it does contribute is correct rather than approximately correct. Since M10b the
     * intake RESTS at this pose (DEPLOYED) for most of a match, so the gate admits that state as
     * well as INTAKING -- see {@code Intake.isAtIntakingAngle()}.
     *
     * <p>MEASURED from CAD in that pose, which is how the assembly happened to be saved. The camera
     * points straight backwards and level, laterally centred to within 0.3 mm -- a good pose for
     * holding tags behind the robot while the turret looks elsewhere.
     */
    public static final Transform3d kRobotToIntakeCamera =
        new Transform3d(
            new Translation3d(-0.614800, -0.000300, 0.387300),
            new Rotation3d(new Quaternion(0.0, 0.0, 0.002886, 0.999996)));

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
     * How slowly the robot must be translating for a QuestNav re-seed to be permitted, in m/s.
     *
     * <p>Section 7.6 requires the robot to be "nearly stationary" before a re-seed without putting
     * a number on it, so this is a judgement rather than a quotation. 0.05 m/s is a robot that has
     * stopped and is settling on its wheels, not one creeping into position: a re-seed writes the
     * current estimate permanently into the headset's frame, and a moving robot's estimate is its
     * worst, because any timestamp mismatch between the headset and the estimator turns into
     * position error in proportion to speed.
     */
    public static final LoggedTunableNumber kQuestReseedMaxSpeedMetersPerSec =
        new LoggedTunableNumber("Vision/QuestReseedMaxSpeed", 0.05);

    /**
     * How slowly the robot must be rotating for a QuestNav re-seed to be permitted, in rad/s.
     *
     * <p>Called out separately from translation because a robot spinning in place has zero
     * translational speed while being the worst possible moment to seed: the headset sits well off
     * the rotation centre, so it is moving quickly even though the chassis is going nowhere. A
     * translation-only check would wave that case straight through.
     */
    public static final LoggedTunableNumber kQuestReseedMaxOmegaRadPerSec =
        new LoggedTunableNumber("Vision/QuestReseedMaxOmega", 0.1);

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

    /**
     * Upper bound on one autonomous scoring leg, seconds.
     *
     * <p>Raised from 2.0 for M10b (plan section 2.4). It was a one-piece timeout; with a hopper it
     * is a max-burst bound, long enough to empty a FULL hopper of 50 pieces. {@code
     * Superstructure.likelyEmpty()} is what normally ends the leg early; this only has to stop a
     * gate that never opens, or a hopper that never seems to empty, from holding the auto for the
     * rest of the period.
     *
     * <p>TODO: MEASURE the time to fire 50 pieces at {@link Indexer#kFeedRpm} and add margin. The
     * placeholder assumes roughly 8 pieces per second.
     */
    public static final LoggedTunableNumber kScoreTimeoutSec =
        new LoggedTunableNumber("Auto/ScoreTimeout", 8.0);

    public static final LoggedTunableNumber kIntakeSettleSec =
        new LoggedTunableNumber("Auto/IntakeSettle", 0.25);

    /**
     * How long an auto waits for the intake to reach a piece-retaining angle before it drives
     * anyway, seconds. Added for M10b (plan section 2.4).
     *
     * <p>The robot must start with the intake STOWED, pieces can fall out of a stowed intake, and
     * the first enable deploys it. So every routine opens by waiting for {@code
     * Intake.isRetainingPieces()}. The bound exists so that a dead pivot costs the auto a second
     * rather than the whole period. TODO: set just above the measured stow-to-feed-assist time.
     */
    public static final LoggedTunableNumber kIntakeRetainTimeoutSec =
        new LoggedTunableNumber("Auto/IntakeRetainTimeout", 1.0);

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
