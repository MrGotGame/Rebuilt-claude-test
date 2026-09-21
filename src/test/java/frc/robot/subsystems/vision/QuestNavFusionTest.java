package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * M9: the QuestNav trust gate, seeding, re-seeding, and the degradation rules.
 *
 * <p>The milestone's checkpoint is two halves. <b>Seed against a tag, cover both Limelights, push
 * the robot 3 m, and the pose still tracks.</b> Then <b>power-cycle the headset mid-run and confirm
 * trust goes false, the warning fires, and the pose degrades to odometry instead of jumping.</b>
 *
 * <p>The first half only means something if odometry alone would <i>not</i> track, so this suite
 * runs the whole thing over a drivetrain whose wheels under-report by {@value
 * #ODOMETRY_SCALE_ERROR} -- the signature of a mis-measured wheel radius or a slipping module, and
 * the exact condition QuestNav exists to paper over between tag sightings. With the cameras blind
 * and the headset dropped, the estimate has nothing left but those wheels and ends the run about 30
 * cm short. {@link #odometryAloneReallyDoesDriftOverThisRun()} asserts that shortfall directly, so
 * that a checkpoint which passes because nothing was ever wrong cannot masquerade as one that
 * passes because the fusion worked.
 *
 * <p>As in M8, the mounting is supplied to both the simulated headset and {@code Vision} with a
 * real offset and a real yaw, because {@code kRobotToQuest} still ships as the identity transform
 * -- and a headset sitting exactly on the robot origin makes the composition direction
 * unobservable.
 */
public class QuestNavFusionTest {
  private static final double LOOP_SEC = 0.02;

  /** Where the robot starts, and where it is pushed from. */
  private static final Pose2d START = new Pose2d(1.0, 2.0, Rotation2d.kZero);

  /** How far the checkpoint pushes the robot, in metres. */
  private static final double PUSH_DISTANCE_METERS = 3.0;

  /** How fast it is pushed, in metres per second. */
  private static final double PUSH_SPEED_METERS_PER_SEC = 1.0;

  /**
   * The fraction of true motion the wheels report.
   *
   * <p>0.90 is a 10 % shortfall: a wheel radius measured 10 % large, or tread worn to the same
   * effect. Chosen because it is large enough to be unambiguous over a 3 m push (30 cm) and small
   * enough to stay inside {@code kMaxAcceptedPoseJumpMeters}, so the observations are fused rather
   * than rejected as teleports -- which would have made the checkpoint pass for the wrong reason.
   */
  private static final double ODOMETRY_SCALE_ERROR = 0.90;

  /**
   * A plausible measured headset mounting: behind and above the robot origin, yawed.
   *
   * <p>The yaw matters as much as the offset. Composing the mounting the wrong way round with a
   * pure translation displaces the pose by twice the offset; with a yaw in it, the error also turns
   * with the robot, which is what makes a mis-composition look like drift rather than like bias.
   */
  private static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          new Translation3d(-0.20, 0.10, 0.80), new Rotation3d(0.0, 0.0, Math.toRadians(15.0)));

  private static final Transform3d NO_TURRET_MOUNTING = new Transform3d();

  private VisionIOSim intakeSim;
  private QuestNavIOSim questSim;
  private Vision vision;

  /** Ground truth, advanced by the test rather than by any simulated physics. */
  private Pose2d truth = START;

  /** Wheel distance as the encoders report it, which is deliberately not the truth. */
  private double reportedWheelDistanceMeters = 0.0;

  /** Brings up the HAL, parks the clock, and builds vision over a simulated headset and camera. */
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialise");
    Constants.disableHal = true;
    SimHooks.pauseTiming();

    RobotState.initialize(new SwerveDriveKinematics(Drive.getModuleTranslations()));

    truth = START;
    reportedWheelDistanceMeters = 0.0;

    intakeSim = new VisionIOSim(Constants.Vision.kIntakeCameraName, () -> truth);
    intakeSim.setNoise(0.0);
    intakeSim.setSolveQuality(3, 2.0, 0.0);

    questSim = new QuestNavIOSim(() -> truth, ROBOT_TO_QUEST);

    vision =
        new Vision(
            new VisionIO() {},
            intakeSim,
            questSim,
            () -> 0.0,
            () -> 0.0,
            // The intake is held at its camera pose throughout, because this suite is about
            // other rules; the gate itself is covered by IntakeCameraGateTest.
            () -> true,
            NO_TURRET_MOUNTING,
            NO_TURRET_MOUNTING,
            ROBOT_TO_QUEST);

    // A clean slate for the headset, since RobotState is a process-wide singleton that outlives
    // any one case. Without this an earlier test's seed would leak in and "not yet seeded" would
    // be untestable.
    RobotState.getInstance().setQuestSeeded(false);
    RobotState.getInstance().setQuestTrusted(false);

    seedOdometry();
  }

  /** Releases the HAL and unregisters the subsystem so cases cannot leak into each other. */
  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().unregisterSubsystem(vision);
    CommandScheduler.getInstance().cancelAll();
    SimHooks.resumeTiming();
    Constants.disableHal = false;
    HAL.shutdown();
  }

  /** Module positions as the encoders currently report them, all four pointing straight ahead. */
  private SwerveModulePosition[] reportedModules() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = new SwerveModulePosition(reportedWheelDistanceMeters, Rotation2d.kZero);
    }
    return positions;
  }

  /**
   * Fills the odometry buffer with the robot stationary at the start pose.
   *
   * <p><b>Outputs:</b> resets the estimator to {@link #START} and pushes a second of samples.
   */
  private void seedOdometry() {
    RobotState.getInstance().resetPose(START, Rotation2d.kZero, reportedModules());
    for (int i = 0; i < 50; i++) {
      RobotState.getInstance()
          .addOdometryObservation(Timer.getFPGATimestamp(), Rotation2d.kZero, reportedModules());
      SimHooks.stepTiming(LOOP_SEC);
    }
    // resetPose clears the seeded flag by design, so re-clear the trust flag that went with it.
    RobotState.getInstance().setQuestSeeded(false);
  }

  /**
   * Runs one loop with the robot moving at a given speed.
   *
   * @param speedMetersPerSec true speed along the field X axis
   *     <p><b>Outputs:</b> advances truth, the under-reporting encoders, odometry, vision and the
   *     clock.
   */
  private void step(double speedMetersPerSec) {
    RobotState.getInstance()
        .addOdometryObservation(Timer.getFPGATimestamp(), Rotation2d.kZero, reportedModules());

    // Speeds arrive by a SEPARATE call from Drive.periodic(), not as a by-product of odometry.
    // Omitting it here left getFieldRelativeSpeeds() reading zero forever, so the re-seed's
    // "nearly stationary" guard was satisfied even at full speed and the test that claimed to
    // exercise it could never have failed. Found by that test reporting three re-seeds during a
    // drive; the guard was right and the harness was lying to it.
    RobotState.getInstance()
        .addDriveSpeedsObservation(
            Timer.getFPGATimestamp(), new ChassisSpeeds(speedMetersPerSec, 0.0, 0.0));

    vision.periodic();

    truth =
        new Pose2d(truth.getX() + speedMetersPerSec * LOOP_SEC, truth.getY(), truth.getRotation());
    reportedWheelDistanceMeters += speedMetersPerSec * LOOP_SEC * ODOMETRY_SCALE_ERROR;
    SimHooks.stepTiming(LOOP_SEC);
  }

  /** Runs several stationary loops. */
  private void stepStationary(int loops) {
    for (int i = 0; i < loops; i++) {
      step(0.0);
    }
  }

  /** How far the fused estimate currently is from where the robot really is, in metres. */
  private double estimateErrorMeters() {
    return RobotState.getInstance()
        .getEstimatedPose()
        .getTranslation()
        .getDistance(truth.getTranslation());
  }

  /**
   * Seeds the headset the way the robot does: with a multi-tag fix in view.
   *
   * <p><b>Outputs:</b> runs enough loops for the first seed and for trust to be granted.
   */
  private void seedAgainstTags() {
    intakeSim.setBlackout(false);
    stepStationary(10);
    assertTrue(
        RobotState.getInstance().isQuestSeeded(),
        "the headset was never seeded with a clean multi-tag fix in view");
  }

  // --------------------------------------------------------------------- seeding, in order ----

  /**
   * An unseeded headset contributes nothing, whatever else is healthy about it.
   *
   * <p>The headset reports in its own frame, whose origin is wherever it happened to be when
   * tracking started. Those poses are not merely imprecise, they are meaningless -- and they arrive
   * with the tightest covariance in the system, so fusing one would drag the estimate off the field
   * entirely.
   */
  @Test
  void anUnseededHeadsetIsNotFused() {
    // No tags, so nothing triggers the first seed.
    intakeSim.setBlackout(true);
    stepStationary(10);

    assertFalse(
        RobotState.getInstance().isQuestSeeded(), "the headset seeded with no tags in view");
    assertFalse(vision.isQuestTrusted(), "an unseeded headset was trusted");
    assertEquals("NotSeeded", vision.getQuestTrustReason());
    assertEquals(
        0,
        vision.getQuestObservationsFused(),
        "an unseeded headset's frames reached the estimator");
  }

  /**
   * The first multi-tag fix seeds the headset, and only then is it trusted.
   *
   * <p>Both halves: the seed happens, and trust follows it rather than preceding it.
   */
  @Test
  void theFirstMultiTagFixSeedsTheHeadsetAndTrustFollows() {
    seedAgainstTags();

    assertEquals(1, vision.getQuestSeedCount(), "the headset should have been seeded exactly once");
    assertTrue(
        vision.isQuestTrusted(), "a seeded, tracking, charged headset was still not trusted");
    assertEquals("Trusted", vision.getQuestTrustReason());
    assertTrue(vision.getQuestObservationsFused() > 0, "no headset frame reached the estimator");
  }

  /**
   * The first seed waits for a multi-tag fix rather than taking whatever is available.
   *
   * <p>Seeding is not a neutral act. It writes the current estimate permanently into the headset's
   * frame, so a seed taken from a poor fix is a bias the headset then reports as confident truth
   * until someone re-seeds it. There is no cost to waiting: an unseeded headset is simply not used.
   */
  @Test
  void theFirstSeedIsRefusedOnASingleTagFix() {
    intakeSim.setSolveQuality(1, 2.0, 0.0);
    stepStationary(15);

    assertFalse(
        RobotState.getInstance().isQuestSeeded(), "the headset was seeded from a single-tag fix");

    // And the same headset seeds immediately once a real fix appears, so the refusal above is a
    // threshold rather than a headset that can never seed at all.
    intakeSim.setSolveQuality(3, 2.0, 0.0);
    stepStationary(5);
    assertTrue(
        RobotState.getInstance().isQuestSeeded(), "a multi-tag fix did not seed the headset");
  }

  /**
   * Seeding commands the headset's pose, not the robot's.
   *
   * <p>{@code setPose} moves the <i>headset</i> origin, so the caller must compose the mounting
   * first. Omitting it, or composing it backwards, leaves every subsequent QuestNav measurement
   * displaced by the mounting offset -- a constant bias arriving at the tightest covariance in the
   * system, which the estimator will faithfully follow.
   *
   * <p>Asserted through the fused estimate with the cameras switched off afterwards, so the only
   * thing holding the pose is the headset. A mounting error would show up here as roughly {@code 2
   * * |ROBOT_TO_QUEST|}, about 45 cm.
   */
  @Test
  void seedingCommandsTheHeadsetPoseAndNotTheRobotPose() {
    seedAgainstTags();

    intakeSim.setBlackout(true);
    stepStationary(25);

    assertTrue(vision.isQuestTrusted(), "trust was lost before the measurement could be taken");
    assertTrue(
        estimateErrorMeters() < 0.02,
        "with only the headset contributing, the estimate sat "
            + estimateErrorMeters()
            + " m from truth. That is the signature of the mounting transform being omitted or"
            + " composed backwards in seedQuestNav or questPoseToRobotPose.");
  }

  // --------------------------------------------------------------- the checkpoint, in order ----

  /**
   * Odometry alone really does drift over this run.
   *
   * <p>The anti-vacuity guard for the checkpoint below, and the reason the whole suite runs on
   * under-reporting wheels. If odometry were perfect, "the pose tracks with both cameras blind"
   * would hold whether or not a single QuestNav frame was ever fused, and the checkpoint would be
   * measuring nothing at all.
   */
  @Test
  void odometryAloneReallyDoesDriftOverThisRun() {
    // Nothing correcting: no tags, no headset.
    intakeSim.setBlackout(true);
    questSim.setConnected(false);

    int loops = (int) Math.round(PUSH_DISTANCE_METERS / PUSH_SPEED_METERS_PER_SEC / LOOP_SEC);
    for (int i = 0; i < loops; i++) {
      step(PUSH_SPEED_METERS_PER_SEC);
    }

    double expectedShortfall = PUSH_DISTANCE_METERS * (1.0 - ODOMETRY_SCALE_ERROR);
    assertTrue(
        estimateErrorMeters() > expectedShortfall * 0.8,
        "odometry tracked a 3 m push to within "
            + estimateErrorMeters()
            + " m despite a 10 % wheel error, so the checkpoint below cannot distinguish a"
            + " working headset from an absent one");
  }

  /**
   * <b>The M9 checkpoint, first half.</b> Seed against a tag, blind both cameras, push 3 m, and the
   * pose still tracks.
   *
   * <p>This is what QuestNav is for: MegaTag2 establishes truth, QuestNav carries it between tags.
   * With the wheels under-reporting by 10 %, the headset is the only thing that can hold the
   * estimate on the robot for the length of the push.
   */
  @Test
  void withBothCamerasBlindTheHeadsetCarriesThePose() {
    seedAgainstTags();

    intakeSim.setBlackout(true);
    int fusedAtStart = vision.getQuestObservationsFused();

    int loops = (int) Math.round(PUSH_DISTANCE_METERS / PUSH_SPEED_METERS_PER_SEC / LOOP_SEC);
    double worstErrorMeters = 0.0;
    for (int i = 0; i < loops; i++) {
      step(PUSH_SPEED_METERS_PER_SEC);
      worstErrorMeters = Math.max(worstErrorMeters, estimateErrorMeters());
    }

    assertTrue(
        vision.getQuestObservationsFused() > fusedAtStart,
        "no headset frame was fused during the push, so the pose tracked on odometry alone");
    assertTrue(vision.isQuestTrusted(), "trust was withdrawn during an otherwise healthy push");
    assertTrue(
        worstErrorMeters < 0.05,
        "the estimate wandered "
            + worstErrorMeters
            + " m from truth during a 3 m push with both cameras blind; the headset is not"
            + " carrying the pose between tag sightings");
  }

  /**
   * <b>The M9 checkpoint, second half.</b> Lose the headset mid-run: trust goes false, the reason
   * is recorded, and the pose degrades rather than jumping.
   *
   * <p>"Degrades instead of jumping" is the whole point of binary trust. A source that is
   * down-weighted on the way out still moves the estimate while it is wrong; a source that is
   * dropped stops moving it at all, and the estimate simply coasts on what remains. The assertion
   * that no single loop moves the estimate more than the robot could have moved is what separates
   * those two behaviours.
   */
  @Test
  void losingTheHeadsetMidRunDegradesThePoseWithoutJumpingIt() {
    seedAgainstTags();
    intakeSim.setBlackout(true);

    for (int i = 0; i < 40; i++) {
      step(PUSH_SPEED_METERS_PER_SEC);
    }
    assertTrue(vision.isQuestTrusted(), "the headset was not healthy before it was interrupted");
    int fusedBeforeLoss = vision.getQuestObservationsFused();

    // The power cycle. Tracking drops; the headset stops publishing.
    questSim.setTracking(false);

    Pose2d previousEstimate = RobotState.getInstance().getEstimatedPose();
    double worstJumpMeters = 0.0;
    for (int i = 0; i < 40; i++) {
      step(PUSH_SPEED_METERS_PER_SEC);
      Pose2d nowEstimate = RobotState.getInstance().getEstimatedPose();
      worstJumpMeters =
          Math.max(
              worstJumpMeters,
              nowEstimate.getTranslation().getDistance(previousEstimate.getTranslation()));
      previousEstimate = nowEstimate;
    }

    assertFalse(vision.isQuestTrusted(), "a headset that stopped tracking was still trusted");
    assertEquals(
        "NotTracking",
        vision.getQuestTrustReason(),
        "trust was withdrawn for the wrong reason, which is what the log will say at an event");
    assertEquals(
        fusedBeforeLoss,
        vision.getQuestObservationsFused(),
        "a frame from an untracked headset was fused; an untrusted source must contribute nothing,"
            + " not merely less");

    // Degradation, not a jump. The robot genuinely moves 2 cm per loop, so anything beyond a small
    // multiple of that is the estimate being yanked rather than coasting.
    double perLoopMotionMeters = PUSH_SPEED_METERS_PER_SEC * LOOP_SEC;
    assertTrue(
        worstJumpMeters < perLoopMotionMeters * 2.0,
        "the estimate jumped "
            + worstJumpMeters
            + " m in a single loop when the headset dropped out, against "
            + perLoopMotionMeters
            + " m of real motion; the pose should coast on odometry, not lurch");

    // And it does then drift, which is the correct, honest outcome of losing your only correction.
    assertTrue(
        estimateErrorMeters() > 0.0,
        "the estimate is perfect after losing every absolute source, which cannot be right");
  }

  // ----------------------------------------------------------------- the rest of the ladder ----

  /** A flat battery withdraws trust, and says so. */
  @Test
  void aLowBatteryWithdrawsTrust() {
    seedAgainstTags();

    questSim.setBatteryPercent((int) Constants.Vision.kQuestMinBatteryPercent.get() - 1);
    stepStationary(3);

    assertFalse(vision.isQuestTrusted(), "a headset below the battery floor was trusted");
    assertTrue(
        vision.getQuestTrustReason().startsWith("BatteryLow"),
        "expected a BatteryLow reason, got " + vision.getQuestTrustReason());
  }

  /**
   * A headset that will not report its battery is not treated as healthy.
   *
   * <p>The IO reports -1 for a health value the headset declined to give. An unknown battery must
   * fail the check, not pass it -- the comparison is against a positive threshold precisely so that
   * the sentinel falls on the safe side rather than needing a special case someone could forget.
   */
  @Test
  void anUnknownBatteryIsNotTreatedAsHealthy() {
    seedAgainstTags();

    questSim.setBatteryPercent(-1);
    stepStationary(3);

    assertFalse(vision.isQuestTrusted(), "a headset with an unknown battery level was trusted");
  }

  /** A disconnected headset withdraws trust, and is distinguished from one that is merely lost. */
  @Test
  void aDisconnectWithdrawsTrustAndIsNamedSeparatelyFromLostTracking() {
    seedAgainstTags();

    questSim.setConnected(false);
    stepStationary(3);

    assertFalse(vision.isQuestTrusted());
    assertEquals(
        "Disconnected",
        vision.getQuestTrustReason(),
        "a disconnected headset was reported as a tracking failure; those have different fixes --"
            + " a cable versus a re-seed -- and the log is what tells them apart");
  }

  /**
   * A tracking loss that happens entirely between two loops is still caught.
   *
   * <p>The subtlest clause in the gate, and the reason the headset publishes a counter at all. A
   * headset jarred by a defender can lose and reacquire tracking inside 20 ms, so {@code
   * isTracking()} reads true both times it is sampled and the flag shows nothing ever happened.
   * What it reports afterwards may be in a shifted frame, so the correct response is not to resume
   * but to re-seed -- and only the counter can force that.
   */
  @Test
  void aTrackingLossBetweenTwoLoopsIsCaughtByTheCounter() {
    seedAgainstTags();
    assertTrue(vision.isQuestTrusted());

    // Lost and regained without a single periodic() in between, exactly as a bump would do it.
    questSim.setTracking(false);
    questSim.setTracking(true);

    step(0.0);

    assertTrue(
        vision.isQuestTrusted() == false,
        "a tracking loss that happened between two loops was missed; the gate is reading the"
            + " tracking flag rather than the tracking-lost counter");
    assertEquals("TrackingLostThisLoop", vision.getQuestTrustReason());
  }

  // ------------------------------------------------------------------------------ re-seeding ----

  /**
   * Sustained drift with the robot parked and tags in view triggers a re-seed.
   *
   * <p>The recovery path. Drift is the failure QuestNav is expected to have -- SLAM has no absolute
   * reference of its own -- so the system must be able to correct it without a reboot.
   */
  @Test
  void sustainedDriftWithTagsInViewTriggersAReseed() {
    seedAgainstTags();
    int seedsAfterFirst = vision.getQuestSeedCount();

    // Well past the residual threshold, and fast enough to cross it promptly.
    questSim.setDrift(1.0);

    // Long enough for the drift to exceed the threshold and then be held past kQuestReseedHoldSec.
    int loops =
        (int)
                Math.ceil(
                    (Constants.Vision.kQuestReseedResidualMeters.get() / 1.0
                            + Constants.Vision.kQuestReseedHoldSec.get())
                        / LOOP_SEC)
            + 20;
    for (int i = 0; i < loops; i++) {
      step(0.0);
    }

    assertTrue(
        vision.getQuestSeedCount() > seedsAfterFirst,
        "a headset drifting at 1 m/s for over a second, parked, with tags in view, was never"
            + " re-seeded");
  }

  /**
   * A re-seed is refused while the robot is moving.
   *
   * <p>The single most dangerous operation in this subsystem is a re-seed taken at a bad moment: it
   * writes the current estimate into the headset permanently, so seeding from a moving robot
   * records that moment's error as the headset's new truth. Drift is bounded and announces itself;
   * a bad seed does neither.
   */
  @Test
  void aReseedIsRefusedWhileTheRobotIsMoving() {
    seedAgainstTags();
    int seedsAfterFirst = vision.getQuestSeedCount();

    questSim.setDrift(1.0);
    for (int i = 0; i < 120; i++) {
      step(PUSH_SPEED_METERS_PER_SEC);
    }

    assertEquals(
        seedsAfterFirst,
        vision.getQuestSeedCount(),
        "the headset was re-seeded while the robot was driving, which writes that moment's"
            + " estimate error permanently into its frame");
  }

  /**
   * A re-seed is refused on a single-tag fix, even parked and genuinely drifting.
   *
   * <p>The other half of the re-seed guard. Without at least two tags the estimate being written
   * into the headset is itself poorly known, so a re-seed would replace a measured drift with an
   * unmeasured one.
   *
   * <p>The tag is close -- half a metre -- and that is not incidental. Covariance scales as {@code
   * distance^2 / tagCount}, so a single tag at 2 m is worth about 0.16 m against QuestNav's 0.02 m
   * and the headset simply outvotes it: the estimate follows the drift and the residual never
   * crosses the threshold, meaning the hold never fires and the tag check is never reached. The
   * first version of this case did exactly that, blacked the camera out entirely, and passed
   * without ever exercising the rule it was named for -- mutation testing caught it. At 0.5 m the
   * camera is tighter than the headset, anchors the estimate near truth, and the drift becomes
   * visible as a real residual. The assertion below that the residual did cross is what keeps this
   * honest.
   */
  @Test
  void aReseedIsRefusedOnASingleTagFixEvenWhenParkedAndDrifting() {
    seedAgainstTags();
    int seedsAfterFirst = vision.getQuestSeedCount();

    intakeSim.setSolveQuality(1, 0.5, 0.0);
    questSim.setDrift(0.5);
    for (int i = 0; i < 120; i++) {
      step(0.0);
    }

    assertTrue(
        vision.getQuestResidualMeters() > Constants.Vision.kQuestReseedResidualMeters.get(),
        "the headset never visibly drifted, so the stationary and hold conditions were never met"
            + " and this case never reached the tag check; residual was "
            + vision.getQuestResidualMeters()
            + " m");
    assertEquals(
        seedsAfterFirst,
        vision.getQuestSeedCount(),
        "the headset was re-seeded from a single-tag fix, writing that fix's error into its frame");
  }

  /**
   * <b>A known limit:</b> with no tags in view, headset drift is not detectable at all.
   *
   * <p>This case exists to record a real gap rather than to defend a behaviour, and it was found by
   * an earlier version of the suite asserting the opposite and failing.
   *
   * <p>The trust gate's last clause compares the headset against {@code RobotState}'s estimate. But
   * a trusted headset is <i>part of</i> that estimate, and at {@code kQuestXyStdDev} it is the most
   * heavily weighted part. So when the headset drifts with no camera to anchor the estimate, the
   * estimate follows it, the residual settles at a small equilibrium value, and the drift is
   * invisible to the very rule meant to catch it. Comparing against odometry instead would not fix
   * it either -- odometry is a relative source with its own drift, which is the whole reason
   * QuestNav is here.
   *
   * <p>It is not detectable, and it is bounded. {@code getVisionStalenessSec} counts only the two
   * Limelights, so the moment tags are lost the staleness clock runs regardless of how healthy
   * QuestNav looks, and the {@code readyToScore} gate closes on it. The robot drives on a possibly
   * drifting pose; it does not shoot on one. Should that ever need tightening, the honest fix is an
   * absolute reference, not a cleverer residual.
   */
  @Test
  void driftIsNotDetectableWithoutAnAbsoluteReference() {
    seedAgainstTags();

    intakeSim.setBlackout(true);
    questSim.setDrift(1.0);
    for (int i = 0; i < 120; i++) {
      step(0.0);
    }

    // The headset is comfortably over a metre out by now, and still trusted.
    assertTrue(
        vision.isQuestTrusted(),
        "this case documents that drift is invisible without tags; if it now fails, the trust gate"
            + " has gained a genuinely independent check and this test should be replaced by one"
            + " asserting the new behaviour");
    assertTrue(
        vision.getQuestResidualMeters() < Constants.Vision.kQuestReseedResidualMeters.get(),
        "the residual grew past the threshold with no camera anchoring the estimate, which is not"
            + " what the equilibrium predicts; got "
            + vision.getQuestResidualMeters()
            + " m");

    // And the containment: the shot gate is blind to QuestNav, so it closes anyway.
    assertTrue(
        RobotState.getInstance().getVisionStalenessSec()
            > Constants.Shot.kMaxVisionStalenessSec.get(),
        "vision staleness did not rise while both cameras were blind, so a drifting headset would"
            + " hold the shot gate open");
  }

  /**
   * A residual spike shorter than the hold does not withdraw trust or trigger a re-seed.
   *
   * <p>The residual jumps every time a camera corrects the estimate, which is the estimate moving
   * rather than the headset slipping. Without the hold, every tag sighting after a gap would look
   * like headset drift and the system would re-seed constantly -- each seed writing the estimate
   * back into the headset, making the pair agree for reasons that have nothing to do with truth.
   *
   * <p>The spike is produced by drifting out and then back, rather than by drifting and stopping.
   * Drift that stops does not recover: the offset it built up stays, the camera holds the estimate
   * on truth, and the residual remains over threshold -- which is <i>sustained</i> drift and ought
   * to re-seed. Only a residual that actually returns is the transient this rule is about.
   */
  @Test
  void aResidualSpikeShorterThanTheHoldIsNotTreatedAsDrift() {
    seedAgainstTags();
    int seedsAfterFirst = vision.getQuestSeedCount();

    double spikeRate = 5.0;
    int loopsEachWay = 5;

    questSim.setDrift(spikeRate);
    for (int i = 0; i < loopsEachWay; i++) {
      step(0.0);
    }
    assertTrue(
        vision.getQuestResidualMeters() > Constants.Vision.kQuestReseedResidualMeters.get(),
        "the injected spike never crossed the residual threshold, so this case proves nothing;"
            + " got "
            + vision.getQuestResidualMeters()
            + " m");

    questSim.setDrift(-spikeRate);
    for (int i = 0; i < loopsEachWay; i++) {
      step(0.0);
    }
    questSim.setDrift(0.0);

    // Long enough that a timer which was NEVER reset would by now have passed the hold and fired a
    // re-seed. Settling for less proves only that the hold had not yet elapsed, which is a weaker
    // claim than the one this case is making -- and mutation testing showed the shorter window let
    // a never-reset timer survive.
    stepStationary(60);

    // The whole excursion lasted 0.2 s against a 0.5 s hold.
    assertTrue(
        2.0 * loopsEachWay * LOOP_SEC < Constants.Vision.kQuestReseedHoldSec.get(),
        "the spike is no longer shorter than the hold window, so this case no longer tests one");
    assertTrue(
        60 * LOOP_SEC > Constants.Vision.kQuestReseedHoldSec.get() * 2.0,
        "the settle window is no longer long enough to distinguish a reset timer from one that is"
            + " merely still counting");
    assertEquals(
        seedsAfterFirst,
        vision.getQuestSeedCount(),
        "a residual spike shorter than kQuestReseedHoldSec triggered a re-seed; the hold timer is"
            + " not being reset when the residual recovers");
    assertTrue(
        vision.isQuestTrusted(),
        "a momentary residual spike withdrew trust, which would drop the headset every time a"
            + " camera corrected the estimate");
  }

  /**
   * A headset that is not tracking publishes nothing at all.
   *
   * <p>A guard on the simulation, in the same spirit as M8's {@code
   * theSimulatedCameraIsMisledByAStaleTransform}. The production gate refuses frames from an
   * untracked headset, so a sim that kept emitting stale poses would be invisible -- every case in
   * this file would still pass. But it would also quietly remove the thing those cases are meant to
   * reproduce, and the next person to add a test here would be testing against a headset that
   * cannot fail the way real ones do.
   *
   * <p>Emitting the last good pose instead is the tempting, "helpful" alternative and is exactly
   * wrong: the estimator would keep receiving a pose that no longer follows the robot, at the
   * tightest covariance in the system, and would follow it.
   */
  @Test
  void theSimulatedHeadsetStopsPublishingWhenTrackingIsLost() {
    QuestNavIOSim probe = new QuestNavIOSim(() -> truth, ROBOT_TO_QUEST);
    var inputs = new QuestNavIO.QuestNavIOInputs();

    probe.updateInputs(inputs);
    assertEquals(1, inputs.questPoses.length, "a healthy simulated headset published nothing");

    probe.setTracking(false);
    probe.updateInputs(inputs);

    assertFalse(inputs.tracking, "the simulated headset still reports itself as tracking");
    assertEquals(
        0,
        inputs.questPoses.length,
        "an untracked simulated headset is still publishing poses, so no test in this file can"
            + " reproduce what a real one does when it loses tracking");
    assertTrue(
        inputs.trackingLostCounter > 0,
        "the tracking-lost counter did not move, so the between-loops case cannot be simulated");
  }

  /**
   * Resetting the pose forces a re-seed, because the headset's frame is now wrong.
   *
   * <p>A deliberate pose reset -- an auto start pose, or a driver re-seed -- moves the field frame
   * out from under the headset. Continuing to fuse it would drag the estimate straight back to
   * where the reset was meant to move it away from.
   */
  @Test
  void resettingThePoseForcesAReseed() {
    seedAgainstTags();
    assertTrue(vision.isQuestTrusted());

    RobotState.getInstance()
        .resetPose(new Pose2d(5.0, 4.0, Rotation2d.kZero), Rotation2d.kZero, reportedModules());

    assertFalse(
        RobotState.getInstance().isQuestSeeded(),
        "a pose reset left the headset marked as seeded against the old frame");

    // With no tags it stays unseeded and unused rather than quietly resuming.
    intakeSim.setBlackout(true);
    int fusedBefore = vision.getQuestObservationsFused();
    stepStationary(5);
    assertFalse(vision.isQuestTrusted(), "the headset resumed without being re-seeded");
    assertEquals(
        fusedBefore,
        vision.getQuestObservationsFused(),
        "frames were fused from a headset whose frame the reset invalidated");
  }
}
