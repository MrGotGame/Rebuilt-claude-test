package frc.robot.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.util.GeomUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The heart of shooting on the move.
 *
 * <p>Given the robot's field pose, field-relative velocity and acceleration, and the goal location,
 * this finds where to aim a shot that will <i>arrive</i> at the goal while the robot keeps moving.
 * It does so by fixed-point iteration on time of flight:
 *
 * <pre>
 * t_0  = tofTable(|g - p|)                                  // seed with the stationary answer
 * repeat k = 1 .. kMaxSolverIterations:
 *     gv_k = g - v*t_{k-1} - 0.5*alpha*a*t_{k-1}^2          // "shoot behind the motion"
 *     d_k  = |gv_k - p|
 *     t_k  = tofTable(d_k)
 *     stop when |t_k - t_{k-1}| &lt; kSolverToleranceSec
 * </pre>
 *
 * <p>Then it converts the converged virtual goal into a launch. The ground-frame velocity the game
 * piece must have is {@code vg = unit(gv - p) * (d/t)}. The robot already supplies {@code v}, so
 * the muzzle velocity the shooter must produce <i>relative to the robot</i> is {@code vm = vg - v}.
 * The turret aims along {@code vm}, and the flywheel RPM comes from {@code |vm|} through a linear
 * fit.
 *
 * <p>This class is deliberately <b>pure</b>: no subsystem references, no {@code Timer}, no
 * NetworkTables. That is precisely what lets {@code ShootingSolverTest} assert convergence and
 * correctness with no HAL present.
 */
public class ShootingSolver {
  private final InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();
  private int lastIterationCount = 0;
  private double lastResidualSec = 0.0;

  /**
   * Whether this solver's tables come from {@code Constants.Shot}.
   *
   * <p>False for the test-only constructor. {@link #reloadTables()} checks this so that a solver
   * built from explicit arrays is never silently overwritten with the (all-zero) Constants tables
   * when the superstructure calls reload on entering a scoring state.
   */
  private final boolean usesConstantsTables;

  /**
   * The linear RPM fit, {@code rpm = rpmPerMeterPerSec * |muzzle velocity| + rpmIntercept}.
   *
   * <p>Suppliers read on every solve, defaulting to {@code Constants.Shot}'s tunables so a retune
   * takes effect live. The explicit form exists for M10b step 7's superstructure tests: both
   * constants are still zero awaiting M11, so every solved shot asks for 0 RPM, the flywheel term
   * of the readiness gate is always false, and no test could ever watch the gate open -- which is
   * exactly what the burst latch and the empty belief need to be tested at all.
   */
  private final DoubleSupplier rpmPerMeterPerSec;

  private final DoubleSupplier rpmIntercept;

  /** Builds a solver whose tables come from {@code Constants.Shot}. */
  public ShootingSolver() {
    // IMPLEMENTATION (was TODO: "Populate both InterpolatingDoubleTreeMaps by zipping
    // Constants.Shot.kDistanceBreakpointsMeters with kTimeOfFlightSec and
    // kLaunchSpeedMetersPerSec. Assert the three arrays are the same length and throw if not - a
    // mismatched table is a silent aiming bug otherwise.")
    //
    // One table now, not two. M11 step 1 deleted the launch-speed table: solve() never read it --
    // the ground speed a shot needs is distance / time of flight, and the flywheel speed comes from
    // the RPM fit -- so it was a measurement that changed nothing.
    usesConstantsTables = true;
    rpmPerMeterPerSec = Constants.Shot.kRpmPerMeterPerSec;
    rpmIntercept = Constants.Shot.kRpmIntercept;
    populate(Constants.Shot.kDistanceBreakpointsMeters, Constants.Shot.kTimeOfFlightSec);
  }

  /**
   * Test-only constructor taking explicit tables, so unit tests need neither {@code Constants} nor
   * NetworkTables.
   *
   * @param distances ascending distance breakpoints, in meters
   * @param timesOfFlight time of flight at each breakpoint, in seconds
   */
  public ShootingSolver(double[] distances, double[] timesOfFlight) {
    // IMPLEMENTATION (was TODO: "Test-only constructor that takes explicit tables so unit tests do
    // not depend on Constants or NetworkTables.")
    this(distances, timesOfFlight, Constants.Shot.kRpmPerMeterPerSec, Constants.Shot.kRpmIntercept);
  }

  /**
   * Test-only constructor taking explicit tables AND an explicit RPM fit.
   *
   * <p>Added for M10b step 7: with the RPM fit still zero in {@code Constants.Shot}, this is the
   * only way a superstructure test can produce a shot the flywheel will actually spin up for, and
   * so the only way to watch the readiness gate open.
   *
   * @param distances ascending distance breakpoints, in meters
   * @param timesOfFlight time of flight at each breakpoint, in seconds
   * @param rpmPerMeterPerSec flywheel RPM per meter per second of muzzle speed
   * @param rpmIntercept flywheel RPM at zero muzzle speed
   */
  public ShootingSolver(
      double[] distances,
      double[] timesOfFlight,
      DoubleSupplier rpmPerMeterPerSec,
      DoubleSupplier rpmIntercept) {
    usesConstantsTables = false;
    this.rpmPerMeterPerSec = rpmPerMeterPerSec;
    this.rpmIntercept = rpmIntercept;
    populate(distances, timesOfFlight);
  }

  /**
   * Rebuilds the time-of-flight table from the current constants.
   *
   * <p><b>Outputs:</b> clears and repopulates the interpolation table. Called from the
   * superstructure's onEnter for scoring states, so a mid-match table edit takes effect without a
   * redeploy.
   */
  public void reloadTables() {
    // IMPLEMENTATION (was TODO: "Clear and repopulate both maps from Constants.Shot. Called from
    // Superstructure's onEnter for scoring states so a mid-match table edit takes effect without a
    // redeploy.")
    //
    // A solver built from explicit test arrays is left alone. Without this guard, a test that
    // happened to call reloadTables would have its carefully-chosen table silently replaced by the
    // all-zero Constants one, and would then fail for a reason that has nothing to do with the
    // behaviour under test.
    if (!usesConstantsTables) {
      return;
    }
    populate(Constants.Shot.kDistanceBreakpointsMeters, Constants.Shot.kTimeOfFlightSec);
  }

  /**
   * Solves for a shot that arrives at the goal while the robot keeps moving.
   *
   * @param robotPose current field pose, blue-origin, in meters and radians
   * @param fieldRelativeSpeeds current field-relative chassis velocity, in meters per second
   * @param fieldRelativeAccel current field-relative chassis acceleration, in meters per second
   *     squared; weighted by {@code kAccelCompensationScalar}
   * @param goal the goal position in field coordinates, already alliance-flipped
   * @return the solved shot, or {@link ShotParameters#invalid()} if no usable solution exists
   */
  public ShotParameters solve(
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      Translation2d fieldRelativeAccel,
      Translation2d goal) {
    // IMPLEMENTATION (was TODO: "Implement the fixed-point iteration described above ... After
    // convergence compute los/d/tof/vg/vm/aim/rpm/omega and return a ShotParameters, with inRange =
    // d between kMinDistanceMeters and kMaxDistanceMeters.")

    Translation2d robot = robotPose.getTranslation();
    Translation2d velocity = GeomUtil.toTranslation(fieldRelativeSpeeds);
    double accelScalar = Constants.Shot.kAccelCompensationScalar.get();

    // Seed with the stationary answer: how long the shot would take if the robot were not moving.
    double tof = timeOfFlightTable.get(goal.getDistance(robot));
    double residual = Double.POSITIVE_INFINITY;
    int iterations = 0;
    Translation2d virtualGoal = goal;

    // Fixed-point iteration. Each pass displaces the aim point backwards along the robot's own
    // motion by however far the robot will travel during the shot's flight, then re-reads the
    // flight time for the new (shorter or longer) range. It converges quickly because time of
    // flight is a slowly-varying function of distance.
    for (int i = 0; i < Constants.Shot.kMaxSolverIterations; i++) {
      iterations = i + 1;

      // "Shoot behind the motion": aim where the goal effectively is, from the perspective of a
      // piece that will be in the air for tof seconds while the robot keeps moving.
      virtualGoal =
          goal.minus(velocity.times(tof))
              .minus(fieldRelativeAccel.times(0.5 * accelScalar * tof * tof));

      double newTof = timeOfFlightTable.get(virtualGoal.getDistance(robot));
      residual = Math.abs(newTof - tof);
      tof = newTof;

      if (residual < Constants.Shot.kSolverToleranceSec.get()) {
        break;
      }
    }
    lastIterationCount = iterations;
    lastResidualSec = residual;

    boolean converged = residual < Constants.Shot.kSolverToleranceSec.get();

    Translation2d lineOfSight = virtualGoal.minus(robot);
    double distance = lineOfSight.getNorm();

    // Guards before any division. tof is zero whenever the time-of-flight table is still all zeros,
    // which is its shipped state until someone measures it; distance is zero if the robot is
    // exactly on the goal. Either would produce an infinite launch speed that the robot would
    // cheerfully try to fire.
    if (tof <= 0.0 || distance <= 0.0) {
      return ShotParameters.invalid();
    }

    // Ground-frame velocity the game piece must leave with to cover `distance` in `tof`.
    Translation2d requiredGroundVelocity =
        new Translation2d(distance / tof, lineOfSight.getAngle());

    // The robot already supplies its own velocity to the piece, so the shooter only has to make up
    // the difference. THIS subtraction is what shooting on the move actually is -- everything above
    // is just working out where to point.
    Translation2d muzzleVelocity = requiredGroundVelocity.minus(velocity);

    double rpm =
        rpmPerMeterPerSec.getAsDouble() * muzzleVelocity.getNorm() + rpmIntercept.getAsDouble();

    // Angular rate the turret must match to hold the aim point while the robot translates. The
    // robot's velocity is negated because the TARGET's apparent motion is the opposite of the
    // robot's own.
    double turretOmega =
        GeomUtil.tangentialComponent(velocity.unaryMinus(), lineOfSight.getAngle()) / distance;

    boolean inRange =
        distance >= Constants.Shot.kMinDistanceMeters.get()
            && distance <= Constants.Shot.kMaxDistanceMeters.get();

    return new ShotParameters(
        muzzleVelocity.getAngle(),
        turretOmega,
        rpm,
        distance,
        tof,
        virtualGoal,
        converged,
        inRange);
  }

  /**
   * Solves the stationary shot, used by the stationary scoring state and as the test baseline.
   *
   * @param robotPose current field pose, blue-origin, in meters and radians
   * @param goal the goal position in field coordinates, already alliance-flipped
   * @return the solved shot, or {@link ShotParameters#invalid()} if no usable solution exists
   */
  public ShotParameters solveStationary(Pose2d robotPose, Translation2d goal) {
    // IMPLEMENTATION (was TODO: "Convenience wrapper calling solve(...) with zeroed speeds and
    // acceleration. Used by the SCORING_STATIONARY state and as the baseline in unit tests: solve()
    // with zero velocity MUST equal solveStationary() exactly.")
    //
    // Delegating rather than duplicating the maths is what MAKES that equivalence true by
    // construction, which is why the corresponding unit test is meaningful as a regression guard on
    // the velocity-compensation terms rather than a tautology about this method.
    return solve(robotPose, new ChassisSpeeds(), Translation2d.kZero, goal);
  }

  /**
   * Looks up the stationary time of flight.
   *
   * @param distanceMeters distance to the target, in meters
   * @return interpolated time of flight, in seconds
   */
  public double getTimeOfFlight(double distanceMeters) {
    // IMPLEMENTATION (was TODO: "timeOfFlightTable.get(distanceMeters).")
    return timeOfFlightTable.get(distanceMeters);
  }

  /**
   * How many iterations the last solve took.
   *
   * @return iteration count; log this so a replay shows whether the solver ever hits the cap
   */
  public int getLastIterationCount() {
    // IMPLEMENTATION (was TODO: "Getter; log it so you can see in replay whether the solver ever
    // hits the iteration cap.")
    return lastIterationCount;
  }

  /**
   * How far the last solve was from converging.
   *
   * @return final time-of-flight residual, in seconds; a residual that stays large means the TOF
   *     table is too coarse or non-monotonic
   */
  public double getLastResidualSec() {
    // IMPLEMENTATION (was TODO: "Getter; a residual that stays large means your TOF table is too
    // coarse or non-monotonic.")
    return lastResidualSec;
  }

  /**
   * Publishes solver internals for AdvantageScope.
   *
   * @param prefix log key prefix, for example {@code "Superstructure/Shot"}
   * @param result the solution to publish
   *     <p><b>Outputs:</b> logs the virtual goal, iteration count, residual, distance and RPM.
   */
  public void logDebug(String prefix, ShotParameters result) {
    // IMPLEMENTATION (was TODO: "Logger.recordOutput(prefix + "/VirtualGoal", new Pose2d(
    // result.virtualGoal(), result.turretFieldAngle())) plus iteration count, residual, distance
    // and RPM.")
    //
    // The virtual goal is published AS A POSE rather than as a bare translation, specifically so it
    // renders as a robot-like marker on AdvantageScope's 2D field. Seeing the aim point slide off
    // the real goal as the robot accelerates is by far the fastest way to tell whether the
    // velocity compensation has the right sign.
    Logger.recordOutput(
        prefix + "/VirtualGoal", new Pose2d(result.virtualGoal(), result.turretFieldAngle()));
    Logger.recordOutput(prefix + "/Iterations", lastIterationCount);
    Logger.recordOutput(prefix + "/ResidualSec", lastResidualSec);
    Logger.recordOutput(prefix + "/DistanceMeters", result.effectiveDistanceMeters());
    Logger.recordOutput(prefix + "/FlywheelRpm", result.flywheelRpm());
    Logger.recordOutput(prefix + "/TimeOfFlightSec", result.timeOfFlightSec());
    Logger.recordOutput(prefix + "/Usable", result.isUsable());
  }

  /**
   * Fills the time-of-flight table from two parallel arrays.
   *
   * @param distances ascending distance breakpoints, in meters
   * @param timesOfFlight time of flight at each breakpoint, in seconds
   *     <p><b>Outputs:</b> clears and repopulates {@code timeOfFlightTable}.
   * @throws IllegalArgumentException if the two arrays are not the same non-zero length
   */
  private void populate(double[] distances, double[] timesOfFlight) {
    // Throwing on a mismatch is deliberate and is worth the crash. A table whose arrays have
    // drifted out of step produces plausible-looking but wrong aim at every distance, with no
    // symptom other than missing -- which costs days to trace. Failing loudly at construction turns
    // that into a stack trace on the first boot after the edit.
    if (distances.length == 0 || distances.length != timesOfFlight.length) {
      throw new IllegalArgumentException(
          "Shot table arrays must be non-empty and the same length; got distances="
              + distances.length
              + " timesOfFlight="
              + timesOfFlight.length);
    }

    timeOfFlightTable.clear();
    for (int i = 0; i < distances.length; i++) {
      timeOfFlightTable.put(distances[i], timesOfFlight[i]);
    }
  }
}
