package frc.robot.shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for the shooting solver.
 *
 * <p>The solver is constructed through its test-only constructor with explicit, round-numbered
 * tables, so these tests depend on neither the (still unmeasured) {@code Constants.Shot} arrays nor
 * NetworkTables, and need no HAL.
 *
 * <p>{@code Constants.disableHal} is set before anything touches {@code Constants}, which keeps
 * {@link frc.robot.util.LoggedTunableNumber} from trying to publish to NetworkTables during class
 * initialization. Without it, merely reading a tunable would fail in a plain JUnit run.
 */
public class ShootingSolverTest {
  /**
   * Deliberately round so expected values can be worked out by hand rather than copied from a
   * failing run.
   */
  private static final double[] DISTANCES = {2.0, 4.0, 6.0};

  private static final double[] TIMES_OF_FLIGHT = {0.4, 0.6, 0.8};
  private static final double[] LAUNCH_SPEEDS = {8.0, 11.0, 14.0};

  private static final Translation2d GOAL = new Translation2d(4.0, 0.0);
  private static final Pose2d ORIGIN = new Pose2d(0.0, 0.0, Rotation2d.kZero);

  private ShootingSolver solver;

  /** Builds a solver with a small, known, monotonic shot table. */
  @BeforeEach
  void setUp() {
    Constants.disableHal = true;
    solver = new ShootingSolver(DISTANCES, TIMES_OF_FLIGHT, LAUNCH_SPEEDS);
  }

  /** The moving solve with zero velocity must equal the stationary solve exactly. */
  @Test
  void stationarySolveMatchesMovingSolveAtZeroVelocity() {
    // The single most valuable test in the file. A sign error in the velocity compensation is
    // completely invisible while standing still, so this is the guard that catches one.
    ShotParameters moving = solver.solve(ORIGIN, new ChassisSpeeds(), Translation2d.kZero, GOAL);
    ShotParameters stationary = solver.solveStationary(ORIGIN, GOAL);

    assertEquals(
        stationary.turretFieldAngle().getRadians(), moving.turretFieldAngle().getRadians(), 1e-12);
    assertEquals(stationary.flywheelRpm(), moving.flywheelRpm(), 1e-12);
    assertEquals(stationary.effectiveDistanceMeters(), moving.effectiveDistanceMeters(), 1e-12);
    assertEquals(stationary.timeOfFlightSec(), moving.timeOfFlightSec(), 1e-12);
  }

  /** With the robot still, the turret should point straight at the goal. */
  @Test
  void stationaryAimPointsDirectlyAtGoal() {
    ShotParameters result = solver.solveStationary(ORIGIN, GOAL);

    // Goal is straight down +X, so the aim bearing is zero and the range is the full 4 m.
    assertEquals(0.0, result.turretFieldAngle().getRadians(), 1e-9);
    assertEquals(4.0, result.effectiveDistanceMeters(), 1e-9);
    assertTrue(result.isUsable());
  }

  /** Moving across the line of sight must shift the aim against the motion. */
  @Test
  void movingPerpendicularShiftsAimOppositeMotion() {
    // Goal is at +X; drive sideways at +Y. The piece inherits that +Y motion, so the turret must
    // aim to -Y to compensate. If the aim leads WITH the motion instead, the virtual-goal
    // subtraction has the wrong sign -- which is the classic shoot-on-the-move bug.
    ShotParameters result =
        solver.solve(ORIGIN, new ChassisSpeeds(0.0, 2.0, 0.0), Translation2d.kZero, GOAL);

    assertTrue(
        result.turretFieldAngle().getRadians() < 0.0,
        "aim should lead against +Y motion, got " + result.turretFieldAngle().getDegrees());

    // The turret must also be sweeping, not merely offset, since the bearing changes during flight.
    assertTrue(Math.abs(result.turretAngularVelocityRadPerSec()) > 0.0);
  }

  /** Moving along the line of sight changes range, not bearing. */
  @Test
  void movingRadiallyChangesRangeNotAngle() {
    ShotParameters closing =
        solver.solve(ORIGIN, new ChassisSpeeds(2.0, 0.0, 0.0), Translation2d.kZero, GOAL);
    ShotParameters still = solver.solveStationary(ORIGIN, GOAL);

    // Driving straight at the goal keeps the bearing at zero...
    assertEquals(0.0, closing.turretFieldAngle().getRadians(), 1e-9);
    // ...but shortens the effective range, because the robot closes during the shot's flight.
    assertTrue(
        closing.effectiveDistanceMeters() < still.effectiveDistanceMeters(),
        "closing on the goal should shorten the effective range");
    // Purely radial motion sweeps no bearing, so the turret feedforward stays zero.
    assertEquals(0.0, closing.turretAngularVelocityRadPerSec(), 1e-9);
  }

  /** The fixed-point iteration must converge inside its cap. */
  @Test
  void solverConvergesWithinIterationCap() {
    for (double vx = -3.0; vx <= 3.0; vx += 1.0) {
      for (double vy = -3.0; vy <= 3.0; vy += 1.0) {
        ShotParameters result =
            solver.solve(ORIGIN, new ChassisSpeeds(vx, vy, 0.0), Translation2d.kZero, GOAL);
        assertTrue(result.converged(), "failed to converge at v=(" + vx + "," + vy + ")");
        assertTrue(
            solver.getLastIterationCount() <= Constants.Shot.kMaxSolverIterations,
            "exceeded the iteration cap");
      }
    }
  }

  /** Solving twice from the same inputs must give the same answer. */
  @Test
  void solverIsIdempotent() {
    // Catches state accidentally carried between calls in the iteration variables.
    ChassisSpeeds speeds = new ChassisSpeeds(1.5, -1.0, 0.0);
    ShotParameters first = solver.solve(ORIGIN, speeds, Translation2d.kZero, GOAL);
    ShotParameters second = solver.solve(ORIGIN, speeds, Translation2d.kZero, GOAL);

    assertEquals(
        first.turretFieldAngle().getRadians(), second.turretFieldAngle().getRadians(), 1e-12);
    assertEquals(first.effectiveDistanceMeters(), second.effectiveDistanceMeters(), 1e-12);
    assertEquals(first.timeOfFlightSec(), second.timeOfFlightSec(), 1e-12);
  }

  /** An out-of-range shot must be reported, not silently clamped into range. */
  @Test
  void outOfRangeIsFlaggedNotClamped() {
    // Well beyond kMaxDistanceMeters. A solver that clamped instead of flagging would confidently
    // fire an impossible shot, and readyToShoot would have no way to know.
    Translation2d farGoal = new Translation2d(50.0, 0.0);
    ShotParameters result = solver.solveStationary(ORIGIN, farGoal);

    assertFalse(result.inRange());
    assertFalse(result.isUsable());
    // The distance is still REPORTED honestly rather than clipped to the table's edge.
    assertTrue(result.effectiveDistanceMeters() > Constants.Shot.kMaxDistanceMeters.get());
  }

  /** With the acceleration scalar at zero, acceleration must have no effect at all. */
  @Test
  void accelerationTermIsDisabledAtScalarZero() {
    // kAccelCompensationScalar defaults to 0, so a large acceleration must change nothing. This is
    // what makes it safe to ship the acceleration term switched off until it is tuned.
    assertEquals(0.0, Constants.Shot.kAccelCompensationScalar.get(), 1e-12);

    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 1.0, 0.0);
    ShotParameters withAccel = solver.solve(ORIGIN, speeds, new Translation2d(5.0, -5.0), GOAL);
    ShotParameters withoutAccel = solver.solve(ORIGIN, speeds, Translation2d.kZero, GOAL);

    assertEquals(
        withoutAccel.turretFieldAngle().getRadians(),
        withAccel.turretFieldAngle().getRadians(),
        1e-12);
    assertEquals(
        withoutAccel.effectiveDistanceMeters(), withAccel.effectiveDistanceMeters(), 1e-12);
  }

  /** Table lookups must clamp at both ends rather than extrapolate. */
  @Test
  void tableLookupClampsAtBothEnds() {
    // Extrapolating a shot table produces confidently wrong flywheel speeds, so the tables clamp.
    assertEquals(TIMES_OF_FLIGHT[0], solver.getTimeOfFlight(0.0), 1e-9);
    assertEquals(TIMES_OF_FLIGHT[TIMES_OF_FLIGHT.length - 1], solver.getTimeOfFlight(100.0), 1e-9);
    assertEquals(LAUNCH_SPEEDS[0], solver.getLaunchSpeed(-5.0), 1e-9);
    assertEquals(LAUNCH_SPEEDS[LAUNCH_SPEEDS.length - 1], solver.getLaunchSpeed(100.0), 1e-9);

    // And interpolate correctly in between: 3 m is halfway between the 2 m and 4 m breakpoints.
    assertEquals(0.5, solver.getTimeOfFlight(3.0), 1e-9);
  }

  /** A table built from mismatched array lengths must fail loudly at construction. */
  @Test
  void mismatchedTableLengthsThrow() {
    // A mismatched table is otherwise a completely silent aiming bug: every distance returns a
    // plausible number that is simply wrong.
    assertThrows(
        IllegalArgumentException.class,
        () -> new ShootingSolver(new double[] {1.0, 2.0}, new double[] {0.1}, new double[] {5.0}));
    assertThrows(
        IllegalArgumentException.class,
        () -> new ShootingSolver(new double[] {}, new double[] {}, new double[] {}));
  }

  /**
   * The turret feedforward must oppose the robot's motion, not follow it.
   *
   * <p>Added after mutation testing: flipping the sign of the feedforward survived the original
   * suite, because {@code movingPerpendicularShiftsAimOppositeMotion} only asserted the magnitude
   * was non-zero. A sign error here doubles the tracking error instead of cancelling it, and only
   * shows up while the robot is actually translating.
   */
  @Test
  void turretFeedforwardOpposesRobotMotion() {
    // Goal at +X, robot sliding toward +Y. The goal's BEARING therefore sweeps clockwise --
    // negative -- and the turret must sweep with it.
    ShotParameters result =
        solver.solve(ORIGIN, new ChassisSpeeds(0.0, 2.0, 0.0), Translation2d.kZero, GOAL);

    assertTrue(
        result.turretAngularVelocityRadPerSec() < 0.0,
        "feedforward should be negative for +Y motion, got "
            + result.turretAngularVelocityRadPerSec());

    // And it must reverse with the motion, rather than being a constant offset.
    ShotParameters mirrored =
        solver.solve(ORIGIN, new ChassisSpeeds(0.0, -2.0, 0.0), Translation2d.kZero, GOAL);
    assertTrue(mirrored.turretAngularVelocityRadPerSec() > 0.0);
  }

  /**
   * An unmeasured (all-zero) time-of-flight table must yield an invalid shot, not infinity.
   *
   * <p>Added after mutation testing: removing the divide-by-zero guard survived the original suite.
   * This is not a hypothetical -- {@code Constants.Shot.kTimeOfFlightSec} ships as all zeros, so
   * this guard is what stands between a freshly-flashed robot and an infinite commanded flywheel
   * speed.
   */
  @Test
  void zeroTimeOfFlightTableYieldsInvalidNotInfinity() {
    ShootingSolver unmeasured =
        new ShootingSolver(
            new double[] {2.0, 4.0, 6.0},
            new double[] {0.0, 0.0, 0.0},
            new double[] {0.0, 0.0, 0.0});

    ShotParameters result = unmeasured.solveStationary(ORIGIN, GOAL);

    assertFalse(result.isUsable());
    // Specifically FINITE, not merely unusable: an infinite RPM that happens to fail isUsable()
    // would still poison anything that logged or averaged it.
    assertTrue(Double.isFinite(result.flywheelRpm()), "RPM must not be infinite");
    assertEquals(0.0, result.flywheelRpm(), 1e-12);
  }
}
