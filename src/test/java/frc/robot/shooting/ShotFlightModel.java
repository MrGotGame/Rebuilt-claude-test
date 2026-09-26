package frc.robot.shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Where a launched piece lands, in the model world the shooting solver assumes.
 *
 * <p>Added for M12 step 3, when following each launch to where it lands found that the solver had
 * counted the robot's velocity twice since M3. Shared by {@code ShootingSolverTest}, which checks
 * the solver's commands, and the superstructure's simulated checkpoint, which checks what the
 * turret and flywheel actually did with them.
 *
 * <p>The model: relative to the robot, a launch is a stationary shot. At muzzle speed s it covers
 * the distance d at which d / tof(d) = s, and takes tof(d) -- exactly what the time-of-flight table
 * was measured as, standing still. The piece also keeps the robot's velocity for the whole flight.
 * A solver that is right lands exactly on the goal under this model, so every centimetre of miss is
 * either a solver error or a mechanism that did not do what it was told.
 */
public final class ShotFlightModel {
  private ShotFlightModel() {}

  /**
   * Follows one launch to the ground.
   *
   * @param solver the solver whose time-of-flight table defines the flight
   * @param robot the release point, in field meters
   * @param robotVelocity the robot's field velocity at release, in meters per second
   * @param muzzle the muzzle's field direction
   * @param muzzleSpeed the muzzle speed relative to the robot, in meters per second
   * @return the landing point, in field meters
   */
  public static Translation2d landingPoint(
      ShootingSolver solver,
      Translation2d robot,
      Translation2d robotVelocity,
      Rotation2d muzzle,
      double muzzleSpeed) {
    // Bisection on d / tof(d) = s. The table rises more slowly than distance, so d / tof(d) climbs
    // with d and the root is unique.
    double lo = 0.0;
    double hi = 30.0;
    for (int i = 0; i < 200; i++) {
      double mid = 0.5 * (lo + hi);
      if (mid / solver.getTimeOfFlight(mid) < muzzleSpeed) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    double reach = 0.5 * (lo + hi);
    return robot
        .plus(new Translation2d(reach, muzzle))
        .plus(robotVelocity.times(solver.getTimeOfFlight(reach)));
  }
}
