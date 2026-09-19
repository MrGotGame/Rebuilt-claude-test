package frc.robot.shooting;

/**
 * Chooses which turret position corresponds to a desired pointing direction.
 *
 * <p>A finite-travel turret can reach a given <i>direction</i> at several <i>positions</i> (theta,
 * theta plus or minus 2*pi, and so on), and some of those positions lie outside the hard stops.
 * This class enumerates the candidate branches, discards any outside the travel limits, prefers the
 * candidate closest to the current position, and requires a hysteresis margin before switching
 * branches so the turret does not oscillate between two nearly-equidistant options at the boundary.
 *
 * <p>When no candidate is in range it reports {@code reachable() == false} along with the closest
 * legal angle, which is what {@code Turret.outOfRange()} holds while the superstructure rotates the
 * drivetrain to help.
 *
 * <p>Pure and unit-tested, because getting this wrong breaks a belt.
 */
public class TurretUnwrapper {
  /**
   * One unwrap solution.
   *
   * @param targetPositionRad the continuous position the turret should be commanded to, in radians
   * @param reachable whether any branch of the requested direction lies inside the travel limits
   * @param requiresLongWayAround true when the chosen branch is more than pi away from current, so
   *     the caller should suppress the velocity feedforward during the sweep
   * @param errorRad signed distance from the current position to the target, in radians
   */
  public record Result(
      double targetPositionRad,
      boolean reachable,
      boolean requiresLongWayAround,
      double errorRad) {}

  private TurretUnwrapper() {}

  /**
   * Picks the best in-range turret position for a desired direction.
   *
   * @param currentPositionRad the turret's current continuous position, in radians
   * @param desiredDirectionRad the desired pointing direction in the turret's frame, in radians;
   *     need not be wrapped
   * @param minPositionRad lower travel limit, in radians
   * @param maxPositionRad upper travel limit, in radians
   * @param hysteresisRad how much closer an alternative branch must be before switching to it, in
   *     radians; suppresses chatter at the boundary
   * @return the chosen solution; never null
   */
  public static Result solve(
      double currentPositionRad,
      double desiredDirectionRad,
      double minPositionRad,
      double maxPositionRad,
      double hysteresisRad) {
    // TODO: Compute the base branch b0 = MathUtil.angleModulus(desiredDirectionRad). Enumerate
    // candidates b0 + 2*pi*k for k from floor((min - b0) / (2*pi)) to ceil((max - b0) / (2*pi)).
    // Keep only candidates inside [minPositionRad, maxPositionRad]. If none remain, return a Result
    // with reachable = false and targetPositionRad = clampIntoRange(b0, min, max).
    // Otherwise pick the candidate minimising |candidate - currentPositionRad|, but only switch
    // away from the branch nearest the current position when the improvement exceeds hysteresisRad.
    // Set requiresLongWayAround when |target - current| > Math.PI.
    //
    // The safe placeholder below is returned rather than null so the robot degrades to "cannot
    // reach" instead of throwing a NullPointerException in the turret's periodic loop.
    return new Result(
        clampIntoRange(desiredDirectionRad, minPositionRad, maxPositionRad), false, false, 0.0);
  }

  /**
   * Zero-hysteresis overload, used by tests that want deterministic branch selection.
   *
   * @param currentPositionRad the turret's current continuous position, in radians
   * @param desiredDirectionRad the desired pointing direction, in radians
   * @param minPositionRad lower travel limit, in radians
   * @param maxPositionRad upper travel limit, in radians
   * @return the chosen solution; never null
   */
  public static Result solve(
      double currentPositionRad,
      double desiredDirectionRad,
      double minPositionRad,
      double maxPositionRad) {
    // TODO: Delegate to the five-argument overload with hysteresisRad = 0.0.
    return solve(currentPositionRad, desiredDirectionRad, minPositionRad, maxPositionRad, 0.0);
  }

  /**
   * Clamps a position into the travel limits.
   *
   * @param positionRad the position to clamp, in radians
   * @param minRad lower travel limit, in radians
   * @param maxRad upper travel limit, in radians
   * @return the clamped position, in radians
   */
  public static double clampIntoRange(double positionRad, double minRad, double maxRad) {
    // TODO: return MathUtil.clamp(positionRad, minRad, maxRad). Kept as its own method so the
    // out-of-range hold behaviour is testable on its own.
    return positionRad;
  }

  /**
   * Whether a sweep between two positions would pass through a forbidden arc.
   *
   * @param fromRad starting position, in radians
   * @param toRad ending position, in radians
   * @param arcMinRad lower bound of the forbidden arc, in radians
   * @param arcMaxRad upper bound of the forbidden arc, in radians
   * @return true when the swept interval overlaps the forbidden arc
   */
  public static boolean passesThroughArc(
      double fromRad, double toRad, double arcMinRad, double arcMaxRad) {
    // TODO: Return true when the closed interval [min(from, to), max(from, to)] overlaps
    // [arcMinRad, arcMaxRad]. The turret uses this to refuse a sweep through the deployed intake.
    return false;
  }
}
