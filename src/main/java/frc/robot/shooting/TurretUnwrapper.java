package frc.robot.shooting;

import edu.wpi.first.math.MathUtil;

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
    // IMPLEMENTATION (was TODO: "Compute the base branch b0 = MathUtil.angleModulus(
    // desiredDirectionRad). Enumerate candidates b0 + 2*pi*k ... Keep only candidates within
    // [min, max]. If none remain, return reachable = false and targetPositionRad = clamp(b0).
    // Otherwise pick the candidate minimizing |candidate - current|, but only switch away from the
    // branch nearest the current position if the improvement exceeds hysteresisRad. Set
    // requiresLongWayAround when |target - current| > Math.PI.")

    // A pointing DIRECTION is an angle mod 2*pi; a turret POSITION is not, because the mechanism
    // has hard stops and can hold the same direction at several distinct positions. b0 is the
    // representative of the requested direction in [-pi, pi).
    double b0 = MathUtil.angleModulus(desiredDirectionRad);

    double best = Double.NaN;
    double bestDistance = Double.POSITIVE_INFINITY;
    double secondBest = Double.NaN;
    double secondBestDistance = Double.POSITIVE_INFINITY;

    // Enumerate every branch of the requested direction that could possibly fall inside the travel
    // limits. The +/-1 padding on the loop bounds absorbs floating-point error at the edges, where
    // a candidate exactly on a limit would otherwise be skipped.
    int kMin = (int) Math.floor((minPositionRad - b0) / (2.0 * Math.PI)) - 1;
    int kMax = (int) Math.ceil((maxPositionRad - b0) / (2.0 * Math.PI)) + 1;

    for (int k = kMin; k <= kMax; k++) {
      double candidate = b0 + 2.0 * Math.PI * k;
      if (candidate < minPositionRad || candidate > maxPositionRad) {
        continue;
      }
      double distance = Math.abs(candidate - currentPositionRad);
      if (distance < bestDistance) {
        secondBest = best;
        secondBestDistance = bestDistance;
        best = candidate;
        bestDistance = distance;
      } else if (distance < secondBestDistance) {
        secondBest = candidate;
        secondBestDistance = distance;
      }
    }

    // No branch of this direction lies inside the travel limits. Report that plainly and hand back
    // the closest legal angle, which is what Turret.outOfRange() holds while the superstructure
    // asks the drivetrain to rotate the whole robot.
    if (Double.isNaN(best)) {
      double clamped = clampIntoRange(b0, minPositionRad, maxPositionRad);
      return new Result(clamped, false, false, clamped - currentPositionRad);
    }

    // Anti-chatter tie-break. When two branches are nearly equidistant, measurement noise in the
    // desired direction can flip which one is nearer from loop to loop, and the turret reverses
    // direction every 20 ms. Within the hysteresis band we therefore ignore the (noisy) distance
    // comparison and deterministically prefer the branch nearer to zero -- turret centre.
    //
    // Preferring centre is not arbitrary: it leaves the most remaining travel in both directions,
    // so the next aim request is less likely to be unreachable. And because the tie-break does not
    // depend on the noisy input at all, it cannot oscillate.
    //
    // NOTE: this is a secondary guard. The primary defence against mid-sweep branch flipping is
    // Turret latching its chosen branch on entry to UNWRAPPING, because once a 300-degree sweep is
    // under way no amount of tie-breaking here should be allowed to reverse it.
    if (!Double.isNaN(secondBest) && Math.abs(secondBestDistance - bestDistance) < hysteresisRad) {
      if (Math.abs(secondBest) < Math.abs(best)) {
        best = secondBest;
        bestDistance = secondBestDistance;
      }
    }

    // More than half a turn away means the turret will sweep the long way around. The caller uses
    // this to suppress the chassis-cancelling velocity feedforward for the duration, since during
    // a long sweep that feedforward is meaningless and actively fights the motion profile.
    boolean longWay = bestDistance > Math.PI;
    return new Result(best, true, longWay, best - currentPositionRad);
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
    // IMPLEMENTATION (was TODO: "Overload with zero hysteresis, used by tests that want
    // deterministic branch selection.")
    //
    // Zero hysteresis disables the tie-break entirely, so selection is purely "nearest branch
    // wins". That is what lets a test assert an exact expected branch without reasoning about the
    // centre-preference rule.
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
    // IMPLEMENTATION (was TODO: "MathUtil.clamp. Separate method so the out-of-range hold behavior
    // is testable on its own.")
    return MathUtil.clamp(positionRad, minRad, maxRad);
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
    // IMPLEMENTATION (was TODO: "Return true when the closed interval [min(from,to), max(from,to)]
    // overlaps [arcMinRad, arcMaxRad]. Turret uses this to refuse a sweep through the deployed
    // intake.")
    //
    // Both endpoints are normalised with min/max so the test is direction-agnostic: sweeping from
    // 1 rad to 4 rad and from 4 rad to 1 rad cross exactly the same ground, and an interlock that
    // only caught one direction would not be an interlock.
    //
    // Closed intervals on purpose -- touching the edge of the forbidden arc counts as entering it.
    double sweepMin = Math.min(fromRad, toRad);
    double sweepMax = Math.max(fromRad, toRad);
    double arcMin = Math.min(arcMinRad, arcMaxRad);
    double arcMax = Math.max(arcMinRad, arcMaxRad);
    return sweepMin <= arcMax && arcMin <= sweepMax;
  }
}
