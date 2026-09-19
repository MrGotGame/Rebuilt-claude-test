package frc.robot.shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The shooting solver's output, as an immutable record so it can be passed around, logged as a
 * struct, and asserted on in tests with no risk of a consumer mutating it.
 *
 * <p><b>Contract:</b> consumers MUST check {@link #isUsable()} before reading any other field. An
 * unconverged or out-of-range solution carries meaningless numbers, not merely imprecise ones.
 *
 * @param turretFieldAngle field-relative azimuth the turret should point along
 * @param turretAngularVelocityRadPerSec how fast that azimuth is sweeping, for the turret's
 *     velocity feedforward, in radians per second
 * @param flywheelRpm flywheel speed that produces the required muzzle velocity, in RPM
 * @param effectiveDistanceMeters distance to the virtual goal, in meters, which is what the shot
 *     tables are indexed on
 * @param timeOfFlightSec converged time of flight, in seconds
 * @param virtualGoal the displaced aim point that accounts for the robot's motion, in field
 *     coordinates
 * @param converged whether the fixed-point iteration met its tolerance before the iteration cap
 * @param inRange whether the effective distance falls inside the usable shooting range
 */
public record ShotParameters(
    Rotation2d turretFieldAngle,
    double turretAngularVelocityRadPerSec,
    double flywheelRpm,
    double effectiveDistanceMeters,
    double timeOfFlightSec,
    Translation2d virtualGoal,
    boolean converged,
    boolean inRange) {

  /**
   * A well-formed "no solution" value, so that callers never receive null.
   *
   * @return a zeroed record with {@code converged} and {@code inRange} both false
   */
  public static ShotParameters invalid() {
    // Implemented rather than stubbed: returning null here would NPE through every consumer, and
    // the whole point of this value is to give callers something safe to check.
    return new ShotParameters(
        Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0, Translation2d.kZero, false, false);
  }

  /**
   * Whether this solution may be acted on.
   *
   * @return true only when the solve converged, the distance is in range, and the RPM is finite
   */
  public boolean isUsable() {
    // IMPLEMENTATION (was TODO: "converged && inRange && Double.isFinite(flywheelRpm).")
    //
    // All three terms are required. converged alone is not enough -- the iteration can converge on
    // a target outside the usable range -- and the isFinite check catches the case where a
    // time-of-flight table entry is still 0.0 and the launch-speed division produced infinity.
    return converged && inRange && Double.isFinite(flywheelRpm);
  }
}
