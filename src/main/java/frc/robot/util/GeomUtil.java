package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Small pure geometry helpers that the shooting solver and the vision transform math need
 * repeatedly.
 *
 * <p>Every method here is a pure function of its arguments with no state, which makes them
 * trivially unit-testable and safe to call from any thread.
 */
public class GeomUtil {
  private GeomUtil() {}

  /**
   * Reinterprets a chassis velocity as a planar vector.
   *
   * @param fieldRelativeSpeeds field-relative chassis speeds, in meters per second
   * @return the translational part as a vector, in meters per second
   */
  public static Translation2d toTranslation(ChassisSpeeds fieldRelativeSpeeds) {
    // TODO: return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).
    return Translation2d.kZero;
  }

  /**
   * Projects a velocity onto the line of sight to the target.
   *
   * <p>This is the component that changes the range during the shot's flight, so it is what the
   * flywheel speed must compensate for.
   *
   * @param velocity field-relative velocity, in meters per second
   * @param lineOfSight field-relative bearing from the robot to the target
   * @return signed speed along the line of sight, in meters per second; positive means closing
   */
  public static double radialComponent(Translation2d velocity, Rotation2d lineOfSight) {
    // TODO: return v.getX() * lineOfSight.getCos() + v.getY() * lineOfSight.getSin().
    return 0.0;
  }

  /**
   * Projects a velocity perpendicular to the line of sight to the target.
   *
   * <p>This is the component that sweeps the bearing during flight, so it drives the turret
   * velocity feedforward as {@code omega = tangential / distance}.
   *
   * @param velocity field-relative velocity, in meters per second
   * @param lineOfSight field-relative bearing from the robot to the target
   * @return signed speed across the line of sight, in meters per second
   */
  public static double tangentialComponent(Translation2d velocity, Rotation2d lineOfSight) {
    // TODO: return -v.getX() * lineOfSight.getSin() + v.getY() * lineOfSight.getCos().
    return 0.0;
  }

  /**
   * Rotates a transform about its own Z axis, used to build the live turret-camera transform.
   *
   * @param base the robot-to-turret-camera transform at zero turret angle
   * @param yawRad turret angle to apply, in radians
   * @return the transform with the turret rotation composed in
   */
  public static Transform3d rotateAboutZ(Transform3d base, double yawRad) {
    // IMPLEMENTATION (was TODO: "Compose base with a Rotation3d(0, 0, yawRad) applied at base's
    // origin. Verify the composition ORDER against the WPILib Transform3d docs -- getting pre-
    // versus post-multiplication backwards is the single most common vision bug.")
    //
    // The order is base-THEN-rotation, because the rotation is the turret's and the turret's axis
    // is where `base` ends. Transform3d.plus composes the argument in the frame the receiver
    // leaves you in, so base.plus(yaw) spins the frame that is already sitting on the turret axis.
    // The translation is untouched: the argument carries none, and plus applies the argument's
    // translation in the receiver's rotated frame, so an origin-preserving spin is exactly what a
    // zero-translation argument produces. That is the "about its own Z axis" of the method name.
    //
    // The backwards version -- rotation first, then base -- is the bug the M8 checkpoint hunts.
    // It would rotate the ROBOT frame before walking out to the turret axis, which drags the axis
    // translation itself around the robot origin. Both versions agree exactly at yawRad == 0, and
    // diverge by roughly 2 * |axis translation| * sin(yaw / 2) everywhere else, which is why a
    // bench test with the turret parked at zero certifies a transform that is wrong the moment the
    // turret moves.
    //
    // A pitched or rolled base is handled correctly by construction: "its own Z axis" then means
    // the tilted axis, which is the physical truth for a turret whose bearing is not level. That
    // is also why this cannot be written as a plain yaw addition on base's Rotation3d -- Euler
    // angles do not add when the earlier rotations are non-trivial.
    return base.plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, 0.0, yawRad)));
  }

  /**
   * Lifts a 2D pose onto the floor plane for AdvantageScope's 3D field view.
   *
   * @param pose a planar pose, in meters and radians
   * @return the same pose at Z = 0 with zero roll and pitch
   */
  public static Pose3d toPose3d(Pose2d pose) {
    // TODO: return new Pose3d(pose).
    return new Pose3d();
  }

  /**
   * Linearly interpolates a lookup table, clamping outside its range.
   *
   * <p>Preferred over {@code InterpolatingDoubleTreeMap} inside the shooting solver specifically,
   * so that the solver stays a pure function of plain arrays and can be unit-tested without WPILib
   * containers.
   *
   * @param xs breakpoints, ascending and the same length as {@code ys}
   * @param ys values at each breakpoint
   * @param x query point
   * @return the interpolated value, clamped to {@code ys[0]} below the range and to the last
   *     element above it
   */
  public static double interpolate(double[] xs, double[] ys, double x) {
    // TODO: Binary-search or linear-scan xs for the bracketing pair, then lerp. Clamp at both ends
    // rather than extrapolating -- extrapolating a shot table past its measured range produces
    // confidently wrong flywheel speeds.
    return 0.0;
  }
}
