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
    // IMPLEMENTATION (was TODO: "new Translation2d(speeds.vxMetersPerSecond,
    // speeds.vyMetersPerSecond).")
    //
    // The angular component is dropped deliberately: callers want the translational velocity as a
    // planar vector, and omega is handled separately by the turret feedforward.
    return new Translation2d(
        fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
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
    // IMPLEMENTATION (was TODO: "Project velocity onto lineOfSight: v.getX()*cos + v.getY()*sin.
    // Positive means closing.")
    //
    // A dot product of the velocity with the unit vector along the line of sight. This is the
    // component that changes RANGE during the shot's flight, so it is what the flywheel speed has
    // to compensate for.
    return velocity.getX() * lineOfSight.getCos() + velocity.getY() * lineOfSight.getSin();
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
    // IMPLEMENTATION (was TODO: "Perpendicular projection: -v.getX()*sin + v.getY()*cos. Drives the
    // turret velocity feedforward (omega = tangential / distance).")
    //
    // The cross-product magnitude against the line-of-sight unit vector, i.e. the dot product with
    // that vector rotated 90 degrees counter-clockwise. This is the component that sweeps the
    // BEARING during flight, which is why dividing it by distance yields the angular rate the
    // turret must match.
    return -velocity.getX() * lineOfSight.getSin() + velocity.getY() * lineOfSight.getCos();
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
    // origin. Used to build the turret camera transform. Verify the composition order against
    // WPILib's Transform3d docs -- getting pre- versus post-multiplication backwards is the classic
    // vision bug.")
    //
    // ORDER MATTERS. Transform3d.plus applies the argument IN THE FRAME OF the receiver, so this
    // rotates about the axis at base's own origin -- which is what a turret physically does. The
    // other order would rotate base about the ROBOT origin instead, producing a camera pose that
    // swings around the robot rather than spinning in place. That failure is invisible at a turret
    // angle of zero and grows with turret angle, which is exactly what makes it hard to find.
    return base.plus(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, 0.0, yawRad)));
  }

  /**
   * Lifts a 2D pose onto the floor plane for AdvantageScope's 3D field view.
   *
   * @param pose a planar pose, in meters and radians
   * @return the same pose at Z = 0 with zero roll and pitch
   */
  public static Pose3d toPose3d(Pose2d pose) {
    // IMPLEMENTATION (was TODO: "new Pose3d(pose) - used when publishing to AdvantageScope's 3D
    // field.")
    //
    // The Pose3d(Pose2d) constructor places the pose on the floor plane with zero roll and pitch,
    // which is what AdvantageScope's 3D field expects for a ground robot.
    return new Pose3d(pose);
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
    // IMPLEMENTATION (was TODO: "Linear interpolation with clamping at both ends, over pre-sorted
    // xs. Used to build the shot tables in a unit-testable way.")
    //
    // CLAMPS rather than extrapolates. Extrapolating a shot table past its measured range produces
    // a confidently wrong flywheel speed, and the robot fires it without hesitation. Clamping at
    // least keeps the error bounded by the edge of what was actually measured.
    if (xs.length == 0 || xs.length != ys.length) {
      throw new IllegalArgumentException(
          "interpolate() needs non-empty arrays of equal length; got xs="
              + xs.length
              + " ys="
              + ys.length);
    }
    if (x <= xs[0]) {
      return ys[0];
    }
    if (x >= xs[xs.length - 1]) {
      return ys[ys.length - 1];
    }

    // Linear scan. These tables are 8-10 entries, so a binary search would add branching complexity
    // for no measurable gain on a 20 ms loop.
    for (int i = 0; i < xs.length - 1; i++) {
      if (x <= xs[i + 1]) {
        double span = xs[i + 1] - xs[i];
        // Guard against duplicate breakpoints, which would divide by zero.
        if (span == 0.0) {
          return ys[i];
        }
        double t = (x - xs[i]) / span;
        return ys[i] + t * (ys[i + 1] - ys[i]);
      }
    }
    return ys[ys.length - 1];
  }
}
