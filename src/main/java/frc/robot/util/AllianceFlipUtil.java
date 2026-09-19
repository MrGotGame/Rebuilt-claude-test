package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/**
 * The only place in this codebase that is allowed to know about the red alliance.
 *
 * <p>Every coordinate elsewhere -- goal locations, auto start poses, scoring targets -- is written
 * once in blue-origin WPILib field coordinates. This class flips them for red. Keeping the flip in
 * exactly one file prevents the classic season-ending bug where the shooter aims correctly on blue
 * and mirrors wrong on red, which typically is not discovered until an elimination match.
 *
 * <p>The 2026 REBUILT field is <b>rotationally symmetric</b>, so the red transform is always a
 * 180-degree rotation about the field centre: both translation axes invert and headings gain pi.
 * That is a confirmed property of the game, not an assumption, so no mirrored-field case is
 * implemented and none should be added.
 *
 * <p>The flip does not produce meaningful output until {@code kFieldLengthMeters} and {@code
 * kFieldWidthMeters} are measured; they are both 0.0 today, so a flipped coordinate currently comes
 * back rotated about the origin rather than about the field centre.
 */
public class AllianceFlipUtil {
  private AllianceFlipUtil() {}

  /**
   * Reports whether coordinates should currently be flipped.
   *
   * <p>Call this every loop rather than caching it at construction: the alliance is not known until
   * the driver station connects, which is usually after robot code has already started.
   *
   * @return true when the driver station reports the red alliance, false when blue or unknown
   */
  public static boolean shouldFlip() {
    // IMPLEMENTATION (was TODO: "DriverStation.getAlliance().orElse(Alliance.Blue) ==
    // Alliance.Red.")
    //
    // Defaulting to Blue when the alliance is unknown is deliberate. Blue-origin coordinates are
    // the identity transform, so an unknown alliance yields unmodified coordinates rather than a
    // half-applied flip -- wrong, but wrong in the direction that is obvious on the field rather
    // than subtly mirrored.
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /**
   * Flips a field position to the current alliance.
   *
   * @param translation position in blue-origin field coordinates, in meters
   * @return the same position when on blue, or the alliance-flipped position when on red, in meters
   */
  public static Translation2d apply(Translation2d translation) {
    // IMPLEMENTATION (was TODO: "If shouldFlip(), return a rotationally symmetric flip
    // (kFieldLength - x, kFieldWidth - y).")
    //
    // REBUILT is rotationally symmetric, so the flip is a 180-degree rotation about the field
    // centre and BOTH axes invert. There is no mirrored-field branch because the game does not
    // have one -- adding a hedge here would only create a second convention to keep in step.
    if (!shouldFlip()) {
      return translation;
    }
    return new Translation2d(
        Constants.Field.kFieldLengthMeters.get() - translation.getX(),
        Constants.Field.kFieldWidthMeters.get() - translation.getY());
  }

  /**
   * Flips a field heading to the current alliance.
   *
   * @param rotation heading in blue-origin field coordinates
   * @return the same heading when on blue, or the alliance-flipped heading when on red
   */
  public static Rotation2d apply(Rotation2d rotation) {
    // IMPLEMENTATION (was TODO: "If shouldFlip(), rotate by Math.PI (rotational symmetry). Must
    // match the convention chosen in apply(Translation2d).")
    //
    // The same 180-degree rotation that apply(Translation2d) performs on position, applied to
    // heading. These two must stay in step: if they ever disagree, poses flip position correctly
    // but face the wrong way, which presents as a turret bug rather than a coordinate bug.
    if (!shouldFlip()) {
      return rotation;
    }
    return rotation.rotateBy(Rotation2d.kPi);
  }

  /**
   * Flips a full field pose to the current alliance.
   *
   * @param pose pose in blue-origin field coordinates, meters and radians
   * @return the same pose when on blue, or the alliance-flipped pose when on red
   */
  public static Pose2d apply(Pose2d pose) {
    // IMPLEMENTATION (was TODO: "Compose the two overloads above.")
    //
    // Composed from the two overloads rather than reimplemented, so there is exactly one place
    // where the symmetry convention is encoded per quantity.
    return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  /**
   * Flips only the X coordinate, for auto routines that reason about field halves.
   *
   * @param xMeters X coordinate in blue-origin field coordinates, in meters
   * @return the flipped X coordinate when on red, otherwise unchanged, in meters
   */
  public static double applyX(double xMeters) {
    // IMPLEMENTATION (was TODO: "shouldFlip() ? Constants.Field.kFieldLength - xMeters : xMeters.")
    //
    // Only X is touched. Callers that need the full flip should use apply(Translation2d); this
    // exists for auto routines that reason about field halves and genuinely want Y left alone.
    return shouldFlip() ? Constants.Field.kFieldLengthMeters.get() - xMeters : xMeters;
  }
}
