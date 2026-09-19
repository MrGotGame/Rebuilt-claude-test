package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The only place in this codebase that is allowed to know about the red alliance.
 *
 * <p>Every coordinate elsewhere -- goal locations, auto start poses, scoring targets -- is written
 * once in blue-origin WPILib field coordinates. This class flips them for red. Keeping the flip in
 * exactly one file prevents the classic season-ending bug where the shooter aims correctly on blue
 * and mirrors wrong on red, which typically is not discovered until an elimination match.
 *
 * <p><b>Unresolved:</b> the flip convention below depends on whether the 2026 REBUILT field is
 * rotationally symmetric or mirrored. Read the official field drawings and commit to one convention
 * in {@link #apply(Translation2d)} and {@link #apply(Rotation2d)} together -- they must agree.
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
    // TODO: return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red.
    return false;
  }

  /**
   * Flips a field position to the current alliance.
   *
   * @param translation position in blue-origin field coordinates, in meters
   * @return the same position when on blue, or the alliance-flipped position when on red, in meters
   */
  public static Translation2d apply(Translation2d translation) {
    // TODO: If shouldFlip(), return the flipped translation. For a ROTATIONALLY SYMMETRIC field
    // that is (kFieldLength - x, kFieldWidth - y); for a MIRRORED field it is (kFieldLength - x,
    // y).
    // TODO: CONFIRM which one REBUILT uses from the field drawings before committing.
    return translation;
  }

  /**
   * Flips a field heading to the current alliance.
   *
   * @param rotation heading in blue-origin field coordinates
   * @return the same heading when on blue, or the alliance-flipped heading when on red
   */
  public static Rotation2d apply(Rotation2d rotation) {
    // TODO: If shouldFlip(), rotate by Math.PI for a rotationally symmetric field, or negate and
    // add
    // PI for a mirrored field. This MUST match the convention chosen in apply(Translation2d) -- if
    // the two disagree, poses will flip position correctly but face the wrong way.
    return rotation;
  }

  /**
   * Flips a full field pose to the current alliance.
   *
   * @param pose pose in blue-origin field coordinates, meters and radians
   * @return the same pose when on blue, or the alliance-flipped pose when on red
   */
  public static Pose2d apply(Pose2d pose) {
    // TODO: Compose the two overloads above: new Pose2d(apply(pose.getTranslation()),
    // apply(pose.getRotation())).
    return pose;
  }

  /**
   * Flips only the X coordinate, for auto routines that reason about field halves.
   *
   * @param xMeters X coordinate in blue-origin field coordinates, in meters
   * @return the flipped X coordinate when on red, otherwise unchanged, in meters
   */
  public static double applyX(double xMeters) {
    // TODO: return shouldFlip() ? Constants.Field.kFieldLength - xMeters : xMeters.
    return xMeters;
  }
}
