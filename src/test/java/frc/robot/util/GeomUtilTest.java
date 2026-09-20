package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

/**
 * {@link GeomUtil#rotateAboutZ} alone, checked against geometry worked out by hand.
 *
 * <p>The M8 checkpoint says of this one method that if the estimated pose swings with the turret,
 * "the transform composition order in GeomUtil.rotateAboutZ is backwards". That makes it the single
 * highest-leverage function in the vision stack, and it has a property that makes it dangerous to
 * test carelessly: <b>the correct and the backwards composition agree exactly at a turret angle of
 * zero</b>, and stay close for small angles. Any test that exercises it near zero, or that compares
 * it against another call to itself, passes either way.
 *
 * <p>So every expected value below is written out as explicit arithmetic on coordinates -- never as
 * a second composition -- and every case uses an angle large enough that the two orders are far
 * apart. A test whose expectation is computed by the code under test measures nothing.
 *
 * <p>The other methods in {@code GeomUtil} are still unimplemented stubs belonging to the shooting
 * milestones, and are deliberately not tested here; a test asserting that a stub returns zero is a
 * test that must be deleted the moment the stub is filled in.
 */
public class GeomUtilTest {
  /** Tolerance for values that should agree to the metre-scale precision the robot cares about. */
  private static final double EPS = 1e-9;

  /**
   * The rotation spins the frame and leaves the origin exactly where it was.
   *
   * <p>This is what "about its own Z axis" means physically: the turret bearing does not move when
   * the turret turns. Asserting the translation is untouched is what separates this from the
   * backwards order, which drags the axis around the robot origin instead.
   */
  @Test
  void rotatingSpinsTheFrameWithoutMovingItsOrigin() {
    Transform3d axis =
        new Transform3d(new Translation3d(0.30, -0.10, 0.65), new Rotation3d(0.0, 0.0, 0.0));

    Transform3d spun = GeomUtil.rotateAboutZ(axis, Math.PI / 2.0);

    assertEquals(0.30, spun.getX(), EPS, "the turret axis moved in X when the turret turned");
    assertEquals(-0.10, spun.getY(), EPS, "the turret axis moved in Y when the turret turned");
    assertEquals(0.65, spun.getZ(), EPS, "the turret axis moved in Z when the turret turned");
    assertEquals(Math.PI / 2.0, spun.getRotation().getZ(), EPS, "the frame did not actually spin");
  }

  /**
   * The rotation composes in the base's frame, not the robot's.
   *
   * <p>The distinguishing case, and the reason it is worth its own test: with the axis offset from
   * the robot origin, composing in the wrong order moves the axis itself. At 180 degrees a 0.30 m X
   * offset ends up at -0.30 m instead of staying at +0.30 m -- 0.60 m of error, which is roughly
   * thirty times the "couple of centimetres" the M8 checkpoint tolerates.
   */
  @Test
  void rotatingComposesInTheBaseFrameAndNotTheRobotFrame() {
    Transform3d axis =
        new Transform3d(new Translation3d(0.30, 0.0, 0.65), new Rotation3d(0.0, 0.0, 0.0));

    Transform3d spun = GeomUtil.rotateAboutZ(axis, Math.PI);

    assertEquals(
        0.30,
        spun.getX(),
        EPS,
        "the axis translation was rotated about the ROBOT origin; the composition order is"
            + " backwards, which is exactly the bug the M8 checkpoint hunts");

    // Stated as a distance so the failure message is in the units the checkpoint is written in.
    Transform3d backwards =
        new Transform3d(Translation3d.kZero, new Rotation3d(0.0, 0.0, Math.PI)).plus(axis);
    assertTrue(
        backwards.getTranslation().getDistance(spun.getTranslation()) > 0.55,
        "the two composition orders must be far apart at 180 degrees, or this test cannot tell"
            + " them apart and proves nothing");
  }

  /**
   * A camera offset from the axis lands where trigonometry says it should.
   *
   * <p>The full chain the turret camera uses -- out to the axis, spin, out to the lens -- against a
   * position computed with sine and cosine rather than with another transform composition.
   */
  @Test
  void theCameraLandsWhereTrigonometrySaysItShould() {
    Translation3d axisOffset = new Translation3d(0.15, 0.05, 0.60);
    Translation3d lensOffsetInTurretFrame = new Translation3d(0.20, 0.0, 0.10);
    double turretAngleRad = Math.toRadians(35.0);

    Transform3d robotToCamera =
        GeomUtil.rotateAboutZ(
                new Transform3d(axisOffset, new Rotation3d(0.0, 0.0, 0.0)), turretAngleRad)
            .plus(new Transform3d(lensOffsetInTurretFrame, new Rotation3d(0.0, 0.0, 0.0)));

    // Hand-computed: the lens offset is rotated by the turret angle in the plane, then added to
    // the axis. Z is unaffected because the rotation is about Z.
    double expectedX = 0.15 + 0.20 * Math.cos(turretAngleRad);
    double expectedY = 0.05 + 0.20 * Math.sin(turretAngleRad);
    double expectedZ = 0.60 + 0.10;

    assertEquals(expectedX, robotToCamera.getX(), 1e-9, "camera X");
    assertEquals(expectedY, robotToCamera.getY(), 1e-9, "camera Y");
    assertEquals(expectedZ, robotToCamera.getZ(), 1e-9, "camera Z");
    assertEquals(turretAngleRad, robotToCamera.getRotation().getZ(), 1e-9, "camera bearing");
  }

  /**
   * Composing a rotation and then its negation returns the original transform.
   *
   * <p>An invariant rather than a value, and it holds for the wrong order too -- so it is here to
   * catch a different class of defect: a composition that is not invertible at all, which is what a
   * transform built by adding Euler angles produces once the base carries a pitch.
   */
  @Test
  void rotatingBackByTheSameAngleIsTheIdentity() {
    Transform3d tilted =
        new Transform3d(
            new Translation3d(0.15, 0.05, 0.60), new Rotation3d(0.0, Math.toRadians(-20.0), 0.0));

    Transform3d there = GeomUtil.rotateAboutZ(tilted, 1.1);
    Transform3d andBack = GeomUtil.rotateAboutZ(there, -1.1);

    assertEquals(
        0.0,
        andBack.getTranslation().getDistance(tilted.getTranslation()),
        1e-9,
        "rotating out and back moved the frame");
    assertEquals(
        0.0,
        new Pose3d().transformBy(andBack).getRotation().minus(tilted.getRotation()).getAngle(),
        1e-9,
        "rotating out and back left a residual rotation");
  }

  /**
   * A pitched base spins about its own tilted axis, not about the robot's vertical.
   *
   * <p>A turret bearing that is not level is a real mounting, and the two readings diverge for it.
   * Spinning about the base's own Z keeps the base's tilt magnitude unchanged, because the rotation
   * is applied within that already-tilted frame.
   */
  @Test
  void aPitchedBaseSpinsAboutItsOwnAxis() {
    double pitchRad = Math.toRadians(-20.0);
    Transform3d tilted =
        new Transform3d(new Translation3d(0.0, 0.0, 0.60), new Rotation3d(0.0, pitchRad, 0.0));

    Transform3d spun = GeomUtil.rotateAboutZ(tilted, Math.PI / 2.0);

    // Rotating within the tilted frame cannot change how far the frame is tilted from the base --
    // only which way the tilt points. The total rotation magnitude from the tilted frame is
    // therefore exactly the applied angle.
    assertEquals(
        Math.PI / 2.0,
        new Pose3d().transformBy(spun).getRotation().minus(tilted.getRotation()).getAngle(),
        1e-9,
        "the spin was not applied in the base's own frame");
  }
}
