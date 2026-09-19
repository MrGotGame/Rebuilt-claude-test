package frc.robot.shooting;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for the turret unwrapper.
 *
 * <p>This class is unit-tested specifically because getting it wrong breaks a belt rather than
 * merely producing a bad number. It is pure, so these tests need no HAL.
 *
 * <p>Every test is {@link Disabled} until written; delete the annotation as you implement each.
 */
public class TurretUnwrapperTest {
  private static final double MIN = -Math.toRadians(200.0);
  private static final double MAX = Math.toRadians(200.0);

  /** The nearest legal branch should win when several are in range. */
  @Test
  @Disabled("TODO: implement")
  void picksNearestBranchWithinLimits() {
    // TODO: From a current position of 0, request a direction of 0.5 rad and assert the target is
    // 0.5 rather than 0.5 - 2*pi.
  }

  /** When the far branch is genuinely closer, it should be chosen. */
  @Test
  @Disabled("TODO: implement")
  void unwrapsPastPiWhenTheFarBranchIsCloser() {
    // TODO: From a current position near +3 rad, request a direction just past -pi and assert the
    // solver picks the branch above +pi instead of sweeping all the way back down.
  }

  /** A branch outside the travel limits must never be selected. */
  @Test
  @Disabled("TODO: implement")
  void refusesToCrossTheHardStop() {
    // TODO: Assert every returned targetPositionRad lies within [MIN, MAX] across a sweep of
    // requested directions. This is the assertion that protects the hardware.
  }

  /** When no branch fits, report unreachable with the closest legal angle. */
  @Test
  @Disabled("TODO: implement")
  void reportsUnreachableWhenNoBranchFits() {
    // TODO: Use a deliberately narrow limit pair and assert reachable() is false and the target is
    // the clamped value, so Turret.outOfRange() has somewhere sensible to hold.
  }

  /** Hysteresis must suppress branch chatter at the boundary. */
  @Test
  @Disabled("TODO: implement")
  void hysteresisPreventsBranchChatter() {
    // TODO: Sit at a position exactly between two equally-distant branches, then nudge the request
    // back and forth by less than the hysteresis, and assert the chosen branch does NOT change.
  }

  /** The zero-hysteresis overload must be deterministic. */
  @Test
  @Disabled("TODO: implement")
  void zeroHysteresisOverloadIsDeterministic() {
    // TODO: Assert repeated calls with identical arguments return identical results.
  }

  /** The reported error must be signed and consistent with the target. */
  @Test
  @Disabled("TODO: implement")
  void errorRadIsSignedAndConsistent() {
    // TODO: Assert errorRad equals targetPositionRad - currentPositionRad, including its sign.
  }

  /** Clamping must behave exactly like MathUtil.clamp. */
  @Test
  @Disabled("TODO: implement")
  void clampIntoRangeMatchesMathUtil() {
    // TODO: Compare clampIntoRange against MathUtil.clamp across values inside and outside range.
  }

  /** Arc overlap detection must work regardless of sweep direction. */
  @Test
  @Disabled("TODO: implement")
  void passesThroughArcDetectsOverlapInBothDirections() {
    // TODO: Assert passesThroughArc is true for a sweep that crosses the arc going up AND for the
    // same sweep going down. This is what stops the turret sweeping through a deployed intake.
  }

  /** The safety margin held back from each stop must be honoured. */
  @Test
  @Disabled("TODO: implement")
  void limitMarginIsRespected() {
    // TODO: Pass limits already reduced by kLimitMarginRad and assert no target lands inside the
    // margin.
  }
}
