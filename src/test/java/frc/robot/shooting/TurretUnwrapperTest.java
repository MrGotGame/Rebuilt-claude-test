package frc.robot.shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for the turret unwrapper.
 *
 * <p>This class is unit-tested specifically because getting it wrong breaks a belt rather than
 * merely producing a bad number. It is pure, so these tests need no HAL and no WPILib simulation.
 *
 * <p>The limits used here mirror the real mechanism: roughly +/-200 degrees of travel, which is
 * what makes several branches of the same pointing direction reachable and the unwrap problem real.
 */
public class TurretUnwrapperTest {
  private static final double MIN = -Math.toRadians(200.0);
  private static final double MAX = Math.toRadians(200.0);
  private static final double EPS = 1e-9;

  /** The nearest legal branch should win when several are in range. */
  @Test
  void picksNearestBranchWithinLimits() {
    // From centre, a modest request should be taken directly -- not via a 2*pi detour.
    TurretUnwrapper.Result result = TurretUnwrapper.solve(0.0, 0.5, MIN, MAX);

    assertTrue(result.reachable());
    assertEquals(0.5, result.targetPositionRad(), EPS);
    assertFalse(result.requiresLongWayAround());
  }

  /** When the far branch is genuinely closer, it should be chosen even though it exceeds pi. */
  @Test
  void unwrapsPastPiWhenTheFarBranchIsCloser() {
    // Turret sits at +3.0 rad. The request wraps to -3.0 rad, which is 6.0 rad away going down --
    // but the SAME direction is also reachable at -3.0 + 2*pi = 3.283 rad, only 0.283 rad away.
    // Taking the wrapped branch is the whole point of this class.
    TurretUnwrapper.Result result = TurretUnwrapper.solve(3.0, -3.0, MIN, MAX);

    assertTrue(result.reachable());
    assertEquals(-3.0 + 2.0 * Math.PI, result.targetPositionRad(), 1e-6);
    assertTrue(result.targetPositionRad() > Math.PI, "should have unwrapped past pi");
  }

  /** No commanded position may ever fall outside the travel limits. */
  @Test
  void refusesToCrossTheHardStop() {
    // Sweep the full circle of requested directions from several starting positions. This is the
    // assertion that actually protects the hardware, so it is exhaustive rather than illustrative.
    for (double current = MIN; current <= MAX; current += 0.25) {
      for (double desired = -Math.PI; desired <= Math.PI; desired += 0.1) {
        TurretUnwrapper.Result result = TurretUnwrapper.solve(current, desired, MIN, MAX);
        assertTrue(
            result.targetPositionRad() >= MIN - EPS && result.targetPositionRad() <= MAX + EPS,
            "target " + result.targetPositionRad() + " escaped limits");
      }
    }
  }

  /** When no branch fits, say so and hand back the closest legal angle. */
  @Test
  void reportsUnreachableWhenNoBranchFits() {
    // A deliberately tiny travel range. No branch of 3.0 rad (3.0, 3.0 +/- 2*pi) lands inside it.
    double narrowMin = 0.1;
    double narrowMax = 0.2;
    TurretUnwrapper.Result result = TurretUnwrapper.solve(0.15, 3.0, narrowMin, narrowMax);

    assertFalse(result.reachable());
    // The held angle must still be legal -- Turret.outOfRange() commands this while the
    // superstructure rotates the chassis to help.
    assertTrue(result.targetPositionRad() >= narrowMin && result.targetPositionRad() <= narrowMax);
  }

  /**
   * Within the hysteresis band, the branch nearer turret centre wins over the marginally nearer
   * one.
   */
  @Test
  void hysteresisPreventsBranchChatter() {
    // NOTE the deliberately WIDER limits here. With the real +/-200 degree travel, the 2*pi branch
    // used below is outside the hard stops and is correctly discarded -- which is
    // refusesToCrossTheHardStop working as intended, but it leaves no second branch to tie-break
    // against. A turret with more than a full turn of travel is needed to exercise this rule.
    double wideMin = -7.0;
    double wideMax = 7.0;

    // Turret at 3.2 rad. The request (0 rad) is reachable at 0.0 -- 3.2 away -- and at 2*pi = 6.283
    // -- 3.083 away. The wrapped branch is nearer by only 0.117 rad.
    //
    // With no hysteresis, "nearest wins" and the turret commits to 6.283, parking itself against
    // the far end of its travel over a margin small enough for sensor noise to reverse.
    TurretUnwrapper.Result noHysteresis = TurretUnwrapper.solve(3.2, 0.0, wideMin, wideMax);
    assertEquals(2.0 * Math.PI, noHysteresis.targetPositionRad(), 1e-6);

    // With a 10-degree band, the two are treated as equivalent and the centre-nearer branch is
    // preferred. That choice does not depend on the noisy input at all, which is precisely why it
    // cannot oscillate -- and it leaves more travel in both directions for the next request.
    TurretUnwrapper.Result withHysteresis =
        TurretUnwrapper.solve(3.2, 0.0, wideMin, wideMax, Math.toRadians(10.0));
    assertEquals(0.0, withHysteresis.targetPositionRad(), 1e-6);
  }

  /** The zero-hysteresis overload must be deterministic. */
  @Test
  void zeroHysteresisOverloadIsDeterministic() {
    // Same inputs must give the same answer every time; the solver holds no state between calls.
    for (int i = 0; i < 50; i++) {
      TurretUnwrapper.Result a = TurretUnwrapper.solve(1.0, 2.5, MIN, MAX);
      TurretUnwrapper.Result b = TurretUnwrapper.solve(1.0, 2.5, MIN, MAX);
      assertEquals(a.targetPositionRad(), b.targetPositionRad(), EPS);
      assertEquals(a.reachable(), b.reachable());
    }
  }

  /** The reported error must be signed and consistent with the chosen target. */
  @Test
  void errorRadIsSignedAndConsistent() {
    TurretUnwrapper.Result forward = TurretUnwrapper.solve(0.0, 1.0, MIN, MAX);
    assertEquals(forward.targetPositionRad() - 0.0, forward.errorRad(), EPS);
    assertTrue(forward.errorRad() > 0.0, "moving positive should report positive error");

    TurretUnwrapper.Result backward = TurretUnwrapper.solve(0.0, -1.0, MIN, MAX);
    assertEquals(backward.targetPositionRad() - 0.0, backward.errorRad(), EPS);
    assertTrue(backward.errorRad() < 0.0, "moving negative should report negative error");
  }

  /** Clamping must behave exactly like MathUtil.clamp. */
  @Test
  void clampIntoRangeMatchesMathUtil() {
    for (double v = -10.0; v <= 10.0; v += 0.37) {
      assertEquals(MathUtil.clamp(v, MIN, MAX), TurretUnwrapper.clampIntoRange(v, MIN, MAX), EPS);
    }
  }

  /** Arc overlap detection must be direction-agnostic. */
  @Test
  void passesThroughArcDetectsOverlapInBothDirections() {
    double arcMin = Math.toRadians(150.0);
    double arcMax = Math.toRadians(210.0);

    // A sweep that crosses the arc going up, and the identical sweep going down, cross exactly the
    // same ground. An interlock that only caught one direction would not be an interlock.
    assertTrue(
        TurretUnwrapper.passesThroughArc(
            Math.toRadians(100.0), Math.toRadians(250.0), arcMin, arcMax));
    assertTrue(
        TurretUnwrapper.passesThroughArc(
            Math.toRadians(250.0), Math.toRadians(100.0), arcMin, arcMax));

    // A sweep entirely clear of the arc must not trip it, or the intake could never deploy.
    assertFalse(TurretUnwrapper.passesThroughArc(0.0, Math.toRadians(100.0), arcMin, arcMax));
    assertFalse(
        TurretUnwrapper.passesThroughArc(
            Math.toRadians(250.0), Math.toRadians(300.0), arcMin, arcMax));
  }

  /**
   * The forbidden arc is physical space, so it must be found on every branch the turret can reach.
   *
   * <p>This turret's travel is plus or minus 200 degrees, which is wider than a full circle, so the
   * unwrapper can legitimately park it at -180 degrees to point where an operator would say +180
   * degrees. With the conflict arc configured as 150 to 210 degrees, the raw interval test says
   * that position is nowhere near the intake -- and the intake's deploy guard opens while the
   * turret is standing squarely in its path.
   *
   * <p>This was not hypothetical. M10's interlock test drove a simulated turret at 180 degrees, the
   * unwrapper chose the negative branch, and the guard reported no conflict. The failure is silent:
   * nothing in the log says anything at all until the two mechanisms meet.
   */
  @Test
  void passesThroughArcWrappedFindsTheArcOnEitherBranch() {
    double arcMin = Math.toRadians(150.0);
    double arcMax = Math.toRadians(210.0);

    // The defect, stated as a test. Both of these are the same physical direction.
    assertFalse(
        TurretUnwrapper.passesThroughArc(
            Math.toRadians(-180.0), Math.toRadians(-180.0), arcMin, arcMax),
        "precondition: the raw test cannot see the negative branch, which is why the wrapped one"
            + " exists");
    assertTrue(
        TurretUnwrapper.passesThroughArcWrapped(
            Math.toRadians(-180.0), Math.toRadians(-180.0), arcMin, arcMax),
        "a turret at -180 degrees is in an arc that spans 180 degrees, whichever branch it took");

    // A sweep that crosses the arc's negative image, in both directions.
    assertTrue(
        TurretUnwrapper.passesThroughArcWrapped(
            Math.toRadians(-100.0), Math.toRadians(-200.0), arcMin, arcMax));
    assertTrue(
        TurretUnwrapper.passesThroughArcWrapped(
            Math.toRadians(-200.0), Math.toRadians(-100.0), arcMin, arcMax));

    // The positive branch still behaves exactly as the raw test does.
    assertTrue(
        TurretUnwrapper.passesThroughArcWrapped(
            Math.toRadians(100.0), Math.toRadians(250.0), arcMin, arcMax));

    // And it must not become a guard that refuses everything, which would be the easy way to pass
    // every assertion above and would leave the intake unable to deploy at all. 0 to 100 degrees is
    // clear of the arc on every branch a 400-degree travel can reach.
    assertFalse(
        TurretUnwrapper.passesThroughArcWrapped(0.0, Math.toRadians(100.0), arcMin, arcMax));
    assertFalse(
        TurretUnwrapper.passesThroughArcWrapped(
            Math.toRadians(-100.0), Math.toRadians(-20.0), arcMin, arcMax));

    // Nonsense input fails CLOSED. A guard whose job is to refuse must not answer "no conflict"
    // because it was handed a NaN.
    assertTrue(TurretUnwrapper.passesThroughArcWrapped(Double.NaN, 0.0, arcMin, arcMax));
  }

  /** The safety margin held back from each stop must be honoured. */
  @Test
  void limitMarginIsRespected() {
    // Callers pass limits already reduced by kLimitMarginRad. Nothing may land inside that margin.
    double margin = Math.toRadians(3.0);
    double softMin = MIN + margin;
    double softMax = MAX - margin;

    for (double desired = -Math.PI; desired <= Math.PI; desired += 0.05) {
      TurretUnwrapper.Result result = TurretUnwrapper.solve(0.0, desired, softMin, softMax);
      assertTrue(
          result.targetPositionRad() >= softMin - EPS
              && result.targetPositionRad() <= softMax + EPS,
          "target entered the safety margin");
    }
  }

  /**
   * The NEAREST in-range branch must win, not merely some in-range branch.
   *
   * <p>Added after mutation testing: replacing the distance comparison with an unconditional "take
   * the last candidate enumerated" survived the original suite, because every case in it happened
   * to make those two the same branch. This case deliberately separates them.
   */
  @Test
  void picksNearestNotMerelyAnyInRangeCandidate() {
    // Wider-than-real travel so three branches of the same direction are all legal.
    double wideMin = -7.0;
    double wideMax = 7.0;

    // Turret parked near the low stop. Branches of "0 rad" sit at -2*pi, 0 and +2*pi, which are
    // 0.283, 6.0 and 12.28 rad away respectively. The nearest is the LOWEST-numbered branch, so a
    // solver that simply took the last one enumerated would pick +2*pi and sweep most of a turn.
    TurretUnwrapper.Result result = TurretUnwrapper.solve(-6.0, 0.0, wideMin, wideMax);

    assertEquals(-2.0 * Math.PI, result.targetPositionRad(), 1e-6);
  }

  /**
   * The long-way-around flag must actually be raised when the sweep exceeds pi.
   *
   * <p>Added after mutation testing: hard-coding this flag to false survived the original suite,
   * because no test asserted it was ever true. Turret suppresses its velocity feedforward on this
   * flag, so a stuck-false value silently fights the motion profile through every long sweep.
   */
  @Test
  void requiresLongWayAroundFlagsLongSweeps() {
    // Travel limited to non-negative angles, so the near branch at -2.88 rad is illegal and the
    // only legal branch is 3.4 rad -- a 3.3 rad sweep, comfortably past pi.
    TurretUnwrapper.Result longSweep = TurretUnwrapper.solve(0.1, -2.88, 0.0, 7.0);
    assertTrue(longSweep.reachable());
    assertTrue(longSweep.requiresLongWayAround(), "a 3.3 rad sweep should be flagged long");

    // A short hop must not be flagged, or the feedforward would be suppressed constantly.
    TurretUnwrapper.Result shortHop = TurretUnwrapper.solve(0.0, 0.5, MIN, MAX);
    assertFalse(shortHop.requiresLongWayAround());
  }
}
