package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Decides when a disabled robot's drivetrain drops from brake to coast, so it can be pushed off the
 * field after a match.
 *
 * <p>Added for M14 step 4. The drivetrain holds in brake while disabled, which is right at the end
 * of autonomous and during a match pause, and wrong once the match is over and the crew has to push
 * the robot off. So after {@link Constants.Drivetrain#kCoastDelaySec} disabled, and only once the
 * robot is nearly still, it releases -- once. Dropping to coast while the robot is still rolling
 * means it keeps rolling, which is why the speed check exists.
 *
 * <p>It releases the DRIVETRAIN only. The intake pivot and the turret stay in brake: a coasting
 * pivot falls under its own weight, possibly onto the turret, and a disabled turret must not flop.
 */
public class CoastRelease {
  private final Timer disabledTimer = new Timer();
  private boolean released = false;

  /**
   * Starts timing a disabled period.
   *
   * <p><b>Outputs:</b> restarts the timer and clears the once-only latch.
   */
  public void onDisabled() {
    disabledTimer.restart();
    released = false;
  }

  /**
   * Whether this is the loop to drop the drivetrain to coast. True at most once per disabled
   * period.
   *
   * @param speedMetersPerSec the chassis speed over the ground, m/s
   * @return true on the one loop the release should happen
   *     <p><b>Outputs:</b> sets the once-only latch when it returns true.
   */
  public boolean shouldReleaseNow(double speedMetersPerSec) {
    if (released
        || !disabledTimer.hasElapsed(Constants.Drivetrain.kCoastDelaySec.get())
        || speedMetersPerSec > Constants.Drivetrain.kCoastReleaseMaxSpeedMetersPerSec) {
      return false;
    }
    released = true;
    return true;
  }

  /**
   * Whether the drivetrain has been released this disabled period.
   *
   * @return true once {@link #shouldReleaseNow} has fired, until the next {@link #onDisabled}
   */
  public boolean isReleased() {
    return released;
  }
}
