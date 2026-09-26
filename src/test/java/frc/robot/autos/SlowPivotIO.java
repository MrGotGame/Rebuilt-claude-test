package frc.robot.autos;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.robot.subsystems.intake.IdealPivotIO;
import frc.robot.subsystems.intake.IntakeIO;

/**
 * The ideal test pivot, made to take real time to arrive once the robot has homed.
 *
 * <p>Added for M13 step 6. {@link IdealPivotIO} reports its goal on the very next loop, so an
 * intake deployed at the start of an auto is retaining pieces before the chassis could possibly
 * reach walking pace. On that pivot, "nothing moves before the intake retains" passes whether or
 * not a routine waits at all. This one slews its reported angle toward the ideal pivot's at {@link
 * #RATE_RAD_PER_SEC}, which brings the arm from stow into the retaining range in about half a
 * second: inside the routines' wait bound, and long enough that a routine which skipped the wait
 * would visibly drive off first.
 *
 * <p>Homing is left ideal, because it seeds the angle rather than moving to it. The harness turns
 * the slew on once the robot has homed.
 */
final class SlowPivotIO implements IntakeIO {
  /** How fast the reported angle follows the ideal one, radians per second. */
  static final double RATE_RAD_PER_SEC = 2.0;

  private static final double LOOP_SEC = SimulatedAutoRobot.LOOP_SEC;

  private final IdealPivotIO ideal = new IdealPivotIO();
  private double reportedRad = Double.NaN;

  /** Whether the slew is on; off while the robot homes. */
  boolean slow = false;

  /**
   * Reports the ideal pivot's inputs, with the angle slewed toward it while {@link #slow}.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    ideal.updateInputs(inputs);
    if (!slow || Double.isNaN(reportedRad)) {
      reportedRad = inputs.pivotPositionRad;
      return;
    }
    double step = RATE_RAD_PER_SEC * LOOP_SEC;
    double error = inputs.pivotPositionRad - reportedRad;
    reportedRad += Math.max(-step, Math.min(step, error));
    inputs.pivotPositionRad = reportedRad;
    inputs.pivotVelocityRadPerSec =
        Math.abs(error) > step ? Math.copySign(RATE_RAD_PER_SEC, error) : 0.0;
  }

  /**
   * Passes a closed-loop goal to the ideal pivot.
   *
   * @param positionRad the goal, radians
   */
  @Override
  public void setPivotPosition(double positionRad) {
    ideal.setPivotPosition(positionRad);
  }

  /**
   * Passes an open-loop drive to the ideal pivot.
   *
   * @param volts the voltage
   */
  @Override
  public void setPivotOpenLoop(double volts) {
    ideal.setPivotOpenLoop(volts);
  }

  /**
   * Passes a roller speed to the ideal pivot.
   *
   * @param velocityRpm the roller speed, RPM
   */
  @Override
  public void setRollerVelocity(double velocityRpm) {
    ideal.setRollerVelocity(velocityRpm);
  }

  /**
   * Passes an open-loop roller drive to the ideal pivot.
   *
   * @param volts the voltage
   */
  @Override
  public void setRollerOpenLoop(double volts) {
    ideal.setRollerOpenLoop(volts);
  }

  /**
   * Seeds the ideal pivot's angle, and the reported one with it.
   *
   * @param positionRad the seeded angle, radians
   */
  @Override
  public void seedPivotPosition(double positionRad) {
    ideal.seedPivotPosition(positionRad);
    reportedRad = positionRad;
  }

  /**
   * Passes brake mode to the ideal pivot.
   *
   * @param brake true for brake
   */
  @Override
  public void setPivotBrakeMode(boolean brake) {
    ideal.setPivotBrakeMode(brake);
  }

  /**
   * Passes the soft-limit setting to the ideal pivot.
   *
   * @param enabled true to enable the soft limits
   */
  @Override
  public void setPivotSoftLimitsEnabled(boolean enabled) {
    ideal.setPivotSoftLimitsEnabled(enabled);
  }

  /**
   * Passes gains to the ideal pivot.
   *
   * @param gains the gains
   */
  @Override
  public void setPivotGains(Slot0Configs gains) {
    ideal.setPivotGains(gains);
  }

  /**
   * Passes the current-limit scalar to the ideal pivot.
   *
   * @param scalar the scalar
   */
  @Override
  public void setCurrentLimitScalar(double scalar) {
    ideal.setCurrentLimitScalar(scalar);
  }

  /** Stops the ideal pivot. */
  @Override
  public void stop() {
    ideal.stop();
  }
}
