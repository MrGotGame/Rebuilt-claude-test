package frc.robot.subsystems.intake;

/**
 * An intake pivot that goes exactly where it is told, and stalls when driven open loop, together
 * with the realistic geometry it is tested against.
 *
 * <p>The simulated arm is the right tool for homing and gravity, but with the placeholder gains it
 * rings for seconds and never settles at a goal, so any case about WHICH state the machine reports
 * once the arm arrives would be testing the gains. This reports the commanded angle on the next
 * update, and the stall current homing looks for whenever it is driven open loop. It also records
 * what it was last commanded, which is what the rocking cases assert on.
 *
 * <p>Written for M10b step 3 inside {@code IntakeFsmTest} and moved out in step 4, because the
 * intake-camera gate needs a real intake that can rest deployed, and that case lives with the
 * vision tests.
 */
public class IdealPivotIO implements IntakeIO {
  /** Stow angle of the test geometry, radians. */
  public static final double STOWED = -1.2;

  /** Deploy angle of the test geometry, radians. */
  public static final double DEPLOYED = 0.3;

  /** Feed-assist angle of the test geometry: between the two, clear of the band. */
  public static final double FEED_ASSIST = -0.2;

  /** Eject angle of the test geometry, radians. */
  public static final double EJECT = 0.5;

  /** Band width of the test geometry: the plan's 15 degrees nearest stow. */
  public static final double BAND = Math.toRadians(15.0);

  /**
   * A real, non-degenerate geometry. Every angle constant is still a 0.0 placeholder, which puts
   * stow, deploy and feed-assist at one point: the band has no direction and the rock no amplitude,
   * and every assertion about either would pass for free. The M8 lesson, again.
   */
  public static final Intake.PivotGeometry GEOMETRY =
      new Intake.PivotGeometry(
          () -> STOWED, () -> DEPLOYED, () -> EJECT, () -> FEED_ASSIST, () -> BAND);

  private double positionRad = 0.0;
  private double goalRad = 0.0;
  private boolean closedLoop = false;
  private double openLoopVolts = 0.0;
  private double lastRollerRpm = 0.0;

  /**
   * Reports the pivot where it was last sent, stalled against a stop while driven open loop.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (closedLoop) {
      positionRad = goalRad;
    }
    inputs.pivotConnected = true;
    inputs.rollerConnected = true;
    inputs.absEncoderConnected = true;
    inputs.pivotPositionRad = positionRad;
    inputs.pivotVelocityRadPerSec = 0.0;
    inputs.pivotStatorCurrentAmps = !closedLoop && openLoopVolts != 0.0 ? 40.0 : 0.0;
  }

  /**
   * Records a closed-loop goal.
   *
   * @param positionRad the goal, in radians
   */
  @Override
  public void setPivotPosition(double positionRad) {
    closedLoop = true;
    goalRad = positionRad;
  }

  /**
   * Records an open-loop drive.
   *
   * @param volts the voltage
   */
  @Override
  public void setPivotOpenLoop(double volts) {
    closedLoop = false;
    openLoopVolts = volts;
  }

  /**
   * Records the roller speed.
   *
   * @param velocityRpm the speed, in RPM
   */
  @Override
  public void setRollerVelocity(double velocityRpm) {
    lastRollerRpm = velocityRpm;
  }

  /**
   * Moves the sensor, as a real seed does.
   *
   * @param positionRad the seeded angle, in radians
   */
  @Override
  public void seedPivotPosition(double positionRad) {
    this.positionRad = positionRad;
  }

  /**
   * Where the pivot is right now.
   *
   * @return the reported angle, in radians
   */
  public double positionRad() {
    return positionRad;
  }

  /**
   * The goal most recently commanded.
   *
   * @return the goal, in radians
   */
  public double lastGoalRad() {
    return goalRad;
  }

  /**
   * The roller speed most recently commanded.
   *
   * @return the speed, in RPM
   */
  public double lastRollerRpm() {
    return lastRollerRpm;
  }
}
