package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.floorroller.FloorRoller;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Feedforward and system-identification routines for every closed-loop mechanism.
 *
 * <p>Every closed-loop mechanism on this robot needs kS, kV and kA before its gains mean anything,
 * and guessing them costs more practice time than measuring them. This class centralises the
 * routines so a student runs one chooser entry, pulls the WPILOG into SysId, and reads off numbers.
 *
 * <p>Two flavours are provided. WPILib {@link SysIdRoutine} sweeps (quasistatic and dynamic, both
 * directions) suit mechanisms that can travel freely. A simpler stair-step routine suits the
 * flywheel and the rollers, where a slow voltage ramp measuring true steady state gives a cleaner
 * kV than a SysId sweep does.
 *
 * <p>Every routine commands the superstructure into its characterisation state first, so nothing
 * else on the robot moves, and every one is registered into the auto chooser only when {@code
 * Constants.kTuningMode} is true.
 */
public class CharacterizationCommands {
  private CharacterizationCommands() {}

  /**
   * Builds a standard SysId routine around one mechanism's voltage input.
   *
   * @param name log key name for this mechanism, for example {@code "Turret"}
   * @param voltageConsumer applies a commanded voltage to the mechanism
   * @param positionRadSupplier the mechanism's position, in radians
   * @param velocityRadPerSecSupplier the mechanism's velocity, in radians per second
   * @param requirement the subsystem to require, so nothing else commands it mid-sweep
   * @return the configured routine
   */
  private static SysIdRoutine buildRoutine(
      String name,
      Consumer<Voltage> voltageConsumer,
      DoubleSupplier positionRadSupplier,
      DoubleSupplier velocityRadPerSecSupplier,
      SubsystemBase requirement) {
    // TODO: Replace the empty state callback below with
    //   state -> Logger.recordOutput("SysId/" + name + "/State", state.toString())
    // Passing null for the log consumer in the Mechanism is correct here, because AdvantageKit's
    // WPILOG already captures the @AutoLog inputs that SysId needs.
    // TODO: VERIFY that against the AdvantageKit 26.0.2 SysId docs before relying on it.
    return new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4.0), null, state -> {}),
        new SysIdRoutine.Mechanism(voltageConsumer, null, requirement));
  }

  /**
   * Full four-leg SysId sweep of the turret, bounded so it stops before the hard stops.
   *
   * @param turret the turret
   * @param superstructure the superstructure, commanded into characterisation first
   * @return the routine command
   */
  public static Command turretSysId(Turret turret, Superstructure superstructure) {
    // TODO: Build a routine driving turret::setCharacterizationVolts and reading
    // turret::getPositionRad and turret::getCharacterizationVelocityRadPerSec. Sequence
    // quasistatic(kForward), quasistatic(kReverse), dynamic(kForward), dynamic(kReverse) with
    // Commands.waitSeconds(1.0) between each.
    // BOUND EVERY LEG with Commands.waitUntil on the turret position approaching
    // Constants.Turret.kMaxAngleRad / kMinAngleRad. This is the one SysId routine where a missing
    // limit check breaks hardware rather than merely producing bad data.
    return Commands.none();
  }

  /**
   * Full four-leg SysId sweep of the intake pivot.
   *
   * @param intake the intake
   * @param superstructure the superstructure, commanded into characterisation first
   * @return the routine command
   */
  public static Command intakePivotSysId(Intake intake, Superstructure superstructure) {
    // TODO: The same four-leg sequence, driving intake::setCharacterizationVolts.
    // Because gravity acts on this mechanism the fit yields kG as well as kS/kV/kA -- analyse it in
    // SysId as an "Arm" mechanism, NOT as "Simple", or kG will be folded into kS and the pivot will
    // sag at one end of its travel.
    return Commands.none();
  }

  /**
   * SysId sweep of the flywheel.
   *
   * @param shooter the shooter
   * @param superstructure the superstructure, commanded into characterisation first
   * @return the routine command
   */
  public static Command shooterSysId(Shooter shooter, Superstructure superstructure) {
    // TODO: Quasistatic and dynamic on shooter::setCharacterizationVolts, reading
    // shooter::getCharacterizationVelocityRotPerSec. Analyse as "Simple".
    // Run this with NO game piece in the indexer, or the piece will be fired mid-sweep.
    return Commands.none();
  }

  /**
   * Stair-step feedforward measurement for the flywheel, which beats a SysId sweep here.
   *
   * @param shooter the shooter
   * @param maxVolts the highest voltage to step to
   * @param stepVolts the increment between steps
   * @return the routine command
   */
  public static Command shooterStaircase(Shooter shooter, double maxVolts, double stepVolts) {
    // TODO: Step the applied voltage from 0 to maxVolts in stepVolts increments, holding each step
    // until velocity settles (change below about 25 RPM over 0.25 s), and log each
    // (volts, steady-state RPM) pair under "Characterization/Shooter/Steps".
    // kV is then the slope of that line and kS its intercept. This measures TRUE STEADY STATE,
    // which is why it gives a better flywheel kV than a swept SysId run.
    return Commands.none();
  }

  /**
   * Stair-step feedforward measurement for the indexer.
   *
   * @param indexer the indexer
   * @return the routine command
   */
  public static Command indexerFeedforward(Indexer indexer) {
    // TODO: The same staircase idea. A four-leg SysId sweep on a roller is overkill.
    return Commands.none();
  }

  /**
   * Stair-step feedforward measurement for the floor roller.
   *
   * @param floorRoller the floor roller
   * @return the routine command
   */
  public static Command floorRollerFeedforward(FloorRoller floorRoller) {
    // TODO: The same staircase idea.
    return Commands.none();
  }

  /**
   * All four drivetrain SysId legs in one chooser entry.
   *
   * @param drive the drivetrain
   * @return the routine command
   */
  public static Command driveSysIdFull(Drive drive) {
    // TODO: Sequence drive.sysIdQuasistatic(kForward), (kReverse), drive.sysIdDynamic(kForward),
    // (kReverse) with waits between, all wrapped in setUseSetpointGenerator(false) and restored to
    // true in finallyDo -- the setpoint generator would otherwise slew the ramp and corrupt the
    // fit.
    // This collapses the stock template's four chooser entries into one.
    return Commands.none();
  }

  /**
   * A safety verification rather than a characterisation: proves the turret never commands itself
   * past a hard stop.
   *
   * @param turret the turret
   * @return the verification command
   */
  public static Command turretUnwrapDemo(Turret turret) {
    // TODO: Command the turret through a scripted list of field angles that straddle the wrap
    // boundary while the chassis is held still, and assert in the log that the commanded position
    // never leaves [kMinAngleRad, kMaxAngleRad].
    // Run this once after EVERY mechanical change to the turret stop positions.
    return Commands.none();
  }

  /**
   * Fires one burst for the shot tables, from a standstill, at a hand-set flywheel speed.
   *
   * <p>Implemented for M11 step 3. This is THE routine that fills {@code Constants.Shot}. Stand the
   * robot at a taped distance, load the hopper, and run it: it aims straight at the goal, spins to
   * the requested speed, and fires the whole burst through the ordinary stationary gate. Count what
   * scores; time the flight on 120 fps video; adjust the speed and run it again until the burst
   * scores. The distance, the speed that scored and the time of flight are one row of the shot
   * table, and {@code tools/fit_shot_table.py} turns the rows into constants.
   *
   * <p>A burst, not one piece (plan section 2.4). Pieces after the first leave a wheel still
   * recovering from the one before, so a table measured one piece at a time over-predicts every
   * shot after the first.
   *
   * <p>The speed and the taped distance are read when the command STARTS, so retuning between
   * bursts needs no redeploy.
   *
   * @param superstructure the superstructure, whose stationary shot takes the speed override
   * @param shooter the flywheel, whose shot count says how many pieces the burst fired
   * @param rpm the flywheel speed to fire at, in RPM
   * @param tapedDistanceMeters the distance measured on the carpet, in meters, logged beside the
   *     pose estimate's so a disagreement between them shows up in the data
   * @param holdSec the longest one burst may take, in seconds
   * @return the measurement command; it logs one {@link ShotTableSample} under {@code ShotTable/}
   */
  public static Command measureShotTablePoint(
      Superstructure superstructure,
      Shooter shooter,
      DoubleSupplier rpm,
      DoubleSupplier tapedDistanceMeters,
      double holdSec) {
    // IMPLEMENTATION (was TODO: "Hold the robot still, force the flywheel to `rpm`, fire one piece,
    // and log "ShotTable/Distance" from RobotState.getInstance().getDistanceToGoal() alongside the
    // commanded RPM.")
    //
    // DEVIATIONS: a burst rather than one piece (above); suppliers rather than a fixed rpm, so the
    // speed is a tunable; and a taped distance beside the estimated one. "Hold the robot still" is
    // enforced by the stationary gate, which will not feed while the chassis moves, rather than by
    // taking the drivetrain away from the driver.
    return measureShotTablePoint(
        superstructure, shooter, rpm, tapedDistanceMeters, holdSec, ShotTableSample::log);
  }

  /**
   * {@link #measureShotTablePoint(Superstructure, Shooter, DoubleSupplier, DoubleSupplier, double)}
   * with the sample delivered to a caller, so a test can assert what a burst reports.
   *
   * <p>Public only because the whole-robot test harness lives in the superstructure's test package.
   * Robot code should call the five-argument form, which logs the sample.
   *
   * @param superstructure the superstructure, whose stationary shot takes the speed override
   * @param shooter the flywheel, whose shot count says how many pieces the burst fired
   * @param rpm the flywheel speed to fire at, in RPM
   * @param tapedDistanceMeters the distance measured on the carpet, in meters
   * @param holdSec the longest one burst may take, in seconds
   * @param sink receives the burst's summary when the command ends, however it ends
   * @return the measurement command
   */
  public static Command measureShotTablePoint(
      Superstructure superstructure,
      Shooter shooter,
      DoubleSupplier rpm,
      DoubleSupplier tapedDistanceMeters,
      double holdSec,
      Consumer<ShotTableSample> sink) {
    return new ShotTablePointCommand(
        superstructure, shooter, rpm, tapedDistanceMeters, holdSec, sink);
  }

  /**
   * One burst's worth of shot-table data.
   *
   * @param rpm the flywheel speed the burst was fired at, in RPM
   * @param estimatedDistanceMeters the pose estimate's distance to the goal at the end, in meters
   * @param tapedDistanceMeters the distance measured on the carpet, in meters
   * @param shotsFired pieces the flywheel counted during this burst
   * @param burstSec how long the burst ran, in seconds
   * @param timedOut true if it hit its time limit rather than emptying the hopper
   */
  public record ShotTableSample(
      double rpm,
      double estimatedDistanceMeters,
      double tapedDistanceMeters,
      int shotsFired,
      double burstSec,
      boolean timedOut) {
    /**
     * Writes the sample to the log, one key per field.
     *
     * <p><b>Outputs:</b> records every field under {@code ShotTable/}.
     */
    public void log() {
      Logger.recordOutput("ShotTable/Rpm", rpm);
      Logger.recordOutput("ShotTable/EstimatedDistanceMeters", estimatedDistanceMeters);
      Logger.recordOutput("ShotTable/TapedDistanceMeters", tapedDistanceMeters);
      Logger.recordOutput("ShotTable/ShotsFired", shotsFired);
      Logger.recordOutput("ShotTable/BurstSec", burstSec);
      Logger.recordOutput("ShotTable/TimedOut", timedOut);
    }
  }

  /**
   * The measurement burst, as a command with its own per-run state.
   *
   * <p>A class rather than a composition because each run needs the shot count and the time at
   * which it started, and because every way it can end -- the hopper emptying, the time limit, a
   * cancel -- must clear the override and report the burst.
   */
  private static final class ShotTablePointCommand extends Command {
    private final Superstructure superstructure;
    private final Shooter shooter;
    private final DoubleSupplier rpm;
    private final DoubleSupplier tapedDistanceMeters;
    private final double holdSec;
    private final Consumer<ShotTableSample> sink;
    private final Timer timer = new Timer();
    private double rpmThisBurst = 0.0;
    private double tapedThisBurst = 0.0;
    private int shotCountAtStart = 0;

    /**
     * Stores what the burst needs; nothing is commanded until it is scheduled.
     *
     * @param superstructure the superstructure
     * @param shooter the flywheel
     * @param rpm the flywheel speed supplier, in RPM
     * @param tapedDistanceMeters the taped distance supplier, in meters
     * @param holdSec the time limit, in seconds
     * @param sink receives the summary
     */
    ShotTablePointCommand(
        Superstructure superstructure,
        Shooter shooter,
        DoubleSupplier rpm,
        DoubleSupplier tapedDistanceMeters,
        double holdSec,
        Consumer<ShotTableSample> sink) {
      this.superstructure = superstructure;
      this.shooter = shooter;
      this.rpm = rpm;
      this.tapedDistanceMeters = tapedDistanceMeters;
      this.holdSec = holdSec;
      this.sink = sink;
      // No requirements, like Superstructure.setWantedStateCommand: requiring the superstructure
      // would interrupt the drivetrain's default command for nothing.
    }

    /**
     * Reads the tunables, sets the override and asks for the stationary shot.
     *
     * <p><b>Outputs:</b> sets the superstructure's shot-table speed and requests SCORE_STATIONARY.
     */
    @Override
    public void initialize() {
      rpmThisBurst = rpm.getAsDouble();
      tapedThisBurst = tapedDistanceMeters.getAsDouble();
      shotCountAtStart = shooter.getShotCount();
      timer.restart();
      // Override first, then the request: setWantedState clears the override for every OTHER
      // request, so the order only matters if someone changes that rule, and this order survives
      // it.
      superstructure.setShotTableRpm(rpmThisBurst);
      superstructure.setWantedState(Superstructure.WantedState.SCORE_STATIONARY);
    }

    /**
     * Whether the burst is over: the hopper reads empty after this burst fired, or time is up.
     *
     * @return true when the burst should end
     */
    @Override
    public boolean isFinished() {
      // "After this burst fired" is load-bearing. The empty belief persists after a burst, and a
      // hopper reloaded by hand does not reset it -- only a shot or intaking does -- so on its own
      // it would end the second burst of a session the instant it started, having measured nothing.
      boolean firedThisBurst = shooter.getShotCount() > shotCountAtStart;
      return (firedThisBurst && superstructure.likelyEmpty()) || timer.hasElapsed(holdSec);
    }

    /**
     * Returns the robot to rest, which clears the override, and reports the burst, however it
     * ended.
     *
     * @param interrupted true if the command was cancelled
     *     <p><b>Outputs:</b> requests DEFAULT, and delivers one sample.
     */
    @Override
    public void end(boolean interrupted) {
      // The DEFAULT request is also what clears the shot-table override: the superstructure drops
      // it on any request other than SCORE_STATIONARY. One rule, so there is one thing to test.
      superstructure.setWantedState(Superstructure.WantedState.DEFAULT);
      sink.accept(
          new ShotTableSample(
              rpmThisBurst,
              RobotState.getInstance().getDistanceToGoal(),
              tapedThisBurst,
              shooter.getShotCount() - shotCountAtStart,
              timer.get(),
              timer.hasElapsed(holdSec)));
    }
  }
}
