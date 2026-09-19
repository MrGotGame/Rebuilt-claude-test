// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Driver-facing drive commands.
 *
 * <p>[TIER-3 MIGRATION DONE] Every heading read now comes from {@code
 * RobotState.getInstance().getRotation()} rather than from {@code Drive}. That substitution is what
 * makes field-relative driving use the FUSED heading -- gyro corrected by vision -- instead of raw
 * wheel odometry, which drifts over a match.
 *
 * <p>[TIER-3 MIGRATION DONE] The private heading/deadband literals have become {@code
 * Constants.Drivetrain} tunables, refreshed each loop through {@code
 * LoggedTunableNumber.ifChanged}, so the heading controller can be retuned from AdvantageScope
 * without a redeploy. The four {@code FF_*} and {@code WHEEL_RADIUS_*} constants below are
 * deliberately NOT tunable: they parameterise one-off characterization routines, not match
 * behaviour, and making them editable at a competition would invite someone to invalidate a fit.
 *
 * <p>Both characterization routines bypass the swerve setpoint generator for their duration, and
 * restore it in {@code finallyDo} so an interrupted sweep cannot leave the drivetrain unslewed.
 */
public class DriveCommands {
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  /**
   * Converts raw stick input into a deadbanded, squared linear velocity vector.
   *
   * @param x forward stick input, in [-1, 1]
   * @param y strafe stick input, in [-1, 1]
   * @return the commanded linear velocity direction and magnitude, normalised to [0, 1]
   */
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), Constants.Drivetrain.kJoystickDeadband.get());
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega =
              MathUtil.applyDeadband(
                  omegaSupplier.getAsDouble(), Constants.Drivetrain.kJoystickDeadband.get());

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? RobotState.getInstance().getRotation().plus(new Rotation2d(Math.PI))
                      : RobotState.getInstance().getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    // Built from the tunables rather than from literals. enableContinuousInput is what lets the
    // controller take the short way around through +/-180 degrees instead of unwinding the long
    // way.
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            Constants.Drivetrain.kHeadingKp.get(),
            0.0,
            Constants.Drivetrain.kHeadingKd.get(),
            new TrapezoidProfile.Constraints(
                Constants.Drivetrain.kHeadingMaxVelRadPerSec.get(),
                Constants.Drivetrain.kHeadingMaxAccelRadPerSec2.get()));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Re-read the heading gains whenever someone edits them on the dashboard. ifChanged
              // fires only on an actual edit, so this costs a map lookup per loop and nothing more.
              LoggedTunableNumber.ifChanged(
                  angleController.hashCode(),
                  () -> {
                    angleController.setPID(
                        Constants.Drivetrain.kHeadingKp.get(),
                        0.0,
                        Constants.Drivetrain.kHeadingKd.get());
                    angleController.setConstraints(
                        new TrapezoidProfile.Constraints(
                            Constants.Drivetrain.kHeadingMaxVelRadPerSec.get(),
                            Constants.Drivetrain.kHeadingMaxAccelRadPerSec2.get()));
                  },
                  Constants.Drivetrain.kHeadingKp,
                  Constants.Drivetrain.kHeadingKd,
                  Constants.Drivetrain.kHeadingMaxVelRadPerSec,
                  Constants.Drivetrain.kHeadingMaxAccelRadPerSec2);

              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      RobotState.getInstance().getRotation().getRadians(),
                      rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? RobotState.getInstance().getRotation().plus(new Rotation2d(Math.PI))
                          : RobotState.getInstance().getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(
            () -> angleController.reset(RobotState.getInstance().getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                }),

            // Allow modules to orient
            Commands.run(
                    () -> {
                      drive.runCharacterization(0.0);
                    },
                    drive)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                    () -> {
                      double voltage = timer.get() * FF_RAMP_RATE;
                      drive.runCharacterization(voltage);
                      velocitySamples.add(drive.getFFCharacterizationVelocity());
                      voltageSamples.add(voltage);
                    },
                    drive)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      int n = velocitySamples.size();
                      double sumX = 0.0;
                      double sumY = 0.0;
                      double sumXY = 0.0;
                      double sumX2 = 0.0;
                      for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                      }
                      double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                      double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                      NumberFormat formatter = new DecimalFormat("#0.00000");
                      System.out.println("********** Drive FF Characterization Results **********");
                      System.out.println("\tkS: " + formatter.format(kS));
                      System.out.println("\tkV: " + formatter.format(kV));
                    }))
        // [TIER-3 ADDITION] Bypass the swerve setpoint generator for the duration of the routine.
        // The generator slew-limits module azimuth rate and wheel force, which is exactly what you
        // want while driving and exactly what you must not have while characterizing: it would
        // reshape the commanded voltage ramp, and the kS/kV fit would describe the generator's
        // behaviour rather than the drivetrain's.
        //
        // finallyDo rather than a trailing runOnce, so the generator is restored even if the
        // routine is interrupted -- otherwise cancelling a SysId sweep would leave the robot
        // driving unslewed for the rest of the match.
        .beforeStarting(() -> drive.setUseSetpointGenerator(false))
        .finallyDo(() -> drive.setUseSetpointGenerator(true));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                      limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                      double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                      state.positions = drive.getWheelRadiusCharacterizationPositions();
                      state.lastAngle = RobotState.getInstance().getRotation();
                      state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                          var rotation = RobotState.getInstance().getRotation();
                          state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                          state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                          double[] positions = drive.getWheelRadiusCharacterizationPositions();
                          double wheelDelta = 0.0;
                          for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                          }
                          double wheelRadius =
                              (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                          NumberFormat formatter = new DecimalFormat("#0.000");
                          System.out.println(
                              "********** Wheel Radius Characterization Results **********");
                          System.out.println(
                              "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                          System.out.println(
                              "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                          System.out.println(
                              "\tWheel Radius: "
                                  + formatter.format(wheelRadius)
                                  + " meters, "
                                  + formatter.format(Units.metersToInches(wheelRadius))
                                  + " inches");
                        })))
        // [TIER-3 ADDITION] Bypass the swerve setpoint generator for the duration of the routine.
        // The generator slew-limits module azimuth rate and wheel force, which is exactly what you
        // want while driving and exactly what you must not have while characterizing: it would
        // reshape the commanded voltage ramp, and the kS/kV fit would describe the generator's
        // behaviour rather than the drivetrain's.
        //
        // finallyDo rather than a trailing runOnce, so the generator is restored even if the
        // routine is interrupted -- otherwise cancelling a SysId sweep would leave the robot
        // driving unslewed for the rest of the match.
        .beforeStarting(() -> drive.setUseSetpointGenerator(false))
        .finallyDo(() -> drive.setUseSetpointGenerator(true));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  // ------------------------------------------------------------ tier-3 additions start here ----

  /**
   * Field-relative drive that lets the superstructure blend in a requested heading.
   *
   * <p>This is what lets the drivetrain help the turret reach an aim point the turret cannot reach
   * alone, without ever taking the robot away from the driver. The override is an <b>additive
   * nudge</b>, never a takeover: a driver who cannot override an assist will not use it.
   *
   * @param drive the drivetrain
   * @param xSupplier forward stick input, in [-1, 1]
   * @param ySupplier strafe stick input, in [-1, 1]
   * @param omegaSupplier rotation stick input, in [-1, 1]
   * @param headingOverrideSupplier the heading the superstructure would like, or empty for none;
   *     bind this to {@code superstructure::getRequestedDriveHeading}
   * @return the drive command, suitable as the default command
   */
  public static Command joystickDriveWithOverride(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Optional<Rotation2d>> headingOverrideSupplier) {
    // IMPLEMENTATION (was TODO: "Each loop, if headingOverrideSupplier.get() is present AND
    // |omegaSupplier| is below Constants.Drivetrain.kJoystickDeadband, run the joystickDriveAtAngle
    // heading controller toward that angle; otherwise use the driver's omega verbatim.")
    //
    // One controller instance is captured by the returned command rather than being rebuilt per
    // loop, so its motion profile carries state across loops the way a profiled controller must.
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            Constants.Drivetrain.kHeadingKp.get(),
            0.0,
            Constants.Drivetrain.kHeadingKd.get(),
            new TrapezoidProfile.Constraints(
                Constants.Drivetrain.kHeadingMaxVelRadPerSec.get(),
                Constants.Drivetrain.kHeadingMaxAccelRadPerSec2.get()));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Tracks whether the override was engaged last loop, so the controller's profile can be reset
    // on the rising edge. Without that reset the profile would start from whatever state it held
    // when the override last disengaged, and the turret-assist nudge would begin with a jolt.
    // A one-element array is used because a lambda can only capture effectively-final locals.
    boolean[] wasOverriding = new boolean[] {false};

    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double rawOmega = omegaSupplier.getAsDouble();
          Optional<Rotation2d> override = headingOverrideSupplier.get();

          // The DRIVER WINS. The override only applies while the rotation stick is inside its
          // deadband, so the moment the driver touches it they take rotation back with no mode
          // switch and no button. An assist a driver cannot override is an assist they will not
          // use.
          boolean driverWantsRotation =
              Math.abs(
                      MathUtil.applyDeadband(
                          rawOmega, Constants.Drivetrain.kJoystickDeadband.get()))
                  > 0.0;
          boolean overrideActive = override.isPresent() && !driverWantsRotation;

          double omega;
          if (overrideActive) {
            if (!wasOverriding[0]) {
              // Rising edge: start the profile from where the robot actually is.
              angleController.reset(RobotState.getInstance().getRotation().getRadians());
            }
            omega =
                angleController.calculate(
                    RobotState.getInstance().getRotation().getRadians(),
                    override.get().getRadians());
          } else {
            // Driver's stick, treated exactly as joystickDrive treats it: deadband, then square for
            // fine control near centre.
            double deadbanded =
                MathUtil.applyDeadband(rawOmega, Constants.Drivetrain.kJoystickDeadband.get());
            omega =
                Math.copySign(deadbanded * deadbanded, deadbanded)
                    * drive.getMaxAngularSpeedRadPerSec();
          }
          wasOverriding[0] = overrideActive;

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega);
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? RobotState.getInstance().getRotation().plus(new Rotation2d(Math.PI))
                      : RobotState.getInstance().getRotation()));

          Logger.recordOutput("Drive/HeadingOverrideActive", overrideActive);
        },
        drive);
  }

  /**
   * Biases rotation toward the intake camera's best game-piece detection while the driver keeps
   * full control of translation.
   *
   * <p>Takes an {@code omegaSupplier} that the plan's signature did not. Without it the command
   * cannot hand rotation back when no target is visible, and a held assist that pins rotation to
   * zero is precisely the "assist that freezes the robot" the design warns against.
   *
   * @param drive the drivetrain
   * @param vision the vision subsystem, queried for the best target
   * @param xSupplier forward stick input, in [-1, 1]
   * @param ySupplier strafe stick input, in [-1, 1]
   * @param omegaSupplier rotation stick input, in [-1, 1]; used whenever no target is visible
   * @return the assist command; bind it to a held button only
   */
  public static Command gamePieceAssist(
      Drive drive,
      Vision vision,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    // IMPLEMENTATION (was TODO: "Read vision.getBestGamePieceTarget(). Rotate the chassis so the
    // target's tx goes to zero using the same ProfiledPIDController gains as joystickDriveAtAngle,
    // and pass the driver's translation through unchanged (robot-relative). If the Optional is
    // empty, degrade to plain joystickDrive rather than stopping.")
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            Constants.Drivetrain.kHeadingKp.get(),
            0.0,
            Constants.Drivetrain.kHeadingKd.get(),
            new TrapezoidProfile.Constraints(
                Constants.Drivetrain.kHeadingMaxVelRadPerSec.get(),
                Constants.Drivetrain.kHeadingMaxAccelRadPerSec2.get()));

    // NOTE: continuous input is deliberately NOT enabled here. Unlike a field heading, tx is a
    // bounded camera-frame error that never wraps, and treating it as circular would let the
    // controller decide to spin the long way around to reach a target a few degrees off centre.

    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          var target = vision.getBestGamePieceTarget();

          double omega;
          if (target.isPresent()) {
            // Drive tx to zero. Negated because a target to the camera's left reads as positive tx
            // and requires a positive (counter-clockwise) chassis rotation to centre it.
            omega = -angleController.calculate(target.get().tx().getRadians(), 0.0);
          } else {
            // Degrade to the driver's own rotation rather than freezing. An assist that pins
            // rotation the instant it loses its target is worse than no assist at all -- the driver
            // is holding the button precisely because they are chasing something, and taking their
            // yaw away mid-chase is how an assist earns a reputation for being unusable.
            double deadbanded =
                MathUtil.applyDeadband(
                    omegaSupplier.getAsDouble(), Constants.Drivetrain.kJoystickDeadband.get());
            omega =
                Math.copySign(deadbanded * deadbanded, deadbanded)
                    * drive.getMaxAngularSpeedRadPerSec();

            // Reset the profile so that reacquiring the target starts from the robot's real yaw
            // rate instead of a stale profile state, which would otherwise show up as a kick at the
            // moment the camera picks the piece back up.
            angleController.reset(0.0);
          }

          // ROBOT-RELATIVE, not field-relative: while this assist is held, "forward" on the stick
          // means "toward the piece", which is what a driver chasing a game piece actually wants.
          drive.runVelocity(
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega));

          Logger.recordOutput("Drive/GamePieceAssistHasTarget", target.isPresent());
        },
        drive);
  }

  /**
   * Snaps the robot to a fixed field heading while the driver keeps translation control.
   *
   * @param drive the drivetrain
   * @param xSupplier forward stick input, in [-1, 1]
   * @param ySupplier strafe stick input, in [-1, 1]
   * @param blueHeading the target heading expressed in blue-alliance coordinates
   * @return the snap command; bind to the four POV directions so a driver can square up without
   *     looking
   */
  public static Command snapToHeading(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Rotation2d blueHeading) {
    // IMPLEMENTATION (was TODO: "joystickDriveAtAngle(drive, x, y, () -> AllianceFlipUtil.apply(
    // blueHeading)). Bound to the four POV directions so a driver can square up to the field
    // without looking.")
    //
    // The heading supplier is evaluated every loop rather than flipped once here, because the
    // alliance is not known until the driver station connects -- which is usually after this
    // command object has already been constructed and bound to a button.
    //
    // NOTE: AllianceFlipUtil is still stubbed, so today this returns the blue-alliance heading
    // unchanged on both alliances. Snapping will be mirrored incorrectly on red until that class is
    // implemented, which in turn needs the REBUILT field dimensions measured.
    return joystickDriveAtAngle(
        drive, xSupplier, ySupplier, () -> AllianceFlipUtil.apply(blueHeading));
  }

  /**
   * Drives to whichever candidate pose is closest right now.
   *
   * @param drive the drivetrain
   * @param candidates supplier of candidate scoring poses, in blue-origin field coordinates
   * @return the auto-align command
   */
  public static Command autoAlignToNearest(Drive drive, Supplier<List<Pose2d>> candidates) {
    // IMPLEMENTATION (was TODO: "Pick the candidate pose minimizing distance from
    // RobotState.getEstimatedPose(), then return new DriveToPoseCommand(drive, () -> chosen).
    // Evaluate the choice at schedule time, not every loop.")
    //
    // Commands.defer is what makes "at schedule time" work: the supplier runs once when the command
    // is scheduled, so the target is chosen from where the robot was at the button press and then
    // held. Choosing every loop would let the target flip back and forth between two
    // nearly-equidistant poses and leave the robot oscillating between them.
    //
    // NOTE: DriveToPoseCommand is still stubbed -- its isFinished() returns false and its execute()
    // commands nothing. Until it is implemented this will hold the drivetrain requirement without
    // moving, so do not bind it to a button yet.
    return Commands.defer(
        () -> {
          Pose2d current = RobotState.getInstance().getEstimatedPose();
          Pose2d nearest = null;
          double bestDistance = Double.POSITIVE_INFINITY;
          for (Pose2d candidate : candidates.get()) {
            double distance = candidate.getTranslation().getDistance(current.getTranslation());
            if (distance < bestDistance) {
              bestDistance = distance;
              nearest = candidate;
            }
          }
          if (nearest == null) {
            // No candidates supplied. Do nothing rather than driving somewhere arbitrary.
            return Commands.none();
          }
          Logger.recordOutput("Drive/AutoAlignTarget", nearest);
          final Pose2d target = nearest;
          return new DriveToPoseCommand(drive, () -> target);
        },
        Set.of(drive));
  }
}
