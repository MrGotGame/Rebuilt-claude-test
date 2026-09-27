// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.util.FaultReporter;
import org.littletonrobotics.junction.Logger;

public class Module {
  /** Each module's corner, in the drivetrain's FL, FR, BL, BR index order. */
  private static final String[] CORNERS = {"Front-left", "Front-right", "Back-left", "Back-right"};

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    // M14 step 2: the template's three plain alerts, registered with FaultReporter instead, and
    // named by corner, CAN ID and bus rather than by module index, since "module 2" does not say
    // which corner to open. Errors that do not block scoring: a robot on three modules still
    // drives, and odometry's trouble shows up in gate terms 5 and 6 rather than here.
    String corner = CORNERS[index];
    String bus = TunerConstants.kCANBus.getName();
    FaultReporter reporter = FaultReporter.getInstance();
    reporter.registerDeviceDisconnect(
        FaultReporter.canDevice(corner + " drive motor", constants.DriveMotorId, bus),
        FaultReporter.Impact.REPORT_ONLY,
        () -> inputs.driveConnected);
    reporter.registerDeviceDisconnect(
        FaultReporter.canDevice(corner + " steer motor", constants.SteerMotorId, bus),
        FaultReporter.Impact.REPORT_ONLY,
        () -> inputs.turnConnected);
    reporter.registerDeviceDisconnect(
        FaultReporter.canDevice(corner + " CANcoder", constants.EncoderId, bus),
        FaultReporter.Impact.REPORT_ONLY,
        () -> inputs.turnEncoderConnected);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }

  // ------------------------------------------------------------ tier-3 additions start here ----

  /**
   * Sets brake or coast on both of this module's motors.
   *
   * @param brake true for brake, false for coast
   *     <p><b>Outputs:</b> forwards to the IO layer, which reconfigures both Talons.
   */
  public void setBrakeMode(boolean brake) {
    // A pure passthrough. Module deliberately holds no neutral-mode state of its own: the Talon is
    // the authority on its own configuration, and caching a shadow copy here would be one more
    // thing that can disagree with hardware after a brownout reconfigures a device.
    io.setBrakeMode(brake);
  }

  /**
   * This module's total stator current draw.
   *
   * @return the drive motor's stator current plus the steer motor's, in amps
   */
  public double getTotalCurrentAmps() {
    // The inputs struct is private to this class, so Drive cannot sum the modules' currents itself.
    // This accessor exists purely so Drive.getTotalCurrentAmps() has something to add up.
    //
    // STATOR current, not supply: stator is what the motor windings actually see and what the
    // current limit acts on, which is the number that matters when deciding whether the drivetrain
    // is what is draining the battery.
    return inputs.driveCurrentAmps + inputs.turnCurrentAmps;
  }
}
