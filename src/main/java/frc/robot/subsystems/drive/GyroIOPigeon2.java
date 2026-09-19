// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  // Tier-3 additions. Pitch and roll feed tip detection; the accelerometer cross-checks the
  // acceleration RobotState derives by differentiating wheel velocity.
  private final StatusSignal<Angle> pitch = pigeon.getPitch();
  private final StatusSignal<Angle> roll = pigeon.getRoll();
  private final StatusSignal<LinearAcceleration> accelX = pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> accelY = pigeon.getAccelerationY();
  private final StatusSignal<LinearAcceleration> accelZ = pigeon.getAccelerationZ();

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);

    // Must run BEFORE optimizeBusUtilization below. Optimization disables every status frame that
    // nobody has explicitly requested, so any signal whose frequency has not yet been set would be
    // switched off and would then read as permanently stale.
    createExtraSignals();

    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  /**
   * Samples the Pigeon2 and drains its queued odometry yaw samples.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // The yaw pair is refreshed on its own, and its status alone defines "connected". The tier-3
    // signals are deliberately NOT folded into this call: they are refreshed by PhoenixUtil's
    // shared batch, and including them here would both double-refresh them and let a single
    // unavailable auxiliary signal mark the whole gyro as disconnected -- which would silently
    // drop the drivetrain onto its kinematics-only heading fallback.
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    // Pigeon2 reports pitch and roll in degrees; the rest of this codebase is radians.
    inputs.pitchRad = Units.degreesToRadians(pitch.getValueAsDouble());
    inputs.rollRad = Units.degreesToRadians(roll.getValueAsDouble());

    // Converted through the units library rather than by a hand-written scale factor. Phoenix
    // reports these in standard gravities, and .in(MetersPerSecondPerSecond) performs the g ->
    // m/s^2
    // conversion correctly whichever native unit the signal actually carries.
    //
    // NOTE: these readings INCLUDE gravity, so a level, stationary robot reports roughly +9.81 on
    // Z, not zero. Compare them against RobotState's differentiated acceleration only after
    // removing the gravity component, or use getGravityVector*() instead if you want that done for
    // you.
    inputs.accelXMetersPerSec2 = accelX.getValue().in(MetersPerSecondPerSecond);
    inputs.accelYMetersPerSec2 = accelY.getValue().in(MetersPerSecondPerSecond);
    inputs.accelZMetersPerSec2 = accelZ.getValue().in(MetersPerSecondPerSecond);

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  /**
   * Creates and registers the pitch, roll and accelerometer signals added for tier 3.
   *
   * <p><b>Outputs:</b> registers five additional status signals for batched refresh.
   */
  private void createExtraSignals() {
    // IMPLEMENTATION (was TODO: "pigeon.getPitch(), getRoll(), getAccelerationX/Y/Z(). Set update
    // frequency 50 Hz, add them to PhoenixUtil.registerSignals, and populate the new inputs in
    // updateInputs. Verify the acceleration signal names against Phoenix6 26.1.2.")
    //
    // The signals themselves are created as field initializers above, because they are final. This
    // method owns the two things that must happen in a specific order relative to
    // optimizeBusUtilization: setting the update rate, and joining the shared refresh batch.
    //
    // 50 Hz rather than the 250 Hz used for yaw. None of these feed odometry: tip detection and an
    // acceleration cross-check are both comfortably served at loop rate, and every extra frame here
    // is bus bandwidth taken from the four modules that do need it.
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, pitch, roll, accelX, accelY, accelZ);

    // Hand them to the shared batch so they are refreshed once per loop alongside every other
    // mechanism signal, rather than in a separate call here.
    PhoenixUtil.registerSignals(pitch, roll, accelX, accelY, accelZ);
  }
}
