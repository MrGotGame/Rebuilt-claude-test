// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CoastRelease;
import frc.robot.util.FaultReporter;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.PowerBudget;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The robot's entry point and lifecycle.
 *
 * <p>The AdvantageKit logger wiring below is the template's, kept verbatim. Four competition
 * behaviors are layered on top, all complete as of M14: real-time thread priority around the
 * scheduler so 250 Hz odometry is not starved; a per-loop call into the fault reporter and power
 * budget; loop-overrun and memory logging so a missed shot can be proven in replay to have been a
 * timing problem or not; and a brake-mode release timer that lets the drivetrain coast a few
 * seconds after a match so the robot can be pushed off the field.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private double autoStartTimeSec = 0.0;
  private final CoastRelease coastRelease = new CoastRelease();

  /** Starts the logger, then builds the robot. The order matters and must not be changed. */
  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger. This MUST precede constructing any subsystem.
    Logger.start();

    robotContainer = new RobotContainer();

    // M14 step 6. Tuning mode must be off at an event: it publishes every constant as editable,
    // and an edit made in the pit survives until the next reboot. The flag is final, so this can
    // only fire on code built with it on; the FMS being attached is what says this is a real match.
    FaultReporter.getInstance()
        .register(
            "Tuning mode is ON with the FMS attached; set Constants.kTuningMode false and redeploy",
            AlertType.kWarning,
            FaultReporter.Impact.REPORT_ONLY,
            () -> Constants.kTuningMode && DriverStation.isFMSAttached());
  }

  /**
   * Runs the scheduler and the per-loop housekeeping, at elevated thread priority.
   *
   * <p><b>Outputs:</b> runs every subsystem's periodic; publishes robot-state, fault, power and
   * loop-timing telemetry.
   */
  @Override
  public void robotPeriodic() {
    // IMPLEMENTATION, M14 step 6 (was TODO: "Wrap the scheduler run in
    // Threads.setCurrentThreadPriority(true, 99) before and Threads.setCurrentThreadPriority(false,
    // 10) after ... Between those, also call, in this order: PhoenixUtil.refreshAll(),
    // CommandScheduler.getInstance().run(), RobotState.getInstance().logState(),
    // FaultReporter.getInstance().periodic(), PowerBudget.getInstance().periodic() ... Log loop
    // duration under \"LoopTimeMs\" from Logger.getRealTimestamp() deltas, and
    // Runtime.getRuntime().freeMemory() under \"MemoryFreeMB\".")
    //
    // Real-time priority for the whole loop body, as AdvantageKit's own template does, so the
    // 250 Hz odometry thread is not starved by a long command loop -- and dropped again before the
    // logger's own work between loops, which must not be.
    Threads.setCurrentThreadPriority(true, 99);

    // Refresh every signal registered with the shared batch BEFORE the scheduler runs, so that
    // subsystem periodics all see measurements from the same instant. Without this call the
    // signals that IO layers registered -- currently the Pigeon2's pitch, roll and accelerometer --
    // are never refreshed and read zero forever, which looks exactly like a dead sensor.
    PhoenixUtil.refreshAll();

    CommandScheduler.getInstance().run();

    // Per-source staleness and standard deviations, so a replay shows which source was driving the
    // estimate at every instant.
    RobotState.getInstance().logState();

    // Implemented in M13 step 7, ahead of the rest of this list. FaultReporter only drives its
    // alerts from periodic(), and nothing called it, so every fault registered with it was silent
    // on the dashboard. After the scheduler, so it reads this loop's state.
    FaultReporter.getInstance().periodic();

    // M14 step 3. The subsystems read their current-limit scalars during the scheduler run above,
    // so they act on the previous loop's budget: 20 ms behind a battery whose filter is five times
    // slower than that, which is why the plan's order is kept.
    PowerBudget.getInstance().periodic();

    // Wall-clock time between the starts of successive loops: the evidence that settles "did we
    // miss that shot because the code stalled". The TODO named Logger.getRealTimestamp(), which
    // AdvantageKit 26 does not have; its Logger.getTimestamp() is the LOG's time, which in replay
    // is
    // the recorded loop rather than this one. The FPGA clock is this machine's.
    long nowMicros = RobotController.getFPGATime();
    if (lastLoopStartMicros > 0) {
      Logger.recordOutput("LoopTimeMs", (nowMicros - lastLoopStartMicros) / 1000.0);
    }
    lastLoopStartMicros = nowMicros;
    Logger.recordOutput("MemoryFreeMB", Runtime.getRuntime().freeMemory() / 1.0e6);

    Threads.setCurrentThreadPriority(false, 10);
  }

  /** When the previous loop started, microseconds of real time; 0 before the first loop. */
  private long lastLoopStartMicros = 0;

  /**
   * Starts the coast-release timer when the robot is disabled.
   *
   * <p><b>Outputs:</b> restarts the disabled timer and parks the superstructure.
   */
  @Override
  public void disabledInit() {
    // IMPLEMENTATION (was TODO: "disabledTimer.restart(); brakeModeReleased = false;
    // robotContainer.onDisable().")
    //
    // Implemented as part of M10, because M10 is the first milestone with a disable path worth
    // running: Superstructure.onDisable drops the force-shoot override, forgets the shot solution
    // and releases any requested drive heading, and none of that happens unless this method calls
    // it. Leaving the call out would have made that safety contract dead code.
    //
    // The timer and the once-only latch moved into CoastRelease in M14 step 4, which also owns the
    // release itself; see disabledPeriodic.
    coastRelease.onDisabled();
    robotContainer.onDisable();
  }

  /**
   * Releases brake mode once the robot has been still for long enough, and refreshes the auto
   * preview.
   *
   * <p><b>Outputs:</b> may drop the drivetrain to coast; logs the selected auto's start pose.
   */
  @Override
  public void disabledPeriodic() {
    // IMPLEMENTATION, M14 step 4 (was TODO: "Once disabledTimer.hasElapsed(
    // Constants.Drivetrain.kCoastDelaySec.get()) AND the robot is nearly stationary, call
    // robotContainer.setDriveBrakeMode(false) ONCE (guard on brakeModeReleased).")
    //
    // The decision is CoastRelease's, where it can be tested; Robot cannot be.
    ChassisSpeeds speeds = RobotState.getInstance().getFieldRelativeSpeeds();
    if (coastRelease.shouldReleaseNow(
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))) {
      robotContainer.setDriveBrakeMode(false);
    }
    //
    // IMPLEMENTATION, M13 step 7 (was TODO: "Also call robotContainer.updateAutoPreview() so the
    // drive team can check placement."). The coast release above stays M14's.
    robotContainer.updateAutoPreview();
  }

  /**
   * Schedules the selected autonomous routine.
   *
   * <p><b>Outputs:</b> re-enables brake mode, records the start time, and schedules the routine.
   */
  @Override
  public void autonomousInit() {
    // IMPLEMENTED by M14 step 4: brake again, since the robot may have been released to coast.
    // IMPLEMENTED by M13 step 7: the start time, for Auto/ElapsedSec.
    robotContainer.setDriveBrakeMode(true);
    autoStartTimeSec = Timer.getFPGATimestamp();
    //
    // IMPLEMENTED in part by M10b step 7: the shot count reset the TODO asked for (now the
    // flywheel's, not the indexer's), and the first-enable intake deploy -- both in
    // onAutonomousInit, BEFORE the routine is scheduled, so the routine's first loop already sees
    // the intake heading down.
    robotContainer.onAutonomousInit();
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /**
   * Logs elapsed autonomous time.
   *
   * <p><b>Outputs:</b> publishes {@code Auto/ElapsedSec}.
   */
  @Override
  public void autonomousPeriodic() {
    // IMPLEMENTATION, M13 step 7 (was TODO: "Logger.recordOutput(\"Auto/ElapsedSec\",
    // Timer.getFPGATimestamp() - autoStartTimeSec), so routine timing can be compared across
    // matches in AdvantageScope.")
    Logger.recordOutput("Auto/ElapsedSec", Timer.getFPGATimestamp() - autoStartTimeSec);
  }

  /**
   * Records how autonomous finished.
   *
   * <p><b>Outputs:</b> logs the final pose and shot count, so routines can be scored from the log.
   */
  @Override
  public void autonomousExit() {
    // IMPLEMENTATION, M13 step 7 (was TODO: "Log the final estimated pose and the shooter's shot
    // count."). In RobotContainer, which holds the shooter.
    robotContainer.logAutonomousExit();
  }

  /**
   * Hands control to the drivers.
   *
   * <p><b>Outputs:</b> cancels the autonomous command, re-enables brake mode, and resets the
   * superstructure goal.
   */
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // IMPLEMENTED by M14 step 4: brake again, since the robot may have been released to coast.
    robotContainer.setDriveBrakeMode(true);
    //
    // IMPLEMENTED by M10b step 7: the superstructure reset to DEFAULT this TODO asked for, so
    // autonomous can never leak a goal into teleop -- plus releasing any intake request an auto
    // left
    // standing, and deploying the intake if this is the first enable. See onTeleopInit.
    robotContainer.onTeleopInit();
  }

  /** Nothing to do; all teleop behavior lives in commands. */
  @Override
  public void teleopPeriodic() {}

  /** Cancels everything before test mode. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Nothing to do. */
  @Override
  public void testPeriodic() {}

  /**
   * Seeds the physics simulation.
   *
   * <p><b>Outputs:</b> places the simulated robot, if maple-sim is installed.
   */
  @Override
  public void simulationInit() {
    // TODO: If you install maple-sim, seed SimulatedArena.getInstance() and place the robot at the
    // selected auto's start pose. Otherwise leave this empty.
  }

  /**
   * Advances the physics simulation.
   *
   * <p><b>Outputs:</b> steps the simulated arena and logs simulated game pieces.
   */
  @Override
  public void simulationPeriodic() {
    // TODO: If maple-sim is installed, call SimulatedArena.getInstance().simulationPeriodic() and
    // log the simulated game pieces under "FieldSimulation/GamePieces" as a Pose3d[].
  }
}
