// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.PhoenixUtil;
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
 * behaviors are layered on top: real-time thread priority around the scheduler so 250 Hz odometry
 * is not starved; a per-loop call into the fault reporter and power budget; loop-overrun and memory
 * logging so a missed shot can be proven in replay to have been a timing problem or not; and a
 * brake-mode release timer that lets the drivetrain coast a few seconds after a match so the robot
 * can be pushed off the field.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private double autoStartTimeSec = 0.0;
  private boolean brakeModeReleased = false;
  private final Timer disabledTimer = new Timer();

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
  }

  /**
   * Runs the scheduler and the per-loop housekeeping, at elevated thread priority.
   *
   * <p><b>Outputs:</b> runs every subsystem's periodic; publishes robot-state, fault, power and
   * loop-timing telemetry.
   */
  @Override
  public void robotPeriodic() {
    // TODO: Wrap the scheduler run in Threads.setCurrentThreadPriority(true, 99) before and
    // Threads.setCurrentThreadPriority(false, 10) after, so the 250 Hz odometry thread is not
    // starved by a long command loop.
    // TODO: Between those, also call, in this order:
    //   PhoenixUtil.refreshAll()                  (one batched CAN read for every mechanism)
    //   CommandScheduler.getInstance().run()
    //   RobotState.getInstance().logState()
    //   FaultReporter.getInstance().periodic()
    //   PowerBudget.getInstance().periodic()
    // TODO: Log loop duration under "LoopTimeMs" from Logger.getRealTimestamp() deltas, and
    // Runtime.getRuntime().freeMemory() under "MemoryFreeMB". Loop-overrun evidence is the only way
    // to settle "did we miss that shot because the code stalled".
    // Refresh every signal registered with the shared batch BEFORE the scheduler runs, so that
    // subsystem periodics all see measurements from the same instant. Without this call the
    // signals that IO layers registered -- currently the Pigeon2's pitch, roll and accelerometer --
    // are never refreshed and read zero forever, which looks exactly like a dead sensor.
    PhoenixUtil.refreshAll();

    CommandScheduler.getInstance().run();
  }

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
    // The two brake-mode statements belong to M14 and are implemented here rather than deferred
    // because they only prepare state: the timer that measures how long the robot has been still,
    // and the once-only latch that goes with it. What actually drops the drivetrain to coast lives
    // in disabledPeriodic and is still M14's, so setting these up now costs nothing and means the
    // timer is honest whenever that arrives.
    disabledTimer.restart();
    brakeModeReleased = false;
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
    // TODO: Once disabledTimer.hasElapsed(Constants.Drivetrain.kCoastDelaySec.get()) AND the robot
    // is nearly stationary, call robotContainer.setDriveBrakeMode(false) ONCE (guard on
    // brakeModeReleased). The stationary check matters -- dropping to coast while the robot is
    // still rolling means it keeps rolling.
    // TODO: Also call robotContainer.updateAutoPreview() so the drive team can check placement.
  }

  /**
   * Schedules the selected autonomous routine.
   *
   * <p><b>Outputs:</b> re-enables brake mode, records the start time, and schedules the routine.
   */
  @Override
  public void autonomousInit() {
    // TODO: robotContainer.setDriveBrakeMode(true); autoStartTimeSec = Timer.getFPGATimestamp().
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
    // TODO: Logger.recordOutput("Auto/ElapsedSec", Timer.getFPGATimestamp() - autoStartTimeSec), so
    // routine timing can be compared across matches in AdvantageScope.
  }

  /**
   * Records how autonomous finished.
   *
   * <p><b>Outputs:</b> logs the final pose and shot count, so routines can be scored from the log.
   */
  @Override
  public void autonomousExit() {
    // TODO: Log the final estimated pose and the shooter's shot count. (The skeleton said the
    // indexer's; M10b moved shot counting to the flywheel, and step 8 deleted the indexer's.)
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
    // TODO: robotContainer.setDriveBrakeMode(true).
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
