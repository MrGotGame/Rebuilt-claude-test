package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/**
 * Physics simulation of the intake, good enough to develop and debug the state machine with no
 * hardware present.
 *
 * <p>The pivot is a {@link SingleJointedArmSim} so that gravity, the hard stops, and the gravity
 * feedforward all behave the way they will on the real robot. The roller is a {@link DCMotorSim}.
 * Both are driven by a WPILib {@link ProfiledPIDController} standing in for the Talon's Motion
 * Magic.
 */
public class IntakeIOSim implements IntakeIO {
  private final SingleJointedArmSim pivotSim;
  private final DCMotorSim rollerSim;
  private final ProfiledPIDController pivotController;
  private boolean pivotClosedLoop = false;
  private double pivotAppliedVolts = 0.0;
  private double rollerAppliedVolts = 0.0;
  private double rollerSetpointRpm = 0.0;

  /**
   * Stand-in for the roller Talon's velocity loop.
   *
   * <p>Added beyond the skeleton's field list. The skeleton's TODO computed the roller voltage
   * inside {@code setRollerVelocity}, which ties the control loop's rate to how often the state
   * machine happens to call it; running it here, once per {@code updateInputs}, matches both the
   * real Talon and {@code FloorRollerIOSim}.
   */
  private final PIDController rollerController = new PIDController(0.0, 0.0, 0.0);

  private boolean rollerClosedLoop = false;
  private double rollerFfVolts = 0.0;

  /**
   * Whether the simulated absolute encoder reports itself as plugged in.
   *
   * <p>Injectable, in the spirit of plan section 9.2: an encoder that cannot fail validates nothing
   * about the code that handles it failing. With this true -- the default, and what the real
   * ThroughBore does when healthy -- {@code Intake} trusts position directly and never needs to
   * home. Setting it false is the only way to exercise the stall-homing fallback, its settle
   * detection and its timeout into FAULTED, all of which are otherwise dead code until the day the
   * encoder is unplugged at an event.
   */
  private boolean absEncoderConnected = true;

  /**
   * Builds the two simulation models and the stand-in profiled controller.
   *
   * <p>NOTE: these fields are final, so construction is pre-filled. The angle bounds below are
   * deliberate placeholders rather than the Constants values, because every Intake angle constant
   * currently returns 0.0 and {@link SingleJointedArmSim} rejects a zero-width range.
   */
  public IntakeIOSim() {
    // TODO: MEASURE. Once kStowedAngleRad and kDeployedAngleRad are measured, replace the two
    // placeholder limits below with Constants.Intake.kStowedAngleRad.get() and
    // kDeployedAngleRad.get() so the simulation enforces the same travel as the real mechanism.
    // The TODO is kept rather than implemented because SingleJointedArmSim rejects a zero-width
    // range and all three angle constants are still 0.0: wiring them through today would throw at
    // construction in every simulation run.
    pivotSim =
        new SingleJointedArmSim(
            // IMPLEMENTATION (was TODO: "VERIFY whether WPILib 2026 exposes a getKrakenX44()
            // factory. If it does, use it. If not, build a DCMotor from the X44 datasheet stall
            // torque and free speed rather than leaving the X60 model here -- the X44 is a
            // materially weaker motor and the sim will otherwise flatter your gains.")
            //
            // VERIFIED: it does. `javap` on wpimath-java-2026.2.1.jar lists getKrakenX44(int) and
            // getKrakenX44Foc(int) on edu.wpi.first.math.system.plant.DCMotor, so no hand-built
            // DCMotor is needed.
            //
            // The non-FOC variant, because every request IntakeIOTalonFX issues is a Voltage
            // request -- MotionMagicVoltage, VelocityVoltage, VoltageOut. The FOC curves model
            // roughly fifteen percent more torque than this code ever asks the motor for, and on
            // the one mechanism plan section 9.2 rates high fidelity, an optimistic motor is
            // exactly the thing that makes a gravity feedforward look adequate in sim and sag on
            // the robot.
            DCMotor.getKrakenX44(1),
            Constants.Intake.kPivotGearRatio,
            Constants.Intake.kPivotMoiKgM2,
            Constants.Intake.kPivotLengthMeters,
            -Math.PI / 2.0,
            Math.PI / 2.0,
            true,
            0.0);
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                // The inertia the skeleton hard-coded here now lives in Constants, carrying its
                // "TODO: from CAD" with it, so that every guessed-but-tunable number in the project
                // is in the one file anybody sweeping for guesses will open.
                Constants.Intake.kRollerMoiKgM2,
                // Was Constants.FloorRoller.kGearRatio in the skeleton, which is a different
                // mechanism's gearbox. Constants.Intake.kRollerGearRatio was added for M5.
                Constants.Intake.kRollerGearRatio),
            DCMotor.getKrakenX60Foc(1));

    // IMPLEMENTATION (was TODO: "Seed the controller constraints from kPivotCruiseRadPerSec /
    // kPivotAccelRadPerSec2 rather than the literals above, so tuning the real robot also tunes the
    // sim.")
    //
    // The constraints are converted to ROTATIONS per second and per second squared, because this
    // controller runs in rotations -- see updateInputs for why. They are latched at construction:
    // re-tuning cruise or acceleration on the dashboard needs a restart, in simulation and on the
    // robot alike, because MotionMagicConfigs is likewise only applied in IntakeIOTalonFX's
    // constructor.
    pivotController =
        new ProfiledPIDController(
            0.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                Units.radiansToRotations(Constants.Intake.kPivotCruiseRadPerSec.get()),
                Units.radiansToRotations(Constants.Intake.kPivotAccelRadPerSec2.get())));
  }

  /**
   * Advances both simulation models one loop period and fills the input struct.
   *
   * @param inputs the struct to populate
   */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // IMPLEMENTATION (was TODO: "When pivotClosedLoop, run pivotController.calculate(
    // pivotSim.getAngleRads()) and use it as pivotAppliedVolts. Clamp both applied voltages to
    // +-12 V, call setInputVoltage on each sim and update(Constants.getLoopPeriodSec()), then fill
    // EVERY input field. Set pivotConnected, rollerConnected and absEncoderConnected to true, and
    // set pivotAbsolutePositionRad equal to pivotPositionRad -- the simulated encoder is perfect,
    // which is exactly why sim will not catch a bad kPivotAbsEncoderOffsetRot.")
    //
    // DEVIATION FROM THE TODO, deliberate, and the same one FloorRollerIOSim makes: both loops run
    // in ROTATIONS, not radians or RPM. Slot0Configs.kP on the real Talon is volts per ROTATION of
    // error (or per rotation-per-second, for the velocity loop). Running the simulated controller
    // in radians would make the same numeric kP 2*pi times weaker here than on hardware, so a gain
    // that looked right in the only place it can be tried before the robot exists would arrive on
    // the robot six times too soft. The radian conversions happen only at the input-struct
    // boundary, where the rest of the project starts reading.
    if (pivotClosedLoop) {
      double measuredRot = Units.radiansToRotations(pivotSim.getAngleRads());

      // The gravity feedforward is applied here as well as on the real robot, and this is the whole
      // reason plan section 9.2 rates this mechanism high fidelity. SingleJointedArmSim measures
      // its angle from horizontal with positive raising the arm, which is the convention
      // Constants.Intake.kPivotKg documents, so a kG of the wrong sign drives the simulated arm
      // into a hard stop here exactly the way it would drive the real one into its own. Leaving the
      // term out of the sim would make the single most damaging mistake this mechanism can make
      // invisible until it happened on hardware.
      double gravityVolts = Constants.Intake.kPivotKg.get() * Math.cos(pivotSim.getAngleRads());
      pivotAppliedVolts = gravityVolts + pivotController.calculate(measuredRot);
    }

    if (rollerClosedLoop) {
      double rollerRotPerSec =
          Units.radiansPerSecondToRotationsPerMinute(rollerSim.getAngularVelocityRadPerSec())
              / 60.0;
      rollerAppliedVolts = rollerFfVolts + rollerController.calculate(rollerRotPerSec);
    } else {
      // Resetting while open loop clears the integrator. Without this, a characterization sweep
      // followed by a closed-loop request would start with whatever integral had accumulated before
      // the mode switch and deliver a step to the roller.
      rollerController.reset();
    }

    // The clamped values are what the models are driven with AND what gets reported, so that
    // saturation is visible in the log. Reporting the raw request instead would show a 20 V trace
    // no Talon can produce, and the first question anyone debugging a sluggish simulated pivot has
    // is whether the controller is simply asking for more than the battery can give.
    double pivotVolts = MathUtil.clamp(pivotAppliedVolts, -12.0, 12.0);
    double rollerVolts = MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0);
    pivotSim.setInputVoltage(pivotVolts);
    rollerSim.setInputVoltage(rollerVolts);
    pivotSim.update(Constants.getLoopPeriodSec());
    rollerSim.update(Constants.getLoopPeriodSec());

    inputs.pivotConnected = true;
    inputs.rollerConnected = true;

    // absEncoderConnected is injectable rather than hard-coded true -- see the field's
    // documentation. When it is false the absolute reading is reported as 0.0 rather than as the
    // true angle, because that is what a DutyCycleEncoder with nothing plugged into its DIO
    // actually returns: it does not fail, it confidently reports zero. Code that reads the value
    // without checking the flag first therefore gets a plausible wrong number in simulation, just
    // as it would on the robot.
    inputs.absEncoderConnected = absEncoderConnected;
    inputs.pivotPositionRad = pivotSim.getAngleRads();
    inputs.pivotAbsolutePositionRad = absEncoderConnected ? pivotSim.getAngleRads() : 0.0;
    inputs.pivotVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
    inputs.pivotAppliedVolts = pivotVolts;

    // Stalling the pivot against a hard stop is a deliberate part of homing, so this current is not
    // decoration: it is the signal Intake.homing() watches to tell "pressed against the stop" apart
    // from "not moving because nothing is driving me". SingleJointedArmSim clamps the arm at its
    // limits while the motor keeps drawing, which is what makes that detection testable here.
    double pivotStatorAmps = Math.abs(pivotSim.getCurrentDrawAmps());
    inputs.pivotStatorCurrentAmps = pivotStatorAmps;

    // A motor controller is a buck converter: supply current is roughly stator current scaled by
    // duty cycle. The approximation is crude, but reporting supply == stator would overstate the
    // battery load by up to six times at low speed and make any sim-side power budgeting
    // meaningless.
    inputs.pivotSupplyCurrentAmps = pivotStatorAmps * Math.abs(pivotVolts) / 12.0;

    // Neither sim class has a thermal model. A fixed room temperature is reported rather than 0.0
    // so that a plot of this field in simulation is obviously constant rather than looking like a
    // sensor reading zero -- which is what a genuinely disconnected Talon reports.
    inputs.pivotTempCelsius = 25.0;

    inputs.rollerVelocityRpm =
        Units.radiansPerSecondToRotationsPerMinute(rollerSim.getAngularVelocityRadPerSec());
    inputs.rollerAppliedVolts = rollerVolts;
    inputs.rollerStatorCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
    inputs.rollerTempCelsius = 25.0;
  }

  /**
   * Commands the simulated pivot to an angle.
   *
   * @param positionRad target angle, in radians
   */
  @Override
  public void setPivotPosition(double positionRad) {
    // IMPLEMENTATION (was TODO: "pivotClosedLoop = true; pivotController.setGoal(positionRad).")
    //
    // The profile is reset on the OPEN-to-CLOSED transition, which the TODO did not ask for and
    // which matters on this mechanism specifically. A ProfiledPIDController remembers the setpoint
    // it reached last time it ran; the loop before this one was homing, which drove the arm open
    // loop to somewhere else entirely. Without the reset the first closed-loop command would
    // generate a profile starting from a stale position -- a step of the whole travel, at full
    // Motion Magic acceleration, immediately after homing. The reset is in rotations, and takes the
    // measured velocity too so a profile begun mid-motion is continuous.
    if (!pivotClosedLoop) {
      pivotController.reset(
          Units.radiansToRotations(pivotSim.getAngleRads()),
          Units.radiansToRotations(pivotSim.getVelocityRadPerSec()));
    }
    pivotClosedLoop = true;
    pivotController.setGoal(Units.radiansToRotations(positionRad));
  }

  /**
   * Drives the simulated pivot open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setPivotOpenLoop(double volts) {
    // IMPLEMENTATION (was TODO: "pivotClosedLoop = false; pivotAppliedVolts = volts.")
    //
    // No gravity term is added here, matching IntakeIOTalonFX.setPivotOpenLoop: homing and
    // characterization both need to know what a known voltage does to the mechanism.
    pivotClosedLoop = false;
    pivotAppliedVolts = volts;
  }

  /**
   * Commands the simulated roller to a speed.
   *
   * @param velocityRpm target speed, in RPM
   */
  @Override
  public void setRollerVelocity(double velocityRpm) {
    // IMPLEMENTATION (was TODO: "Store the setpoint, then drive rollerAppliedVolts with a simple kV
    // feedforward plus a proportional term on the RPM error.")
    //
    // The feedforward is computed here and the proportional term in updateInputs, so the loop runs
    // once per simulation step rather than once per call. The setpoint is in rotations per second
    // for the unit reason given in updateInputs.
    //
    // Math.signum on the kS term is not decoration on this mechanism: EJECT commands a NEGATIVE
    // RPM, and a fixed +kS would push against the very motion being asked for -- a bug that hides
    // completely on a flywheel, which only ever spins one way.
    rollerClosedLoop = true;
    rollerSetpointRpm = velocityRpm;
    rollerFfVolts =
        Constants.Intake.kRollerKs.get() * Math.signum(velocityRpm)
            + Constants.Intake.kRollerKv.get() * velocityRpm / 60.0;
    rollerController.setP(Constants.Intake.kRollerKp.get());
    rollerController.setSetpoint(velocityRpm / 60.0);
  }

  /**
   * Drives the simulated roller open loop.
   *
   * @param volts commanded voltage
   */
  @Override
  public void setRollerOpenLoop(double volts) {
    // IMPLEMENTATION (was TODO: "rollerAppliedVolts = volts.")
    //
    // The feedforward and the setpoint are cleared as well. rollerFfVolts is added unconditionally
    // whenever the closed loop runs, so leaving a stale one standing would offset every later
    // command by whatever the last closed-loop request happened to need.
    rollerClosedLoop = false;
    rollerFfVolts = 0.0;
    rollerSetpointRpm = 0.0;
    rollerAppliedVolts = volts;
  }

  /**
   * Teleports the simulated pivot to a known angle.
   *
   * @param positionRad the angle to assume, in radians
   */
  @Override
  public void seedPivotPosition(double positionRad) {
    // IMPLEMENTATION (was TODO: "pivotSim.setState(positionRad, 0.0).")
    //
    // Worth knowing when reading a simulated homing sequence: on the real robot seeding moves only
    // the SENSOR, leaving the arm exactly where it is. In simulation the sensor and the plant are
    // the same object, so the arm visibly teleports. A jump in Intake/PivotAngleRad at the end of a
    // simulated homing run is therefore expected, and is not a sign the homing logic misfired.
    pivotSim.setState(positionRad, 0.0);
  }

  /**
   * Applies gains to the stand-in controller.
   *
   * @param gains the gain set; only kP, kI and kD are meaningful in simulation
   */
  @Override
  public void setPivotGains(Slot0Configs gains) {
    // IMPLEMENTATION (was TODO: "pivotController.setPID(gains.kP, gains.kI, gains.kD). Note that
    // sim gains are NOT real gains: this controller is a WPILib profiled PID in volts, while the
    // robot runs Motion Magic on the Talon. Tuning kP here does not transfer.")
    //
    // The warning in the TODO is true of the loop's SHAPE and stays true: Motion Magic's internal
    // controller is not this one. Its UNITS, though, have been made to match -- this controller
    // runs in rotations, so a kP here means the same volts per rotation of error that it means on
    // the Talon. Expect the right order of magnitude to transfer, not the exact number.
    //
    // kS, kV and kG are ignored here, as WPILib's controllers have no feedforward slot; kG is read
    // straight from Constants in updateInputs, where the cosine can be applied.
    pivotController.setPID(gains.kP, gains.kI, gains.kD);
  }

  /** Zeroes both applied voltages and leaves closed loop. */
  @Override
  public void stop() {
    // IMPLEMENTATION (was TODO: "pivotAppliedVolts = 0; rollerAppliedVolts = 0; pivotClosedLoop =
    // false.")
    //
    // Zero volts is a coast in simulation, for both mechanisms. That is right for the roller, which
    // the real IO configures to coast, but it is a known LIMITATION for the pivot: the real one is
    // configured to Brake and holds itself roughly in place, while the simulated arm falls under
    // gravity to its lower stop. Do not read a simulated pivot dropping on FAULTED as a bug in the
    // state machine; SingleJointedArmSim has no brake to model.
    pivotClosedLoop = false;
    rollerClosedLoop = false;
    rollerFfVolts = 0.0;
    rollerSetpointRpm = 0.0;
    pivotAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;
  }

  /**
   * Injects an absolute-encoder failure, so the stall-homing fallback can be exercised in
   * simulation.
   *
   * <p>Not part of {@link IntakeIO}: like the indexer's injected piece detection in plan section
   * 9.2, this is a simulation-only hook, reached from a simulation keyboard binding or a test
   * rather than from robot code. With a perfect encoder the fallback never runs, so without
   * something like this the homing sequence, its settle detection and its timeout into FAULTED are
   * first exercised at an event with a broken robot.
   *
   * @param connected false to make the simulated ThroughBore report itself unplugged
   *     <p><b>Outputs:</b> changes what {@code inputs.absEncoderConnected} and {@code
   *     inputs.pivotAbsolutePositionRad} report from the next {@link #updateInputs} onward.
   */
  public void setAbsEncoderConnected(boolean connected) {
    absEncoderConnected = connected;
  }

  /**
   * The roller speed the closed loop is currently chasing.
   *
   * <p>Exists so the field the skeleton declared is readable rather than write-only; useful from a
   * simulation test asserting that the state machine commanded what it was meant to.
   *
   * @return the commanded roller speed, in RPM, or zero when the roller is open loop or stopped
   */
  public double getRollerSetpointRpm() {
    return rollerSetpointRpm;
  }
}
