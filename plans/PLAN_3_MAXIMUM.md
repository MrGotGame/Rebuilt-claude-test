# Tier 3 — Maximum Sophistication — FRC 2026 REBUILT Robot Code Plan

> **Grounding note.** This plan was written against the actual contents of
> `AdvantageKit_TalonFXSwerveTemplate` (stock AdvantageKit 2026 TalonFX Swerve Template,
> AdvantageKit 26.0.2 / Phoenix6 26.1.2 / PathplannerLib 2026.1.2 / questnavlib 2026-3.0.0-beta /
> Studica 2026.0.0, Java 17, spotless+google-java-format on every `compileJava`).
>
> The architecture mirrors FRC Team 2910's `2026CompetitionRobot-Public`. I fetched that repository's
> file tree and read `SuperStructure.java`, `RobotState.java`, `IntakeSubsystem.java`,
> `ShooterInterpolation.java` and `LoggedTunableNumber.java` directly, so the FSM idiom below
> (`WantedState` / `SystemState`, `handleStateTransition()` → `applyState()` → `setWantedState()`,
> a plain-object `RobotState`, a `SuperStructure extends SubsystemBase` that owns cross-mechanism
> sequencing) is copied from the real source, not from memory. Two places where I deliberately
> **diverge** from 2910 are called out and justified in §1.
>
> **Game-knowledge discipline.** Everything that depends on the REBUILT field or game piece
> (goal location, goal height, piece mass/diameter, starting poses, scoring zones) appears in this
> plan only as a *named tunable constant with a `TODO`* telling the student where to measure it.
> No REBUILT dimension is asserted anywhere in this document.

---

## 1. Design Philosophy

**Who this is for.** Three to five experienced student programmers (each has written a working
subsystem before, can read a stack trace, understands PID and units) plus a mentor who can review
pull requests and arbitrate design arguments. This is a *team* codebase, not one student's project.
At least one student must be comfortable with Git branches and one must own logging/replay.

**What it optimizes.** Ceiling, then maintainability, then time-to-first-drive — in that order.
The design assumes the robot must be able to *fire accurately while translating at full speed*,
because a turreted shooter that only works standing still throws away most of the reason to build
a turret. Every layer below exists to make shoot-on-the-move actually land:

| Layer | Concrete capability it buys |
| --- | --- |
| `RobotState` with time-interpolated pose/velocity/turret buffers | Lets vision measurements be applied at their true capture timestamp, and lets the turret camera's robot→camera transform be evaluated at that same timestamp. Without it, every vision fix is ~60–120 ms stale and the turret-mounted camera is simply wrong. |
| 250 Hz odometry (already in the template) | Interpolated pose at an arbitrary vision timestamp is only as good as the odometry sample rate. |
| Iterative time-of-flight shooting solver | Converts "aim at the goal" into "aim at where the goal will be relative to a moving robot when the piece arrives." This is the difference between shooting on the move and stopping to shoot. |
| Turret velocity feedforward from chassis yaw rate | The turret holds the target while the driver spins the robot; without it the turret lags and `readyToShoot` never latches during a rotate-and-shoot. |
| Explicit turret unwrap with finite travel limits | A turret with hard stops *will* try to cross them. Unwrap logic converts "impossible aim" into "rotate the drivetrain" instead of a snapped belt. |
| Dual-source fusion (MegaTag2 + QuestNav) with per-source covariance | QuestNav gives centimeter-class, 100 Hz relative pose with no tag in view; MegaTag2 gives absolute field truth. Fused, you keep aiming when your only tag is occluded by a defender. |
| Superstructure FSM with typed goals | One place decides "is it legal to swing the intake arm through the turret's sweep right now." Ad-hoc command compositions cannot enforce that. |
| Tunable constants everywhere | Shot maps get retuned between every match on a practice field. Redeploying takes 45 s; a tunable takes 0 s. |
| `Alert` fault system + power budget | A disconnected CANcoder discovered in the pits is a fixed robot; discovered on the field it is a lost match. |

**Explicit trade-offs accepted.**

1. **Nothing works until several things work.** The shooter cannot be tested without `RobotState`,
   which cannot be trusted without vision. Milestone ordering in §10 exists specifically to give
   testable checkpoints before that dependency chain closes; ignore it at your peril.
2. **More surface area to get wrong.** ~45 source files, ~330 methods. A rookie cannot navigate
   this alone. Code review is mandatory, not optional.
3. **Log volume.** This plan logs aggressively. WPILOG files will be 5–20 MB/match. Budget a
   dedicated USB stick and delete logs weekly.
4. **Two divergences from 2910, on purpose:**
   - *We keep the AdvantageKit `Drive`/`Module`/`ModuleIO` stack rather than 2910's
     `SwerveIOCTRE` wrapper around CTRE's `CommandSwerveDrivetrain`.* Reason: AdvantageKit log
     replay is the single highest-leverage debugging tool available to a student team, and the
     template's `PhoenixOdometryThread` + per-module `@AutoLog` inputs make swerve fully replayable.
     CTRE's `SwerveDrivetrain` owns its own odometry thread and its internals are not replayable.
     The CTRE-generated file (`TunerConstants.java`) is still the source of truth for module
     geometry, gearing and gains — see §5.
   - *We keep per-subsystem `XxxIO` interfaces rather than 2910's generic
     `ServoMotorSubsystem`/`RollerMotorSubsystem` base classes.* Reason: AdvantageKit's `@AutoLog`
     annotation processor generates one `XxxIOInputsAutoLogged` per annotated class, and
     per-subsystem input structs give you readable, replayable AdvantageScope keys
     (`Turret/Inputs/positionRad`) instead of a generic blob. The duplication is real (~6 IO
     implementations look similar) and is the price of replay fidelity. Say this out loud in your
     design review so nobody "cleans it up" in week 4.

**Assumed student skill level.** Java generics, records, enums with fields, lambdas, and
`Optional` are assumed known. Threading is *not* assumed known — this plan confines all shared
mutable state to one lock in `RobotState` and one in the existing `PhoenixOdometryThread`, and
says exactly which thread touches what.

**Rough size.** ~7,000–8,500 lines of Java across ~45 files (excluding the regenerated
`TunerConstants.java` and generated `*AutoLogged` classes), plus ~400 lines of JUnit tests.

**Rough calendar.** With 3–4 students working in parallel per §10:

| Checkpoint | Weeks from kickoff |
| --- | --- |
| Robot drives, 250 Hz odometry logged, AdvantageScope replay works | 1.5 |
| All six mechanisms move under closed loop in sim and on the robot | 4 |
| Vision fused, `RobotState` pose trusted to ±5 cm | 5.5 |
| Stationary shooting reliable | 7 |
| Shoot-on-the-move converged and tuned | 9 |
| Eight auto routines, auto-shoot-on-move in auto | 11 |
| Competition hardening (faults, power budget, fallbacks) | 12–14 |

This is a build-season-long plan. If your first competition is in week 6, build Tier 2.

---

## 2. Architecture Overview

### 2.1 The FSM model

Every mechanism and the superstructure implement the same three-part contract, copied from 2910:

```java
public enum WantedState { ... }   // what someone ASKED for  (external, set by Superstructure/driver)
public enum SystemState { ... }   // what the mechanism IS DOING (internal, authoritative)

private SystemState handleStateTransition();          // pure-ish: returns the next SystemState
private void applyState();                            // side-effecting: drives the IO layer
public void setWantedState(WantedState wanted);       // the only external mutator
public Command setWantedStateCommand(WantedState w);  // command-based adapter for button bindings
```

`periodic()` always runs the same five steps, in this order:

```
1. io.updateInputs(inputs);  Logger.processInputs("<Name>", inputs);
2. SystemState next = handleStateTransition();
3. if (next != systemState) { onExit(systemState); stateTimer.restart(); onEnter(next); systemState = next; }
4. applyState();
5. Logger.recordOutput("<Name>/WantedState", wantedState);
   Logger.recordOutput("<Name>/SystemState", systemState);
   Logger.recordOutput("<Name>/StateTime", stateTimer.get());
```

Four properties this buys you, all of which are requirements for this tier:

* **Entry/exit actions.** Step 3 fires `onEnter`/`onExit` exactly once per transition. Use it to
  reset profiles, zero integrators, and latch setpoints.
* **State timeouts.** `stateTimer` is reset on every transition, so `handleStateTransition()` can
  say "if I have been `HOMING` for more than `kHomingTimeoutSec`, go to `FAULTED`."
* **Guarded transitions.** `handleStateTransition()` is a `switch (wantedState)` whose arms contain
  guards (`isHomed`, `lastShot.isUsable()`, `Superstructure.allowsDeploy()`). Illegal requests silently
  resolve to a safe state instead of throwing.
* **Illegal transitions unrepresentable where reasonable.** The cross-mechanism illegal
  combinations (intake deployed *and* turret swept through the intake's arc) live in exactly one
  place: `Superstructure.applyState()`. No subsystem can be commanded into them from a button
  binding, because buttons bind to `Superstructure.setWantedStateCommand(...)`, never to a
  subsystem directly. (SysId/characterization bindings are the one exception and are gated behind
  `Constants.kTuningMode`.)

### 2.2 Worked example — driver presses right bumper ("score on the move")

```
Driver holds RIGHT BUMPER
  └─ RobotContainer.configureButtonBindings()
       driver.rightBumper()
         .whileTrue(superstructure.setWantedStateCommand(Superstructure.WantedState.SCORE_ON_MOVE))
         .onFalse(superstructure.setWantedStateCommand(Superstructure.WantedState.DEFAULT));

Superstructure.setWantedState(SCORE_ON_MOVE)        // just stores the enum; no work here
  ...20 ms later, CommandScheduler runs subsystem periodic() in registration order...

Drive.periodic()
  ├─ gyroIO.updateInputs(...) ; module[i].periodic()             // 250 Hz samples drained here
  └─ RobotState.getInstance().addOdometryObservation(t[i], gyroYaw[i], modulePositions[i])
        └─ poseEstimator.updateWithTime(...) ; poseBuffer.addSample(t, pose)

Turret.periodic()
  └─ RobotState.getInstance().addTurretObservation(Timer.getFPGATimestamp(), inputs.positionRad)

Vision.periodic()
  ├─ VisionIOLimelight("limelight-turret").updateInputs(...)      // MegaTag2 pose + timestamp
  ├─ Vision.robotToTurretCameraAt(obsTimestamp)                   // uses RobotState turret buffer
  └─ RobotState.getInstance().addVisionObservation(pose, t, stdDevs, Source.LIMELIGHT_TURRET)

Superstructure.periodic()
  ├─ handleStateTransition():  SCORE_ON_MOVE -> SystemState.SCORING_ON_THE_MOVE
  │      (guards: no critical fault, else SPINNING_UP; usable shot solution, else WAITING_FOR_TURRET.
  │       There is deliberately no "has a piece" guard — see §2.4)
  ├─ resolveIntakeMode():  intake not held, posture DEPLOYED -> IntakeMode.FEED_ASSIST once the
  │       feed gate opens (intake held -> INTAKING instead: shoot, move and intake at once)
  ├─ onEnter(SCORING_ON_THE_MOVE): shotSolver.reset(); shootTimer.restart();
  └─ applyState() -> scoringOnTheMove():
        Pose2d       p  = RobotState.getInstance().getEstimatedPose();
        ChassisSpeeds v = RobotState.getInstance().getFieldRelativeSpeeds();
        Translation2d a = RobotState.getInstance().getFieldRelativeAccel();
        ShotParameters shot = shotSolver.solve(p, v, a, Constants.Field.goalTranslation());
        turret.setGoal(shot.turretFieldAngle(), shot.turretAngularVelocityRadPerSec());
        shooter.setGoal(shot.flywheelRpm());
        if (isReadyToScore(shot)) indexer.setWantedState(Indexer.WantedState.FEED);
        else                      indexer.setWantedState(Indexer.WantedState.IDLE);
        // FEED streams the whole hopper; the burst latch keeps it open between pieces (§2.4).

Turret.periodic()   (next loop)
  ├─ handleStateTransition(): wantedState == AIM (set by Turret.setGoal) -> SystemState.AIMING
  └─ applyState() -> aiming():
        double robotRel = goalFieldAngleRad - RobotState.getInstance().getEstimatedPose()
                                                 .getRotation().getRadians();
        TurretUnwrapper.Result r = TurretUnwrapper.solve(
              inputs.positionRad, robotRel, Constants.Turret.kMinAngleRad, Constants.Turret.kMaxAngleRad);
        double ffRadPerSec = goalAngularVelocityRadPerSec
                             - RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
        io.setPositionSetpoint(r.targetPositionRad(), ffRadPerSec);

TurretIOTalonFX.setPositionSetpoint(double, double)
  └─ talon.setControl(motionMagicVoltage
            .withPosition(Units.radiansToRotations(pos) * kGearRatio)
            .withFeedForward(kV * Units.radiansToRotations(vel) * kGearRatio));
                                          // -> Kraken X44 turret motor spins
```

The same trace for the flywheel ends at
`ShooterIOTalonFX.setVelocity(double rpm, double feedforwardVolts)` →
`leader.setControl(velocityVoltage.withVelocity(rpm/60.0))` with the follower already bound via
`follower.setControl(new Follower(leaderId, opposeDirection))`.

### 2.3 ASCII control-flow diagram

```
                     ┌──────────────────────────────────────────────────────────┐
  Driver / Operator  │  RobotContainer.configureButtonBindings()                 │
  ──────────────────▶│  buttons -> superstructure.setWantedStateCommand(...)     │
                     └───────────────────────────┬──────────────────────────────┘
                                                 │ WantedState (enum only)
                                                 ▼
 ┌───────────────────────────────────────────────────────────────────────────────────────┐
 │ Superstructure extends SubsystemBase                                                  │
 │   periodic(): handleStateTransition() -> onExit/onEnter -> applyState() -> log        │
 │   SystemState: IDLE  HOMING  RESTING  SPINNING_UP  WAITING_FOR_TURRET  EJECTING       │
 │                SCORING_ON_THE_MOVE  SCORING_STATIONARY  SCORING_FORCED  PASSING       │
 │                CHARACTERIZING  FAULTED   (+ CLIMB_SAFE hole)                          │
 │   IntakeMode (orthogonal, §2.4): STOWED  DEPLOYED  INTAKING  FEED_ASSIST              │
 └───┬───────────┬───────────┬────────────┬────────────┬────────────┬────────────────────┘
     │           │           │            │            │            │
     ▼           ▼           ▼            ▼            ▼            ▼
 ┌────────┐ ┌──────────┐ ┌─────────┐ ┌────────┐  ┌─────────┐  ┌──────────────┐
 │ Intake │ │FloorRoll.│ │ Indexer │ │ Turret │  │ Shooter │  │ Drive        │
 │  FSM   │ │   FSM    │ │   FSM   │ │  FSM   │  │   FSM   │  │ (no FSM:     │
 │        │ │          │ │         │ │        │  │         │  │  velocity in)│
 └───┬────┘ └────┬─────┘ └────┬────┘ └───┬────┘  └────┬────┘  └──────┬───────┘
     │IO         │IO          │IO        │IO          │IO            │IO
     ▼           ▼            ▼          ▼            ▼              ▼
 ╔═══════════════════════════════════════════════════════════════════════════════╗
 ║  XxxIO interface  ->  XxxIOTalonFX (REAL) | XxxIOSim (SIM) | XxxIO {} (REPLAY) ║
 ║  selected by switch (Constants.currentMode) in RobotContainer                  ║
 ╚═══════════════════════════════════════════════════════════════════════════════╝
     │                                     ▲                          │
     │ @AutoLog inputs                     │ reads                    │ 250 Hz odom samples
     ▼                                     │                          ▼
 ┌───────────────────────────────────────────────────────────────────────────────┐
 │ AdvantageKit Logger  ──▶ WPILOG (/U/logs) + NT4 ──▶ AdvantageScope             │
 └───────────────────────────────────────────────────────────────────────────────┘
                                           │
     ┌─────────────────────────────────────┴──────────────────────────────────────┐
     │ RobotState  (singleton, ReentrantReadWriteLock)                             │
     │   SwerveDrivePoseEstimator           <- odometry @250 Hz, vision @ ~30 Hz   │
     │   TimeInterpolatableBuffer<Pose2d>   <- pose history, 2.0 s                 │
     │   TimeInterpolatableBuffer<Double>   <- turret angle history, 2.0 s         │
     │   field-relative ChassisSpeeds + LinearFilter-differentiated accel          │
     │   per-source confidence + staleness                                         │
     └───────────────────────▲──────────────────────────────▲─────────────────────┘
                             │                              │
              ┌──────────────┴──────────────┐   ┌───────────┴──────────────┐
              │ Vision (2x Limelight 4)     │   │ QuestNavIO (Quest 3S)    │
              │  MegaTag2 botpose + stddev  │   │  Pose3d @ ~100 Hz        │
              │  turret cam transform(t)    │   │  seeded / re-seeded      │
              └─────────────────────────────┘   └──────────────────────────┘
                             │
                             ▼
              ┌──────────────────────────────────────────┐
              │ ShootingSolver (pure, unit-tested)       │
              │   iterate: gv = g - v*t - 0.5*a*t^2      │
              │   -> ShotParameters(angle, omega, rpm,   │
              │                     tof, valid)          │
              └──────────────────────────────────────────┘
```

### 2.4 Game pieces: a hopper, not a slot

> **Replanned 2026-09-24.** This section supersedes the single-piece model the original M5 and M10
> specifications were written against. `feature-indexer` and `feature-superstructure` implement
> the *old* model; milestone **M10b** (§10.2) retrofits them. Wherever the rest of this document
> disagrees with this section, this section wins. Every other section has been updated to match it.

**The rule that forced the change.** REBUILT lets a robot carry as many game pieces as it can
physically hold. The original plan modelled the robot as a one-piece slot: `Indexer.hasPiece()` was
a single debounced boolean, and three transitions pivoted on it — `INTAKING → INDEXING` the moment
the first piece arrived, every `SCORE_*` request diverting to `SPINNING_UP` without one, and
`scoreUntilEmptyCommand()` ending when it fell. With a hopper, every one of those is wrong:

| Single-piece assumption | What actually happens with a hopper |
| --- | --- |
| Intaking ends when the first piece is detected | The driver wants to keep collecting. Ending on the first piece leaves the rest on the carpet. |
| "Has a piece" is a meaningful yes/no | The hopper holds anywhere from zero to a full load. A current-and-velocity reading at the indexer can only see the piece touching the indexer wheel, not the hopper behind it. |
| One piece per feed; a shot is the falling edge of `hasPiece` | Pieces stream back to back. The indexer is never unloaded between them, so there is no falling edge, and the old counter would count a whole burst as one shot. |
| Stowing the intake after a pickup is safe | Raising the intake lets pieces fall back out of the robot. |

#### Sensing decision: no dedicated piece sensors

The robot has no beam break, time-of-flight or other piece sensor, and this replan does not add one.
The consequence has to be stated plainly, because it drives everything else: **the robot cannot know
how many pieces it holds, and cannot reliably know that it holds any.** So the design stops asking.
Three signals remain, in decreasing order of trust:

1. **Shots fired — `Shooter`.** A piece passing through the flywheel takes a sharp, measurable bite
   out of wheel speed. That is the most reliable piece signal on the robot: the energy a piece
   removes is large compared with loop noise, and it happens at the one place every scored piece
   must pass. **Shot counting moves from `Indexer` to `Shooter`.**
2. **Indexer load — `Indexer.isLoaded()`.** Stator current above a threshold, evaluated *only
   while feeding* and only once the spin-up settle window has passed. (The old piece detector also
   required a velocity shortfall, to reject the spin-up current spike; the settle window does that
   now, and a shortfall requirement would miss a stiff loop that pushes pieces at full speed, so
   M10b step 6 dropped it.) It answers "is the indexer pushing against something right now",
   which is what jam detection needs, and it corroborates signal 3. It is logged but gates nothing.
3. **`Superstructure.likelyEmpty()` — a belief, not a measurement.** True once the robot has spent
   `kEmptyConfirmSec` of *gate-open, unloaded* feeding with no shot detected. Autos use it to end a
   scoring leg early and the driver controller rumbles on it. **It never gates a teleop shot.**

The time only accumulates while the feed gate is open *and* the indexer is feeding unloaded. Time
spent with the gate closed (waiting on the turret, say) says nothing about the hopper, and counting
it would end an auto's scoring leg while the hopper is still full. That distinction is the first
thing M10b's mutation run should attack.

#### Nothing is gated on having a piece any more

The readiness gate loses its piece term and becomes **seven** terms (§6.5). A scoring request enters
its scoring state whether or not the hopper has anything in it. Feeding an empty hopper just spins
the indexer for nothing, which costs a few watts. The old reason for the term was "feed would … desync
the shot counter", and that reason goes away now that the counter watches the flywheel, not the indexer.

This is also what makes shoot-while-intaking fall out naturally. If the driver holds the score
trigger with an empty robot and then drives into pieces, they are fired as they arrive. No state
needs to notice that the hopper went from empty to not-empty.

#### Intake posture: down stays down

* **Holding INTAKE** deploys the intake and runs the roller. **Releasing it stops the roller and
  leaves the intake deployed** (`Intake.WantedState.DEPLOY_ONLY`), because raising it dumps the
  hopper.
* **The intake stows only when explicitly asked**: a new `Superstructure.WantedState.STOW` on an
  operator button, plus `HOME`. Stowing mid-match is allowed but logged as a warning, because
  **pieces can fall out of the robot while the intake is stowed**. At the partial-raise height
  they stay in.
* The Superstructure remembers the choice in an `intakePosture` field (`STOWED` / `DEPLOYED`), and
  `DEFAULT` honours it instead of stowing. The old `STOWED` system state is renamed **`RESTING`**,
  because the name would otherwise describe an intake that is usually down.
* **Start of match.** The starting-configuration rules require the robot to start with the intake
  STOWED, carrying **8 preloaded pieces**. Because a stowed intake lets pieces fall out, the intake
  must be **deployed before the robot moves**:
  * `Superstructure.onEnable()` is called from `autonomousInit` and `teleopInit`, and switches the
    posture to `DEPLOYED`, unconditionally. So the first enable deploys the intake with no button
    press. (An earlier draft excluded `HOMING` and `FAULTED`. Step 7 dropped that: an enable
    arriving during the boot home must not be lost, and the posture is simply acted on once homing
    finishes.)
  * `Intake.isRetainingPieces()` is true once the pivot is anywhere between `kDeployedAngleRad`
    and `kFeedAssistAngleRad`, inclusive. That is the range in which pieces stay aboard.
  * **Every auto routine starts with `waitUntilIntakeRetaining()`** before its first path, bounded
    by a short timeout so a dead pivot cannot eat the whole auto. Teleop does not hold the driver
    back: the intake is already down from auto in a real match. If the robot is moving while
    `!isRetainingPieces()`, `Superstructure/StowedWhileMoving` is logged and a warning alert is
    raised.
* **New intake position `FEED_ASSIST`:** while scoring, the intake **rocks** between
  `kDeployedAngleRad` and `kFeedAssistAngleRad` once every `kFeedAssistPeriodSec`. Rocking lets
  pieces flow off the intake onto the hopper floor and into the indexer better than a static
  raise. Setting the period to `0` holds the raised position still, which is kept only as a
  diagnostic and as the fallback when power is short. The roller runs inward at
  `kFeedAssistRollerRpm`, so pieces at the mouth are pulled in rather than pushed out. Both ends of
  the rock are outside the turret-conflict band, so feed-assist never waits on the turret.
* FEED_ASSIST applies only while scoring **and not intaking**. Intaking takes priority, because the
  mouth has to be all the way down to collect.
* **Side effect on vision.** The intake camera is only fused while the intake is settled at its
  intaking angle (the gating decision recorded in `PROJECT_OVERVIEW.md`). Down-stays-down means that
  camera can now contribute for most of the match, not just during pickups — **but only if the gate
  admits the new resting `DEPLOYED` state**, which holds the same pivot angle as `INTAKING`. The
  implemented gate, `Intake.isIntaking()`, accepts `INTAKING` alone, so M10b widens it to
  `isAtIntakingAngle()` (`INTAKING` or `DEPLOYED`). FEED_ASSIST drops it out
  while scoring, which is acceptable because the turret camera is the one that matters for aim. The
  gate must treat `FEED_ASSISTING` like `EJECTING`: out at a different angle, so **not** fusable.

#### Intake and scoring are independent requests

The driver must be able to **shoot, move and intake at once**. Those are two independent intents, so
they are modelled as **orthogonal regions** of the Superstructure rather than as a product of enum
values:

* `Superstructure.WantedState` keeps the *shooter-side* intent: `IDLE`, `DEFAULT`, `STOW`, `HOME`,
  `EJECT`, `SPIN_UP`, `SCORE_ON_MOVE`, `SCORE_STATIONARY`, `SCORE_FORCED`, `PASS`,
  `AUTO_SCORE_ON_MOVE`, `CHARACTERIZE`. **`INTAKE` leaves this enum.**
* A second mutator, `setIntakeRequested(boolean)` (with `intakeWhileHeldCommand()` as its
  command adapter), carries the *intake* intent.
* Each loop, after the shooter-side `SystemState` is resolved, `resolveIntakeMode()` picks one
  `IntakeMode` — `STOWED`, `DEPLOYED`, `INTAKING` or `FEED_ASSIST` — from the system state, the
  intake request, the posture and the interlock. `HOMING`, `EJECTING`, `CHARACTERIZING` and
  `FAULTED` override it, as they always have. `Superstructure/IntakeMode` is logged alongside
  `Superstructure/SystemState`, so the state timeline stays readable.
* **`INTAKING` and `INDEXING` are removed as system states** (14 → 12). `INDEXING` was the seating
  pass that parked one piece in front of the flywheel; with a hopper there is nothing to park.

This keeps the two rules that matter: the Superstructure is still the only class that commands more
than one subsystem, and the interlock still lives in exactly one place. What it relaxes is "exactly
one mutator" to "two mutators, both on the Superstructure". The rejected alternative is
`INTAKE_AND_SCORE_ON_MOVE` and friends. That doubles the scoring states from four to eight, and
every future scoring state would need an intake twin that somebody will forget to write.

How the regions combine. `posture` means `DEPLOY_ONLY` or `STOW`, whichever the operator last chose.
"Gate" means `isReadyToScore()`, including the burst latch described below.

| Situation | Intake held? | Intake | Floor roller | Indexer |
| --- | --- | --- | --- | --- |
| `HOMING` / `FAULTED` / `CHARACTERIZING` / `EJECTING` | ignored | override (as before) | override | override |
| No shot requested (`RESTING`, or `SPIN_UP` pre-spin) | no | posture | `IDLE` | `IDLE` |
| same | yes | `DEPLOY_AND_INTAKE` | `IDLE` | `IDLE` |
| Shot requested (`SCORE_*`, `AUTO_SCORE_ON_MOVE`, `PASS`), gate closed | no | posture | **`FEED`** | `IDLE` |
| same, gate closed | yes | `DEPLOY_AND_INTAKE` | **`FEED`** | `IDLE` |
| Shot requested, gate **open** | no | `FEED_ASSIST` if posture is deployed, else `STOW` | `FEED` | `FEED` |
| Shot requested, gate **open** | yes | `DEPLOY_AND_INTAKE` | `FEED` | `FEED` |

"Shot requested" means the shooter-side **wanted** state, not the system state. The floor roller
therefore also runs while a score request has been diverted to `SPINNING_UP` or
`WAITING_FOR_TURRET`.

**The floor roller leads and the indexer is the gate.** The floor roller exists only to feed the
indexer. It does not run while intaking: the intake roller alone puts pieces into the hopper. It
starts the moment a shot is *requested*, before the shooter is ready, so the pile is already packed
against the indexer. The indexer stays stopped in brake mode as the physical block between the
hopper and the flywheel until the gate opens. Then the first piece is already in contact and
leaves at once, rather than after the floor roller has had to move the pile. The floor roller
stalls against the stopped indexer while it waits. Its stator limit and `LOW` power priority cover
that, and `FloorRoller/Inputs/statorCurrentAmps` shows what it costs. `SPIN_UP` on its own, as a
pre-spin while driving to a shooting position, does not start the floor roller, because nothing
has asked to shoot yet. As a result, `FloorRoller.WantedState.INTAKE` has no caller; M10b deletes
it rather than leaving a dead state.

#### The interlock, corrected

The original interlock assumed the *deployed* intake sits in the turret's sweep. The turret's
guard was literally `() -> intake.isStowed()`. **That is backwards for this robot: a deployed intake
cannot conflict with the turret.** The conflict is the arm *passing through* the turret's sweep, so
it is a band of **pivot angles**, not a deployed/stowed boolean:

* New constant `Constants.Intake.kTurretConflictBandRad`: the width of the pivot band, measured
  **from `kStowedAngleRad` toward `kDeployedAngleRad`**, over which any part of the intake lies
  inside the turret's conflict arc. The band is roughly the **top ~15° of the intake's travel**, so
  the *stowed* intake conflicts with the turret. The deployed and feed-assist positions, and
  everything between them, do not. It is expressed as a width from stow rather than two absolute
  angles for two reasons: the band's direction then follows the pivot's sign convention
  automatically, and its placeholder (15°) is already the real shape. Two absolute placeholders of
  `0.0` would describe an empty band and leave the interlock inert. `TODO: MEASURE the exact edge
  from the Onshape model (tools/onshape_measurements.py reads named mate connectors; add one at the
  band edge) and confirm on the robot.`
* **What the driver experiences.** While the intake is stowed, which in practice is only at match
  start or after an explicit `STOW`, the turret may not enter its conflict arc. Since the first
  enable deploys the intake (see above), this costs a fraction of a second at the start of auto and
  nothing afterwards.
* **Turret guard:** the turret may not enter or sweep through its conflict arc while the pivot is
  inside the band **or its commanded motion crosses the band**.
* **Intake guard:** the pivot may not move into or through the band while the turret is inside its
  conflict arc.
* **While scoring, the turret wins.** The feed-assist rock stays well below the band, so on this
  robot this rule never fires. It is kept as a guard against a mis-tuned `kFeedAssistAngleRad`. If
  the angle is ever configured inside the band and the shot needs the arc, the intake stays at
  `DEPLOYED` instead of feed-assisting, `Superstructure/FeedAssistBlocked` is logged, and a
  warning alert names the misconfiguration.
* **Intaking no longer stows the turret.** The old `intaking()` stowed the turret *only* to open the
  deploy guard. Now the turret keeps aiming while the intake comes down, unless the deploy path
  crosses the band while the turret is in the arc. In that case the deploy waits, which takes a
  fraction of a second.
* M10's hard-won lesson carries over unchanged. The turret half of the guard still uses
  `TurretUnwrapper.passesThroughArcWrapped`, because the turret's travel is wider than a full
  circle. The intake half is a plain interval test, because the pivot's travel is not.

#### Continuous feeding and flywheel recovery

* **Burst hysteresis.** Each piece knocks the flywheel down, so gate term 3 (`willBeAtSpeedIn`)
  would flicker between pieces and start and stop the indexer every few loops. Instead, once the
  feed has opened within the current scoring state, term 3 is evaluated against a wider
  `Constants.Shooter.kBurstToleranceRpm`. That holds until *any other* term closes the gate or the
  state changes. The other six terms are never relaxed.
* **Cadence is a tunable.** `Indexer.kFeedRpm` sets pieces per second. Pieces 2..N leave a
  recovering wheel, so **M11's shot tables must be measured firing bursts at the intended cadence,
  not single pieces.** A table built from single pieces over-predicts every shot after the first.
* **Shot counting at cadence.** At a 50 Hz loop, back-to-back pieces can fall inside one
  refractory window and undercount. `TODO: measure the dip width in a logged burst before tuning
  kShotRefractorySec; if pieces merge, count dips in the timestamped high-rate velocity samples
  instead of at loop rate.` An undercount
  is tolerable by design: the count is a post-match diagnostic, and `likelyEmpty()` depends on *time
  since the last shot*, not on the count.

#### Jams

With many pieces, a jam is a matter of when, not if. The indexer treats a stall while feeding as a
jam: stator current above `kJamCurrentAmps` **and** velocity below `kJamVelocityRpm` for
`kJamDetectSec`. It responds by entering `UNJAMMING`, which reverses at `kUnjamRpm` for `kUnjamSec`
and then resumes feeding. After `kMaxUnjamAttempts` within one feed request it gives up, stops in
`JAMMED` and raises an alert. The operator's `EJECT` is the recovery from there. While the indexer
is unjamming, the Superstructure reverses the floor roller too, and the `likelyEmpty` clock pauses.

#### Power

Shooting, moving and intaking at once is the robot's worst current case: drive (`CRITICAL`),
flywheel recovery and turret (`HIGH`), indexer (`MEDIUM`), and the intake pivot, intake roller and
floor roller (`LOW`). `PowerBudget` already sheds load in that order, so combined mode degrades the
intake first. That is the right order: a slower intake loses a piece, but a sagging flywheel misses
every piece in the burst. Two additions:

* FEED_ASSIST's rocking falls back to holding the raised position whenever the `LOW` scalar is
  below 1. Rocking is the preferred behaviour, but it is pure pivot current, so it is the first
  luxury to go. Holding still keeps pieces aboard, just with a slower flow.
* M14's checkpoint gains a scripted combined-mode run, driving a figure-eight while intaking and
  firing continuously. It is logged against `Power/FilteredVoltage`.

#### What this replan deletes

* **Indexer:** `hasPiece()`; the `INDEX`/`HOLD` wanted states and `INDEXING`/`HOLDING` system
  states; the latched-sample replay that existed only to answer `hasPiece` while stopped; the shot
  counter; the beam-break path, its alert, `Constants.Indexer.kHasBeamBreak`,
  `Constants.Dio.kIndexerBeamBreak`; `IndexerIOSim.setSimulatedPiecePresent`; and the piece
  detector's constants (`kIndexRpm`, `kHasPiece*`, `kShotDetectDebounceSec`,
  `kSimPieceDetectionMargin`). **The spin-up settle window stays** (`kDetectionSettleSec`): a
  wheel spinning up from rest reads as both "loaded" and "jammed", so the load and jam readings
  need it as much as the piece detector did. An earlier version of this list deleted it too; M10b
  step 1 found it was still needed.
* **Floor roller:** `WantedState.INTAKE`, `SystemState.INTAKING` and `kIntakeRpm`, which lose
  their only caller once the floor roller stops running for intaking.
* **Superstructure:** `WantedState.INTAKE`; `SystemState.INTAKING` and `INDEXING`;
  `Constants.Superstructure.kIndexFeedSec`; the piece guard in `scoringTransition`; gate term 4.
* **Autos:** the `WaitForPiece` marker and `waitForPieceCommand`, and `bailoutIfEmpty`. With no
  sensor, a missed pickup cannot be detected. The cost of not knowing is small in a
  shoot-on-the-move auto, because a scoring leg over an empty hopper costs no stop.

#### Answers to the review questions (2026-09-25)

1. **Capacity is 50 pieces, and overfilling does not jam.** So no "hopper full" signal is needed,
   and intaking needs no capacity limit. `kScoreTimeoutSec` is sized to empty 50 pieces.
2. **The stowed position conflicts with the turret, over roughly the top 15° of intake travel.**
   The feed-assist position does not conflict at all. See "The interlock, corrected".
3. **Rock back and forth.** It lets pieces flow off the intake onto the hopper floor and into the
   indexer.
4. **The robot starts with 8 pieces and the intake STOWED**, as the starting-configuration rules
   require. Pieces can fall out while stowed but stay in at partial raise, so the intake deploys on
   the first enable and autos wait for it before moving. The 8-piece preload also gives a free
   calibration of the shot detector (§10.2 M11).
5. **The floor roller runs only to feed the indexer, and it leads.** It starts when a shot is
   requested, and the stopped indexer blocks pieces until the shooter is ready. It does not run
   while intaking.

Still open, for the robot rather than for review: the exact band edge and the rocking period.

#### Where each change lands

| Section | Change |
| --- | --- |
| §2.1–2.3 | Guard examples, the worked trace and the state list no longer mention `hasPiece`, `INTAKING` or `INDEXING`. |
| §3 | File-tree notes for the indexer and shooter. |
| §4.1 `Constants` | `Dio.kIndexerBeamBreak` deleted; new intake, indexer, shooter and superstructure constants. |
| §4.4 Intake | `FEED_ASSIST`, a resting `DEPLOYED` state, and the pivot-band guard. |
| §4.6 Indexer | Rewritten: `IDLE`/`FEEDING`/`UNJAMMING`/`JAMMED`/`EJECTING`/`CHARACTERIZING`, jam handling, `isLoaded()`. |
| §4.8 Shooter | Shot detection and the count; `willBeAtSpeedIn` gains a tolerance overload for burst hysteresis. |
| §4.11 Superstructure | Orthogonal intake region, `RESTING`, `STOW`, `likelyEmpty()`, seven-term gate. |
| §4.12 Autos | Marker vocabulary; `driveIntakeAndShoot`; `bailoutIfEmpty` and `waitForPieceCommand` removed. |
| §4.14 Tests | Piece-gate tests replaced; hopper, burst, jam and interlock-band tests added. |
| §5.3 | `Auto.kScoreTimeoutSec` becomes a max-burst bound. |
| §5.6 | Beam-break row deleted from the device table. |
| §6.5 | Term 4 removed; burst hysteresis documented. |
| §8.2, §8.3, §8.5 | Marker table and the auto notes. |
| §9.2 | Simulation now injects flywheel shots and indexer load instead of a present piece. |
| §10.2 | New **M10b**; M11, M13 and M14 checkpoints amended. |
| §11 | The multi-piece omission row is replaced; two new failure modes. |

---

## 3. Complete File Tree

```
AdvantageKit_TalonFXSwerveTemplate/
├── build.gradle                                        [MODIFY] Add maple-sim dep block (optional) + test deps note
├── vendordeps/
│   ├── AdvantageKit.json                               [KEEP]   26.0.2
│   ├── Phoenix6.json                                   [KEEP]   26.1.2
│   ├── PathplannerLib.json                             [KEEP]   2026.1.2
│   ├── questnavlib.json                                [KEEP]   2026-3.0.0-beta
│   ├── Studica.json                                    [DELETE] NavX unused; Pigeon2 is the gyro
│   ├── WPILibNewCommands.json                          [KEEP]
│   └── maple-sim.json                                  [NEW]    OPTIONAL — see §9.3 for install step
├── src/main/deploy/pathplanner/
│   ├── settings.json                                   [MODIFY] Update mass/MOI/track width/wheel radius for the real robot
│   ├── navgrid.json                                    [MODIFY] Regenerate against the REBUILT field in PathPlanner
│   ├── paths/*.path                                    [NEW]    ~14 paths, see §8.4
│   └── autos/*.auto                                    [NEW]    8+ autos, see §8.3
├── src/main/java/frc/robot/
│   ├── Main.java                                       [KEEP]   Entry point; do not touch
│   ├── BuildConstants.java                             [KEEP]   gversion-generated
│   ├── Robot.java                                      [MODIFY] RT thread priority, fault polling, power budget, GC logging
│   ├── RobotContainer.java                             [MODIFY] Instantiate all subsystems + IO switch + all bindings
│   ├── RobotState.java                                 [NEW]    Single source of truth for pose/velocity/turret history
│   ├── Constants.java                                  [MODIFY] Full restructure: nested per-subsystem tunable constants
│   ├── generated/
│   │   └── TunerConstants.java                         [MODIFY] REGENERATE in Tuner X for Mk5n/Kraken; see §5.5
│   ├── subsystems/
│   │   ├── drive/
│   │   │   ├── Drive.java                              [MODIFY] Feed RobotState; add setpoint generator + brake mode
│   │   │   ├── Module.java                             [KEEP]
│   │   │   ├── ModuleIO.java                           [KEEP]
│   │   │   ├── ModuleIOTalonFX.java                    [MODIFY] Tunable gains, current-budget scalar hook
│   │   │   ├── ModuleIOSim.java                        [MODIFY] Optional maple-sim backing
│   │   │   ├── ModuleIOTalonFXS.java                   [DELETE] Unused hardware path
│   │   │   ├── GyroIO.java                             [KEEP]
│   │   │   ├── GyroIOPigeon2.java                      [MODIFY] Add pitch/roll/accel signals for tip + accel fusion
│   │   │   ├── GyroIONavX.java                         [DELETE] Pigeon2 is the gyro
│   │   │   └── PhoenixOdometryThread.java              [KEEP]
│   │   ├── intake/
│   │   │   ├── Intake.java                             [NEW]    Pivot + roller FSM, homing, pivot-band guard, feed assist
│   │   │   ├── IntakeIO.java                           [NEW]    @AutoLog inputs for pivot, roller, absolute encoder
│   │   │   ├── IntakeIOTalonFX.java                    [NEW]    Kraken X44 pivot + X60 roller + DIO ThroughBore
│   │   │   └── IntakeIOSim.java                        [NEW]    SingleJointedArmSim + DCMotorSim
│   │   ├── floorroller/
│   │   │   ├── FloorRoller.java                        [NEW]    Velocity/duty FSM
│   │   │   ├── FloorRollerIO.java                      [NEW]
│   │   │   ├── FloorRollerIOTalonFX.java               [NEW]    Kraken X60
│   │   │   └── FloorRollerIOSim.java                   [NEW]
│   │   ├── indexer/
│   │   │   ├── Indexer.java                            [NEW]    Feed/unjam FSM + load diagnostic (no piece count, §2.4)
│   │   │   ├── IndexerIO.java                          [NEW]
│   │   │   ├── IndexerIOTalonFX.java                   [NEW]    Kraken X60 (no piece sensor)
│   │   │   └── IndexerIOSim.java                       [NEW]
│   │   ├── turret/
│   │   │   ├── Turret.java                             [NEW]    FSM, unwrap, homing, velocity FF, RobotState publish
│   │   │   ├── TurretIO.java                           [NEW]
│   │   │   ├── TurretIOTalonFX.java                    [NEW]    Kraken X44 + DIO ThroughBore on gearbox output
│   │   │   └── TurretIOSim.java                        [NEW]    DCMotorSim with soft limits
│   │   ├── shooter/
│   │   │   ├── Shooter.java                            [NEW]    Flywheel FSM, spin-up prediction, at-speed gate, shot counter
│   │   │   ├── ShooterIO.java                          [NEW]
│   │   │   ├── ShooterIOTalonFX.java                   [NEW]    2x Kraken X60, leader/follower
│   │   │   └── ShooterIOSim.java                       [NEW]    FlywheelSim
│   │   ├── vision/
│   │   │   ├── Vision.java                             [NEW]    Both Limelights + QuestNav fusion policy
│   │   │   ├── VisionIO.java                           [NEW]    PoseObservation / TargetObservation records
│   │   │   ├── VisionIOLimelight.java                  [NEW]    MegaTag2 via NT, per-camera transform push
│   │   │   ├── VisionIOSim.java                        [NEW]    Ideal-camera sim from ground-truth pose
│   │   │   ├── QuestNavIO.java                         [NEW]
│   │   │   ├── QuestNavIOReal.java                     [NEW]    gg.questnav.questnav.QuestNav wrapper
│   │   │   └── QuestNavIOSim.java                      [NEW]    Perfect-tracking sim with injectable dropout
│   │   └── superstructure/
│   │       └── Superstructure.java                     [NEW]    The cross-mechanism state machine
│   ├── shooting/
│   │   ├── ShootingSolver.java                         [NEW]    Iterative time-of-flight virtual-target solver
│   │   ├── ShotParameters.java                         [NEW]    Immutable solver output record
│   │   └── TurretUnwrapper.java                        [NEW]    Finite-travel continuous-angle resolver
│   ├── autos/
│   │   ├── AutoFactory.java                            [NEW]    AutoBuilder config + NamedCommands registry
│   │   ├── AutoRoutines.java                           [NEW]    8+ composable routines
│   │   └── AutoSelector.java                           [NEW]    LoggedDashboardChooser + start-pose preview
│   ├── commands/
│   │   ├── DriveCommands.java                          [MODIFY] Heading snap, aim-assist, RobotState-sourced pose
│   │   ├── DriveToPoseCommand.java                     [NEW]    Profiled auto-align + pathfind handoff
│   │   └── CharacterizationCommands.java               [NEW]    SysId + FF routines for every mechanism
│   └── util/
│       ├── PhoenixUtil.java                            [MODIFY] Add tryUntilOk(int, Supplier) variants + signal registry
│       ├── LocalADStarAK.java                          [KEEP]
│       ├── LoggedTunableNumber.java                    [NEW]    2910/6328 tunable-constant primitive
│       ├── LoggedTunableGains.java                     [NEW]    Bundles kP/kI/kD/kS/kV/kA/kG as one tunable group
│       ├── AllianceFlipUtil.java                       [NEW]    Blue-origin <-> alliance-relative flipping
│       ├── FaultReporter.java                          [NEW]    Central WPILib Alert registry + NT publish
│       ├── PowerBudget.java                            [NEW]    Brownout prediction + current-limit scaling
│       └── GeomUtil.java                               [NEW]    Small Pose/Twist/Transform helpers used by the solver
└── src/test/java/frc/robot/
    ├── shooting/ShootingSolverTest.java                [NEW]    Convergence, stationary equivalence, moving offset
    ├── shooting/TurretUnwrapperTest.java               [NEW]    Wrap, limits, long-way-around, hysteresis
    └── subsystems/superstructure/SuperstructureFsmTest.java [NEW] Guarded transitions + timeouts
```

**Counts:** 41 `[NEW]` + 13 `[MODIFY]` + 9 `[KEEP]` + 3 `[DELETE]` Java/JSON files.

---

## 4. File-by-File Specification

> Conventions used in every skeleton below: units are **meters, radians, seconds, volts, amps**
> unless a field name says otherwise (`...Rpm`, `...Deg`, `...Inches` are the only allowed
> exceptions and they appear only at hardware/dashboard boundaries). Every `TODO` names the API to
> reach for. Where a signature could not be confirmed against 2026 vendor docs, the TODO says
> "verify signature" — do not guess silently.

### 4.1 Core

#### `src/main/java/frc/robot/Constants.java` — `package frc.robot;` [MODIFY]

The single entry point for every number in the codebase. Today it holds only the AdvantageKit
`Mode` enum; it becomes a nested-class tree where each mechanism owns a `public static final class`
of `LoggedTunableNumber` fields. Three rules the whole team must follow: (1) no numeric literal
appears in any file other than this one, `TunerConstants.java`, and unit tests; (2) anything you
might want to change at a competition is a `LoggedTunableNumber`, anything that is physically
fixed (gear ratio, CAN ID) is a plain `static final`; (3) every field whose value depends on the
REBUILT field or game piece carries a `TODO` saying where to measure it. The file also owns the
comp-vs-practice robot identity switch, so a single `Constants.getRobot()` call selects the right
offsets and gains for whichever chassis is on the cart.

```java
package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.LoggedTunableNumber;

public final class Constants {
  private Constants() {}

  // ---------------------------------------------------------------- runtime mode & identity ----

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum RobotType {
    COMP,
    PRACTICE,
    SIMBOT
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** roboRIO serial numbers. TODO: read these off each RIO with RobotController.getSerialNumber(). */
  public static final String kCompRoboRioSerial = "TODO-COMP-SERIAL";

  public static final String kPracticeRoboRioSerial = "TODO-PRACTICE-SERIAL";

  /** Master switch for tunable numbers and SysId bindings. MUST be false at competition. */
  public static final boolean kTuningMode = true;

  /** Set true only by unit tests so LoggedTunableNumber does not touch NetworkTables. */
  public static boolean disableHal = false;

  public static RobotType getRobot() {
    // TODO: In SIM/REPLAY return SIMBOT. On a real RIO compare RobotController.getSerialNumber()
    // to kCompRoboRioSerial / kPracticeRoboRioSerial and return the match, defaulting to COMP with
    // a FaultReporter alert if neither matches so an unknown RIO is loud, not silent.
    return RobotType.COMP;
  }

  public static double getLoopPeriodSec() {
    // TODO: Return edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod (0.02). Exists so the solver and
    // setpoint generator never hardcode 0.02.
    return 0.02;
  }

  // ----------------------------------------------------------------------------- CAN & ports ----

  public static final class Can {
    private Can() {}

    public static final String kDriveBus = "canivore";
    public static final String kRioBus = "rio";

    public static final int kPigeonId = 13;
    public static final int kIntakePivotId = 20;
    public static final int kIntakeRollerId = 21;
    public static final int kFloorRollerId = 22;
    public static final int kIndexerId = 23;
    public static final int kTurretId = 24;
    public static final int kFlywheelLeaderId = 25;
    public static final int kFlywheelFollowerId = 26;
    public static final int kPdhId = 1;
  }

  public static final class Dio {
    private Dio() {}

    public static final int kIntakePivotAbsEncoder = 0;
    public static final int kTurretAbsEncoder = 1;
    // No piece sensors: the robot carries a hopper and does not count pieces (section 2.4).
  }

  // -------------------------------------------------------------------------------- the field ----

  public static final class Field {
    private Field() {}

    /**
     * Blue-alliance goal center on the field floor plane, meters, blue-origin coordinates.
     * TODO: MEASURE. Get X/Y from the official 2026 REBUILT field drawings (FIRST "Field Layout and
     * Marking Diagram" PDF) or by driving the robot to the goal and reading Odometry/Robot in
     * AdvantageScope. Do not guess.
     */
    public static final LoggedTunableNumber kGoalXMeters =
        new LoggedTunableNumber("Field/GoalX", 0.0);

    public static final LoggedTunableNumber kGoalYMeters =
        new LoggedTunableNumber("Field/GoalY", 0.0);

    /** Height of the goal opening above the carpet. TODO: MEASURE from the field drawings. */
    public static final LoggedTunableNumber kGoalHeightMeters =
        new LoggedTunableNumber("Field/GoalHeight", 0.0);

    /** Field length/width used only by AllianceFlipUtil. TODO: take from the AprilTag layout JSON. */
    public static final LoggedTunableNumber kFieldLengthMeters =
        new LoggedTunableNumber("Field/Length", 0.0);

    public static final LoggedTunableNumber kFieldWidthMeters =
        new LoggedTunableNumber("Field/Width", 0.0);

    /** Mass of one REBUILT game piece, kg. TODO: MEASURE on a kitchen scale; used only by maple-sim. */
    public static final LoggedTunableNumber kGamePieceMassKg =
        new LoggedTunableNumber("Field/GamePieceMass", 0.0);

    /** Game-piece diameter, meters. TODO: MEASURE with calipers; used by intake-camera ranging. */
    public static final LoggedTunableNumber kGamePieceDiameterMeters =
        new LoggedTunableNumber("Field/GamePieceDiameter", 0.0);

    public static Translation2d goalTranslation() {
      // TODO: Build a Translation2d from kGoalX/kGoalY, then pass it through
      // AllianceFlipUtil.apply(...) so red-alliance matches aim at the mirrored goal.
      return new Translation2d();
    }

    public static Translation3d goalTranslation3d() {
      // TODO: Same as goalTranslation() but with kGoalHeightMeters as Z. Used by the ballistic
      // TOF table fit and by the AdvantageScope 3D overlay.
      return new Translation3d();
    }
  }

  // -------------------------------------------------------------------------------- the drive ----

  public static final class Drivetrain {
    private Drivetrain() {}

    /** Mass and MOI feed PathPlanner's RobotConfig. TODO: weigh the robot; MOI from CAD or a spin test. */
    public static final double kRobotMassKg = 74.088;

    public static final double kRobotMoiKgM2 = 6.883;
    public static final double kWheelCoefficientOfFriction = 1.2;

    public static final LoggedTunableNumber kJoystickDeadband =
        new LoggedTunableNumber("Drive/Deadband", 0.08);

    public static final LoggedTunableNumber kHeadingKp = new LoggedTunableNumber("Drive/HeadingKp", 5.0);
    public static final LoggedTunableNumber kHeadingKd = new LoggedTunableNumber("Drive/HeadingKd", 0.4);
    public static final LoggedTunableNumber kHeadingMaxVelRadPerSec =
        new LoggedTunableNumber("Drive/HeadingMaxVel", 8.0);
    public static final LoggedTunableNumber kHeadingMaxAccelRadPerSec2 =
        new LoggedTunableNumber("Drive/HeadingMaxAccel", 20.0);

    public static final LoggedTunableNumber kAutoAlignTranslationKp =
        new LoggedTunableNumber("Drive/AutoAlignKp", 4.0);
    public static final LoggedTunableNumber kAutoAlignMaxVelMetersPerSec =
        new LoggedTunableNumber("Drive/AutoAlignMaxVel", 3.0);
    public static final LoggedTunableNumber kAutoAlignMaxAccelMetersPerSec2 =
        new LoggedTunableNumber("Drive/AutoAlignMaxAccel", 3.0);
    public static final LoggedTunableNumber kAutoAlignToleranceMeters =
        new LoggedTunableNumber("Drive/AutoAlignTolerance", 0.03);

    /** Max module azimuth rate for PathPlanner's SwerveSetpointGenerator, rad/s. */
    public static final double kMaxModuleAzimuthRadPerSec = 20.0;
  }

  // ------------------------------------------------------------------------------- the intake ----

  public static final class Intake {
    private Intake() {}

    public static final double kPivotGearRatio = 1.0; // TODO: motor rotations per gearbox-output rotation.
    public static final boolean kPivotInverted = false;
    public static final boolean kRollerInverted = false;

    /** ThroughBore reading, in rotations, when the pivot is exactly at kStowedAngleRad. TODO: measure. */
    public static final LoggedTunableNumber kPivotAbsEncoderOffsetRot =
        new LoggedTunableNumber("Intake/PivotAbsOffset", 0.0);

    public static final LoggedTunableNumber kStowedAngleRad =
        new LoggedTunableNumber("Intake/StowedAngle", 0.0);
    public static final LoggedTunableNumber kDeployedAngleRad =
        new LoggedTunableNumber("Intake/DeployedAngle", 0.0);
    public static final LoggedTunableNumber kEjectAngleRad =
        new LoggedTunableNumber("Intake/EjectAngle", 0.0);

    /**
     * Partial-raise angle used while scoring to push the hopper toward the indexer. Between
     * kDeployedAngleRad and kStowedAngleRad. TODO: find on the robot; start just high enough to
     * touch the pile.
     */
    public static final LoggedTunableNumber kFeedAssistAngleRad =
        new LoggedTunableNumber("Intake/FeedAssistAngle", 0.0);

    /** Roller speed while feed-assisting: inward, so pieces at the mouth are pulled in, not out. */
    public static final LoggedTunableNumber kFeedAssistRollerRpm =
        new LoggedTunableNumber("Intake/FeedAssistRollerRpm", 1000.0);

    /**
     * Full rocking period between the deployed and feed-assist angles. Rocking is the intended
     * behaviour: it lets pieces flow off the intake onto the hopper floor and into the indexer. 0
     * holds feed-assist still (diagnostic / low-power fallback). TODO: tune on the robot.
     */
    public static final LoggedTunableNumber kFeedAssistPeriodSec =
        new LoggedTunableNumber("Intake/FeedAssistPeriod", 0.5);

    /**
     * Width of the turret-conflict band, measured from kStowedAngleRad toward kDeployedAngleRad:
     * roughly the top 15 degrees of travel. The STOWED intake is inside it; the deployed and
     * feed-assist positions are not. TODO: MEASURE the exact edge from Onshape (a mate connector at
     * the band edge) and confirm on the robot.
     */
    public static final LoggedTunableNumber kTurretConflictBandRad =
        new LoggedTunableNumber("Intake/TurretConflictBand", Math.toRadians(15.0));

    public static final LoggedTunableNumber kPivotToleranceRad =
        new LoggedTunableNumber("Intake/PivotTolerance", 0.05);

    public static final LoggedTunableNumber kPivotKp = new LoggedTunableNumber("Intake/PivotKp", 40.0);
    public static final LoggedTunableNumber kPivotKd = new LoggedTunableNumber("Intake/PivotKd", 0.0);
    public static final LoggedTunableNumber kPivotKs = new LoggedTunableNumber("Intake/PivotKs", 0.0);
    public static final LoggedTunableNumber kPivotKv = new LoggedTunableNumber("Intake/PivotKv", 0.0);
    public static final LoggedTunableNumber kPivotKg = new LoggedTunableNumber("Intake/PivotKg", 0.0);
    public static final LoggedTunableNumber kPivotCruiseRadPerSec =
        new LoggedTunableNumber("Intake/PivotCruise", 8.0);
    public static final LoggedTunableNumber kPivotAccelRadPerSec2 =
        new LoggedTunableNumber("Intake/PivotAccel", 30.0);

    public static final LoggedTunableNumber kIntakeRollerRpm =
        new LoggedTunableNumber("Intake/RollerRpm", 3000.0);
    public static final LoggedTunableNumber kEjectRollerRpm =
        new LoggedTunableNumber("Intake/EjectRollerRpm", -3000.0);

    public static final LoggedTunableNumber kHomingDutyCycle =
        new LoggedTunableNumber("Intake/HomingDutyCycle", -0.15);
    public static final LoggedTunableNumber kHomingVelocityThresholdRadPerSec =
        new LoggedTunableNumber("Intake/HomingVelThreshold", 0.15);
    public static final LoggedTunableNumber kHomingSettleSec =
        new LoggedTunableNumber("Intake/HomingSettle", 0.25);
    public static final LoggedTunableNumber kHomingTimeoutSec =
        new LoggedTunableNumber("Intake/HomingTimeout", 3.0);

    public static final double kPivotStatorLimitAmps = 60.0;
    public static final double kRollerStatorLimitAmps = 80.0;

    /** Moment of inertia and length for SingleJointedArmSim. TODO: pull from CAD. */
    public static final double kPivotMoiKgM2 = 0.05;

    public static final double kPivotLengthMeters = 0.35;
  }

  public static final class FloorRoller {
    private FloorRoller() {}

    public static final double kGearRatio = 1.0; // TODO: measure.
    public static final boolean kInverted = false;
    public static final double kStatorLimitAmps = 60.0;

    // No kIntakeRpm: the floor roller runs only to feed the indexer (section 2.4).
    public static final LoggedTunableNumber kFeedRpm =
        new LoggedTunableNumber("FloorRoller/FeedRpm", 3000.0);
    public static final LoggedTunableNumber kEjectRpm =
        new LoggedTunableNumber("FloorRoller/EjectRpm", -2500.0);
    public static final LoggedTunableNumber kKp = new LoggedTunableNumber("FloorRoller/Kp", 0.1);
    public static final LoggedTunableNumber kKv = new LoggedTunableNumber("FloorRoller/Kv", 0.12);
    public static final LoggedTunableNumber kKs = new LoggedTunableNumber("FloorRoller/Ks", 0.0);
  }

  public static final class Indexer {
    private Indexer() {}

    public static final double kGearRatio = 1.0; // TODO: measure.
    public static final boolean kInverted = false;
    public static final double kStatorLimitAmps = 50.0;

    /** Sets the burst cadence (pieces per second). M11 measures the shot tables at this speed. */
    public static final LoggedTunableNumber kFeedRpm =
        new LoggedTunableNumber("Indexer/FeedRpm", 3500.0);
    public static final LoggedTunableNumber kEjectRpm =
        new LoggedTunableNumber("Indexer/EjectRpm", -1500.0);
    public static final LoggedTunableNumber kKp = new LoggedTunableNumber("Indexer/Kp", 0.1);
    public static final LoggedTunableNumber kKv = new LoggedTunableNumber("Indexer/Kv", 0.12);

    /**
     * How long after entering FEEDING the load and jam readings are ignored. Spin-up from rest
     * draws high current at low speed, which reads as loaded AND jammed at once; without this
     * every burst would open with a spurious unjam pulse. TODO: MEASURE the spin-up duration.
     */
    public static final LoggedTunableNumber kDetectionSettleSec =
        new LoggedTunableNumber("Indexer/DetectionSettleSec", 0.4);

    /**
     * "The indexer is pushing against something" while FEEDING, past the spin-up settle window:
     * current above this. Current alone -- the settle window rejects the spin-up spike, and a
     * velocity-shortfall term would miss a stiff loop pushing pieces at full speed. A diagnostic
     * that corroborates Superstructure.likelyEmpty(); it gates nothing.
     * TODO: tune from Indexer/Inputs while feeding a full hopper and an empty one.
     */
    public static final LoggedTunableNumber kLoadedCurrentAmps =
        new LoggedTunableNumber("Indexer/LoadedCurrent", 25.0);
    public static final LoggedTunableNumber kLoadedDebounceSec =
        new LoggedTunableNumber("Indexer/LoadedDebounce", 0.06);

    /** Jam = current above kJamCurrentAmps AND speed below kJamVelocityRpm for kJamDetectSec. */
    public static final LoggedTunableNumber kJamCurrentAmps =
        new LoggedTunableNumber("Indexer/JamCurrent", 40.0);

    public static final LoggedTunableNumber kJamVelocityRpm =
        new LoggedTunableNumber("Indexer/JamVelocity", 200.0);
    public static final LoggedTunableNumber kJamDetectSec =
        new LoggedTunableNumber("Indexer/JamDetect", 0.15);
    public static final LoggedTunableNumber kUnjamRpm =
        new LoggedTunableNumber("Indexer/UnjamRpm", -1500.0);
    public static final LoggedTunableNumber kUnjamSec = new LoggedTunableNumber("Indexer/Unjam", 0.2);
    public static final int kMaxUnjamAttempts = 3;
  }

  // ------------------------------------------------------------------------------- the turret ----

  public static final class Turret {
    private Turret() {}

    public static final double kGearRatio = 1.0; // TODO: motor rotations per turret rotation.
    public static final boolean kInverted = false;
    public static final double kStatorLimitAmps = 40.0;

    /** ThroughBore rotations when the turret points straight forward (robot +X). TODO: measure. */
    public static final LoggedTunableNumber kAbsEncoderOffsetRot =
        new LoggedTunableNumber("Turret/AbsOffset", 0.0);

    /**
     * Mechanical travel limits in the turret's own frame, robot-relative radians. TODO: MEASURE by
     * driving the turret slowly to each hard stop with kTuningMode on and reading
     * Turret/Inputs/positionRad, then back off by kLimitMarginRad.
     */
    public static final LoggedTunableNumber kMinAngleRad =
        new LoggedTunableNumber("Turret/MinAngle", -Math.toRadians(200.0));

    public static final LoggedTunableNumber kMaxAngleRad =
        new LoggedTunableNumber("Turret/MaxAngle", Math.toRadians(200.0));
    public static final LoggedTunableNumber kLimitMarginRad =
        new LoggedTunableNumber("Turret/LimitMargin", Math.toRadians(3.0));

    public static final LoggedTunableNumber kKp = new LoggedTunableNumber("Turret/Kp", 50.0);
    public static final LoggedTunableNumber kKd = new LoggedTunableNumber("Turret/Kd", 0.5);
    public static final LoggedTunableNumber kKs = new LoggedTunableNumber("Turret/Ks", 0.15);
    public static final LoggedTunableNumber kKv = new LoggedTunableNumber("Turret/Kv", 0.12);
    public static final LoggedTunableNumber kKa = new LoggedTunableNumber("Turret/Ka", 0.0);
    public static final LoggedTunableNumber kCruiseRadPerSec =
        new LoggedTunableNumber("Turret/Cruise", 12.0);
    public static final LoggedTunableNumber kAccelRadPerSec2 =
        new LoggedTunableNumber("Turret/Accel", 60.0);

    public static final LoggedTunableNumber kToleranceRad =
        new LoggedTunableNumber("Turret/Tolerance", Math.toRadians(1.5));
    public static final LoggedTunableNumber kVelocityToleranceRadPerSec =
        new LoggedTunableNumber("Turret/VelTolerance", 0.5);

    /** Below this angular error the unwrapper refuses to switch branches (anti-chatter). */
    public static final LoggedTunableNumber kUnwrapHysteresisRad =
        new LoggedTunableNumber("Turret/UnwrapHysteresis", Math.toRadians(10.0));

    public static final LoggedTunableNumber kStowAngleRad =
        new LoggedTunableNumber("Turret/StowAngle", 0.0);

    /** Turret angles (robot-relative) that would sweep the camera or shooter through the intake. */
    public static final LoggedTunableNumber kIntakeConflictMinRad =
        new LoggedTunableNumber("Turret/IntakeConflictMin", Math.toRadians(150.0));

    public static final LoggedTunableNumber kIntakeConflictMaxRad =
        new LoggedTunableNumber("Turret/IntakeConflictMax", Math.toRadians(210.0));

    public static final double kMoiKgM2 = 0.02; // TODO: from CAD, for TurretIOSim.
  }

  // ------------------------------------------------------------------------------ the shooter ----

  public static final class Shooter {
    private Shooter() {}

    public static final double kGearRatio = 1.0; // TODO: motor rotations per flywheel rotation.
    public static final boolean kLeaderInverted = false;
    public static final boolean kFollowerOpposesLeader = true; // TODO: verify against the gearbox.
    public static final double kStatorLimitAmps = 80.0;
    public static final double kSupplyLimitAmps = 60.0;

    public static final LoggedTunableNumber kKp = new LoggedTunableNumber("Shooter/Kp", 0.2);
    public static final LoggedTunableNumber kKi = new LoggedTunableNumber("Shooter/Ki", 0.0);
    public static final LoggedTunableNumber kKd = new LoggedTunableNumber("Shooter/Kd", 0.0);
    public static final LoggedTunableNumber kKs = new LoggedTunableNumber("Shooter/Ks", 0.0);
    public static final LoggedTunableNumber kKv = new LoggedTunableNumber("Shooter/Kv", 0.118);
    public static final LoggedTunableNumber kKa = new LoggedTunableNumber("Shooter/Ka", 0.0);

    public static final LoggedTunableNumber kIdleRpm = new LoggedTunableNumber("Shooter/IdleRpm", 1500.0);
    public static final LoggedTunableNumber kToleranceRpm =
        new LoggedTunableNumber("Shooter/ToleranceRpm", 75.0);
    public static final LoggedTunableNumber kMaxRpm = new LoggedTunableNumber("Shooter/MaxRpm", 6000.0);
    public static final LoggedTunableNumber kSpinUpTimeoutSec =
        new LoggedTunableNumber("Shooter/SpinUpTimeout", 3.0);

    /**
     * Wider at-speed tolerance used while a burst is already feeding, so the gate does not flicker
     * between pieces (section 2.4). TODO: tune against the hit rate of pieces 2..N in a burst.
     */
    public static final LoggedTunableNumber kBurstToleranceRpm =
        new LoggedTunableNumber("Shooter/BurstToleranceRpm", 300.0);

    /**
     * Shot detection: a loop-to-loop speed drop larger than kShotDipRpm, while the wheel was within
     * kShotArmToleranceRpm of its goal, is one piece through the wheels; then ignore further drops
     * for kShotRefractorySec. TODO: measure the dip size and width from a logged burst before tuning.
     */
    public static final LoggedTunableNumber kShotDipRpm =
        new LoggedTunableNumber("Shooter/ShotDipRpm", 150.0);

    public static final LoggedTunableNumber kShotArmToleranceRpm =
        new LoggedTunableNumber("Shooter/ShotArmTolerance", 400.0);
    public static final LoggedTunableNumber kShotRefractorySec =
        new LoggedTunableNumber("Shooter/ShotRefractory", 0.04);

    /** FlywheelSim moment of inertia, kg*m^2. TODO: from CAD (wheels + shafts). */
    public static final double kMoiKgM2 = 0.004;
  }

  // ------------------------------------------------------------------- shooting-on-the-move ----

  public static final class Shot {
    private Shot() {}

    /**
     * Distances (m) the shot table is sampled at. Build it by standing still at each distance,
     * firing bursts with measureShotTablePoint (section 4.13) and raising ShotTable/Rpm until the
     * burst scores. TODO: MEASURE at 8-10 distances spanning your usable range.
     *
     * (There was a kLaunchSpeedMetersPerSec table here. The solver never read it: the ground speed a
     * shot needs is distance / time of flight, and the RPM comes from the fit below. M11 deleted it.)
     */
    public static final double[] kDistanceBreakpointsMeters = {1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0};

    /** Distance (m) -> time of flight (s), from the same bursts. TODO: MEASURE with video at 120 fps. */
    public static final double[] kTimeOfFlightSec = {0, 0, 0, 0, 0, 0, 0};

    /** Flywheel speed and taped distance for the next measurement burst (M11). Tuning only. */
    public static final LoggedTunableNumber kMeasureRpm = new LoggedTunableNumber("ShotTable/Rpm", 3000.0);
    public static final LoggedTunableNumber kMeasureTapedDistanceMeters =
        new LoggedTunableNumber("ShotTable/TapedDistance", 0.0);

    /** Linear map from required muzzle speed (m/s) to flywheel RPM. TODO: fit from your shot data. */
    public static final LoggedTunableNumber kRpmPerMeterPerSec =
        new LoggedTunableNumber("Shot/RpmPerMps", 0.0);

    public static final LoggedTunableNumber kRpmIntercept = new LoggedTunableNumber("Shot/RpmIntercept", 0.0);

    /** Solver iteration count and convergence tolerance. */
    public static final int kMaxSolverIterations = 6;

    public static final LoggedTunableNumber kSolverToleranceSec =
        new LoggedTunableNumber("Shot/SolverTolerance", 0.002);

    /** 0 disables acceleration compensation; 1 applies it fully. Start at 0, raise once velocity works. */
    public static final LoggedTunableNumber kAccelCompensationScalar =
        new LoggedTunableNumber("Shot/AccelComp", 0.0);

    /** Gates for readyToShoot. */
    public static final LoggedTunableNumber kMaxChassisAccelForShotMetersPerSec2 =
        new LoggedTunableNumber("Shot/MaxAccel", 3.0);

    public static final LoggedTunableNumber kMaxVisionStalenessSec =
        new LoggedTunableNumber("Shot/MaxVisionStaleness", 0.6);
    public static final LoggedTunableNumber kMaxPoseStdDevMeters =
        new LoggedTunableNumber("Shot/MaxPoseStdDev", 0.20);
    public static final LoggedTunableNumber kMinDistanceMeters =
        new LoggedTunableNumber("Shot/MinDistance", 1.0);
    public static final LoggedTunableNumber kMaxDistanceMeters =
        new LoggedTunableNumber("Shot/MaxDistance", 7.0);
  }

  // ------------------------------------------------------------------------------- the vision ----

  public static final class Vision {
    private Vision() {}

    public static final String kTurretCameraName = "limelight-turret";
    public static final String kIntakeCameraName = "limelight-intake";

    /**
     * Robot origin -> turret rotation axis. Only the translation and any fixed tilt live here; the
     * turret's own rotation is composed in at runtime. TODO: MEASURE from CAD.
     */
    public static final Transform3d kRobotToTurretAxis =
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

    /** Turret frame (0 rad) -> turret camera lens. TODO: MEASURE from CAD, then verify with a tag. */
    public static final Transform3d kTurretToCamera =
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

    /** Robot origin -> intake camera lens (fixed; camera rides the frame, not the pivot). TODO: MEASURE. */
    public static final Transform3d kRobotToIntakeCamera =
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

    /** Robot origin -> Quest 3S headset origin. TODO: MEASURE; this is the single biggest QuestNav error source. */
    public static final Transform3d kRobotToQuest =
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

    /** Base MegaTag2 standard deviations at 1 m with 1 tag, [x, y, theta]. Scaled by distance^2 / tagCount. */
    public static final LoggedTunableNumber kMegaTag2XyStdDevBase =
        new LoggedTunableNumber("Vision/MT2XyStdDev", 0.04);

    public static final LoggedTunableNumber kMegaTag2ThetaStdDev =
        new LoggedTunableNumber("Vision/MT2ThetaStdDev", 9999.0); // MT2 heading is an input, not an output.

    public static final LoggedTunableNumber kQuestXyStdDev =
        new LoggedTunableNumber("Vision/QuestXyStdDev", 0.02);
    public static final LoggedTunableNumber kQuestThetaStdDev =
        new LoggedTunableNumber("Vision/QuestThetaStdDev", 0.035);

    /** Reject any vision pose farther than this from the current estimate. */
    public static final LoggedTunableNumber kMaxAcceptedPoseJumpMeters =
        new LoggedTunableNumber("Vision/MaxJump", 1.0);

    public static final LoggedTunableNumber kMaxAmbiguity = new LoggedTunableNumber("Vision/MaxAmbiguity", 0.3);
    public static final LoggedTunableNumber kMaxZErrorMeters = new LoggedTunableNumber("Vision/MaxZError", 0.4);

    /** Reject turret-camera frames captured while the turret is slewing faster than this. */
    public static final LoggedTunableNumber kMaxTurretRateForVisionRadPerSec =
        new LoggedTunableNumber("Vision/MaxTurretRate", 1.5);

    /** QuestNav health gates. */
    public static final LoggedTunableNumber kQuestMinBatteryPercent =
        new LoggedTunableNumber("Vision/QuestMinBattery", 15.0);

    public static final LoggedTunableNumber kQuestReseedResidualMeters =
        new LoggedTunableNumber("Vision/QuestReseedResidual", 0.30);
    public static final LoggedTunableNumber kQuestReseedHoldSec =
        new LoggedTunableNumber("Vision/QuestReseedHold", 0.5);
  }

  // --------------------------------------------------------------------------- power & faults ----

  public static final class Power {
    private Power() {}

    public static final LoggedTunableNumber kBrownoutWarningVolts =
        new LoggedTunableNumber("Power/BrownoutWarnVolts", 7.5);

    public static final LoggedTunableNumber kBrownoutCriticalVolts =
        new LoggedTunableNumber("Power/BrownoutCritVolts", 6.9);
    public static final LoggedTunableNumber kTotalCurrentBudgetAmps =
        new LoggedTunableNumber("Power/TotalBudget", 250.0);
    public static final LoggedTunableNumber kMinCurrentScalar =
        new LoggedTunableNumber("Power/MinScalar", 0.5);
  }
}
```

---

#### `src/main/java/frc/robot/RobotState.java` — `package frc.robot;` [NEW]

The single source of truth for "where is the robot, how fast is it going, and where was the turret
pointing at time *t*." Every consumer — the shooting solver, the vision transform math, auto-align,
PathPlanner's `AutoBuilder` — reads from here, and nothing else owns a pose estimator. It wraps a
`SwerveDrivePoseEstimator` (fed odometry at 250 Hz from `Drive.periodic()` and vision at ~30 Hz from
`Vision.periodic()`) plus two `TimeInterpolatableBuffer`s: robot pose and turret angle, both 2.0 s
deep. All writes happen on the main robot thread; the read side is guarded by a
`ReentrantReadWriteLock` so a future background consumer cannot tear a `Pose2d`. It also tracks
per-source health (last-update timestamp, current covariance) so `Superstructure.isReadyToScore()`
can ask "do I trust this pose enough to take a shot."

```java
package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  /** Where a pose observation came from; used for covariance, logging and fallback rules. */
  public enum PoseSource {
    ODOMETRY,
    LIMELIGHT_TURRET,
    LIMELIGHT_INTAKE,
    QUESTNAV
  }

  /** One timestamped absolute-pose measurement with its covariance. */
  public record PoseObservation(
      Pose2d pose, double timestampSec, Matrix<N3, N1> stdDevs, PoseSource source) {}

  private static RobotState instance = null;

  private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(2.0);
  private final TimeInterpolatableBuffer<Double> turretAngleBuffer =
      TimeInterpolatableBuffer.createDoubleBuffer(2.0);

  private final LinearFilter accelXFilter = LinearFilter.movingAverage(5);
  private final LinearFilter accelYFilter = LinearFilter.movingAverage(5);

  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions;
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private Translation2d fieldRelativeAccel = Translation2d.kZero;
  private ChassisSpeeds lastFieldRelativeSpeeds = new ChassisSpeeds();
  private double lastSpeedsTimestampSec = 0.0;

  private final double[] lastUpdateTimestampBySource = new double[PoseSource.values().length];
  private final double[] lastStdDevBySource = new double[PoseSource.values().length];
  private boolean questTrusted = false;
  private boolean questSeeded = false;

  public static RobotState getInstance() {
    // TODO: Lazily construct the singleton with Drive.getModuleTranslations() as the kinematics
    // source. Keep it lazy so unit tests can build one without a HAL.
    return instance;
  }

  private RobotState(SwerveDriveKinematics kinematics) {
    // TODO: Store kinematics, initialize lastModulePositions to four zeroed SwerveModulePosition
    // objects, and construct the SwerveDrivePoseEstimator at Pose2d.kZero with odometry std devs
    // VecBuilder.fill(0.003, 0.003, 0.002) and placeholder vision std devs (overridden per call).
  }

  public static void initialize(SwerveDriveKinematics kinematics) {
    // TODO: Create the singleton explicitly from RobotContainer before any subsystem uses it, so
    // construction order is deterministic instead of depending on who calls getInstance() first.
  }

  // ------------------------------------------------------------------------------ write side ----

  public void addOdometryObservation(
      double timestampSec, Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
    // TODO: Take the write lock. Call poseEstimator.updateWithTime(timestampSec, gyroYaw,
    // modulePositions), then poseBuffer.addSample(timestampSec, poseEstimator.getEstimatedPosition()).
    // Record lastUpdateTimestampBySource[ODOMETRY]. Called ~250x per second from Drive.periodic().
  }

  public void addDriveSpeedsObservation(double timestampSec, ChassisSpeeds robotRelative) {
    // TODO: Under the write lock, store robotRelative, convert to field relative with
    // ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, getEstimatedPose().getRotation()), then
    // differentiate against lastFieldRelativeSpeeds over (timestampSec - lastSpeedsTimestampSec)
    // and push through accelXFilter/accelYFilter to get fieldRelativeAccel.
  }

  public void addVisionObservation(PoseObservation observation) {
    // TODO: Under the write lock, reject the observation if it is older than the buffer window, if
    // its translation differs from getEstimatedPose() by more than
    // Constants.Vision.kMaxAcceptedPoseJumpMeters, or if any stdDev is non-finite. Otherwise call
    // poseEstimator.addVisionMeasurement(pose, timestampSec, stdDevs) and update the per-source
    // bookkeeping arrays. Log accepted and rejected counts.
  }

  public void addTurretObservation(double timestampSec, double turretAngleRad) {
    // TODO: Under the write lock, turretAngleBuffer.addSample(timestampSec, turretAngleRad). Called
    // once per Turret.periodic(); the buffer is what makes the moving camera transform correct.
  }

  public void resetPose(Pose2d pose, Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
    // TODO: Under the write lock call poseEstimator.resetPosition(gyroYaw, modulePositions, pose)
    // and clear poseBuffer. Also set questSeeded = false so Vision re-seeds QuestNav to the new pose.
  }

  public void setQuestTrusted(boolean trusted) {
    // TODO: Write-locked setter used by Vision's health policy; logged so you can see in replay
    // exactly when QuestNav dropped out.
  }

  public void setQuestSeeded(boolean seeded) {
    // TODO: Write-locked setter; Vision sets this true after a successful QuestNav.setPose().
  }

  // ------------------------------------------------------------------------------- read side ----

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    // TODO: Read-locked poseEstimator.getEstimatedPosition().
    return Pose2d.kZero;
  }

  public Optional<Pose2d> getPoseAt(double timestampSec) {
    // TODO: Read-locked poseBuffer.getSample(timestampSec). Returns empty if the timestamp predates
    // the buffer, which callers must treat as "skip this measurement", never as "use current pose".
    return Optional.empty();
  }

  public Optional<Double> getTurretAngleAt(double timestampSec) {
    // TODO: Read-locked turretAngleBuffer.getSample(timestampSec). Used by
    // Vision.robotToTurretCameraAt(t).
    return Optional.empty();
  }

  @AutoLogOutput(key = "RobotState/FieldRelativeSpeeds")
  public ChassisSpeeds getFieldRelativeSpeeds() {
    // TODO: Read-locked copy of fieldRelativeSpeeds. This is the velocity the shooting solver uses.
    return new ChassisSpeeds();
  }

  @AutoLogOutput(key = "RobotState/RobotRelativeSpeeds")
  public ChassisSpeeds getRobotRelativeSpeeds() {
    // TODO: Read-locked copy; PathPlanner's AutoBuilder consumes this one.
    return new ChassisSpeeds();
  }

  @AutoLogOutput(key = "RobotState/FieldRelativeAccel")
  public Translation2d getFieldRelativeAccel() {
    // TODO: Read-locked copy of the filtered acceleration, m/s^2, blue-origin field frame.
    return Translation2d.kZero;
  }

  public Rotation2d getRotation() {
    // TODO: getEstimatedPose().getRotation().
    return Rotation2d.kZero;
  }

  @AutoLogOutput(key = "RobotState/DistanceToGoalMeters")
  public double getDistanceToGoal() {
    // TODO: getEstimatedPose().getTranslation().getDistance(Constants.Field.goalTranslation()).
    return 0.0;
  }

  @AutoLogOutput(key = "RobotState/VisionStalenessSec")
  public double getVisionStalenessSec() {
    // TODO: Timer.getFPGATimestamp() minus the most recent of the two Limelight entries in
    // lastUpdateTimestampBySource. Used as a readyToShoot gate.
    return Double.POSITIVE_INFINITY;
  }

  @AutoLogOutput(key = "RobotState/PoseConfidence")
  public double getPoseStdDevEstimate() {
    // TODO: Return a scalar confidence proxy: the smallest recently-applied XY std dev, inflated by
    // getVisionStalenessSec(). Compare against Constants.Shot.kMaxPoseStdDevMeters.
    return Double.POSITIVE_INFINITY;
  }

  public boolean isQuestTrusted() {
    // TODO: Read-locked getter.
    return false;
  }

  public boolean isQuestSeeded() {
    // TODO: Read-locked getter.
    return false;
  }

  public void logState() {
    // TODO: Called once per robotPeriodic from Robot. Log per-source staleness and std devs under
    // RobotState/Sources/<NAME>/... with Logger.recordOutput so replay shows exactly which source
    // was driving the estimate at any instant.
  }
}
```

---

#### `src/main/java/frc/robot/Robot.java` — `package frc.robot;` [MODIFY]

Keeps the template's AdvantageKit logger wiring verbatim (metadata, `WPILOGWriter`, `NT4Publisher`,
replay source) and adds four competition behaviors: real-time thread priority around the scheduler
so 250 Hz odometry does not get starved, a per-loop call into `FaultReporter` and `PowerBudget`,
loop-overrun and GC logging so you can prove in replay whether a missed shot was a timing problem,
and a brake-mode-release timer that lets the drivetrain coast a few seconds after a match so the
robot can be pushed off the field.

```java
package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private double autoStartTimeSec = 0.0;
  private boolean brakeModeReleased = false;
  private final edu.wpi.first.wpilibj.Timer disabledTimer = new edu.wpi.first.wpilibj.Timer();

  public Robot() {
    // TODO: KEEP the template body exactly as-is (Logger.recordMetadata from BuildConstants, the
    // switch on Constants.currentMode adding WPILOGWriter/NT4Publisher or the replay source, then
    // Logger.start()). After Logger.start(), add Logger.registerURCL if you adopt URCL, then
    // construct RobotContainer. Do not reorder: Logger.start() must precede any subsystem.
  }

  @Override
  public void robotPeriodic() {
    // TODO: Threads.setCurrentThreadPriority(true, 99); CommandScheduler.getInstance().run();
    // Threads.setCurrentThreadPriority(false, 10). Between the two, also call
    // RobotState.getInstance().logState(), FaultReporter.getInstance().periodic() and
    // PowerBudget.getInstance().periodic(). Log Logger.getRealTimestamp()-based loop duration under
    // "LoopTimeMs" and Runtime.getRuntime().freeMemory() under "MemoryFreeMB".
  }

  @Override
  public void disabledInit() {
    // TODO: disabledTimer.restart(); brakeModeReleased = false. Also call
    // robotContainer.onDisable() so subsystems can stop integrators.
  }

  @Override
  public void disabledPeriodic() {
    // TODO: After disabledTimer.hasElapsed(kCoastDelaySec) and while the robot is nearly stationary,
    // call robotContainer.setDriveBrakeMode(false) once so the robot can be pushed. Also refresh the
    // auto chooser preview: robotContainer.updateAutoPreview().
  }

  @Override
  public void autonomousInit() {
    // TODO: robotContainer.setDriveBrakeMode(true); autoStartTimeSec = Timer.getFPGATimestamp();
    // autonomousCommand = robotContainer.getAutonomousCommand(); schedule it if non-null.
    // Call superstructure.onEnable() first and shooter.resetShotCount(): the robot starts with the
    // intake STOWED (a rule) and 8 pieces aboard, and onEnable deploys it so none fall out.
  }

  @Override
  public void autonomousPeriodic() {
    // TODO: Log "Auto/ElapsedSec" so you can compare routine timing across matches in AdvantageScope.
  }

  @Override
  public void autonomousExit() {
    // TODO: Log the final auto pose and shot count so you can score your own routines from the log.
  }

  @Override
  public void teleopInit() {
    // TODO: Cancel autonomousCommand if non-null, robotContainer.setDriveBrakeMode(true), and reset
    // the superstructure to its DEFAULT wanted state with setIntakeRequested(false), so auto never
    // leaks a goal into teleop, then superstructure.onEnable(). (The intake posture is deliberately
    // NOT reset to stowed: an intake deployed in auto stays deployed, because the hopper is
    // presumably not empty.)
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // TODO: CommandScheduler.getInstance().cancelAll().
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    // TODO: If maple-sim is installed, seed SimulatedArena.getInstance() and place the robot at the
    // selected auto start pose. Otherwise leave empty.
  }

  @Override
  public void simulationPeriodic() {
    // TODO: If maple-sim is installed, SimulatedArena.getInstance().simulationPeriodic() and log
    // the simulated game pieces under "FieldSimulation/GamePieces" as a Pose3d[].
  }
}
```

---

#### `src/main/java/frc/robot/RobotContainer.java` — `package frc.robot;` [MODIFY]

Grows from "make a Drive and bind four buttons" into the wiring harness for the whole robot: the
`switch (Constants.currentMode)` selects REAL / SIM / REPLAY IO implementations for all seven
subsystems, `RobotState.initialize()` runs before anything reads it, `AutoFactory` registers named
commands, and the two-controller binding scheme routes every driver action through
`Superstructure.setWantedStateCommand(...)` rather than touching subsystems directly. SysId and
characterization bindings are guarded by `Constants.kTuningMode` so they cannot fire at an event.

```java
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoSelector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.floorroller.FloorRoller;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
  private final Drive drive;
  private final Intake intake;
  private final FloorRoller floorRoller;
  private final Indexer indexer;
  private final Turret turret;
  private final Shooter shooter;
  private final Vision vision;
  private final Superstructure superstructure;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final AutoSelector autoSelector;

  public RobotContainer() {
    // TODO: switch (Constants.currentMode) { REAL -> new XxxIOTalonFX(...); SIM -> new XxxIOSim();
    // default -> new XxxIO() {} } for drive modules/gyro, intake, floorRoller, indexer, turret,
    // shooter and the two vision cameras + QuestNav. THEN call
    // RobotState.initialize(new SwerveDriveKinematics(Drive.getModuleTranslations())) BEFORE
    // constructing Superstructure. Finally build AutoFactory, AutoSelector, and call
    // configureButtonBindings() and configureCharacterizationBindings().
  }

  private void configureButtonBindings() {
    // TODO: drive.setDefaultCommand(DriveCommands.joystickDrive(...)) reading RobotState for
    // heading. Bind: rightBumper -> SCORE_ON_MOVE (whileTrue) / DEFAULT (onFalse);
    // rightTrigger -> superstructure.intakeWhileHeldCommand() (independent of the scoring
    // buttons, so intake + score + drive can all be held at once - section 2.4; release stops the
    // roller and leaves the intake DOWN); leftBumper -> SCORE_STATIONARY; leftTrigger -> PASS;
    // X -> EJECT; Y -> HOME; operator button -> STOW (the only thing that raises the intake);
    // rumble the driver on superstructure.likelyEmpty() while scoring;
    // B -> reset pose to the vision estimate (ignoringDisable(true));
    // POV up/down/left/right -> DriveCommands.snapToHeading(...);
    // A -> DriveToPoseCommand to the nearest scoring pose. Operator gets manual turret nudge,
    // manual flywheel RPM trim, and a "force shoot" override that bypasses readyToShoot gates.
  }

  private void configureCharacterizationBindings() {
    // TODO: if (!Constants.kTuningMode) return; then add every SysId and feedforward routine from
    // CharacterizationCommands to the auto chooser as named options, exactly as the stock template
    // does for the drivetrain. Guarding on kTuningMode keeps them off the field.
  }

  public Command getAutonomousCommand() {
    // TODO: return autoSelector.get().
    return null;
  }

  public void setDriveBrakeMode(boolean brake) {
    // TODO: drive.setBrakeMode(brake) and turret/intake neutral-mode passthrough so a disabled robot
    // can be pushed but a disabled turret does not flop.
  }

  public void onDisable() {
    // TODO: superstructure.setWantedState(Superstructure.WantedState.IDLE) and tell every subsystem
    // to reset its profiles so re-enabling does not produce a step input.
  }

  public void updateAutoPreview() {
    // TODO: autoSelector.logStartingPose() so AdvantageScope's 2D field shows where the selected
    // routine expects the robot to start. Catching a misplaced robot before the match starts is
    // worth the twenty lines.
  }
}
```

---

### 4.2 Utilities

#### `src/main/java/frc/robot/util/LoggedTunableNumber.java` — `package frc.robot.util;` [NEW]

The primitive that makes "all constants are tunable" real. In `kTuningMode` it publishes a
`LoggedNetworkNumber` under `/Tuning/<key>` (AdvantageScope's tuning tab writes there and the value
is logged, so replay reproduces whatever you typed mid-match); outside tuning mode it returns the
compiled default with zero NetworkTables traffic. `hasChanged(int id)` and the static
`ifChanged(...)` helpers let a subsystem reapply Phoenix `Slot0Configs` only when a gain actually
moved — reapplying every loop floods the CAN bus. This is the 2910/6328 class, reproduced because
the template does not ship it.

```java
package frc.robot.util;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  public LoggedTunableNumber(String dashboardKey) {
    // TODO: this.key = tableKey + "/" + dashboardKey. Do not touch NetworkTables here.
  }

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    // TODO: this(dashboardKey) then initDefault(defaultValue).
  }

  public void initDefault(double defaultValue) {
    // TODO: If !hasDefault, latch the default and, when Constants.kTuningMode && !Constants.disableHal,
    // construct new LoggedNetworkNumber(key, defaultValue).
  }

  public double get() {
    // TODO: Return 0.0 if no default. Otherwise return dashboardNumber.get() in tuning mode with HAL
    // available, else defaultValue.
    return 0.0;
  }

  public boolean hasChanged(int id) {
    // TODO: Compare get() against lastHasChangedValues.get(id); store and return true on change.
    // Callers pass hashCode() of the owning object as the id so two subsystems do not collide.
    return false;
  }

  @Override
  public double getAsDouble() {
    // TODO: return get().
    return 0.0;
  }

  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    // TODO: If any tunable hasChanged(id), call action with the current values as a double[].
  }

  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    // TODO: Delegate to the Consumer overload, ignoring the values.
  }
}
```

---

#### `src/main/java/frc/robot/util/LoggedTunableGains.java` — `package frc.robot.util;` [NEW]

A thin bundle of six `LoggedTunableNumber`s (kP, kI, kD, kS, kV, kA, optionally kG) plus a single
`ifChanged` hook, so a subsystem writes one line — `gains.ifChanged(hashCode(), io::setGains)` —
instead of seven. It exists because every closed-loop mechanism here (pivot, turret, flywheel, four
drive modules) needs the same boilerplate, and getting it wrong in one place means a gain silently
never reaches the Talon.

```java
package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import java.util.function.Consumer;

public class LoggedTunableGains {
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;
  private final LoggedTunableNumber kG;

  public LoggedTunableGains(String prefix, double p, double i, double d, double s, double v, double a) {
    // TODO: Create one LoggedTunableNumber per gain under "<prefix>/kP" etc., with kG defaulted to 0.
  }

  public LoggedTunableGains withGravity(double g) {
    // TODO: initDefault kG and return this, so arm/pivot mechanisms can opt in fluently.
    return this;
  }

  public Slot0Configs toSlot0Configs() {
    // TODO: Build a Slot0Configs with withKP/withKI/withKD/withKS/withKV/withKA/withKG from the
    // current tunable values. Verify the kG and GravityType setters against Phoenix6 26.1.2 docs.
    return new Slot0Configs();
  }

  public void ifChanged(int id, Consumer<Slot0Configs> action) {
    // TODO: LoggedTunableNumber.ifChanged(id, () -> action.accept(toSlot0Configs()), kP, kI, kD,
    // kS, kV, kA, kG). The action should call tryUntilOk(5, () -> talon.getConfigurator()
    // .apply(configs, 0.25)) so a dropped CAN frame does not silently lose the new gains.
  }

  public double kv() {
    // TODO: return kV.get(); needed for open-loop velocity feedforward outside Phoenix.
    return 0.0;
  }

  public double ks() {
    // TODO: return kS.get().
    return 0.0;
  }
}
```

---

#### `src/main/java/frc/robot/util/AllianceFlipUtil.java` — `package frc.robot.util;` [NEW]

Everything in this codebase lives in blue-origin WPILib field coordinates. This class is the only
place allowed to know about red alliance. It flips translations, rotations and poses across the
field so `Constants.Field.goalTranslation()`, auto start poses, and scoring targets are written once
for blue and work on both sides. Keeping the flip in one file prevents the classic bug where the
shooter aims correctly on blue and mirrors wrong on red.

```java
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AllianceFlipUtil {
  private AllianceFlipUtil() {}

  public static boolean shouldFlip() {
    // TODO: DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red. Call this every loop,
    // never cache it at construction: the alliance is not known until the DS connects.
    return false;
  }

  public static Translation2d apply(Translation2d translation) {
    // TODO: If shouldFlip(), return a rotationally symmetric flip
    // (kFieldLength - x, kFieldWidth - y). TODO: CONFIRM whether the 2026 REBUILT field is
    // rotationally symmetric or mirrored; if mirrored, use (kFieldLength - x, y) instead. Read the
    // field drawings before committing to one.
    return translation;
  }

  public static Rotation2d apply(Rotation2d rotation) {
    // TODO: If shouldFlip(), rotate by Math.PI (rotational symmetry) or negate + add PI (mirror).
    // Must match the convention chosen in apply(Translation2d).
    return rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    // TODO: Compose the two overloads above.
    return pose;
  }

  public static double applyX(double xMeters) {
    // TODO: Scalar convenience used by auto routines that only need the X coordinate flipped.
    return xMeters;
  }
}
```

---

#### `src/main/java/frc/robot/util/FaultReporter.java` — `package frc.robot.util;` [NEW]

A central registry of WPILib `Alert` objects so device health is discoverable in one place rather
than scattered across seven subsystems. Subsystems register a named check with a `BooleanSupplier`
("is this Talon connected"), and the reporter polls them once per loop, drives the `Alert`, counts
how long each fault has been active, and publishes a summary boolean (`Faults/AnyCritical`) that the
driver dashboard can show as a single red light. The concrete competitive value: a CANcoder that
browns out between matches shows up on the dashboard in the queue line, not at the first shot.

```java
package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class FaultReporter {
  private record Fault(
      String name, BooleanSupplier isFaulted, Alert alert, AlertType type, double[] activeSince) {}

  private static FaultReporter instance = null;
  private final List<Fault> faults = new ArrayList<>();

  public static FaultReporter getInstance() {
    // TODO: Lazy singleton.
    return instance;
  }

  public void register(String name, AlertType type, BooleanSupplier isFaulted) {
    // TODO: Create a new Alert(name, type) and append a Fault record. Call this from every subsystem
    // constructor, once per device, e.g. register("Turret motor disconnected", kError,
    // () -> !inputs.motorConnected).
  }

  public void registerDeviceDisconnect(String deviceName, BooleanSupplier connected) {
    // TODO: Convenience wrapper: register(deviceName + " disconnected", kError, () -> !connected).
  }

  public void periodic() {
    // TODO: For each fault, call alert.set(isFaulted.getAsBoolean()), track activeSince, and
    // Logger.recordOutput("Faults/" + name, active) plus "Faults/AnyCritical" and "Faults/Count".
  }

  public boolean anyCritical() {
    // TODO: Return true if any registered kError fault is currently active. Superstructure can use
    // this to refuse to enter SCORING states when a shooter motor is dead.
    return false;
  }

  public List<String> getActiveFaultNames() {
    // TODO: Return the names of currently-active faults for the operator dashboard string.
    return List.of();
  }
}
```

---

#### `src/main/java/frc/robot/util/PowerBudget.java` — `package frc.robot.util;` [NEW]

Six Kraken X60s and two X44s can, momentarily, ask for far more current than a battery at 40 %
state-of-charge can supply; the result is a roboRIO brownout mid-match, which drops every closed
loop simultaneously. This class reads bus voltage and PDH total current, predicts an impending
brownout from the voltage slope, and publishes a `getScalar()` in `[kMinCurrentScalar, 1.0]` that
every subsystem multiplies into its stator current limit. Non-critical mechanisms (intake roller,
floor roller) are throttled first; the flywheel and drive are throttled last. The capability bought
is simple: you finish the match still able to drive and shoot instead of rebooting at 30 seconds.

```java
package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PowerBudget {
  /** Priority tiers, throttled from lowest to highest as headroom shrinks. */
  public enum Priority {
    LOW,
    MEDIUM,
    HIGH,
    CRITICAL
  }

  private static PowerBudget instance = null;
  private PowerDistribution pdh;
  private double filteredVoltage = 12.0;
  private double voltageSlopeVoltsPerSec = 0.0;
  private double scalar = 1.0;

  public static PowerBudget getInstance() {
    // TODO: Lazy singleton; construct PowerDistribution(Constants.Can.kPdhId, ModuleType.kRev) only
    // when Constants.currentMode == REAL so sim and replay do not touch the HAL.
    return instance;
  }

  public void periodic() {
    // TODO: Read RobotController.getBatteryVoltage() through a LinearFilter.singlePoleIIR, estimate
    // the slope, and compute scalar: 1.0 above kBrownoutWarningVolts, ramping linearly to
    // kMinCurrentScalar at kBrownoutCriticalVolts. Also log pdh.getTotalCurrent(),
    // RobotController.isBrownedOut() and the scalar.
  }

  public double getScalar(Priority priority) {
    // TODO: Return 1.0 for CRITICAL, and progressively more aggressive fractions of `scalar` for
    // HIGH/MEDIUM/LOW so the intake gives up its amps before the drivetrain does.
    return 1.0;
  }

  public double getFilteredVoltage() {
    // TODO: Getter, logged.
    return 12.0;
  }

  public boolean isBrownoutImminent() {
    // TODO: filteredVoltage < kBrownoutWarningVolts && voltageSlope < 0. Superstructure can use this
    // to refuse a spin-up it will not be able to finish.
    return false;
  }
}
```

---

#### `src/main/java/frc/robot/util/GeomUtil.java` — `package frc.robot.util;` [NEW]

Small pure geometry helpers the shooting solver and vision transform math need repeatedly. Pure
functions with no state, so they are trivially unit-testable and safe to call from anywhere.

```java
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GeomUtil {
  private GeomUtil() {}

  public static Translation2d toTranslation(ChassisSpeeds fieldRelativeSpeeds) {
    // TODO: new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).
    return Translation2d.kZero;
  }

  public static double radialComponent(Translation2d velocity, Rotation2d lineOfSight) {
    // TODO: Project velocity onto lineOfSight: v.getX()*cos + v.getY()*sin. Positive means closing.
    return 0.0;
  }

  public static double tangentialComponent(Translation2d velocity, Rotation2d lineOfSight) {
    // TODO: Perpendicular projection: -v.getX()*sin + v.getY()*cos. Drives the turret velocity
    // feedforward (omega = tangential / distance).
    return 0.0;
  }

  public static Transform3d rotateAboutZ(Transform3d base, double yawRad) {
    // TODO: Compose base with a Rotation3d(0, 0, yawRad) applied at base's origin. Used to build the
    // turret camera transform. Verify the composition order against WPILib's Transform3d docs -
    // getting pre- vs post-multiplication backwards is the classic vision bug.
    return base;
  }

  public static Pose3d toPose3d(Pose2d pose) {
    // TODO: new Pose3d(pose) - used when publishing to AdvantageScope's 3D field.
    return new Pose3d();
  }

  public static double interpolate(double[] xs, double[] ys, double x) {
    // TODO: Linear interpolation with clamping at both ends, over pre-sorted xs. Used to build the
    // shot tables in a unit-testable way. Prefer this over InterpolatingDoubleTreeMap only inside
    // ShootingSolver so the solver stays a pure function for tests.
    return 0.0;
  }
}
```

---

#### `src/main/java/frc/robot/util/PhoenixUtil.java` — `package frc.robot.util;` [MODIFY]

The template ships one method. Add a batched signal-refresh registry so every non-swerve Talon on
the RIO bus is refreshed with a single `BaseStatusSignal.refreshAll(...)` call per loop instead of
one call per subsystem (measurably cheaper, and it gives every subsystem the same timestamp), plus
config helpers that wrap `tryUntilOk`.

```java
package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class PhoenixUtil {
  private static final List<BaseStatusSignal> registeredSignals = new ArrayList<>();
  private static BaseStatusSignal[] signalArray = new BaseStatusSignal[0];

  private PhoenixUtil() {}

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    // TODO: KEEP the existing template body (loop until error.isOK()). Add a FaultReporter alert
    // when all attempts fail, so a config that never applied is visible instead of silent.
  }

  public static void registerSignals(BaseStatusSignal... signals) {
    // TODO: Append to registeredSignals and rebuild signalArray. Call from every IO constructor for
    // its non-odometry signals.
  }

  public static void refreshAll() {
    // TODO: BaseStatusSignal.refreshAll(signalArray) once per robotPeriodic, before subsystems run.
    // Log the returned StatusCode under "Phoenix/RefreshStatus".
  }

  public static void optimizeBus(ParentDevice... devices) {
    // TODO: ParentDevice.optimizeBusUtilizationForAll(devices) wrapped in tryUntilOk. Every Talon in
    // this project must be optimized or the RIO bus will saturate with 9 devices.
  }

  public static double getLatencyCompensatedSeconds(BaseStatusSignal signal) {
    // TODO: Return signal.getTimestamp().getTime() converted to the FPGA time base the same way
    // PhoenixOdometryThread does (FPGA now minus signal latency). Verify against Phoenix6 26.1.2.
    return 0.0;
  }
}
```

---

### 4.3 Drivetrain

#### `src/main/java/frc/robot/subsystems/drive/Drive.java` — `package frc.robot.subsystems.drive;` [MODIFY]

Three changes to the stock file. (1) The `SwerveDrivePoseEstimator` moves out into `RobotState`;
`Drive.periodic()` now *feeds* `RobotState` with each of the ~5 odometry samples it drains per loop
instead of owning the estimate. That is what lets vision, the solver and PathPlanner all read one
pose. (2) `runVelocity` gains a PathPlanner `SwerveSetpointGenerator` path so commanded chassis
speeds respect module azimuth rate and wheel friction limits — this removes the wheel-slip that
otherwise corrupts odometry exactly when you are driving hard and trying to shoot. (3) Brake-mode
control and an aggregate current draw getter for `PowerBudget`. `AutoBuilder.configure` moves to
`AutoFactory`. Everything else — the odometry loop, SysId, the `Alert`s, the characterization
getters — stays exactly as the template wrote it.

```java
package frc.robot.subsystems.drive;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;

public class Drive extends SubsystemBase {
  public static final double ODOMETRY_FREQUENCY = 0.0; // KEEP template expression.
  public static final double DRIVE_BASE_RADIUS = 0.0; // KEEP template expression.
  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert;
  private final SwerveDriveKinematics kinematics;
  private final SwerveSetpointGenerator setpointGenerator;

  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions;
  private SwerveSetpoint previousSetpoint;
  private boolean useSetpointGenerator = true;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    // TODO: KEEP the template constructor's module construction, HAL.report, and
    // PhoenixOdometryThread.getInstance().start(). REMOVE the AutoBuilder.configure block (it moves
    // to AutoFactory). ADD: construct SwerveSetpointGenerator(Drive.PP_CONFIG (M13: it lives
    // here, built from TunerConstants, because this constructor needs it first),
    // Constants.Drivetrain.kMaxModuleAzimuthRadPerSec) and seed previousSetpoint from the current
    // module states. KEEP the SysIdRoutine block verbatim.
  }

  @Override
  public void periodic() {
    // TODO: KEEP the template body through the odometry sample loop, but replace
    // poseEstimator.updateWithTime(...) with RobotState.getInstance().addOdometryObservation(
    // sampleTimestamps[i], rawGyroRotation, modulePositions). After the loop, call
    // RobotState.getInstance().addDriveSpeedsObservation(Timer.getFPGATimestamp(),
    // getChassisSpeeds()). KEEP the gyro-disconnect alert and the disabled-state module stop.
  }

  public void runVelocity(ChassisSpeeds speeds) {
    // TODO: If useSetpointGenerator, previousSetpoint = setpointGenerator.generateSetpoint(
    // previousSetpoint, speeds, Constants.getLoopPeriodSec()) and run
    // previousSetpoint.moduleStates(); otherwise fall back to the template's discretize +
    // toSwerveModuleStates + desaturateWheelSpeeds path. Log both the requested and generated speeds.
  }

  public void runVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    // TODO: Overload used by PathPlanner's output callback. Pass the per-module torque-current
    // feedforwards from DriveFeedforwards through to Module.runSetpoint so path following gets
    // accurate acceleration. Verify DriveFeedforwards accessor names against PathplannerLib 2026.1.2.
  }

  public void runCharacterization(double output) {
    // TODO: KEEP template body.
  }

  public void stop() {
    // TODO: KEEP template body (runVelocity with zeroed ChassisSpeeds).
  }

  public void stopWithX() {
    // TODO: KEEP template body (resetHeadings to module translation angles, then stop).
  }

  public void setBrakeMode(boolean brake) {
    // TODO: Loop modules calling a new Module.setBrakeMode(boolean) which forwards to
    // ModuleIO.setBrakeMode. Called from Robot.disabledPeriodic after the coast delay.
  }

  public void setUseSetpointGenerator(boolean use) {
    // TODO: Escape hatch for SysId and wheel-radius characterization, which need un-slewed commands.
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // TODO: KEEP template body.
    return null;
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // TODO: KEEP template body.
    return null;
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    // TODO: KEEP template body; widen visibility from private to public so RobotState/AutoFactory
    // can read it.
    return new SwerveModuleState[4];
  }

  public SwerveModulePosition[] getModulePositions() {
    // TODO: KEEP template body; widen to public (RobotState.resetPose needs it).
    return new SwerveModulePosition[4];
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    // TODO: KEEP template body; widen to public.
    return new ChassisSpeeds();
  }

  public SwerveDriveKinematics getKinematics() {
    // TODO: Return the kinematics object so RobotState.initialize and AutoFactory share one instance.
    return kinematics;
  }

  public Rotation2d getRawGyroRotation() {
    // TODO: Return rawGyroRotation, needed by RobotState.resetPose.
    return Rotation2d.kZero;
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    // TODO: KEEP template body.
    return new double[4];
  }

  public double getFFCharacterizationVelocity() {
    // TODO: KEEP template body.
    return 0.0;
  }

  @AutoLogOutput(key = "Drive/TotalCurrentAmps")
  public double getTotalCurrentAmps() {
    // TODO: Sum every module's drive + turn stator current from its logged inputs. PowerBudget reads
    // this to decide whether the drivetrain is what is eating the battery.
    return 0.0;
  }

  public double getMaxLinearSpeedMetersPerSec() {
    // TODO: KEEP template body.
    return 0.0;
  }

  public double getMaxAngularSpeedRadPerSec() {
    // TODO: KEEP template body.
    return 0.0;
  }

  public static Translation2d[] getModuleTranslations() {
    // TODO: KEEP template body (reads LocationX/LocationY from TunerConstants).
    return new Translation2d[4];
  }
}
```

---

#### `src/main/java/frc/robot/subsystems/drive/ModuleIO.java` — `package frc.robot.subsystems.drive;` [KEEP + one added method]

One method is appended to the existing interface; the `@AutoLog` inputs class is untouched.

```java
  /** Enables or disables brake mode on both motors. */
  public default void setBrakeMode(boolean brake) {}
```

#### `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java` [MODIFY]

Keep the entire template implementation. Add exactly three things: an implementation of the new
`setBrakeMode`, a `LoggedTunableGains` pair (drive + steer) wired through `ifChanged` so you can
retune swerve gains from AdvantageScope without a redeploy, and a `PowerBudget`-scaled stator limit
reapplied only when the scalar moves by more than 5 %.

```java
  @Override
  public void setBrakeMode(boolean brake) {
    // TODO: Build a MotorOutputConfigs with NeutralModeValue.Brake/Coast and apply it to both
    // driveTalon and turnTalon via tryUntilOk. Do NOT re-apply the whole TalonFXConfiguration here;
    // that drops the Slot0 gains for a loop.
  }

  private void updateTunableGains() {
    // TODO: Called at the end of updateInputs. driveGains.ifChanged(hashCode(), cfg ->
    // tryUntilOk(5, () -> driveTalon.getConfigurator().apply(cfg, 0.25))) and the same for steer.
  }

  private void updateCurrentLimit() {
    // TODO: double scalar = PowerBudget.getInstance().getScalar(Priority.CRITICAL). If it differs
    // from the last applied value by >5 %, apply a CurrentLimitsConfigs with
    // StatorCurrentLimit = constants.SlipCurrent * scalar.
  }
```

#### `src/main/java/frc/robot/subsystems/drive/ModuleIOSim.java` [MODIFY]

Keep the template's `DCMotorSim` implementation as the default. Add a `setBrakeMode` no-op and an
optional maple-sim backing selected by a `Constants` flag (see §9.3) so that when maple-sim is
installed the module reports positions produced by the rigid-body physics engine instead of four
independent motor sims.

```java
  @Override
  public void setBrakeMode(boolean brake) {
    // TODO: No-op in the DCMotorSim path; if maple-sim is enabled, forward to the SwerveModuleSimulation.
  }
```

#### `src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java` [MODIFY]

Keep everything. Add pitch, roll and the three world-frame accelerometer signals to the
`@AutoLog` inputs so you can (a) detect a tip-over and refuse to shoot, and (b) cross-check
`RobotState`'s differentiated acceleration against a real accelerometer while tuning the solver.

```java
  // Added to GyroIO.GyroIOInputs:
  //   public double pitchRad = 0.0;
  //   public double rollRad = 0.0;
  //   public double accelXMetersPerSec2 = 0.0;
  //   public double accelYMetersPerSec2 = 0.0;
  //   public double accelZMetersPerSec2 = 0.0;

  private void createExtraSignals() {
    // TODO: pigeon.getPitch(), getRoll(), getAccelerationX/Y/Z(). Set update frequency 50 Hz, add
    // them to PhoenixUtil.registerSignals, and populate the new inputs in updateInputs. Verify the
    // acceleration signal names against Phoenix6 26.1.2 - they may be getAccelerationX or
    // getGravityVectorX depending on frame.
  }
```

---

### 4.4 Intake (pivot + roller)

#### `src/main/java/frc/robot/subsystems/intake/IntakeIO.java` — `package frc.robot.subsystems.intake;` [NEW]

The hardware contract for the pivoting intake: one Kraken X44 on the pivot, one Kraken X60 on the
roller, and a WCP ThroughBore absolute encoder reading the *gearbox output*, which means the pivot
angle is directly observable with no homing sequence required once the offset is calibrated (we keep
a homing routine anyway as a fallback for a dead encoder). Every field is a public primitive so
`@AutoLog` can generate the replayable input struct.

```java
package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean pivotConnected = false;
    public boolean absEncoderConnected = false;
    public double pivotPositionRad = 0.0;
    public double pivotAbsolutePositionRad = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotStatorCurrentAmps = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotTempCelsius = 0.0;

    public boolean rollerConnected = false;
    public double rollerVelocityRpm = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
    public double rollerTempCelsius = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Motion-Magic position control of the pivot, radians at the gearbox output. */
  public default void setPivotPosition(double positionRad) {}

  /** Open-loop pivot control for homing and manual jogging. */
  public default void setPivotOpenLoop(double volts) {}

  /** Closed-loop roller velocity, RPM at the roller shaft. */
  public default void setRollerVelocity(double velocityRpm) {}

  public default void setRollerOpenLoop(double volts) {}

  /** Seeds the pivot's internal rotor position from the absolute encoder. */
  public default void seedPivotPosition(double positionRad) {}

  public default void setPivotBrakeMode(boolean brake) {}

  public default void setPivotGains(com.ctre.phoenix6.configs.Slot0Configs gains) {}

  public default void setCurrentLimitScalar(double scalar) {}

  public default void stop() {}
}
```

#### `src/main/java/frc/robot/subsystems/intake/IntakeIOTalonFX.java` [NEW]

Real-hardware implementation. The one subtle piece: the ThroughBore is wired to a roboRIO DIO and
read with `DutyCycleEncoder`, which lives on the RIO, not the CAN bus — so it has no Phoenix status
signal and no latency compensation. We read it once at boot (and again whenever the pivot is at rest
and the seed drifts) to `setPosition()` the Talon's internal rotor sensor, then run closed loop
off the Talon's own high-rate sensor. A one-line upgrade path is documented for a CTRE CANdi.

```java
package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX pivot;
  private final TalonFX roller;
  private final DutyCycleEncoder pivotAbsEncoder;

  private final MotionMagicVoltage pivotPositionRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotVolts;
  private final StatusSignal<Current> pivotStator;
  private final StatusSignal<Current> pivotSupply;
  private final StatusSignal<Temperature> pivotTemp;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerVolts;
  private final StatusSignal<Current> rollerStator;
  private final StatusSignal<Temperature> rollerTemp;

  private final Debouncer pivotConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rollerConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private double currentLimitScalar = 1.0;

  public IntakeIOTalonFX() {
    // TODO: Construct both TalonFX on Constants.Can.kRioBus, apply a TalonFXConfiguration with
    // Feedback.SensorToMechanismRatio = Constants.Intake.kPivotGearRatio, Slot0 from
    // Constants.Intake gains, MotionMagic cruise/accel, SoftwareLimitSwitch forward/reverse at the
    // stow/deploy limits, Brake neutral mode, and stator limits. Create the DutyCycleEncoder on
    // Constants.Dio.kIntakePivotAbsEncoder. Register all signals with PhoenixUtil.registerSignals
    // at 50 Hz, then PhoenixUtil.optimizeBus(pivot, roller). Finally seed the pivot from the
    // absolute encoder. UPGRADE PATH: replacing the DIO encoder with a CTRE CANdi lets you set
    // Feedback.FeedbackSensorSource = FusedCANdiPWM1 and delete the seeding logic entirely.
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO: BaseStatusSignal.refreshAll on the pivot signals and the roller signals separately so
    // each group's isOK() feeds its own Debouncer into inputs.pivotConnected/rollerConnected. Convert
    // rotations to radians with Units.rotationsToRadians and rotations/sec to RPM with *60. Read
    // pivotAbsEncoder.get() and pivotAbsEncoder.isConnected() for the absolute fields.
  }

  @Override
  public void setPivotPosition(double positionRad) {
    // TODO: pivot.setControl(pivotPositionRequest.withPosition(Units.radiansToRotations(positionRad))
    // .withFeedForward(Constants.Intake.kPivotKg.get() * Math.cos(positionRad))). The cosine term is
    // the gravity feedforward for a pivoting arm; drop it if your pivot is counterbalanced.
  }

  @Override
  public void setPivotOpenLoop(double volts) {
    // TODO: pivot.setControl(voltageRequest.withOutput(volts)).
  }

  @Override
  public void setRollerVelocity(double velocityRpm) {
    // TODO: roller.setControl(rollerVelocityRequest.withVelocity(velocityRpm / 60.0)).
  }

  @Override
  public void setRollerOpenLoop(double volts) {
    // TODO: roller.setControl(voltageRequest.withOutput(volts)).
  }

  @Override
  public void seedPivotPosition(double positionRad) {
    // TODO: PhoenixUtil.tryUntilOk(5, () -> pivot.setPosition(Units.radiansToRotations(positionRad),
    // 0.25)).
  }

  @Override
  public void setPivotBrakeMode(boolean brake) {
    // TODO: Apply a MotorOutputConfigs with the requested NeutralModeValue via tryUntilOk.
  }

  @Override
  public void setPivotGains(Slot0Configs gains) {
    // TODO: tryUntilOk(5, () -> pivot.getConfigurator().apply(gains, 0.25)).
  }

  @Override
  public void setCurrentLimitScalar(double scalar) {
    // TODO: Store it and, when it moves more than 5 %, apply CurrentLimitsConfigs with
    // Constants.Intake.kPivotStatorLimitAmps * scalar and kRollerStatorLimitAmps * scalar.
  }

  @Override
  public void stop() {
    // TODO: pivot.stopMotor(); roller.stopMotor().
  }
}
```

#### `src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java` [NEW]

Physics sim good enough to develop the FSM without hardware: a `SingleJointedArmSim` for the pivot
(so gravity, hard stops and the gravity feedforward all behave) and a `DCMotorSim` for the roller,
each driven by a WPILib `ProfiledPIDController` standing in for Motion Magic.

```java
package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  private final SingleJointedArmSim pivotSim;
  private final DCMotorSim rollerSim;
  private final ProfiledPIDController pivotController;
  private boolean pivotClosedLoop = false;
  private double pivotAppliedVolts = 0.0;
  private double rollerAppliedVolts = 0.0;
  private double rollerSetpointRpm = 0.0;

  public IntakeIOSim() {
    // TODO: pivotSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1) -- TODO verify a
    // getKrakenX44() factory exists in WPILib 2026; if not, build a DCMotor with the X44 datasheet
    // stall torque/free speed -- kPivotGearRatio, kPivotMoiKgM2, kPivotLengthMeters,
    // kStowedAngleRad, kDeployedAngleRad, true, kStowedAngleRad). rollerSim from
    // LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio).
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO: Run the profiled controller when pivotClosedLoop, clamp to +-12 V, call
    // pivotSim.setInputVoltage / rollerSim.setInputVoltage and update(Constants.getLoopPeriodSec()),
    // then fill every input field. Set pivotConnected/rollerConnected/absEncoderConnected true and
    // pivotAbsolutePositionRad equal to pivotPositionRad (a perfect encoder).
  }

  @Override
  public void setPivotPosition(double positionRad) {
    // TODO: pivotClosedLoop = true; pivotController.setGoal(positionRad).
  }

  @Override
  public void setPivotOpenLoop(double volts) {
    // TODO: pivotClosedLoop = false; pivotAppliedVolts = volts.
  }

  @Override
  public void setRollerVelocity(double velocityRpm) {
    // TODO: Store the setpoint and drive rollerAppliedVolts with a simple kV feedforward plus P term.
  }

  @Override
  public void setRollerOpenLoop(double volts) {
    // TODO: rollerAppliedVolts = volts.
  }

  @Override
  public void seedPivotPosition(double positionRad) {
    // TODO: pivotSim.setState(positionRad, 0.0).
  }

  @Override
  public void setPivotGains(Slot0Configs gains) {
    // TODO: pivotController.setPID(gains.kP, gains.kI, gains.kD).
  }

  @Override
  public void stop() {
    // TODO: Zero both applied voltages and clear pivotClosedLoop.
  }
}
```

#### `src/main/java/frc/robot/subsystems/intake/Intake.java` [NEW]

The pivoting intake's state machine, written in the 2910 idiom. Two things make it more than a
"go to angle" wrapper. First, a real homing sequence: drive the pivot open-loop into its hard stop
until velocity stays below a threshold for a settle time, then seed position — this is the fallback
when the ThroughBore is unplugged, and it has a timeout so a jammed intake enters `FAULTED` instead
of stalling a motor for the rest of the match. Second, a pivot guard: the intake refuses to move into
or through the turret-conflict pivot band (the `kTurretConflictBandRad` of travel nearest stow)
while the turret is inside its conflict arc, which is the single most likely way to destroy this
robot. The *stowed* position is inside that band; the deployed and feed-assist positions are not, so
once deployed the intake never blocks the turret (§2.4). The robot starts stowed (a rule) and the
first enable deploys it, because pieces can fall out of a stowed intake. Third, because the robot carries a hopper, **the intake rests deployed**: releasing the
intake button stops the roller but leaves the arm down, and a partial-raise `FEED_ASSIST` position
pushes the hopper toward the indexer while scoring.

```java
package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends SubsystemBase {
  public enum WantedState {
    IDLE,
    STOW,
    DEPLOY_AND_INTAKE,
    DEPLOY_ONLY, // deployed, roller stopped: the resting posture while holding pieces
    FEED_ASSIST, // partial raise (optionally oscillating) to push the hopper into the indexer
    EJECT,
    HOME,
    CHARACTERIZE
  }

  public enum SystemState {
    IDLE,
    STOWING,
    STOWED,
    DEPLOYING,
    DEPLOYED,
    INTAKING,
    FEED_ASSISTING,
    EJECTING,
    HOMING,
    CHARACTERIZING,
    FAULTED
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();
  private final Timer homingSettleTimer = new Timer();

  private WantedState wantedState = WantedState.HOME;
  private SystemState systemState = SystemState.HOMING;
  private boolean isHomed = false;
  private double characterizationVolts = 0.0;
  private java.util.function.BooleanSupplier deployAllowed = () -> true;

  public Intake(IntakeIO io) {
    // TODO: Store io, start stateTimer, and register FaultReporter checks for pivotConnected,
    // rollerConnected and absEncoderConnected.
  }

  public void setDeployGuard(java.util.function.BooleanSupplier deployAllowed) {
    // TODO: Superstructure injects a supplier that returns false while the turret is inside its
    // conflict arc. Stored, not evaluated, here. Consulted only for pivot motions whose path enters
    // or crosses the turret-conflict pivot band (see crossesTurretConflictBand); a motion that stays
    // outside the band - e.g. DEPLOYED <-> INTAKING, or a feed-assist angle below the band - never
    // waits on the turret.
  }

  @Override
  public void periodic() {
    // TODO: io.updateInputs(inputs); Logger.processInputs("Intake", inputs); apply tunable gains via
    // LoggedTunableGains.ifChanged; run the transition/entry/exit/apply/log sequence from section 2.1;
    // push PowerBudget scalar into io.setCurrentLimitScalar.
  }

  private SystemState handleStateTransition() {
    // TODO: switch (wantedState): HOME -> HOMING (or FAULTED if stateTimer exceeds
    // kHomingTimeoutSec); anything else while !isHomed -> HOMING; STOW -> STOWING then STOWED when
    // atGoal(); DEPLOY_AND_INTAKE -> DEPLOYING until atGoal() then INTAKING; DEPLOY_ONLY ->
    // DEPLOYING until atGoal() then DEPLOYED; FEED_ASSIST -> FEED_ASSISTING (only from a deployed
    // state - from STOWED it deploys first); EJECT -> EJECTING; CHARACTERIZE -> CHARACTERIZING;
    // IDLE -> IDLE. Any transition whose pivot motion crossesTurretConflictBand(...) holds the
    // current state while !deployAllowed.getAsBoolean(), rather than falling back to STOWED.
    return SystemState.IDLE;
  }

  private void onEnter(SystemState state) {
    // TODO: HOMING -> isHomed = false, homingSettleTimer.reset(); CHARACTERIZING -> zero the volts.
  }

  private void onExit(SystemState state) {
    // TODO: HOMING -> io.seedPivotPosition(Constants.Intake.kStowedAngleRad.get()) and isHomed = true
    // when the exit was successful (not a timeout).
  }

  private void applyState() {
    // TODO: switch (systemState) dispatching to the private handler methods below.
  }

  private void stowing() {
    // TODO: io.setPivotPosition(kStowedAngleRad); io.setRollerVelocity(0).
  }

  private void deploying() {
    // TODO: io.setPivotPosition(kDeployedAngleRad); io.setRollerVelocity(0) until atGoal().
  }

  private void intaking() {
    // TODO: io.setPivotPosition(kDeployedAngleRad); io.setRollerVelocity(kIntakeRollerRpm).
  }

  private void deployed() {
    // TODO: io.setPivotPosition(kDeployedAngleRad); io.setRollerVelocity(0). The resting posture
    // while carrying pieces - raising the arm would let them fall out.
  }

  private void feedAssisting() {
    // TODO: Rock: alternate the pivot goal between kDeployedAngleRad and kFeedAssistAngleRad every
    // half of kFeedAssistPeriodSec, timed from stateTimer. Hold kFeedAssistAngleRad still instead
    // when the period is 0 or the LOW PowerBudget scalar is below 1.0. Roller at
    // kFeedAssistRollerRpm (inward). Both ends are outside the conflict band on this robot; if a
    // mis-tuned kFeedAssistAngleRad ever puts one inside it while the deploy guard is shut, hold the
    // deployed end instead.
  }

  private void ejecting() {
    // TODO: io.setPivotPosition(kEjectAngleRad); io.setRollerVelocity(kEjectRollerRpm).
  }

  private void homing() {
    // TODO: io.setPivotOpenLoop(12.0 * kHomingDutyCycle). When
    // Math.abs(inputs.pivotVelocityRadPerSec) < kHomingVelocityThresholdRadPerSec, start
    // homingSettleTimer; when it passes kHomingSettleSec, mark homed. Bail to FAULTED after
    // kHomingTimeoutSec.
  }

  private void characterizing() {
    // TODO: io.setPivotOpenLoop(characterizationVolts) - driven by CharacterizationCommands.
  }

  private void faulted() {
    // TODO: io.stop() and hold. Only a HOME request clears FAULTED.
  }

  public void setWantedState(WantedState wanted) {
    // TODO: Assign. If wanted == HOME, set isHomed = false so the machine actually re-homes.
  }

  public Command setWantedStateCommand(WantedState wanted) {
    // TODO: return runOnce(() -> setWantedState(wanted)) - the command-based adapter for bindings.
    return null;
  }

  public void setCharacterizationVolts(double volts) {
    // TODO: Setter used by CharacterizationCommands while systemState == CHARACTERIZING.
  }

  @AutoLogOutput(key = "Intake/AtGoal")
  public boolean atGoal() {
    // TODO: |pivotPositionRad - goalRad| < kPivotToleranceRad.
    return false;
  }

  @AutoLogOutput(key = "Intake/IsHomed")
  public boolean isHomed() {
    // TODO: Getter.
    return false;
  }

  @AutoLogOutput(key = "Intake/IsStowed")
  public boolean isStowed() {
    // TODO: systemState == STOWED. Logged for the driver; the turret interlock no longer uses it.
    return false;
  }

  @AutoLogOutput(key = "Intake/InTurretConflictBand")
  public boolean isInTurretConflictBand() {
    // TODO: The pivot's measured angle OR its commanded goal lies within kTurretConflictBandRad of
    // kStowedAngleRad (on the deployed side), or the path between them crosses that band. True
    // while STOWED - that is the real conflict. The turret asks this before entering or sweeping
    // through its conflict arc. Replaces the old "!isStowed()" test, which had it backwards: it
    // treated a DEPLOYED intake as the conflict (section 2.4).
    return false;
  }

  @AutoLogOutput(key = "Intake/RetainingPieces")
  public boolean isRetainingPieces() {
    // TODO: The pivot's measured angle lies between kDeployedAngleRad and kFeedAssistAngleRad,
    // inclusive (plus kPivotToleranceRad). Pieces can fall out of a stowed intake but stay in
    // anywhere in this range. Autos wait on this before their first path.
    return false;
  }

  private boolean crossesTurretConflictBand(double fromRad, double toRad) {
    // TODO: The closed interval [min(from,to), max(from,to)] intersects the band. Plain interval
    // arithmetic: the pivot's travel is far less than a full turn, unlike the turret's.
    return false;
  }

  public double getPivotAngleRad() {
    // TODO: inputs.pivotPositionRad - published for the AdvantageScope 3D component pose.
    return 0.0;
  }

  public double getCharacterizationVelocityRadPerSec() {
    // TODO: inputs.pivotVelocityRadPerSec.
    return 0.0;
  }
}
```

---

### 4.5 Floor roller

#### `src/main/java/frc/robot/subsystems/floorroller/FloorRollerIO.java` [NEW]

```java
package frc.robot.subsystems.floorroller;

import org.littletonrobotics.junction.AutoLog;

public interface FloorRollerIO {
  @AutoLog
  public static class FloorRollerIOInputs {
    public boolean connected = false;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(FloorRollerIOInputs inputs) {}

  public default void setVelocity(double velocityRpm) {}

  public default void setOpenLoop(double volts) {}

  public default void setGains(com.ctre.phoenix6.configs.Slot0Configs gains) {}

  public default void setCurrentLimitScalar(double scalar) {}

  public default void stop() {}
}
```

#### `src/main/java/frc/robot/subsystems/floorroller/FloorRollerIOTalonFX.java` [NEW]

Single Kraken X60 on the RIO bus, closed-loop velocity with `VelocityVoltage`.

```java
package frc.robot.subsystems.floorroller;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FloorRollerIOTalonFX implements FloorRollerIO {
  private final TalonFX motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public FloorRollerIOTalonFX() {
    // TODO: new TalonFX(Constants.Can.kFloorRollerId, Constants.Can.kRioBus); configure
    // Feedback.SensorToMechanismRatio = kGearRatio, Slot0 gains, Coast neutral mode (a roller should
    // not fight the game piece when disabled), stator limit, and inversion. Register signals at
    // 50 Hz and optimize the bus.
  }

  @Override
  public void updateInputs(FloorRollerIOInputs inputs) {
    // TODO: refreshAll -> Debouncer -> fill inputs; velocityRpm = velocity.getValueAsDouble() * 60.
  }

  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: motor.setControl(velocityRequest.withVelocity(velocityRpm / 60.0)).
  }

  @Override
  public void setOpenLoop(double volts) {
    // TODO: motor.setControl(voltageRequest.withOutput(volts)).
  }

  @Override
  public void setGains(Slot0Configs gains) {
    // TODO: tryUntilOk apply.
  }

  @Override
  public void setCurrentLimitScalar(double scalar) {
    // TODO: Scale kStatorLimitAmps; this roller is PowerBudget.Priority.LOW and gives up amps first.
  }

  @Override
  public void stop() {
    // TODO: motor.stopMotor().
  }
}
```

#### `src/main/java/frc/robot/subsystems/floorroller/FloorRollerIOSim.java` [NEW]

```java
package frc.robot.subsystems.floorroller;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FloorRollerIOSim implements FloorRollerIO {
  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;

  public FloorRollerIOSim() {
    // TODO: DCMotorSim from LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001,
    // Constants.FloorRoller.kGearRatio).
  }

  @Override
  public void updateInputs(FloorRollerIOInputs inputs) {
    // TODO: Compute appliedVolts from ff + PID when closedLoop, clamp, sim.update(0.02), fill inputs.
  }

  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: closedLoop = true; ffVolts = kKs*sign + kKv*rpm/60; controller.setSetpoint(rad/s).
  }

  @Override
  public void setOpenLoop(double volts) {
    // TODO: closedLoop = false; appliedVolts = volts.
  }

  @Override
  public void setGains(Slot0Configs gains) {
    // TODO: controller.setPID(...).
  }

  @Override
  public void stop() {
    // TODO: Zero everything.
  }
}
```

#### `src/main/java/frc/robot/subsystems/floorroller/FloorRoller.java` [NEW]

The simplest FSM in the project, and deliberately still an FSM: uniformity means a student who can
read one subsystem can read all six, and it gives the floor roller the same timeout and logging
behavior as everything else.

> **Replanned (§2.4).** The floor roller runs only to feed the indexer, never while intaking, so it
> has no `INTAKE` request and no `INTAKING` state. M5 built them; M10b step 8 deleted them.

```java
package frc.robot.subsystems.floorroller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class FloorRoller extends SubsystemBase {
  public enum WantedState {
    IDLE,
    FEED,
    EJECT,
    CHARACTERIZE
  }

  public enum SystemState {
    IDLE,
    FEEDING,
    EJECTING,
    CHARACTERIZING
  }

  private final FloorRollerIO io;
  private final FloorRollerIOInputsAutoLogged inputs = new FloorRollerIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();
  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private double characterizationVolts = 0.0;

  public FloorRoller(FloorRollerIO io) {
    // TODO: Store io, start stateTimer, register a FaultReporter disconnect check.
  }

  @Override
  public void periodic() {
    // TODO: Standard five-step periodic from section 2.1, plus PowerBudget scalar push at
    // Priority.LOW.
  }

  private SystemState handleStateTransition() {
    // TODO: Direct WantedState -> SystemState mapping; no guards needed for a roller.
    return SystemState.IDLE;
  }

  private void applyState() {
    // TODO: switch dispatching io.setVelocity(kFeedRpm / kEjectRpm), io.stop(), or
    // io.setOpenLoop(characterizationVolts).
  }

  public void setWantedState(WantedState wanted) {
    // TODO: Assign.
  }

  public Command setWantedStateCommand(WantedState wanted) {
    // TODO: runOnce adapter.
    return null;
  }

  public void setCharacterizationVolts(double volts) {
    // TODO: Setter for CharacterizationCommands.
  }

  @AutoLogOutput(key = "FloorRoller/AtSpeed")
  public boolean atSpeed() {
    // TODO: |velocityRpm - goalRpm| < a tunable tolerance; used by the Superstructure's feed gate.
    return false;
  }

  public double getVelocityRpm() {
    // TODO: inputs.velocityRpm.
    return 0.0;
  }
}
```

---

### 4.6 Indexer

> **Replanned (§2.4).** The indexer no longer answers "do I have a game piece". The robot carries a
> hopper and has no piece sensor, so the indexer's job is reduced to what it can actually do well:
> feed a stream on request, notice when that stream jams, and report whether it is pushing against
> anything. Shot counting has moved to `Shooter`. `feature-indexer` implements the superseded
> single-piece version; M10b brings it in line.

#### `src/main/java/frc/robot/subsystems/indexer/IndexerIO.java` [NEW]

Same shape as the floor roller. There is no beam-break input: this robot has no piece sensor, and
the interface does not pretend otherwise.

```java
package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected = false;
    public double positionRotations = 0.0;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVelocity(double velocityRpm) {}

  public default void setOpenLoop(double volts) {}

  public default void setGains(com.ctre.phoenix6.configs.Slot0Configs gains) {}

  public default void setCurrentLimitScalar(double scalar) {}

  public default void stop() {}
}
```

#### `src/main/java/frc/robot/subsystems/indexer/IndexerIOTalonFX.java` [NEW]

```java
package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IndexerIOTalonFX() {
    // TODO: Same construction pattern as FloorRollerIOTalonFX using Constants.Can.kIndexerId, but
    // use Brake neutral mode: a stopped indexer is the only thing between the hopper and the
    // flywheel while the floor roller pushes the pile toward it.
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // TODO: refreshAll and fill inputs.
  }

  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: velocityRequest.withVelocity(rpm / 60.0).
  }

  @Override
  public void setOpenLoop(double volts) {
    // TODO: voltageRequest.withOutput(volts).
  }

  @Override
  public void setGains(Slot0Configs gains) {
    // TODO: tryUntilOk apply.
  }

  @Override
  public void setCurrentLimitScalar(double scalar) {
    // TODO: Scale kStatorLimitAmps; Priority.MEDIUM.
  }

  @Override
  public void stop() {
    // TODO: motor.stopMotor().
  }
}
```

#### `src/main/java/frc/robot/subsystems/indexer/IndexerIOSim.java` [NEW]

```java
package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean closedLoop = false;
  private boolean simulatedLoad = false;
  private boolean simulatedJam = false;
  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;

  public IndexerIOSim() {
    // TODO: DCMotorSim as in FloorRollerIOSim.
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // TODO: Standard sim update. When simulatedLoad, report current above kLoadedCurrentAmps (and
    // below kJamCurrentAmps) at full speed while the motor is driven forward. When
    // simulatedJam, clamp velocity near zero and report current above kJamCurrentAmps whenever
    // the motor is driven FORWARD (a reverse pulse clears nothing by itself; the test clears the
    // jam, which is what lets it prove UNJAMMING retries and then gives up).
  }

  public void setSimulatedLoad(boolean loaded) {
    // TODO: Test hook: "the hopper is pressing pieces into the indexer". Replaces the old
    // setSimulatedPiecePresent - there is no longer a piece to be present or absent.
  }

  public void setSimulatedJam(boolean jammed) {
    // TODO: Test hook for the jam path.
  }

  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: closedLoop = true; set ff + setpoint.
  }

  @Override
  public void setOpenLoop(double volts) {
    // TODO: closedLoop = false; appliedVolts = volts.
  }

  @Override
  public void setGains(Slot0Configs gains) {
    // TODO: controller.setPID(...).
  }

  @Override
  public void stop() {
    // TODO: Zero.
  }
}
```

#### `src/main/java/frc/robot/subsystems/indexer/Indexer.java` [NEW]

The gate between the hopper and the flywheel. It runs when the Superstructure says the shot is
ready and stops when it is not; it does not decide either. What it *does* own is jam recovery,
because only the indexer can see the stall that defines one, and a loaded-while-feeding diagnostic
that the Superstructure uses to corroborate its "probably empty" belief.

Deliberately **not** here any more: `hasPiece()`, a shot counter, `INDEX`/`HOLD`. The old
fallback's hardest code — replaying the last loaded sample while stopped so a held piece was not
"forgotten" — existed only because the question was being asked of a stopped wheel. The load
predicate is now evaluated only while the wheel is being driven forward, so that whole class of
bug cannot occur.

```java
package frc.robot.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Indexer extends SubsystemBase {
  public enum WantedState {
    IDLE,
    FEED,
    EJECT,
    CHARACTERIZE
  }

  public enum SystemState {
    IDLE,
    FEEDING,
    UNJAMMING,
    JAMMED,
    EJECTING,
    CHARACTERIZING
  }

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();
  private final Debouncer loadedDebouncer = new Debouncer(0.0, Debouncer.DebounceType.kBoth);
  private final Debouncer jamDebouncer = new Debouncer(0.0, Debouncer.DebounceType.kRising);

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private boolean loaded = false;
  private int unjamAttempts = 0;
  private double characterizationVolts = 0.0;

  public Indexer(IndexerIO io) {
    // TODO: Store io, start stateTimer, register the motor with FaultReporter, and create a
    // warning Alert for JAMMED ("Indexer jammed; EJECT to clear").
  }

  @Override
  public void periodic() {
    // TODO: Standard five-step periodic. Between steps 1 and 2 call updateLoadAndJam() so the
    // transition sees this loop's stall. Log loaded, unjamAttempts and goal/error RPM.
  }

  private void updateLoadAndJam() {
    // TODO: Push kLoadedDebounceSec / kJamDetectSec into the debouncers via ifChanged. Only while
    // systemState == FEEDING (motor driven forward and past a short settle after the goal
    // changed): loaded = loadedDebouncer.calculate(current > kLoadedCurrentAmps); jamDetected = jamDebouncer.calculate(current > kJamCurrentAmps &&
    // velocity < kJamVelocityRpm). In every other state loaded = false - the question has no
    // meaning for a stopped or reversing wheel, and answering it anyway is what made the old
    // single-piece detector so intricate.
  }

  private SystemState handleStateTransition() {
    // TODO: FEED -> FEEDING; FEEDING + jamDetected -> UNJAMMING (unjamAttempts++); UNJAMMING after
    // kUnjamSec -> FEEDING; a jam with unjamAttempts == kMaxUnjamAttempts -> JAMMED instead, which
    // latches until the wanted state changes (the same "latch until intent changes" rule the eject
    // timeout and Shooter's fault use). EJECT -> EJECTING; CHARACTERIZE -> CHARACTERIZING;
    // IDLE -> IDLE. setWantedState resets unjamAttempts whenever the request changes.
    return SystemState.IDLE;
  }

  private void applyState() {
    // TODO: IDLE/JAMMED -> io.setVelocity(0) (brake holds the pile); FEEDING -> kFeedRpm;
    // UNJAMMING -> kUnjamRpm; EJECTING -> kEjectRpm; CHARACTERIZING ->
    // io.setOpenLoop(characterizationVolts).
  }

  public void setWantedState(WantedState wanted) {
    // TODO: Assign; reset unjamAttempts on change.
  }

  public Command setWantedStateCommand(WantedState wanted) {
    // TODO: runOnce adapter.
    return null;
  }

  @AutoLogOutput(key = "Indexer/Loaded")
  public boolean isLoaded() {
    // TODO: Getter. True only while FEEDING and pushing against something. Superstructure's
    // likelyEmpty() clock does not advance while this is true.
    return false;
  }

  @AutoLogOutput(key = "Indexer/Unjamming")
  public boolean isUnjamming() {
    // TODO: systemState == UNJAMMING. Superstructure reverses the floor roller in step with it and
    // pauses the likelyEmpty() clock.
    return false;
  }

  public boolean isJammed() {
    // TODO: systemState == JAMMED.
    return false;
  }

  public void setCharacterizationVolts(double volts) {
    // TODO: Setter.
  }

  public double getVelocityRpm() {
    // TODO: inputs.velocityRpm.
    return 0.0;
  }
}
```

---

### 4.7 Turret

#### `src/main/java/frc/robot/subsystems/turret/TurretIO.java` [NEW]

```java
package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public boolean absEncoderConnected = false;
    /** Continuous (unwrapped) turret position in the robot frame, radians. */
    public double positionRad = 0.0;

    /** Raw absolute encoder reading mapped into [-pi, pi), radians. */
    public double absolutePositionRad = 0.0;

    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double closedLoopErrorRad = 0.0;
    public double closedLoopReferenceRad = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Motion-Magic position control with an added velocity feedforward.
   *
   * @param positionRad continuous target position in the turret's own frame
   * @param velocityFeedforwardRadPerSec expected target angular rate, used to cancel chassis rotation
   */
  public default void setPositionSetpoint(double positionRad, double velocityFeedforwardRadPerSec) {}

  public default void setOpenLoop(double volts) {}

  public default void seedPosition(double positionRad) {}

  public default void setGains(com.ctre.phoenix6.configs.Slot0Configs gains) {}

  public default void setSoftLimits(double minRad, double maxRad) {}

  public default void setBrakeMode(boolean brake) {}

  public default void setCurrentLimitScalar(double scalar) {}

  public default void stop() {}
}
```

#### `src/main/java/frc/robot/subsystems/turret/TurretIOTalonFX.java` [NEW]

Kraken X44 with a WCP ThroughBore on the gearbox output, read on a roboRIO DIO. The Talon's
software limit switches are set from the same tunable limits the unwrapper uses, so even a software
bug cannot drive past the hard stops. `ContinuousWrap` is explicitly **off** — this is a
finite-travel turret, not a swerve azimuth, and letting Phoenix wrap the setpoint would defeat the
whole unwrap design.

```java
package frc.robot.subsystems.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;
  private final DutyCycleEncoder absEncoder;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Double> closedLoopError;
  private final StatusSignal<Double> closedLoopReference;
  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public TurretIOTalonFX() {
    // TODO: new TalonFX(Constants.Can.kTurretId, Constants.Can.kRioBus). Configure:
    // Feedback.SensorToMechanismRatio = Constants.Turret.kGearRatio; Slot0 from the tunable gains;
    // MotionMagic cruise/accel; ClosedLoopGeneral.ContinuousWrap = FALSE (critical - this turret has
    // hard stops); SoftwareLimitSwitch forward/reverse enabled at kMax/kMinAngleRad minus
    // kLimitMarginRad; Brake neutral mode; stator limit kStatorLimitAmps. Create the DutyCycleEncoder
    // on Constants.Dio.kTurretAbsEncoder. Register signals at 100 Hz (the shot solver wants a fresh
    // turret angle) and optimize the bus. Finally seed position from the absolute encoder.
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // TODO: refreshAll; fill inputs. absolutePositionRad = MathUtil.angleModulus(
    // Units.rotationsToRadians(absEncoder.get() - kAbsEncoderOffsetRot.get())). Read
    // closedLoopError/closedLoopReference via motor.getClosedLoopError()/getClosedLoopReference()
    // (verify these signal getters exist in Phoenix6 26.1.2) and convert rotations to radians.
  }

  @Override
  public void setPositionSetpoint(double positionRad, double velocityFeedforwardRadPerSec) {
    // TODO: motor.setControl(positionRequest
    //   .withPosition(Units.radiansToRotations(positionRad))
    //   .withFeedForward(kS*Math.signum(vff) + kV*Units.radiansToRotations(vff)));
    // The feedforward is in volts because the request is MotionMagicVoltage. Verify the
    // withFeedForward unit convention against Phoenix6 26.1.2 - it is volts for *Voltage requests
    // and amps for *TorqueCurrentFOC requests.
  }

  @Override
  public void setOpenLoop(double volts) {
    // TODO: motor.setControl(voltageRequest.withOutput(volts)).
  }

  @Override
  public void seedPosition(double positionRad) {
    // TODO: tryUntilOk(5, () -> motor.setPosition(Units.radiansToRotations(positionRad), 0.25)).
  }

  @Override
  public void setGains(Slot0Configs gains) {
    // TODO: tryUntilOk apply.
  }

  @Override
  public void setSoftLimits(double minRad, double maxRad) {
    // TODO: Apply a SoftwareLimitSwitchConfigs with both limits enabled, converted to rotations.
    // Called whenever the limit tunables change so a retune cannot outrun the firmware guard.
  }

  @Override
  public void setBrakeMode(boolean brake) {
    // TODO: MotorOutputConfigs neutral-mode apply.
  }

  @Override
  public void setCurrentLimitScalar(double scalar) {
    // TODO: Scale kStatorLimitAmps; Priority.HIGH (the turret is nearly as important as the drive).
  }

  @Override
  public void stop() {
    // TODO: motor.stopMotor().
  }
}
```

#### `src/main/java/frc/robot/subsystems/turret/TurretIOSim.java` [NEW]

```java
package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim sim;
  private final ProfiledPIDController controller;
  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;
  private double minRad = 0.0;
  private double maxRad = 0.0;

  public TurretIOSim() {
    // TODO: DCMotorSim from LinearSystemId.createDCMotorSystem with the X44 gearbox (see the note in
    // IntakeIOSim about DCMotor factories), Constants.Turret.kMoiKgM2 and kGearRatio. Build the
    // ProfiledPIDController from the tunable gains and Motion Magic constraints.
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // TODO: Run the controller when closedLoop, add ffVolts, clamp to +-12 V, update the sim, and
    // CLAMP the simulated position to [minRad, maxRad] with zeroed velocity at the stop - the sim
    // must reproduce hard stops or the unwrap logic will never be exercised before the robot runs.
  }

  @Override
  public void setPositionSetpoint(double positionRad, double velocityFeedforwardRadPerSec) {
    // TODO: closedLoop = true; controller.setGoal(positionRad); ffVolts = kV * vff.
  }

  @Override
  public void setOpenLoop(double volts) {
    // TODO: closedLoop = false; appliedVolts = volts.
  }

  @Override
  public void seedPosition(double positionRad) {
    // TODO: sim.setState(positionRad, 0.0).
  }

  @Override
  public void setGains(Slot0Configs gains) {
    // TODO: controller.setPID(...).
  }

  @Override
  public void setSoftLimits(double minRad, double maxRad) {
    // TODO: Store both for the clamp in updateInputs.
  }

  @Override
  public void stop() {
    // TODO: Zero.
  }
}
```

#### `src/main/java/frc/robot/subsystems/turret/Turret.java` [NEW]

The most subtle subsystem in the project. It converts a *field-relative* aim azimuth into a
*continuous, in-range, robot-relative* turret setpoint via `TurretUnwrapper`, applies a velocity
feedforward equal to (target's angular rate) − (chassis yaw rate) so the turret tracks while the
drivetrain spins, publishes its angle into `RobotState`'s interpolation buffer every loop for the
moving-camera transform, and refuses to sweep through its intake-conflict arc while the intake's
pivot is inside, or moving through, the turret-conflict pivot band (§2.4 — a *deployed* intake is
outside that band, so the turret aims freely while the robot intakes).
When the desired azimuth is unreachable it enters `OUT_OF_RANGE` and reports that to the
Superstructure, which then asks the drivetrain to rotate — no jammed belts.

```java
package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Turret extends SubsystemBase {
  public enum WantedState {
    IDLE,
    STOW,
    AIM,
    MANUAL,
    HOME,
    CHARACTERIZE
  }

  public enum SystemState {
    IDLE,
    STOWING,
    AIMING,
    OUT_OF_RANGE,
    UNWRAPPING,
    MANUAL,
    HOMING,
    CHARACTERIZING,
    FAULTED
  }

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private Rotation2d goalFieldAngle = Rotation2d.kZero;
  private double goalFieldAngularVelocityRadPerSec = 0.0;
  private double manualVolts = 0.0;
  private double characterizationVolts = 0.0;
  private double lastCommandedPositionRad = 0.0;
  private boolean isHomed = false;
  private BooleanSupplier intakeArcBlocked = () -> false;

  public Turret(TurretIO io) {
    // TODO: Store io, start stateTimer, register FaultReporter checks (motorConnected,
    // absEncoderConnected), and push the initial soft limits with io.setSoftLimits.
  }

  public void setIntakeArcGuard(BooleanSupplier blocked) {
    // TODO: Superstructure injects intake::isInTurretConflictBand; stored, evaluated each loop.
    // (Originally () -> !intake.isStowed(), which wrongly treated a deployed intake as a conflict.)
  }

  @Override
  public void periodic() {
    // TODO: Standard five-step periodic. Additionally: (a) call
    // RobotState.getInstance().addTurretObservation(Timer.getFPGATimestamp(), inputs.positionRad)
    // immediately after processInputs so the buffer is as fresh as possible; (b) re-apply soft limits
    // and gains through LoggedTunableNumber.ifChanged; (c) log the unwrap solution and the
    // field-relative angle the turret is actually pointing.
  }

  private SystemState handleStateTransition() {
    // TODO: HOME -> HOMING (FAULTED on timeout); !isHomed -> HOMING; MANUAL -> MANUAL;
    // CHARACTERIZE -> CHARACTERIZING; STOW -> STOWING; AIM -> compute the unwrap result: if
    // !result.reachable() -> OUT_OF_RANGE; else if result.requiresLongWayAround() -> UNWRAPPING
    // until |error| is small, then AIMING; else AIMING. If intakeArcBlocked and the solution passes
    // through the conflict arc, stay in OUT_OF_RANGE.
    return SystemState.IDLE;
  }

  private void onEnter(SystemState state) {
    // TODO: HOMING -> isHomed = false; UNWRAPPING -> latch the chosen branch so it cannot flip
    // mid-sweep; OUT_OF_RANGE -> log a one-shot event.
  }

  private void onExit(SystemState state) {
    // TODO: HOMING -> io.seedPosition from inputs.absolutePositionRad and set isHomed = true.
  }

  private void applyState() {
    // TODO: switch dispatching to the handlers below.
  }

  private void aiming() {
    // TODO: double robotRelGoal = goalFieldAngle.getRadians()
    //         - RobotState.getInstance().getRotation().getRadians();
    // TurretUnwrapper.Result r = TurretUnwrapper.solve(inputs.positionRad, robotRelGoal,
    //     kMinAngleRad.get(), kMaxAngleRad.get(), kUnwrapHysteresisRad.get());
    // double ff = goalFieldAngularVelocityRadPerSec
    //         - RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
    // io.setPositionSetpoint(r.targetPositionRad(), ff); lastCommandedPositionRad = r.targetPositionRad().
  }

  private void unwrapping() {
    // TODO: Same as aiming() but with the velocity feedforward forced to zero and the latched branch
    // target, because during a 300-degree sweep the chassis-cancelling feedforward is meaningless.
  }

  private void outOfRange() {
    // TODO: Hold the closest reachable angle to the goal (clamped into the limits) so the turret is
    // already as close as possible when the drivetrain finishes rotating.
  }

  private void stowing() {
    // TODO: io.setPositionSetpoint(kStowAngleRad.get(), 0.0).
  }

  private void homing() {
    // TODO: If absEncoderConnected, seed immediately and finish in one loop. Otherwise drive
    // open-loop toward one hard stop at a low duty cycle, watch for the velocity to settle, seed, and
    // time out to FAULTED.
  }

  private void manual() {
    // TODO: io.setOpenLoop(manualVolts), clamped, and refuse to drive past the soft limits.
  }

  private void characterizing() {
    // TODO: io.setOpenLoop(characterizationVolts).
  }

  public void setGoal(Rotation2d fieldAngle, double fieldAngularVelocityRadPerSec) {
    // TODO: Store both and set wantedState = AIM. Called from Superstructure.applyState() with the
    // solver output, every loop.
  }

  public void setWantedState(WantedState wanted) {
    // TODO: Assign; HOME clears isHomed.
  }

  public Command setWantedStateCommand(WantedState wanted) {
    // TODO: runOnce adapter.
    return null;
  }

  public void setManualVolts(double volts) {
    // TODO: Operator nudge input.
  }

  public void setCharacterizationVolts(double volts) {
    // TODO: Setter.
  }

  @AutoLogOutput(key = "Turret/PositionRad")
  public double getPositionRad() {
    // TODO: inputs.positionRad.
    return 0.0;
  }

  @AutoLogOutput(key = "Turret/FieldAngle")
  public Rotation2d getFieldAngle() {
    // TODO: Rotation2d.fromRadians(inputs.positionRad).plus(RobotState.getInstance().getRotation()).
    // This is what the AdvantageScope 3D turret component and the vision transform both key off.
    return Rotation2d.kZero;
  }

  public double getVelocityRadPerSec() {
    // TODO: inputs.velocityRadPerSec - Vision uses this to reject frames taken while slewing.
    return 0.0;
  }

  @AutoLogOutput(key = "Turret/AtGoal")
  public boolean atGoal() {
    // TODO: |closedLoopErrorRad| < kToleranceRad AND |velocityRadPerSec - ff| <
    // kVelocityToleranceRadPerSec. Both terms matter: a turret sweeping through the right angle at
    // speed is not on target.
    return false;
  }

  @AutoLogOutput(key = "Turret/IsOutOfRange")
  public boolean isOutOfRange() {
    // TODO: systemState == OUT_OF_RANGE. Superstructure reads this to decide to rotate the chassis.
    return false;
  }

  public boolean isHomed() {
    // TODO: Getter.
    return false;
  }

  public double getCharacterizationVelocityRadPerSec() {
    // TODO: inputs.velocityRadPerSec.
    return 0.0;
  }
}
```

---

### 4.8 Shooter (flywheel)

#### `src/main/java/frc/robot/subsystems/shooter/ShooterIO.java` [NEW]

Two Kraken X60s in leader/follower. Both motors' telemetry is logged separately, because a follower
that has lost its `Follower` request draws almost no current while the leader draws double — a
failure that is invisible unless you log both.

```java
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public double velocityRpm = 0.0;
    public double goalRpm = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double followerAppliedVolts = 0.0;
    public double leaderStatorCurrentAmps = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;
    public double followerTempCelsius = 0.0;
    public double closedLoopErrorRpm = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Closed-loop flywheel velocity in RPM at the flywheel shaft. */
  public default void setVelocity(double velocityRpm) {}

  public default void setOpenLoop(double volts) {}

  public default void setGains(com.ctre.phoenix6.configs.Slot0Configs gains) {}

  public default void setCurrentLimitScalar(double scalar) {}

  public default void setNeutralMode(boolean brake) {}

  public default void stop() {}
}
```

#### `src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java` [NEW]

```java
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leader;
  private final TalonFX follower;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> leaderVolts;
  private final StatusSignal<Voltage> followerVolts;
  private final StatusSignal<Current> leaderStator;
  private final StatusSignal<Current> followerStator;
  private final StatusSignal<Current> leaderSupply;
  private final StatusSignal<Current> followerSupply;
  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<Temperature> followerTemp;
  private final Debouncer leaderDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private double goalRpm = 0.0;

  public ShooterIOTalonFX() {
    // TODO: Construct both TalonFX on Constants.Can.kRioBus. Configure the leader with
    // Feedback.SensorToMechanismRatio = kGearRatio, Slot0 gains, Coast neutral mode (never brake a
    // flywheel), stator limit kStatorLimitAmps, supply limit kSupplyLimitAmps with a supply-time
    // threshold, and inversion. Then
    // follower.setControl(new Follower(Constants.Can.kFlywheelLeaderId,
    //     Constants.Shooter.kFollowerOpposesLeader));
    // Re-issue the Follower request in setVelocity/setOpenLoop as well - a power cycle of the
    // follower otherwise leaves it idle. Register signals at 100 Hz and optimize the bus.
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // TODO: refreshAll, fill all fields, velocityRpm = velocity.getValueAsDouble() * 60,
    // closedLoopErrorRpm = goalRpm - velocityRpm. Separate Debouncers give independent
    // leaderConnected/followerConnected.
  }

  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: goalRpm = velocityRpm; leader.setControl(velocityRequest.withVelocity(velocityRpm/60.0));
    // and re-assert the Follower request on the follower.
  }

  @Override
  public void setOpenLoop(double volts) {
    // TODO: leader.setControl(voltageRequest.withOutput(volts)) plus the Follower re-assert.
  }

  @Override
  public void setGains(Slot0Configs gains) {
    // TODO: tryUntilOk apply to the leader only (the follower mirrors output, not control).
  }

  @Override
  public void setCurrentLimitScalar(double scalar) {
    // TODO: Scale both limits on both motors; Priority.HIGH.
  }

  @Override
  public void setNeutralMode(boolean brake) {
    // TODO: Apply to both. Normally always coast; brake only exists for a pit-safety mode.
  }

  @Override
  public void stop() {
    // TODO: leader.stopMotor(); the follower coasts with it.
  }
}
```

#### `src/main/java/frc/robot/subsystems/shooter/ShooterIOSim.java` [NEW]

```java
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim sim;
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;
  private double goalRpm = 0.0;

  public ShooterIOSim() {
    // TODO: FlywheelSim from LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(2),
    // Constants.Shooter.kMoiKgM2, Constants.Shooter.kGearRatio), with DCMotor.getKrakenX60Foc(2) as
    // the gearbox argument. Two motors, one inertia.
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // TODO: appliedVolts = ffVolts + (closedLoop ? controller.calculate(sim.getAngularVelocityRPM())
    // : 0), clamp, sim.setInputVoltage, sim.update(0.02), then mirror the same values into both the
    // leader and follower input fields so sim and real logs have the same shape.
  }

  @Override
  public void setVelocity(double velocityRpm) {
    // TODO: closedLoop = true; goalRpm = velocityRpm;
    // ffVolts = kKs*Math.signum(rpm) + kKv*rpm/60; controller.setSetpoint(velocityRpm).
  }

  @Override
  public void setOpenLoop(double volts) {
    // TODO: closedLoop = false; appliedVolts = volts.
  }

  @Override
  public void setGains(Slot0Configs gains) {
    // TODO: controller.setPID(...).
  }

  public void injectShot() {
    // TODO: Test hook: one game piece passes through the wheels. Remove a tunable fraction of the
    // wheel's kinetic energy in a single update (a speed step down), which is what the shot
    // detector keys on. Called by tests and by a sim keyboard binding, so bursts of N pieces can be
    // replayed at any cadence with no maple-sim.
  }

  @Override
  public void stop() {
    // TODO: Zero.
  }
}
```

#### `src/main/java/frc/robot/subsystems/shooter/Shooter.java` [NEW]

Flywheel FSM with one capability that matters for shooting on the move: *predictive* readiness. A
flywheel recovering from a shot takes a fixed, measurable time to return to speed; rather than
waiting for the error to actually fall inside tolerance, `willBeAtSpeedIn(double seconds)` projects
the current velocity forward using the measured spin-up slope, so the Superstructure can start the
feed a few tens of milliseconds early and keep the shot cadence up. It also holds an idle RPM
between shots — spinning from idle instead of from zero roughly halves recovery time.

It also **counts shots** (§2.4). With a hopper and no piece sensor, the flywheel is the most
reliable piece detector on the robot: every piece that scores passes through it and takes a sharp
bite out of wheel speed. A loop-to-loop drop larger than `kShotDipRpm`, while the wheel was already
within `kShotArmToleranceRpm` of its goal, is one shot. After it, further drops are ignored for
`kShotRefractorySec`. The arming condition is what stops a goal change or a spin-up from counting,
and the refractory window is what stops one piece's dip from counting twice. The count is a
post-match diagnostic. `getSecondsSinceLastShot()` is the live signal, because the Superstructure's
`likelyEmpty()` depends on it.

```java
package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase {
  public enum WantedState {
    IDLE,
    COAST,
    SPIN_UP,
    SHOOT,
    CHARACTERIZE
  }

  public enum SystemState {
    IDLE,
    COASTING,
    SPINNING_UP,
    AT_SPEED,
    CHARACTERIZING,
    FAULTED
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Timer stateTimer = new Timer();
  private final LinearFilter accelFilter = LinearFilter.movingAverage(5);

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;
  private double goalRpm = 0.0;
  private double lastVelocityRpm = 0.0;
  private double accelRpmPerSec = 0.0;
  private double characterizationVolts = 0.0;
  private int shotCount = 0;
  private final Timer sinceLastShot = new Timer();

  public Shooter(ShooterIO io) {
    // TODO: Store io, start stateTimer, register FaultReporter checks for BOTH motors separately.
  }

  @Override
  public void periodic() {
    // TODO: Standard five-step periodic. Also differentiate inputs.velocityRpm through accelFilter
    // into accelRpmPerSec, push gains via ifChanged, push the PowerBudget scalar at Priority.HIGH,
    // and log goalRpm, error, and both motors' currents. Call detectShot() after inputs are fresh.
  }

  private void detectShot() {
    // TODO: armed = goalRpm > 0 && |lastVelocityRpm - goalRpm| < kShotArmToleranceRpm. If armed &&
    // (lastVelocityRpm - inputs.velocityRpm) > kShotDipRpm && sinceLastShot.hasElapsed(
    // kShotRefractorySec): shotCount++, sinceLastShot.restart(), log Shooter/ShotDetected true for
    // this loop. The signal is registered at 100 Hz but this runs at the 50 Hz loop rate. If a
    // logged burst shows back-to-back pieces merging into one dip, the fix is to count dips in the
    // timestamped high-rate samples (the PhoenixOdometryThread pattern), not to shrink the window.
  }

  private SystemState handleStateTransition() {
    // TODO: CHARACTERIZE -> CHARACTERIZING; IDLE -> IDLE; COAST -> COASTING; SPIN_UP/SHOOT ->
    // SPINNING_UP until atSpeed(), then AT_SPEED; if SPINNING_UP exceeds kSpinUpTimeoutSec ->
    // FAULTED (a jammed or unpowered flywheel must not silently block the scoring state forever).
    return SystemState.IDLE;
  }

  private void applyState() {
    // TODO: IDLE -> io.setVelocity(kIdleRpm); COASTING -> io.stop(); SPINNING_UP/AT_SPEED ->
    // io.setVelocity(goalRpm); CHARACTERIZING -> io.setOpenLoop(characterizationVolts);
    // FAULTED -> io.stop().
  }

  public void setGoal(double rpm) {
    // TODO: goalRpm = MathUtil.clamp(rpm, 0, kMaxRpm.get()); wantedState = SPIN_UP. Called every loop
    // from Superstructure with the solver's RPM.
  }

  public void setWantedState(WantedState wanted) {
    // TODO: Assign.
  }

  public Command setWantedStateCommand(WantedState wanted) {
    // TODO: runOnce adapter.
    return null;
  }

  public void setCharacterizationVolts(double volts) {
    // TODO: Setter.
  }

  @AutoLogOutput(key = "Shooter/AtSpeed")
  public boolean atSpeed() {
    // TODO: |inputs.velocityRpm - goalRpm| < kToleranceRpm.get() && goalRpm > 0.
    return false;
  }

  @AutoLogOutput(key = "Shooter/WillBeAtSpeed")
  public boolean willBeAtSpeedIn(double horizonSec) {
    // TODO: willBeAtSpeedIn(horizonSec, kToleranceRpm.get()).
    return false;
  }

  public boolean willBeAtSpeedIn(double horizonSec, double toleranceRpm) {
    // TODO: Project velocityRpm + accelRpmPerSec * horizonSec and test it against +/- toleranceRpm.
    // Lets the Superstructure open the feed slightly before the flywheel actually arrives. The
    // Superstructure passes kBurstToleranceRpm once a burst is feeding, so recovery between pieces
    // does not stop and restart the indexer (section 2.4). A zero goal is never "at speed".
    return false;
  }

  @AutoLogOutput(key = "Shooter/ShotCount")
  public int getShotCount() {
    // TODO: Getter. Pieces FIRED, not scored - nothing on the robot can see a score.
    return 0;
  }

  public double getSecondsSinceLastShot() {
    // TODO: sinceLastShot.get(), or +infinity before the first shot of the match.
    return Double.POSITIVE_INFINITY;
  }

  public void resetShotCount() {
    // TODO: Zero it in autonomousInit so per-match counts are clean. (Moved here from Indexer.)
  }

  public double getVelocityRpm() {
    // TODO: inputs.velocityRpm.
    return 0.0;
  }

  public double getGoalRpm() {
    // TODO: goalRpm.
    return 0.0;
  }

  public double getCharacterizationVelocityRotPerSec() {
    // TODO: inputs.velocityRpm / 60.0 - CharacterizationCommands fits kS/kV in Phoenix native units.
    return 0.0;
  }
}
```

---

### 4.9 Shooting on the move

#### `src/main/java/frc/robot/shooting/ShotParameters.java` — `package frc.robot.shooting;` [NEW]

An immutable record so the solver output can be passed around, logged as a struct, and asserted on
in tests without any risk of a consumer mutating it.

```java
package frc.robot.shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public record ShotParameters(
    Rotation2d turretFieldAngle,
    double turretAngularVelocityRadPerSec,
    double flywheelRpm,
    double effectiveDistanceMeters,
    double timeOfFlightSec,
    Translation2d virtualGoal,
    boolean converged,
    boolean inRange) {

  public static ShotParameters invalid() {
    // TODO: Return a record with zeroed values, converged = false, inRange = false. Consumers must
    // check inRange() before using any field; make that the documented contract.
    return null;
  }

  public boolean isUsable() {
    // TODO: converged && inRange && Double.isFinite(flywheelRpm).
    return false;
  }
}
```

#### `src/main/java/frc/robot/shooting/ShootingSolver.java` — `package frc.robot.shooting;` [NEW]

The heart of the tier. Given the robot's field pose, field-relative velocity, field-relative
acceleration, and the goal location, it finds where to aim a shot that will *arrive* at the goal
while the robot keeps moving. It does this by fixed-point iteration on time of flight:

```
t_0  = tofTable(|g - p|)                      // seed with the stationary answer
repeat k = 1 .. kMaxSolverIterations:
    gv_k = g - v*t_{k-1} - 0.5*alpha*a*t_{k-1}^2      // virtual goal ("shoot behind the motion")
    d_k  = |gv_k - p|
    t_k  = tofTable(d_k)
    stop when |t_k - t_{k-1}| < kSolverToleranceSec
```

Then it converts the converged virtual goal into a launch. Relative to the robot the shot is a
stationary shot at the virtual goal: the muzzle velocity is `vm = unit(gv - p) * (d/t)`. The turret
aims along `vm`, at the virtual goal, and the flywheel RPM comes from `|vm|` through a linear fit.
The piece also keeps the robot's `v`, which carries it from `gv` to the real goal over the flight.
(This paragraph used to say `vm = vg - v`, which subtracted `v` a second time; M12 found and fixed
it. See §6.2.) The turret's angular rate feedforward is the
tangential component of the *relative* velocity divided by distance. The class is **pure** — no
subsystem references, no `Timer`, no NetworkTables — precisely so `ShootingSolverTest` can assert
convergence and correctness with no HAL.

```java
package frc.robot.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootingSolver {
  private final InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();
  private int lastIterationCount = 0;
  private double lastResidualSec = 0.0;

  public ShootingSolver() {
    // TODO: Populate the InterpolatingDoubleTreeMap by zipping
    // Constants.Shot.kDistanceBreakpointsMeters with kTimeOfFlightSec. Assert the two arrays are
    // the same length and throw if not - a mismatched table is a silent aiming bug otherwise.
    // (M11 deleted a second, launch-speed table: nothing in solve() ever read it.)
  }

  public ShootingSolver(double[] distances, double[] timesOfFlight) {
    // TODO: Test-only constructor that takes explicit tables so unit tests do not depend on
    // Constants or NetworkTables.
  }

  public void reloadTables() {
    // TODO: Clear and repopulate the map from Constants.Shot. Called from Superstructure's
    // onEnter for scoring states so a mid-match table edit takes effect without a redeploy.
  }

  public ShotParameters solve(
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      Translation2d fieldRelativeAccel,
      Translation2d goal) {
    // TODO: Implement the fixed-point iteration described above. Use
    // Constants.Shot.kAccelCompensationScalar to weight the acceleration term (start at 0). Track
    // lastIterationCount and lastResidualSec. After convergence compute:
    //   Translation2d los   = virtualGoal.minus(robotPose.getTranslation());
    //   double d            = los.getNorm();
    //   double tof          = timeOfFlightTable.get(d);
    //   Translation2d vm    = new Translation2d(d / tof, los.getAngle());  // NOT minus v: M12
    //   Rotation2d aim      = vm.getAngle();
    //   double rpm          = Constants.Shot.kRpmPerMeterPerSec.get() * vm.getNorm()
    //                         + Constants.Shot.kRpmIntercept.get();
    //   double omega        = GeomUtil.tangentialComponent(
    //                            GeomUtil.toTranslation(fieldRelativeSpeeds).unaryMinus(),
    //                            los.getAngle()) / d;
    // Return a ShotParameters, with inRange = d between kMinDistanceMeters and kMaxDistanceMeters.
    return ShotParameters.invalid();
  }

  public ShotParameters solveStationary(Pose2d robotPose, Translation2d goal) {
    // TODO: Convenience wrapper calling solve(...) with zeroed speeds and acceleration. Used by the
    // SCORING_STATIONARY state and as the baseline in unit tests: solve() with zero velocity MUST
    // equal solveStationary() exactly.
    return ShotParameters.invalid();
  }

  public double getTimeOfFlight(double distanceMeters) {
    // TODO: timeOfFlightTable.get(distanceMeters).
    return 0.0;
  }

  public ShotParameters solveStationaryAtRpm(Pose2d robotPose, Translation2d goal, double rpm) {
    // Added by M11: the shot that MEASURES the tables, so it cannot need them. Standing still the
    // aim is straight down the line of sight; the speed is the operator's. Usable on an all-zero
    // table, still range-gated, and invalid for a speed that is not a positive finite number.
    return ShotParameters.invalid();
  }

  public int getLastIterationCount() {
    // TODO: Getter; log it so you can see in replay whether the solver ever hits the iteration cap.
    return 0;
  }

  public double getLastResidualSec() {
    // TODO: Getter; a residual that stays large means your TOF table is too coarse or non-monotonic.
    return 0.0;
  }

  public void logDebug(String prefix, ShotParameters result) {
    // TODO: Logger.recordOutput(prefix + "/VirtualGoal", new Pose2d(result.virtualGoal(),
    // result.turretFieldAngle())) plus iteration count, residual, distance and RPM. Publishing the
    // virtual goal as a Pose2d makes the aim offset visible on AdvantageScope's 2D field, which is
    // by far the fastest way to tell whether the solver is behaving.
  }
}
```

#### `src/main/java/frc/robot/shooting/TurretUnwrapper.java` — `package frc.robot.shooting;` [NEW]

A finite-travel turret can reach a given *direction* at several *positions* (θ, θ±2π, …), and some
of those positions are outside the hard stops. This pure class picks the right one: enumerate the
candidate branches, discard any outside `[min, max]`, prefer the candidate closest to the current
position, and require a hysteresis margin before switching branches so the turret does not oscillate
between two nearly-equidistant options at the boundary. When no candidate is in range it reports
`reachable() == false` along with the closest legal angle, which is what `Turret.outOfRange()` holds
while the Superstructure rotates the drivetrain. Unit-tested because getting this wrong breaks a
belt.

```java
package frc.robot.shooting;

public class TurretUnwrapper {
  /**
   * @param targetPositionRad the continuous position the turret should be commanded to
   * @param reachable whether any branch of the requested direction lies inside the travel limits
   * @param requiresLongWayAround true when the chosen branch is more than pi away from current
   * @param errorRad signed distance from current to target
   */
  public record Result(
      double targetPositionRad, boolean reachable, boolean requiresLongWayAround, double errorRad) {}

  private TurretUnwrapper() {}

  public static Result solve(
      double currentPositionRad,
      double desiredDirectionRad,
      double minPositionRad,
      double maxPositionRad,
      double hysteresisRad) {
    // TODO: Compute the base branch b0 = MathUtil.angleModulus(desiredDirectionRad). Enumerate
    // candidates b0 + 2*pi*k for k in [floor((min - b0)/(2pi)), ceil((max - b0)/(2pi))]. Keep only
    // candidates within [minPositionRad, maxPositionRad]. If none remain, return a Result with
    // reachable = false and targetPositionRad = MathUtil.clamp(b0, min, max). Otherwise pick the
    // candidate minimizing |candidate - currentPositionRad|, but only switch away from the branch
    // nearest the current position if the improvement exceeds hysteresisRad. Set
    // requiresLongWayAround when |target - current| > Math.PI.
    return null;
  }

  public static Result solve(
      double currentPositionRad,
      double desiredDirectionRad,
      double minPositionRad,
      double maxPositionRad) {
    // TODO: Overload with zero hysteresis, used by tests that want deterministic branch selection.
    return null;
  }

  public static double clampIntoRange(double positionRad, double minRad, double maxRad) {
    // TODO: MathUtil.clamp. Separate method so the out-of-range hold behavior is testable on its own.
    return 0.0;
  }

  public static boolean passesThroughArc(
      double fromRad, double toRad, double arcMinRad, double arcMaxRad) {
    // TODO: Return true when the closed interval [min(from,to), max(from,to)] overlaps
    // [arcMinRad, arcMaxRad]. Turret uses this to refuse a sweep through the intake while the
    // intake's pivot is in the turret-conflict band. (M10 replaced its callers with
    // passesThroughArcWrapped, which tests every full-turn image of the arc.)
    return false;
  }
}
```

---

### 4.10 Vision & QuestNav

#### `src/main/java/frc/robot/subsystems/vision/VisionIO.java` [NEW]

One interface for both Limelights. `PoseObservation` carries everything the fusion policy needs to
assign a covariance — tag count, average tag distance, ambiguity, and the capture timestamp — rather
than a bare pose, because a single-tag fix at 6 m and a four-tag fix at 2 m deserve wildly different
trust. `TargetObservation` carries the game-piece detection from the intake camera.

```java
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  public record PoseObservation(
      double timestampSec,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistanceMeters) {}

  public record TargetObservation(double timestampSec, Rotation2d tx, Rotation2d ty, double area) {}

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public double latencySec = 0.0;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public TargetObservation[] targetObservations = new TargetObservation[0];
    public int[] tagIds = new int[0];
    public boolean hasTarget = false;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Pushes the current robot-to-camera transform to the camera so MegaTag2 can solve in robot space.
   * Must be called every loop for the turret camera because the transform moves.
   */
  public default void setRobotToCamera(Transform3d robotToCamera) {}

  /** Pushes the robot's current yaw, required by MegaTag2 before every pose read. */
  public default void setRobotOrientation(Rotation2d yaw, double yawRateRadPerSec) {}

  public default void setPipeline(int index) {}

  public default void setLeds(boolean on) {}
}
```

#### `src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java` [NEW]

Talks to a Limelight 4 over NetworkTables. Two details are load-bearing. First, MegaTag2 requires
`SetRobotOrientation` to be written *before* the botpose entry is read, every loop, or the camera
solves with a stale heading. Second, for the turret camera the `camerapose_robotspace` entry must be
rewritten every loop from the turret angle; the plan also *predicts* one loop ahead to cancel the
write latency, and `Vision` separately discards frames captured while the turret was slewing fast.

> **Dependency note.** `LimelightHelpers.java` is **not** a vendordep — it is a single file you copy
> from `github.com/LimelightVision/limelightlib-wpijava` into `frc/robot/util/`. You may instead read
> the NetworkTables entries directly, which is what this skeleton assumes so the project keeps zero
> extra dependencies. Whichever you choose, do it once and write it down.

```java
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelight implements VisionIO {
  private final String name;
  private final NetworkTable table;
  private final DoubleArrayPublisher robotOrientationPublisher;
  private final DoubleArrayPublisher cameraPosePublisher;
  private final DoublePublisher pipelinePublisher;
  private final DoubleArraySubscriber megaTag2Subscriber;
  private final DoubleArraySubscriber megaTag1Subscriber;
  private final DoubleArraySubscriber targetSubscriber;
  private final IntegerSubscriber tagIdSubscriber;

  public VisionIOLimelight(String name) {
    // TODO: table = NetworkTableInstance.getDefault().getTable(name). Create publishers for
    // "robot_orientation_set" and "camerapose_robotspace_set" and "pipeline". Subscribe to
    // "botpose_orb_wpiblue" (MegaTag2) and "botpose_wpiblue" (MegaTag1, for the QuestNav seeding
    // cross-check) using subscribe(new double[] {}) so you get timestamped values, plus "tv", "tx",
    // "ty", "ta" and "tid". VERIFY every NT key against the Limelight 4 docs for your firmware
    // version - key names have changed between LL releases.
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // TODO: inputs.connected = the botpose subscriber has received a value within a 250 ms window.
    // For each queued botpose_orb_wpiblue sample: the array layout is
    // [x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgTagDist, avgTagArea]; build a
    // PoseObservation with timestampSec = sampleTimestampMicros/1e6 - latencyMs/1000.0. Collect the
    // TargetObservations from tx/ty/ta when tv == 1. VERIFY the array layout against your firmware.
  }

  @Override
  public void setRobotToCamera(Transform3d robotToCamera) {
    // TODO: cameraPosePublisher.set(new double[] {x, y, z, rollDeg, pitchDeg, yawDeg}) - note the
    // Limelight expects DEGREES here while WPILib Rotation3d is radians. This is the single most
    // common turret-camera bug.
  }

  @Override
  public void setRobotOrientation(Rotation2d yaw, double yawRateRadPerSec) {
    // TODO: robotOrientationPublisher.set(new double[] {yaw.getDegrees(),
    // Units.radiansToDegrees(yawRateRadPerSec), 0, 0, 0, 0}) then
    // NetworkTableInstance.getDefault().flush() so the camera sees it before it solves this frame.
  }

  @Override
  public void setPipeline(int index) {
    // TODO: pipelinePublisher.set(index). Used to switch the intake camera between AprilTag and
    // game-piece detection pipelines.
  }

  @Override
  public void setLeds(boolean on) {
    // TODO: Write 3 (force on) or 1 (force off) to the "ledMode" entry.
  }
}
```

#### `src/main/java/frc/robot/subsystems/vision/VisionIOSim.java` [NEW]

A deliberately simple ideal-camera sim: given the ground-truth sim pose and a list of tag poses, it
reports which tags are inside the camera frustum and emits a noisy `PoseObservation`. This is enough
to develop and tune the fusion policy without adding PhotonVision as a dependency, and it lets you
inject exactly the failure you want to test (no tags, one tag at long range, a 30 cm outlier).

```java
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;

public class VisionIOSim implements VisionIO {
  private final String name;
  private final Supplier<Pose2d> groundTruthPoseSupplier;
  private Transform3d robotToCamera = new Transform3d();
  private double noiseStdDevMeters = 0.02;
  private boolean forcedBlackout = false;

  public VisionIOSim(String name, Supplier<Pose2d> groundTruthPoseSupplier) {
    // TODO: Store both. The supplier is the maple-sim ground truth if installed, otherwise
    // RobotState.getInstance()::getEstimatedPose (which makes the sim self-consistent but not
    // independent - good enough for FSM development, not for tuning covariances).
  }

  public void setNoise(double stdDevMeters) {
    // TODO: Test hook.
  }

  public void setBlackout(boolean blackout) {
    // TODO: Test hook that simulates a camera losing all tags, so you can watch the fallback rules
    // work in sim before trusting them at an event.
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // TODO: If forcedBlackout, emit no observations and connected = true. Otherwise load the field
    // tag layout with AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField) -- TODO: confirm
    // the 2026 REBUILT enum constant name in WPILib 2026 -- transform each tag into camera space via
    // robotToCamera, keep tags within a tunable FOV and range, and emit one PoseObservation at
    // Timer.getFPGATimestamp() - a simulated 40 ms latency with Gaussian noise added.
  }

  @Override
  public void setRobotToCamera(Transform3d robotToCamera) {
    // TODO: Store it - the sim must honor the moving turret transform exactly like the real camera.
  }

  @Override
  public void setRobotOrientation(Rotation2d yaw, double yawRateRadPerSec) {
    // TODO: No-op in sim.
  }
}
```

#### `src/main/java/frc/robot/subsystems/vision/QuestNavIO.java` [NEW]

```java
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestNavIO {
  @AutoLog
  public static class QuestNavIOInputs {
    public boolean connected = false;
    public boolean tracking = false;
    public int batteryPercent = 0;
    public int trackingLostCounter = 0;
    public int frameCount = 0;
    /** Headset poses in the Quest's own frame, oldest first, drained each loop. */
    public Pose3d[] questPoses = new Pose3d[0];

    public double[] questPoseTimestamps = new double[0];
  }

  public default void updateInputs(QuestNavIOInputs inputs) {}

  /** Re-seeds the headset's origin so its reported pose matches a known field pose. */
  public default void setPose(Pose3d fieldToQuest) {}

  /** Must be called once per loop for the questnavlib command/response machinery to run. */
  public default void commandPeriodic() {}
}
```

#### `src/main/java/frc/robot/subsystems/vision/QuestNavIOReal.java` [NEW]

Thin wrapper over `gg.questnav.questnav.QuestNav` (questnavlib 2026-3.0.0-beta). Verified API
surface: `commandPeriodic()`, `getAllUnreadPoseFrames()` returning `PoseFrame[]` whose components are
`questPose3d()`, `dataTimestamp()`, `appTimestamp()`, `frameCount()`, `isTracking()`; plus
`isConnected()`, `isTracking()`, `getBatteryPercent()` and `getTrackingLostCounter()` returning
`Optional<Integer>`, and `setPose(Pose3d)`.

```java
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.QuestNav;

public class QuestNavIOReal implements QuestNavIO {
  private final QuestNav questNav = new QuestNav();

  public QuestNavIOReal() {
    // TODO: Register the event callbacks so tracking transitions land in the log with no polling:
    // questNav.onTrackingLost(() -> Logger.recordOutput("QuestNav/Event", "TrackingLost")),
    // onTrackingAcquired, onConnected, onDisconnected, and
    // onLowBattery(Constants.Vision.kQuestMinBatteryPercent, pct -> ...). Callbacks only fire from
    // commandPeriodic(), so that call is not optional.
  }

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    // TODO: inputs.connected = questNav.isConnected(); inputs.tracking = questNav.isTracking();
    // batteryPercent/trackingLostCounter from the Optional<Integer> getters with orElse(-1). Drain
    // questNav.getAllUnreadPoseFrames() into the parallel questPoses / questPoseTimestamps arrays
    // using frame.questPose3d() and frame.dataTimestamp(). Drain every frame each loop; leaving
    // frames unread is what causes the "QuestNav pose is 200 ms behind" complaint.
  }

  @Override
  public void setPose(Pose3d fieldToQuest) {
    // TODO: questNav.setPose(fieldToQuest). Note this sets the HEADSET pose, not the robot pose, so
    // the caller must pre-compose Constants.Vision.kRobotToQuest.
  }

  @Override
  public void commandPeriodic() {
    // TODO: questNav.commandPeriodic(). Call first thing in Vision.periodic().
  }
}
```

#### `src/main/java/frc/robot/subsystems/vision/QuestNavIOSim.java` [NEW]

```java
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Supplier;

public class QuestNavIOSim implements QuestNavIO {
  private final Supplier<Pose2d> groundTruthPoseSupplier;
  private Pose3d seedOffset = new Pose3d();
  private boolean trackingForced = true;
  private double driftMetersPerSec = 0.0;

  public QuestNavIOSim(Supplier<Pose2d> groundTruthPoseSupplier) {
    // TODO: Store the supplier.
  }

  public void setTracking(boolean tracking) {
    // TODO: Test hook to drop tracking on demand and watch the fallback engage.
  }

  public void setDrift(double metersPerSec) {
    // TODO: Test hook that injects slow drift so the re-seed logic is exercised in sim.
  }

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    // TODO: Emit one pose per loop at Timer.getFPGATimestamp() - 0.01, equal to the ground truth
    // composed with kRobotToQuest and seedOffset, plus accumulated drift. connected = true,
    // tracking = trackingForced, batteryPercent = 100.
  }

  @Override
  public void setPose(Pose3d fieldToQuest) {
    // TODO: Recompute seedOffset so the next reported pose equals fieldToQuest exactly.
  }

  @Override
  public void commandPeriodic() {
    // TODO: No-op.
  }
}
```

#### `src/main/java/frc/robot/subsystems/vision/Vision.java` [NEW]

The fusion policy lives here, and only here. Each loop it: pushes fresh robot orientation to both
Limelights; recomputes the turret camera's `robotToCamera` from the turret angle and pushes it;
drains both cameras' `PoseObservation`s, computes a per-observation covariance from tag count and
distance, applies rejection rules (ambiguity, Z error, pose jump, turret slew rate, timestamp
staleness), and hands survivors to `RobotState`. Then it evaluates QuestNav health, decides whether
QuestNav is trusted, seeds or re-seeds it against the MegaTag2 estimate when required, and feeds its
poses in with much tighter covariance. Everything it decides is logged so a bad pose in a match can
be diagnosed from the log alone.

```java
package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Vision extends SubsystemBase {
  private final VisionIO turretCameraIO;
  private final VisionIO intakeCameraIO;
  private final QuestNavIO questNavIO;
  private final VisionIOInputsAutoLogged turretInputs = new VisionIOInputsAutoLogged();
  private final VisionIOInputsAutoLogged intakeInputs = new VisionIOInputsAutoLogged();
  private final QuestNavIOInputsAutoLogged questInputs = new QuestNavIOInputsAutoLogged();

  private final DoubleSupplier turretAngleSupplier;
  private final DoubleSupplier turretVelocitySupplier;

  private final Timer questResidualTimer = new Timer();
  private int acceptedCount = 0;
  private int rejectedCount = 0;
  private boolean questTrusted = false;

  public Vision(
      VisionIO turretCameraIO,
      VisionIO intakeCameraIO,
      QuestNavIO questNavIO,
      DoubleSupplier turretAngleSupplier,
      DoubleSupplier turretVelocitySupplier) {
    // TODO: Store everything. Register FaultReporter checks for both cameras and the headset:
    // "Turret Limelight disconnected", "Intake Limelight disconnected", "QuestNav not tracking"
    // (warning, not error - the robot still works without it) and "QuestNav battery low".
  }

  @Override
  public void periodic() {
    // TODO: Sequence, in this exact order:
    //   1. questNavIO.commandPeriodic()
    //   2. pushRobotOrientation()
    //   3. turretCameraIO.setRobotToCamera(robotToTurretCameraNow())
    //   4. updateInputs + processInputs for all three IOs
    //   5. processCamera(turretInputs, Source.LIMELIGHT_TURRET, true)
    //   6. processCamera(intakeInputs, Source.LIMELIGHT_INTAKE, false)
    //   7. updateQuestNavTrust(); processQuestNav()
    //   8. log accepted/rejected counts and every rejection reason
  }

  private void pushRobotOrientation() {
    // TODO: Rotation2d yaw = RobotState.getInstance().getRotation(); double rate =
    // RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond; push to both cameras.
    // MegaTag2 is only as good as this heading, so it must come from the fused estimate, not raw gyro.
  }

  public Transform3d robotToTurretCameraNow() {
    // TODO: GeomUtil.rotateAboutZ(Constants.Vision.kRobotToTurretAxis,
    // turretAngleSupplier.getAsDouble() + turretVelocitySupplier.getAsDouble() *
    // Constants.getLoopPeriodSec()) then .plus(Constants.Vision.kTurretToCamera). The one-loop
    // velocity extrapolation cancels the NT write latency.
  }

  public Transform3d robotToTurretCameraAt(double timestampSec) {
    // TODO: Same composition but using RobotState.getInstance().getTurretAngleAt(timestampSec).
    // Returns the identity transform and logs a warning if the buffer has no sample - callers must
    // then reject the observation rather than use a wrong transform.
    return new Transform3d();
  }

  private void processCamera(
      VisionIOInputs inputs, RobotState.PoseSource source, boolean isTurretMounted) {
    // TODO: For each PoseObservation: reject if tagCount == 0; if tagCount == 1 and ambiguity >
    // kMaxAmbiguity; if |pose.getZ()| > kMaxZErrorMeters; if the pose is outside the field bounds;
    // if isTurretMounted and |turretVelocity| > kMaxTurretRateForVisionRadPerSec; if
    // RobotState.getPoseAt(timestamp) is empty. Otherwise compute std devs with
    // computeStdDevs(...) and call RobotState.addVisionObservation(...). Count and log every
    // rejection with its reason under Vision/<source>/Rejections/<reason>.
  }

  private Matrix<N3, N1> computeStdDevs(
      int tagCount, double averageDistanceMeters, RobotState.PoseSource source) {
    // TODO: xy = kMegaTag2XyStdDevBase * averageDistance^2 / tagCount; theta =
    // kMegaTag2ThetaStdDev (left huge on purpose: MegaTag2 consumes your heading, so trusting its
    // heading output creates a feedback loop). Return VecBuilder.fill(xy, xy, theta).
    return null;
  }

  private void updateQuestNavTrust() {
    // TODO: questTrusted = questInputs.connected && questInputs.tracking
    //   && questInputs.batteryPercent > kQuestMinBatteryPercent
    //   && trackingLostCounter has not increased this loop
    //   && questSeeded
    //   && the residual between the Quest-derived robot pose and RobotState's estimate has stayed
    //      below kQuestReseedResidualMeters. Push the result into RobotState.setQuestTrusted and log
    //   the specific reason whenever it flips to false.
  }

  private void processQuestNav() {
    // TODO: For each drained frame, convert the headset Pose3d to a robot Pose2d by applying
    // Constants.Vision.kRobotToQuest.inverse(), and if questTrusted call
    // RobotState.addVisionObservation with VecBuilder.fill(kQuestXyStdDev, kQuestXyStdDev,
    // kQuestThetaStdDev). If !questSeeded, or the residual has exceeded kQuestReseedResidualMeters
    // continuously for kQuestReseedHoldSec while the robot is nearly stationary AND at least two
    // tags are visible, call seedQuestNav().
  }

  public void seedQuestNav() {
    // TODO: Pose2d p = RobotState.getInstance().getEstimatedPose(); questNavIO.setPose(
    // new Pose3d(p).transformBy(Constants.Vision.kRobotToQuest)); RobotState.setQuestSeeded(true);
    // and log the seed event with the pose so you can count re-seeds per match.
  }

  public java.util.Optional<VisionIO.TargetObservation> getBestGamePieceTarget() {
    // TODO: Return the largest-area TargetObservation from intakeInputs when the intake camera is on
    // its game-piece pipeline. Consumed by DriveCommands.gamePieceAssist.
    return java.util.Optional.empty();
  }

  public java.util.Optional<Pose2d> getGamePieceFieldPose() {
    // TODO: Convert the best target's tx/ty into a field position using the planar-ground
    // assumption: range = (kRobotToIntakeCamera.getZ() - kGamePieceDiameterMeters/2)
    // / Math.tan(cameraPitch + ty), bearing = -tx, then transform by the robot pose at the
    // observation timestamp. TODO: kGamePieceDiameterMeters must be MEASURED first.
    return java.util.Optional.empty();
  }

  public void setIntakePipeline(int index) {
    // TODO: intakeCameraIO.setPipeline(index) - Superstructure switches it to the game-piece
    // pipeline while IntakeMode is INTAKING and back to AprilTags otherwise. Keyed on the intake
    // REQUEST, not the arm position: the arm now rests deployed for most of the match (section
    // 2.4), and keying on position would take the camera off AprilTags permanently.
  }

  @AutoLogOutput(key = "Vision/QuestTrusted")
  public boolean isQuestTrusted() {
    // TODO: Getter.
    return false;
  }

  @AutoLogOutput(key = "Vision/AcceptedObservations")
  public int getAcceptedCount() {
    // TODO: Getter.
    return 0;
  }
}
```

---

### 4.11 Superstructure

#### `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java` [NEW]

The cross-mechanism state machine, and the only class allowed to command more than one subsystem.
It owns the `ShootingSolver`, converts its output into turret and flywheel goals, decides when the
indexer may feed, enforces the intake/turret interlock in both directions, and handles the
"turret cannot reach" case by asking the drivetrain for a heading. It exposes exactly one mutator to
the rest of the robot through two mutators, `setWantedState` for the shooter-side intent and
`setIntakeRequested` for the intake intent, so a binding cannot put the robot into an illegal
combination. The two intents are **orthogonal regions** (§2.4). The driver can shoot, move and
intake at once, and `resolveIntakeMode()` reconciles them each loop. Its `readyToScore` gate is
the seven-term combination described in §6.5. It asks nothing about game pieces, because with a
hopper and no piece sensor there is nothing trustworthy to ask.

```java
package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shooting.ShootingSolver;
import frc.robot.shooting.ShotParameters;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.floorroller.FloorRoller;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {
  public enum WantedState {
    IDLE,
    DEFAULT, // RESTING, intake keeps its posture (a deployed intake stays deployed)
    STOW, // RESTING, and set the intake posture to STOWED - the only way the intake retracts
    HOME,
    EJECT,
    SPIN_UP,
    SCORE_ON_MOVE,
    SCORE_STATIONARY,
    SCORE_FORCED,
    PASS,
    AUTO_SCORE_ON_MOVE,
    CHARACTERIZE
  }

  public enum SystemState {
    IDLE,
    HOMING,
    RESTING, // was STOWED; renamed because the intake is usually deployed while resting
    EJECTING,
    SPINNING_UP,
    SCORING_ON_THE_MOVE,
    SCORING_STATIONARY,
    SCORING_FORCED,
    PASSING,
    WAITING_FOR_TURRET,
    CHARACTERIZING,
    FAULTED
  }

  /** The intake region, resolved every loop from the SystemState and the intake request. */
  public enum IntakeMode {
    STOWED,
    DEPLOYED,
    INTAKING,
    FEED_ASSIST
  }

  private enum IntakePosture {
    STOWED,
    DEPLOYED
  }

  private final Drive drive;
  private final Intake intake;
  private final FloorRoller floorRoller;
  private final Indexer indexer;
  private final Turret turret;
  private final Shooter shooter;
  private final Vision vision;
  private final ShootingSolver solver = new ShootingSolver();

  private final Timer stateTimer = new Timer();
  private WantedState wantedState = WantedState.HOME;
  private SystemState systemState = SystemState.HOMING;
  private ShotParameters lastShot = ShotParameters.invalid();
  private boolean ignoreShotTolerances = false;
  private Optional<Rotation2d> requestedDriveHeading = Optional.empty();

  private boolean intakeRequested = false;
  private IntakePosture intakePosture = IntakePosture.STOWED; // the robot starts inside its frame
  private IntakeMode intakeMode = IntakeMode.STOWED;
  private boolean burstLatched = false; // feed has opened in this scoring state (section 2.4)
  private double unproductiveFeedSec = 0.0; // gate-open, unloaded feeding since the last shot

  public Superstructure(
      Drive drive,
      Intake intake,
      FloorRoller floorRoller,
      Indexer indexer,
      Turret turret,
      Shooter shooter,
      Vision vision) {
    // TODO: Store everything, start stateTimer, and wire the two interlocks:
    // intake.setDeployGuard(() -> !turretInIntakeConflictArc());
    // turret.setIntakeArcGuard(intake::isInTurretConflictBand);
    // This is the one place the interlock exists; do not duplicate it. Note the turret half keys
    // on the intake's PIVOT BAND, not on "stowed": a deployed intake cannot reach the turret.
  }

  @Override
  public void periodic() {
    // TODO: Standard five-step periodic (this subsystem has no IO layer). Before
    // handleStateTransition(), recompute lastShot = solver.solve(...) exactly once per loop and
    // solver.logDebug("Superstructure/Shot", lastShot) so every state reads the same solution.
    // After the transition and before applyState(): intakeMode = resolveIntakeMode(), then
    // updateEmptyBelief(). applyState() commands the shooter side (turret, shooter) from the
    // SystemState handler and the feed side (intake, floor roller, indexer) from applyFeedSide(),
    // so no handler has to re-derive the intake logic. Log IntakeMode, IntakePosture, BurstLatched
    // and UnproductiveFeedSec.
  }

  private SystemState handleStateTransition() {
    // TODO: Guarded switch on wantedState. Key guards: any SCORE_* requires
    // !FaultReporter.getInstance().anyCritical(), else fall through to SPINNING_UP - there is NO
    // piece guard (section 2.4); SCORE_ON_MOVE additionally requires lastShot.isUsable(), else
    // WAITING_FOR_TURRET; if turret.isOutOfRange() -> WAITING_FOR_TURRET; HOME -> HOMING until
    // intake.isHomed() && turret.isHomed(), with a state timeout to FAULTED; DEFAULT and STOW ->
    // RESTING (STOW also sets intakePosture = STOWED).
    return SystemState.IDLE;
  }

  private void onEnter(SystemState state) {
    // TODO: Any SCORING_* -> solver.reloadTables(), burstLatched = false. shooter.resetShotCount()
    // is NOT called here (that belongs to autonomousInit). The intake-camera pipeline switch moves
    // to resolveIntakeMode(): game-piece pipeline on entering IntakeMode.INTAKING, AprilTags on
    // leaving it.
  }

  private void onExit(SystemState state) {
    // TODO: Any SCORING_* -> indexer.setWantedState(IDLE), burstLatched = false, and clear
    // requestedDriveHeading.
  }

  private void applyState() {
    // TODO: switch dispatching to the handlers below.
  }

  private void resting() {
    // TODO: turret STOW, shooter IDLE (idle RPM, not zero). The feed side comes from
    // applyFeedSide(): the intake keeps its posture - it does NOT stow just because nothing is
    // happening, since raising it would dump the hopper.
  }

  private void homing() {
    // TODO: intake HOME, turret HOME, everything else IDLE.
  }

  private IntakeMode resolveIntakeMode() {
    // TODO: The table in section 2.4. HOMING/EJECTING/CHARACTERIZING/FAULTED are overridden by
    // their own handlers and return the mode unchanged. Otherwise: intakeRequested -> INTAKING
    // (and intakePosture = DEPLOYED); else scoring state with the gate open and posture DEPLOYED
    // -> FEED_ASSIST, unless a mis-tuned kFeedAssistAngleRad is in the turret-conflict band and the
    // turret needs the arc (then DEPLOYED, log Superstructure/FeedAssistBlocked and raise the
    // misconfiguration alert - never expected on this robot); else the posture.
    // Switch the intake-camera pipeline on entering/leaving INTAKING. Also log
    // Superstructure/StowedWhileMoving (and drive its warning alert) when the chassis is moving and
    // !intake.isRetainingPieces() - pieces can fall out of a stowed intake.
    return IntakeMode.STOWED;
  }

  private void applyFeedSide() {
    // TODO: Skipped in the override states. Otherwise, from intakeMode, shotRequested =
    // wantedState is SCORE_* / AUTO_SCORE_ON_MOVE / PASS (the WANTED state, so a request diverted
    // to SPINNING_UP or WAITING_FOR_TURRET still counts), and feedOpen = scoring system state &&
    // isReadyToScore():
    //   intake:      INTAKING -> DEPLOY_AND_INTAKE; FEED_ASSIST -> FEED_ASSIST;
    //                DEPLOYED -> DEPLOY_ONLY; STOWED -> STOW
    //   floorRoller: indexer.isUnjamming() ? EJECT : shotRequested ? FEED : IDLE
    //   indexer:     feedOpen ? FEED : IDLE
    // The floor roller LEADS: it packs the pile against the stopped indexer as soon as a shot is
    // requested, and the braked indexer is the block until the shooter is ready. It never runs for
    // intaking alone. Nothing here stows the turret: a deployed intake is outside the conflict band.
  }

  private void updateEmptyBelief() {
    // TODO: If a scoring state is active, the gate is open, the indexer is FEEDING (not
    // UNJAMMING), !indexer.isLoaded(), and shooter.getSecondsSinceLastShot() > one loop, add one
    // loop period to unproductiveFeedSec. Reset it to 0 when a shot is detected and whenever
    // IntakeMode is INTAKING (new pieces may be arriving). Time with the gate CLOSED must not
    // accumulate - it says nothing about the hopper.

  private void ejecting() {
    // TODO: intake EJECT, floorRoller EJECT, indexer EJECT, shooter COAST. Held-button, not
    // timed: emptying a full hopper takes as long as it takes.
  }

  private void spinningUp() {
    // TODO: shooter.setGoal(lastShot.isUsable() ? lastShot.flywheelRpm() :
    // Constants.Shooter.kIdleRpm.get()); turret aims if the solution is usable, else stows. The
    // feed side (including intaking, if held) comes from applyFeedSide().
  }

  private void scoringOnTheMove() {
    // TODO: turret.setGoal(lastShot.turretFieldAngle(), lastShot.turretAngularVelocityRadPerSec());
    // shooter.setGoal(lastShot.flywheelRpm());
    // The indexer, floor roller and intake are commanded by applyFeedSide(): FEED streams the
    // hopper while the gate is open, the intake feed-assists (or keeps intaking, if held), and the
    // pivot-band interlock - not a blanket stow - keeps the arm out of the turret's way.
  }

  private void scoringStationary() {
    // TODO: Same as scoringOnTheMove but using solver.solveStationary(...) and additionally
    // requiring |chassis speed| < a tunable threshold in isReadyToScore().
    // M11 adds a shot-table override: while setShotTableRpm(rpm) is set and SCORE_STATIONARY is
    // the request, the solve is solver.solveStationaryAtRpm(pose, goal, rpm) instead, which needs
    // no table. Everything else is unchanged -- same gate, tight-band first piece, burst latch,
    // still chassis. ANY request other than SCORE_STATIONARY clears it (the measurement command's
    // closing DEFAULT and onDisable's IDLE included), so it cannot leak into a real shot.
  }

  private void scoringForced() {
    // TODO: Operator override: aim with lastShot but feed unconditionally. Log a loud warning so a
    // log reviewer can tell forced shots from gated ones when scoring the match.
  }

  private void passing() {
    // TODO: Aim at Constants.Field's pass target (a separate tunable translation, TODO: choose a
    // sensible pass destination once the field is known) with a lower, separately-tuned RPM.
  }

  private void waitingForTurret() {
    // TODO: The turret cannot reach the aim point. Set requestedDriveHeading to the field angle that
    // would bring the aim point back inside the turret's travel, hold the flywheel at the solved RPM,
    // and keep the indexer IDLE (applyFeedSide sees the gate closed). DriveCommands reads getRequestedDriveHeading() and blends it into
    // the driver's rotation input.
  }

  private void characterizing() {
    // TODO: Pass through to whichever subsystem CharacterizationCommands has selected; command the
    // others to IDLE so nothing else moves during a SysId sweep.
  }

  private void faulted() {
    // TODO: Command every subsystem to IDLE. Only HOME clears FAULTED.
  }

  private boolean turretInIntakeConflictArc() {
    // TODO: TurretUnwrapper.passesThroughArc(turret.getPositionRad(), turret's commanded position,
    // Constants.Turret.kIntakeConflictMinRad.get(), kIntakeConflictMaxRad.get()).
    return false;
  }

  @AutoLogOutput(key = "Superstructure/ReadyToScore")
  public boolean isReadyToScore() {
    // TODO: The full gate (see section 6.5): lastShot.isUsable() && turret.atGoal() &&
    // shooter.willBeAtSpeedIn(Constants.getLoopPeriodSec() * 2,
    //     burstLatched ? kBurstToleranceRpm : kToleranceRpm) &&
    // RobotState.getVisionStalenessSec() < kMaxVisionStalenessSec &&
    // RobotState.getPoseStdDevEstimate() < kMaxPoseStdDevMeters &&
    // RobotState.getFieldRelativeAccel().getNorm() < kMaxChassisAccelForShotMetersPerSec2 &&
    // !FaultReporter.anyCritical(). Return true unconditionally when ignoreShotTolerances.
    // Seven terms; there is no piece term (section 2.4). Set burstLatched when the gate first
    // opens in a scoring state; clear it when any term OTHER than term 3 is false. Compute the
    // gate once per loop and cache it, so applyFeedSide and the log see the same answer.
    // Log EACH term separately under Superstructure/ReadyTerms/* - when the robot will not shoot,
    // this log is how you find out which term is false in under ten seconds.
    return false;
  }

  @AutoLogOutput(key = "Superstructure/LikelyEmpty")
  public boolean likelyEmpty() {
    // TODO: unproductiveFeedSec >= Constants.Superstructure.kEmptyConfirmSec.get(). A BELIEF, not a
    // measurement: autos use it to end a scoring leg and the driver controller rumbles on it.
    // Never gate a teleop shot on it.
    return false;
  }

  public void setWantedState(WantedState wanted) {
    // TODO: Assign.
  }

  public Command setWantedStateCommand(WantedState wanted) {
    // TODO: runOnce(() -> setWantedState(wanted)) WITHOUT requiring this subsystem, so a held button
    // does not interrupt the default drive command.
    return null;
  }

  public Command scoreUntilEmptyCommand() {
    // TODO: Command used by autos: setWantedStateCommand(AUTO_SCORE_ON_MOVE) then
    // Commands.waitUntil(this::likelyEmpty).withTimeout(Constants.Auto.kScoreTimeoutSec) then back
    // to DEFAULT. The timeout is now a max-burst bound (long enough to empty a full hopper), and
    // likelyEmpty() is what ends the leg early.
    return null;
  }

  public void onEnable() {
    // TODO: Called from Robot.autonomousInit and teleopInit. Set intakePosture = DEPLOYED,
    // unconditionally -- an enable during the boot home must not be lost. The robot must START
    // stowed (a rule) but pieces fall out of a stowed intake, so the first enable deploys it with
    // no button press. An intake an operator deliberately STOWed during auto is redeployed at
    // teleopInit, for the same reason.
  }

  public void setIntakeRequested(boolean requested) {
    // TODO: Assign. The second mutator (section 2.4). Independent of wantedState, so the driver can
    // hold INTAKE and SCORE at the same time.
  }

  public Command intakeWhileHeldCommand() {
    // TODO: Commands.startEnd(() -> setIntakeRequested(true), () -> setIntakeRequested(false))
    // WITHOUT requiring this subsystem, for a whileTrue binding. Release stops the roller; the
    // intake stays deployed.
    return null;
  }

  @AutoLogOutput(key = "Superstructure/IntakeMode")
  public IntakeMode getIntakeMode() {
    // TODO: Getter.
    return intakeMode;
  }

  public void setIgnoreShotTolerances(boolean ignore) {
    // TODO: Operator "force shoot" toggle.
  }

  public Optional<Rotation2d> getRequestedDriveHeading() {
    // TODO: Getter read by DriveCommands.
    return Optional.empty();
  }

  public ShotParameters getLastShot() {
    // TODO: Getter, used by AutoRoutines to decide whether to keep driving or hold.
    return lastShot;
  }

  @AutoLogOutput(key = "Superstructure/SystemState")
  public SystemState getSystemState() {
    // TODO: Getter. The @AutoLogOutput on an enum publishes it as a string AdvantageScope can graph
    // as a state timeline, which is the single most useful plot in a post-match review.
    return systemState;
  }
}
```

<!--SECTION4D-->




---

### 4.12 Autos

> **Corrected by M13** (`feature-autos`). The skeletons below are kept as written, as the record of
> the plan; the implementation differs in these ways, each for a reason found in the code:
>
> - **There is no `AutoFactory.PP_CONFIG`.** The one config is `Drive.PP_CONFIG`, built from
>   `TunerConstants`, because the setpoint generator needs it in `Drive`'s constructor. `settings.json`
>   is not loaded: it is compared with `Drive.PP_CONFIG` and a warning names each difference
>   (`loadRobotConfig` became `checkGuiSettings`). The shipped file could not even be parsed.
> - **The flip callback is `AutoFactory.shouldFlipPaths()`**, which sets PathPlanner's
>   `FlippingUtil` to rotational symmetry over `Constants.Field` first (§8.1, risk #4).
> - **`AutoFactory` does not take the turret**, and `AutoRoutines` has no `aimTurretAt`: the
>   superstructure overwrites any direct turret request on the next loop. Spinning up pre-aims.
> - **Every shooter-side request is held for its leg** (`AutoFactory.holdWantedState`) and handed
>   back as `DEFAULT` when the leg ends, however it ends. `spinUpWhile` sets `SPIN_UP` before the
>   inner leg rather than beside it. `shootInPlace` holds the drive stopped for the whole shot and
>   takes its bound as a supplier (`Constants.Auto.kPreloadTimeoutSec` for the preload).
> - **`getStartingPoseFromAutoFile` does not exist** in 2026.1.2. Paths come through a
>   `PathSource`; `buildAuto` reads the file's path group first, because the `PathPlannerAuto`
>   constructor swallows a missing file.
> - **`AutoSelector` has one chooser, of names**, not two parallel choosers. The preview takes a
>   routine's start from its first path, so no companion `.auto` files are needed.
> - Added: `centerThreePieceStationary()` (§8.6 step 3), `getPathEndingPose`, `pathfindTo`,
>   `getPreloadFired`, and Scavenge sweeps each cycle rather than driving back to shoot.

#### `src/main/java/frc/robot/autos/AutoFactory.java` — `package frc.robot.autos;` [NEW]

Owns everything PathPlanner needs to exist, in one place, so `Drive` stays a drivetrain and not a
path-follower. It holds the single `RobotConfig` instance (`PP_CONFIG`) that both `AutoBuilder` and
`Drive`'s `SwerveSetpointGenerator` share — if those two disagree about robot mass or wheel COF the
setpoint generator will silently clip the speeds PathPlanner asks for and every path will undershoot.
It calls `AutoBuilder.configure(...)` wired to `RobotState` for the pose supplier (not `Drive`, which
no longer owns a pose estimator), registers the `NamedCommands` vocabulary that `.auto` files are
allowed to use, installs `LocalADStarAK` as the pathfinder so pathfinding is replayable, and hooks
`PathPlannerLogging` so the active path and target pose land in the log for AdvantageScope's 2D
field. The concrete capability: a student can add an auto in the PathPlanner GUI, drop in a marker
named `ScoreOnMove`, and it works with no Java change.

```java
package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;

// Holds the Shooter, not the Indexer (M10b step 9): the indexer's only auto role was answering
// hasPiece, and the flywheel's shot count is the only piece count the robot has.
public class AutoFactory {
  /**
   * The one RobotConfig for the whole robot. Drive's SwerveSetpointGenerator and AutoBuilder must
   * both use THIS instance.
   */
  public static RobotConfig PP_CONFIG = null;

  private final Drive drive;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Shooter shooter;
  private final Turret turret;
  private final Vision vision;

  public AutoFactory(
      Drive drive,
      Superstructure superstructure,
      Intake intake,
      Shooter shooter,
      Turret turret,
      Vision vision) {
    // TODO: Store all references, then call loadRobotConfig(), configureAutoBuilder() and
    // registerNamedCommands() in that order. Construct this exactly once, from RobotContainer,
    // after RobotState.initialize().
  }

  private void loadRobotConfig() {
    // TODO: Try PP_CONFIG = RobotConfig.fromGUISettings() (reads src/main/deploy/pathplanner/
    // settings.json; throws IOException/ParseException). On failure, fall back to a hand-built
    // RobotConfig(Constants.Drivetrain.kRobotMassKg, Constants.Drivetrain.kRobotMoiKgM2,
    // new ModuleConfig(TunerConstants.kWheelRadius, maxDriveVelocity, kWheelCoefficientOfFriction,
    // DCMotor.getKrakenX60(1), driveGearRatio, statorCurrentLimit, 1),
    // Drive.getModuleTranslations()) and raise a FaultReporter error alert so a stale settings.json
    // is loud. Verify the RobotConfig/ModuleConfig constructor argument order against
    // PathplannerLib 2026.1.2 before relying on it.
  }

  private void configureAutoBuilder() {
    // TODO: AutoBuilder.configure(
    //   () -> RobotState.getInstance().getEstimatedPose(),
    //   pose -> RobotState.getInstance().resetPose(pose, drive.getRawGyroRotation(),
    //                                              drive.getModulePositions()),
    //   () -> RobotState.getInstance().getRobotRelativeSpeeds(),
    //   (speeds, feedforwards) -> drive.runVelocity(speeds, feedforwards),
    //   new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0),
    //                                  new PIDConstants(5.0, 0.0, 0.0)),
    //   PP_CONFIG, AllianceFlipUtil::shouldFlip, drive);
    // Then Pathfinding.setPathfinder(new LocalADStarAK()) and the two PathPlannerLogging callbacks
    // from the stock template (setLogActivePathCallback -> "Odometry/Trajectory",
    // setLogTargetPoseCallback -> "Odometry/TrajectorySetpoint").
  }

  private void registerNamedCommands() {
    // TODO: NamedCommands.registerCommand(name, command) for the full vocabulary below. Keep the
    // names EXACTLY as written here - they are typed by hand into the PathPlanner GUI and a typo
    // produces a silently-skipped marker, not an error:
    //   "Intake"          -> Commands.runOnce(() -> superstructure.setIntakeRequested(true))
    //   "StopIntake"      -> Commands.runOnce(() -> superstructure.setIntakeRequested(false))
    //                        (roller stops; the intake stays deployed)
    //   "StowIntake"      -> superstructure.setWantedStateCommand(WantedState.STOW)
    //   "SpinUp"          -> superstructure.setWantedStateCommand(WantedState.SPIN_UP)
    //   "ScoreOnMove"     -> superstructure.scoreUntilEmptyCommand()
    //   "ScoreStationary" -> scoreStationaryCommand()
    //   "Eject"           -> superstructure.setWantedStateCommand(WantedState.EJECT)
    //   "Home"            -> superstructure.setWantedStateCommand(WantedState.HOME)
    //   "StowTurret"      -> turret.setWantedStateCommand(Turret.WantedState.STOW)
    //   "GamePiecePipeline"-> Commands.runOnce(() -> vision.setIntakePipeline(1))
    //   "TagPipeline"     -> Commands.runOnce(() -> vision.setIntakePipeline(0))
  }

  public Command buildAuto(String autoName) {
    // TODO: return new PathPlannerAuto(autoName) (or AutoBuilder.buildAuto(autoName)); wrap in a
    // try/catch that returns Commands.none() and fires a FaultReporter alert if the .auto file is
    // missing, so a renamed file cannot crash robotInit at an event.
    return null;
  }

  public Command followPath(String pathName) {
    // TODO: PathPlannerPath.fromPathFile(pathName) inside try/catch (IOException,
    // org.json.simple.parser.ParseException), then AutoBuilder.followPath(path). Return
    // Commands.none() plus an alert on failure.
    return null;
  }

  public Command scoreStationaryCommand() {
    // TODO: superstructure.setWantedStateCommand(SCORE_STATIONARY)
    //   .andThen(Commands.waitUntil(superstructure::likelyEmpty))
    //   .withTimeout(Constants.Auto.kScoreTimeoutSec.get())
    //   .andThen(superstructure.setWantedStateCommand(DEFAULT)).
    return null;
  }

  public Command logPreloadFiredCommand() {
    // Implemented in M10b step 9: Commands.runOnce(() -> Logger.recordOutput("Auto/PreloadFired",
    // shooter.getShotCount())), read when it runs. The count is zeroed at autonomousInit and the
    // rules load exactly 8 pieces, so logged after the preload burst it checks the shot detector
    // in every match. It gates nothing.
    return null;
  }

  public Command resetPoseToPathStart(String pathName) {
    // TODO: Commands.runOnce(() -> { PathPlannerPath p = PathPlannerPath.fromPathFile(pathName);
    // Pose2d start = AllianceFlipUtil.shouldFlip() ? p.flipPath().getStartingHolonomicPose()
    // .orElseThrow() : p.getStartingHolonomicPose().orElseThrow();
    // RobotState.getInstance().resetPose(start, drive.getRawGyroRotation(),
    // drive.getModulePositions()); }). Verify getStartingHolonomicPose()'s Optional return type
    // against PathplannerLib 2026.1.2.
    return null;
  }

  public Optional<Pose2d> getStartingPose(String autoName) {
    // TODO: PathPlannerAuto.getStartingPoseFromAutoFile(autoName), alliance-flipped, wrapped in
    // Optional and try/catch. AutoSelector logs this so the drive team can verify robot placement
    // before the match. Verify the static helper's exact name in PathplannerLib 2026.1.2.
    return Optional.empty();
  }

  public static PathConstraints defaultPathfindConstraints() {
    // TODO: new PathConstraints(Constants.Auto.kPathfindMaxVel.get(),
    // Constants.Auto.kPathfindMaxAccel.get(), Constants.Auto.kPathfindMaxAngVel.get(),
    // Constants.Auto.kPathfindMaxAngAccel.get()). Shared by DriveToPoseCommand's pathfind handoff.
    return null;
  }

  public Drive getDrive() {
    // TODO: Getter used by AutoRoutines so it does not need its own Drive reference.
    return drive;
  }

  public Superstructure getSuperstructure() {
    // TODO: Getter used by AutoRoutines.
    return superstructure;
  }

  public Intake getIntake() {
    // Getter used by AutoRoutines.waitUntilIntakeRetaining().
    return intake;
  }
}
```

> **Note.** `AutoFactory` and the two files below reference a `Constants.Auto` nested class that
> does not exist in the tree written in §4.1. Add it; §5.3 gives the exact block to paste.

#### `src/main/java/frc/robot/autos/AutoRoutines.java` — `package frc.robot.autos;` [NEW]

The composable routine builder. A `.auto` file drawn in the PathPlanner GUI is the right tool for
*geometry*; it is the wrong tool for *logic* like "if the first piece was missed, skip the second
pickup and shoot from here." So every routine in this file is a small Java composition of
`AutoFactory.followPath(...)` segments and superstructure commands, with the GUI owning only path
shapes. Each routine is a `public Command xxx()` returning a fresh command instance every call
(never a cached one — a `Command` may only be in one composition at a time, and re-selecting an auto
in the chooser would otherwise throw). Two building blocks do the heavy lifting: `driveAndIntake`
(follow a path while the superstructure is in `INTAKE`, ending when a piece is detected or the path
ends) and `driveAndShoot` (follow a path while the superstructure is in `AUTO_SCORE_ON_MOVE`, which
is where shoot-while-moving during auto actually happens — the turret and solver do not care that
the chassis velocity came from PathPlanner instead of a joystick).

```java
package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.List;

public class AutoRoutines {
  private final AutoFactory factory;
  private final Superstructure superstructure;

  public AutoRoutines(AutoFactory factory) {
    // TODO: Store the factory and cache factory.getSuperstructure(). Do NOT build any Command here;
    // build them lazily in the accessors so each chooser selection gets a fresh instance.
  }

  // ------------------------------------------------------------------------- building blocks ----

  public Command driveAndIntake(String pathName) {
    // TODO: Commands.deadline(factory.followPath(pathName),
    //   superstructure.intakeWhileHeldCommand())
    //   .andThen(Commands.deadline(Commands.waitSeconds(Constants.Auto.kIntakeSettleSec.get()),
    //       superstructure.intakeWhileHeldCommand())).
    // A fixed dwell, not a wait-for-piece: there is no piece sensor (section 2.4), so the
    // routine cannot know whether the pickup worked and does not pretend to.
    return null;
  }

  public Command driveIntakeAndShoot(String pathName) {
    // TODO: Commands.deadline(factory.followPath(pathName),
    //   superstructure.intakeWhileHeldCommand(),
    //   superstructure.setWantedStateCommand(Superstructure.WantedState.AUTO_SCORE_ON_MOVE))
    //   .andThen(superstructure.setWantedStateCommand(Superstructure.WantedState.DEFAULT)).
    // The hopper-robot pattern: sweep through pieces while firing whatever is aboard, no stop and
    // no separate pickup leg. Pieces are fired as they arrive; the intake stays down throughout.
    return null;
  }

  public Command driveAndShoot(String pathName) {
    // TODO: Commands.deadline(factory.followPath(pathName),
    //   superstructure.setWantedStateCommand(Superstructure.WantedState.AUTO_SCORE_ON_MOVE)).
    // This is shoot-on-the-move in auto: the path supplies the chassis velocity, RobotState
    // measures it, and the solver's virtual-goal offset handles the rest. No stop is inserted.
    return null;
  }

  public Command shootInPlace(double timeoutSec) {
    // TODO: factory.scoreStationaryCommand().withTimeout(timeoutSec) preceded by a
    // Commands.runOnce(drive::stop). Used only for the preload shot on routines that start
    // pre-aimed, where standing still for 0.6 s is faster than solving a moving shot.
    return null;
  }

  public Command spinUpWhile(Command inner) {
    // TODO: Commands.deadline(inner,
    // superstructure.setWantedStateCommand(Superstructure.WantedState.SPIN_UP)) so the flywheel is
    // already at speed when the shooting segment begins. Saves ~0.8 s per cycle.
    return null;
  }

  public Command sequenceOf(List<String> pathNames, boolean shootBetween) {
    // TODO: Fold pathNames into a SequentialCommandGroup alternating driveAndIntake and
    // driveAndShoot (or plain followPath when shootBetween is false). Exists so a student can add a
    // new N-piece routine by listing path names, with zero new control-flow code.
    return null;
  }

  public Command aimTurretAt(Rotation2d fieldAngle) {
    // TODO: Commands.runOnce(() -> turret.setGoal(fieldAngle, 0.0)) then
    // Commands.waitUntil(turret::atGoal).withTimeout(1.0). Used to pre-position the turret during a
    // long drive so the first shot of a segment does not wait on turret travel.
    return null;
  }

  // ------------------------------------------------------------------------------- routines ----

  public Command doNothing() {
    // TODO: Commands.none(). MUST be the chooser's default option - a mis-selected auto that does
    // nothing loses 15 points; one that drives into a partner loses the match.
    return null;
  }

  public Command driveForward() {
    // TODO: factory.followPath("Leave") only. The zero-risk mobility auto; also the smoke test that
    // proves AutoBuilder, RobotConfig and the pose reset are all wired correctly.
    return null;
  }

  public Command preloadOnly() {
    // TODO: resetPoseToPathStart("Center_Preload") -> waitUntilIntakeRetaining() ->
    // spinUpWhile(shootInPlace(...)) -> followPath("Center_Preload"). Fires the 8 preloaded pieces
    // then leaves. The fallback routine when vision is down: it depends on nothing but odometry.
    // Then factory.logPreloadFiredCommand(), which logs Auto/PreloadFired: 8 pieces in is a known
    // ground truth, so every match's preload burst checks the shot detector for free.
    return null;
  }

  public Command waitUntilIntakeRetaining() {
    // Implemented in M10b step 9, with the timeout read inside Commands.defer so a retune takes
    // effect without a reboot: Commands.waitUntil(factory.getIntake()::isRetainingPieces)
    //   .withTimeout(Constants.Auto.kIntakeRetainTimeoutSec.get()). EVERY routine
    // starts with this before its first path: the robot starts with the intake STOWED (a rule),
    // Superstructure.onEnable() deploys it at autonomousInit, and pieces fall out of a stowed
    // intake if the robot moves first. The timeout exists so a dead pivot costs a second, not
    // the auto.
    return null;
  }

  public Command centerThreePiece() {
    // TODO: Center start. resetPose -> shootInPlace -> driveAndIntake("Center_ToPiece1") ->
    // driveAndShoot("Center_Score1") -> driveAndIntake("Center_ToPiece2") ->
    // driveAndShoot("Center_Score2"). (There is no bailoutIfEmpty any more: a missed pickup cannot
    // be detected without a sensor, and a moving scoring leg over an empty hopper costs no stop.)
    return null;
  }

  public Command centerFivePiece() {
    // TODO: The showcase routine. centerThreePiece() extended with
    // driveAndIntake("Center_ToPiece3") -> driveAndShoot("Center_Score3"), and every scoring leg
    // uses driveAndShoot (never a stop). Expect to need kAccelCompensationScalar > 0 for this one
    // because the path's acceleration segments are where the moving shots go wide.
    return null;
  }

  public Command ampSideThreePiece() {
    // TODO: Left/amp-side mirror of centerThreePiece using the Amp_* paths. Written blue-only;
    // AllianceFlipUtil + AutoBuilder's shouldFlip callback handle red.
    return null;
  }

  public Command sourceSideThreePiece() {
    // TODO: Right/source-side mirror using the Source_* paths.
    return null;
  }

  public Command sourceSideSteal() {
    // TODO: Drive across to the opponent's staging area on "Source_Steal", intake two pieces, and
    // return to shoot. High risk, high reward - only run it when the alliance agrees beforehand.
    return null;
  }

  public Command dynamicScavenge() {
    // TODO: The vision-driven routine. Loop up to kMaxScavengeCycles: if
    // vision.getGamePieceFieldPose() is present, AutoBuilder.pathfindToPose(that pose,
    // AutoFactory.defaultPathfindConstraints(), 0.0) with intakeWhileHeldCommand(); else drive to the next static
    // waypoint. Then driveAndShoot back. Depends on the intake Limelight's game-piece pipeline
    // being tuned, so gate it behind a chooser entry students can avoid at an early event.
    return null;
  }

  public Command characterizationPlaceholder() {
    // TODO: Commands.none(); RobotContainer replaces this chooser slot with the
    // CharacterizationCommands entries when Constants.kTuningMode is true.
    return null;
  }
}
```

#### `src/main/java/frc/robot/autos/AutoSelector.java` — `package frc.robot.autos;` [NEW]

Wraps AdvantageKit's `LoggedDashboardChooser<Command>` (logged, therefore replayable: you can tell
from a log which routine the drive team actually selected) and adds the one thing the stock chooser
lacks — a *preview*. Every disabled loop it publishes the selected routine's expected starting pose
as `AutoSelector/StartingPose`, so AdvantageScope's 2D field shows a ghost robot where the routine
thinks you are standing. Comparing that ghost against the live `RobotState/EstimatedPose` in the
queue line catches a robot placed 30 cm off the tape, which is the single most common cause of an
auto that "worked in practice."

```java
package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoSelector {
  private final LoggedDashboardChooser<Command> chooser =
      new LoggedDashboardChooser<>("Auto Choice");
  private final LoggedDashboardChooser<String> startPoseNameChooser =
      new LoggedDashboardChooser<>("Auto Start Path");
  private final AutoFactory factory;
  private final AutoRoutines routines;
  private String lastSelectedName = "";

  public AutoSelector(AutoFactory factory, AutoRoutines routines) {
    // TODO: Store both, then call populate(). Construct after AutoFactory so NamedCommands are
    // already registered - building a PathPlannerAuto whose markers are not yet registered throws.
  }

  private void populate() {
    // TODO: chooser.addDefaultOption("Do Nothing", routines.doNothing()); then addOption for each
    // of the eight routines in AutoRoutines, plus a parallel startPoseNameChooser option per
    // routine naming the path whose start pose to preview. Keep the two choosers' option order
    // identical so the preview always matches the selection.
  }

  public void addOption(String name, Command command) {
    // TODO: chooser.addOption(name, command). Public so RobotContainer can append the
    // CharacterizationCommands entries only when Constants.kTuningMode is true.
  }

  public Command get() {
    // TODO: Command selected = chooser.get(); return selected == null ? Commands.none() : selected.
    // Never return null - Robot.autonomousInit would then silently do nothing.
    return null;
  }

  public String getSelectedName() {
    // TODO: chooser.getSendableChooser().getSelected() is NOT the display name; instead track the
    // name via the parallel startPoseNameChooser. Return that string, logged as
    // "AutoSelector/Selected".
    return "";
  }

  public Optional<Pose2d> getSelectedStartingPose() {
    // TODO: factory.getStartingPose(getSelectedName()) if the selection maps to a .auto file, else
    // derive it from the first path in the routine. Alliance-flipped by AutoFactory already.
    return Optional.empty();
  }

  public void logStartingPose() {
    // TODO: Called every disabledPeriodic from Robot. Logger.recordOutput(
    // "AutoSelector/StartingPose", getSelectedStartingPose().orElse(Pose2d.kZero)) and
    // Logger.recordOutput("AutoSelector/StartPoseError",
    // RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(startingPose
    // .getTranslation())). Also fire a FaultReporter warning alert when that error exceeds
    // Constants.Auto.kStartPoseWarnMeters - a warning in the queue line is free; a blown auto is not.
  }

  public boolean selectionChanged() {
    // TODO: Compare getSelectedName() to lastSelectedName, update it, and return whether it moved.
    // Robot uses this to avoid re-logging the preview at 50 Hz when nothing changed.
    return false;
  }
}
```

---

### 4.13 Commands

#### `src/main/java/frc/robot/commands/DriveCommands.java` — `package frc.robot.commands;` [MODIFY]

The template's file already has `joystickDrive`, `joystickDriveAtAngle`,
`feedforwardCharacterization` and `wheelRadiusCharacterization`. Four changes. (1) Every read of
`drive.getRotation()` becomes `RobotState.getInstance().getRotation()`, because `Drive` no longer
owns a pose — this is the one-line change that makes field-relative driving use the *fused* heading
instead of raw odometry. (2) The private tuning constants become `Constants.Drivetrain` tunables.
(3) A new `joystickDriveWithOverride` lets the `Superstructure`'s `WAITING_FOR_TURRET` state blend a
requested heading into the driver's rotation stick, so the drivetrain helps the turret reach an aim
point it cannot reach alone, without ever taking the robot away from the driver. (4) A
`gamePieceAssist` that biases translation toward the intake camera's best game-piece detection while
the driver still controls speed. Both assists are *additive nudges*, never full takeovers — a driver
who cannot override an assist will not use it.

```java
package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double FF_START_DELAY = 2.0; // KEEP template value.
  private static final double FF_RAMP_RATE = 0.1; // KEEP template value.
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // KEEP template value.
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // KEEP template value.

  private DriveCommands() {}

  private static edu.wpi.first.math.geometry.Translation2d getLinearVelocityFromJoysticks(
      double x, double y) {
    // TODO: KEEP the template body (deadband on the magnitude, square the magnitude, rebuild via
    // Pose2d/Translation2d rotation) but read the deadband from
    // Constants.Drivetrain.kJoystickDeadband.get() instead of the private DEADBAND field.
    return null;
  }

  public static Command joystickDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    // TODO: KEEP the template body, with one substitution: build the field-relative ChassisSpeeds
    // with ChassisSpeeds.fromFieldRelativeSpeeds(..., isFlipped ? RobotState.getInstance()
    // .getRotation().plus(Rotation2d.kPi) : RobotState.getInstance().getRotation()).
    return null;
  }

  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    // TODO: KEEP the template body (ProfiledPIDController with continuous input enabled), but take
    // kP/kD/max velocity/max acceleration from Constants.Drivetrain.kHeading* tunables and refresh
    // them each loop with LoggedTunableNumber.ifChanged. Heading comes from RobotState.
    return null;
  }

  public static Command joystickDriveWithOverride(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Optional<Rotation2d>> headingOverrideSupplier) {
    // TODO: Each loop, if headingOverrideSupplier.get() is present AND |omegaSupplier| is below
    // Constants.Drivetrain.kJoystickDeadband, run the joystickDriveAtAngle heading controller
    // toward that angle; otherwise use the driver's omega verbatim. Bind this as the default drive
    // command with superstructure::getRequestedDriveHeading as the supplier. Log
    // "Drive/HeadingOverrideActive".
    return null;
  }

  public static Command gamePieceAssist(
      Drive drive, Vision vision, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    // TODO: Read vision.getBestGamePieceTarget(). Rotate the chassis so the target's tx goes to
    // zero using the same ProfiledPIDController gains as joystickDriveAtAngle, and pass the
    // driver's translation through unchanged (robot-relative, not field-relative, so "forward" means
    // "toward the piece"). If the Optional is empty, degrade to plain joystickDrive rather than
    // stopping. Bind to a held button only.
    return null;
  }

  public static Command snapToHeading(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Rotation2d blueHeading) {
    // TODO: joystickDriveAtAngle(drive, x, y, () -> AllianceFlipUtil.apply(blueHeading)). Bound to
    // the four POV directions so a driver can square up to the field without looking.
    return null;
  }

  public static Command feedforwardCharacterization(Drive drive) {
    // TODO: KEEP the template body verbatim, but wrap it in
    // Commands.runOnce(() -> drive.setUseSetpointGenerator(false)) ... .finallyDo(() ->
    // drive.setUseSetpointGenerator(true)); the setpoint generator would otherwise slew the ramp
    // and corrupt the kV/kS fit.
    return null;
  }

  public static Command wheelRadiusCharacterization(Drive drive) {
    // TODO: KEEP the template body verbatim (including the private
    // WheelRadiusCharacterizationState holder class), with the same setpoint-generator bypass as
    // above.
    return null;
  }

  public static Command autoAlignToNearest(
      Drive drive, Supplier<java.util.List<edu.wpi.first.math.geometry.Pose2d>> candidates) {
    // TODO: Pick the candidate pose minimizing distance from RobotState.getEstimatedPose(), then
    // return new DriveToPoseCommand(drive, () -> chosen). Evaluate the choice at schedule time, not
    // every loop, so the target does not flip back and forth between two nearly-equal poses.
    return null;
  }
}
```

#### `src/main/java/frc/robot/commands/DriveToPoseCommand.java` — `package frc.robot.commands;` [NEW]

Auto-align with a pathfind handoff. Far from the goal (beyond
`Constants.Drivetrain.kPathfindHandoffMeters`) it delegates to
`AutoBuilder.pathfindToPose(...)`, which runs the AD* pathfinder and routes around field obstacles
from `navgrid.json`. Inside that radius it switches to two `ProfiledPIDController`s on X and Y plus
one on heading, because pathfinding's terminal accuracy is worse than a direct profiled controller's
and its replanning near the goal produces visible twitch. The concrete capability: the driver taps A
anywhere on the field and the robot arrives at a scoring pose, square, at a repeatable ±3 cm — which
is what makes a scripted teleop cycle possible.

```java
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveToPoseCommand extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> targetSupplier;
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private Pose2d target = Pose2d.kZero;
  private Command pathfindCommand = null;
  private boolean usingPathfind = false;

  public DriveToPoseCommand(Drive drive, Supplier<Pose2d> targetSupplier) {
    // TODO: Store both; construct the three ProfiledPIDControllers from
    // Constants.Drivetrain.kAutoAlign* tunables with TrapezoidProfile.Constraints; call
    // thetaController.enableContinuousInput(-Math.PI, Math.PI); addRequirements(drive).
  }

  public DriveToPoseCommand(Drive drive, Pose2d target) {
    // TODO: Delegate to this(drive, () -> target).
  }

  @Override
  public void initialize() {
    // TODO: target = targetSupplier.get(); reset all three controllers to the CURRENT measurement
    // and CURRENT velocity (x.reset(pose.getX(), speeds.vxMetersPerSecond), etc.) so the profile
    // starts from the robot's real motion rather than from zero - this is what removes the
    // characteristic lurch at the start of an auto-align. Decide usingPathfind from the distance to
    // target and, if true, schedule AutoBuilder.pathfindToPose(target,
    // AutoFactory.defaultPathfindConstraints(), 0.0).
  }

  @Override
  public void execute() {
    // TODO: If usingPathfind and still outside kPathfindHandoffMeters, let the scheduled pathfind
    // command run and do nothing here. Otherwise cancel it, set usingPathfind = false, and compute
    // field-relative vx/vy/omega from the three controllers evaluated against
    // RobotState.getInstance().getEstimatedPose(), then
    // drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega,
    // RobotState.getInstance().getRotation())). Log target, error and mode every loop.
  }

  @Override
  public boolean isFinished() {
    // TODO: atGoal() - translation error below Constants.Drivetrain.kAutoAlignToleranceMeters AND
    // heading error below a tunable AND chassis speed below a tunable, all true for two consecutive
    // loops (use a Debouncer) so a single noisy vision frame cannot end the command early.
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // TODO: Cancel pathfindCommand if non-null and call drive.stop().
  }

  public boolean atGoal() {
    // TODO: Public form of the isFinished() test, so the Superstructure can gate a shot on it.
    return false;
  }

  public double getDistanceToTargetMeters() {
    // TODO: RobotState.getInstance().getEstimatedPose().getTranslation()
    // .getDistance(target.getTranslation()); logged as "DriveToPose/Distance".
    return 0.0;
  }
}
```

#### `src/main/java/frc/robot/commands/CharacterizationCommands.java` — `package frc.robot.commands;` [NEW]

Every closed-loop mechanism on this robot needs kS/kV/kA before its gains mean anything, and
guessing them costs more practice time than measuring them. This file centralizes the routines so a
student runs one chooser entry, pulls the WPILOG into SysId, and reads off numbers. Two flavors are
provided: WPILib `SysIdRoutine` sweeps (quasistatic + dynamic, both directions) for mechanisms that
can travel freely, and a simpler stair-step "measure kS and kV directly" routine for the flywheel,
where a slow voltage ramp gives a cleaner kV than a SysId sweep does. Every routine calls
`Superstructure.setWantedState(CHARACTERIZE)` first so nothing else moves, and every one is
registered into the auto chooser only when `Constants.kTuningMode` is true.

```java
package frc.robot.commands;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.floorroller.FloorRoller;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class CharacterizationCommands {
  private CharacterizationCommands() {}

  private static SysIdRoutine buildRoutine(
      String name,
      Consumer<Voltage> voltageConsumer,
      DoubleSupplier positionRadSupplier,
      DoubleSupplier velocityRadPerSecSupplier,
      SubsystemBase requirement) {
    // TODO: new SysIdRoutine(new SysIdRoutine.Config(null, Volts.of(4.0), null,
    // state -> Logger.recordOutput("SysId/" + name + "/State", state.toString())),
    // new SysIdRoutine.Mechanism(voltageConsumer, null, requirement)). Passing null for the log
    // consumer is correct here because AdvantageKit's WPILOG already captures the @AutoLog inputs
    // SysId needs; verify against AdvantageKit 26.0.2's SysId docs before relying on it.
    return null;
  }

  public static Command turretSysId(Turret turret, Superstructure superstructure) {
    // TODO: Build a routine driving turret::setCharacterizationVolts and reading
    // turret::getPositionRad / turret::getCharacterizationVelocityRadPerSec. Sequence
    // quasistatic(kForward), quasistatic(kReverse), dynamic(kForward), dynamic(kReverse) with
    // Commands.waitSeconds(1.0) between, and bound each leg with
    // Commands.waitUntil(() -> turret.getPositionRad() > Constants.Turret.kMaxAngleRad.get() - 0.2)
    // so the sweep stops before the hard stop. This is the one SysId routine where a missing limit
    // check breaks hardware.
    return null;
  }

  public static Command intakePivotSysId(Intake intake, Superstructure superstructure) {
    // TODO: Same four-leg sequence driving intake::setCharacterizationVolts. Gravity means the
    // resulting fit yields kG as well as kS/kV/kA - use SysId's "Arm" mechanism type when
    // analyzing, not "Simple".
    return null;
  }

  public static Command shooterSysId(Shooter shooter, Superstructure superstructure) {
    // TODO: Quasistatic + dynamic on shooter::setCharacterizationVolts reading
    // shooter::getCharacterizationVelocityRotPerSec. Analyze as "Simple". Run it with a game piece
    // NOT in the indexer.
    return null;
  }

  public static Command shooterStaircase(Shooter shooter, double maxVolts, double stepVolts) {
    // TODO: Hand-rolled alternative to SysId for the flywheel: step the applied voltage from 0 to
    // maxVolts in stepVolts increments, holding each step until velocity settles (velocity change
    // < 25 RPM over 0.25 s), logging (volts, steady-state RPM) pairs under
    // "Characterization/Shooter/Steps". kV is the slope of that line and kS is its intercept. This
    // gives a better flywheel kV than a SysId sweep because it measures true steady state.
    return null;
  }

  public static Command indexerFeedforward(Indexer indexer) {
    // TODO: Same staircase idea for the indexer, since a 4-leg SysId sweep on a roller is overkill.
    return null;
  }

  public static Command floorRollerFeedforward(FloorRoller floorRoller) {
    // TODO: Same staircase for the floor roller.
    return null;
  }

  public static Command driveSysIdFull(Drive drive) {
    // TODO: Sequence drive.sysIdQuasistatic(kForward), (kReverse), drive.sysIdDynamic(kForward),
    // (kReverse) with waits, wrapped in setUseSetpointGenerator(false)/true. Exists so the stock
    // template's four separate chooser entries collapse into one.
    return null;
  }

  public static Command turretUnwrapDemo(Turret turret) {
    // TODO: Not a characterization - a safety verification. Command the turret through a scripted
    // list of field angles that straddle the wrap boundary while the chassis is held still, and
    // assert in the log that the commanded position never leaves [kMinAngleRad, kMaxAngleRad].
    // Run this once after every mechanical change to the turret stop positions.
    return null;
  }

  public static Command measureShotTablePoint(
      Superstructure superstructure,
      Shooter shooter,
      DoubleSupplier rpm,
      DoubleSupplier tapedDistanceMeters,
      double holdSec) {
    // Rewritten by M11 (the original fired one piece, which section 2.4 retired). Reads the speed
    // and the taped distance when it starts; sets Superstructure.setShotTableRpm(rpm) and requests
    // SCORE_STATIONARY, which aims at the goal with no table and fires a whole BURST through the
    // ordinary stationary gate. Ends when likelyEmpty() after at least one shot of THIS burst (the
    // belief outlives a burst, and a hand-reloaded hopper does not reset it), or at holdSec. However
    // it ends, it requests DEFAULT -- which also clears the override -- and logs one sample under
    // ShotTable/: speed, estimated and taped distance, shots fired this burst, duration, timed out.
    // Bound in tuning mode to the /Tuning/ShotTable/Measure dashboard toggle. The rows go through
    // tools/fit_shot_table.py into Constants.Shot.
    return null;
  }
}
```

---

### 4.14 Tests

> These run with `./gradlew test` on a laptop, with no robot and no HAL. That is the entire point:
> the three things most likely to be wrong in this codebase — solver convergence, turret unwrap, and
> FSM guards — are exactly the three things that are miserable to debug on a robot. WPILib's
> `frc.gradlerio` plugin already puts JUnit 5 (`org.junit.jupiter`) on the test classpath, so no
> build.gradle change is needed for these. `ShootingSolverTest` and `TurretUnwrapperTest` touch no
> WPILib HAL at all; `SuperstructureFsmTest` does, so it must set `Constants.disableHal = true` and
> call `edu.wpi.first.hal.HAL.initialize(500, 0)` in `@BeforeEach`.

#### `src/test/java/frc/robot/shooting/ShootingSolverTest.java` — `package frc.robot.shooting;` [NEW]

Uses the test-only `ShootingSolver(double[], double[])` constructor with a synthetic,
monotonic time-of-flight table, so the tests assert the *algorithm*, not your (currently unmeasured)
shot data. The single most valuable assertion here is `stationarySolveMatchesMovingSolveAtZeroVelocity`:
if that fails, the virtual-goal math has a sign error, and a sign error in the virtual goal is
indistinguishable on the field from a mis-tuned shot table.

```java
package frc.robot.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShootingSolverTest {
  private static final double kEps = 1e-6;
  private static final double[] kDistances = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  private static final double[] kTof = {0.30, 0.42, 0.55, 0.69, 0.84, 1.00};

  private ShootingSolver solver;

  @BeforeEach
  void setUp() {
    // TODO: Constants.disableHal = true; solver = new ShootingSolver(kDistances, kTof). Setting
    // disableHal first keeps LoggedTunableNumber off NetworkTables.
  }

  @Test
  void stationarySolveMatchesMovingSolveAtZeroVelocity() {
    // TODO: ShotParameters a = solver.solve(pose, new ChassisSpeeds(), Translation2d.kZero, goal);
    // ShotParameters b = solver.solveStationary(pose, goal);
    // assertEquals on turretFieldAngle().getRadians(), flywheelRpm(), timeOfFlightSec() within kEps.
  }

  @Test
  void stationaryAimPointsDirectlyAtGoal() {
    // TODO: With the robot at (0,0) and the goal at (4,0), assert
    // solveStationary(...).turretFieldAngle().getRadians() == 0.0 within kEps, and with the goal at
    // (0,4) assert it equals Math.PI/2. Catches a swapped atan2 argument order.
  }

  @Test
  void movingPerpendicularShiftsAimOppositeMotion() {
    // TODO: Robot at (0,0), goal at (4,0), field-relative velocity (0, +2 m/s). Assert the solved
    // turretFieldAngle is NEGATIVE (aim behind the motion) and that its magnitude is approximately
    // atan2(v * tof, d). This is the shoot-on-the-move test; it is the one that fails when someone
    // "fixes" a sign.
  }

  @Test
  void movingRadiallyChangesRangeNotAngle() {
    // TODO: Same geometry with velocity (+2, 0). Assert the aim angle is still ~0 rad but that
    // effectiveDistanceMeters is SMALLER than the stationary distance (you are closing on the goal,
    // so the piece needs less range).
  }

  @Test
  void solverConvergesWithinIterationCap() {
    // TODO: Over a grid of distances 1.5-6.0 m and speeds 0-4 m/s in both axes, assert
    // solver.getLastIterationCount() <= Constants.Shot.kMaxSolverIterations and
    // solver.getLastResidualSec() <= Constants.Shot.kSolverToleranceSec.get() for every case.
  }

  @Test
  void solverIsIdempotent() {
    // TODO: Call solve(...) twice with identical arguments and assert the two ShotParameters are
    // equal. Records give you equals() for free; this catches accidental mutable state in the solver.
  }

  @Test
  void outOfRangeIsFlaggedNotClamped() {
    // TODO: Place the goal 20 m away. Assert inRange() is false and isUsable() is false, and assert
    // the returned flywheelRpm is NOT silently clamped to kMaxRpm - a clamped RPM would let the
    // Superstructure fire a shot that cannot reach.
  }

  @Test
  void accelerationTermIsDisabledAtScalarZero() {
    // TODO: With kAccelCompensationScalar at its default 0, assert that a large
    // fieldRelativeAccel argument produces an identical result to a zero one. Guards the "start at
    // zero, enable later" rollout plan in section 6.3.
  }

  @Test
  void tableLookupClampsAtBothEnds() {
    // TODO: assertEquals(kTof[0], solver.getTimeOfFlight(0.1)) and
    // assertEquals(kTof[kTof.length - 1], solver.getTimeOfFlight(99.0)). Extrapolating a TOF table
    // produces negative flight times and NaN aim angles; clamping must be verified.
  }

  @Test
  void mismatchedTableLengthsThrow() {
    // TODO: assertThrows(IllegalArgumentException.class, () -> new ShootingSolver(new double[3],
    // new double[2], new double[3])).
  }
}
```

#### `src/test/java/frc/robot/shooting/TurretUnwrapperTest.java` — `package frc.robot.shooting;` [NEW]

`TurretUnwrapper` is a pure static function over four doubles, which makes it the cheapest thing in
this codebase to test exhaustively and the most expensive thing to get wrong on the robot (a snapped
turret belt costs a match; these tests cost ten minutes).

```java
package frc.robot.shooting;

import org.junit.jupiter.api.Test;

public class TurretUnwrapperTest {
  private static final double kMin = Math.toRadians(-200.0);
  private static final double kMax = Math.toRadians(200.0);
  private static final double kEps = 1e-9;

  @Test
  void picksNearestBranchWithinLimits() {
    // TODO: solve(current = 0.0, desired = 0.1, kMin, kMax) must return targetPositionRad == 0.1,
    // not 0.1 - 2*PI. Assert reachable() is true and requiresLongWayAround() is false.
  }

  @Test
  void unwrapsPastPiWhenTheFarBranchIsCloser() {
    // TODO: current = Math.toRadians(170), desired = Math.toRadians(-170) (which is +190 unwrapped).
    // The correct answer is +190 deg, NOT -170 deg: it is 20 deg of travel instead of 340. Assert
    // targetPositionRad is within kEps of Math.toRadians(190).
  }

  @Test
  void refusesToCrossTheHardStop() {
    // TODO: current = Math.toRadians(195), desired = Math.toRadians(-150) with limits +/-200. The
    // only in-range candidate is -150 deg (345 deg of travel the other way); assert the returned
    // target is inside [kMin, kMax] and that requiresLongWayAround() is true.
  }

  @Test
  void reportsUnreachableWhenNoBranchFits() {
    // TODO: Narrow the limits to +/- 30 deg and request 180 deg. Assert reachable() is false and
    // targetPositionRad equals MathUtil.clamp of the base branch into [kMin, kMax] - the turret
    // must hold at its stop, not wind up against it.
  }

  @Test
  void hysteresisPreventsBranchChatter() {
    // TODO: Pick a current position exactly equidistant from two legal branches. Call solve with
    // hysteresis = Math.toRadians(10) twice, perturbing the desired direction by +/-0.01 rad
    // between calls, and assert the chosen branch does not change. Without this, the turret
    // oscillates 360 deg at the boundary - loudly, and in front of judges.
  }

  @Test
  void zeroHysteresisOverloadIsDeterministic() {
    // TODO: Assert the four-argument overload returns the same Result for the same inputs on
    // repeated calls, and that it always picks the strictly-nearest candidate.
  }

  @Test
  void errorRadIsSignedAndConsistent() {
    // TODO: Assert result.errorRad() == result.targetPositionRad() - currentPositionRad for several
    // cases, including one where the target is negative. Turret.atGoal() compares against this.
  }

  @Test
  void clampIntoRangeMatchesMathUtil() {
    // TODO: Spot-check clampIntoRange against MathUtil.clamp for values below, inside and above.
  }

  @Test
  void passesThroughArcDetectsOverlapInBothDirections() {
    // TODO: passesThroughArc(from = 2.0, to = 4.0, arcMin = 2.6, arcMax = 3.7) is true;
    // passesThroughArc(4.0, 2.0, 2.6, 3.7) is also true (direction must not matter);
    // passesThroughArc(0.0, 1.0, 2.6, 3.7) is false. This is the intake interlock's core predicate.
  }

  @Test
  void limitMarginIsRespected() {
    // TODO: Pass limits already reduced by Constants.Turret.kLimitMarginRad and assert no returned
    // target ever equals exactly kMin or kMax over a sweep of 720 desired directions in 1 deg steps.
  }
}
```

#### `src/test/java/frc/robot/subsystems/superstructure/SuperstructureFsmTest.java` — `package frc.robot.subsystems.superstructure;` [NEW]

Builds a real `Superstructure` on top of real subsystems wired to *empty* `XxxIO` implementations
(the same `new XxxIO() {}` anonymous classes `RobotContainer` uses in REPLAY mode), then steps
`periodic()` by hand. No `CommandScheduler`, no HAL beyond `HAL.initialize`. This is how you verify
the guards and timeouts that are otherwise only exercised by a misbehaving robot: you can assert
"requesting SCORE without a usable shot solution never feeds" in 40 ms instead of discovering it in
a qualification match. The hopper replan (§2.4) retires the piece-gate cases and adds the intake
region, burst, empty-belief and jam cases below; the Shooter and Indexer FSM suites gain matching
unit cases (shot detection and jam recovery respectively).

```java
package frc.robot.subsystems.superstructure;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SuperstructureFsmTest {
  private Superstructure superstructure;
  private frc.robot.subsystems.intake.Intake intake;
  private frc.robot.subsystems.floorroller.FloorRoller floorRoller;
  private frc.robot.subsystems.indexer.Indexer indexer;
  private frc.robot.subsystems.turret.Turret turret;
  private frc.robot.subsystems.shooter.Shooter shooter;

  @BeforeEach
  void setUp() {
    // TODO: Constants.disableHal = true; assert HAL.initialize(500, 0); construct each subsystem
    // with an empty anonymous IO (new IntakeIO() {} etc.), construct RobotState via
    // RobotState.initialize(new SwerveDriveKinematics(Drive.getModuleTranslations())), then
    // construct the Superstructure. Drive and Vision can be passed as null ONLY if no tested path
    // dereferences them; otherwise stub them the same way.
  }

  @AfterEach
  void tearDown() {
    // TODO: HAL.shutdown() and reset the RobotState singleton (add a package-private
    // RobotState.resetForTest() if needed) so tests do not leak state into one another.
  }

  private void step(int loops) {
    // TODO: Call superstructure.periodic() (and each subsystem's periodic()) `loops` times, in the
    // same order RobotContainer registers them, so ordering bugs show up here rather than on the
    // robot.
  }

  @Test
  void startsInHomingAndReachesStowed() {
    // TODO: assertEquals(SystemState.HOMING, superstructure.getSystemState()) at construction; mark
    // both mechanisms homed via their empty IO inputs; step(5); assert RESTING, and assert the
    // intake is STOWED (the robot starts inside its frame; nothing has asked it to deploy).
  }

  @Test
  void scoringDoesNotRequireAPiece() {
    // TODO: The replacement for the retired scoreRequestWithoutPieceNeverReachesScoring. With a
    // usable shot and no simulated load anywhere, setWantedState(SCORE_ON_MOVE); step(50). Assert
    // the state reaches SCORING_ON_THE_MOVE and the indexer is asked to FEED once the gate opens.
    // Pins the decision in section 2.4 so nobody quietly re-adds a piece guard.
  }

  @Test
  void scoreOnMoveRequiresUsableShotSolution() {
    // TODO: Leave the shot tables at their default zeros so ShotParameters.isUsable() is false.
    // Assert the FSM settles in WAITING_FOR_TURRET, not SCORING_ON_THE_MOVE, and the indexer is
    // never FEED.
  }

  @Test
  void releasingIntakeLeavesItDeployed() {
    // TODO: setIntakeRequested(true); step until the intake is INTAKING; setIntakeRequested(false);
    // step(20). Assert the intake is DEPLOYED with the roller stopped, and never passed through
    // STOWING. Then setWantedState(DEFAULT) and assert it STILL does not stow; only STOW does.
  }

  @Test
  void intakeScoreAndDriveTogether() {
    // TODO: Hold both setIntakeRequested(true) and SCORE_ON_MOVE with a usable shot. Assert
    // IntakeMode == INTAKING, the turret is AIMING (not stowed), and the indexer FEEDs when the
    // gate opens. The old intaking() stowed the turret; this proves that coupling is gone.
  }

  @Test
  void feedAssistOnlyWhileGateOpenAndNotIntaking() {
    // TODO: Deployed posture, SCORE_ON_MOVE, intake not held. Gate closed -> IntakeMode DEPLOYED;
    // gate open -> FEED_ASSIST; then hold intake -> INTAKING wins. Release score -> DEPLOYED.
  }

  @Test
  void feedAssistRocksAndNeverBlocksTheTurret() {
    // TODO: With the default constants, score through the turret's conflict arc. Assert the
    // intake's pivot goal alternates between the deployed and feed-assist angles every half
    // period, isInTurretConflictBand() stays false throughout, and the turret still aims. Then
    // drop the LOW power scalar below 1 and assert the rocking stops at the feed-assist angle.
  }

  @Test
  void misconfiguredFeedAssistYieldsToTurret() {
    // TODO: The guard for a mis-tuned constant: set kFeedAssistAngleRad inside the band, aim
    // through the arc, assert the intake stays DEPLOYED, FeedAssistBlocked is logged and the alert
    // fires.
  }

  @Test
  void floorRollerLeadsAndIndexerGates() {
    // TODO: Request SCORE_ON_MOVE with the gate held closed (e.g. unusable shot). Assert the floor
    // roller is FEED from the first loop while the indexer stays IDLE; open the gate and assert
    // the indexer then FEEDs. Separately: intake held with no shot requested -> floor roller IDLE;
    // SPIN_UP alone -> floor roller IDLE.
  }

  @Test
  void firstEnableDeploysTheIntake() {
    // TODO: Boot (posture STOWED), home, call onEnable(); step until settled. Assert the intake
    // reaches DEPLOYED and isRetainingPieces() becomes true. Assert the turret was refused its
    // conflict arc until the pivot left the band, and allowed it afterwards.
  }

  @Test
  void burstLatchKeepsFeedingThroughRecovery() {
    // TODO: Open the gate, then injectShot() into ShooterIOSim at a steady cadence. Assert the
    // indexer stays FEED across every dip that stays inside kBurstToleranceRpm (no FEED->IDLE
    // chatter), and that a dip beyond it DOES close the gate. Then stale the pose (term 5) and
    // assert the latch clears. Count FEED->IDLE transitions, not the final state.
  }

  @Test
  void likelyEmptyOnlyAccumulatesWithTheGateOpen() {
    // TODO: The mutation target. (a) Gate closed for 10 x kEmptyConfirmSec: assert likelyEmpty()
    // stays false. (b) Gate open, no load, no shots: assert it becomes true after ~kEmptyConfirmSec.
    // (c) Repeat (b) but with setSimulatedLoad(true): stays false. (d) Repeat (b) with a shot
    // injected halfway: the clock restarts. (e) Intake held: the clock resets.
  }

  @Test
  void scoreUntilEmptyEndsOnBeliefOrTimeout() {
    // TODO: Run scoreUntilEmptyCommand() by hand-stepping its command. Assert it ends early once
    // likelyEmpty() is true, and at kScoreTimeoutSec when shots keep arriving (a hopper that never
    // empties must not hold the auto forever).
  }

  @Test
  void jamReversesTheFloorRollerAndPausesTheEmptyClock() {
    // TODO: setSimulatedJam(true) while feeding. Assert the indexer enters UNJAMMING, the floor
    // roller is EJECT for the same loops, unproductiveFeedSec does not advance, and after
    // kMaxUnjamAttempts the indexer latches JAMMED with its alert raised.
  }

  @Test
  void theBurstLatchBelongsToOneBurst() {
    // Added by M10b step 10's mutation pass. Over two bursts: the latch is clear on every loop
    // before the gate first opens, and the gate opens on a wheel inside the TIGHT band (the first
    // piece of a burst never gets the wide one); it latches once open; and it clears when the burst
    // ends. End each burst in SPIN_UP, not DEFAULT: at rest the turret stows and its term clears
    // the latch anyway, so only the pre-spin proves the latch's own state check works.
  }

  @Test
  void everyShotRequestRunsTheFloorRollerEvenWhenDiverted() {
    // Added by M10b step 10's mutation pass. SCORE_ON_MOVE, SCORE_STATIONARY, SCORE_FORCED,
    // AUTO_SCORE_ON_MOVE and PASS each run the floor roller, on the shipped all-zero tables so the
    // aimed requests are diverted to WAITING_FOR_TURRET -- "shot requested" is the WANTED state.
    // Assert at least one really was diverted, and that rest idles the roller.
  }

  @Test
  void theHopperCheckpointHoldsAcrossOneMatch() {
    // The section 10.2 M10b simulation checkpoint as one continuous match: boot stowed; a shot that
    // needs the arc is refused the turret while stowed, with the floor roller leading; the first
    // enable deploys and the turret gets its arc no earlier than the pivot leaves the band;
    // intake, score and drive together; release and the intake stays down and rocks.
  }

  @Test
  void theMovingShotLandsWhileStrafingAtTwoMetersPerSecond() {
    // The section 10.2 M12 checkpoint in simulation (added by M12). Orbit the goal at 3 m and
    // 2 m/s facing it -- a strafe that stays perpendicular to the line of sight -- and SCORE_ON_MOVE.
    // Follow every released piece with a flight model built from the turret's MEASURED angle and
    // the flywheel's MEASURED speed: each lands inside the gate's own error budget, and the mean
    // miss is a small fraction of the uncompensated drift. Also: the virtual goal sits upstream of
    // the motion by v * tof, and the solver stays under its iteration cap.
  }

  @Test
  void homingTimesOutToFaulted() {
    // TODO: setWantedState(HOME) and never report homed. Step past
    // Constants.Intake.kHomingTimeoutSec and assert SystemState.FAULTED. This proves the state
    // timeout mechanism works at all - if it does not, every other timeout in the codebase is
    // decorative.
  }

  @Test
  void onlyHomeClearsFaulted() {
    // TODO: Drive the FSM to FAULTED, then request INTAKE, SPIN_UP and SCORE_ON_MOVE in turn and
    // assert it stays FAULTED. Then request HOME and assert it leaves.
  }

  @Test
  void intakeAndTurretInterlockHoldsBothWays() {
    // TODO: Rewritten for the pivot band (section 2.4). (a) With the intake STOWED (inside the
    // band - the real conflict on this robot), command a turret goal inside the conflict arc;
    // assert the turret's commanded position stays outside the arc (use passesThroughArcWrapped -
    // the travel is wider than a full turn). (b) Park the turret inside the arc, then request STOW
    // from deployed; assert the pivot holds short of the band. (c) The new negative case: with the
    // intake DEPLOYED or feed-assisting (outside the band), the turret MAY enter the arc. Both directions matter; teams routinely implement only one, and (c) is
    // what stops the interlock being "fixed" back into a blanket stow.
  }

  @Test
  void entryAndExitActionsFireExactlyOnce() {
    // TODO: Instrument via the logged "Superstructure/StateTime" value: assert it resets to ~0 on
    // the loop a transition occurs and monotonically increases otherwise, over a scripted sequence
    // of five transitions.
  }

  @Test
  void forceShootBypassesGatesButStillAims() {
    // TODO: setIgnoreShotTolerances(true) with a piece present; assert isReadyToScore() is true
    // even with a stale pose, AND that the turret still received a setGoal call (a forced shot is
    // an ungated shot, not an unaimed one).
  }

  @Test
  void disablingClearsWantedStateSafely() {
    // TODO: From SCORING_ON_THE_MOVE, call setWantedState(IDLE); step(3); assert the indexer's
    // wanted state is IDLE and never FEED. This is the "auto leaked a goal into teleop" bug.
  }
}
```

---

### 4.15 Non-Java assets, build files & deletions

#### `src/main/deploy/pathplanner/settings.json` [MODIFY]

**Corrected by M13:** this file is NOT the robot's config. `Drive.PP_CONFIG`, built from
`TunerConstants`, is; `AutoFactory` reads this file only to compare it, and a warning names each
value that differs. The GUI still previews paths with it, which is why it should match. (The plan
had `RobotConfig.fromGUISettings()` read this file at runtime for `Drive`'s
`SwerveSetpointGenerator` too. The file shipped in a format PathplannerLib 2026 cannot parse.) Every number in it that is wrong makes path following wrong in a way that
looks like a tuning problem. Edit it through the PathPlanner GUI (Settings tab), never by hand, then
commit the result. The fields that must change from the template's defaults:

| GUI field | JSON key | What to put there |
| --- | --- | --- |
| Robot Mass | `robotMass` | Weigh the robot **with battery and bumpers**, in kg. TODO: MEASURE. |
| Robot MOI | `robotMOI` | kg·m² about the vertical axis. Take it from CAD, or spin the robot at a known ω and fit. TODO: MEASURE. |
| Robot Width / Length | `robotWidth`, `robotLength` | Bumper-to-bumper, meters. Used for collision display and pathfinding inflation. |
| Wheel Radius | `driveWheelRadius` | Must equal `TunerConstants.kWheelRadius` after the wheel-radius characterization routine. |
| Drive Gearing | `driveGearing` | Must equal `TunerConstants` drive gear ratio for the Mk5n ratio you ordered. |
| Max Drive Speed | `maxDriveSpeed` | Free speed from the same characterization, m/s. |
| Wheel COF | `wheelCOF` | Start at 1.2 for a standard tread and lower it if the setpoint generator lets you slip. |
| Drive Motor | `driveMotorType` | `krakenX60` (the Mk5n drive motor in this manifest). |
| Drive Current Limit | `driveCurrentLimit` | Match `TunerConstants`' stator limit; PathPlanner uses it to cap acceleration. |
| Module X/Y offsets | `moduleConfig` entries | Must equal `TunerConstants` `kFrontLeftXPos` etc. |

Also set **Holonomic Rotation** true and **Default Constraints** conservatively (roughly 70 % of
your measured max), because PathPlanner's defaults are what a newly drawn path inherits.

#### `src/main/deploy/pathplanner/navgrid.json` [MODIFY]

Regenerate from inside the PathPlanner GUI once the 2026 REBUILT field is loaded — it is the
obstacle grid AD\* pathfinding searches, so `DriveToPoseCommand`'s pathfind leg and
`AutoRoutines.dynamicScavenge()` both depend on it. **TODO:** do not hand-edit; open the project in
PathPlanner, select the REBUILT field image, and use the navgrid editor to paint the permanent field
elements as obstacles. A stale navgrid from the template's example field will route the robot
through a real wall.

#### `src/main/deploy/pathplanner/paths/*.path` [NEW] and `autos/*.auto` [NEW]

Drawn in the GUI, not written by hand. §8.4 lists the fourteen paths and eight autos by name and
says what each one must connect; the names there are the exact strings `AutoRoutines` passes to
`AutoFactory.followPath(...)`, so a renamed path is a runtime failure, not a compile error. Keep the
example `Example Path.path` / `Example Auto.auto` until `driveForward()` works, then delete them.

#### AdvantageScope 3D robot asset — `config.json` [NEW]

Lives in AdvantageScope's user-assets folder (Help → *Show Assets Folder*), **not** in this repo's
`src/`, in a directory named `Robot_Rebuilt2026/` alongside `model.glb` (the chassis) and one
`model_N.glb` per articulating component. Committing a copy of the folder to
`docs/advantagescope-assets/` in this repo is strongly recommended so a new laptop can be set up in
one minute. Two components matter for this robot: the intake pivot and the turret. With this in
place, AdvantageScope's 3D field animates the real mechanism positions from
`Logger.recordOutput("ComponentPoses", Pose3d[])`, which turns "the intake looks like it is going
through the turret" from a guess into a picture.

```json
{
  "name": "Rebuilt2026",
  "sourceUrl": "",
  "disableSimplification": false,
  "rotations": [{ "axis": "z", "degrees": 0 }],
  "position": [0, 0, 0],
  "cameras": [
    {
      "name": "Turret Limelight",
      "rotations": [{ "axis": "y", "degrees": 0 }],
      "position": [0, 0, 0],
      "resolution": [1280, 800],
      "fov": 82
    },
    {
      "name": "Intake Limelight",
      "rotations": [{ "axis": "y", "degrees": 0 }],
      "position": [0, 0, 0],
      "resolution": [1280, 800],
      "fov": 82
    }
  ],
  "components": [
    { "zeroedRotations": [{ "axis": "y", "degrees": 0 }], "zeroedPosition": [0, 0, 0] },
    { "zeroedRotations": [{ "axis": "z", "degrees": 0 }], "zeroedPosition": [0, 0, 0] }
  ]
}
```

**TODO for the student:** every `position` and `degrees` above is a placeholder. `components[0]` is
the intake pivot and `components[1]` is the turret; `zeroedPosition` is the component's pivot/rotation
axis expressed in robot-origin meters (X forward, Y left, Z up) and `zeroedRotations` orients the
exported mesh so that a zero mechanism angle looks right. Measure both from CAD, export each
component as its own `.glb` with its origin at the robot origin, and verify by commanding the
mechanism to a known angle and watching the 3D view. The two `cameras` entries must match
`Constants.Vision.kTurretToCamera` and `kRobotToIntakeCamera`; the turret camera's entry is only
approximate in this file because the real transform rotates with the turret — AdvantageScope's
component animation will carry it if you attach it to `components[1]`.

The matching log line, added to `Robot.robotPeriodic()`:

```java
Logger.recordOutput(
    "ComponentPoses",
    new Pose3d[] {
      new Pose3d(
          Constants.Intake.kPivotAxisTranslation,
          new Rotation3d(0.0, intake.getPivotAngleRad(), 0.0)),
      new Pose3d(
          Constants.Turret.kAxisTranslation,
          new Rotation3d(0.0, 0.0, turret.getPositionRad()))
    });
```

**TODO:** add `kPivotAxisTranslation` (`Translation3d`) to `Constants.Intake` and `kAxisTranslation`
to `Constants.Turret`; both are plain `static final`, measured from CAD, not tunables.

#### `build.gradle` [MODIFY]

Only one change is required and one is optional.

* **Required (none for tests).** JUnit 5 already arrives via the `edu.wpi.first.GradleRIO` plugin's
  test configuration, so §4.14 compiles as-is. If `./gradlew test` reports "no tests found", add
  `test { useJUnitPlatform() }` — that is the usual missing line.
* **Optional (maple-sim).** See §9.3 for the exact vendordep install step. If you adopt it, no
  `build.gradle` edit is needed at all — `vendordeps/maple-sim.json` is sufficient, because GradleRIO
  resolves vendordeps from that directory.

Do **not** add a `spotless` block: the template already applies `googleJavaFormat` on `compileJava`
at 2-space indent / 100 columns, and a second configuration will fight it.

#### `vendordeps/maple-sim.json` [NEW, OPTIONAL]

Not installed today. §9.3 gives the one-command install. Everything in this plan works without it;
`ModuleIOSim`, `IntakeIOSim`, `TurretIOSim`, `ShooterIOSim` and `VisionIOSim` are all written against
plain WPILib `*Sim` classes so maple-sim is strictly an upgrade, never a dependency.

#### `src/main/java/frc/robot/generated/TunerConstants.java` [MODIFY]

**Regenerate; do not hand-edit.** Open CTRE Tuner X → *Swerve Project Generator*, select Mk5n
geometry, Kraken X60 drive / Kraken X44 steer, your CANcoder IDs and the Pigeon2 ID, run the
generator's calibration wizard, and let it overwrite this file. Everything else in this plan
(`Drive`, `Module`, `ModuleIOTalonFX`, `AutoFactory.PP_CONFIG`, `settings.json`) reads its module
translations, gear ratios, wheel radius and gains from here, so this file is the drivetrain's single
source of truth. Three post-generation edits are allowed and expected:

1. Set the CAN bus name to match `Constants.Can.kDriveBus`.
2. Replace the generated encoder offsets after the wheel-alignment procedure, each time you rebuild
   a module.
3. If you run the comp/practice identity switch (§5.4), wrap the offsets in a
   `Constants.getRobot()` ternary — this is the only place where per-robot values legitimately live
   outside `Constants.java`, because Tuner X owns the surrounding file.

#### Deletions

| File | Why |
| --- | --- |
| `vendordeps/Studica.json` | NavX is not on this robot; the Pigeon2 is the gyro. Removing it also removes a vendordep that must be version-bumped every WPILib update. |
| `subsystems/drive/GyroIONavX.java` | Same reason; it will not compile once Studica.json is gone. |
| `subsystems/drive/ModuleIOTalonFXS.java` | TalonFXS is not on this robot (Kraken X60/X44 are TalonFX). Deleting it removes a whole IO path nobody will test. |

Delete all three in the *same* commit as the `RobotContainer` switch update, or the build breaks in
between.

---

## 5. Constants Strategy

### 5.1 The shape of `Constants.java`

One file, one public final class, a tree of nested `public static final class`es — one per
mechanism, plus `Can`, `Dio`, `Field`, `Shot`, `Vision`, `Power`, and (added by §4.12/§4.13)
`Auto` and `Controls`. The full tree is written out in §4.1; this section explains the rules that
govern it.

```
Constants
├── Mode / RobotType / currentMode / kTuningMode / disableHal / getRobot() / getLoopPeriodSec()
├── Can          plain static final ints + two bus-name strings
├── Dio          plain static final ints
├── Field        tunables, every one carrying a MEASURE TODO
├── Drivetrain   mass/MOI/COF (plain) + driver feel and auto-align gains (tunable)
├── Intake       gear ratio + inversion + sim MOI (plain); angles, gains, RPM, homing (tunable)
├── FloorRoller  gear ratio + inversion (plain); RPM and gains (tunable)
├── Indexer      gear ratio + inversion (plain); RPM, gains, load and jam thresholds (tunable)
├── Turret       gear ratio + inversion + sim MOI (plain); limits, gains, tolerances, arcs (tunable)
├── Shooter      gear ratio + inversions + sim MOI (plain); gains, idle/max RPM, tolerances, shot detection (tunable)
├── Shot         the three shot-table arrays + solver settings + every readyToShoot gate
├── Vision       camera names + four Transform3ds (plain); covariances and rejection gates (tunable)
├── Power        brownout thresholds and the current-budget floor
├── Superstructure [ADD - §5.3] homing timeout and the likelyEmpty() confirmation time
├── Auto         [ADD - §5.3] path-following timeouts and pathfind constraints
└── Controls     [ADD - §5.3] deadbands, trigger thresholds, rumble levels
```

**The three rules, restated so they can be quoted in a code review:**

1. **No numeric literal outside this file.** The only exemptions are `TunerConstants.java` (CTRE
   generates it), unit tests (which deliberately hard-code their own fixtures), and the small
   structural integers that are part of an API call rather than a robot property — array lengths,
   `new Pose3d[4]`, `0.0` initializers. If you find yourself typing `* 0.02`, you want
   `Constants.getLoopPeriodSec()`.
2. **Tunable iff you might change it at an event.** Angles, RPMs, gains, tolerances, thresholds,
   timeouts → `LoggedTunableNumber`. Gear ratios, CAN IDs, inversions, sim moments of inertia,
   `Transform3d`s, roboRIO serials → plain `static final`. The distinguishing question is not "might
   this ever change" but "might this change between two matches on the same day."
3. **Field/game-piece values carry a `TODO` naming the measurement.** Not "TODO: fill in" — "TODO:
   MEASURE from the FIRST Field Layout and Marking Diagram, page N" or "TODO: MEASURE with calipers."
   A TODO a student cannot act on is noise.

### 5.2 The tunable mechanism

`LoggedTunableNumber` (§4.2) is the primitive. Three properties earn its complexity:

* **It is logged, so it replays.** The value is published through AdvantageKit's
  `LoggedNetworkNumber` under `/Tuning/<key>`, which means the number you typed at 3 p.m. in the
  pits is *in the WPILOG*. Replaying that match reproduces the behavior exactly. A plain
  `SmartDashboard.getNumber` does not, and that difference is the whole reason for the class.
* **It costs nothing in competition.** With `Constants.kTuningMode = false`, `get()` returns the
  compiled default and no NT entry is ever created. Flip one boolean before an event and the
  NetworkTables traffic disappears.
* **`hasChanged(int id)` gates the expensive reaction.** Phoenix `Slot0Configs` must not be
  re-applied every loop — it is a CAN transaction. The idiom, used identically in every subsystem:

  ```java
  // In Turret.periodic(), after processInputs:
  LoggedTunableNumber.ifChanged(
      hashCode(),
      () -> io.setGains(gains.toSlot0Configs()),
      Constants.Turret.kKp, Constants.Turret.kKd, Constants.Turret.kKs,
      Constants.Turret.kKv, Constants.Turret.kKa);
  ```

  `hashCode()` as the id is deliberate: two different subsystems watching the same tunable get
  independent change flags, so one consuming a change does not hide it from the other.

`LoggedTunableGains` (§4.2) exists purely to collapse that five-to-seven-argument call into
`gains.ifChanged(hashCode(), io::setGains)` for the five closed-loop mechanisms. Use it for anything
that maps onto a `Slot0Configs`; use bare `LoggedTunableNumber`s for everything else.

**How a student actually tunes.** Open AdvantageScope, connect to the robot (NT4), open the
*Tuning* tab, expand `/Tuning`, type a new value, watch the mechanism respond within one loop. No
deploy, no disable, no rebuild. This is the mechanism that makes the ~180 tunables in §4.1 an asset
rather than a liability — but it is also why rule 2 exists: 180 tunables is already at the edge of
what a person can navigate, and 400 would be unusable.

### 5.3 Three nested classes to add

`AutoFactory`, `AutoRoutines`, `AutoSelector`, `DriveToPoseCommand`, `RobotContainer` and
`Superstructure` reference these; paste them into the `Constants` tree between `Drivetrain` and
`Intake`.

```java
  public static final class Superstructure {
    private Superstructure() {}

    /** Longer than either mechanism's own homing timeout, so the mechanism faults first and says which. */
    public static final LoggedTunableNumber kHomingTimeoutSec =
        new LoggedTunableNumber("Superstructure/HomingTimeout", 5.0);

    /**
     * Gate-open, unloaded feeding with no shot detected before likelyEmpty() turns true (section
     * 2.4). Must be longer than the longest gap between pieces in a real burst, or an auto leaves
     * pieces aboard. TODO: read the gap distribution off Shooter/ShotDetected in a logged burst.
     */
    public static final LoggedTunableNumber kEmptyConfirmSec =
        new LoggedTunableNumber("Superstructure/EmptyConfirm", 0.75);
  }

  public static final class Auto {
    private Auto() {}

    /**
     * Upper bound on one scoring leg. With a hopper this is a max-burst bound, not a one-piece
     * timeout: long enough to empty a FULL hopper (50 pieces) at kFeedRpm, since likelyEmpty() is
     * what ends the leg early. TODO: time emptying 50 pieces and add margin; the placeholder
     * assumes roughly 8 pieces per second.
     */
    public static final LoggedTunableNumber kScoreTimeoutSec =
        new LoggedTunableNumber("Auto/ScoreTimeout", 8.0);
    public static final LoggedTunableNumber kIntakeSettleSec =
        new LoggedTunableNumber("Auto/IntakeSettle", 0.25);

    /** Bound on every routine's opening wait for the intake to leave stow (section 2.4). */
    public static final LoggedTunableNumber kIntakeRetainTimeoutSec =
        new LoggedTunableNumber("Auto/IntakeRetainTimeout", 1.0);
    public static final LoggedTunableNumber kStartPoseWarnMeters =
        new LoggedTunableNumber("Auto/StartPoseWarn", 0.15);
    public static final LoggedTunableNumber kPathfindMaxVel =
        new LoggedTunableNumber("Auto/PathfindMaxVel", 3.0);
    public static final LoggedTunableNumber kPathfindMaxAccel =
        new LoggedTunableNumber("Auto/PathfindMaxAccel", 3.0);
    public static final LoggedTunableNumber kPathfindMaxAngVel =
        new LoggedTunableNumber("Auto/PathfindMaxAngVel", 6.0);
    public static final LoggedTunableNumber kPathfindMaxAngAccel =
        new LoggedTunableNumber("Auto/PathfindMaxAngAccel", 12.0);
    public static final int kMaxScavengeCycles = 3;
  }

  public static final class Controls {
    private Controls() {}

    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final LoggedTunableNumber kTriggerThreshold =
        new LoggedTunableNumber("Controls/TriggerThreshold", 0.25);
    public static final LoggedTunableNumber kTurretNudgeRadPerSec =
        new LoggedTunableNumber("Controls/TurretNudgeRate", 0.5);
    public static final LoggedTunableNumber kRpmTrimStep =
        new LoggedTunableNumber("Controls/RpmTrimStep", 50.0);
    public static final LoggedTunableNumber kReadyRumbleStrength =
        new LoggedTunableNumber("Controls/ReadyRumble", 0.4);
  }
```

`Drivetrain` also needs one addition used by `DriveToPoseCommand`:

```java
    public static final LoggedTunableNumber kPathfindHandoffMeters =
        new LoggedTunableNumber("Drive/PathfindHandoff", 1.5);
    public static final LoggedTunableNumber kAutoAlignThetaToleranceRad =
        new LoggedTunableNumber("Drive/AutoAlignThetaTolerance", Math.toRadians(2.0));
    public static final LoggedTunableNumber kAutoAlignSpeedToleranceMps =
        new LoggedTunableNumber("Drive/AutoAlignSpeedTolerance", 0.1);
```

### 5.4 Naming and unit conventions

**Naming.**

| Pattern | Meaning | Example |
| --- | --- | --- |
| `kXxx` | any constant, tunable or not | `kStowedAngleRad` |
| `<Class>/<Name>` NT key | the tunable's dashboard path; class first, always | `"Turret/Kp"` |
| suffix names the unit | mandatory on every scalar | `kFeedRpm`, `kHomingTimeoutSec` |
| `kMax…` / `kMin…` | a hard bound the code enforces | `kMaxAngleRad` |
| `k…Tolerance…` | an at-goal comparison window | `kToleranceRad` |
| `k…Threshold…` | a detection trip point | `kHasPieceCurrentAmps` |
| `k…Timeout…` | an FSM state timeout, always seconds | `kSpinUpTimeoutSec` |

The NT key must repeat the owning class (`"Turret/Kp"`, not `"Kp"`) because AdvantageScope's tuning
tree is flat within `/Tuning` and three `Kp`s side by side are unusable.

**Units — the whole codebase, no exceptions.**

| Quantity | Unit | Notes |
| --- | --- | --- |
| Length | **meters** | Including all `Transform3d`, `Translation3d`, tolerances. |
| Angle | **radians** | Written as `Math.toRadians(deg)` in `Constants` when a degree value is more legible, converted *at the constant*, never at the use site. |
| Time | **seconds** | FPGA timestamps included. Never milliseconds. |
| Linear velocity | m/s | |
| Angular velocity | rad/s | The one exception below. |
| **Flywheel speed** | **RPM** | Deliberate exception: shot tables, dashboards and the whole team talk in RPM. The conversion to rot/s happens *inside* `ShooterIOTalonFX` and nowhere else. Every flywheel variable ends in `Rpm` so the exception is visible at the use site. |
| Current | amps (stator unless the name says supply) | |
| Voltage | volts | |
| Covariance / std dev | meters and radians (1σ) | Matches `SwerveDrivePoseEstimator`'s contract. |

**Frames.** Two, and only two, with a strict rule about which appears where:

* **Field frame** — WPILib blue-origin, X toward the red alliance wall, Y left, CCW-positive. *All*
  poses, the goal translation, the shot solver's inputs and its `turretFieldAngle` output live here.
  Red alliance is handled only by `AllianceFlipUtil`.
* **Robot frame** — X forward, Y left, origin at the chassis center on the floor. Module
  translations, all four camera `Transform3d`s, and the turret's `positionRad` live here.

The turret's *goal* is a field angle; the turret's *position* is a robot angle; the subtraction
between them happens in exactly one place (`Turret.aiming()`). If you find that subtraction
somewhere else, it is a bug.

### 5.5 Comp vs. practice identity

Two chassis with different encoder offsets, slightly different gearing wear and different gain
sweet spots is the normal state of a team with a practice robot, and the classic failure is
deploying practice offsets to the comp bot on Friday morning. `Constants.getRobot()` resolves the
identity from `RobotController.getSerialNumber()` against `kCompRoboRioSerial` /
`kPracticeRoboRioSerial`, returns `SIMBOT` in sim/replay, and — critically — raises a
`FaultReporter` **error** alert when the serial matches neither, rather than silently defaulting.

The switch is consumed in exactly three ways, in descending order of preference:

1. **A `switch` inside a static initializer, for plain constants.**

   ```java
   public static final LoggedTunableNumber kAbsEncoderOffsetRot =
       new LoggedTunableNumber(
           "Turret/AbsOffset",
           switch (getRobot()) {
             case COMP -> 0.0;      // TODO: MEASURE on the comp bot.
             case PRACTICE -> 0.0;  // TODO: MEASURE on the practice bot.
             case SIMBOT -> 0.0;
           });
   ```

   This is the right shape for anything with a *measured* per-robot value: absolute encoder offsets
   (turret, intake pivot, four CANcoders), and nothing else if you can help it.

2. **A ternary in `TunerConstants.java`** for the four swerve encoder offsets — the only legitimate
   per-robot values outside `Constants.java`, because Tuner X owns that file's structure.

3. **A guard in `RobotContainer`** for hardware that exists on one robot and not the other (a
   practice bot with no Quest headset): `questNavIO = Constants.getRobot() == RobotType.PRACTICE ?
   new QuestNavIO() {} : new QuestNavIOReal();`

**What must NOT be per-robot:** gains. If the comp and practice robots need different kP, something
mechanical differs and you want to know that, not paper over it. Tune on one, verify on the other,
and investigate any gain that will not transfer.

### 5.6 CAN ID and bus assignment

Two buses. **Everything that carries odometry goes on the CANivore** (`"canivore"`): the CANivore
gives the swerve modules and Pigeon2 the bandwidth and timestamp quality that AdvantageKit's
250 Hz `PhoenixOdometryThread` needs, and it keeps a saturated RIO bus from degrading the pose
estimate. **Everything else goes on the RIO bus** (`"rio"`), where a 20 ms update is plenty. The
numbering scheme is deliberate: 1–12 swerve, 13 gyro, 20s mechanisms, so an unknown ID's owner is
obvious from its decade.

| Device | Type | CAN ID | Bus | Constant | Notes |
| --- | --- | --- | --- | --- | --- |
| FL drive | Kraken X60 (TalonFX) | 1 | `canivore` | `TunerConstants` | Generated by Tuner X |
| FL steer | Kraken X44 (TalonFX) | 2 | `canivore` | `TunerConstants` | |
| FR drive | Kraken X60 | 3 | `canivore` | `TunerConstants` | |
| FR steer | Kraken X44 | 4 | `canivore` | `TunerConstants` | |
| BL drive | Kraken X60 | 5 | `canivore` | `TunerConstants` | |
| BL steer | Kraken X44 | 6 | `canivore` | `TunerConstants` | |
| BR drive | Kraken X60 | 7 | `canivore` | `TunerConstants` | |
| BR steer | Kraken X44 | 8 | `canivore` | `TunerConstants` | |
| FL CANcoder | CANcoder | 9 | `canivore` | `TunerConstants` | Offset is per-robot (§5.5) |
| FR CANcoder | CANcoder | 10 | `canivore` | `TunerConstants` | |
| BL CANcoder | CANcoder | 11 | `canivore` | `TunerConstants` | |
| BR CANcoder | CANcoder | 12 | `canivore` | `TunerConstants` | |
| Gyro | Pigeon2 | 13 | `canivore` | `Constants.Can.kPigeonId` | Must share the odometry bus |
| Intake pivot | Kraken X44 | 20 | `rio` | `Constants.Can.kIntakePivotId` | |
| Intake roller | Kraken X60 | 21 | `rio` | `Constants.Can.kIntakeRollerId` | |
| Floor roller | Kraken X60 | 22 | `rio` | `Constants.Can.kFloorRollerId` | |
| Indexer | Kraken X60 | 23 | `rio` | `Constants.Can.kIndexerId` | |
| Turret | Kraken X44 | 24 | `rio` | `Constants.Can.kTurretId` | |
| Flywheel leader | Kraken X60 | 25 | `rio` | `Constants.Can.kFlywheelLeaderId` | |
| Flywheel follower | Kraken X60 | 26 | `rio` | `Constants.Can.kFlywheelFollowerId` | `Follower(25, kFollowerOpposesLeader)` |
| PDH | REV PDH | 1 | `rio` | `Constants.Can.kPdhId` | PDH default is 1; PDP would be 0 |

Non-CAN devices:

| Device | Connection | Port / address | Constant |
| --- | --- | --- | --- |
| Intake pivot ThroughBore | roboRIO DIO (absolute PWM) | DIO 0 | `Constants.Dio.kIntakePivotAbsEncoder` |
| Turret ThroughBore | roboRIO DIO (absolute PWM) | DIO 1 | `Constants.Dio.kTurretAbsEncoder` |
| Turret Limelight 4 | Ethernet / NT | `limelight-turret` | `Constants.Vision.kTurretCameraName` |
| Intake Limelight 4 | Ethernet / NT | `limelight-intake` | `Constants.Vision.kIntakeCameraName` |
| Meta Quest 3S | Ethernet (USB-C adapter) / NT | questnavlib default table | handled inside `QuestNavIOReal` |

**Three rules for whoever wires it.** (1) Both ThroughBore encoders read the *gearbox output*, so
they are true absolute references for the pivot and turret and there is no gear-ratio scaling
between encoder and mechanism — the offsets in `Constants` are raw rotations. (2) Give the two
Limelights and the Quest static IPs on the radio's subnet, and label them; a Limelight that
DHCP-renames itself mid-event costs you vision for a match. (3) Both Limelight names are compiled
into `Constants.Vision`, so the hostname you set in the Limelight web UI must match exactly,
lower-case.

---

## 6. Localization & Shooting on the Move

### 6.1 Data flow, rate by rate, frame by frame

Five producers write into `RobotState`; everything else reads from it. Nothing else in the codebase
owns a pose estimator, and nothing computes a pose from raw sensors on its own.

| Producer | Rate | Thread | Frame it produces | Enters `RobotState` via |
| --- | --- | --- | --- | --- |
| `PhoenixOdometryThread` → `Drive.periodic()` | **250 Hz** sampled, drained ~5 samples per 50 Hz loop | Phoenix thread samples; main thread drains under `Drive.odometryLock` | module positions (robot) + Pigeon2 yaw (field) | `addOdometryObservation(t, gyroYaw, modulePositions)` |
| `Drive.periodic()` chassis speeds | 50 Hz | main | robot-relative `ChassisSpeeds` | `addDriveSpeedsObservation(t, robotRelative)` |
| `Turret.periodic()` | 50 Hz | main | turret angle, robot frame, radians | `addTurretObservation(t, positionRad)` |
| `Vision.periodic()` — 2× Limelight 4 MegaTag2 | ~20–40 Hz each, timestamped at capture | main | field-frame `Pose3d` → `Pose2d` | `addVisionObservation(PoseObservation)` |
| `Vision.periodic()` — QuestNav | ~100 Hz, drained in batches | main | headset `Pose3d` → robot `Pose2d` | `addVisionObservation(PoseObservation)` |

**Why 250 Hz odometry matters here and not in a simpler tier.** A MegaTag2 frame arrives with a
capture timestamp 60–120 ms in the past. To apply it correctly, `SwerveDrivePoseEstimator` must
rewind its odometry history to that instant, insert the measurement, and replay forward. The
accuracy of that rewind is bounded by the odometry sample spacing: at 50 Hz you are interpolating
across 20 ms of motion (at 4 m/s, 8 cm of error); at 250 Hz it is 4 ms (1.6 cm). The same argument
applies with more force to the *turret* buffer — see §7.2.

**Computed where.**

```
Drive.periodic()      -> raw odometry samples          -> RobotState (write)
RobotState            -> fused Pose2d, field speeds, filtered field accel, turret history
Vision.periodic()     -> covariance + rejection policy -> RobotState (write)
Superstructure.periodic()
   1. read pose / speeds / accel from RobotState        (ONE read per loop, shared by all states)
   2. ShootingSolver.solve(...)  -> ShotParameters      (pure function, no I/O)
   3. turret.setGoal(field angle, field angular rate)
      shooter.setGoal(rpm)
   4. isReadyToScore() -> indexer FEED or IDLE (burst latch holds FEED between pieces)
Turret.periodic()     -> field angle - robot rotation -> TurretUnwrapper -> motor position + FF
```

The solver runs **once per loop**, in `Superstructure.periodic()`, before
`handleStateTransition()`. Every state handler reads the same cached `lastShot`. This is not an
optimization — it guarantees that the state machine's *decision* and the turret's *command* are
derived from the same solution, which is what keeps `readyToScore` from latching against a shot that
was never actually commanded.

### 6.2 The virtual-goal formulation

The robot is at **p** with field-relative velocity **v** and acceleration **a**. The goal is the
static field point **g**. A shot launched now arrives after a flight time *t* that depends on how
far the piece must travel. If we simply aimed at **g**, the piece would inherit the robot's velocity
**v** and drift downrange of the goal by **v**·*t*.

The fix is to aim at a *virtual goal* — the point that, once you add back the robot's motion, sends
the piece to the real goal:

```
gv = g − v·t − ½·α·a·t²
```

where α is `Constants.Shot.kAccelCompensationScalar` (default **0**; see §6.3). Because *t* itself
depends on the distance to **gv**, this is implicit, and we solve it by fixed-point iteration:

```
t₀ = tofTable(‖g − p‖)                               // seed with the stationary answer
for k = 1 … kMaxSolverIterations (6):
    gv_k = g − v·t_{k−1} − ½·α·a·t_{k−1}²
    d_k  = ‖gv_k − p‖
    t_k  = tofTable(d_k)
    if |t_k − t_{k−1}| < kSolverToleranceSec (2 ms):  break   // converged
```

**Convergence criteria and why they hold.** `tofTable` is a monotonically increasing, piecewise-linear
function of distance, clamped at both ends. The map `t ↦ tofTable(‖g − v·t − p‖)` is therefore a
contraction whenever `|v| · (dt/dd) < 1` — in words, whenever the robot cannot outrun the rate at
which flight time grows with distance. With a realistic table (roughly 0.14 s of flight per extra
metre) that condition is satisfied for any `|v|` below about 7 m/s, which is above the robot's top
speed. So convergence is guaranteed in practice, and in the tests it takes **2–3 iterations**; the
cap of 6 exists only to bound the worst case. Two guards make a failure visible rather than silent:
`lastIterationCount` and `lastResidualSec` are both logged, and if the iteration cap is ever hit the
solver still returns a result but with `converged = false`, which fails `isUsable()`, which blocks
the shot.

**Converting the converged virtual goal into commands.** With `los = gv − p`, `d = ‖los‖`, and
`tof = tofTable(d)`:

| Quantity | Formula | Frame | Consumer |
| --- | --- | --- | --- |
| Required muzzle velocity, relative to the robot | `vm = (d / tof) · unit(los)` | field | intermediate |
| The piece's ground velocity (for reference) | `vm + v = (g − p) / tof` | field | — |
| Turret aim | `turretFieldAngle = atan2(vm.y, vm.x)`, i.e. at the virtual goal | **field** | `Turret.setGoal` |
| Flywheel speed | `rpm = kRpmPerMeterPerSec · ‖vm‖ + kRpmIntercept` | — | `Shooter.setGoal` |
| Turret rate feedforward | `ω_goal = tangential(−v, ∠los) / d` | **field** | `Turret.setGoal` |
| Effective distance | `d` | — | logging, range gate |

The turret points at the virtual goal, which is *rotated off the line of sight to the real goal* by
exactly the amount needed to cancel the robot's drift. When **v** = 0 the virtual goal is the goal
and the answer collapses to "point at the goal"; that identity is asserted by
`stationarySolveMatchesMovingSolveAtZeroVelocity` in §4.14.

> **Corrected in M12.** This table used to derive a ground velocity `vg = (d / tof) · unit(los)`
> and then set the muzzle velocity to `vm = vg − v`. But `los` already points at the virtual goal,
> which already absorbs the robot's motion, so that subtracted **v** twice. At 3 m and 2 m/s of
> strafe it aimed about 44° off the line of sight instead of about 26°, and every moving shot
> missed by roughly the robot's drift over the flight. M3's tests checked the lead's direction,
> which the double count also satisfied. It was found when M12's simulated checkpoint followed
> each launch to where it lands. `ShootingSolverTest.aMovingShotLandsOnTheGoal` now does the same
> for the solver alone.

**Why the flywheel speed comes from ‖vm‖ and not from distance.** A distance-only shot map is
correct only when standing still. Driving *toward* the goal at 3 m/s means the piece already has
3 m/s of downrange speed and the flywheel must supply less; driving away means it must supply more.
Feeding ‖vm‖ through a linear speed→RPM fit handles both automatically. The cost is that you must
measure a *speed* table, not just an RPM table — §4.13's `measureShotTablePoint` exists to collect
exactly that.

### 6.3 The acceleration term, and why it ships disabled

`kAccelCompensationScalar` defaults to **0**, which makes the `½·a·t²` term vanish. This is
deliberate sequencing, not timidity:

* The velocity term is first-order and always helps. The acceleration term is second-order; at
  `a` = 3 m/s² and `t` = 0.6 s it moves the virtual goal by 0.54 m — large enough to be worth having
  and large enough to ruin every shot if `RobotState`'s differentiated acceleration is noisy.
* `fieldRelativeAccel` is a *numerical derivative* of a velocity that is itself an estimate. Even
  behind a 5-sample moving average it is the noisiest signal in `RobotState`.
* So: get shoot-on-the-move working with α = 0 (which already covers constant-velocity strafing —
  the common case), confirm it, then raise α in 0.25 steps while watching `Shot/VirtualGoal` on the
  2D field. If accuracy degrades, you have learned that your accel estimate is not good enough, and
  the fix is a better filter, not a bigger α.

The `readyToScore` gate independently refuses shots taken while
`|fieldRelativeAccel| > kMaxChassisAccelForShotMetersPerSec2` (default 3 m/s²), so even with α = 0
the robot does not fire during hard acceleration — it fires during the constant-velocity portions,
where the first-order model is exact.

### 6.4 Turret velocity feedforward

The turret must track a *field* angle while sitting on a chassis that is itself rotating. Two
separate rates combine:

```
ω_goal_field  =  tangential(−v, ∠los) / d            // how fast the aim direction sweeps in FIELD frame
ω_turret_robot =  ω_goal_field − ω_chassis           // what the turret motor must actually do
```

`ω_chassis` is `RobotState.getFieldRelativeSpeeds().omegaRadiansPerSecond`. The subtraction is the
whole trick: when the driver spins the robot at 6 rad/s, the turret must counter-rotate at −6 rad/s
*just to stay still in the field frame*, plus whatever the geometry term asks for. A position-only
controller discovers that need only through accumulated error, so it lags by
`ω_chassis / kP_effective` — tens of degrees during a fast rotate. The feedforward supplies it
openly, and the position loop only cleans up the residual.

Mechanically this lands in `Turret.aiming()` (§4.7) as:

```java
double robotRelativeGoal = goalFieldAngle.getRadians() - RobotState.getInstance().getRotation().getRadians();
TurretUnwrapper.Result r = TurretUnwrapper.solve(inputs.positionRad, robotRelativeGoal,
    Constants.Turret.kMinAngleRad.get(), Constants.Turret.kMaxAngleRad.get(),
    Constants.Turret.kUnwrapHysteresisRad.get());
double ffRadPerSec = goalFieldAngularVelocityRadPerSec
    - RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
io.setPositionSetpoint(r.targetPositionRad(), ffRadPerSec);
```

and `TurretIOTalonFX.setPositionSetpoint` converts `ffRadPerSec` into Phoenix's units and passes it
as the `MotionMagicVoltage` feedforward term. **Two failure modes to watch.** (1) If the sign of
`ω_chassis` is inverted, the turret *doubles* its lag instead of cancelling it — the symptom is a
turret that tracks fine standing still and flies off during rotation, and the test is a slow
in-place spin with `Turret/Inputs/positionRad` plotted against `RobotState/EstimatedPose` rotation.
(2) The feedforward is computed from the *fused* chassis rate, not the raw Pigeon2 rate, so a vision
glitch that jerks the fused heading also jerks the feedforward; the pose-jump rejection in §7.4 is
what keeps that from reaching the turret.

### 6.5 The `readyToShoot` gate

`Superstructure.isReadyToScore()` is an AND of seven terms. (It was eight; the piece term was removed
by the hopper replan, §2.4 — the numbering below keeps the gap so logs and discussions from before
the change still line up.) Every one of them is logged separately
under `Superstructure/ReadyTerms/*`, because the question you will actually ask at 11 p.m. on a
practice field is not "is it ready" but "**which** term is false."

| # | Term | Default | What it protects against |
| --- | --- | --- | --- |
| 1 | `lastShot.isUsable()` | — | Solver did not converge, or the goal is outside `[kMinDistanceMeters, kMaxDistanceMeters]`. Firing a non-converged solution aims at whatever the last iteration happened to produce. |
| 2 | `turret.atGoal()` | `kToleranceRad` = 1.5°, `kVelocityToleranceRadPerSec` = 0.5 | Turret still slewing. The *velocity* half matters as much as the position half: a turret passing through the right angle at 3 rad/s is not aimed. |
| 3 | `shooter.willBeAtSpeedIn(2 · loopPeriod, tol)` | `kToleranceRpm` = 75; `kBurstToleranceRpm` = 300 once the burst has latched | Predictive, not instantaneous. The piece takes ~40 ms to leave the indexer, so the question is "will the wheel be at speed *when the piece arrives*", answered by extrapolating the filtered flywheel acceleration, with the projection saturated at the goal (M12: the filtered acceleration lags, so an unsaturated projection overshot the band as a spin-up arrived and dropped the gate mid-burst). This is what removes the one-shot-low that plagues instantaneous `atSpeed()` gates. The wider burst tolerance applies only after the feed has opened in the current scoring state, so the first piece of every burst still meets the tight band, and the indexer does not stop and restart between pieces. |
| ~~4~~ | ~~`indexer.hasPiece()`~~ | — | **Removed (§2.4).** With a hopper and no piece sensor, "has a piece" cannot be answered. Feeding an empty hopper costs nothing, and shot counting no longer depends on the indexer. |
| 5 | `RobotState.getVisionStalenessSec() < kMaxVisionStalenessSec` | 0.6 s | No tag seen recently. Odometry alone drifts; past ~0.6 s the pose is a dead-reckoning guess and the aim angle is a guess with it. |
| 6 | `RobotState.getPoseStdDevEstimate() < kMaxPoseStdDevMeters` | 0.20 m | Pose is fresh but *uncertain* (single distant tag). At 5 m, 20 cm of lateral pose error is ~2.3° of aim error — roughly the whole error budget. |
| 7 | `RobotState.getFieldRelativeAccel().getNorm() < kMaxChassisAccelForShotMetersPerSec2` | 3.0 m/s² | Hard acceleration, where the α = 0 first-order model is wrong (§6.3). |
| 8 | `!FaultReporter.getInstance().anyCritical()` | — | A dead flywheel follower or a disconnected turret encoder. Better to not shoot than to shoot into a partner's robot. |

`SCORING_STATIONARY` adds one more term — chassis speed below a tunable — and `SCORING_FORCED` sets
`ignoreShotTolerances`, which returns `true` unconditionally. The forced path still aims: it skips
the *gates*, not the *solver*. Every forced shot is logged loudly so a post-match review can
separate "the gate was too tight" from "the operator overrode a correct refusal."

**How to tune the gate.** Start every tolerance ~2× looser than you think and tighten while watching
the hit rate. A gate that is too tight presents identically to a broken robot — the trigger does
nothing — which is why term-by-term logging is mandatory rather than nice to have.

---

## 7. Vision & QuestNav Integration

### 7.1 Three sources, one estimator, one policy file

`Vision.java` is the only class that decides what a pose measurement is worth. `RobotState` applies
whatever it is handed (after a final sanity check); the subsystems consume the result. Keeping the
policy in one file is what makes "why did the pose jump in match 42" a ten-minute log question
instead of an archaeology project.

| Source | Hardware | What it is good at | What it is bad at |
| --- | --- | --- | --- |
| Turret Limelight 4 (`limelight-turret`) | rides the turret, so it points where the shooter points | Absolute field truth, and it is *already aimed at the goal* — the tags you most want are the ones in frame when scoring | Its robot→camera transform moves; frames captured mid-slew are motion-blurred and mis-transformed |
| Intake Limelight 4 (`limelight-intake`) | fixed to the frame, facing opposite the turret when the intake is deployed | Tags behind the robot (so you keep a fix when the turret camera is occluded), plus game-piece detection on a second pipeline | Only one job at a time — pipeline switching costs ~100 ms and it cannot do both |
| Meta Quest 3S / QuestNav | inside-out SLAM, ~100 Hz | Centimetre-class *relative* motion with no tags in view at all; survives a defender parked in front of you | No absolute field reference of its own; drifts; loses tracking on impact; runs on a battery |

The fusion rule in one sentence: **MegaTag2 establishes truth, QuestNav carries it between tags.**

### 7.2 The turret camera transform, evaluated at the vision timestamp

This is the single most subtle piece of code in the project and the one most likely to be silently
wrong.

A Limelight computes `botpose` as `fieldToCamera ⊖ robotToCamera`, using whatever `robotToCamera`
value was most recently written to its `camerapose_robotspace_set` entry. For a fixed camera that is
a constant. For a camera bolted to a turret that slews at up to 12 rad/s, it is a *function of
time* — and the value the camera used was the one in effect at **capture** time, typically 60–120 ms
before the frame reaches your code. At 12 rad/s, 100 ms is 69° of turret travel. A pose computed
with a 69°-wrong camera transform is not slightly wrong; it is nonsense.

Three mechanisms, layered, handle it:

1. **Push with a one-loop lead.** `robotToTurretCameraNow()` composes
   `Constants.Vision.kRobotToTurretAxis`, a Z rotation of
   `turretAngle + turretVelocity · getLoopPeriodSec()`, and `Constants.Vision.kTurretToCamera`, and
   pushes it every loop. The lead term cancels the NetworkTables write latency, so the value sitting
   in the camera is approximately the transform at the *next* capture rather than the last one.

2. **Correct at the true capture timestamp.** Push-ahead is approximate; the turret history buffer
   makes it exact. `Vision` records the transform it actually pushed, `T_pushed`, alongside the
   timestamp it was pushed. When an observation arrives stamped `t_c`, `processCamera` recovers the
   camera's field pose and re-derives the robot pose using the transform the turret *actually* had at
   `t_c`:

   ```
   fieldToCamera        = reportedRobotPose ⊕ T_pushed
   correctedRobotPose   = fieldToCamera ⊖ robotToTurretCameraAt(t_c)
   ```

   `robotToTurretCameraAt(t_c)` reads `RobotState.getTurretAngleAt(t_c)` from the 2.0 s
   `TimeInterpolatableBuffer` written every `Turret.periodic()`. If that buffer has no sample for
   `t_c` — the frame is older than the buffer, or the turret has not reported yet — the observation is
   **rejected**, never processed with the current angle. "Use the current angle as a fallback" is the
   bug this whole mechanism exists to prevent.

3. **Reject fast-slew frames outright.** Even with a perfect transform, a frame captured while the
   turret was moving at more than `kMaxTurretRateForVisionRadPerSec` (default 1.5 rad/s) is
   motion-blurred, and blur inflates MegaTag2's corner error in a way no covariance model captures.
   Discard it and wait 20 ms for the next one.

The intake camera needs none of this: its transform is the constant `kRobotToIntakeCamera`, pushed
once at construction.

### 7.3 MegaTag2 ordering, which is not optional

Every loop, for every Limelight, in this order:

```
1. setRobotOrientation(RobotState.getRotation(), RobotState.getFieldRelativeSpeeds().omegaRadiansPerSecond)
2. (turret camera only) setRobotToCamera(robotToTurretCameraNow())
3. NetworkTableInstance.getDefault().flush()          // force the writes out THIS loop
4. read botpose_orb_wpiblue + its timestamp
```

MegaTag2 *consumes* your heading to disambiguate the tag solution; it does not produce one. Reading
the botpose entry before writing the orientation gives you a pose solved against last loop's
heading. Two consequences flow from this and both are already encoded in §4.1:

* **The heading you push must be the fused one** (`RobotState.getRotation()`), not the raw Pigeon2
  yaw. The fused heading is the better estimate, and it is what every other consumer uses.
* **`kMegaTag2ThetaStdDev` is deliberately 9999.** MegaTag2's reported heading is a copy of the
  heading you just gave it. Feeding it back into the estimator as a measurement creates a
  self-reinforcing loop in which the estimator becomes ever more confident in a heading nothing
  independently measured. Trust MegaTag2 for X/Y; trust the Pigeon2 for θ.

### 7.4 Per-source covariance

`SwerveDrivePoseEstimator.addVisionMeasurement(pose, timestamp, stdDevs)` weights each measurement
by the inverse square of the standard deviations you supply. Getting these wrong in either direction
is a real failure: too tight and one bad frame teleports the robot; too loose and vision never
corrects the drift you built it to correct.

| Source | σ_x, σ_y | σ_θ | Reasoning |
| --- | --- | --- | --- |
| Limelight, MegaTag2 | `kMegaTag2XyStdDevBase · d̄² / n` (base 0.04 m) | `kMegaTag2ThetaStdDev` = 9999 | Tag corner pixel error maps to a range error that grows with the **square** of distance; averaging `n` independent tags reduces variance linearly. A 4-tag fix at 2 m gets σ = 0.04 m; a 1-tag fix at 6 m gets σ = 1.44 m and is nearly ignored — which is correct. |
| QuestNav | `kQuestXyStdDev` = 0.02 m | `kQuestThetaStdDev` = 0.035 rad | Tighter than any single-tag MegaTag2 fix, because SLAM's *short-term* relative accuracy genuinely is that good. This is safe **only** because the trust gate in §7.6 is strict: an untrusted Quest contributes nothing at all, rather than contributing weakly. |
| Odometry (inside the estimator) | 0.003, 0.003 m | 0.002 rad | Set once at construction; the template's values, appropriate for 250 Hz Kraken/CANcoder odometry on good tread. |

The numbers are tunable (`Vision/MT2XyStdDev`, `Vision/QuestXyStdDev`, …) so they can be adjusted
against measured behavior rather than argued about. The tuning procedure: park the robot on a known
field point, drive a 3 m square, and plot `RobotState/EstimatedPose` against the tape. If vision
corrections look like steps, loosen; if drift accumulates between tag sightings, tighten.

### 7.5 Outlier rejection — the full ladder

Applied in `Vision.processCamera`, cheapest test first, every rejection counted and logged under
`Vision/<source>/Rejections/<reason>` so the per-match totals tell you which rule is doing the work:

| # | Test | Rejects | Why it exists |
| --- | --- | --- | --- |
| 1 | `tagCount == 0` | frames with no solution | Free. |
| 2 | `tagCount == 1 && ambiguity > kMaxAmbiguity` (0.3) | ambiguous single-tag flips | A single tag viewed near-on-axis has two mathematically valid solutions; one is a mirror through the tag plane, tens of centimetres away. |
| 3 | `abs(pose.getZ()) > kMaxZErrorMeters` (0.4 m) | physically impossible poses | The robot is on the carpet. A solution that puts it 1 m in the air is a bad solve, and Z is a free, independent check on a solution whose X/Y you cannot check. |
| 4 | pose outside field bounds | off-field poses | Uses `kFieldLengthMeters` / `kFieldWidthMeters` with a small margin. |
| 5 | turret camera && `abs(turretVelocity) > kMaxTurretRateForVisionRadPerSec` | motion-blurred frames | §7.2. |
| 6 | `RobotState.getPoseAt(t_c)` empty | un-rewindable frames | The measurement predates the 2.0 s buffer; the estimator cannot place it. |
| 7 | `RobotState.getTurretAngleAt(t_c)` empty (turret camera) | untransformable frames | §7.2. |
| 8 | translation differs from current estimate by > `kMaxAcceptedPoseJumpMeters` (1.0 m) | teleports | Final backstop, applied inside `RobotState.addVisionObservation` so **every** source passes through it, QuestNav included. |
| 9 | any σ non-finite | NaN poisoning | One NaN in `addVisionMeasurement` poisons the estimator permanently; it does not recover, and the robot drives away from the field in AdvantageScope. Cheap insurance. |

**The one exception to rule 8**: a *deliberate* reset — driver presses B, or an auto's
`resetPoseToPathStart` — goes through `RobotState.resetPose`, not `addVisionObservation`, and
bypasses the jump check entirely. That is the intended escape hatch when the estimate is genuinely
lost.

### 7.6 QuestNav: field-frame calibration, seeding, and re-seeding

QuestNav reports the headset's pose in **its own arbitrary frame**, whose origin is wherever the
headset was when tracking started. Two transforms convert that into something the estimator can use:

```
fieldToRobot = fieldToQuest ⊖ kRobotToQuest        // kRobotToQuest is measured from CAD, static
```

and the Quest's frame is aligned to the field by *telling* it where it is:

```
seedQuestNav():
    Pose2d p = RobotState.getEstimatedPose()                  // MegaTag2-derived truth
    questNavIO.setPose(new Pose3d(p).transformBy(kRobotToQuest))
    RobotState.setQuestSeeded(true)
```

**`kRobotToQuest` is the single largest error source in this subsystem.** A 2 cm mistake in the
headset's measured mounting offset is a permanent 2 cm bias on every QuestNav measurement, and
because the Quest's covariance is the tightest of any source, that bias wins the fusion. Measure it
from CAD, then verify empirically: park the robot on a known point, seed, drive a 3 m square, return,
and check that the Quest-derived pose closes the loop. If it does not, the transform is wrong before
anything else is.

**When to seed.**

| Trigger | Condition |
| --- | --- |
| First tag fix after boot | `!questSeeded` and ≥ 2 tags visible |
| After any `RobotState.resetPose` | `resetPose` sets `questSeeded = false`, forcing a re-seed |
| Drift detected | residual > `kQuestReseedResidualMeters` (0.30 m) continuously for `kQuestReseedHoldSec` (0.5 s) **and** the robot is nearly stationary **and** ≥ 2 tags visible |
| Tracking recovered | `trackingLostCounter` increased and has since stabilized |

The stationary + multi-tag condition on the drift trigger is what keeps a re-seed from *writing in*
a bad pose: you only overwrite the Quest's frame when the MegaTag2 estimate you are overwriting it
with is at its most trustworthy. Every seed event is logged with its pose, so "how many times did we
re-seed this match" is a number you can read off — and a rising count across matches means the
headset mount is loose.

### 7.7 Degradation rules

Each source can fail independently and the robot must stay useful. The ladder, from full health
down:

| Condition | Response | What still works |
| --- | --- | --- |
| **All healthy** | Full fusion, tightest covariance | Everything |
| **QuestNav untrusted** (any of: `!connected`, `!tracking`, `batteryPercent < kQuestMinBatteryPercent` (15 %), `trackingLostCounter` increased this loop, `!questSeeded`, sustained residual over threshold) | `RobotState.setQuestTrusted(false)`; Quest observations stop entering the estimator entirely — they are **not** down-weighted, they are dropped. A `warning` Alert fires. | MegaTag2 + odometry. Shooting still works, with a shorter tolerable gap between tag sightings. |
| **One Limelight disconnected** | `error` Alert; the other camera carries on. `getVisionStalenessSec()` naturally rises, and gate 5 of §6.5 starts blocking shots when the remaining camera also loses sight. | Driving, intaking, auto. Shooting works whenever the surviving camera sees tags. |
| **Both Limelights out of tags** (occluded by a defender) | Estimator coasts on odometry + QuestNav if trusted. If the Quest is trusted, accuracy holds for tens of seconds; if not, staleness crosses `kMaxVisionStalenessSec` (0.6 s) and `readyToScore` blocks. | Driving and intaking are unaffected. Shooting is blocked — correctly. The operator's force-shoot override exists for exactly this case when the drive team can see the shot is fine. |
| **Both Limelights disconnected** (hardware/power) | Two `error` Alerts, `anyCritical()` true, `readyToScore` term 8 false. | Driving and intaking. The `preloadOnly` auto (§4.12) is the routine that still works, because it depends on odometry alone. |
| **Quest battery critical** | Warning alert at 15 %. | Plan for it: the Quest should be on a powered USB-C feed from the robot, and the 15 % threshold is a *last* warning, not the primary defense. |

**One rule above all of these:** a degraded source is dropped, never blended in weakly. A source
that is wrong in an unknown way has no correct covariance, so there is no honest weight to give it.
Binary trust, logged at the moment it flips, with the specific failing reason recorded — that is
what makes a match-day vision failure diagnosable from the log instead of from memory.

---

## 8. Autonomous / PathPlanner

### 8.1 `AutoBuilder` configuration

One call, in `AutoFactory.configureAutoBuilder()`, made once from `RobotContainer` after
`RobotState.initialize(...)`. Four of its seven arguments differ from what the stock template does,
and each difference matters:

```java
AutoBuilder.configure(
    () -> RobotState.getInstance().getEstimatedPose(),                       // (1)
    pose -> RobotState.getInstance().resetPose(                              // (2)
                pose, drive.getRawGyroRotation(), drive.getModulePositions()),
    () -> RobotState.getInstance().getRobotRelativeSpeeds(),                 // (3)
    (speeds, feedforwards) -> drive.runVelocity(speeds, feedforwards),       // (4)
    new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0),
                                   new PIDConstants(5.0, 0.0, 0.0)),
    Drive.PP_CONFIG,                                                          // (5)
    AutoFactory::shouldFlipPaths,                                             // (6)
    drive);
```

1. **Pose comes from `RobotState`, not `Drive`.** The drivetrain no longer owns an estimator. This
   single substitution is what makes PathPlanner follow paths against the *vision-fused* pose — so a
   path drawn to a scoring location actually ends at that location, rather than wherever odometry
   thinks it is after 15 seconds of wheel slip.
2. **Reset goes through `RobotState.resetPose`**, which also clears `questSeeded`, forcing the Quest
   to re-seed against the new frame. Skipping that leaves the Quest pulling the estimate back toward
   the pre-reset frame.
3. `getRobotRelativeSpeeds()` — PathPlanner wants robot-relative; the solver wants field-relative.
   `RobotState` publishes both and never converts at a call site.
4. The `DriveFeedforwards` overload, so per-module torque-current feedforwards from PathPlanner's
   trajectory reach `Module.runSetpoint`. Without it, path following relies on the controller alone
   and undershoots every acceleration segment.

Plus two lines the template already has and that must be **kept**:
`Pathfinding.setPathfinder(new LocalADStarAK())` (AdvantageKit's replay-safe wrapper around AD\* —
without it, pathfinding is nondeterministic and your replays diverge) and the two
`PathPlannerLogging` callbacks that publish `Odometry/Trajectory` and
`Odometry/TrajectorySetpoint`.

5. **M13:** `Drive.PP_CONFIG`, not an `AutoFactory` field, because `Drive`'s constructor needs it
   before `AutoFactory` exists. It is built from `TunerConstants`, which is authoritative.
   `settings.json` is only compared against it, and a warning names each value that differs.
6. **M13:** not `AllianceFlipUtil::shouldFlip`. PathPlanner flips paths with its own `FlippingUtil`,
   which ships mirrored on a 16.54 × 8.07 m field; `shouldFlipPaths()` sets it to rotational
   symmetry over `Constants.Field` before answering, so a red path lands where `AllianceFlipUtil`
   puts everything else. Without it, red autos drive on one field and aim at a goal on another.

**`PP_CONFIG` is shared with `Drive`'s `SwerveSetpointGenerator`.** If those two ever hold different
`RobotConfig` objects, the setpoint generator will clip the speeds PathPlanner commands and every
path will run slow in a way that looks exactly like a tuning problem. One static field, assigned
once, read by both.

### 8.2 The `NamedCommands` vocabulary

`NamedCommands` is the bridge between a marker typed into the PathPlanner GUI and a `Command`. The
registry must be populated **before** any `.auto` file is parsed, which is why `AutoFactory` is
constructed before `AutoSelector`. A marker whose name is not registered is silently skipped — no
exception, no warning — so the names below are a contract, and they belong printed on a card taped
to the programming laptop.

| Marker name | Command | Typical use |
| --- | --- | --- |
| `Intake` | `superstructure.setIntakeRequested(true)` | At the start of a pickup path. Independent of any scoring marker, so a path can intake and shoot at once |
| `StopIntake` | `superstructure.setIntakeRequested(false)` | After a pickup. Stops the roller; **the intake stays deployed** so the hopper does not spill (§2.4) |
| `StowIntake` | `setWantedStateCommand(STOW)` | Rarely. Only where the intake must come in; raising it may dump pieces |
| `SpinUp` | `setWantedStateCommand(SPIN_UP)` | ~1 s before a scoring leg; saves the spin-up time |
| `ScoreOnMove` | `superstructure.scoreUntilEmptyCommand()` | On a scoring path, no stop. Ends on `likelyEmpty()` or `kScoreTimeoutSec` |
| `ScoreStationary` | `factory.scoreStationaryCommand()` | Preload shot only |
| `Eject` | `setWantedStateCommand(EJECT)` | Jam recovery |
| `Home` | `setWantedStateCommand(HOME)` | Never in a real routine; useful when testing |
| `StowTurret` | `setWantedStateCommand(DEFAULT)` | Before a path that passes near a wall. **M13:** was `turret.setWantedStateCommand(STOW)`, which the superstructure overwrote on the next loop; resting stows the turret |

**M13 removed `GamePiecePipeline` and `TagPipeline`.** Since M10b the superstructure switches the
intake camera itself, on the edge into and out of intaking, so a marker could only disagree with it,
and `GamePiecePipeline` outside an intake leg would leave the camera off AprilTags. `AutoFactoryTest`
fails on any drawn file that still uses a name not in this table.

**Rule for students:** add a marker name here *and* to this table in the same commit. An unregistered
marker is the most common "my auto works in the GUI but not on the robot" failure, and it produces
no error message at all.

### 8.3 Routines and the `.auto` files

The chooser has ten entries. Eight are real routines; `Do Nothing` is the default and
`Characterization` is a `kTuningMode`-only placeholder that `RobotContainer` replaces.

| # | Chooser name | Method | Start | Pieces | Risk | Depends on |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | Do Nothing | `doNothing()` | any | 0 | none | nothing |
| 1 | Leave | `driveForward()` | any | 0 | none | `AutoBuilder`, `RobotConfig` |
| 2 | Preload Only | `preloadOnly()` | center | 1 | very low | odometry + shot table. **The vision-down fallback.** |
| 3 | Center 3 | `centerThreePiece()` | center | 3 | medium | vision, intake, moving shots |
| 4 | Center 5 | `centerFivePiece()` | center | 5 | high | all of the above, tuned |
| 5 | Amp 3 | `ampSideThreePiece()` | amp side | 3 | medium | same as Center 3 |
| 6 | Source 3 | `sourceSideThreePiece()` | source side | 3 | medium | same as Center 3 |
| 7 | Source Steal | `sourceSideSteal()` | source side | 2–4 | very high | alliance coordination |
| 8 | Scavenge | `dynamicScavenge()` | center | 2–4 | high | intake-camera game-piece pipeline + pathfinding + navgrid |
| 9 | Characterization | placeholder | — | — | — | `kTuningMode` |
| — | Center 3 (stationary) | `centerThreePieceStationary()` | center | 3 | low | `kTuningMode`; §8.6 step 3 (added by M13) |

**M13:** the characterization entries are added by M6, as each routine stops being a skeleton.

**Corrected by M13: no companion `.auto` files are needed.** A routine starts where its first path
does, and the preview reads that. (The plan had each routine 1–8 carry a companion `.auto` file with
the **same name**, whose only job was to declare the starting pose so
`AutoSelector.logStartingPose()` could draw the preview ghost.) Draw it in the GUI
with the correct start pose and a single trivial path; the Java routine is what actually runs. (If
you prefer pure-GUI authoring for the simple ones, `AutoFactory.buildAuto("Leave")` runs the `.auto`
directly — both paths through the code are supported, and mixing them per-routine is fine.)

**Why Java routines instead of pure `.auto` files.** The GUI is excellent at geometry and poor at
control flow: loops like `dynamicScavenge`, the `sequenceOf` builder, and timeouts computed from
tunables at schedule time. (The original argument here was `bailoutIfEmpty` — skip a scoring leg
after a missed pickup. The hopper replan removed it, §2.4: without a piece sensor a missed pickup
cannot be detected, and a moving scoring leg over an empty hopper costs no stop.)

**Hopper robots change what a routine looks like.** The "Pieces" column above counts pickups from
the original single-piece design. With a hopper, the natural unit is a *sweep*:
`driveIntakeAndShoot(path)` drives through a group of pieces with the intake down and fires
whatever is aboard as it goes. `TODO: once the REBUILT field layout and preload rules are known,
redraw the routine list around sweeps rather than piece-by-piece cycles; the building blocks
already support both.`

### 8.4 Paths to draw

Seventeen paths. The first eight are the critical path — draw those in week 8 and the rest once the
first routines run. Names are the exact strings passed to `AutoFactory.followPath(...)`; a rename is
a runtime failure, not a compile error.

| Path file | From → To | Ends with | Used by |
| --- | --- | --- | --- |
| `Leave.path` | start → past the starting line | any | 1 |
| `Center_Preload.path` | center start → first shooting position | stopped | 2 |
| `Center_ToPiece1.path` | shooting position → nearest staged piece | slow, piece in intake | 3, 4 |
| `Center_Score1.path` | piece 1 → shooting position | **moving** | 3, 4 |
| `Center_ToPiece2.path` | shooting position → second piece | slow | 3, 4 |
| `Center_Score2.path` | piece 2 → shooting position | **moving** | 3, 4 |
| `Center_ToPiece3.path` | shooting position → third piece | slow | 4 |
| `Center_Score3.path` | piece 3 → shooting position | **moving** | 4 |
| `Amp_ToPiece1.path`, `Amp_Score1.path`, `Amp_ToPiece2.path`, `Amp_Score2.path` | amp-side mirror | as above | 5 |
| `Source_ToPiece1.path`, `Source_Score1.path`, `Source_ToPiece2.path`, `Source_Score2.path` | source-side mirror | as above | 6 |
| `Source_Steal.path` | source start → opponent staging → return | moving | 7 |

**TODO before any of these can be drawn:** the 2026 REBUILT field must be selected in the PathPlanner
project settings, and `navgrid.json` regenerated (§4.15). Nothing in this plan asserts where a game
piece is staged or where a scoring position is — those come from the field drawings and from driving
the robot and reading `RobotState/EstimatedPose` in AdvantageScope.

**Five drawing rules.**

1. **Draw everything blue.** `AllianceFlipUtil::shouldFlip` is `AutoBuilder`'s flip callback, so red
   is automatic. Drawing a red mirror by hand guarantees the two drift apart.
2. **Pickup paths end slow; scoring paths do not end at all.** Set a low end velocity on `*_ToPiece*`
   so the intake has time to pull the pieces in; leave the end velocity **high** on `*_Score*` — that is
   where shoot-on-the-move happens, and a path that decelerates into its own scoring point throws
   away the advantage the whole turret exists to provide.
3. **Never set a rotation target to "face the goal."** The turret does that. Set the chassis rotation
   for *intaking* and for the next path's entry, and let the turret handle aim. Fighting the turret
   with the chassis is the most common way a first-year turret robot scores worse than a fixed
   shooter.
4. **Keep the turret's travel limits in mind when choosing chassis headings.** A path whose heading
   forces the aim point outside `[kMinAngleRad, kMaxAngleRad]` puts the superstructure into
   `WAITING_FOR_TURRET`, which asks the drivetrain for a heading and fights the path. Check
   `Turret/SystemState` in the replay of every new path.
5. **Set per-path constraints, not global ones.** A pickup path at 2 m/s and a transit path at 4.5 m/s
   is normal; one global maximum makes both wrong.

### 8.5 Shoot-while-moving during autonomous

Nothing special happens. That is the design goal, and it is worth stating explicitly because it is
what the whole architecture buys:

```
AutoRoutines.driveAndShoot("Center_Score1")
  = Commands.deadline(
        factory.followPath("Center_Score1"),                           // PathPlanner drives
        factory.holdWantedState(AUTO_SCORE_ON_MOVE));                  // the FSM shoots
  // M13: held for the leg and handed back as DEFAULT, not set once; a one-shot outlived the path
```

`RobotState` measures the chassis velocity the same way whether it came from a joystick or from
PathPlanner. `ShootingSolver` consumes that velocity. The turret's feedforward subtracts the chassis
yaw rate, which during a path is PathPlanner's commanded rotation. `readyToScore` gates the feed.
There is no auto-specific aiming code anywhere in this codebase, and there must never be — a second
aiming path would be a second thing to tune and a second thing to be wrong.

Three auto-specific details that *are* real:

* **`AUTO_SCORE_ON_MOVE` vs `SCORE_ON_MOVE`.** Same behavior, different wanted state, so the log
  shows unambiguously whether a shot was a driver shot or an auto shot when you score your own
  routines from the WPILOG afterward.
* **`scoreUntilEmptyCommand()` ends on a belief, bounded by a timeout.** It ends when
  `likelyEmpty()` turns true. That happens after `kEmptyConfirmSec` of gate-open feeding with no
  shot, §2.4. `Constants.Auto.kScoreTimeoutSec` (8.0 s placeholder) is the upper bound, sized to
  empty a *full* 50-piece hopper. An auto that waits forever for a gate that will never open loses the
  whole routine, not one shot.
* **An auto can intake and shoot on the same path.** `driveIntakeAndShoot` holds the intake request
  and `AUTO_SCORE_ON_MOVE` together. It is the same orthogonal-region mechanism the driver uses,
  so again there is no auto-specific code.
* **Pre-spin the flywheel.** `spinUpWhile(...)` wraps a pickup leg in `SPIN_UP`, so the flywheel is
  at speed before the scoring leg begins. Worth roughly 0.8 s per cycle, which is the difference
  between a 4-piece and a 5-piece auto.

### 8.6 What to verify, in order

1. `Leave` runs and ends where the path says (proves `AutoBuilder`, `RobotConfig`, pose reset).
2. `Preload Only` scores (proves the shot table at one distance, and the stationary solver).
3. `Center 3` with `ScoreStationary` substituted for `ScoreOnMove` (proves pickup and sequencing
   without the moving shot). M13 made this a chooser entry, `Center 3 (stationary)`, in tuning mode.
4. `Center 3` as written (proves shoot-on-the-move under PathPlanner velocity).
5. `Center 5` (proves it repeats, which is a different claim).
6. Everything else.

Do not skip step 3. If step 4 fails after step 3 passed, the fault is in the moving-shot math; if you
went straight to step 4, it could be anywhere.

---

## 9. Logging, Simulation & AdvantageScope

### 9.1 Log key layout

AdvantageKit writes three families of keys. Knowing which is which is how you read a log quickly:

* **`/RealOutputs/…`** — everything from `Logger.recordOutput(...)` and `@AutoLogOutput`. Computed
  values. On replay these are *recomputed* from the inputs, so if you change the code and replay, the
  outputs change. This is the whole point of replay.
* **`<Subsystem>/…`** (from `Logger.processInputs("<Name>", inputs)`) — raw sensor inputs. On replay
  these are read back verbatim and never recomputed. These are the ground truth of what the robot saw.
* **`NT:/…`, `DriverStation/…`, `SystemStats/…`, `PowerDistribution/…`** — captured automatically.

The convention for this project, all of it already encoded in §4:

| Key | Type | Written by | What you use it for |
| --- | --- | --- | --- |
| `Drive/Inputs/…`, `Module<n>/Inputs/…`, `Gyro/Inputs/…` | inputs | template | Wheel slip, CAN dropouts, gyro health |
| `Intake/Inputs/…`, `Indexer/Inputs/…`, `FloorRoller/Inputs/…`, `Turret/Inputs/…`, `Shooter/Inputs/…` | inputs | each `XxxIO` | Motor currents, temps, positions, connection booleans |
| `Vision/Inputs/…` ×2, `QuestNav/Inputs/…` | inputs | vision IOs | Raw pose observations, tag counts, tracking state |
| `<Name>/WantedState`, `<Name>/SystemState`, `<Name>/StateTime` | string, string, double | every FSM's `periodic()` | **The single most useful plot in a post-match review.** Graph all six as a state timeline and the match reads like a story. |
| `RobotState/EstimatedPose` | `Pose2d` | `@AutoLogOutput` | Drop on AdvantageScope's 2D field |
| `RobotState/FieldRelativeSpeeds`, `/RobotRelativeSpeeds`, `/FieldRelativeAccel` | struct | `@AutoLogOutput` | Solver inputs |
| `RobotState/DistanceToGoalMeters`, `/VisionStalenessSec`, `/PoseConfidence` | double | `@AutoLogOutput` | Gate terms 5 and 6 |
| `RobotState/Sources/<NAME>/{Staleness,StdDev,Accepted,Rejected}` | double/int | `logState()` | Which source was driving the estimate at any instant |
| `Superstructure/Shot/{VirtualGoal,Iterations,Residual,Distance,Rpm}` | mixed | `ShootingSolver.logDebug` | `VirtualGoal` as a `Pose2d` on the 2D field is the fastest solver diagnostic that exists |
| `Superstructure/ReadyToScore` + `Superstructure/ReadyTerms/<1-3,5-8>` | boolean ×8 | `isReadyToScore()` | Why the robot will not shoot, answered in ten seconds (term 4 retired by §2.4; numbering kept) |
| `Superstructure/{IntakeMode,BurstLatched,UnproductiveFeedSec,LikelyEmpty,FeedAssistBlocked,StowedWhileMoving}`, `Intake/RetainingPieces` | enum/boolean/double | `Superstructure.periodic()` | The hopper model (§2.4): why the intake is where it is, and why an auto thought it was empty |
| `Shooter/{ShotDetected,ShotCount}` | boolean/int | `Shooter.detectShot()` | Pieces fired, per burst; the gaps between pulses size `kEmptyConfirmSec` |
| `Indexer/{Loaded,Unjamming}` | boolean | `@AutoLogOutput` | Whether the feed is pushing pieces, and jam recovery |
| `Vision/<source>/Rejections/<reason>` | int counters | `processCamera` | Which rejection rule is actually firing |
| `Vision/QuestTrusted`, `Vision/SeedEvents` | boolean, int | `Vision` | Quest dropouts and re-seed count per match |
| `Faults/<name>`, `Faults/AnyCritical`, `Faults/Count` | boolean/int | `FaultReporter.periodic()` | One red light for the dashboard |
| `Power/{FilteredVoltage,TotalCurrent,Scalar,BrownoutImminent}` | double/boolean | `PowerBudget.periodic()` | Correlating a missed shot with a sag |
| `Drive/TotalCurrentAmps` | double | `@AutoLogOutput` | Is the drivetrain what is eating the battery |
| `LoopTimeMs`, `MemoryFreeMB` | double | `Robot.robotPeriodic()` | Proving a missed shot was (or was not) a timing problem |
| `Odometry/Trajectory`, `Odometry/TrajectorySetpoint` | `Pose2d[]`, `Pose2d` | `PathPlannerLogging` | Path vs. actual, overlaid |
| `ComponentPoses` | `Pose3d[]` | `Robot.robotPeriodic()` | Animates the 3D model (§9.4) |
| `AutoSelector/{Selected,StartingPose,StartPoseError}` | string/`Pose2d`/double | `AutoSelector` | Catching a misplaced robot in the queue line |
| `/Tuning/…` | doubles | `LoggedTunableNumber` | Every tunable value, logged, therefore replayable |

**Log volume and hygiene.** Expect 5–20 MB per match. The template already writes to `/U/logs`;
put a dedicated USB stick in the RIO and **delete logs weekly** — a full stick makes `WPILOGWriter`
fail silently and you lose the logs for the matches you most want. One student owns this.

### 9.2 Simulation fidelity, mechanism by mechanism

Every `XxxIOSim` is written against plain WPILib `edu.wpi.first.wpilibj.simulation` classes, so
simulation works today with zero new dependencies. The fidelity is deliberately uneven — high where
logic errors hide, low where they cannot.

| Mechanism | Sim class | Fidelity | What it genuinely validates |
| --- | --- | --- | --- |
| Swerve modules | `DCMotorSim` ×2 per module (template's `ModuleIOSim`) | Medium | Odometry integration, kinematics, path following geometry. **Not** traction or slip. |
| Intake pivot | `SingleJointedArmSim` | High | Gravity feedforward sign, travel limits, homing-by-current logic, the intake/turret interlock |
| Intake roller, floor roller, indexer | `DCMotorSim` | Low | FSM transitions and command routing only. Indexer load and jams are *injected* (see below), not simulated. |
| Turret | `DCMotorSim` + software soft limits | **High — the important one** | Unwrap branch selection, limit behavior, velocity feedforward sign, and the full aim loop against a simulated pose. If the turret tracks correctly in sim while the sim robot drives a square, the math is right. |
| Flywheel | `FlywheelSim` + `injectShot()` | High | Spin-up time, `willBeAtSpeedIn()` prediction, the `atSpeed` gate's behavior during recovery between shots, burst hysteresis, and shot detection against injected pieces at any cadence |
| Vision (`VisionIOSim`) | ideal camera from ground-truth pose | Medium | The transform composition, the rejection ladder, and covariance scaling — by *injecting* latency, noise and dropouts. An ideal camera validates nothing; a camera you can make lie validates the policy. |
| QuestNav (`QuestNavIOSim`) | perfect tracking + injectable dropout | Medium | Seeding, re-seeding, and every degradation rule in §7.7, on demand, without a headset |

**Game pieces are not simulated** without maple-sim. Instead, two things are *injected*, from a
simulation keyboard binding, a timer or a test. `ShooterIOSim.injectShot()` takes one piece's worth
of energy out of the flywheel. `IndexerIOSim.setSimulatedLoad` / `setSimulatedJam` make the indexer
behave as if the hopper is pressing on it, or jammed. Replaying a burst of N shots at a chosen
cadence exercises shot counting, burst hysteresis and `likelyEmpty()` with no hopper model at all.
This is the honest limit of the WPILib-only sim: it validates logic, not physics. In particular it
cannot tell you the hopper's capacity or whether FEED_ASSIST actually helps.

**The simulation workflow that actually pays for itself:**

1. `./gradlew simulateJava`, open AdvantageScope, connect to `localhost` (NT4).
2. Drive the sim robot with a real controller on port 0.
3. Drop `RobotState/EstimatedPose`, `Odometry/Trajectory` and `Superstructure/Shot/VirtualGoal` on
   the 2D field.
4. Graph the six `SystemState` keys as a timeline.
5. Run every auto routine before the robot exists. **Every auto in §8.3 should run in sim first** —
   sim catches marker-name typos, missing path files, and sequencing deadlocks in seconds, and those
   are the majority of auto failures.

### 9.3 maple-sim — optional, and exactly how to install it

maple-sim adds rigid-body chassis dynamics (real traction, wheel slip, robot-on-robot collisions) and
simulated game pieces that can be intaked and shot. It is **not installed** in this project today.
Everything in this plan works without it. Add it when you want to validate auto routines against
realistic acceleration limits, or to see pieces move.

**Install (one of two ways):**

```bash
# From the project root:
./gradlew vendordep --url=https://shenzhen-robotics-alliance.github.io/maple-sim/vendordep/maple-sim.json
```

or in VS Code: **WPILib icon → Manage Vendor Libraries → Install new libraries (online)** → paste the
same URL. Either writes `vendordeps/maple-sim.json`; no `build.gradle` edit is needed.

> **Verify the URL before trusting it.** Vendordep URLs move between seasons. Check the current one
> on the maple-sim documentation site or its GitHub releases page and confirm the 2026 version is
> compatible with WPILib 2026 before installing. If the URL above 404s, that is why.

**What changes if you adopt it** (all of it additive — nothing in §4 has to be rewritten):

| File | Change |
| --- | --- |
| `ModuleIOSim` | Back it with maple-sim's swerve module simulation instead of two `DCMotorSim`s. Traction limits become real, and auto routines that were optimistic in sim stop being optimistic. |
| `Robot.simulationInit()` | `SimulatedArena.getInstance()` setup; place the drivetrain sim at the selected auto's start pose. |
| `Robot.simulationPeriodic()` | `SimulatedArena.getInstance().simulationPeriodic()` and log `FieldSimulation/GamePieces` as a `Pose3d[]` for the 3D view. |
| `IntakeIOSim` / `IndexerIOSim` / `ShooterIOSim` | Replace injected indexer load and flywheel shots with maple-sim's intake and game-piece simulation — the only way to model hopper capacity. |
| `Constants.Field` | `kGamePieceMassKg` and `kGamePieceDiameterMeters` stop being unused and must be measured. |
| `VisionIOSim` | Can stay as-is. |

**Do not adopt it before the WPILib-only sim works.** Adding a physics engine on top of code you
have not yet validated means every bug has two possible homes.

### 9.4 AdvantageScope 3D asset and component poses

§4.15 gives the `config.json`. The mechanics:

1. Export three GLB meshes from CAD — `model.glb` (chassis, everything that does not move),
   `model_0.glb` (the intake, including the pivot), `model_1.glb` (the turret and shooter assembly).
   **Each must be exported with its origin at the robot origin**, not at its own centroid; the
   `zeroedPosition` in `config.json` then tells AdvantageScope where the rotation axis is.
2. Put them plus `config.json` in a folder named `Robot_Rebuilt2026/` inside AdvantageScope's assets
   folder (**Help → Show Assets Folder**). Commit a copy to `docs/advantagescope-assets/` in this repo
   so a new laptop is a one-minute setup.
3. Publish one `Pose3d[]` per loop under `ComponentPoses`, in the same order as the `components`
   array: index 0 = intake pivot, index 1 = turret. The code is in §4.15.
4. In AdvantageScope's 3D Field tab, select `Rebuilt2026` as the robot model, drop
   `RobotState/EstimatedPose` on the robot and `ComponentPoses` on its components.

**What this buys, concretely.** The intake/turret interlock (§4.11) is an abstract claim about two
angle ranges until you watch the 3D model and see the turret stop at the edge of the intake's arc.
It is also the fastest way to confirm `kRobotToTurretAxis` and `kTurretToCamera`: attach the turret
camera to `components[1]` in `config.json` and the 3D view draws the camera frustum sweeping with the
turret. A frustum that points into the floor means your `Transform3d` is wrong, which you would
otherwise discover as "vision is inaccurate and we don't know why."

**Also drop these on the 2D field**, in this order, and you have a complete aiming diagnostic on one
screen: `RobotState/EstimatedPose` (the robot), `Superstructure/Shot/VirtualGoal` (a ghost showing
where the solver is actually aiming), `Odometry/Trajectory` (the path), `AutoSelector/StartingPose`
(the preview ghost).

### 9.5 Replay, which is the reason for all of it

```bash
# Pull the log off the RIO, then:
./gradlew simulateJava            # with Constants.currentMode == Mode.REPLAY
```

`Robot`'s constructor switches the logger to `new WPILOGReader(logPath)` and a `WPILOGWriter` with a
`_sim` suffix. Every `XxxIO.updateInputs` is bypassed and the recorded inputs are replayed; every
`recordOutput` and every `@AutoLogOutput` is **recomputed**. So you can:

* Change `ShootingSolver`, replay a match, and see whether the shots that missed would have landed.
* Add a `Logger.recordOutput` you wish you had had, replay, and get the data retroactively.
* Reproduce a one-time FSM lockup deterministically, on a laptop, with a debugger attached.

This capability is the reason §1 kept the AdvantageKit `Drive`/`ModuleIO` stack instead of CTRE's
`CommandSwerveDrivetrain`, and the reason for the per-subsystem `@AutoLog` duplication. **One student
owns replay**: they should be able to take any match log and answer "why did that shot miss" within
an hour. If nobody on the team can do that, this tier's main advantage over Tier 2 is unrealized and
you are paying its complexity cost for nothing.

---

## 10. Student Build Order

### 10.1 Roles

Four lanes, assigned on day one and *not* rotated — this codebase is too large for everyone to own
everything, and the cost of a lane owner is one person who can answer questions about that lane
without reading the code first.

| Lane | Owner | Files |
| --- | --- | --- |
| **A — Drivetrain & Localization** | veteran | `Drive`, `ModuleIO*`, `GyroIOPigeon2`, `RobotState`, `DriveCommands`, `DriveToPoseCommand` |
| **B — Mechanisms** | one veteran + one rookie | `Intake*`, `FloorRoller*`, `Indexer*`, `Turret*`, `Shooter*` (all IO layers and FSMs) |
| **C — Vision & Localization sources** | veteran | `Vision`, `VisionIO*`, `QuestNavIO*` |
| **D — Shooting, Superstructure & Autos** | strongest veteran | `ShootingSolver`, `TurretUnwrapper`, `ShotParameters`, `Superstructure`, `autos/*`, all tests |
| **X — Infrastructure (part-time, anyone)** | rotates | `Constants`, `util/*`, `Robot`, `RobotContainer`, logging/replay ownership |

Lane D is the hardest and most isolated: `ShootingSolver` and `TurretUnwrapper` are pure functions
with unit tests, so lane D can be **fully productive in week 1 with no robot and no other lane
finished**. Start them there.

### 10.2 Milestones

Each milestone has a checkpoint you can *demonstrate*. "It compiles" is not a checkpoint.

---

**M0 — Repo hygiene (½ day, lane X, blocks everything).**
Delete `Studica.json`, `GyroIONavX.java`, `ModuleIOTalonFXS.java` in one commit. Restructure
`Constants.java` per §4.1 (the whole tree, with placeholder values). Add `LoggedTunableNumber`,
`LoggedTunableGains`, `GeomUtil`, `AllianceFlipUtil`, `FaultReporter`, `PowerBudget`, and the
`PhoenixUtil` additions.
**Checkpoint:** `./gradlew build` passes; `./gradlew simulateJava` starts; a `LoggedTunableNumber`
you created appears under `/Tuning` in AdvantageScope and changing it changes a logged output.

---

**M1 — Regenerate the drivetrain (1 day, lane A).**
Run CTRE Tuner X's Swerve Project Generator for Mk5n / Kraken X60 drive / Kraken X44 steer / CANcoders
/ Pigeon2 with the CAN IDs from §5.6. Overwrite `TunerConstants.java`. Wire `RobotContainer`'s mode
switch for drive only.
**Checkpoint:** the robot drives field-relative; all four modules point the right way; `Drive/Inputs`
and `Module<n>/Inputs` appear in AdvantageScope; no gyro-disconnect alert.

---

**M2 — `RobotState` owns the pose (2 days, lane A). ← everything downstream depends on this.**
Move the `SwerveDrivePoseEstimator` out of `Drive` into `RobotState`. Add both
`TimeInterpolatableBuffer`s and the field-relative speeds/accel differentiation. Point
`DriveCommands` at `RobotState.getRotation()`.
**Checkpoint:** drive a 3 m square by hand and return to the start; `RobotState/EstimatedPose` closes
the loop within ~10 cm; `RobotState/FieldRelativeSpeeds` matches a stopwatch estimate;
`getPoseAt(Timer.getFPGATimestamp() - 0.1)` returns a pose ~0.1 s behind the current one. **Do not
proceed until the last one is verified** — the whole vision and shooting stack rests on it.

---

**M3 — Two pure classes, fully tested (2 days, lane D — runs in parallel with M1–M2, no hardware).**
Write `ShotParameters`, `ShootingSolver`, `TurretUnwrapper`, and both test files from §4.14.
**Checkpoint:** `./gradlew test` passes all ~20 tests, including
`stationarySolveMatchesMovingSolveAtZeroVelocity` and `unwrapsPastPiWhenTheFarBranchIsCloser`. This
is a laptop-only milestone; it should be **done before the robot exists**.

---

**M4 — One mechanism end to end (3 days, lane B).**
Pick the **flywheel** — it is the simplest FSM, has no limits to crash into, and its sim is
high-fidelity. Write `ShooterIO`, `ShooterIOTalonFX`, `ShooterIOSim`, `Shooter`.
**Checkpoint:** in sim, `Shooter/SystemState` walks `IDLE → SPINNING_UP → AT_SPEED`; on the robot, the
flywheel holds a commanded RPM within `kToleranceRpm`; changing `Shooter/Kp` under `/Tuning` visibly
changes the response with no redeploy. **This milestone is the template for the other four** — once
it is reviewed and accepted, lane B can build the rest in parallel.

---

**M5 — The remaining four mechanisms (1.5 weeks, lane B, parallelizable two students).**
`FloorRoller` and `Indexer` (easy, a rookie's first real contribution), then `Intake` (homing +
`SingleJointedArmSim` + the deploy guard) and `Turret` (unwrap + velocity feedforward + homing).
**Checkpoint per mechanism:** it moves under closed loop in sim *and* on the robot; every FSM state
is reachable and logged; every motor and encoder is registered with `FaultReporter` and unplugging it
raises the alert. **Turret-specific checkpoint:** run `CharacterizationCommands.turretUnwrapDemo` and
confirm from the log that the commanded position never leaves `[kMinAngleRad, kMaxAngleRad]`.

---

**M6 — Characterize everything (2 days, lanes A + B together).**
Run `driveSysIdFull`, `turretSysId`, `intakePivotSysId`, `shooterStaircase`, `indexerFeedforward`,
`floorRollerFeedforward`, plus `DriveCommands.wheelRadiusCharacterization`.
**Checkpoint:** every kS/kV/kA/kG in `Constants` is a *measured* number with the SysId screenshot
committed to `docs/`. The measured wheel radius is in both `TunerConstants` and
`pathplanner/settings.json`, and they agree.

---

**M7 — One Limelight (3 days, lane C).**
`VisionIO`, `VisionIOLimelight`, `VisionIOSim`, and `Vision` with the **intake** camera only (fixed
transform — do the easy one first). Full rejection ladder and covariance model.
**Checkpoint:** place the robot at a surveyed point, cover and uncover the camera, and watch
`RobotState/EstimatedPose` converge to truth within ±5 cm. Every rejection counter in
`Vision/limelight-intake/Rejections/*` is nonzero at least once when you deliberately provoke it.

---

**M8 — The turret camera (2 days, lane C; needs M5's turret and M2's buffer).**
Add the turret camera with `robotToTurretCameraNow()`, `robotToTurretCameraAt(t)`, and the slew-rate
rejection.
**Checkpoint:** with the robot stationary and a tag in view, **slew the turret through 180° and
confirm `RobotState/EstimatedPose` does not move** by more than a couple of centimetres. If the pose
swings with the turret, the transform composition order in `GeomUtil.rotateAboutZ` is backwards.
This is the highest-value single test in the project.

---

**M9 — QuestNav (2 days, lane C).**
`QuestNavIO`, `QuestNavIOReal`, `QuestNavIOSim`, the trust gate, seeding and re-seeding.
**Checkpoint:** seed against a tag, then cover both Limelights and push the robot 3 m. The pose
tracks. Then power-cycle the headset mid-run and confirm `Vision/QuestTrusted` goes false, the
warning alert fires, and the pose degrades to odometry instead of jumping.

---

**M10 — The Superstructure (1 week, lane D; needs M3 + M5).**
`Superstructure` with all fourteen `SystemState`s, both interlocks, the `readyToScore` gate, and
`SuperstructureFsmTest`.
**Checkpoint:** all ten FSM tests pass; on the robot, every driver button produces the expected state
transition with nothing else moving; `Superstructure/ReadyTerms/*` shows eight separately-logged
booleans. Deliberately deploy the intake into the turret's arc and confirm both interlock directions
hold.

---

**M10b — Hopper retrofit (4–5 days, lanes B + D; needs M10). Added by the §2.4 replan.**
Bring the built M5 mechanisms and M10 superstructure from the single-piece model to the hopper
model, as its own feature branch cut from `feature-superstructure`. Nothing here needs hardware.
In outline, so each step compiles and its suite passes before the next. **The authoritative,
step-by-step order is "M10b: the order in which built code changes" in `PROJECT_OVERVIEW.md`**. It
splits these four into eleven steps (0–10), including the vision-gate widening and a separate interlock
commit:
1. `Shooter`: shot detection, `getShotCount`/`resetShotCount`/`getSecondsSinceLastShot`, the
   tolerance overload of `willBeAtSpeedIn`, and `ShooterIOSim.injectShot()`.
2. `Indexer`: the §4.6 rewrite. Delete `hasPiece`, `INDEX`/`HOLD`, the latched-sample replay, the
   shot counter and the beam-break path; add jam handling and `isLoaded()`. The existing
   `IndexerFsmTest` cases are about the deleted behaviour; replace them rather than adapt them.
3. `Intake`: `DEPLOY_ONLY` resting in a real `DEPLOYED` state, `FEED_ASSIST`, and
   `isInTurretConflictBand()` replacing `isStowed()` in the turret guard.
4. `Superstructure`: the orthogonal intake region, `RESTING`/`STOW`, the seven-term gate with the
   burst latch, `likelyEmpty()`, and the §4.14 test changes. `RobotContainer` bindings and
   `AutoFactory` markers follow.

**Checkpoint:** every §4.14 case passes. That includes the three negative cases that pin the
replan's decisions: a deployed intake *may* share the arc with the turret, scoring does *not*
require a piece, and gate-closed time does *not* advance `likelyEmpty()`. Run a mutation pass
aimed at the `likelyEmpty()` accumulator and the burst latch, since those are the two places a
plausible-looking bug passes every "happy path" test. In sim: hold intake, score and drive
together and confirm from the log that `Superstructure/IntakeMode` reads `INTAKING` while the
turret is `AIMING`. Then release intake and confirm the intake stays deployed. From boot, confirm
the intake starts STOWED, deploys on enable, and the turret is refused its arc only until the pivot
leaves the stow band. While scoring, confirm the floor roller runs before the indexer and the
intake rocks. M10's
"eight separately-logged booleans" becomes seven. (M10b step 10 ran this as a JUnit case,
`theHopperCheckpointHoldsAcrossOneMatch`, over the same simulated IO, so it fails on every build
rather than once; the seven-booleans clause is checked by reading the code, because the logger
records nothing under test.)

---

**M11 — Stationary shooting (1 week, lanes B + D).**
Collect the shot tables: run `measureShotTablePoint` at each of the 8–10
`kDistanceBreakpointsMeters`, with a tape measure and a 120 fps phone video for the time of flight.
Fill `kTimeOfFlightSec`, `kRpmPerMeterPerSec` and `kRpmIntercept`; `tools/fit_shot_table.py` turns
one CSV row per burst into all three. (An earlier version of this line also listed
`kLaunchSpeedMetersPerSec`, which the solver never read; M11 deleted it. The measurement shot needs
no table of its own, because it runs inside `SCORE_STATIONARY` with a hand-set speed.)
**Measure with bursts, not single pieces** (§2.4): load the hopper, fire at `kFeedRpm`, and fit to
the whole burst. A table built from single pieces over-predicts every piece after the first, which
leave a recovering wheel. While doing it, log `Shooter/ShotDetected` and tune `kShotDipRpm`,
`kShotRefractorySec`, `kBurstToleranceRpm` and `kEmptyConfirmSec` from the same logs.
**Checkpoint:** 8 of 10 shots score from a standstill at three different distances, with
`SCORE_STATIONARY`, counting every piece of a burst rather than only the first.
`Shooter/ShotCount` matches a hand count of pieces fired to within one per burst. The 8-piece
preload is a known quantity, so firing it is the easiest ground-truth check of the detector, and
every auto repeats it (`Auto/PreloadFired`). **This is the gate for M12** — shoot-on-the-move cannot work if the stationary
model is wrong, and you will not be able to tell the two failures apart.

---

**M12 — Shoot on the move (1–2 weeks, lane D).**
Enable `SCORE_ON_MOVE`. Tune the gates. Keep `kAccelCompensationScalar` at 0 until constant-velocity
shots are reliable, then raise it in 0.25 steps (§6.3).
**Checkpoint:** 7 of 10 shots score while strafing perpendicular to the goal at ≥ 2 m/s. (M12 ran
this in simulation as `theMovingShotLandsWhileStrafingAtTwoMetersPerSecond`, and it found a
double-counted velocity in the solver, corrected in §6.2.)
`Superstructure/Shot/VirtualGoal` visibly leads or lags the real goal in the correct direction on
AdvantageScope's 2D field. `solver.getLastIterationCount()` stays below the cap.

---

**M13 — Autos (1.5 weeks, lane D + lane A).**
`AutoFactory`, `AutoRoutines`, `AutoSelector`. Draw the paths per §8.4. Follow the verification order
in §8.6 exactly.
**Checkpoint:** all eight routines run in sim; every routine starts with `waitUntilIntakeRetaining()`
and none moves before the intake is deployed (check `Superstructure/StowedWhileMoving` is never true);
`Auto/PreloadFired` reads 8 at the end of `Preload Only`; `Leave`, `Preload Only` and `Center 3` run
on the real field; `AutoSelector/StartPoseError` is under 5 cm when the robot is placed correctly and the warning
alert fires when it is not. (M13 ran the simulation clauses as `AutoCheckpointTest`, on synthetic
paths and on both alliances, and the selector's as `AutoSelectorTest`. "Starts with the wait" is
checked as nothing at all happening before the intake retains: movement alone could not see a
missing wait behind a stationary preload.)

---

**M14 — Competition hardening (1 week, everyone).**
`FaultReporter` registrations complete for every device. `PowerBudget` wired into every subsystem's
current limit. Brake-mode-release timer. `kTuningMode = false` and a full re-test with it false.
Practice-bot identity verified (§5.5). A pit checklist that includes "check the Alerts widget."
**Checkpoint:** unplug any single CAN device and the robot tells you which one, by name, on the
dashboard, within one second. Run a full 2:30 match in practice and confirm the battery finishes
above `kBrownoutWarningVolts`. **Combined-mode run (§2.4):** drive a figure-eight while intaking
and firing continuously for 20 s. Confirm from `Power/FilteredVoltage` that no brownout occurs,
that the `LOW` scalar (intake, floor roller) is the first to drop, and that FEED_ASSIST oscillation
is suppressed while it does.

---

### 10.3 The parallel schedule

```
Week:      1     2     3     4     5     6     7     8     9    10    11    12    13
Lane A:  M1 M2 ─────────────────┐                          ┌─ M13 (paths) ─┐
Lane B:        M4 ── M5 ────────┼─ M6 ──┐         ┌─ M11 ──┤               │
Lane C:              M7 ── M8 ──┴─ M9 ──┘         │        │               │
Lane D:  M3 ──────────────── M10 ─────────────────┴─ M12 ──┴─ M13 ─────────┼─ M14
Lane X:  M0 ── (Constants/util maintenance, logging ownership) ────────────┘
```

**The five hard dependencies** — everything else can float:

1. **M2 blocks M7, M8, M10, M12.** `RobotState` is the hub. If M2 slips, everything slips.
2. **M5's turret blocks M8.** No turret angle buffer, no moving-camera transform.
3. **M3 + M5 block M10.** The Superstructure needs both the solver and the mechanisms.
4. **M10b blocks M11.** The shot tables are measured in bursts, which needs the hopper feed path,
   burst latch and shot detector.
5. **M11 blocks M12.** Non-negotiable — see the checkpoint note.

**The deliberate parallelism win:** lane D's M3 needs no hardware at all, so the hardest math in the
project gets written and unit-tested during the two weeks everyone else is waiting for a robot. If
you take one scheduling idea from this section, take that one.

**If you fall behind**, cut in this order: `dynamicScavenge` (M13) → `Source Steal` (M13) →
QuestNav entirely (M9 — MegaTag2 alone is sufficient for shooting) → `DriveToPoseCommand` (M13) →
`kAccelCompensationScalar` tuning (M12). **Never cut:** M2, M11, or the `FaultReporter` registrations
in M14.

---

## 11. Deliberate Omissions & Risks

### 11.1 What this tier does not do

Every item below was considered and left out. Knowing *why* matters, because the second-most common
way a plan like this fails is a student adding one of them in week 6 without understanding what it
displaces.

| Omitted | Why | When you would add it |
| --- | --- | --- |
| **Any climb / endgame mechanism** | Not in the hardware manifest. `Superstructure` has a `CLIMB_SAFE`-shaped hole (§2.3 names it) but no states or subsystem. | The moment a climber exists. Add it as a sixth mechanism FSM plus one `SystemState`; the interlock pattern already handles "nothing else may move." |
| **Variable hood / adjustable launch angle** | The manifest has two fixed flywheels and no hood actuator. The shot model is therefore **1-D**: distance → (speed, time of flight). | If a hood is added, every `InterpolatingDoubleTreeMap` in `ShootingSolver` becomes a 2-D interpolation and `ShotParameters` gains a `hoodAngle`. That is a real rewrite of §6.2, not an addition. |
| **A physics model of the shot** | No drag, no Magnus, no spin. The time-of-flight table and the RPM fit are *measured*, which absorbs all of it empirically and exactly, at your robot's actual conditions. | Never, for a competition robot. A measured table beats a modeled one every time. |
| **Turret backlash compensation** | A dead-band model needs a direction-of-travel state and adds a discontinuity the velocity feedforward fights. | Only if characterization shows backlash larger than `kToleranceRad` (1.5°). Fix it mechanically first. |
| **A piece count (how many are in the hopper)** | The robot carries a hopper (§2.4) but has no piece sensor, so it cannot count what it holds. It counts pieces *fired* at the flywheel and keeps a `likelyEmpty()` belief; nothing is gated on either. | Only if a live count is wanted for the driver. A "hopper full" signal is not needed: the hopper holds 50 and overfilling does not jam. A count would need an entry sensor at the intake mouth, with count = 8 preloaded + in − fired and drift handling. The FSM would not change shape, only gain inputs. |
| **Shot outcome detection** | Nothing on this robot can observe whether a piece went in. `getShotCount()` counts *fires*, not *scores*. | With a goal-side camera pipeline or a field-published score feed. Until then, hit rate is a human count from match video. |
| **Vision on a coprocessor thread** | Both Limelights do their own processing; the RIO only reads NetworkTables. Nothing here needs a thread. | Never, with Limelights. Only if you moved to PhotonVision on an Orange Pi doing RIO-side solving. |
| **Defender modeling / dynamic obstacle avoidance** | `navgrid.json` knows static field elements only. AD\* replans around nothing that moves. | This is a multi-season research project, not a build-season feature. |
| **Robot-to-robot alliance communication** | Field rules and bandwidth make it fragile; `Source Steal` is coordinated by humans in the queue line. | Never, at this level. |
| **Redundant gyro** | One Pigeon2, on the CANivore. A gyro failure is caught by the disconnect alert, and MegaTag2 + QuestNav can carry heading badly for a while. | If you have a spare CANivore port and a spare Pigeon2. It is cheap insurance; it is just not *necessary*. |
| **Choreo** | PathPlanner's 2026 solver is good enough, it is already in the vendordeps, and `AutoBuilder` + `NamedCommands` + the GUI is a smaller thing for students to learn. | If your autos are time-limited by trajectory optimality rather than by cycle logic. They almost certainly are not. |
| **Automatic gain tuning from logs** | Fun; not a competitive advantage at this level. | Never. |
| **`SwerveSetpointGenerator` on the turret or any mechanism** | Motion Magic already profiles them in the Talon. | Never. |
| **Slip detection / odometry rejection from wheel-current mismatch** | The `SwerveSetpointGenerator` prevents most slip up front, and MegaTag2 corrects what remains. | If odometry drift between tag sightings proves unacceptable after M8. Measure before building. |

**The honest summary of the ceiling:** this tier gets you accurate shooting while translating, robust
localization through occlusion, eight autos with real branching, and post-match diagnosability. It
does **not** get you a climber, a variable-angle shot, or any awareness of other robots.

### 11.2 The failure modes that will actually bite you

Ordered by expected cost — probability times damage. Each entry says how it *presents*, because the
presentation is almost never the cause.

---

**1. `GeomUtil.rotateAboutZ` composition order is backwards.**
*Presents as:* "vision is inaccurate." The pose is fine when the turret is at 0° and progressively
worse as it slews. Nobody suspects the turret, because the turret is visibly tracking correctly.
*Detection:* M8's checkpoint — stationary robot, tag in view, slew the turret 180°, watch
`RobotState/EstimatedPose`. If it swings, this is it.
*Mitigation:* the checkpoint is a mandatory milestone gate, and `robotToTurretCameraAt(t)` returns
identity plus a logged warning rather than a guess when the buffer misses.

---

**2. The stationary shot table is wrong and you are debugging the moving-shot math.**
*Presents as:* shoot-on-the-move misses by a consistent amount, and every change to
`kAccelCompensationScalar` or the solver makes it differently wrong.
*Detection:* impossible after the fact; you must order the work correctly.
*Mitigation:* M11 is a hard gate before M12, and
`stationarySolveMatchesMovingSolveAtZeroVelocity` proves the solver is not the variable.

---

**3. Turret velocity feedforward sign inverted.**
*Presents as:* turret tracks perfectly while the chassis translates, and flies off target the moment
the driver rotates — so it looks like "the turret is too slow" and someone raises `kP` until it
oscillates.
*Detection:* slow in-place spin with `Turret/Inputs/positionRad` and
`RobotState/FieldRelativeSpeeds.omegaRadiansPerSecond` on the same plot. Correct behavior: the turret
position counter-rotates 1:1.
*Mitigation:* §6.4 spells out the subtraction; put it in the code review checklist.

---

**4. `AllianceFlipUtil` uses the wrong field symmetry.**
*Presents as:* everything works perfectly on blue and the robot aims at nothing on red — discovered
in your first red-alliance qualification match.
*Detection:* force `DriverStation.getAlliance()` to red in sim and re-run every auto plus a
stationary shot. Do this in M13, not at the event.
*Mitigation:* the flip lives in exactly one file, and `apply(Translation2d)` carries a **TODO
requiring you to confirm the 2026 REBUILT field's symmetry from the drawings** before committing to
either formula. Do not guess this one.

---

**5. `RobotState.getInstance()` called before `initialize()`.**
*Presents as:* a `NullPointerException` in `robotInit` — on the field, a robot that never enables.
It is order-dependent, so it can appear when someone reorders two lines in `RobotContainer`.
*Detection:* it crashes immediately, every time, which is the good news.
*Mitigation:* `RobotContainer` calls `RobotState.initialize(...)` immediately after constructing
`Drive` and before anything else. Consider making `getInstance()` throw a named
`IllegalStateException` rather than return null.

---

**6. A `NamedCommands` marker name typo.**
*Presents as:* an auto that drives the path perfectly and never intakes or shoots. No error, no
warning, nothing in the log except the absence of a state change.
*Detection:* run every auto in sim (§9.2 step 5) before the robot exists.
*Mitigation:* the §8.2 table is the contract; add the name to the table and the registry in the same
commit.

---

**7. Brownout during a spin-up while driving.**
*Presents as:* the robot "glitches" — all six closed loops drop simultaneously for ~100 ms, the pose
estimator sees a discontinuity, and the driver reports the robot "stuttered."
*Detection:* `Power/FilteredVoltage` and `RobotController.isBrownedOut()` in the log, correlated with
`Shooter/SystemState == SPINNING_UP`.
*Mitigation:* `PowerBudget` scales non-critical current limits first, `isBrownoutImminent()` lets the
Superstructure refuse a spin-up it cannot finish, and M14's checkpoint is a full 2:30 practice match
ending above `kBrownoutWarningVolts`. **Fresh batteries are still the primary defense.**

---

**8. `kRobotToQuest` mis-measured.**
*Presents as:* a small, constant position bias that vision "cannot correct," because the Quest's
tight covariance wins the fusion. Shots miss by a consistent offset in one direction.
*Detection:* the closed-loop square test in §7.6, run with QuestNav trusted and then with it forced
untrusted. If the two differ by a constant, this is it.
*Mitigation:* measure from CAD, verify empirically, and remember that `Vision/QuestTrusted` is a
switch you can flip — running a match with QuestNav disabled is a valid diagnostic and a valid
fallback.

---

**9. Turret belt skip / absolute encoder offset drift.**
*Presents as:* aim is off by a fixed angle after a collision, and the turret happily drives into a
hard stop because the software thinks it is elsewhere.
*Detection:* `Turret/Inputs` absolute-encoder reading vs. integrated position; they diverge.
*Mitigation:* the ThroughBore is on the gearbox **output**, so it re-establishes truth every boot;
`Turret` re-homes to it, and `kLimitMarginRad` (3°) keeps the commanded target off the stops. Add
"check turret zero" to the pit checklist.

---

**10. Two disagreeing `RobotConfig`s.**
*Presents as:* paths run slower than drawn and end short, consistently, and it looks exactly like a
PID tuning problem — so someone raises the PathPlanner gains, which makes it oscillate *and* short.
*Detection:* log both `AutoFactory.PP_CONFIG` values and the setpoint generator's limits at boot and
compare.
*Mitigation:* one static field, assigned once (§8.1). Never construct a second `RobotConfig`.

---

**11. A NaN reaches `addVisionMeasurement`.**
*Presents as:* the robot's pose jumps to infinity in AdvantageScope and **never recovers**, even
after the bad frame passes. The estimator's internal state is permanently poisoned; only a
`resetPose` fixes it.
*Detection:* obvious once it happens; impossible to miss.
*Mitigation:* rejection rule 9 in §7.5 checks every σ for finiteness before the call. It is three
lines and it has saved teams matches.

---

**12. `kTuningMode` left `true` at an event.**
*Presents as:* elevated NetworkTables traffic, occasionally a loop overrun, and — far worse — live
SysId bindings on the operator controller, one button press from sweeping the turret into a hard stop
during a match.
*Detection:* check the boolean. Every time.
*Mitigation:* put "set `kTuningMode = false`, rebuild, redeploy, re-test" on the load-in checklist,
and make the final pre-event commit the one that flips it.

---

**13. `likelyEmpty()` ends an auto's scoring leg with pieces still aboard.**
*Presents as:* autos that "leave pieces in the robot" and drive off early, only sometimes. It looks
like an intake problem, because the robot visibly had pieces.
*Detection:* `Superstructure/UnproductiveFeedSec` climbing while `Superstructure/ReadyToScore` is
false, which is the accumulator counting gate-closed time. Or `kEmptyConfirmSec` shorter than the
longest gap between `Shooter/ShotDetected` pulses in a burst.
*Mitigation:* the accumulator only runs with the gate open and the indexer unloaded (§2.4), and
`likelyEmptyOnlyAccumulatesWithTheGateOpen` is a mutation target in M10b. Size `kEmptyConfirmSec`
from logged bursts, not by feel.

---

**14. The turret-conflict pivot band is the wrong width.**
*Presents as:* either a turret that is refused its arc for longer than it should be after the
intake leaves stow (band too wide), or the intake arm and turret meeting as the intake stows or
deploys (band too narrow). The second one is the expensive one, and it is silent until they touch.
*Detection:* `Intake/InTurretConflictBand` against the arm's actual position, filmed through a
slow stow with the turret parked just outside its arc.
*Mitigation:* the band is a width from stow with a realistic 15° placeholder, not two absolute
angles defaulting to zero. The trap of an empty, inert band (M10's identity-transform trap in
another form) cannot occur by default. Measure the exact edge from CAD and add margin rather than
trimming it.

---

**15. The `/U/logs` USB stick fills up.**
*Presents as:* no logs for exactly the matches you most want to analyze. `WPILOGWriter` fails
quietly.
*Mitigation:* one named student owns log offload; delete weekly; check free space in the pit.

---

### 11.3 The meta-risk

**The real risk in this tier is not any one bug — it is that nobody on the team fully understands the
localization stack.** Four lanes, ~45 files, and a hub class that every other class reads from means
there is exactly one question that decides whether this plan succeeds: *can someone on your team, at
11 p.m. on a Friday, open a match log and say why a shot missed?*

If yes, everything in §11.2 is a one-hour fix. If no, the same bugs are unbounded, and you would have
been better served by Tier 2's simpler design, which you could have debugged by inspection. §9.5's
"one student owns replay" is not an organizational nicety; it is the load-bearing assumption of this
entire tier.

---

## 12. Method Count Summary

Counts are the number of method bodies a student must actually write, taken directly from the
skeletons in §4 — constructors, `@Override`s, private FSM handlers and IO-interface defaults all
included. (§1's "~330 methods" was a rough estimate of the *public* API; this table is the real
number, and it is larger. Plan against this one.) `Constants.java` is listed as 4 methods because
its thirteen private no-op constructors are boilerplate, but it also carries ~180 constant
declarations, which is where its real effort lives.

**Difficulty:** 1 = mechanical, copy the pattern · 2 = straightforward with the plan in hand ·
3 = needs real understanding of one API · 4 = needs understanding of several interacting pieces ·
5 = get this wrong and the robot is subtly, expensively broken.

| File | # methods | Difficulty | Suggested owner |
| --- | ---: | :---: | --- |
| **4.1 Core** | | | |
| `frc/robot/Constants.java` | 4 (+~180 constants) | 2 | rookie (typing) + veteran (review) |
| `frc/robot/RobotState.java` | 23 | **5** | veteran (lane A) |
| `frc/robot/Robot.java` | 13 | 3 | veteran (lane X) |
| `frc/robot/RobotContainer.java` | 7 | 3 | veteran (lane X) |
| **4.2 Utilities** | | | |
| `frc/robot/util/LoggedTunableNumber.java` | 8 | 2 | rookie |
| `frc/robot/util/LoggedTunableGains.java` | 6 | 2 | rookie |
| `frc/robot/util/AllianceFlipUtil.java` | 6 | **4** | veteran — field symmetry must be confirmed, not guessed |
| `frc/robot/util/FaultReporter.java` | 6 | 2 | rookie |
| `frc/robot/util/PowerBudget.java` | 5 | 3 | veteran |
| `frc/robot/util/GeomUtil.java` | 7 | **4** | veteran — `rotateAboutZ` is failure mode #1 |
| `frc/robot/util/PhoenixUtil.java` | 6 | 3 | veteran |
| **4.3 Drivetrain** | | | |
| `subsystems/drive/Drive.java` | 22 (most KEEP) | 4 | veteran (lane A) |
| `subsystems/drive/ModuleIO.java` | 1 | 1 | rookie |
| `subsystems/drive/ModuleIOTalonFX.java` | 3 | 3 | veteran |
| `subsystems/drive/ModuleIOSim.java` | 1 | 2 | rookie |
| `subsystems/drive/GyroIOPigeon2.java` | 1 | 2 | rookie |
| **4.4 Intake** | | | |
| `subsystems/intake/IntakeIO.java` | 10 | 1 | rookie |
| `subsystems/intake/IntakeIOTalonFX.java` | 11 | 3 | veteran |
| `subsystems/intake/IntakeIOSim.java` | 9 | 3 | rookie (with help on `SingleJointedArmSim`) |
| `subsystems/intake/Intake.java` | 26 | 4 | veteran (lane B) — homing + pivot-band guard + feed assist |
| **4.5 Floor roller** | | | |
| `subsystems/floorroller/FloorRollerIO.java` | 6 | 1 | rookie |
| `subsystems/floorroller/FloorRollerIOTalonFX.java` | 7 | 2 | rookie |
| `subsystems/floorroller/FloorRollerIOSim.java` | 6 | 2 | rookie |
| `subsystems/floorroller/FloorRoller.java` | 9 | 2 | **rookie — the best first FSM on the robot** |
| **4.6 Indexer** | | | |
| `subsystems/indexer/IndexerIO.java` | 6 | 1 | rookie |
| `subsystems/indexer/IndexerIOTalonFX.java` | 7 | 2 | rookie |
| `subsystems/indexer/IndexerIOSim.java` | 8 | 2 | rookie |
| `subsystems/indexer/Indexer.java` | 12 | 2 | rookie — feed/unjam FSM; jam thresholds need tuning patience (§2.4 replan) |
| **4.7 Turret** | | | |
| `subsystems/turret/TurretIO.java` | 9 | 1 | rookie |
| `subsystems/turret/TurretIOTalonFX.java` | 10 | 4 | veteran — abs encoder + Motion Magic + FF units |
| `subsystems/turret/TurretIOSim.java` | 8 | 3 | rookie |
| `subsystems/turret/Turret.java` | 26 | **5** | veteran (lane B) — unwrap + velocity FF + homing |
| **4.8 Shooter** | | | |
| `subsystems/shooter/ShooterIO.java` | 7 | 1 | rookie |
| `subsystems/shooter/ShooterIOTalonFX.java` | 8 | 3 | veteran — leader/follower + `Follower` opposition |
| `subsystems/shooter/ShooterIOSim.java` | 7 | 2 | rookie |
| `subsystems/shooter/Shooter.java` | 18 | 3 | veteran — `willBeAtSpeedIn()` prediction + shot detection |
| **4.9 Shooting on the move** | | | |
| `shooting/ShotParameters.java` | 2 | 1 | rookie |
| `shooting/ShootingSolver.java` | 10 | **5** | strongest veteran (lane D) |
| `shooting/TurretUnwrapper.java` | 5 | **5** | strongest veteran (lane D) |
| **4.10 Vision & QuestNav** | | | |
| `subsystems/vision/VisionIO.java` | 5 | 1 | rookie |
| `subsystems/vision/VisionIOLimelight.java` | 6 | 4 | veteran (lane C) — MegaTag2 write-then-read ordering |
| `subsystems/vision/VisionIOSim.java` | 6 | 3 | rookie |
| `subsystems/vision/QuestNavIO.java` | 3 | 1 | rookie |
| `subsystems/vision/QuestNavIOReal.java` | 4 | 3 | veteran — beta vendordep, verify every signature |
| `subsystems/vision/QuestNavIOSim.java` | 6 | 2 | rookie |
| `subsystems/vision/Vision.java` | 15 | **5** | veteran (lane C) — the whole fusion policy |
| **4.11 Superstructure** | | | |
| `subsystems/superstructure/Superstructure.java` | 33 | **5** | strongest veteran (lane D) — two orthogonal regions |
| **4.12 Autos** | | | |
| `autos/AutoFactory.java` | 12 | 4 | veteran (lane D) |
| `autos/AutoRoutines.java` | 18 | 3 | veteran + rookie pair |
| `autos/AutoSelector.java` | 8 | 2 | rookie |
| **4.13 Commands** | | | |
| `commands/DriveCommands.java` | 10 (most KEEP) | 3 | veteran (lane A) |
| `commands/DriveToPoseCommand.java` | 8 | 4 | veteran (lane A) |
| `commands/CharacterizationCommands.java` | 11 | 3 | veteran — the turret sweep needs limit guards |
| **4.14 Tests** | | | |
| `test/…/shooting/ShootingSolverTest.java` | 11 | 3 | veteran (lane D) |
| `test/…/shooting/TurretUnwrapperTest.java` | 10 | 2 | rookie — ideal first contribution, no hardware |
| `test/…/superstructure/SuperstructureFsmTest.java` | 20 | 4 | veteran (lane D) |
| **4.15 Non-Java & generated** | | | |
| `deploy/pathplanner/settings.json` | 0 (11 fields) | 2 | rookie + mentor (needs a scale) |
| `deploy/pathplanner/navgrid.json` | 0 (GUI) | 2 | rookie |
| `deploy/pathplanner/paths/*.path` ×17, `autos/*.auto` ×8 | 0 (GUI) | 3 | veteran (lane D) |
| AdvantageScope `config.json` + 3 `.glb` | 0 (9 fields) | 3 | rookie + CAD student |
| `build.gradle` | 0–1 lines | 1 | veteran (lane X) |
| `vendordeps/maple-sim.json` | 0 (optional) | 1 | veteran (lane X) |
| `generated/TunerConstants.java` | 0 (regenerate) | 3 | veteran (lane A) |
| **TOTAL — 57 Java files** | **532** | — | — |

**Reading the table for scheduling.** The five difficulty-5 files — `RobotState`, `Turret`,
`ShootingSolver`, `TurretUnwrapper`, `Vision`, `Superstructure` — are 130 of the 532 methods (24 %)
and account for essentially all of §11.2's failure modes. They belong to three named veterans and
must be code-reviewed by a second person. Everything rated 1–2 is 157 methods (30 %) of genuinely
rookie-appropriate work that can proceed in parallel from week 1: every `XxxIO` interface, every
`XxxIOSim`, `FloorRoller` and `Indexer` end to end (the §2.4 replan made the indexer a
difficulty-2 file), the three simple utilities, `AutoSelector`, and `TurretUnwrapperTest`.

**Rough line estimate from this table:** at 12–18 lines per method plus imports and the constants
file, 532 methods lands at **~7,000–8,500 lines of Java**, which matches §1's estimate. About 400 of
those lines are the JUnit tests.

---

*End of plan.*
