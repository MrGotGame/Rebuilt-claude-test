# Project Overview

## What this repository is

This repository is a **stress test of Claude Code's ability to plan and then implement a
non-trivial software project end to end** — specifically, the robot code for an FRC team competing
in the 2026 game *REBUILT*.

The robot code is real and intended to work. But the reason the repository exists, and the reason
its history is structured the way it is, is to produce a legible, step-by-step record of how an AI
coding agent takes a project from "here is the hardware and some requirements" to "here is working,
tested, documented code" — and to make every step of that process reviewable after the fact.

Two things follow from that, and they explain most of the choices in this repo:

1. **The git history is part of the deliverable, not a byproduct.** Each branch is a self-contained
   demonstration of one unit of work. Commit messages are long and explain *why*, not just *what*,
   because the history is meant to be read.
2. **Verification claims are expected to be falsifiable.** "It compiles" means a build was run.
   "Tests pass" is treated as weak evidence unless the tests have been shown to fail when the code
   is broken. Where something could not be verified, the code and the commit say so explicitly
   rather than implying success.

## The robot

A swerve-drive robot with a turreted shooter, built for the 2026 FRC game *REBUILT*.

| Subsystem | Hardware |
|---|---|
| Drivetrain | SDS Mk5n modules — Kraken X60 drive, Kraken X44 steer, CTRE CANcoders, CTRE Pigeon2 |
| Pivoting intake | Kraken X44 pivot with WCP ThroughBore on the gearbox output; Kraken X60 roller |
| Floor roller | Kraken X60 |
| Indexer | Kraken X60 |
| Turreted shooter | Kraken X44 turret with WCP ThroughBore on the gearbox output; 2x Kraken X60 flywheels in leader-follower |
| Vision | 2x Limelight 4 (one turret-mounted, one intake-mounted); Meta Quest 3S running QuestNav |

The drivetrain is on a CANivore; the mechanisms are on the roboRIO bus. CAN IDs are scoped per
device type, so the same numeric ID on two *different* kinds of device is legal and is not a
conflict.

Software stack: WPILib 2026, AdvantageKit 26.0.2 (IO-layer logging and log replay), CTRE Phoenix 6
26.1.2, PathPlannerLib 2026.1.2, questnavlib 2026-3.0.0-beta. Java 17, Gradle, with Spotless and
`googleJavaFormat` applied automatically on `compileJava`.

## The plan

**`plans/PLAN_3_MAXIMUM.md` is the single authoritative plan.** It is a ~7,200-line document
covering architecture, a file-by-file specification with every method signature, constants strategy,
the shooting-on-the-move math, vision fusion, autonomous, logging and simulation, a student build
order with milestones, and a risk register.

Three competing plans were originally generated at different sophistication tiers — simple,
balanced, and maximum. Tier 3 was chosen, and **tiers 1 and 2 were deliberately deleted** so that
future work follows exactly one document with no ambiguity about which architecture is
authoritative. They should not be recreated.

The architecture it specifies, in brief:

- A central `RobotState` holds the only pose estimator, fed odometry at ~250 Hz and vision at ~30 Hz,
  with time-interpolated buffers for pose and turret angle.
- Every subsystem uses AdvantageKit's IO pattern (`XxxIO` + `@AutoLog` inputs + `XxxIOTalonFX` +
  `XxxIOSim` + an empty implementation for log replay).
- Finite state machines per subsystem, coordinated by a single `Superstructure` that is the only
  class allowed to command more than one subsystem.
- All constants live in `Constants.java` as tunable values editable from AdvantageScope.

## How work is organised

### Directory layout

Each branch gets its own working directory, as a sibling clone. They all share one GitHub remote.

```
AdvantageKitRebuilt/
├── AdvantageKit_TalonFXSwerveTemplate/   branch: main          reference copy, do not edit
├── claude-test-branch/                   branch: claude-test   integration line
├── feature-drive/                        branch: feature-drive
├── feature-robot-state/                  branch: feature-robot-state
├── feature-shooting-parameters/          branch: feature-shooting-parameters
└── feature-flywheel/                     branch: feature-flywheel
```

Separate directories rather than one repo with branch switching, so that a known-good checkout is
always available for comparison and for deploying to the robot while experimental work proceeds
elsewhere.

### Branch roles

- **`main`** — the reference line. Holds the original skeleton and shared documentation such as this
  file. Feature work is **not** merged here.
- **`claude-test`** — the integration line that features branch from and eventually merge into.
- **`feature-*`** — one branch per unit of work, usually one milestone from the plan's build order.

### Creating a feature branch

Clone locally from the branch you are basing on — not from GitHub, because the repository is private
and a non-interactive shell cannot answer a credential prompt — then repoint the remote:

```bash
cd AdvantageKitRebuilt
git clone ./claude-test-branch feature-example
cd feature-example
git remote set-url origin https://github.com/MrGotGame/Rebuilt-claude-test.git
git checkout -b feature-example claude-test
```

Most features branch from `claude-test`. A feature branches from *another feature* only when it
genuinely depends on it — `feature-robot-state` branches from `feature-drive` because the drivetrain
must exist before anything can feed the pose estimator. That dependency is itself part of what the
history demonstrates: the plan's build order claims certain milestones can proceed in parallel, and
the branch graph is the evidence for or against that claim.

### Picking up shared documentation without merging features back

Files like this one live on `main` and are pulled *into* feature branches. Feature work never flows
back into `main`:

```bash
cd feature-example
git fetch ../AdvantageKit_TalonFXSwerveTemplate main
git merge FETCH_HEAD -m "Pull shared documentation from main"
```

### Where this is going

Future branches will be **integration branches** that combine completed features, to demonstrate how
merge conflicts are handled — or, preferably, avoided.

The stated strategy is to **avoid conflicts by implementing stand-alone features one at a time**.
Where two branches genuinely need the same new file, the practical technique used so far is to make
the shared file byte-identical on both branches so the merge is trivial. `LoggedTunableNumber.java`
was copied verbatim from `feature-drive` into `feature-shooting-parameters` for exactly this reason.

A sensible merge order for the current branches is `feature-drive` → `claude-test` first, then
`feature-robot-state` (which fast-forwards cleanly, being stacked on drive), then
`feature-shooting-parameters` (independent, with `LoggedTunableNumber` identical on both sides), and
`feature-flywheel` last. Flywheel goes last because it carries copies of four util classes taken
byte-identical from `feature-drive` — `PhoenixUtil`, `PowerBudget`, `LoggedTunableNumber` and
`LoggedTunableGains` — so once drive is in, those four resolve to no change at all.

## Milestone status

Milestones are defined in section 10 of `PLAN_3_MAXIMUM.md`.

| Milestone | Scope | Branch | Status |
|---|---|---|---|
| M1 | Regenerate drivetrain, field-relative driving | `feature-drive` | Code complete, builds; on-robot checkpoint outstanding |
| M2 | `RobotState` owns the pose | `feature-robot-state` | Code complete, builds; **hardware checkpoint outstanding** |
| M3 | `ShotParameters`, `ShootingSolver`, `TurretUnwrapper` + tests | `feature-shooting-parameters` | Complete — 24 tests pass, mutation score 13/13 |
| M4 | Flywheel: `ShooterIO`, `ShooterIOTalonFX`, `ShooterIOSim`, `Shooter` | `feature-flywheel` | Code complete, builds; **simulation checkpoint complete**; hardware checkpoint outstanding |
| M5 | Intake, floor roller, indexer, turret | `feature-intake`, `feature-floor-roller`, `feature-indexer`, `feature-turret` | Code complete, builds; FSM suites pass; hardware checkpoints outstanding |
| M6 | Characterize everything | — | not started; needs hardware |
| M7 | Intake Limelight, rejection ladder, covariance model | `feature-limelight` | Complete in simulation — rejection ladder 5/5 under mutation; hardware checkpoint outstanding |
| M8 | Turret camera: `robotToTurretCameraNow/At`, slew-rate rejection | `feature-limelight-complete` | **Simulation checkpoint complete** — 12 + 5 tests, mutation score 7/7; hardware checkpoint outstanding |
| M9 | QuestNav: trust gate, seeding, re-seeding, degradation | `feature-questnav` | **Simulation checkpoint complete** — 18 tests, mutation score 12/12; hardware checkpoint outstanding |
| M10+ | Superstructure, commands, autos | not started | — |

M2's checkpoint is a hard gate in the plan: drive a 3 m square by hand and confirm the estimated
pose closes within ~10 cm, that field-relative speeds match a stopwatch, and critically that
`getPoseAt(Timer.getFPGATimestamp() - 0.1)` returns a pose roughly 0.1 s behind the current one. The
entire vision and shooting stack rests on that last one.

M4's simulation checkpoint **has** been performed, and deliberately included negative cases, because
this repository treats a passing observation as weak evidence until it has been seen to fail. What
was confirmed in simulation:

- `Shooter/SystemState` walks `IDLE → SPINNING_UP → AT_SPEED` on command.
- `Shooter/WillBeAtSpeed` goes true *before* `Shooter/AtSpeed`. The gap between those two series is
  the shot cadence that predictive readiness buys; no gap would mean the filtered acceleration
  estimate is not working.
- With the gains zeroed the flywheel never reaches speed, and the machine goes
  `SPINNING_UP → FAULTED` after `kSpinUpTimeoutSec` with the alert raised. This is what proves
  `atSpeed()` is not trivially true and that the timeout is live code rather than an unreachable
  branch.
- A goal of zero resolves to `IDLE` rather than faulting. That guard is not in the plan's
  specification; it was added because `Superstructure` calls `setGoal(0.0)` whenever the shooting
  solver returns an unusable solution, which — with the shot tables still all zeros — is its shipped
  behaviour today.
- Editing `Shooter/Kp` alone, with no redeploy, visibly changes the response: a high `kP` spins up
  quickly and overshoots, a low `kP` spins up slowly and settles short. That isolates the
  `LoggedTunableNumber.ifChanged → io.setGains` path, which matters because a broken `ifChanged`
  fails *silently* — the flywheel simply keeps running on its compiled-in gains and nothing reports
  a problem.

That is every clause of M4's checkpoint that can be answered without a robot. **What M4 has not
established is anything about hardware**: the flywheel has never spun, so "holds a commanded RPM
within `kToleranceRpm`" is untested, and `kFollowerOpposesLeader` is still a guess that must be
checked against the gearbox before the first spin-up.

One observation from the `kP` sweep is worth carrying into M6. That a *low* `kP` settles short —
rather than simply arriving slowly — means the proportional term is making up a steady-state error
the feedforward should have covered, and with `kI` at zero there is nothing else to close it. That
is the signature of a `kV` that under-predicts the voltage needed to hold speed, which is expected:
`kV` is still the guessed 0.118, and `kMoiKgM2` feeding the simulated plant is a placeholder marked
*TODO: from CAD*. It means `kP` is currently compensating for a feedforward error rather than doing
its own job. Do not tune around it — M6's `shooterStaircase` measures `kS` and `kV` at true steady
state, and the gains should be revisited only after that, on hardware, where the numbers that
actually matter live.

### M8, and what "the highest-value single test in the project" cost to make real

M8's checkpoint — slew the turret through 180° with the robot stationary and confirm
`RobotState/EstimatedPose` does not move — now runs as a JUnit case on every push. Getting there
took four separate fixes, and every one of them was a way the test could have passed while proving
nothing. They are worth recording, because the same shapes will recur:

- **The thing under test was a stub.** `GeomUtil.rotateAboutZ` returned its argument. The checkpoint
  names that method as the first suspect when the pose swings, so it could not be deferred.
- **The clamping bug again.** `RobotState.getTurretAngleAt` delegated straight to
  `TimeInterpolatableBuffer.getSample`, which clamps out-of-range queries to its nearest endpoint
  instead of returning empty — the identical defect M7 found in `getPoseAt`. Rule 7 of the rejection
  ladder fires on that method returning empty, so the rule was unreachable code. **When a method's
  contract says "or empty if out of range", check that WPILib agrees.**
- **The simulator could not be wrong.** `VisionIOSim` reported ground truth directly, so no transform
  error could exist in simulation at all. It now solves the way a Limelight does — it fixes its lens
  in the field using the *true* mounting, then subtracts the mounting the robot code *pushed* — so a
  wrong transform produces a wrong pose. A permanent test asserts the sim can still be fooled,
  because the day that stops holding, every other case in the file passes for free.
- **The constants made the geometry degenerate.** `kRobotToTurretAxis` and `kTurretToCamera` are both
  still identity pending CAD, and a lens on the rotation axis does not move when the axis turns —
  so every composition order passes. `Vision` now takes the mounting as constructor arguments,
  defaulting to the constants, and the tests supply a real 0.23 m lever arm.

The expected geometry in those tests is computed with sine and cosine, never with the composition
under test. Deriving it from `rotateAboutZ` would let a backwards composition cancel against itself
— and the correct and backwards orders agree **exactly** at a turret angle of zero, which is why a
bench check with the turret parked certifies a transform that is wrong the moment it moves.

**Mutation testing earned its place here.** The first version of the suite scored 4/7, and the three
survivors were worth more than the four kills. Deleting the capture-time correction entirely, and
un-composing with the newest pushed transform rather than the one in effect at capture, both left
the checkpoint green: its 2 cm tolerance is sized for a backwards composition worth tens of
centimetres, and those defects cost about one. The lesson generalises — **a checkpoint tolerance
calibrated for the loudest failure will not notice the quiet ones, so each mechanism also needs a
test asserting the property it actually promises.** The third survivor was the current-angle
fallback the plan forbids by name; it hid because the rejection it should have caused reappeared
under the same counter from the other half of the same check. Splitting `NO_TURRET_ANGLE` from
`NO_PUSHED_TRANSFORM` fixed both the test and the pit diagnostics. Second run: 7/7.

What M8 has **not** established is anything about hardware. The turret camera has never seen a tag,
and because the mounting constants are still identity, every turret-camera pose on a real robot is
biased by the true camera offset — with the bias rotating as the turret tracks, which reads as a
pose that wanders exactly when the shooter is aiming. The observations are still fused rather than
suppressed, because a silently inert camera is the harder failure to notice, but
`turretTransformUnmeasuredAlert` fires continuously until someone measures it.

### M9, and the difference between a limitation and a bug

M9 completes section 7. Both halves of its checkpoint run as tests: seed against a tag, blind both
Limelights, push 3 m and the pose still tracks; then drop the headset mid-run and confirm trust
goes false, the reason is recorded, and the pose degrades to odometry rather than jumping.

The suite runs on **wheels that under-report by 10 %**, and that choice is the whole point. With
perfect odometry, "the pose tracks with both cameras blind" would hold whether or not a single
QuestNav frame was ever fused — the same vacuity M8 had to design around, in a new costume. One
test asserts the odometry shortfall directly so the checkpoint cannot quietly become a no-op.

Two things are worth carrying forward.

**A test that cannot fail is worse than no test.** The case covering "refuse a re-seed while the
robot is moving" reported three re-seeds during a 3 m drive. The guard was right; the *harness* was
lying to it. Chassis speeds reach `RobotState` through a separate `addDriveSpeedsObservation` call
that `Drive.periodic()` makes and the test did not, so `getFieldRelativeSpeeds()` read zero forever
and the "nearly stationary" guard was satisfied at full speed. **When a test drives `RobotState`
directly, check which of its write paths the real subsystems feed and which the harness has
silently left at zero** — a defaulted input makes a guard untestable without making it look broken.

**Some failures are limits, not defects, and they should be written down rather than tuned away.**
A case asserting that sustained drift withdraws trust with no tags in view failed, and it was right
to. The trust gate's last clause compares the headset against `RobotState`'s estimate — but a
trusted headset *is* that estimate's most heavily weighted part, so with no camera anchoring it the
estimate follows the drift and the residual settles at a small equilibrium. Drift is simply not
detectable without an absolute reference; comparing against odometry instead would not help, since
odometry is a relative source with its own drift. That is now a named, permanent test recording the
limit and what bounds it: M7 had already excluded QuestNav from `getVisionStalenessSec`, so the
staleness clock runs the moment tags are lost and `readyToScore` closes however healthy the headset
looks. **The robot drives on a possibly drifting pose; it does not shoot on one.**

Mutation testing scored 4/7 on M8 and 8/12 here on the first run, and in both cases the survivors
were worth more than the kills. All three real survivors were tests that passed for the wrong
reason — one had blacked out the camera so thoroughly that the rule it was named for was never
reached, one settled for less time than the hold it was meant to outlast, and one covered a
simulator property that production defends against anyway. A fourth apparent survivor was a
*defective mutation*, aimed at a code branch the suite never exercises, which is its own reminder:
**a surviving mutation is a question, not a verdict — check that it mutated something the tests can
actually reach before concluding the tests are at fault.** Second run: 12/12.

One more verification habit earned its keep. The `QuestNavIOReal` skeleton carried
`TODO: VERIFY that surface against the installed beta`, and doing it with `javap` found the plan
wrong: `getBatteryPercent` and `getTrackingLostCounter` return `OptionalInt`, not
`Optional<Integer>`. That particular error is loud, but the lesson is not about that one method —
**a beta dependency's signatures are worth checking against the jar rather than against the
document that describes it**, so the surface this project depends on is now listed in the class's
own documentation for one-pass re-checking.

## Conventions

These are followed throughout and should be maintained.

**Every method carries a permanent documentation block** — a description, an input list (`@param`),
and an output list (`@return`, or an explicit `Outputs:` note for `void` methods describing what it
writes and what hardware it commands).

**Implementation notes are separate from documentation.** Where a skeleton method has been
implemented, the body opens with an `IMPLEMENTATION (was TODO: "...")` comment quoting the original
TODO and explaining how the implementation satisfies it and why the approach was chosen. The Javadoc
survives implementation; the `// TODO:` inside a body is deleted when the work is done.

**All constants live in `Constants.java`**, as `LoggedTunableNumber` when they might need changing
at a competition, and as plain `static final` when they are physically fixed such as a gear ratio or
a CAN ID.

**Field- and game-dependent values are never guessed.** Anything depending on the REBUILT field
carries a `TODO: MEASURE` naming where the number comes from.

## Known global gaps

These affect everything downstream and are not bugs in any one branch:

- **Field dimensions and goal coordinates are still `0.0`.** Alliance flipping and distance-to-goal
  are structurally correct but numerically meaningless until measured from the official field
  drawings or the AprilTag layout JSON.
- **The shot tables are all zeros.** `ShootingSolver` correctly returns an invalid shot rather than
  an infinite flywheel command, but cannot aim until the tables are measured on a real field.
- **PathPlanner is unconfigured.** `AutoFactory.configureAutoBuilder()` is unimplemented, so every
  autonomous routine resolves to `Commands.none()` — silently.
- **Every camera mounting transform is still identity.** `kRobotToTurretAxis`, `kTurretToCamera`,
  `kRobotToIntakeCamera` and `kRobotToQuest` are all `TODO: MEASURE from CAD`. The transform *maths*
  is now implemented and tested, but until the numbers exist each camera's poses are biased by its
  true offset from the robot origin — and for the turret camera that bias rotates with the turret.
  `Vision` raises a warning alert for each unmeasured mounting, and the measurement is a
  prerequisite for M8's and M9's on-robot checkpoints rather than a tuning step after them.
  **`kRobotToQuest` is the worst of the four**: section 7.6 calls it the single largest error source
  in the subsystem, because QuestNav carries the tightest covariance of any source, so a 2 cm error
  there is a 2 cm bias that *wins* the fusion against every camera that disagrees with it.
- **The whole vision stack is code-complete and hardware-untested.** Every method in
  `frc/robot/subsystems/vision/` is implemented as of M9 — which makes it easy to mistake the
  subsystem for finished. No Limelight has ever seen a tag and no headset has ever been connected.
- **No hardware verification has been performed** on any milestone.

## Testing standard

**This is not optional, and it is not advice. Code goes onto a robot only after it has been proven
in simulation, and a test counts as proof only once it has been shown capable of failing.**

The reason is specific to this project. A mechanism defect costs a practice day, a broken belt, or a
match. Every one is cheaper to find in a JUnit run that takes three seconds than on a field.

### Every mechanism gets an FSM suite

One `XxxFsmTest` per subsystem, built over that subsystem's `XxxIOSim`, asserting at minimum:

- the machine reaches **every** `SystemState` (this is clause 2 of the per-mechanism checkpoint);
- the mechanism actually converges under closed loop, not merely that a state name changed;
- each guard **refuses** what it should refuse — a guard nobody tested is a guard nobody has;
- any invariant the class promises. `TurretFsmTest` sweeps 2,880 aim angles asserting the commanded
  position never leaves the travel limits, which is the M5 turret checkpoint discharged as a
  property rather than as a screenshot.

The harness pattern, proven and copyable from any of the M5 suites:

```java
HAL.initialize(500, 0);          // FIRST
Constants.disableHal = true;     // then this, so every tunable reads its compiled-in default
SimHooks.pauseTiming();          // the test drives the FPGA clock, not wall time
// ... build the subsystem over XxxIOSim ...
// step(): periodic(); SimHooks.stepTiming(0.02);
// tearDown(): unregisterSubsystem, resumeTiming, disableHal = false, HAL.shutdown()
```

`SimHooks` matters more than it looks: any state timeout measured in seconds would otherwise depend
on how fast the machine running the suite happens to be.

### Tests must be falsified, not merely passed

A suite whose author also wrote the code proves nothing until it has been broken on purpose. Two
ways, and both are expected:

**Mutation testing.** Inject a defect, confirm a test goes red, revert. M3 set the bar at 13/13;
M5's floor roller and indexer ran 3/3 each, M7's vision ladder 5/5.

> **Commit before you mutate.** A mutation harness reverts with `git checkout --`, which does not
> distinguish your mutation from work you have not committed yet. Doing this to uncommitted M7 code
> destroyed an afternoon of it, and the only reason the loss was recoverable was that the edits
> happened to still be in the session. Commit first, mutate second — the revert can then only
> restore something git already has. **The harness must assert that the substitution actually
changed the file** — a mutation whose anchor silently fails to match reports a perfect score for
doing nothing, which is worse than no mutation testing because it is actively reassuring. That
happened once here and was caught only because the assertion existed.

**Deliberate-failure cases as permanent tests.** Build the FSM over the *empty* `XxxIO` — a
stand-in for a dead motor — and assert the machine still walks its states while every success
predicate stays false. State is a software concept; speed, position and "at goal" are physical ones.
A suite that cannot tell them apart passes against a robot that does not move.

### Vacuous passes seen in this project, all real

These are the shapes to look for, because each one was live in committed code:

- `|measured − goal| < tolerance` is **true at rest** when both are zero. Every such predicate needs
  a non-zero-goal term, or a stopped mechanism reports itself ready.
- **Stall detection on velocity alone cannot see a dead motor**, which reads zero forever. Homing
  accepted it and marked the mechanism trustworthy at an imaginary zero. Require stall *current*.
- A threshold constant left at `0.0` makes `x >= threshold` **vacuously true** and silently disables
  the check it was added for.
- A **spin-up transient** can satisfy a load predicate — high current and a large velocity shortfall
  are also what a game piece looks like. Gate on a settling window, not on a longer debounce.
- `onEnter` **never fires for the state a machine boots into**, because no transition occurs. Entry
  actions for the initial state must be run explicitly in the constructor.

### An intermittent test is a bug report, not flakiness

The indexer suite failed roughly two runs in three and looked flaky. It was reporting a real defect
that would have fired on the robot every single time. **Never retry-until-green and never delete a
flaky test.** Run it several times, find what differs, and fix whichever of the code or the test is
wrong — in that order of suspicion. Derive test timing margins from the constants they depend on
rather than guessing a loop count, so retuning a constant cannot silently re-introduce the problem.

### Continuous integration

`.github/workflows/build.yml` compiles and tests every push and pull request, on every branch. Test
results are rendered into the job summary by `.github/scripts/test_summary.py`, so a red check names
the suite, the case and the assertion without anyone opening a log. A branch that executes **zero**
tests is called out in the summary rather than passing quietly.

## Building and testing

```bash
./gradlew build         # compile, format, and run the unit tests
./gradlew test          # tests only; HTML report at build/reports/tests/test/index.html
```

Note that `build` already includes `test`; there is no way to compile through CI without the tests
running.

Spotless runs automatically before compilation and will reformat sources and strip imports that are
only referenced inside commented-out code.

The `SuperstructureFsmTest` cases report as SKIPPED. They are `@Disabled` placeholders belonging to
a later milestone and are expected to stay that way until the superstructure is implemented.
