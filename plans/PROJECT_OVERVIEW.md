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

**`plans/PLAN_3_MAXIMUM.md` is the single authoritative plan.** It is a ~7,900-line document
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
- **`claude-test`** — reserved for the integration of the **entire system**: where the full, working
  robot code goes. It is *not* where milestone groups are combined.
- **`integration-*`** — one branch per group of milestones being combined, each with its own sibling
  directory. `integration-m5_m7` joined M5's mechanisms to M7's vision; `integration-m1_m9` combines
  everything through M9 and is the base for M10.
- **`feature-*`** — one branch per unit of work, usually one milestone from the plan's build order.

**No branch is ever deleted**, including ones fully merged into an integration branch. The branches
*are* the documentation — they get walked through with students — so a branch whose content now
lives elsewhere is still the record of how that piece was built. Merged is not the same as finished
with.

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
| — | Camera mountings measured from CAD; intake-camera gating | `feature-cad-constants` | 3 of 4 transforms measured; 6 tests, mutation score 6/6; `kRobotToQuest` still unmeasured |
| — | Combine M1–M9 as the base for M10 | `integration-m1_m9` | Complete — M3 and M4 merged in; 112 tests, 103 passing, 0 failures |
| M10 | `Superstructure`: fourteen states, both interlocks, the `readyToScore` gate | `feature-superstructure` | **Simulation checkpoint complete** — 122 tests, 0 skipped, mutation score 26/26; hardware checkpoint outstanding |
| M10b | Hopper retrofit: multi-piece model, intake rests deployed, intake + shoot at once | `feature-hopper` | **Replanned 2026-09-24** (plan §2.4). Branch cut from `feature-superstructure`; steps 0–4 done (140 tests pass). Steps 5–10 not started; change order below |
| M11+ | Shot tables (now measured in bursts), shoot-on-the-move, autos, hardening | not started | — |

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

### M10, and two interlocks that were each half an interlock

M10 is the `Superstructure`: the only class allowed to command more than one subsystem. All
fourteen `SystemState`s, both directions of the intake/turret interlock, the eight-term
`readyToScore` gate of section 6.5, and `SuperstructureFsmTest`. The nine `@Disabled` placeholders
are gone; the suite runs 121 tests with none skipped.

**The checkpoint asks for ten FSM tests and the skeleton shipped nine.** The plan was right and the
skeleton was one short. The missing case is `everySystemStateIsReachable`, which this project's own
testing standard requires of every state machine and which none of the nine placeholders covered.

**Four things were latent in already-merged code and only a cross-mechanism client could find them.**
Each is the same shape: a rule that was correct for the single caller that existed and wrong for
the one M10 added.

- **`setWantedState(HOME)` is an edge, not a level.** Both position mechanisms clear their homed
  flag and arm a one-shot request that their transition step consumes *ahead of* their own timeout.
  A `homing()` handler that commanded HOME once per loop — which is how every other handler in the
  superstructure commands its mechanisms — restarts homing on every loop, so neither mechanism can
  finish and neither can fault either. The robot sits in HOMING until the robot-level timeout with
  no mechanism alert to say which one was stuck. **A mechanism whose setter has side effects cannot
  be driven from a per-loop handler**, and nothing in the signature says so.
- **The intake's own home latch had the matching hole.** A HOME arriving while the machine is
  already in HOMING produces no transition, so `onEnter` never fires and the flag lingers —
  disabling a FAULTED latch written as "faulted AND nobody is asking to re-home". The turret never
  had the bug, because it consumes its own request unconditionally at the top of its transition
  method. Two mechanisms, the same field, two different lifecycles.
- **The superstructure's fault latch could not be copied from the mechanisms.** They boot into
  IDLE; this one boots into HOMING with HOME already requested, so "faulted AND `wantedState !=
  HOME`" is true at boot and stays true. A homing timeout latched FAULTED for exactly one loop
  before dropping back into HOMING. Three tests caught it at once. **A latch keyed on a request
  being absent is wrong for any machine whose boot state is that request**; key it on the edge.
- **`FaultReporter.getInstance()` returned `null`.** Term 8 of the gate dereferences it, so the
  superstructure would have thrown on its first loop. Every subsystem already carried a comment
  promising to register "once that class is implemented", so implementing it was a prerequisite
  rather than a diversion. The per-device registrations are still M14's, which means
  `anyCritical()` returns false on the robot today and term 8 is inert until they arrive.

**The interlock had a silent hole, and it is the one worth remembering.** The conflict arc is
configured as 150°–210° while the turret's travel is ±200° — *wider than a full circle*. So the
unwrapper legitimately parks the turret at −180° to point where an operator would say +180°, and
`TurretUnwrapper.passesThroughArc`, which compares raw intervals, does not see that branch at all.
The guard reported no conflict for a turret standing squarely in the intake's path: the deploy
guard opened and the arm would have swung into it. **Nothing in the log says anything at all until
the two mechanisms meet.** `passesThroughArcWrapped` tests every full-turn image of the arc and
fails closed on non-finite input; both halves of the interlock use it. The generalisation is worth
carrying: **an angular guard written as an interval is only correct while everything it compares
lives on the same branch**, and a turret with more than 360° of travel guarantees that they do not.

**Mutation testing ran 22/26, then 25/26, then 26/26**, and the fourth survivor was the instructive
one. "The intake must latch FAULTED rather than cycle back into HOMING" was first written as a
single end-of-run sample, which a machine cycling every three seconds passes about half the time.
The obvious repair — watch every loop across a window longer than two timeouts — **also survived**,
and re-running the harness rather than trusting the repair is the only reason that was caught. The
defect is a *count*, not a state: the spurious re-home happens early, finishes before any window
that begins after the robot has faulted, and leaves the machine sitting in a perfectly well-behaved
FAULTED. The case now counts entries into HOMING from boot and asserts exactly one. **When a defect
is "it happened twice", no assertion about the final state can see it.**

**Two things deliberately do not work, and both are recorded as named tests rather than left to be
rediscovered.** The readiness gate cannot open at all on this branch: `kRpmPerMeterPerSec` and
`kRpmIntercept` are both zero awaiting M11, so every solved shot carries a flywheel speed of zero
and `Shooter.willBeAtSpeedIn` refuses a zero goal by design. Term 3 is false however good every
other input is, and `theReadinessGateCannotOpenUntilTheShotTablesAreMeasured` makes the other seven
true first and then pins the one that cannot be. Separately, `getRequestedDriveHeading` is
unreachable: the other route into `WAITING_FOR_TURRET` is an aim point outside the travel, and a
travel wider than a full circle reaches every direction on some branch. **It is not dead code** —
it becomes reachable the moment M6 measures hard stops that are almost certainly narrower than the
±200° placeholder.

What M10 has **not** established is anything about hardware. No interlock has been tested against
real geometry, and `kIntakeConflictMinRad` and `kIntakeConflictMaxRad` are still guesses. The
checkpoint clause that matters most — deliberately deploy the intake into the turret's arc and
confirm both directions hold — has been discharged in simulation against simulated geometry only,
and simulated geometry cannot be wrong about where the mechanisms actually are.

### Replanned: the robot carries a hopper, not one piece

REBUILT lets a robot hold as many game pieces as it can carry, and the plan had modelled the robot
as a one-piece slot. `Indexer.hasPiece()` was the pivot of intaking, scoring and autos, and M5 and
M10 were built that way. Section 2.4 of the plan is the replan. It is written down here because it
reverses decisions already implemented on `feature-indexer` and `feature-superstructure`. Those
branches are left as they are, as the record of the single-piece design; the new milestone **M10b**
retrofits them on a new branch.

The decisions, each made by the user:

- **No piece sensors.** The robot cannot count what it holds, and cannot reliably tell that it
  holds anything. So nothing is gated on having a piece. The readiness gate drops from eight terms
  to seven, and scoring feeds whenever the shot is ready. Shots are counted at the **flywheel**,
  where every scored piece takes a measurable bite out of wheel speed. "Probably empty" is a
  timed belief that autos use to end a scoring leg, never a teleop gate.
- **The intake rests deployed.** Releasing the intake button stops the roller and leaves the arm
  down, because raising it would dump the hopper. It stows only on an explicit `STOW`. A new
  partial-raise `FEED_ASSIST` position pushes the hopper toward the indexer while scoring.
- **Shoot, move and intake at once.** Intake and scoring become two independent requests, modelled
  as orthogonal regions of the Superstructure rather than doubling the scoring states. The
  existing `PowerBudget` priorities already shed the intake first when current runs short.
- **The interlock was stricter than the robot.** It treated a *deployed* intake as being in the
  turret's way. On this robot the deployed intake cannot reach the turret, so the conflict becomes
  a band of pivot angles the arm passes *through*, to be measured from CAD. Until that band is
  measured the interlock is inert. That is the same shape as M8's identity-transform trap, and it
  is recorded as a named risk.

The review questions were answered on 2026-09-25 and are recorded in §2.4:
- The hopper holds 50 pieces and does not jam when overfilled, so no "full" signal is needed.
- The *stowed* intake does conflict with the turret, over roughly the top 15° of its travel. The
  feed-assist position does not.
- Feed-assist rocks back and forth rather than holding still.
- The robot starts with 8 pieces and the intake stowed, as the rules require. Pieces can fall out of
  a stowed intake, so the first enable deploys it and every auto waits for that before moving.
- The floor roller runs only to feed the indexer. It starts as soon as a shot is requested, and the
  stopped indexer holds pieces back until the shooter is ready.

What remains is on-robot tuning: the exact band edge and the rocking period.

### M10b: the order in which built code changes

The replan touches code that already exists. M5's intake, indexer and floor roller, M4's flywheel,
M10's superstructure, and the vision intake-camera gate were all built to the single-piece model.
This section is the order to change them in. It was written against `feature-superstructure` as it
stood when the replan was accepted: 122 tests, all passing. (M10's record below says 121; `1244f94`,
which gave `PASSING` its own gate, added the 122nd after that record was written.)

**Where the work happens.** M10b gets its own branch, `feature-hopper`, cut from
`feature-superstructure` because it depends on M10. It lives in its own sibling directory, and main's
documentation is merged in first so the branch carries the §2.4 plan it implements:

```bash
cd AdvantageKitRebuilt
git clone ./feature-superstructure feature-hopper
cd feature-hopper
git remote set-url origin https://github.com/MrGotGame/Rebuilt-claude-test.git
git checkout -b feature-hopper feature-superstructure
git fetch ../AdvantageKit_TalonFXSwerveTemplate main && git merge FETCH_HEAD
```

`feature-indexer`, `feature-intake` and `feature-superstructure` are **not** edited. They stay as
the record of the single-piece design, which is the point of keeping every branch.

**The rules that fix the order:**

1. **Add before you switch, and switch before you delete.** New behaviour goes in alongside the old
   API first. The superstructure then switches over. Only after that is the old API deleted, in a
   commit whose sole job is deletion, so the compiler proves nothing still uses it.
2. **Bottom up.** The superstructure is the only client of the mechanisms' new behaviour, so the
   mechanisms change first and are tested on their own suites.
3. **Every step ends with `./gradlew build test` green.** Every behaviour change also gets a case
   that fails without it. That is the testing standard below, not a new rule.
4. **One step, one commit, one reason.** The interlock correction in particular is its own commit.
   It is the part of this replan most likely to be walked through with students.

| # | Step | Files | Why it sits here | Done when |
|---|---|---|---|---|
| 0 | Branch setup and baseline | — | Establishes the starting test count, so later steps can show what they replaced. | Branch builds; 122 tests pass. |
| 1 | **Constants, additive only** | `Constants.java` | Every later step reads these. Adding them changes no behaviour. Old constants such as `kHasBeamBreak`, `kHasPiece*` and `kIndexFeedSec` stay until step 8. | New intake (`kFeedAssist*`, `kTurretConflictBandRad`), indexer (`kLoaded*`, `kJam*`, `kUnjam*`), shooter (`kBurstToleranceRpm`, `kShot*`), `Superstructure.kEmptyConfirmSec`, `Auto.kIntakeRetainTimeoutSec`, and `Auto.kScoreTimeoutSec` → 8.0. Build green. **Done: `76eac25`.** |
| 2 | **Shooter: shot detection** | `Shooter`, `ShooterIOSim`, `ShooterFsmTest` | Fully independent of everything else and purely additive. It also has to exist before anything can stop counting shots at the indexer. | `getShotCount` / `getSecondsSinceLastShot` / `resetShotCount`, the `willBeAtSpeedIn(horizon, tolerance)` overload, and `injectShot()`. New cases: one shot per injected piece at cadence, spin-up is not a shot, the refractory window holds. **Done: `f4ea778`, plus `c9658df` for the three conditions the first mutation run showed the simulated plant could not isolate (35/36, the one survivor being M10's known equivalent mutant).** |
| 3 | **Intake: resting and feed-assist states, band predicates** | `Intake`, `IntakeFsmTest` | The superstructure's new intake region needs these to exist. `isStowed()` is kept, so nothing that calls it breaks yet. | `DEPLOY_ONLY` settles in a real `DEPLOYED` state rather than lingering in `DEPLOYING`. `FEED_ASSIST` rocks between the two angles, and holds still at period 0 or when power is short. `isInTurretConflictBand()` is true while stowed and false when deployed. `isRetainingPieces()` is added. **Moved to step 5:** making the deploy guard consult only motions into the band. `SuperstructureFsmTest`'s interlock case pins the old rule, so changing this half alone would end the step red; both halves now change together. **Done: `e98bedb`; all 11 step 3 mutations killed on the first run (46/47 overall, the survivor being M10's known equivalent).** Writing it also found and fixed a latent M5 bug: arrival was judged against the previous loop's goal, so every change of request reported a false arrival for one loop -- including a false `STOWED` that the M10 turret interlock reads. |
| 4 | **Vision: widen the intake-camera gate** | `Intake.isAtIntakingAngle()`, `RobotContainer` (the three `intake::isIntaking` gate suppliers), `IntakeCameraGateTest` | Needs step 3's `DEPLOYED` state. It is separate from the superstructure because it is a *vision* decision with its own suite and its own failure mode. | The gate admits `INTAKING` and `DEPLOYED`, which share one pivot angle, and still rejects `EJECTING`, `FEED_ASSISTING` and every transit state. A new case fuses a frame while `DEPLOYED`. The existing `EJECTING` rejection case still passes. **Done: `f5c0ac7`; all four gate mutations killed on the first run (50/51 overall, the survivor being M10's known equivalent). The new case wires a real intake, not a bare boolean, behind the gate.** |
| 5 | **The interlock correction** | `Superstructure` (one line of wiring plus `intaking()`), `Turret` Javadoc, `SuperstructureFsmTest` | The smallest change that makes the interlock match the real robot, and **both halves of it at once**, so they can never disagree. It has to land before step 7, because step 7 lets the turret aim while intaking, which is only safe once the guard keys on the band. | `turret.setIntakeArcGuard(intake::isInTurretConflictBand)` replaces `() -> !intake.isStowed()`. On the intake side, the deploy guard now blocks only motions *into* the band (in practice, a stow); leaving the band is always allowed, and a blocked stow waits clear of the band instead of falling back to stowing. `IntakeFsmTest.feedAssistIsRefusedByTheGuardToo` is rewritten to match. `intaking()` no longer stows the turret. The interlock test gains its negative case: a **deployed** intake *may* share the arc with the turret, while a **stowed** one blocks it in both directions. |
| 6 | **Indexer: jam handling and the load diagnostic, additive** | `Indexer`, `IndexerIOSim`, `IndexerFsmTest` | The superstructure's step 7 needs `isUnjamming()` and `isLoaded()`. `hasPiece()` and `INDEX`/`HOLD` are left working, because the superstructure still calls them until step 7. | `FEEDING` detects a stall, then goes to `UNJAMMING`, then back to `FEEDING`, and latches `JAMMED` with an alert after `kMaxUnjamAttempts`. `isLoaded()` is evaluated only while feeding forward. `setSimulatedLoad` / `setSimulatedJam` hooks are added. |
| 7 | **Superstructure switchover** | `Superstructure`, `RobotContainer` bindings, `Robot` init hooks, `SuperstructureFsmTest` | The one big step, and last among the behaviour changes because it consumes all of steps 2–6. `RobotContainer` has to change in the same commit, because removing `WantedState.INTAKE` breaks its binding. | `INTAKE`, `INTAKING` and `INDEXING` are removed. `STOWED` is renamed `RESTING` and `STOW` is added. It adds `setIntakeRequested`, `intakeWhileHeldCommand`, `resolveIntakeMode`, `applyFeedSide` (floor roller leads, indexer gates), `onEnable` (first enable deploys) and `StowedWhileMoving`. The piece guard and gate term 4 are gone. It adds the burst latch and `likelyEmpty()`, and `scoreUntilEmptyCommand` waits on it. `Robot.autonomousInit`/`teleopInit` call `onEnable` and `shooter.resetShotCount`. The §4.14 cases are all present. **No production code calls `hasPiece()`, `INDEX`, `HOLD` or `FloorRoller.INTAKE` any more.** |
| 8 | **Delete the single-piece model** | `Indexer`, `IndexerIO*`, `FloorRoller`, `Constants`, `IndexerFsmTest`, `FloorRollerFsmTest` | Only safe now: the compiler proves step 7 left no caller. It is a deletion-only commit, so the diff reads as exactly what the replan retired. | `hasPiece`, `INDEX`/`HOLD`, `INDEXING`/`HOLDING`, the latched-sample replay and settle window, the indexer shot counter, the beam-break path and alert, `setSimulatedPiecePresent`, `FloorRoller.INTAKE`/`INTAKING`, and the old constants (`kHasBeamBreak`, `Dio.kIndexerBeamBreak`, `kHasPiece*`, `kShotDetectDebounceSec`, `kIndexRpm`, `kIndexFeedSec`) are all deleted. **`kDetectionSettleSec` stays:** a spin-up from rest reads as both "loaded" and "jammed", so step 6's detectors need the same settle window the old piece detector did. Test cases about deleted behaviour are deleted too, not adapted. Build green. |
| 9 | **Autos: markers and skeletons** | `AutoFactory`, `AutoRoutines` | These are still skeletons (PathPlanner is unconfigured), so this is mostly the vocabulary and TODOs. It comes last because it depends on step 7's API and nothing depends on it. | `Intake`/`StopIntake` markers use `setIntakeRequested`, and `StowIntake` is added. `WaitForPiece`, `waitForPieceCommand` and `bailoutIfEmpty` are removed. `waitUntilIntakeRetaining()` opens every routine, and `driveIntakeAndShoot` and `Auto/PreloadFired` are added. |
| 10 | **Verify, then record** | `docs`, this file, `CONTEXT_AND_CONVENTIONS.md` | Evidence before claims. | A mutation pass aimed at the four places a plausible bug survives happy-path tests: the `likelyEmpty()` accumulator (gate-closed time must not count), the burst latch, the band predicate's direction, and the floor-roller-leads rule. The simulation checkpoint from plan §10.2 M10b is run. Then the milestone table, the branch table, and a short M10b write-up in the style of the M8–M10 sections, including what surprised us. |

**Two ordering traps, recorded before they happen:**

- **Do not fold step 5 into step 7.** If the turret guard still reads `!intake.isStowed()` when
  step 7 lets the turret aim during intaking, the turret is blocked for exactly the whole time it
  now needs to aim, and shoot-while-intaking silently never fires. It would look like a gate
  problem. Landing the interlock first, with its own test, makes that failure impossible to
  mistake for anything else.
- **Step 4 before step 7, not after.** Once step 7 lands, the intake rests in `DEPLOYED` for most
  of the match. With the old gate that quietly switches the intake camera off, and nothing fails.
  The camera just contributes nothing, which is the "silently inert" failure this project has met
  three times already.

**Downstream.** M11 builds on `feature-hopper`, not on `feature-superstructure`, because the shot
tables are measured in bursts through the hopper feed path.

### The intake camera is not where the plan says it is

Section 7.2 of `PLAN_3_MAXIMUM.md` states:

> *"The intake camera needs none of this: its transform is the constant `kRobotToIntakeCamera`,
> pushed once at construction."*

**That is wrong, and the CAD is what showed it.** The intake Limelight rides the *pivoting* intake,
roughly **0.4 m from the pivot axis** — nearly three times the turret camera's 0.139 m lever arm —
so a 60° deploy sweeps the lens through about 0.4 m. A single constant transform describes exactly
one pivot position and is wrong everywhere else.

The chosen response is **gating rather than generalising**: the camera is fused only while the
intake is settled at its intaking position, which is the pose the constant was measured in. That
buys correctness cheaply, at the cost of the camera contributing nothing while stowed, ejecting or
in transit. Giving it M8's full moving-transform machinery remains the option if that cost bites.

Two details are worth carrying forward because both were nearly missed:

- **`EJECTING` must not count as "deployed".** The intake *is* out in that state, but at a
  different angle, so admitting it would fuse poses computed with the wrong transform. That error
  is a plausible-looking constant rather than an obvious sweep, which makes it the harder of the
  two to notice.
- **The gate judges the moment of capture, not of arrival** — M8's blur-gate lesson in a new
  costume. An intake that has just reached its setpoint still has 60–120 ms of frames in flight
  that were exposed mid-arc, and they arrive looking entirely ordinary. **Any rule about where a
  mechanism was needs a timestamped history, not a live read.** That is now three separate places
  this has come up; treat a live read of mechanism state inside vision as a defect by default.

### What CAD could and could not answer

`tools/step_inspect.py` and `tools/onshape_measurements.py` exist for this, and
`docs/CAD_MEASUREMENTS.md` carries the detail. The short version, because the expectation is easy
to get wrong: **CAD answered four of roughly thirty-three outstanding measurements.** Every SysId
gain, every encoder offset, every hard-stop angle, the field dimensions, the game piece and the
robot mass come from somewhere else. It is not a shortcut past M6.

What it did answer, it answered well. The hardest fact — where the robot origin is — came free:
the assembly origin already coincides with it, confirmed three independent ways. And the lens
positions were recovered from the geometry rather than guessed, by finding the concentric barrel of
circles each Limelight model carries along its optical axis. **Using the housing bounding-box
centre instead would have been ~19 mm out along the camera's own view direction**, which is right
at the M8 checkpoint's tolerance and would have rotated with the turret.

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
- **One camera mounting transform is still identity.** `kRobotToTurretAxis`, `kTurretToCamera`
  and `kRobotToIntakeCamera` have been **measured from CAD** and carry real values;
  `kRobotToQuest` remains `TODO: MEASURE`, because the headset is a tessellated placeholder in
  the STEP export with no geometry. Measure it to the **centre of the headset's front face**:
  QuestNav reports a "third-eye" pose on the lens plane, and the Quest 3S has no lens tilt
  (the Quest 3's 7° does, but that is a different headset). The transform *maths*
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

### Shared singletons make tests order-dependent, and merges expose it

`RobotState` is a process-wide singleton, the suite runs in **one JVM with no forking**, and
`RobotState.initialize()` is idempotent — so every test class shares one pose estimator, and class
ordering decides what state each class inherits.

This surfaced when M3 and M4 were merged into `integration-m1_m9`.
`TurretCameraTransformTest.theCorrectionIsExactAndNotMerelyClose` began failing about **half** of
all full-suite runs, at ~1.75 mm against a 1 mm bound, **with no production code changed**. Adding
two test classes had changed the ordering. The case had been passing partly because the previous
ordering happened to suit it: it was latently fragile before the merge, and the merge only revealed
it.

Two things are worth copying from how it was resolved:

- **Measure the defect before touching the bound.** Injecting the two defects the case exists to
  catch gave 4.27 mm and 8.53 mm, against 1.75 mm of contamination — which is what established
  that the production code was fine and the bound was what needed to move. Without those numbers
  the only honest options would have been to guess or to leave it red.
- **A loosened bound must be re-falsified.** Widening a tolerance until a test passes is the move
  this standard forbids. Deriving it from measured defect magnitudes is different, but only if you
  then prove the new bound still catches them — all three relevant mutations were re-run and all
  three still fail. **A bound that no longer catches its defects is worse than no bound**, because
  it still reads as coverage.

The margin left is thinner than it should be, and the real fix is to stop sharing `RobotState`
between test classes rather than to keep re-deriving the number. That is recorded in the test
itself as the standing recommendation.

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

`SuperstructureFsmTest`'s cases used to report as SKIPPED, as `@Disabled` placeholders belonging to
a later milestone. **M10 implemented them**, and the suite now runs with nothing skipped at all. A
skipped test appearing in the summary again is therefore a regression to investigate rather than an
expected state.
