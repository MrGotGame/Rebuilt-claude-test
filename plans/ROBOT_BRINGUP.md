# Robot Bring-Up: Tuning, Measuring and Testing on Hardware

Everything in this repository through M13 is complete **in simulation** and has never run on a
robot. This file is the plan for the day a robot is available: what to measure, tune and test,
in what order, why that order is safe, and **which branch each result is committed to**.

Read it with `PROJECT_OVERVIEW.md` (why the repo exists, the milestone records) and
`PLAN_3_MAXIMUM.md` §10.2 (each milestone's checkpoint, whose hardware clauses are the pass
criteria used below).

---

## 0. What does and does not need the robot

**M14 needs the robot for its checkpoint, not for most of its code.** The checkpoint — unplug each
CAN device and see it named on the dashboard, run a full match and finish above
`kBrownoutWarningVolts` — is hardware. **Its code half is done** on `feature-comp_ready`: every device
registered by name, `PowerBudget`, the coast release, `Constants.getRobot()`, the robot loop, and
`kTuningMode = false` with the suite and a simulation boot re-run. The simulation halves of its
checkpoint run as tests. `PROJECT_OVERVIEW.md`'s M14 record has the detail.

Three things still block robot days and need **no** robot. Do them first, or the robot sits idle:

| Before the robot | Why it blocks | Branch |
|---|---|---|
| **M6 code.** Every routine in `CharacterizationCommands` except `measureShotTablePoint` is still a skeleton returning `Commands.none()` (8 of them on `feature-autos`). | Phase 4 cannot characterize anything until they exist. | `feature-superstructure` — the first branch with the `CHARACTERIZE` wanted state that routes a mechanism through the superstructure — then merged forward (§3). Add each to `RobotContainer.configureCharacterizationBindings()` in the same commit. |
| **M13's paths**, drawn in the PathPlanner GUI with the REBUILT field loaded, `navgrid.json` regenerated, and `settings.json` re-saved from `TunerConstants`. | Phase 7. The list is in `PROJECT_OVERVIEW.md`'s M13 record. | `feature-autos` |
| **Field constants** from the official drawings and the AprilTag layout JSON: `Field.kGoalXMeters`, `kGoalYMeters`, `kGoalHeightMeters`, `kFieldLengthMeters`, `kFieldWidthMeters`. | Distance-to-goal, alliance flipping and every shot. They are all `0.0`. | `feature-robot-state` |

---

## 1. The branch map

This is the whole robot's history, as a graph of where each piece was built. `main` holds only the
plan documents, and feature work never flows into it; documentation flows *out* of it into every
branch.

```
main ── plans only (PLAN_3_MAXIMUM, PROJECT_OVERVIEW, CONTEXT_AND_CONVENTIONS, this file)
          │ docs are merged OUT of main into every branch; nothing is merged back

claude-test (skeleton; reserved for the final full-system integration)
 ├─ feature-drive ─────────┬─ feature-robot-state ── feature-limelight ──┐              M1, M2, M7
 │                         └─ feature-turret ──────────────────────────────┤           M5 (turret)
 ├─ feature-intake ────────────────────────────────────────────────────────┤           M5
 ├─ feature-indexer ───────────────────────────────────────────────────────┤           M5
 ├─ feature-floor-roller ──────────────────────────────────────────────────┤           M5
 │                                                                         ▼
 │                                          integration-m5_m7 ── feature-limelight-complete (M8)
 │                                                                  └─ feature-questnav (M9)
 │                                                                      └─ feature-cad-constants
 ├─ feature-flywheel (M4) ───────────────────────────────────────────────────────────┐
 ├─ feature-shooting-parameters (M3) ────────────────────────────────────────────────┤
 │                                                   feature-cad-constants ──────────┤
 │                                                                                   ▼
 │                                                                         integration-m1_m9
 │                                                                                   │
 │   feature-superstructure (M10) ── feature-hopper (M10b) ── feature-still-shot (M11)
 │     ── feature-sotm (M12) ── feature-autos (M13)  ◄ tune here ── feature-comp_ready (M14)
 │
 └─ (full-system integration lands back on claude-test, later)
```

| Branch | Built from | Milestone | What it owns, and so what bring-up commits to it |
|---|---|---|---|
| `claude-test` | — | skeleton | Nothing is tuned here. It receives the full system at the end. |
| `feature-drive` | `claude-test` | M1 | The drivetrain: `TunerConstants.java` (regenerated), `Constants.Drivetrain`. |
| `feature-robot-state` | `feature-drive` | M2 | `RobotState`, alliance flipping: `Constants.Field` (not the pass target), vision standard deviations and staleness. |
| `feature-turret` | `feature-drive` | M5 | The turret: every `Constants.Turret` value, including the intake-conflict arc. |
| `feature-intake` | `claude-test` | M5 | The intake's motors, homing and gains: `Constants.Intake` **except its angles** (see §3). |
| `feature-indexer` | `claude-test` | M5 | The indexer's motor and gains. |
| `feature-floor-roller` | `claude-test` | M5 | The floor roller. |
| `feature-flywheel` | `claude-test` | M4 | The flywheel's motors and gains, and its bench harness. |
| `feature-shooting-parameters` | `claude-test` | M3 | The solver: the shot tables in `Constants.Shot`. |
| `feature-limelight` | `feature-robot-state` | M7 | The intake Limelight's rejection ladder. |
| `integration-m5_m7` | the four M5 branches + M7 | — | Merge target only. |
| `feature-limelight-complete` | `integration-m5_m7` | M8 | The turret camera. |
| `feature-questnav` | `feature-limelight-complete` | M9 | QuestNav: `kRobotToQuest`, the Quest standard deviations. |
| `feature-cad-constants` | `feature-questnav` | — | The three CAD-measured camera mountings. Tuning them is done **here**, because this is where their lines were last written. |
| `integration-m1_m9` | cad-constants, flywheel, shooting-parameters | — | Merge target only. |
| `feature-superstructure` | `integration-m1_m9` | M10 | The superstructure: its constants, the pass target, the pipeline indices, and (§0) the M6 characterization code. |
| `feature-hopper` | `feature-superstructure` | M10b | The hopper model: every constant it added (feed-assist, conflict band, load and jam, shot detection, empty belief) **and all the intake angles**. |
| `feature-still-shot` | `feature-hopper` | M11 | The measurement burst (`Shot.kMeasureRpm`). |
| `feature-sotm` | `feature-still-shot` | M12 | Shooting on the move; no constants of its own. |
| `feature-autos` | `feature-sotm` | M13 | Autos: `Constants.Auto`, the paths, `settings.json`. **Deploy this one for Phases 1–7**: it has tuning mode on. |
| `feature-comp_ready` | `feature-autos` | M14 | The competition build, with `kTuningMode` **false**. `Constants.Power`, the roboRIO serials, `Constants.PracticeRobot` (the practice robot's encoder offsets), and the fault registrations. Newest of all, so every fan-out ends here. |

---

## 2. The one rule: run on the newest branch, commit on the origin branch

**Where you RUN is not where you COMMIT.**

- **Run** almost everything on `feature-autos`, with `Constants.kTuningMode` true, so every value is
  live-editable from AdvantageScope's `/Tuning` table. Exceptions are named in the steps: M1 and M2
  run on their own branches, M4 on `feature-flywheel`, and Phase 8 on `feature-comp_ready`, the
  competition build, which has tuning mode off and so cannot be tuned live.
- **Commit** each measured value on the branch listed for its constant in §3 — the branch where the
  subsystem was implemented, adjusted where the history requires — and then **merge that branch
  forward** into every branch descended from it, exactly as the plan documents are fanned out from
  `main`.

A value edited live in AdvantageScope is gone at the next reboot. Transcribe it into
`Constants.java` on its commit branch before you forget which number worked.

### Why this does not conflict — checked, not assumed

This was tested before this file was written, in a scratch repository holding every branch:

- All 198 tunable constants in `Constants.java` were edited on the branch §3 assigns them to. So
  was one line of `TunerConstants.java`, on `feature-drive`, standing in for a regeneration.
- Each edited branch was merged into every branch descended from it: 98 merges, and 112 once
  `feature-comp_ready` existed and was added as the newest descendant.
- **Result: no conflicts, and all 198 edits plus the `TunerConstants` change reached
  `feature-autos`, and `feature-comp_ready`.**
- A negative control proved the check could fail. A camera transform tuned on its original vision
  branch *does* conflict with `feature-cad-constants`, which rewrote that line.

The check has caught the same trap twice: a line added next to a tuned one.

- **The first run** found `Intake.kEjectAngleRad`. Tuned on `feature-intake`, it conflicts at
  `feature-hopper`, because M10b inserted its feed-assist constants on the very next line. That is
  §3's one exception: **all intake angles are tuned on `feature-hopper`**, the branch that gave the
  intake its final set of positions.
- **M14 made it again.** Its step 4 added a constant on the line after `Drivetrain.kCoastDelaySec`,
  which is tuned on `feature-drive`. Re-running the check with `feature-comp_ready` added showed
  nine conflicts, all cascading from that one line. The constant was moved, and the check is clean.

**What the check does not cover**, and what will break it:

1. **The constants rename.** `CONTEXT_AND_CONVENTIONS.md` §2 plans renaming every constant to
   `SCREAMING_SNAKE_CASE`. That rewrites every line tuned here, so every tuning merge after it
   would conflict. **Do the rename after bring-up**, as its own branch, once the values have
   settled.
2. **Code changes.** The check covered changed values on existing lines. A code fix found on the
   robot should go on the branch that owns the code. Before committing it, merge a scratch copy
   forward to see if it conflicts. M10b rewrote much of the intake, indexer and superstructure, so
   a fix to those on their M5 branches may well conflict.
3. **New constants.** Adding a line next to one that a later branch edited is the `kEjectAngleRad`
   case again. Add new constants on the newest branch that needs them.
4. **Editing a value directly on a later branch.** Don't. The origin branch would then carry a
   stale value forever, and the next person to tune it there would reintroduce it.

### The fan-out, after each tuning commit

The same loop as the documentation fan-out, run from `AdvantageKitRebuilt/`. `SRC` is the branch
you committed on, and `DESCENDANTS` lists its descendants from the map in §1:

```bash
SRC=feature-turret
DESCENDANTS="integration-m5_m7 feature-limelight-complete feature-questnav feature-cad-constants \
  integration-m1_m9 feature-superstructure feature-hopper feature-still-shot feature-sotm feature-autos \
  feature-comp_ready"
for d in $DESCENDANTS; do
  [ -n "$(git -C $d status --porcelain)" ] && { echo "DIRTY $d"; continue; }
  git -C $d fetch -q ../$SRC $SRC
  git -C $d -c user.name="Josh G" -c user.email="jfgindt@mtu.edu" \
      merge -q FETCH_HEAD -m "Pull the tuned <thing> from $SRC" || echo "CONFLICT $d"
done
```

Merging into the `integration-*` branches keeps them what they are: the combination of their
inputs, now with the inputs' tuned values. Nothing is ever *committed* on them directly.

---

## 3. Where each value is committed

| Values | Commit branch |
|---|---|
| `TunerConstants.java` (regenerate in Tuner X; never hand-edit) | `feature-drive` |
| `Drivetrain.*` (mass, MOI, wheel friction, heading gains, coast delay, auto-align) | `feature-drive` |
| `Field.*` except the pass target; `Vision.kMegaTag2*`, `Vision.kStalenessGrowthMetersPerSec` | `feature-robot-state` |
| `Turret.*` — every value, including `kIntakeConflictMinRad`/`MaxRad` | `feature-turret` |
| `Intake.*` motors, gains, gear ratios, encoder offset, homing (`kHomingDutyCycle`, `kHomingCurrentAmps`), `kPivotKg`, MOIs | `feature-intake` |
| `Intake.kStowedAngleRad`, `kDeployedAngleRad`, `kEjectAngleRad`, and everything M10b added: `kFeedAssist*`, `kTurretConflictBandRad` | `feature-hopper` |
| `Indexer.*` motor, gains, `kDetectionSettleSec`, `kEjectDurationSec` | `feature-indexer` |
| `Indexer.kLoaded*`, `kJam*`, `kUnjam*` | `feature-hopper` |
| `FloorRoller.*` | `feature-floor-roller` |
| `Shooter.*` motors, gains, follower direction, current limits | `feature-flywheel` |
| `Shooter.kBurstToleranceRpm`, `kShotDipRpm`, `kShotRefractorySec`, `kShotArmToleranceRpm` | `feature-hopper` |
| `Shot.*` tables and fit (`kDistanceBreakpointsMeters`, `kTimeOfFlightSec`, `kRpmPerMeterPerSec`, `kRpmIntercept`, `kAccelCompensationScalar`) | `feature-shooting-parameters` |
| `Shot.kMeasureRpm` | `feature-still-shot` |
| `Vision.kRobotToTurretAxis`, `kTurretToCamera`, `kRobotToIntakeCamera` (verify with a tag) | `feature-cad-constants` |
| `Vision.kRobotToQuest`, `Vision.kQuest*` | `feature-questnav` |
| `Superstructure.*` (M10's), `Field.kPassTarget*`, `Vision.k*PipelineIndex` | `feature-superstructure` |
| `Superstructure.kEmptyConfirmSec`; `Auto.kIntakeRetainTimeoutSec` | `feature-hopper` |
| `Auto.*` (the rest), the paths, `settings.json`, `navgrid.json` | `feature-autos` |
| `Power.*`, the roboRIO serial numbers, `Constants.PracticeRobot.*` (the practice robot's encoder offsets) | `feature-comp_ready` |
| SysId screenshots for `docs/` (M6's checkpoint) | the same branch as the gains they produced |

Both candidate branches for the shot tables were tested. `feature-shooting-parameters` (where the
solver was built) and `feature-still-shot` (where M11 measures them) both merge cleanly. The table
follows your rule and uses the branch where the solver was implemented.

---

## 4. The order, and why it is safe

The order follows one principle: **each step may only rely on what an earlier step has verified.**
Lowest energy first. Nothing moves under robot code until its direction has been seen by hand. No
mechanism with hard stops moves until those stops are known. No integrated branch is enabled until
everything its first enable does automatically is known to be safe.

That last point matters more than it looks. **On every branch from `feature-superstructure` on, the
first enable homes the intake and the turret by itself**, driving each into a hard stop. It also
deploys the intake. So Phases 1–3 exist to make that first enable of the integrated robot a
non-event.

### Phase 0 — No power: measure, count and settle (robot on the bench)

| # | Step | How | Commit to |
|---|---|---|---|
| 0.1 | **Settle the intake homing direction.** Open since M10b: `kHomingDutyCycle = -0.15` homes by *lowering* the arm, but the stowed position is the *top* of the travel. If both are true, homing seeds the stow angle at the deploy stop and every pivot angle is wrong by the full travel. Homing is the one intake motion the turret interlock does not guard. | Look at the mechanism: which stop is stow, and which motor direction drives towards it. Fix the sign in the phase 2 direction check once the motor's positive direction is known. | `feature-intake` |
| 0.2 | **Turret travel at the encoder.** If the turret's travel exceeds one encoder turn, a turret parked past ±180° at power-on can be seeded a full turn wrong, and the soft limits cannot catch it (`Turret.java`'s `TODO: MEASURE`). | Measure the mechanical travel. If it exceeds 360° at the encoder, raise it with the mechanical team **before** the turret is ever driven. | — |
| 0.3 | **Weigh the robot**, with battery and bumpers. | `Drivetrain.kRobotMassKg` | `feature-drive` |
| 0.4 | **Count every gear ratio.** | `Intake.kPivotGearRatio`, `kRollerGearRatio`, `FloorRoller.kGearRatio`, `Indexer.kGearRatio`, `Turret.kGearRatio`, `Shooter.kGearRatio`. The drive ratio goes into `TunerConstants` in 1.2. | each mechanism's branch (§3) |
| 0.5 | **From CAD**: moments of inertia (robot, intake pivot and roller, indexer, floor roller, turret, flywheel), the intake pivot length, and the intake's turret-conflict band edge. | `tools/onshape_measurements.py` for the band (add a mate connector at its edge). The MOIs feed the simulator and PathPlanner. | §3 |
| 0.6 | **The game piece**: mass (kitchen scale) and diameter (calipers). | `Field.kGamePieceMassKg`, `kGamePieceDiameterMeters`. The diameter switches on the intake camera's game-piece ranging. | `feature-robot-state` |
| 0.7 | **The pass target**: a tactical choice about the REBUILT field. | `Field.kPassTargetXMeters`, `kPassTargetYMeters` | `feature-superstructure` |

### Phase 1 — Powered, robot code disabled: inventory and senses

A disabled robot commands nothing, so deploying the newest branch and reading its logged inputs is
safe. Deploy `feature-autos` and leave the robot **disabled** throughout this phase.

| # | Step | Pass when | Commit to |
|---|---|---|---|
| 1.1 | **CAN inventory in Tuner X.** Every device answers at the ID in `Constants.Can` (mechanisms on the roboRIO bus, IDs 20–26) and the drivetrain's IDs on the CANivore. Note the CANivore's real name; the skeleton assumes `"canivore"`. | Nothing missing, nothing duplicated within a device type. (The same ID on two *different* device types is legal.) | — |
| 1.2 | **Regenerate `TunerConstants.java`** with Tuner X's swerve generator: module offsets, inversions, drive ratio, wheel radius, IDs, bus name. It is still the unregenerated skeleton on every branch. | The generator's own module checks pass. It is byte-identical on every branch, so it merges forward cleanly (checked, §2). | `feature-drive` |
| 1.3 | **Camera and headset connectivity.** Both Limelights publish, and QuestNav connects. Check `VisionIOLimelight`'s NetworkTables keys and botpose array layout against your firmware; the code carries a `TODO: VERIFY` for both. | `Vision/Inputs/*` populate for all three sources. | code fixes: `feature-limelight` / `feature-questnav` |
| 1.4 | **Sensor directions by hand.** Move each mechanism by hand and watch its logged position: turret (`Turret/Inputs/positionRad`), intake pivot (`Intake/Inputs/pivotPositionRad`). Positive must be the direction the code assumes. | Each sensor reads the documented positive direction. If not, fix the sensor inversion, never a sign in code. | `feature-turret` / `feature-intake` |
| 1.5 | **Absolute encoder offsets**: the intake pivot on its stow stop, and the turret at its zero. | `Intake.kPivotAbsEncoderOffsetRot`, `Turret.kAbsEncoderOffsetRot` | `feature-intake` / `feature-turret` |
| 1.6 | **Turret hard stops, by hand**: read the position at each stop, then back off by `kLimitMarginRad`. | `Turret.kMinAngleRad`, `kMaxAngleRad` | `feature-turret` |
| 1.7 | **Intake angles, by hand**: stow, deploy, eject, and the feed-assist height (raised over a full hopper until the pile just starts moving). | `Intake.kStowedAngleRad`, `kDeployedAngleRad`, `kEjectAngleRad`, `kFeedAssistAngleRad` | `feature-hopper` |
| 1.8 | **The turret's intake-conflict arc**, by hand, with the intake stowed. | `Turret.kIntakeConflictMinRad`, `kIntakeConflictMaxRad`; confirm `Intake.kTurretConflictBandRad` from 0.5 | `feature-turret`; the band on `feature-hopper` |
| 1.9 | **roboRIO serial numbers** for the comp and practice robots. Until they are entered, `feature-comp_ready` refuses to shoot (see 8.4); `feature-autos` does not check. | `kCompRoboRioSerial`, `kPracticeRoboRioSerial` | `feature-comp_ready` |

### Phase 2 — The drivetrain (M1, M2), and every direction seen before code drives it

| # | Step | Run on | Pass when (§10.2) | Commit to |
|---|---|---|---|---|
| 2.1 | **Mechanism directions and inversions**, one motor at a time at low output, from Tuner X with robot code disabled. (Check CTRE's documentation for how Tuner X control interacts with robot state on your firmware.) Order: floor roller, indexer, intake roller (no travel), then the turret and intake pivot, a few degrees each, well inside their stops. | Tuner X | Positive output moves each the way the code assumes. Fix it with `k*Inverted`, never with a sign in code: the turret's own `TODO: VERIFY` says why. | each mechanism's branch |
| 2.2 | **`Shooter.kFollowerOpposesLeader`**, before the flywheel is EVER spun under code: spin leader and follower separately at low output and compare against the gearbox. Wrong here means the pair fights itself at 80 A per motor. | Tuner X | Both wheels push a piece the same way. | `feature-flywheel` |
| 2.3 | **M1**: the drivetrain, on blocks and then on the ground. | `feature-drive` (merge 1.2 in first) | Drives field-relative; all four modules point the right way; no gyro alert. | `feature-drive` |
| 2.4 | **Robot MOI and wheel friction**, if not from CAD: spin test, and watch PathPlanner corners later. | `feature-drive` | — | `feature-drive` |
| 2.5 | **M2, the hard gate**: drive a 3 m square; `getPoseAt(now - 0.1)` returns a pose about 0.1 s behind. **Do not proceed until it passes**: the whole vision and shooting stack rests on it. | `feature-robot-state` | Closes within about 10 cm; speeds match a stopwatch; the lag is right. | `feature-robot-state` |

M1 and M2 run on their own branches deliberately. Those branches carry the drivetrain and
`RobotState` with none of the superstructure's automatic homing. Their mechanism classes are
pre-M5: they configure their motors, but no code path commands them.

### Phase 3 — Each mechanism under its own closed loop (M4, M5)

Lowest stored energy and no travel first; mechanisms with hard stops and interlocks last.

| # | Step | Run on | Pass when | Commit to |
|---|---|---|---|---|
| 3.1 | **M4, the flywheel**, by its bench toggle `/Tuning/Shooter/BenchSpinUp`. Also measure the steady-state supply current at `kMaxRpm`. | `feature-flywheel` (only the flywheel is alive) | Holds a commanded RPM within `kToleranceRpm`; editing `Shooter/Kp` changes the response live. | `feature-flywheel` (`kSupplyLowerLimitAmps`) |
| 3.2 | **The first enable of the integrated robot.** On blocks, hand on the disable, with Phase 1's angles and stops and step 0.1's homing direction committed and merged forward. Watch the boot homing: intake and turret each find their stop, the intake deploys. Then **measure the homing stall currents** (read the stator current in the stop, and set the threshold at about half). | `feature-autos` | Both homes finish; `Superstructure/SystemState` reaches `RESTING`; nothing hits anything. | `Intake.kHomingCurrentAmps` → `feature-intake`; `Turret.kHomingCurrentAmps` → `feature-turret` |
| 3.3 | **M5, each remaining mechanism under closed loop**, from the driver buttons, in the same order as 2.1. Then the turret unwrap check (§10.2's turret clause), once M6's `turretUnwrapDemo` exists. | `feature-autos` | Each moves under closed loop; every FSM state is reachable and logged. | each mechanism's branch |
| 3.4 | **Intake gravity**: at horizontal, in brake, find the open-loop voltage that just holds the arm. | `feature-autos` | `Intake.kPivotKg`, with its sign | `feature-intake` |
| 3.5 | **Indexer spin-up duration** from the logged inputs, and **eject duration** from a real piece. | `feature-autos` | `Indexer.kDetectionSettleSec`, `kEjectDurationSec` | `feature-indexer` |

### Phase 4 — Characterization (M6)

Needs §0's M6 code. **Measure before tuning.** Every placeholder gain is visibly wrong in
simulation (the turret and intake pivot ring for about eight seconds, and the flywheel's `kV`
under-predicts), and tuning `kP` around a wrong feedforward hides the error rather than fixing it.

| # | Step | Pass when (§10.2 M6) | Commit to |
|---|---|---|---|
| 4.1 | **Drivetrain SysId** and the **wheel-radius** routine. | Measured drive gains; the measured wheel radius is in both `TunerConstants` and `settings.json`, and they agree. | `feature-drive` (regenerate `TunerConstants`); `settings.json` on `feature-autos` |
| 4.2 | **Flywheel staircase** (`kS`, `kV` at true steady state), then `kP`. Revisit the M4 note that a low `kP` settling short points at `kV`. | Measured `kS`/`kV`; screenshot in `docs/`. | `feature-flywheel` |
| 4.3 | **Floor roller and indexer** staircases. | Measured `kS`/`kV`. | their branches |
| 4.4 | **Turret SysId**, every leg bounded by the soft limits: the one routine where a missing limit breaks hardware. Then `kP`/`kD`, and the ring is gone. | Measured `kS`/`kV`/`kA`; settles a 180° move without ringing. | `feature-turret` |
| 4.5 | **Intake pivot SysId, analysed as an Arm** (so `kG` is fitted apart from `kS`). | Measured `kS`/`kV`/`kA`/`kG`. | `feature-intake` |

### Phase 5 — The integrated robot: interlocks, hopper and vision (M10, M10b, M7–M9)

All run on `feature-autos`.

| # | Step | Pass when | Commit to |
|---|---|---|---|
| 5.1 | **M10's interlock, both directions, on real geometry.** Deliberately deploy the intake into the turret's arc, and aim the turret into a stowed intake. | Every driver button produces its state with nothing else moving; both interlock directions hold; `Superstructure/ReadyTerms/*` are separately logged. | code fixes on `feature-superstructure` |
| 5.2 | **The hopper (M10b)**: the conflict band's exact edge; the feed-assist rocking period and roller speed; indexer loaded current (full and empty hopper); jam current (stall it on purpose), detect and unjam times. | Feed-assist rocks without stalling the flow; a deliberate jam is recovered; a full and an empty hopper read differently. | `feature-hopper` |
| 5.3 | **Shot detection** from a logged burst: dip size, dip width, arm tolerance, and the longest gap between detected shots. | `Shooter/ShotCount` matches a hand count within one per burst; `kEmptyConfirmSec` exceeds the longest gap. | `feature-hopper` |
| 5.4 | **M7, the intake Limelight**, at a surveyed point: cover and uncover it, and provoke each rejection. Then the MegaTag2 standard deviations. | The pose converges within ±5 cm; every rejection counter fires at least once. | `feature-limelight` (code); `Vision.kMegaTag2*` → `feature-robot-state` |
| 5.5 | **M8, the turret camera — the highest-value single test in the project.** Stationary, tag in view, slew the turret 180°. | `RobotState/EstimatedPose` moves no more than a couple of centimetres. If it swings, verify the CAD transforms with a tag first. | transforms → `feature-cad-constants` |
| 5.6 | **M9, QuestNav**: measure `kRobotToQuest` to the centre of the headset's front face; seed against a tag, blind both Limelights, push 3 m; power-cycle the headset mid-run. | The pose tracks; `Vision/QuestTrusted` goes false and the pose degrades to odometry without jumping. `kRobotToQuest` is the worst transform to get wrong: it wins the fusion. | `feature-questnav` |

### Phase 6 — Shooting (M11, M12)

| # | Step | Run on | Pass when | Commit to |
|---|---|---|---|---|
| 6.1 | **M11's shot tables**, at 8–10 taped distances, by `/Tuning/ShotTable/Measure` bursts. The full procedure is in `PROJECT_OVERVIEW.md` ("Running the M11 checkpoint on the robot"). Fit with `tools/fit_shot_table.py`. | `feature-autos` (it has M12's stutter fix) | 8 of 10 from three distances with `SCORE_STATIONARY`. | `feature-shooting-parameters` |
| 6.2 | **Stationary-shot speed gate and burst band**, tuned against the hit rate. | `feature-autos` | — | `kMaxChassisSpeedForStationaryShot…` → `feature-superstructure`; `kBurstToleranceRpm` → `feature-hopper` |
| 6.3 | **M12, shooting on the move**: constant-velocity strafing first with `kAccelCompensationScalar = 0`, then raise it in 0.25 steps. | `feature-autos` | 7 of 10 at 2 m/s or more; `Superstructure/Shot/VirtualGoal` leads correctly. | `feature-shooting-parameters` |
| 6.4 | **The pass**: `kPassRpm` for the chosen target. Measuring it also makes M10's surviving mutant killable (see `SuperstructureFsmTest.passingDoesNotWaitOnAShotSolution`). | `feature-autos` | Passes land at the target. | `feature-superstructure` |

### Phase 7 — Autos (M13)

Needs §0's paths. In §8.6's order, never skipping step 3.

| # | Step | Pass when | Commit to |
|---|---|---|---|
| 7.1 | Place the robot until `AutoSelector/StartPoseError` reads under 5 cm; move it 30 cm and see the warning. | The warning fires off the start and clears on it. | `feature-autos` |
| 7.2 | **Leave**. Tune `Auto.kTranslationKp`/`kRotationKp` against `Odometry/TrajectorySetpoint`. | Ends where the path does. | `feature-autos` |
| 7.3 | **Preload Only.** Time enable to the eighth shot. | `Auto/PreloadFired` reads 8. | `kPreloadTimeoutSec` → `feature-autos` |
| 7.4 | **Center 3 (stationary)** (tuning mode), then **Center 3**, then **Center 5**. If the moving one fails after the stationary one passed, the fault is in the moving-shot math. | Each runs to its end inside 15 s. | `feature-autos` |
| 7.5 | **Scoring and retain timeouts** from real runs: the time to fire 50 pieces, and stow-to-feed-assist. | — | `kScoreTimeoutSec` → `feature-autos`; `kIntakeRetainTimeoutSec` → `feature-hopper` |
| 7.6 | **Every routine on red**, forced in the Driver Station (the plan's risk #4, already passing in simulation). | Red mirrors blue. | — |

### Phase 8 — Competition hardening (M14)

On `feature-comp_ready`, the competition build, after every tuned value from Phases 1–7 has been
merged forward into it. Tuning mode is off there, so this phase measures and verifies; it does not
tune. `docs/PIT_CHECKLIST.md` on that branch is the pre-match routine these steps prove out.

The simulation halves already pass as tests, which is what each hardware step now has to confirm:
`DeviceFaultTest` unplugs each of 23 devices alone, `PowerBudgetTest` sags the battery until the
intake stops rocking, `CoastReleaseTest`, and `RobotIdentityTest`.

| # | Step | Pass when (§10.2 M14) |
|---|---|---|
| 8.1 | Unplug each CAN device in turn, including the Pigeon2, the one device the simulation could not unplug. | The dashboard names it, with its CAN ID and bus, within a second. `Faults/Blocking` lists it only for the flywheel leader, the flywheel follower and the turret motor. |
| 8.2 | A full 2:30 match in practice; then the combined-mode run: a figure-eight while intaking and firing for 20 s. Set `Constants.Power` from the logs. | The battery ends above `kBrownoutWarningVolts`; no brownout in `Power/FilteredVoltage`; `Power/Scalar/LOW` is the first to drop, and the intake stops rocking while it is below 1. |
| 8.3 | A full re-test with `kTuningMode` false, already so on this branch. | Everything above still passes with nothing live-editable. |
| 8.4a | **The coast release.** Disable after a drive; wait. | About 5 s after disable, with the robot still, `Drive/BrakeMode` goes false and the robot pushes freely; the intake and turret stay in brake. Every enable restores brake. |
| 8.4 | Practice-bot identity (§5.5). **Enter both serials first (step 1.9)**: `getRobot()` raises a *blocking* error for an unknown roboRIO, so an unentered serial means the robot will not shoot. | Each robot resolves to itself, with no "Unknown roboRIO" alert, and the practice robot reads `Constants.PracticeRobot`'s offsets. |

---

## 5. After bring-up

- **The constants rename** (`CONTEXT_AND_CONVENTIONS.md` §2), now that the values have settled, as
  its own branch.
- **Full-system integration onto `claude-test`**, which is reserved for exactly that.
- Record each phase in `PROJECT_OVERVIEW.md` the way the simulation milestones were recorded. On
  this project, a hardware checkpoint counts as passed when someone has seen it fail for the reason
  the test exists (a covered camera, an unplugged device, a deliberately stalled indexer), not only
  pass.
