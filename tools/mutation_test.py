#!/usr/bin/env python3
"""Mutation harness.

`PROJECT_OVERVIEW.md` makes mutation testing part of this project's testing standard:
a suite whose author also wrote the code proves nothing until it has been broken on
purpose. Every milestone so far re-derived a one-off script for that. This is the same
thing kept, so the score is reproducible by somebody who did not write the tests.

    python3 tools/mutation_test.py

Each entry injects one deliberate defect, runs the suite, reports whether a test
noticed, and reverts. Three rules from the standard are ENFORCED here rather than
trusted, because each has cost this project real time:

  * The tree must be clean before anything runs. The revert is `git checkout --`, which
    cannot tell a mutation from work you have not committed. Doing this to uncommitted
    M7 code destroyed an afternoon of it.

  * The substitution must be proven to have landed, by requiring the anchor to match
    exactly once and reading the file back afterwards. A mutation whose anchor silently
    fails to match reports a perfect score for doing nothing, which is worse than no
    mutation testing at all because it is actively reassuring. This harness reports
    those as BAD ANCHOR and excludes them from the score rather than counting them as
    kills -- and it has already caught two of its own, after a line they targeted was
    rewritten.

  * A surviving mutation is a QUESTION, not a verdict. Check the mutation was reachable
    before concluding the tests are at fault. One M9 "survivor" was a defective mutation
    aimed at dead code, and M10 has one genuinely equivalent mutant, recorded below.

THE MUTATION LIST IS PER MILESTONE AND SHOULD BE EXTENDED, NOT REPLACED. The first
block targets M10: the superstructure's guards, its entry and exit actions, both halves
of the intake/turret interlock, the readiness gate, FaultReporter's severity handling,
and the two latent bugs M10 found in already-merged code. The second block is M10b's,
growing one retrofit step at a time: step 2 added the flywheel shot detector, step 3
the intake's resting and feed-assist states and its two geometric predicates, step 4
the intake-camera gate, step 5 both halves of the corrected interlock,
step 6 the indexer's jam recovery and load reading, step 7 the superstructure switchover.
Step 8 deleted the single-piece model and added no defects of its own -- a deletion has
no new behaviour to break -- but it rewrote the piece-guard entry, which named the
deleted hasPiece(). Step 9 added the autos' one runnable hopper-era block, the wait
for the intake to retain pieces. Step 10 is the pass the retrofit order asked for: subtler
defects in the four places a plausible bug survives happy-path tests (the empty clock,
the burst latch, the band's direction, the floor roller leading), plus the misconfigured
feed-assist guard, which had none.

A third block is M11's: the table-free measurement shot, the override that
selects it inside SCORE_STATIONARY and the one rule that clears it, and the
measurement burst's end condition and the numbers it reports.

A fourth is M12's: the flywheel projection's saturation (the stutter fix), the
turret feedforward's chassis-rate sign, and the moving shot itself -- including
the double-counted velocity M12's simulated checkpoint found in the solver, put
back.

A sixth is M14's: the fault registry's loud-versus-blocking split and its
one-entry-per-name rule, the blocking set of devices, PowerBudget's tier
ordering, the coast release, and the comp/practice identity.

A fifth is M13's, one group per step: the settings.json check, the synced
alliance flip, the loud handling of a missing file, the marker vocabulary, the
leg-scoped requests of the building blocks (with the plan's own forms put back),
the routines, and the selector's preview. Several M10 anchors name code that later M10b steps replace (the
piece guard, the isStowed turret guard), and those entries must be rewritten against
the new code in the step that changes it, not deleted. Adding a milestone means adding its
defects to the list and re-running the whole thing, so an old mutation cannot quietly
stop being caught.

KNOWN EQUIVALENT MUTANT. "pass gate borrows the shot gate" survives, and correctly so.
readyToPass and readyToScore differ only when every non-solution term is true while the
solution term is false, and the non-solution terms cannot all be true while
Constants.Superstructure.kPassRpm is zero -- the flywheel term needs a non-zero goal. No
test can tell the two apart today. Measuring kPassRpm makes the difference observable;
the note in SuperstructureFsmTest.passingDoesNotWaitOnAShotSolution says what to add
when it does. It is listed rather than deleted so that the score reads 26/27 with a
stated reason, instead of 26/26 with a mutation quietly removed for failing to die.
"""
import subprocess, sys, os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SS = "src/main/java/frc/robot/subsystems/superstructure/Superstructure.java"
TU = "src/main/java/frc/robot/shooting/TurretUnwrapper.java"
FR = "src/main/java/frc/robot/util/FaultReporter.java"
IN = "src/main/java/frc/robot/subsystems/intake/Intake.java"
AF = "src/main/java/frc/robot/autos/AutoFactory.java"
AS = "src/main/java/frc/robot/autos/AutoSelector.java"
SH = "src/main/java/frc/robot/subsystems/shooter/Shooter.java"
IX = "src/main/java/frc/robot/subsystems/indexer/Indexer.java"
AR = "src/main/java/frc/robot/autos/AutoRoutines.java"
SV = "src/main/java/frc/robot/shooting/ShootingSolver.java"
CC = "src/main/java/frc/robot/commands/CharacterizationCommands.java"
TR = "src/main/java/frc/robot/subsystems/turret/Turret.java"
VI = "src/main/java/frc/robot/subsystems/vision/Vision.java"
MO = "src/main/java/frc/robot/subsystems/drive/Module.java"
PB = "src/main/java/frc/robot/util/PowerBudget.java"
CR = "src/main/java/frc/robot/util/CoastRelease.java"
CO = "src/main/java/frc/robot/Constants.java"

MUTATIONS = [
    # Re-anchored in M10b step 7, which removed the piece half of this guard (plan section 2.4). The
    # first entry is the fault guard's deletion as before; the second is the reverse question --
    # the piece guard coming back. Step 7 wrote it against indexer.hasPiece(); step 8 deleted that
    # method, and a mutation that no longer compiles would "die" to the compiler and prove nothing.
    # So it is rewritten against isLoaded(), which the constants call the hopper-era replacement
    # for "has a piece" and which is the guard a well-meaning edit would reach for. It gates
    # nothing by design: gating on it would mean never scoring, since it is only true while
    # feeding.
    ("critical-fault guard deleted", SS,
     "    if (FaultReporter.getInstance().anyCritical()) {\n      return SystemState.SPINNING_UP;\n    }",
     "    if (false) {\n      return SystemState.SPINNING_UP;\n    }"),
    ("piece guard re-added as a load guard, reverting step 7's decision", SS,
     "    if (FaultReporter.getInstance().anyCritical()) {\n      return SystemState.SPINNING_UP;",
     "    if (!indexer.isLoaded() || FaultReporter.getInstance().anyCritical()) {\n      return SystemState.SPINNING_UP;"),
    ("usable-solution guard deleted", SS,
     "    if (!lastShot.isUsable() || turret.isOutOfRange()) {",
     "    if (turret.isOutOfRange()) {"),
    ("out-of-range guard deleted", SS,
     "    if (!lastShot.isUsable() || turret.isOutOfRange()) {",
     "    if (!lastShot.isUsable()) {"),
    ("forced scoring no longer skips the aim guards", SS,
     "    if (requested == SystemState.SCORING_FORCED) {\n      return SystemState.SCORING_FORCED;\n    }",
     "    if (false) {\n      return SystemState.SCORING_FORCED;\n    }"),
    ("HOME re-issued to the mechanisms every loop", SS,
     "    if (homeRequestPending) {\n      intake.setWantedState(Intake.WantedState.HOME);\n      turret.setWantedState(Turret.WantedState.HOME);\n      homeRequestPending = false;\n    }",
     "    intake.setWantedState(Intake.WantedState.HOME);\n    turret.setWantedState(Turret.WantedState.HOME);"),
    ("fault latch deleted", SS,
     "    if (systemState == SystemState.FAULTED) {\n      return SystemState.FAULTED;\n    }",
     "    if (false) {\n      return SystemState.FAULTED;\n    }"),
    ("homing timeout never fires", SS,
     "    if (systemState == SystemState.HOMING\n        && stateTimer.hasElapsed(Constants.Superstructure.kHomingTimeoutSec.get())) {",
     "    if (false) {"),
    ("homing waits on the turret only", SS,
     "    if (intake.isHomed() && turret.isHomed()) {",
     "    if (turret.isHomed()) {"),
    ("boot entry action removed from the constructor", SS,
     "    onEnter(systemState);", "    // onEnter(systemState);"),
    ("leaving intaking no longer restores the tag pipeline", SS,
     "    } else if (next != IntakeMode.INTAKING && intakeMode == IntakeMode.INTAKING) {\n      vision.setIntakePipeline(Constants.Vision.kAprilTagPipelineIndex);",
     "    } else if (false) {\n      vision.setIntakePipeline(Constants.Vision.kAprilTagPipelineIndex);"),
    ("force-shoot override ignored by the gate", SS,
     "    readyToScore = ignoreShotTolerances || (shotUsable && everythingButTheSolution);",
     "    readyToScore = shotUsable && everythingButTheSolution;"),
    ("gate vacuously open", SS,
     "    readyToScore = ignoreShotTolerances || (shotUsable && everythingButTheSolution);",
     "    readyToScore = true;"),
    ("pass gate borrows the shot gate, solution term and all", SS,
     "    readyToPass = ignoreShotTolerances || everythingButTheSolution;",
     "    readyToPass = readyToScore;"),
    ("intake deploy guard always open", SS,
     "    intake.setDeployGuard(() -> !turretInIntakeConflictArc());",
     "    intake.setDeployGuard(() -> true);"),
    ("turret intake-arc guard never blocks", SS,
     "    turret.setIntakeArcGuard(intake::isInTurretConflictBand);",
     "    turret.setIntakeArcGuard(() -> false);"),
    ("forced scoring holds instead of feeding", SS,
     "      case SCORING_FORCED -> true;",
     "      case SCORING_FORCED -> false;"),
    ("forced-shot count raised on entry to any scoring state", SS,
     "        if (state == SystemState.SCORING_FORCED) {\n          forcedShotCount++;",
     "        if (true) {\n          forcedShotCount++;"),
    # Re-anchored in M11: step 2 put the shot-table override's note between the two statements this
    # entry used to span, and the entry then matched nothing -- reported as BAD ANCHOR, not scored.
    ("disable does not clear the force-shoot override", SS,
     "    ignoreShotTolerances = false;\n\n    // The shot-table override (M11 step 2)",
     "    // The shot-table override (M11 step 2)"),
    ("disable does not park the machine", SS,
     "    setWantedState(WantedState.IDLE);\n  }",
     "  }"),
    ("STOWED coasts the flywheel instead of idling it", SS,
     "    turret.setWantedState(Turret.WantedState.STOW);\n    shooter.setWantedState(Shooter.WantedState.IDLE);\n  }",
     "    turret.setWantedState(Turret.WantedState.STOW);\n    shooter.setWantedState(Shooter.WantedState.COAST);\n  }"),
    ("IDLE idles the flywheel instead of coasting it", SS,
     "    turret.setWantedState(Turret.WantedState.IDLE);\n    shooter.setWantedState(Shooter.WantedState.COAST);\n  }\n\n  /** Establishes trustworthy zeros",
     "    turret.setWantedState(Turret.WantedState.IDLE);\n    shooter.setWantedState(Shooter.WantedState.IDLE);\n  }\n\n  /** Establishes trustworthy zeros"),
    ("wrapped arc check reverted to the raw interval test", TU,
     "    for (int turns = lowestTurn; turns <= highestTurn; turns++) {",
     "    for (int turns = 0; turns <= 0; turns++) {"),
    ("wrapped arc check fails OPEN on nonsense input", TU,
     "        || !Double.isFinite(arcMax)) {\n      return true;",
     "        || !Double.isFinite(arcMax)) {\n      return false;"),
    # Both re-anchored in M14 step 1, which made anyCritical() read a fault's Impact rather than its
    # alert type. The second now means "any fault blocks", which is what it always tested.
    ("anyCritical() always reports healthy", FR,
     "    return faults.stream()\n        .anyMatch(\n            fault -> fault.impact() == Impact.BLOCKS_SCORING && fault.isFaulted().getAsBoolean());",
     "    return false;"),
    ("severity ignored, any fault counts as critical", FR,
     "            fault -> fault.impact() == Impact.BLOCKS_SCORING && fault.isFaulted().getAsBoolean());",
     "            fault -> fault.isFaulted().getAsBoolean());"),
    ("intake home-latch fix reverted", IN,
     "      homeRequested = false;\n\n      if (homingSucceeded) {",
     "      if (homingSucceeded) {"),

    # ---- M10b step 2: the flywheel shot detector (plan section 2.4). Each of the detector's four
    # conditions, its refractory window, its ordering against the acceleration update, and the
    # accessors the superstructure's "probably empty" belief will be built on.
    ("shot detector counts in any state", SH,
     "    boolean holdingGoal =\n        (systemState == SystemState.SPINNING_UP || systemState == SystemState.AT_SPEED)\n            && goalRpm > 0.0;",
     "    boolean holdingGoal = goalRpm > 0.0;"),
    ("shot detector never needs arming", SH,
     "    boolean armed =\n        Math.abs(lastVelocityRpm - goalRpm) < Constants.Shooter.kShotArmToleranceRpm.get();",
     "    boolean armed = true;"),
    ("shot detector ignores the dip threshold", SH,
     "    boolean sharpDrop = dropRpm > Constants.Shooter.kShotDipRpm.get();",
     "    boolean sharpDrop = dropRpm > 0.0;"),
    ("shot detector drops the ends-below-goal check", SH,
     "    boolean endedBelowGoal = inputs.velocityRpm < goalRpm;",
     "    boolean endedBelowGoal = true;"),
    ("shot detector refractory window deleted", SH,
     "    boolean outsideRefractory =\n        !hasShotSinceReset || sinceLastShot.hasElapsed(Constants.Shooter.kShotRefractorySec.get());",
     "    boolean outsideRefractory = true;"),
    ("shot detection runs after the acceleration update", SH,
     "    detectShot();\n    updateAccelerationEstimate();",
     "    updateAccelerationEstimate();\n    detectShot();"),
    ("time since last shot reads the raw timer before any shot", SH,
     "    return hasShotSinceReset ? sinceLastShot.get() : Double.POSITIVE_INFINITY;",
     "    return sinceLastShot.get();"),
    ("reset keeps the last-shot time", SH,
     "    shotCount = 0;\n    hasShotSinceReset = false;",
     "    shotCount = 0;"),
    ("burst overload ignores the tolerance it was given", SH,
     "    return Math.abs(projectedRpm - goalRpm) < toleranceRpm && goalRpm > 0.0;",
     "    return Math.abs(projectedRpm - goalRpm) < Constants.Shooter.kToleranceRpm.get()\n        && goalRpm > 0.0;"),

    # ---- M10b step 3: the intake's resting and feed-assist states, and its two geometric
    # predicates. The arrival fix is attacked where it bites -- judging arrival against the previous
    # loop's goal again -- and every predicate is attacked on its fail-closed branch as well as its
    # main one, because a predicate that fails open near the turret is the expensive kind of wrong.
    ("DEPLOY_ONLY never settles into DEPLOYED", IN,
     "          deployed, atAngle(deployed) ? SystemState.DEPLOYED : SystemState.DEPLOYING);",
     "          deployed, SystemState.DEPLOYING);"),
    ("arrival judged against the previous loop's goal again", IN,
     "          deployed, atAngle(deployed) ? SystemState.INTAKING : SystemState.DEPLOYING);",
     "          deployed, atGoal() ? SystemState.INTAKING : SystemState.DEPLOYING);"),
    ("intake guard refuses nothing, even a stow beside the turret", IN,
     "    if (entersBand && !deployAllowed.getAsBoolean()) {",
     "    if (false) {"),
    ("feed-assist roller does not run", IN,
     "    io.setRollerVelocity(Constants.Intake.kFeedAssistRollerRpm.get());",
     "    io.setRollerVelocity(0.0);"),
    ("rock never alternates", IN,
     "    return halfPeriodsElapsed % 2 == 0 ? feedAssistRad : deployedRad;",
     "    return feedAssistRad;"),
    ("rock ignores the low-power fallback", IN,
     "    if (periodSec <= 0.0 || !rockingAllowed) {",
     "    if (periodSec <= 0.0) {"),
    ("conflict band fails OPEN on an unhomed arm", IN,
     "    if (!isHomed) {\n      return true;\n    }",
     "    if (!isHomed) {\n      return false;\n    }"),
    ("conflict band ignores the commanded goal", IN,
     "    return inTurretConflictBand(inputs.pivotPositionRad, stowed, deployed, width)\n        || inTurretConflictBand(pivotGoalRad, stowed, deployed, width);",
     "    return inTurretConflictBand(inputs.pivotPositionRad, stowed, deployed, width);"),
    ("conflict band on the wrong side when stow is below deploy", IN,
     "      return angleRad <= stowedRad + widthRad;",
     "      return angleRad >= stowedRad + widthRad;"),
    ("conflict band fails OPEN on degenerate geometry", IN,
     "    return Math.abs(angleRad - stowedRad) <= widthRad;",
     "    return false;"),
    ("retaining pieces trusts an unhomed encoder", IN,
     "    return isHomed\n        && withinRetainingRange(",
     "    return withinRetainingRange("),

    # ---- M10b step 4: the intake-camera gate. Each of its ways to be wrong: narrowed back to the
    # M10 gate (the silent failure -- the camera switched off while resting), widened to the rock or
    # to transit, and turned into a position test, which the rock's pass through the deploy angle
    # exists to catch. The RobotContainer wiring itself is NOT mutated: nothing constructs
    # RobotContainer in a test, so a mutation there could not be killed and would only be noise.
    ("camera gate narrowed back to INTAKING only", IN,
     "    return systemState == SystemState.INTAKING || systemState == SystemState.DEPLOYED;",
     "    return systemState == SystemState.INTAKING;"),
    ("camera gate admits the feed-assist rock", IN,
     "    return systemState == SystemState.INTAKING || systemState == SystemState.DEPLOYED;",
     "    return systemState == SystemState.INTAKING\n        || systemState == SystemState.DEPLOYED\n        || systemState == SystemState.FEED_ASSISTING;"),
    ("camera gate admits transit", IN,
     "    return systemState == SystemState.INTAKING || systemState == SystemState.DEPLOYED;",
     "    return systemState == SystemState.INTAKING\n        || systemState == SystemState.DEPLOYED\n        || systemState == SystemState.DEPLOYING;"),
    ("camera gate tests position instead of state", IN,
     "    return systemState == SystemState.INTAKING || systemState == SystemState.DEPLOYED;",
     "    return atAngle(geometry.deployedRad().getAsDouble());"),

    # ---- M10b step 5: the interlock correction, both halves. The turret half reverted to M10's
    # backwards rule, which the new negative case -- a DEPLOYED intake must not block the turret --
    # exists to catch. The intake half made to refuse leaving the band, made to fall back to
    # stowing as M10's did, and its wait broken two ways: a hold that does not hold, and a hold
    # angle that was never latched.
    ("turret guard reverted to M10's backwards rule", SS,
     "    turret.setIntakeArcGuard(intake::isInTurretConflictBand);",
     "    turret.setIntakeArcGuard(() -> !intake.isStowed());"),
    ("intake guard also refuses moves out of the band", IN,
     "    boolean entersBand =\n        !inTurretConflictBand(inputs.pivotPositionRad, stowed, deployed, width)\n            && inTurretConflictBand(targetRad, stowed, deployed, width);",
     "    boolean entersBand = true;"),
    ("a refused stow falls back to stowing, as M10's guard did", IN,
     "    if (entersBand && !deployAllowed.getAsBoolean()) {\n      return SystemState.WAITING_FOR_TURRET;",
     "    if (entersBand && !deployAllowed.getAsBoolean()) {\n      return SystemState.STOWING;"),
    ("the wait drives on to stow instead of holding", IN,
     "    pivotGoalRad = waitHoldRad;",
     "    pivotGoalRad = geometry.stowedRad().getAsDouble();"),
    ("the wait's hold angle is never latched", IN,
     "      case WAITING_FOR_TURRET -> waitHoldRad = inputs.pivotPositionRad;",
     "      case WAITING_FOR_TURRET -> {}"),

    # ---- M10b step 6: the indexer's jam recovery and load reading. The load reading attacked on
    # each of its guards (the state test at the accessor, the settle window, the current
    # threshold); the unjam policy on each of its bounds (the attempt limit, the pulse length, the
    # latch, the per-request reset) and on what it commands.
    ("loaded leaks out of FEEDING through the debounce", IX,
     "    return loaded && systemState == SystemState.FEEDING;",
     "    return loaded;"),
    ("load and jam ignore the spin-up settle window", IX,
     "        systemState == SystemState.FEEDING\n            && stateTimer.hasElapsed(Constants.Indexer.kDetectionSettleSec.get());",
     "        systemState == SystemState.FEEDING;"),
    ("loaded ignores the current threshold", IX,
     "        measuring && inputs.statorCurrentAmps > Constants.Indexer.kLoadedCurrentAmps.get();",
     "        measuring;"),
    ("unjam attempts unbounded", IX,
     "      if (unjamAttempts >= Constants.Indexer.kMaxUnjamAttempts) {",
     "      if (false) {"),
    ("an unjam pulse never ends", IX,
     "      return stateTimer.hasElapsed(Constants.Indexer.kUnjamSec.get())\n          ? SystemState.FEEDING\n          : SystemState.UNJAMMING;",
     "      return SystemState.UNJAMMING;"),
    ("JAMMED latch released while FEED is still requested", IX,
     "    if (systemState == SystemState.JAMMED) {\n      return SystemState.JAMMED;\n    }",
     "    if (false) {\n      return SystemState.JAMMED;\n    }"),
    ("a new FEED request inherits the spent unjam allowance", IX,
     "      unjamAttempts = 0;\n    }",
     "    }"),
    ("an unjam pulse pushes forward instead of reversing", IX,
     "      case UNJAMMING -> commandVelocity(Constants.Indexer.kUnjamRpm.get());",
     "      case UNJAMMING -> commandVelocity(Constants.Indexer.kFeedRpm.get());"),

    # ---- M10b step 7: the superstructure switchover. The intake region (request, posture,
    # feed-assist), the feed side (floor roller leads, indexer gates, reverse with an unjam), the
    # burst latch in both directions, the empty belief on each of its conditions and resets, the
    # auto command that waits on it, and the two posture writers (first enable, STOW). The
    # RobotContainer bindings and the Robot hooks are NOT mutated: nothing constructs either in a
    # test, so no mutation there could be killed.
    ("feed side commanded in the override states too", SS,
     "    if (!isOverrideState(systemState)) {\n      applyFeedSide();\n    }",
     "    applyFeedSide();"),
    ("intake request ignored", SS,
     "    } else if (intakeRequested) {",
     "    } else if (false) {"),
    ("an intake request does not deploy the posture", SS,
     "      intakePosture = IntakePosture.DEPLOYED;\n      next = IntakeMode.INTAKING;",
     "      next = IntakeMode.INTAKING;"),
    ("the intake rests stowed whatever its posture", SS,
     "    IntakeMode resting =\n        intakePosture == IntakePosture.DEPLOYED ? IntakeMode.DEPLOYED : IntakeMode.STOWED;",
     "    IntakeMode resting = IntakeMode.STOWED;"),
    ("feed-assist ignores the gate", SS,
     "    } else if (feedOpen && intakePosture == IntakePosture.DEPLOYED) {",
     "    } else if (intakePosture == IntakePosture.DEPLOYED) {"),
    ("floor roller waits for the gate instead of leading", SS,
     "    } else if (shotRequested) {\n      roller = FloorRoller.WantedState.FEED;",
     "    } else if (feedOpen) {\n      roller = FloorRoller.WantedState.FEED;"),
    ("floor roller runs for intaking alone", SS,
     "    } else if (shotRequested) {\n      roller = FloorRoller.WantedState.FEED;",
     "    } else if (shotRequested || intakeMode == IntakeMode.INTAKING) {\n      roller = FloorRoller.WantedState.FEED;"),
    ("floor roller does not reverse with an unjam", SS,
     "    if (indexer.isUnjamming()) {",
     "    if (false) {"),
    ("indexer feeds on the request, not the gate", SS,
     "    indexer.setWantedState(feedOpen ? Indexer.WantedState.FEED : Indexer.WantedState.IDLE);",
     "    indexer.setWantedState(shotRequested ? Indexer.WantedState.FEED : Indexer.WantedState.IDLE);"),
    ("burst latch never sets", SS,
     "    } else if (gateOpenForThisState) {\n      burstLatched = true;",
     "    } else if (gateOpenForThisState) {\n      burstLatched = false;"),
    ("burst latch never clears", SS,
     "    if (!burstState || !otherTermsHold) {\n      burstLatched = false;",
     "    if (false) {\n      burstLatched = false;"),
    ("empty clock counts any unloaded time, gate or no gate", SS,
     "    } else if (feedOpen\n        && indexer.getSystemState() == Indexer.SystemState.FEEDING\n        && !indexer.isLoaded()) {",
     "    } else if (!indexer.isLoaded()) {"),
    ("empty clock runs through an unjam pulse", SS,
     "        && indexer.getSystemState() == Indexer.SystemState.FEEDING\n        && !indexer.isLoaded()) {",
     "        && !indexer.isJammed()\n        && !indexer.isLoaded()) {"),
    ("a load does not hold off the empty clock", SS,
     "        && indexer.getSystemState() == Indexer.SystemState.FEEDING\n        && !indexer.isLoaded()) {",
     "        && indexer.getSystemState() == Indexer.SystemState.FEEDING) {"),
    ("a shot does not reset the empty clock", SS,
     "    if (newShot || intakeMode == IntakeMode.INTAKING) {",
     "    if (intakeMode == IntakeMode.INTAKING) {"),
    ("intaking does not reset the empty clock", SS,
     "    if (newShot || intakeMode == IntakeMode.INTAKING) {",
     "    if (newShot) {"),
    ("likelyEmpty ignores kEmptyConfirmSec", SS,
     "    return unproductiveFeedSec >= Constants.Superstructure.kEmptyConfirmSec.get();",
     "    return unproductiveFeedSec > 0.0;"),
    ("scoreUntilEmpty ignores the empty belief", SS,
     "                Commands.waitUntil(this::likelyEmpty)",
     "                Commands.waitUntil(() -> false)"),
    ("the first enable does not deploy the intake", SS,
     "  public void onEnable() {\n    intakePosture = IntakePosture.DEPLOYED;\n  }",
     "  public void onEnable() {}"),
    ("STOW does not retract the intake", SS,
     "    if (wanted == WantedState.STOW) {\n      intakePosture = IntakePosture.STOWED;\n    }",
     "    if (wanted == WantedState.STOW) {}"),

    # ---- M10b step 9: the auto routines' wait for the intake to retain pieces, the only step 9
    # code that runs without PathPlanner. Attacked on each side: ending at once (the preload is
    # shed), never ending early (a second lost every auto), keying on the intaking angle (a
    # feed-assisting intake, already safe, holds the routine), and losing its bound (a dead pivot
    # costs the whole auto). Re-anchored in M13 step 4, which moved the wait into
    # AutoRoutines.waitUntilRetaining(Intake), a function of the intake alone.
    ("the retain wait does not wait", AR,
     "            Commands.waitUntil(intake::isRetainingPieces)",
     "            Commands.waitUntil(() -> true)"),
    ("the retain wait always runs to its timeout", AR,
     "            Commands.waitUntil(intake::isRetainingPieces)",
     "            Commands.waitUntil(() -> false)"),
    ("the retain wait keys on the intaking angle", AR,
     "            Commands.waitUntil(intake::isRetainingPieces)",
     "            Commands.waitUntil(intake::isAtIntakingAngle)"),
    ("the retain wait has no timeout", AR,
     "            Commands.waitUntil(intake::isRetainingPieces)\n                .withTimeout(Constants.Auto.kIntakeRetainTimeoutSec.get()),",
     "            Commands.waitUntil(intake::isRetainingPieces),"),

    # ---- M10b step 10: the verification pass. The retrofit order names four places where a
    # plausible bug passes every happy-path test. Each already has a blunt entry above ("burst latch
    # never sets", "floor roller waits for the gate"); these are the subtler versions that still
    # read like correct code.
    #
    # The burst latch. It must set only once the gate has OPENED -- the first piece of every burst
    # meets the tight band -- and must drop when the robot leaves the states that feed on the gate.
    ("burst latch sets before the gate ever opens", SS,
     "    } else if (gateOpenForThisState) {\n      burstLatched = true;",
     "    } else {\n      burstLatched = true;"),
    ("burst latch survives leaving the scoring states", SS,
     "    if (!burstState || !otherTermsHold) {\n      burstLatched = false;",
     "    if (!otherTermsHold) {\n      burstLatched = false;"),
    # The empty clock. It must count only gate-open feeding the indexer is actually doing; a
    # version keyed on the gate alone counts the loop a jam latches, and every loop after it.
    ("empty clock counts the gate, not the feeding", SS,
     "    } else if (feedOpen\n        && indexer.getSystemState() == Indexer.SystemState.FEEDING\n        && !indexer.isLoaded()) {",
     "    } else if (feedOpen && !indexer.isLoaded()) {"),
    ("a shot only restarts the clock while intaking", SS,
     "    if (newShot || intakeMode == IntakeMode.INTAKING) {",
     "    if (newShot && intakeMode == IntakeMode.INTAKING) {"),
    # The band's direction. The band is measured IN FROM STOW; measured in from deploy it is M10's
    # backwards rule again, inside the predicate this time rather than in the wiring. And a reading
    # past the stow stop is the arm at its most dangerous, not clear.
    ("band measured in from deploy instead of stow", IN,
     "      return angleRad <= stowedRad + widthRad;",
     "      return angleRad >= deployedRad - widthRad;"),
    ("a reading past the stow stop reads clear", IN,
     "      return angleRad <= stowedRad + widthRad;",
     "      return angleRad >= stowedRad && angleRad <= stowedRad + widthRad;"),
    # The floor roller leading. It keys on the WANTED state, so it also runs while a request is
    # diverted to SPINNING_UP or WAITING_FOR_TURRET; it runs for every request that shoots,
    # including the autos'; and a bare pre-spin does not run it.
    ("floor roller keys on the system state, not the request", SS,
     "        switch (wantedState) {\n          case SCORE_ON_MOVE, SCORE_STATIONARY, SCORE_FORCED, AUTO_SCORE_ON_MOVE, PASS -> true;",
     "        switch (systemState) {\n          case SCORING_ON_THE_MOVE, SCORING_STATIONARY, SCORING_FORCED, PASSING -> true;"),
    ("floor roller does not lead in autonomous", SS,
     "          case SCORE_ON_MOVE, SCORE_STATIONARY, SCORE_FORCED, AUTO_SCORE_ON_MOVE, PASS -> true;",
     "          case SCORE_ON_MOVE, SCORE_STATIONARY, SCORE_FORCED, PASS -> true;"),
    ("a bare pre-spin runs the floor roller", SS,
     "          case SCORE_ON_MOVE, SCORE_STATIONARY, SCORE_FORCED, AUTO_SCORE_ON_MOVE, PASS -> true;",
     "          case SCORE_ON_MOVE, SCORE_STATIONARY, SCORE_FORCED, AUTO_SCORE_ON_MOVE, PASS, SPIN_UP -> true;"),
    # The misconfigured feed-assist guard, which had no test until step 10.
    ("a refused feed-assist raises no warning", SS,
     "    feedAssistBlockedAlert.set(feedAssistBlocked);",
     "    feedAssistBlockedAlert.set(false);"),
    ("every feed-assist reads as refused", SS,
     "        next == IntakeMode.FEED_ASSIST\n            && intake.getSystemState() == Intake.SystemState.WAITING_FOR_TURRET;",
     "        next == IntakeMode.FEED_ASSIST;"),

    # ---- M11: stationary shooting. The override is the part that can hurt a match -- a leaked
    # one fires every later stationary shot at the last test speed -- so its scope and its clear
    # are attacked first. Then the table-free solve, and then the burst: ending on a belief a
    # previous burst left behind, and reporting numbers that are not its own.
    ("the shot-table override applies to every shot", SS,
     "    if (wantedState == WantedState.SCORE_STATIONARY && shotTableRpm.isPresent()) {",
     "    if (shotTableRpm.isPresent()) {"),
    ("another request does not clear the override", SS,
     "    if (wanted != WantedState.SCORE_STATIONARY) {\n      shotTableRpm = OptionalDouble.empty();",
     "    if (false) {\n      shotTableRpm = OptionalDouble.empty();"),
    ("the measurement shot aims along the robot's heading", SV,
     "        lineOfSight.getAngle(),\n        0.0,",
     "        robotPose.getRotation(),\n        0.0,"),
    ("the measurement shot ignores the range gate", SV,
     "        true,\n        inRange);",
     "        true,\n        true);"),
    ("the measurement shot accepts a zero speed", SV,
     "    if (distance <= 0.0 || !(flywheelRpm > 0.0) || !Double.isFinite(flywheelRpm)) {",
     "    if (distance <= 0.0 || !Double.isFinite(flywheelRpm)) {"),
    ("a burst ends on the last burst's empty belief", CC,
     "      return (firedThisBurst && superstructure.likelyEmpty()) || timer.hasElapsed(holdSec);",
     "      return superstructure.likelyEmpty() || timer.hasElapsed(holdSec);"),
    ("a burst never ends on the belief", CC,
     "      return (firedThisBurst && superstructure.likelyEmpty()) || timer.hasElapsed(holdSec);",
     "      return timer.hasElapsed(holdSec);"),
    ("a burst never sets the override", CC,
     "      superstructure.setShotTableRpm(rpmThisBurst);",
     "      // superstructure.setShotTableRpm(rpmThisBurst);"),
    ("a burst does not return the robot to rest", CC,
     "      superstructure.setWantedState(Superstructure.WantedState.DEFAULT);",
     "      // superstructure.setWantedState(Superstructure.WantedState.DEFAULT);"),
    ("a burst reports every shot since boot", CC,
     "              shooter.getShotCount() - shotCountAtStart,",
     "              shooter.getShotCount(),"),
    ("a burst never reports a timeout", CC,
     "              timer.hasElapsed(holdSec)));",
     "              false));"),

    # ---- M12: shooting on the move. The stutter fix first -- removed, made strict so a wheel that
    # has ARRIVED still projects out of the band, and applied to every projection so a wheel that
    # cannot reach its goal reads ready. Then the feedforward's chassis term, flipped and dropped:
    # plan section 6.4's first failure mode. Then the moving shot: the velocity subtracted twice
    # again (the bug the checkpoint found), and the virtual goal leading WITH the motion.
    ("the flywheel projection is no longer saturated", SH,
     "    if ((inputs.velocityRpm - goalRpm) * (projectedRpm - goalRpm) <= 0.0) {",
     "    if (false) {"),
    ("a wheel exactly at its goal is not saturated", SH,
     "    if ((inputs.velocityRpm - goalRpm) * (projectedRpm - goalRpm) <= 0.0) {",
     "    if ((inputs.velocityRpm - goalRpm) * (projectedRpm - goalRpm) < 0.0) {"),
    ("every projection is saturated", SH,
     "    if ((inputs.velocityRpm - goalRpm) * (projectedRpm - goalRpm) <= 0.0) {",
     "    if (true) {"),
    ("the feedforward adds the chassis rate", TR,
     "        goalFieldAngularVelocityRadPerSec\n            - RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;",
     "        goalFieldAngularVelocityRadPerSec\n            + RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;"),
    ("the feedforward ignores the chassis rate", TR,
     "        goalFieldAngularVelocityRadPerSec\n            - RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;",
     "        goalFieldAngularVelocityRadPerSec;"),
    ("the moving shot counts the robot's velocity twice", SV,
     "    Translation2d muzzleVelocity = new Translation2d(distance / tof, lineOfSight.getAngle());",
     "    Translation2d muzzleVelocity =\n        new Translation2d(distance / tof, lineOfSight.getAngle()).minus(velocity);"),
    ("the virtual goal leads with the motion", SV,
     "          goal.minus(velocity.times(tof))",
     "          goal.plus(velocity.times(tof))"),

    # ---- M13 step 1: settings.json checked against Drive.PP_CONFIG. The catch must stay broader than
    # the declared exceptions (the shipped file fails with a NullPointerException), rounding must
    # not warn, and the two values RobotConfig hides -- the gearing inside the motor model, and the
    # module positions -- must still be compared.
    ("settings.json errors escape the check", AF,
     "    } catch (Exception e) {\n      // Broader than the IOException",
     "    } catch (RuntimeException e) {\n      throw e;\n    } catch (Exception e) {\n      // Broader than the IOException"),
    ("the config comparison has no tolerance", AF,
     "GUI_CONFIG_RELATIVE_TOLERANCE = 0.005;",
     "GUI_CONFIG_RELATIVE_TOLERANCE = 0.0;"),
    ("the gearing is not compared", AF,
     "    compare(\n        mismatches,\n        \"driveGearing/driveMotorType\",\n        robot.moduleConfig.driveMotor.freeSpeedRadPerSec,\n        gui.moduleConfig.driveMotor.freeSpeedRadPerSec);\n",
     ""),
    ("module positions are not compared", AF,
     "        if (robot.moduleLocations[i].getDistance(gui.moduleLocations[i])\n            > GUI_MODULE_POSITION_TOLERANCE_METERS) {",
     "        if (false) {"),

    # ---- M13 step 2: AutoBuilder and the one flip. PathPlanner's FlippingUtil left as it ships, left
    # mirrored, never asked, and the drivetrain never commanded.
    ("no flip sync", AF,
     "    FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;\n    FlippingUtil.fieldSizeX = Constants.Field.kFieldLengthMeters.get();\n    FlippingUtil.fieldSizeY = Constants.Field.kFieldWidthMeters.get();\n",
     ""),
    ("symmetry left mirrored", AF,
     "FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;",
     "FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kMirrored;"),
    ("flip callback never flips", AF,
     "        AutoFactory::shouldFlipPaths,",
     "        () -> false,"),
    ("output dropped", AF,
     "(speeds, feedforwards) -> drive.runVelocity(speeds, feedforwards),",
     "(speeds, feedforwards) -> {},"),

    # ---- M13 step 3: loading paths and autos, loudly. The warning dropped, raised as an error (which
    # would stop teleop scoring), the reset's flip dropped or fixed at build time, the auto start
    # unflipped, and the auto file not checked before PathPlannerAuto swallows its absence.
    ("missing file not reported", AF,
     "      FaultReporter.getInstance().register(message, AlertType.kWarning, () -> true);\n",
     ""),
    ("missing file is an error", AF,
     "register(message, AlertType.kWarning, () -> true);",
     "register(message, AlertType.kError, () -> true);"),
    ("reset not flipped", AF,
     "                            AllianceFlipUtil.apply(start),",
     "                            start,"),
    ("reset flip read at build time", AF,
     "    Optional<Pose2d> blueStart =\n        loadPath(pathName).flatMap(PathPlannerPath::getStartingHolonomicPose);\n    return Commands.runOnce(\n        () ->\n            blueStart.ifPresent(\n                start ->\n                    RobotState.getInstance()\n                        .resetPose(\n                            AllianceFlipUtil.apply(start),",
     "    Optional<Pose2d> blueStart =\n        loadPath(pathName).flatMap(PathPlannerPath::getStartingHolonomicPose).map(AllianceFlipUtil::apply);\n    return Commands.runOnce(\n        () ->\n            blueStart.ifPresent(\n                start ->\n                    RobotState.getInstance()\n                        .resetPose(\n                            start,"),
    ("auto start not flipped", AF,
     ": group.get(0).getStartingHolonomicPose().map(AllianceFlipUtil::apply);",
     ": group.get(0).getStartingHolonomicPose();"),
    ("auto file not validated", AF,
     "      paths.autoPaths(autoName);\n      return new PathPlannerAuto(autoName);",
     "      return new PathPlannerAuto(autoName);"),

    # ---- M13 step 4: the marker vocabulary. StopIntake cancelling scoring, StowTurret commanding the
    # turret directly (which the superstructure overwrites), and a marker dropped.
    ("StopIntake cancels scoring", AF,
     "markers.put(\"StopIntake\", Commands.runOnce(() -> superstructure.setIntakeRequested(false)));",
     "markers.put(\"StopIntake\", Commands.runOnce(() -> { superstructure.setIntakeRequested(false); superstructure.setWantedState(Superstructure.WantedState.DEFAULT); }));"),
    ("StowTurret commands the turret directly", AF,
     "\"StowTurret\", superstructure.setWantedStateCommand(Superstructure.WantedState.DEFAULT));",
     "\"StowTurret\", turret.setWantedStateCommand(Turret.WantedState.STOW));"),
    ("Home marker dropped", AF,
     "    markers.put(\"Home\", superstructure.setWantedStateCommand(Superstructure.WantedState.HOME));\n",
     ""),

    # ---- M13 step 5: the building blocks, each holding its request for exactly its leg. The plan's own
    # forms, put back: one-shot requests, the spin-up beside the leg, a stationary shot with a
    # trailing DEFAULT.
    ("driveAndShoot sets its request once (plan form)", AR,
     "        factory.followPath(pathName),\n        factory.holdWantedState(Superstructure.WantedState.AUTO_SCORE_ON_MOVE));",
     "        factory.followPath(pathName),\n        superstructure.setWantedStateCommand(Superstructure.WantedState.AUTO_SCORE_ON_MOVE));"),
    ("spinUpWhile as a deadline beside the leg (plan form)", AR,
     "    return Commands.sequence(\n            superstructure.setWantedStateCommand(Superstructure.WantedState.SPIN_UP), inner)\n        .finallyDo(() -> superstructure.setWantedState(Superstructure.WantedState.DEFAULT));",
     "    return Commands.deadline(\n        inner, factory.holdWantedState(Superstructure.WantedState.SPIN_UP));"),
    ("stationary shot as a sequence with a trailing DEFAULT (plan form)", AF,
     "    return Commands.deadline(\n        Commands.defer(\n            () ->\n                Commands.waitUntil(superstructure::likelyEmpty)\n                    .withTimeout(Constants.Auto.kScoreTimeoutSec.get()),\n            Set.of()),\n        holdWantedState(Superstructure.WantedState.SCORE_STATIONARY));",
     "    return Commands.sequence(\n        superstructure.setWantedStateCommand(Superstructure.WantedState.SCORE_STATIONARY),\n        Commands.defer(\n            () ->\n                Commands.waitUntil(superstructure::likelyEmpty)\n                    .withTimeout(Constants.Auto.kScoreTimeoutSec.get()),\n            Set.of()),\n        superstructure.setWantedStateCommand(Superstructure.WantedState.DEFAULT));"),
    ("holdWantedState never hands back", AF,
     "        () -> superstructure.setWantedState(wanted),\n        () -> superstructure.setWantedState(Superstructure.WantedState.DEFAULT));",
     "        () -> superstructure.setWantedState(wanted),\n        () -> {});"),
    ("pickup has no dwell", AR,
     "Commands.sequence(factory.followPath(pathName), intakeSettle())",
     "factory.followPath(pathName)"),
    ("sequence starts with a score", AR,
     "      } else if (i % 2 == 0) {",
     "      } else if (i % 2 == 1) {"),
    ("sequence pickups not spun up", AR,
     "        legs[i] = spinUpWhile(driveAndIntake(path));",
     "        legs[i] = driveAndIntake(path);"),
    ("spinUpWhile exactly as the plan wrote it", AR,
     "    return Commands.sequence(\n            superstructure.setWantedStateCommand(Superstructure.WantedState.SPIN_UP), inner)\n        .finallyDo(() -> superstructure.setWantedState(Superstructure.WantedState.DEFAULT));",
     "    return Commands.deadline(\n        inner, superstructure.setWantedStateCommand(Superstructure.WantedState.SPIN_UP));"),

    # ---- M13 step 6: the routines. The intake wait skipped (the second entry needed the checkpoint's
    # 'nothing before the intake retains' clause to die), the preload count never logged, a leg
    # dropped, the scavenge's two sources of where to go, the stop issued once, and the end pose
    # unflipped.
    ("Leave skips the intake wait", AR,
     "        factory.resetPoseToPathStart(\"Leave\"),\n        waitUntilIntakeRetaining(),\n",
     "        factory.resetPoseToPathStart(\"Leave\"),\n"),
    ("opening skips the intake wait", AR,
     "        waitUntilIntakeRetaining(),\n        spinUpWhile(shootInPlace(Constants.Auto.kPreloadTimeoutSec)),",
     "        spinUpWhile(shootInPlace(Constants.Auto.kPreloadTimeoutSec)),"),
    ("preload count never logged", AR,
     "        spinUpWhile(shootInPlace(Constants.Auto.kPreloadTimeoutSec)),\n        factory.logPreloadFiredCommand());",
     "        spinUpWhile(shootInPlace(Constants.Auto.kPreloadTimeoutSec)));"),
    ("Center 5 drops its last scoring leg", AR,
     "                \"Center_ToPiece3\",\n                \"Center_Score3\"),",
     "                \"Center_ToPiece3\"),"),
    ("scavenge has no static fallback", AR,
     "                          .or(() -> factory.getPathEndingPose(waypointPath))\n",
     ""),
    ("scavenge ignores the camera", AR,
     "                      gamePieces\n                          .get()\n                          .or(() -> factory.getPathEndingPose(waypointPath))",
     "                      factory\n                          .getPathEndingPose(waypointPath)"),
    ("stationary shot stops the drive once (plan form)", AR,
     "        Commands.run(factory.getDrive()::stop, factory.getDrive()));",
     "        Commands.runOnce(factory.getDrive()::stop, factory.getDrive()));"),
    ("path end pose not flipped", AF,
     "                  poses.get(poses.size() - 1).getTranslation(), path.getGoalEndState().rotation());\n            })\n        .map(AllianceFlipUtil::apply);",
     "                  poses.get(poses.size() - 1).getTranslation(), path.getGoalEndState().rotation());\n            });"),

    # ---- M13 step 7: the selector. A cached routine, an unflipped preview, the warning's test inverted
    # or never registered, and a default other than Do Nothing.
    ("selector hands out one cached routine", AS,
     "    builder.get();\n    builders.put(name, builder);",
     "    Command built = builder.get();\n    builders.put(name, () -> built);"),
    ("preview start not flipped", AF,
     "        .flatMap(PathPlannerPath::getStartingHolonomicPose)\n        .map(AllianceFlipUtil::apply);",
     "        .flatMap(PathPlannerPath::getStartingHolonomicPose);"),
    ("misplacement test inverted", AS,
     "return startPoseErrorMeters > Constants.Auto.kStartPoseWarnMeters.get();",
     "return startPoseErrorMeters < Constants.Auto.kStartPoseWarnMeters.get();"),
    ("misplacement never registered as a fault", AS,
     "    FaultReporter.getInstance()\n        .register(",
     "    if (false) FaultReporter.getInstance()\n        .register("),
    ("default is not Do Nothing", AS,
     "chooser.addDefaultOption(DO_NOTHING, DO_NOTHING);",
     "chooser.addDefaultOption(DO_NOTHING, \"Leave\");"),

    # ---- M14 step 1: how loud a fault is versus whether it blocks. The registry never replacing,
    # anyCritical() reading the alert type again, the blocking list listing everything, the old
    # register() never blocking, and a device disconnect report-only by default.
    ("registry never replaces", FR,
     "    faults.removeIf(\n",
     "    if (false) faults.removeIf(\n"),
    ("anyCritical reads the alert type again", FR,
     "            fault -> fault.impact() == Impact.BLOCKS_SCORING && fault.isFaulted().getAsBoolean());",
     "            fault -> fault.type() == AlertType.kError && fault.isFaulted().getAsBoolean());"),
    ("blocking list is every active fault", FR,
     "        .filter(fault -> !blockingOnly || fault.impact() == Impact.BLOCKS_SCORING)\n",
     ""),
    ("plain register never blocks", FR,
     "        type == AlertType.kError ? Impact.BLOCKS_SCORING : Impact.REPORT_ONLY,",
     "        Impact.REPORT_ONLY,"),
    ("device disconnect is report-only by default", FR,
     "    registerDeviceDisconnect(deviceName, Impact.BLOCKS_SCORING, connected);",
     "    registerDeviceDisconnect(deviceName, Impact.REPORT_ONLY, connected);"),

    # ---- M14 step 2: every device registered by name. The blocking set changed in both directions (the
    # follower and the turret motor not blocking, the intake roller and a Limelight blocking), a
    # module device named by the wrong ID, and a connection check inverted.
    ("flywheel follower does not block", SH,
     "            \"Flywheel follower motor\", Constants.Can.kFlywheelFollowerId, Constants.Can.kRioBus),\n        FaultReporter.Impact.BLOCKS_SCORING,",
     "            \"Flywheel follower motor\", Constants.Can.kFlywheelFollowerId, Constants.Can.kRioBus),\n        FaultReporter.Impact.REPORT_ONLY,"),
    ("turret motor does not block", TR,
     "        FaultReporter.canDevice(\"Turret motor\", Constants.Can.kTurretId, Constants.Can.kRioBus),\n        FaultReporter.Impact.BLOCKS_SCORING,",
     "        FaultReporter.canDevice(\"Turret motor\", Constants.Can.kTurretId, Constants.Can.kRioBus),\n        FaultReporter.Impact.REPORT_ONLY,"),
    ("intake roller blocks scoring", IN,
     "            \"Intake roller motor\", Constants.Can.kIntakeRollerId, Constants.Can.kRioBus),\n        FaultReporter.Impact.REPORT_ONLY,",
     "            \"Intake roller motor\", Constants.Can.kIntakeRollerId, Constants.Can.kRioBus),\n        FaultReporter.Impact.BLOCKS_SCORING,"),
    ("a Limelight blocks scoring", VI,
     "        \"Turret Limelight (\" + Constants.Vision.kTurretCameraName + \")\",\n        FaultReporter.Impact.REPORT_ONLY,",
     "        \"Turret Limelight (\" + Constants.Vision.kTurretCameraName + \")\",\n        FaultReporter.Impact.BLOCKS_SCORING,"),
    ("steer motor named by the drive motor's ID", MO,
     "FaultReporter.canDevice(corner + \" steer motor\", constants.SteerMotorId, bus),",
     "FaultReporter.canDevice(corner + \" steer motor\", constants.DriveMotorId, bus),"),
    ("indexer check inverted", IX,
     "            () -> inputs.connected);",
     "            () -> !inputs.connected);"),

    # ---- M14 step 3: PowerBudget. The tiers starting together, the drivetrain throttled, the filter
    # starting from zero, imminent ignoring the slope, the severity never computed, and the intake
    # never giving up its rock.
    ("tiers all start together", PB,
     "  private static final double MEDIUM_STARTS_AT = 1.0 / 3.0;",
     "  private static final double MEDIUM_STARTS_AT = 0.0;"),
    ("the drivetrain is throttled too", PB,
     "          case CRITICAL -> 1.0;",
     "          case CRITICAL -> HIGH_STARTS_AT;"),
    ("the filter starts from zero", PB,
     "    if (Double.isNaN(filteredVoltage)) {\n      filteredVoltage = raw;",
     "    if (Double.isNaN(filteredVoltage)) {\n      filteredVoltage = 0.0;"),
    ("imminent ignores the slope", PB,
     "    return filteredVoltage < Constants.Power.kBrownoutWarningVolts.get()\n        && voltageSlopeVoltsPerSec < 0.0;",
     "    return filteredVoltage < Constants.Power.kBrownoutWarningVolts.get();"),
    ("severity never computed", PB,
     "    severity = MathUtil.clamp((warn - filteredVoltage) / (warn - critical), 0.0, 1.0);",
     "    severity = 0.0;"),
    ("the intake never gives up rocking", IN,
     "            lowPriorityCurrentScalar >= 1.0,",
     "            true,"),

    # ---- M14 step 4: the coast release. No delay, no speed check, firing every loop, and the latch
    # surviving a new disabled period.
    ("no delay", CR,
     "        || !disabledTimer.hasElapsed(Constants.Drivetrain.kCoastDelaySec.get())\n",
     ""),
    ("no speed check", CR,
     "        || speedMetersPerSec > Constants.Drivetrain.kCoastReleaseMaxSpeedMetersPerSec) {",
     "        || false) {"),
    ("fires every loop after the delay", CR,
     "    released = true;\n    return true;",
     "    return true;"),
    ("the latch survives a new disabled period", CR,
     "    disabledTimer.restart();\n    released = false;",
     "    disabledTimer.restart();"),

    # ---- M14 step 5: comp and practice identity. Simulation as the comp robot, an unknown serial silent
    # or running the practice constants, and the practice value read everywhere.
    ("simulation resolves to the comp robot", CO,
     "    if (mode != Mode.REAL) {\n      return RobotType.SIMBOT;",
     "    if (mode != Mode.REAL) {\n      return RobotType.COMP;"),
    ("an unknown serial is silent", CO,
     "    onUnknown.accept(serial);\n    return RobotType.COMP;",
     "    return RobotType.COMP;"),
    ("an unknown serial runs the practice constants", CO,
     "    onUnknown.accept(serial);\n    return RobotType.COMP;",
     "    onUnknown.accept(serial);\n    return RobotType.PRACTICE;"),
    ("forRobot reads the practice value everywhere", CO,
     "    return robotType == RobotType.PRACTICE ? practiceValue : compValue;",
     "    return robotType != RobotType.COMP ? practiceValue : compValue;"),
]


def run(cmd, **kw):
    return subprocess.run(cmd, shell=True, cwd=REPO, capture_output=True, text=True, **kw)


def clean():
    return run("git status --porcelain").stdout.strip() == ""


def main():
    if not clean():
        sys.exit("REFUSING TO RUN: working tree is dirty. Commit first -- the revert is "
                 "`git checkout --` and cannot tell a mutation from your work.")

    killed, survived, broken = [], [], []
    for i, (name, path, old, new) in enumerate(MUTATIONS, 1):
        full = os.path.join(REPO, path)
        before = open(full).read()
        if before.count(old) != 1:
            broken.append((name, f"anchor matched {before.count(old)} times, expected exactly 1"))
            print(f"[{i:2d}/{len(MUTATIONS)}] BAD ANCHOR  {name}")
            continue
        after = before.replace(old, new)
        assert after != before
        open(full, "w").write(after)
        # Prove the substitution landed, on disk, not in memory.
        assert open(full).read() == after, "write did not land"

        r = run("JAVA_HOME=/home/joshua/wpilib/2026/jdk ./gradlew -g /home/joshua/gradle-home "
                "--offline test --rerun-tasks -q")
        run(f"git checkout -- {path}")
        assert clean(), f"revert failed after {name}"

        if r.returncode != 0:
            failing = sorted({l.split(" > ")[1].split("(")[0]
                              for l in r.stdout.splitlines() + r.stderr.splitlines()
                              if " > " in l and "FAILED" in l})
            killed.append((name, failing))
            print(f"[{i:2d}/{len(MUTATIONS)}] KILLED      {name}"
                  f"  <- {', '.join(failing) if failing else 'build/test failure'}")
        else:
            survived.append(name)
            print(f"[{i:2d}/{len(MUTATIONS)}] SURVIVED    {name}")

    print(f"\nscore {len(killed)}/{len(MUTATIONS) - len(broken)}")
    if broken:
        print("\nBAD ANCHORS (these tested nothing):")
        for n, why in broken:
            print(f"  - {n}: {why}")
    if survived:
        print("\nSURVIVORS (a question, not a verdict -- check the mutation was reachable):")
        for n in survived:
            print(f"  - {n}")


main()
