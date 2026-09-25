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
the intake's resting and feed-assist states and its two geometric predicates. Several
M10 anchors name code that later M10b steps replace (the piece guard, the isStowed turret
guard), and those entries must be rewritten against the new code in the step that
changes it, not deleted. Adding a milestone means adding its
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
SH = "src/main/java/frc/robot/subsystems/shooter/Shooter.java"

MUTATIONS = [
    ("piece-and-fault guard deleted", SS,
     "    if (!indexer.hasPiece() || FaultReporter.getInstance().anyCritical()) {\n      return SystemState.SPINNING_UP;\n    }",
     "    if (false) {\n      return SystemState.SPINNING_UP;\n    }"),
    ("critical-fault half of that guard deleted", SS,
     "if (!indexer.hasPiece() || FaultReporter.getInstance().anyCritical()) {",
     "if (!indexer.hasPiece()) {"),
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
    ("exit from INTAKING no longer restores the tag pipeline", SS,
     "      case INTAKING -> vision.setIntakePipeline(Constants.Vision.kAprilTagPipelineIndex);",
     "      case INTAKING -> {}"),
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
     "    turret.setIntakeArcGuard(() -> !intake.isStowed());",
     "    turret.setIntakeArcGuard(() -> false);"),
    ("forced scoring holds instead of feeding", SS,
     "    indexer.setWantedState(Indexer.WantedState.FEED);\n    floorRoller.setWantedState(FloorRoller.WantedState.FEED);",
     "    indexer.setWantedState(Indexer.WantedState.HOLD);\n    floorRoller.setWantedState(FloorRoller.WantedState.IDLE);"),
    ("forced-shot count raised on entry to any scoring state", SS,
     "        if (state == SystemState.SCORING_FORCED) {\n          forcedShotCount++;",
     "        if (true) {\n          forcedShotCount++;"),
    ("disable does not clear the force-shoot override", SS,
     "    ignoreShotTolerances = false;\n\n    // A stale solution",
     "    // A stale solution"),
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
    ("anyCritical() always reports healthy", FR,
     "    return faults.stream()\n        .anyMatch(fault -> fault.type() == AlertType.kError && fault.isFaulted().getAsBoolean());",
     "    return false;"),
    ("severity ignored, any fault counts as critical", FR,
     "        .anyMatch(fault -> fault.type() == AlertType.kError && fault.isFaulted().getAsBoolean());",
     "        .anyMatch(fault -> fault.isFaulted().getAsBoolean());"),
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
     "        yield atAngle(deployed) ? SystemState.DEPLOYED : SystemState.DEPLOYING;",
     "        yield SystemState.DEPLOYING;"),
    ("arrival judged against the previous loop's goal again", IN,
     "        yield atAngle(deployed) ? SystemState.INTAKING : SystemState.DEPLOYING;",
     "        yield atGoal() ? SystemState.INTAKING : SystemState.DEPLOYING;"),
    ("FEED_ASSIST escapes the M10 deploy guard", IN,
     "      case FEED_ASSIST -> {\n        if (!deployAllowed.getAsBoolean()) {\n          yield atAngle(stowed) ? SystemState.STOWED : SystemState.STOWING;\n        }",
     "      case FEED_ASSIST -> {"),
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
