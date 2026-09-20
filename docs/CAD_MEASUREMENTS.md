# Getting measurements out of CAD

`tools/onshape_measurements.py` reads named mate connectors from the robot's Onshape assembly and
prints `Transform3d` literals ready to paste into `Constants.Vision`. This document is what you
have to do in Onshape first, and — more importantly — what this does **not** get you.

## What it fills in, and what it cannot

Four constants. That is the whole list:

| Constant | What it is |
|---|---|
| `kRobotToTurretAxis` | robot origin → turret rotation axis |
| `kTurretToCamera` | turret frame at 0 rad → turret camera lens |
| `kRobotToIntakeCamera` | robot origin → intake camera lens |
| `kRobotToQuest` | robot origin → Quest headset origin |

They matter out of proportion to their number: they are the last thing blocking M8's and M9's
on-robot checkpoints, and `Vision` currently raises a warning alert for each one that is still the
identity transform.

**Everything else on the outstanding list comes from somewhere other than CAD**, and it is worth
being blunt about that, because "we have the CAD, so we have the numbers" is a comfortable and
wrong thing to believe:

| Source | Constants | Why not CAD |
|---|---|---|
| **M6 SysId, on hardware** | every `kS`, `kV`, `kA`, `kG` across five mechanisms | Electromechanical, not geometric. No model predicts friction. |
| **The assembled robot** | `PivotAbsOffset`, `Turret/AbsOffset`, hard-stop angles, stall currents | An encoder offset depends on how the magnet happened to be installed that afternoon. |
| **FIRST field drawings** | `Field/Length`, `Field/Width`, `Goal X/Y/Height` | Wrong artifact entirely. |
| **Calipers and a scale** | `GamePieceDiameter`, `GamePieceMass` | It is not part of the robot. |
| **A scale** | `Robot/MassKg` | CAD mass is systematically light — it omits wire, tape, zip ties, bumpers. |
| **A real field** | shot tables, `RpmPerMps`, `RpmIntercept`, flight times | Empirical, and the plan says so. |

Mass properties (`--mass`) can give the moments of inertia marked `TODO: from CAD`, but those only
feed the simulation plants, so getting them slightly wrong costs simulation fidelity rather than a
match.

## Why CAD is a starting value, not an answer

The plan already says this at each constant — *"MEASURE from CAD, verify with a tag"* for the
turret camera, *"measure from CAD, then verify empirically"* for the headset. Three reasons:

1. **A lens's optical centre is not its mount face.** The Limelight's aperture sits some
   millimetres behind whatever surface your mount model touches, and your model of the camera may
   not include that offset at all.
2. **Mounting slop is real**, especially a headset in a printed cradle.
3. **Things get bent.** The camera you measured in October is not at the same angle in March.

This matters more than usual because of *how* these fail. An identity transform fails **loudly** —
the alerts fire continuously and nothing pretends to work. A plausible-but-wrong CAD transform
fails **silently**: every pose is biased, and for the turret camera the bias rotates as the shooter
tracks, which reads as a drifting gyro and gets diagnosed as one for a long time.

So: extract, paste, then run M8's slew test and M9's push test on the real robot.

## What to create in Onshape

Add five **mate connectors** to the top-level robot assembly and rename them exactly as below
(right-click → Rename; the script matches case-insensitively but the spelling must match).

| Name | Where to put it | How to orient it |
|---|---|---|
| `MC_RobotOrigin` | **Required.** Centre of the frame perimeter, on the floor | +X robot forward, +Y robot left, +Z up |
| `MC_TurretAxis` | On the turret's rotation axis, at the height the axis is defined | +Z **along the rotation axis**, +X pointing where the turret points at angle zero |
| `MC_TurretCamera` | At the turret camera's lens | +X **out of the lens**, +Y left, +Z up |
| `MC_IntakeCamera` | At the intake camera's lens | +X out of the lens, +Y left, +Z up |
| `MC_Quest` | At the headset's tracking origin | +X out of the headset's front face, +Y left, +Z up |

Three things are easy to get wrong here, and each one produces a plausible number:

- **The robot origin is an FRC convention, not an Onshape one.** Centre of the *frame perimeter*,
  at *floor level* — not the assembly origin, not the centre of the drivetrain, not a bumper
  corner. Onshape's assembly origin is wherever the first part happened to land, which is why the
  script refuses to run without `MC_RobotOrigin` rather than falling back to it.
- **Camera +X points out of the lens.** This is WPILib's camera convention, not "the direction the
  mounting plate faces". A camera tilted *up* ends up with a **negative** pitch, because WPILib
  positive pitch is nose-down. The script prints roll/pitch/yaw as a comment so you can sanity
  check the sign — if your upward-tilted camera reads positive, the connector is flipped.
- **Pose the turret at zero before extracting.** `kTurretToCamera` is defined in the turret frame
  *at 0 rad*. If your CAD is saved with the turret swung somewhere else, that angle is baked into
  the mate connector and every turret-camera pose will be wrong by it — the exact failure M8's
  checkpoint hunts, arriving from the CAD instead of from the code. Either pose the assembly at
  turret zero, or pass `--turret-angle-deg <angle>` and the script will un-rotate it.

## API keys

Create a read-only key pair at <https://dev-portal.onshape.com/keys>. Put them in a file
**outside the repository**:

```
# ~/onshape-keys.env   — NOT inside the working tree
ONSHAPE_ACCESS_KEY=...
ONSHAPE_SECRET_KEY=...
```

The script refuses to read a key file from inside the repo, because a file in here is one
`git add -A` away from being published to a repository that, private or not, you do not want
holding credentials. `.gitignore` covers the usual names as a backstop, but keeping the file out of
the tree is the actual protection.

## Running it

```bash
# Check the geometry works before involving the network or any credentials:
python3 tools/onshape_measurements.py --self-test

# See what mate connectors the assembly actually has:
python3 tools/onshape_measurements.py --dump \
    --url 'https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>' \
    --key-file ~/onshape-keys.env

# Emit the Java:
python3 tools/onshape_measurements.py \
    --url 'https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>' \
    --key-file ~/onshape-keys.env
```

Use the URL from the **assembly** tab, not a part studio.

The emitted transforms carry their orientation as a **quaternion**, not as roll/pitch/yaw. That is
deliberate: `Rotation3d(roll, pitch, yaw)` builds `Rz·Ry·Rx`, and getting that order backwards
produces a transform that is correct only near zero — which is exactly the bug M8's checkpoint
exists to catch, and exactly the one that would be hardest to spot in a generated constant. A
quaternion has no convention to get wrong. You will need to add
`import edu.wpi.first.math.geometry.Quaternion;` to `Constants.java`.

## If the first run finds nothing

Onshape nests mate connectors differently depending on whether they sit on the assembly itself or
on an instance, and the shape has moved between API versions. The script searches rather than
assuming a fixed path, but `--dump` prints every named connector it found so the mismatch is
visible in one look rather than guessed at. If the names are there but the transforms are not,
`--dump` is the right thing to paste into a conversation.

## The STEP route, and what it established

`tools/step_inspect.py` reads a STEP export instead of the live document. It cannot replace the
mate-connector route — a STEP file carries no datum saying "this is the robot origin" and no
annotation saying "the lens is here, pointing that way" — but it is very good at confirming the
things that are easy to get wrong and expensive to find late.

Run against `Robot.step` it established, by three independent routes, that **the assembly origin
already is the FRC robot origin**:

- the frame bounding box is centred on it in XY (0.703 × 0.656 m, centre within 1 mm of zero);
- the four swerve modules are symmetric about it, at roughly ±0.26 m — which agrees with
  `TunerConstants`' ±10 inches, so that value is confirmed rather than a Tuner X placeholder;
- the module hardware bottoms out at exactly Z = 0.0000, so Z = 0 is the floor.

That is the single hardest fact to establish and the one a mis-placed `MC_RobotOrigin` would get
wrong, so it is worth having independently.

Two traps were hit writing that tool, both of which produce plausible-looking wrong answers:

1. **The transform direction is not fixed by the standard.** A
   `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` names two representations and which is the
   parent varies by exporter. In this Onshape export `rep_1` is the *child*, the opposite of the
   usual assumption, and composing it the other way silently produces a wrong assembly. The
   giveaway was two flywheels on one shooter coming out 90° apart with Y and Z swapped. The tool
   now decides per relationship instead of assuming.
2. **B-rep geometry contains points that are not on the part.** Unbounded surface and curve
   definitions carry control points far outside the solid; one bracket measured 501 m across on a
   first pass. Bounds are percentile-based for that reason.

## Testing

`--self-test` checks the geometry against hand-computed answers with no credentials and no network:
inverse round-trips, origin-relative composition including a yawed origin, the quaternion
conversion across all four branches of Shepperd's method, the turret un-rotation, the
scale/mirror guard, URL parsing, and the refusal to read credentials from inside the repo.

It has been mutation tested at 12/12 — every injected defect turns it red. That bar exists because
the two survivors of the first attempt were both real holes: the quaternion check used a single
pure-axis rotation, where the components a defect would corrupt are zero anyway. The suite now
asserts its own branch coverage so that particular gap cannot reopen silently.
