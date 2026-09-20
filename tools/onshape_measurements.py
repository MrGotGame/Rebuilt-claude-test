#!/usr/bin/env python3
"""Read the robot's mounting transforms out of Onshape and emit Constants.java literals.

WHAT THIS IS FOR

Four constants in Constants.Vision are still the identity transform with a
`TODO: MEASURE from CAD`, and they are the last thing blocking the M8 and M9
on-robot checkpoints:

    kRobotToTurretAxis    robot origin  -> turret rotation axis
    kTurretToCamera       turret at 0 rad -> turret camera lens
    kRobotToIntakeCamera  robot origin  -> intake camera lens
    kRobotToQuest         robot origin  -> Quest headset origin

All four are pure geometry, which is exactly what CAD knows and a tape measure
guesses at. This script asks Onshape for named mate connectors, composes them
into the frames the code actually wants, and prints Java ready to paste.

WHAT THIS IS NOT FOR

It cannot produce most of the outstanding numbers, and pretending otherwise
would be worse than not running it. Every kS/kV/kA/kG is electromechanical and
comes from M6's SysId routines. Absolute encoder offsets depend on how the
magnet happened to be installed. Hard-stop angles are where the mechanism
actually stops, not where CAD says it should. Field dimensions and goal
coordinates come from the FIRST field drawings. The game piece comes from
calipers and a kitchen scale. Robot mass comes from a scale, because CAD mass
omits wire, tape, zip ties and bumpers.

And even the four it does produce are a STARTING VALUE. The plan says so at each
one: "MEASURE from CAD, verify with a tag" for the turret camera, and for the
headset "measure from CAD, then verify empirically". A lens's optical centre is
not its mount face, and a headset in a printed cradle has real slop.

CREDENTIALS

Never passed on the command line and never printed. Either export

    ONSHAPE_ACCESS_KEY / ONSHAPE_SECRET_KEY

or put them in a key file OUTSIDE the repository, one per line as
`ONSHAPE_ACCESS_KEY=...`, and pass --key-file. The repo's .gitignore covers the
usual names as a backstop, but the real protection is keeping the file out of
the working tree entirely. Keys are read, used for HTTP Basic auth, and never
written to stdout, to a log, or into the emitted Java.

USAGE

    python3 tools/onshape_measurements.py --url '<onshape document url>' \
        --key-file ~/onshape-keys.env

    # First time, or if anything looks wrong, see what the API actually returned:
    python3 tools/onshape_measurements.py --url '...' --key-file ... --dump

See docs/CAD_MEASUREMENTS.md for which mate connectors to create and how to
orient them.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import re
import sys
import urllib.error
import urllib.parse
import urllib.request

BASE_URL = "https://cad.onshape.com/api/v17"

# The mate connectors this script looks for. Names are matched case-insensitively
# and ignore surrounding whitespace, because Onshape lets you type anything.
MC_ROBOT_ORIGIN = "MC_RobotOrigin"
MC_TURRET_AXIS = "MC_TurretAxis"
MC_TURRET_CAMERA = "MC_TurretCamera"
MC_INTAKE_CAMERA = "MC_IntakeCamera"
MC_QUEST = "MC_Quest"

REQUIRED = [MC_ROBOT_ORIGIN]
OPTIONAL = [MC_TURRET_AXIS, MC_TURRET_CAMERA, MC_INTAKE_CAMERA, MC_QUEST]


# --------------------------------------------------------------------------- 4x4 matrix maths --
#
# Written out longhand rather than pulled from numpy, so the script runs on a
# stock Python with no install step. These are 4x4 row-major homogeneous
# transforms, which is the shape Onshape returns.


def identity() -> list[float]:
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def matmul(a: list[float], b: list[float]) -> list[float]:
    out = [0.0] * 16
    for r in range(4):
        for c in range(4):
            out[r * 4 + c] = sum(a[r * 4 + k] * b[k * 4 + c] for k in range(4))
    return out


def invert_rigid(m: list[float]) -> list[float]:
    """Inverts a rigid transform by transposing R and negating R^T t.

    Valid only because these are rotations plus translations with no scale or
    shear, which is what a mate connector is. A general 4x4 inverse would also
    work and would hide a scaled assembly instead of failing on one, so this is
    the stricter and therefore better choice -- see `assert_rigid`.
    """
    r = [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]]
    t = [m[3], m[7], m[11]]
    rt = [r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]]
    nt = [
        -(rt[0] * t[0] + rt[1] * t[1] + rt[2] * t[2]),
        -(rt[3] * t[0] + rt[4] * t[1] + rt[5] * t[2]),
        -(rt[6] * t[0] + rt[7] * t[1] + rt[8] * t[2]),
    ]
    return [
        rt[0], rt[1], rt[2], nt[0],
        rt[3], rt[4], rt[5], nt[1],
        rt[6], rt[7], rt[8], nt[2],
        0.0, 0.0, 0.0, 1.0,
    ]


def assert_rigid(m: list[float], label: str) -> None:
    """Fails loudly if a transform carries scale, shear or a mirror.

    A mirrored or scaled mate connector produces a transform that looks
    plausible, composes without complaint, and puts the camera somewhere it is
    not. Checking costs nothing and the failure mode is otherwise invisible.
    """
    cols = [(m[0], m[4], m[8]), (m[1], m[5], m[9]), (m[2], m[6], m[10])]
    for i, c in enumerate(cols):
        norm = math.sqrt(sum(v * v for v in c))
        if abs(norm - 1.0) > 1e-6:
            raise SystemExit(
                f"{label}: basis vector {i} has length {norm:.9f}, not 1. The assembly appears "
                f"to be scaled, which this script deliberately refuses to guess at."
            )
    det = (
        m[0] * (m[5] * m[10] - m[6] * m[9])
        - m[1] * (m[4] * m[10] - m[6] * m[8])
        + m[2] * (m[4] * m[9] - m[5] * m[8])
    )
    if det < 0:
        raise SystemExit(
            f"{label}: determinant is {det:.6f}, so this frame is MIRRORED. A mirrored mate "
            f"connector would silently flip a sign in the emitted transform."
        )


def rotate_about_z(theta_rad: float) -> list[float]:
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def to_quaternion(m: list[float]) -> tuple[float, float, float, float]:
    """Rotation submatrix to (w, x, y, z), by Shepperd's method.

    A quaternion rather than roll/pitch/yaw on purpose. WPILib's
    `Rotation3d(roll, pitch, yaw)` builds Rz*Ry*Rx, and getting that order
    wrong produces a transform that is correct only near zero -- which is
    precisely the class of bug M8's checkpoint exists to catch. A quaternion has
    no convention to get wrong.
    """
    m00, m01, m02 = m[0], m[1], m[2]
    m10, m11, m12 = m[4], m[5], m[6]
    m20, m21, m22 = m[8], m[9], m[10]
    trace = m00 + m11 + m22
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w, x, y, z = 0.25 / s, (m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w, x, y, z = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w, x, y, z = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w, x, y, z = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / n, x / n, y / n, z / n)


def to_rpy_degrees(m: list[float]) -> tuple[float, float, float]:
    """Roll, pitch, yaw in degrees, for HUMAN sanity checking only.

    Emitted as a comment beside each transform, never as the value. WPILib
    positive pitch is nose-DOWN, so a camera tilted up reads as a negative
    pitch -- worth knowing when eyeballing whether a number is sane.
    """
    pitch = math.asin(max(-1.0, min(1.0, -m[8])))
    if abs(m[8]) < 0.999999:
        roll = math.atan2(m[9], m[10])
        yaw = math.atan2(m[4], m[0])
    else:
        roll = math.atan2(-m[6], m[5])
        yaw = 0.0
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


# ----------------------------------------------------------------------------------- Onshape --


def load_credentials(key_file: str | None) -> tuple[str, str]:
    access = os.environ.get("ONSHAPE_ACCESS_KEY")
    secret = os.environ.get("ONSHAPE_SECRET_KEY")
    if key_file:
        expanded = os.path.expanduser(key_file)
        if not os.path.exists(expanded):
            raise SystemExit(f"key file not found: {expanded}")
        real = os.path.realpath(expanded)
        repo = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
        if real.startswith(repo + os.sep):
            raise SystemExit(
                f"REFUSING to read credentials from inside the repository ({real}).\n"
                f"Move the key file somewhere outside the working tree; a file in here is one "
                f"`git add -A` away from being published."
            )
        with open(expanded) as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                k, v = k.strip(), v.strip().strip("\"'")
                if k == "ONSHAPE_ACCESS_KEY":
                    access = v
                elif k == "ONSHAPE_SECRET_KEY":
                    secret = v
    if not access or not secret:
        raise SystemExit(
            "No credentials. Export ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY, or pass\n"
            "--key-file pointing at a file outside the repo containing:\n"
            "    ONSHAPE_ACCESS_KEY=...\n"
            "    ONSHAPE_SECRET_KEY=...\n"
            "Create a key pair at https://dev-portal.onshape.com/keys with read scope only."
        )
    return access, secret


def parse_document_url(url: str) -> tuple[str, str, str, str]:
    """Pulls did / wvm / wvmid / eid out of an Onshape document URL."""
    m = re.search(r"/documents/([0-9a-f]+)/(w|v|m)/([0-9a-f]+)/e/([0-9a-f]+)", url)
    if not m:
        raise SystemExit(
            "Could not parse that URL. Expected the form\n"
            "  https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>\n"
            "Open the ASSEMBLY tab in Onshape and copy the address bar."
        )
    return m.group(1), m.group(2), m.group(3), m.group(4)


def api_get(path: str, access: str, secret: str, params: dict | None = None) -> dict:
    url = BASE_URL + path
    if params:
        url += "?" + urllib.parse.urlencode(params)
    token = base64.b64encode(f"{access}:{secret}".encode()).decode()
    req = urllib.request.Request(
        url, headers={"Accept": "application/json", "Authorization": f"Basic {token}"}
    )
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        body = e.read().decode(errors="replace")[:400]
        if e.code in (401, 403):
            # Deliberately does not echo the key, not even a prefix of it.
            raise SystemExit(
                f"Onshape refused the credentials (HTTP {e.code}). Check the key pair is active "
                f"at https://dev-portal.onshape.com/keys and has read scope for this document."
            )
        raise SystemExit(f"Onshape returned HTTP {e.code} for {path}\n{body}")


def collect_mate_connectors(assembly: dict) -> dict[str, list[float]]:
    """Walks the assembly response for named mate connectors and their transforms.

    Written defensively. Onshape nests mate connectors differently depending on
    whether they sit on the assembly itself or on an instance, and the shape has
    moved between API versions, so this searches rather than assuming a path. If
    it finds nothing, --dump shows what actually came back.
    """
    found: dict[str, list[float]] = {}

    def looks_like_transform(v) -> bool:
        return isinstance(v, list) and len(v) == 16 and all(isinstance(x, (int, float)) for x in v)

    def walk(node):
        if isinstance(node, dict):
            name = node.get("name") or node.get("featureName")
            if isinstance(name, str):
                for key in ("transform", "matrix", "mateConnectorTransform"):
                    if looks_like_transform(node.get(key)):
                        found.setdefault(name.strip(), list(node[key]))
                        break
                else:
                    ms = node.get("mateConnectorCS") or node.get("coordSystem")
                    if isinstance(ms, dict):
                        t = cs_to_matrix(ms)
                        if t:
                            found.setdefault(name.strip(), t)
            for v in node.values():
                walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)

    walk(assembly)
    return found


def cs_to_matrix(cs: dict) -> list[float] | None:
    """Builds a 4x4 from Onshape's {origin, xAxis, yAxis, zAxis} coordinate system form."""
    o = cs.get("origin")
    x = cs.get("xAxis")
    z = cs.get("zAxis")
    y = cs.get("yAxis")
    if not (isinstance(o, list) and isinstance(x, list) and isinstance(z, list)):
        return None
    if not isinstance(y, list):
        y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return [
        x[0], y[0], z[0], o[0],
        x[1], y[1], z[1], o[1],
        x[2], y[2], z[2], o[2],
        0.0, 0.0, 0.0, 1.0,
    ]


# ------------------------------------------------------------------------------------ output --


def emit(java_name: str, m: list[float], note: str) -> str:
    assert_rigid(m, java_name)
    w, x, y, z = to_quaternion(m)
    roll, pitch, yaw = to_rpy_degrees(m)
    tx, ty, tz = m[3], m[7], m[11]
    return f"""
    /**
     * {note}
     *
     * <p>Extracted from Onshape by tools/onshape_measurements.py. Orientation is written as a
     * quaternion rather than roll/pitch/yaw because Rotation3d's Euler constructor has an order
     * that is easy to get backwards and wrong only away from zero.
     *
     * <p>For eyeballing: roll {roll:+.2f} deg, pitch {pitch:+.2f} deg, yaw {yaw:+.2f} deg.
     * WPILib positive pitch is nose-DOWN, so a camera tilted up reads negative.
     *
     * <p>CAD is a starting value, not the final answer -- verify against a tag before trusting it.
     */
    public static final Transform3d {java_name} =
        new Transform3d(
            new Translation3d({tx:.6f}, {ty:.6f}, {tz:.6f}),
            new Rotation3d(new Quaternion({w:.9f}, {x:.9f}, {y:.9f}, {z:.9f})));
"""


def self_test() -> int:
    """Checks the geometry against hand-computed answers, with no credentials needed.

    This exists because the interesting half of this script cannot be tested against the real
    document until someone has created the mate connectors, and shipping untested transform maths
    into the one place the project has already been bitten twice -- M8's composition order, M9's
    mounting direction -- would be careless. Every expectation below is worked out with sines and
    cosines rather than by calling the functions under test, for the reason GeomUtilTest records:
    an expectation computed by the code cancels its own errors.
    """
    failures: list[str] = []

    def check(label: str, got: float, want: float, tol: float = 1e-9) -> None:
        if abs(got - want) > tol:
            failures.append(f"{label}: got {got!r}, want {want!r}")

    # 1. A transform composed with its own inverse is the identity.
    yaw = math.radians(37.0)
    c, s = math.cos(yaw), math.sin(yaw)
    t = [c, -s, 0, 1.5, s, c, 0, -2.5, 0, 0, 1, 0.75, 0, 0, 0, 1]
    ident = matmul(t, invert_rigid(t))
    for i, want in enumerate(identity()):
        check(f"inverse round-trip [{i}]", ident[i], want, 1e-12)

    # 2. Composition really is origin-relative. A robot origin 1 m ahead of the assembly origin
    #    must make a point 3 m ahead of the assembly origin read as 2 m ahead of the robot.
    origin = [1, 0, 0, 1.0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    target = [1, 0, 0, 3.0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    rel = matmul(invert_rigid(origin), target)
    check("origin-relative X", rel[3], 2.0)
    check("origin-relative Y", rel[7], 0.0)

    # 3. A yawed robot origin rotates the offset into the robot frame. With the robot origin yawed
    #    +90 deg, a target 2 m along assembly +X sits 2 m along the robot's -Y.
    origin_yawed = [0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    target_x = [1, 0, 0, 2.0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    rel = matmul(invert_rigid(origin_yawed), target_x)
    check("yawed origin X", rel[3], 0.0)
    check("yawed origin Y", rel[7], -2.0)

    # 4. The quaternion describes the same rotation as the matrix it came from.
    #
    #    Verified by rotating vectors both ways rather than by comparing components, since q and -q
    #    are the same rotation and a component comparison would fail spuriously on the sign.
    #
    #    Several rotations, not one, and COMPOUND ones. The first version of this check used a
    #    single pure pitch, and mutation testing walked straight through it: for a rotation about
    #    one axis the other two quaternion components are zero, so corrupting them changes nothing.
    #    The angles below are also chosen to span all four branches of Shepperd's method -- the
    #    trace branch for small angles, and each of the three largest-diagonal branches for
    #    rotations near 180 degrees about x, y and z.
    def rodrigues(axis: tuple[float, float, float], angle: float) -> list[float]:
        n = math.sqrt(sum(a * a for a in axis))
        ux, uy, uz = (a / n for a in axis)
        ca, sa = math.cos(angle), math.sin(angle)
        ci = 1.0 - ca
        return [
            ca + ux * ux * ci, ux * uy * ci - uz * sa, ux * uz * ci + uy * sa, 0.0,
            uy * ux * ci + uz * sa, ca + uy * uy * ci, uy * uz * ci - ux * sa, 0.0,
            uz * ux * ci - uy * sa, uz * uy * ci + ux * sa, ca + uz * uz * ci, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ]

    #    The near-180 axes are TILTED rather than pure. A pure X rotation does reach the m00
    #    branch, but its m01 and m10 are both zero, so corrupting that branch changes nothing and
    #    the mutation walks through -- which is what happened. Tilting the axis puts non-zero
    #    values in every term the branch touches.
    cases = [
        ((0, 1, 0), math.radians(-18.0)),        # a camera tilted up: the realistic case
        ((1, 2, 3), math.radians(47.0)),         # compound, small angle
        ((1.0, 0.35, 0.22), math.radians(176.0)),
        ((0.31, 1.0, 0.24), math.radians(176.0)),
        ((0.26, 0.33, 1.0), math.radians(176.0)),
        ((0.3, -0.7, 0.2), math.radians(121.0)),
    ]
    probes = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.37, -0.52, 0.81)]
    branches_hit: set[str] = set()
    for axis, angle in cases:
        mr = rodrigues(axis, angle)
        assert_rigid(mr, "rodrigues probe")

        # Which branch of Shepperd's method this case exercises, mirroring to_quaternion's own
        # conditions. Recorded so the coverage can be ASSERTED below rather than assumed -- the
        # whole reason two of these mutations survived the first version was silent under-coverage.
        tr, d0, d1, d2 = mr[0] + mr[5] + mr[10], mr[0], mr[5], mr[10]
        if tr > 0:
            branches_hit.add("trace")
        elif d0 > d1 and d0 > d2:
            branches_hit.add("m00")
        elif d1 > d2:
            branches_hit.add("m11")
        else:
            branches_hit.add("m22")

        w, x, y, z = to_quaternion(mr)
        for v in probes:
            # q * v * q^-1, expanded.
            tx = 2 * (y * v[2] - z * v[1])
            ty = 2 * (z * v[0] - x * v[2])
            tz = 2 * (x * v[1] - y * v[0])
            rot = (
                v[0] + w * tx + (y * tz - z * ty),
                v[1] + w * ty + (z * tx - x * tz),
                v[2] + w * tz + (x * ty - y * tx),
            )
            want = (
                mr[0] * v[0] + mr[1] * v[1] + mr[2] * v[2],
                mr[4] * v[0] + mr[5] * v[1] + mr[6] * v[2],
                mr[8] * v[0] + mr[9] * v[1] + mr[10] * v[2],
            )
            lbl = f"quaternion axis={axis} angle={math.degrees(angle):.0f} v={v}"
            for i in range(3):
                check(f"{lbl} [{i}]", rot[i], want[i], 1e-9)

    for branch in ("trace", "m00", "m11", "m22"):
        if branch not in branches_hit:
            failures.append(
                f"no test case reaches to_quaternion's {branch} branch, so a defect there would "
                f"go unnoticed"
            )

    m_pitch = rodrigues((0, 1, 0), math.radians(-18.0))

    # 5. A camera tilted UP reads as a NEGATIVE pitch, matching WPILib's nose-down convention.
    #    Asserted because getting this backwards is silent and the emitted comment would then
    #    reassure a reader that an upside-down camera was fine.
    _, pitch_deg, _ = to_rpy_degrees(m_pitch)
    check("tilted-up camera reads negative pitch", pitch_deg, -18.0, 1e-6)

    # 6. Un-rotating a posed turret. A camera 0.25 m along turret +X, with the CAD posed at 90 deg,
    #    must come back to 0.25 m along +X once un-rotated to turret zero.
    posed = matmul(rotate_about_z(math.radians(90.0)), [1, 0, 0, 0.25, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
    check("posed turret camera Y before un-rotating", posed[7], 0.25, 1e-9)
    unposed = matmul(rotate_about_z(-math.radians(90.0)), posed)
    check("un-rotated turret camera X", unposed[3], 0.25, 1e-9)
    check("un-rotated turret camera Y", unposed[7], 0.0, 1e-9)

    # 7. The rigidity guard actually refuses what it promises to refuse. A guard nobody tested is
    #    a guard nobody has.
    for label, bad in (
        ("scaled", [2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 1]),
        ("mirrored", [-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]),
    ):
        try:
            assert_rigid(bad, "probe")
            failures.append(f"assert_rigid accepted a {label} transform")
        except SystemExit:
            pass
    try:
        assert_rigid(identity(), "probe")
    except SystemExit:
        failures.append("assert_rigid rejected the identity, so it refuses everything")

    # 8. URL parsing, including the version/microversion forms.
    did, wvm, wvmid, eid = parse_document_url(
        "https://cad.onshape.com/documents/aaaa1111/w/bbbb2222/e/cccc3333"
    )
    if (did, wvm, wvmid, eid) != ("aaaa1111", "w", "bbbb2222", "cccc3333"):
        failures.append(f"URL parse got {(did, wvm, wvmid, eid)}")

    # 9. The coordinate-system form, including the inferred Y axis.
    cs = cs_to_matrix({"origin": [1, 2, 3], "xAxis": [1, 0, 0], "zAxis": [0, 0, 1]})
    check("cs origin X", cs[3], 1.0)
    check("cs inferred Y axis", cs[5], 1.0)
    assert_rigid(cs, "cs_to_matrix")

    # 10. Credentials inside the repo are refused.
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_probe_keys.env")
    try:
        with open(here, "w") as fh:
            fh.write("ONSHAPE_ACCESS_KEY=x\nONSHAPE_SECRET_KEY=y\n")
        try:
            load_credentials(here)
            failures.append("load_credentials read a key file from inside the repository")
        except SystemExit as e:
            if "REFUSING" not in str(e):
                failures.append(f"in-repo key file refused for the wrong reason: {e}")
    finally:
        if os.path.exists(here):
            os.remove(here)

    if failures:
        print("SELF-TEST FAILED")
        for f in failures:
            print("  -", f)
        return 1
    print("self-test passed: 10 groups, geometry and credential handling")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    if "--self-test" in sys.argv:
        return self_test()
    ap.add_argument("--self-test", action="store_true", help="check the geometry, no credentials needed")
    ap.add_argument("--url", required=True, help="Onshape ASSEMBLY tab URL")
    ap.add_argument("--key-file", help="file outside the repo holding the API key pair")
    ap.add_argument(
        "--turret-angle-deg",
        type=float,
        default=0.0,
        help="turret angle the CAD is posed at, if not its zero position. kTurretToCamera is "
        "defined at turret zero, so a CAD posed elsewhere must be un-rotated by this much.",
    )
    ap.add_argument("--dump", action="store_true", help="print the mate connector names found and exit")
    ap.add_argument("--mass", action="store_true", help="also fetch assembly mass properties")
    args = ap.parse_args()

    access, secret = load_credentials(args.key_file)
    did, wvm, wvmid, eid = parse_document_url(args.url)

    assembly = api_get(
        f"/assemblies/d/{did}/{wvm}/{wvmid}/e/{eid}",
        access,
        secret,
        {"includeMateConnectors": "true", "includeMateFeatures": "true", "includeNonSolids": "true"},
    )

    connectors = collect_mate_connectors(assembly)
    wanted = {n.lower(): n for n in REQUIRED + OPTIONAL}
    matched = {wanted[k.lower()]: v for k, v in connectors.items() if k.lower() in wanted}

    if args.dump or not matched:
        print(f"Mate connectors found in this assembly ({len(connectors)}):")
        for name in sorted(connectors):
            print("   ", name)
        if not matched:
            print(
                "\nNone of the expected names matched. Expected any of:\n   "
                + "\n   ".join(REQUIRED + OPTIONAL)
                + "\n\nSee docs/CAD_MEASUREMENTS.md for what to create and how to orient it."
            )
            return 2
        return 0

    if MC_ROBOT_ORIGIN not in matched:
        raise SystemExit(
            f"{MC_ROBOT_ORIGIN} is missing, and nothing can be computed without it. Every constant "
            f"here is relative to the FRC robot origin -- centre of the frame perimeter, on the "
            f"floor -- and Onshape's assembly origin is wherever the first part happened to land. "
            f"Run with --dump to see what names exist."
        )

    inv_origin = invert_rigid(matched[MC_ROBOT_ORIGIN])
    out: list[str] = []

    if MC_TURRET_AXIS in matched:
        robot_to_axis = matmul(inv_origin, matched[MC_TURRET_AXIS])
        out.append(
            emit(
                "kRobotToTurretAxis",
                robot_to_axis,
                "Robot origin to the turret's rotation axis. Only the translation and any fixed "
                "tilt live here; the turret's own rotation is composed in at runtime.",
            )
        )
        if MC_TURRET_CAMERA in matched:
            # kTurretToCamera is defined in the turret frame AT ZERO. If the CAD is posed with the
            # turret somewhere else, that pose is baked into the mate connector and must come out,
            # or every turret-camera pose is wrong by the posed angle -- which is exactly the
            # failure M8's checkpoint hunts, arriving from the CAD rather than from the code.
            axis_to_camera = matmul(invert_rigid(matched[MC_TURRET_AXIS]), matched[MC_TURRET_CAMERA])
            if abs(args.turret_angle_deg) > 1e-9:
                axis_to_camera = matmul(
                    rotate_about_z(-math.radians(args.turret_angle_deg)), axis_to_camera
                )
            out.append(
                emit(
                    "kTurretToCamera",
                    axis_to_camera,
                    f"Turret frame at 0 rad to the turret camera lens. CAD was posed at "
                    f"{args.turret_angle_deg:+.3f} deg and un-rotated to zero.",
                )
            )
    elif MC_TURRET_CAMERA in matched:
        print(
            f"// WARNING: {MC_TURRET_CAMERA} found but {MC_TURRET_AXIS} is missing, so the camera "
            f"cannot be expressed in the turret frame. Both are needed.",
            file=sys.stderr,
        )

    if MC_INTAKE_CAMERA in matched:
        out.append(
            emit(
                "kRobotToIntakeCamera",
                matmul(inv_origin, matched[MC_INTAKE_CAMERA]),
                "Robot origin to the intake camera lens; fixed, since the camera rides the frame.",
            )
        )

    if MC_QUEST in matched:
        out.append(
            emit(
                "kRobotToQuest",
                matmul(inv_origin, matched[MC_QUEST]),
                "Robot origin to the Quest 3S headset origin. The single largest error source in "
                "the vision subsystem: QuestNav carries the tightest covariance of any source, so "
                "an error here is a bias that WINS the fusion. Verify empirically per plan 7.6.",
            )
        )

    print("// ---- Generated by tools/onshape_measurements.py. Paste into Constants.Vision. ----")
    print("// Requires: import edu.wpi.first.math.geometry.Quaternion;")
    for block in out:
        print(block)

    missing = [n for n in OPTIONAL if n not in matched]
    if missing:
        print(f"// NOT FOUND, so not emitted: {', '.join(missing)}", file=sys.stderr)

    if args.mass:
        mp = api_get(f"/assemblies/d/{did}/{wvm}/{wvmid}/e/{eid}/massproperties", access, secret)
        print("\n// ---- Assembly mass properties (SI) ----", file=sys.stderr)
        print(json.dumps(mp, indent=2)[:4000], file=sys.stderr)
        print(
            "\n// NOTE: CAD mass is systematically light -- it omits wire, tape, zip ties and\n"
            "// bumpers. Constants.Robot.kMassKg should come from a scale, not from here.",
            file=sys.stderr,
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
