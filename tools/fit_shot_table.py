#!/usr/bin/env python3
"""Turn a practice field's shot-table measurements into Constants.Shot literals.

WHAT THIS IS FOR

M11 fills the shot tables. On the practice field, at each taped distance, you
fire bursts with CharacterizationCommands.measureShotTablePoint (the
/Tuning/ShotTable/Measure toggle), adjust ShotTable/Rpm until the burst scores,
and time the flight on 120 fps video. Each burst is one row of a CSV. This
script turns the rows into the three things the solver reads:

    kDistanceBreakpointsMeters   the distances you measured at
    kTimeOfFlightSec             the flight time at each of them
    kRpmPerMeterPerSec           slope   \\  of the fit  rpm = slope * speed + intercept,
    kRpmIntercept                intercept / where speed = distance / time of flight

That is the whole model. The solver asks the time-of-flight table how long a
shot takes, gets the ground speed the piece needs as distance / time, and
turns that speed into RPM through the fit. (There was also a launch-speed
table. The solver never read it, and M11 step 1 deleted it.)

THE CSV

One row per burst. A header row is required, "#" starts a comment, and
columns may come in any order:

    distance_m   the TAPED distance, meters            (required)
    rpm          the ShotTable/Rpm the burst used       (required)
    tof_s        time of flight, seconds                (this, or the next two)
    tof_frames   frames from release to arrival
    fps          the video's frame rate
    fired        pieces fired (ShotTable/ShotsFired)    (optional)
    scored       pieces that went in, by hand count      (optional)

Record every burst, including the misses. Rows with fired and scored are
kept only if scored / fired reaches --min-hit-rate (default 0.8, the M11
checkpoint's 8 of 10), so the misses you tuned past drop out on their own.
Rows without them are always kept. Where several kept rows share a distance,
the time of flight is their mean, and every row is its own point in the RPM
fit.

    python3 tools/fit_shot_table.py shots.csv
    python3 tools/fit_shot_table.py --self-test

WHAT IT CHECKS

It refuses a non-positive time of flight, and data that cannot fit a line
(fewer than two different speeds). It warns about fewer than 8 distances
(M11 asks for 8-10), about a time of flight that falls as distance rises
(the solver's iteration expects it to rise), and about any row the fit misses
by more than --warn-residual-rpm. A large residual usually means a mistyped
row or a burst fired from the wrong place; compare that row's
ShotTable/EstimatedDistanceMeters with its tape.

It changes no file. It prints Java; you paste it into Constants.Shot, rebuild,
and run the M11 checkpoint.
"""
import argparse
import csv
import math
import sys


class ShotTableError(ValueError):
    """The data cannot produce a table. The message says which row and why."""


def read_rows(lines, min_hit_rate):
    """Parse CSV lines into kept (distance, rpm, tof) rows.

    Args:
        lines: an iterable of CSV text lines, header first.
        min_hit_rate: the scored / fired ratio a row needs to be kept, when it
            has both columns.

    Returns:
        A tuple (kept, dropped): kept is a list of (distance_m, rpm, tof_s)
        tuples, and dropped is a list of (line_number, reason) for rows left
        out by the hit-rate filter. Line numbers are the file's own, comments
        and blank lines included, so a flagged row can be found in an editor.

    Raises:
        ShotTableError: on a missing column, a bad number, or a non-positive
            time of flight.
    """
    numbered = [(number, line) for number, line in enumerate(lines, start=1)
                if line.strip() and not line.lstrip().startswith("#")]
    reader = csv.DictReader([line for _, line in numbered])
    if reader.fieldnames is None:
        raise ShotTableError("the CSV is empty")
    fields = {name.strip() for name in reader.fieldnames}
    for required in ("distance_m", "rpm"):
        if required not in fields:
            raise ShotTableError(f"missing the '{required}' column")
    if "tof_s" not in fields and not {"tof_frames", "fps"} <= fields:
        raise ShotTableError("need a 'tof_s' column, or both 'tof_frames' and 'fps'")

    kept, dropped = [], []
    for (number, _), raw in zip(numbered[1:], reader):
        row = {k.strip(): (v or "").strip() for k, v in raw.items() if k is not None}

        def value(name):
            try:
                return float(row[name])
            except (KeyError, ValueError):
                raise ShotTableError(f"row {number}: '{name}' is not a number: {row.get(name)!r}")

        distance, rpm = value("distance_m"), value("rpm")
        tof = value("tof_s") if row.get("tof_s") else value("tof_frames") / value("fps")
        if not (tof > 0.0 and math.isfinite(tof)):
            raise ShotTableError(f"row {number}: time of flight must be positive, got {tof}")
        if not (distance > 0.0 and math.isfinite(distance)):
            raise ShotTableError(f"row {number}: distance must be positive, got {distance}")

        if row.get("fired") and row.get("scored"):
            fired, scored = value("fired"), value("scored")
            if fired <= 0 or scored / fired < min_hit_rate:
                dropped.append((number, f"{scored:g} of {fired:g} scored"))
                continue
        kept.append((distance, rpm, tof))
    return kept, dropped


def fit(kept):
    """Build the table and the RPM fit from kept rows.

    Args:
        kept: (distance_m, rpm, tof_s) tuples, as read_rows returns them.

    Returns:
        A dict with 'distances' and 'tofs' (the table, ascending), 'slope' and
        'intercept' (the RPM fit), and 'residuals': (distance, speed, rpm,
        fitted rpm) per row.

    Raises:
        ShotTableError: if fewer than two different speeds were measured.
    """
    by_distance = {}
    for distance, _, tof in kept:
        by_distance.setdefault(distance, []).append(tof)
    distances = sorted(by_distance)
    tofs = [sum(by_distance[d]) / len(by_distance[d]) for d in distances]

    points = [(distance / tof, rpm) for distance, rpm, tof in kept]
    n = len(points)
    mean_v = sum(v for v, _ in points) / n if n else 0.0
    spread = sum((v - mean_v) ** 2 for v, _ in points)
    if n < 2 or spread <= 1e-12:
        raise ShotTableError(
            "need at least two different speeds (distance / time of flight) to fit a line")
    mean_rpm = sum(r for _, r in points) / n
    slope = sum((v - mean_v) * (r - mean_rpm) for v, r in points) / spread
    intercept = mean_rpm - slope * mean_v
    residuals = [
        (distance, distance / tof, rpm, slope * distance / tof + intercept)
        for distance, rpm, tof in kept
    ]
    return {"distances": distances, "tofs": tofs, "slope": slope, "intercept": intercept,
            "residuals": residuals}


def warnings_for(result, warn_residual_rpm):
    """List the things worth a second look before pasting the result.

    Args:
        result: the dict fit() returns.
        warn_residual_rpm: how far off the line a row may be before it is named.

    Returns:
        A list of human-readable warning strings, empty when nothing stands out.
    """
    notes = []
    if len(result["distances"]) < 8:
        notes.append(f"only {len(result['distances'])} distances; M11 asks for 8-10 across the "
                     "usable range")
    for (d0, t0), (d1, t1) in zip(zip(result["distances"], result["tofs"]),
                                  zip(result["distances"][1:], result["tofs"][1:])):
        if t1 < t0:
            notes.append(f"time of flight falls from {t0:.3f} s at {d0:g} m to {t1:.3f} s at "
                         f"{d1:g} m; the solver expects it to rise with distance")
    for distance, speed, rpm, fitted in result["residuals"]:
        if abs(rpm - fitted) > warn_residual_rpm:
            notes.append(f"the row at {distance:g} m, {rpm:.0f} RPM is {rpm - fitted:+.0f} RPM off "
                         f"the fit (speed {speed:.2f} m/s)")
    return notes


def java(result):
    """Render the result as Java to paste into Constants.Shot.

    Args:
        result: the dict fit() returns.

    Returns:
        The Java source text.
    """
    def array(values, digits):
        return ", ".join(f"{v:.{digits}f}" for v in values)

    return (
        "    public static final double[] kDistanceBreakpointsMeters = {"
        f"{array(result['distances'], 3)}}};\n"
        "    public static final double[] kTimeOfFlightSec = {"
        f"{array(result['tofs'], 4)}}};\n"
        "    public static final LoggedTunableNumber kRpmPerMeterPerSec =\n"
        f"        new LoggedTunableNumber(\"Shot/RpmPerMps\", {result['slope']:.2f});\n"
        "    public static final LoggedTunableNumber kRpmIntercept =\n"
        f"        new LoggedTunableNumber(\"Shot/RpmIntercept\", {result['intercept']:.2f});\n")


def self_test():
    """Prove the fit recovers a known answer and the checks refuse bad data.

    Returns:
        0 on success. Raises AssertionError on the first failure.
    """
    # A known model: flight time rising with distance, RPM linear in speed. Two rows at 3 m, so the
    # averaging and the every-row-is-a-point rules are both exercised.
    slope, intercept = 380.0, 650.0
    rows = ["distance_m,rpm,tof_s,fired,scored"]
    for d in (1.5, 2.0, 2.5, 3.0, 3.0, 4.0, 5.0, 6.0):
        tof = 0.30 + 0.12 * d
        rows.append(f"{d},{slope * d / tof + intercept},{tof},10,9")
    rows.append("4.5,9999,0.9,10,5")  # a tuned-past miss: dropped by the 8-of-10 filter
    kept, dropped = read_rows(rows, 0.8)
    assert len(kept) == 8 and len(dropped) == 1, (len(kept), dropped)
    result = fit(kept)
    assert abs(result["slope"] - slope) < 1e-6, result["slope"]
    assert abs(result["intercept"] - intercept) < 1e-6, result["intercept"]
    assert result["distances"] == [1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0], result["distances"]
    assert abs(result["tofs"][3] - 0.66) < 1e-12, result["tofs"]
    assert not any("off the fit" in w for w in warnings_for(result, 50.0))

    # Reported line numbers are the file's own, counting comments: the miss here is line 4.
    _, dropped = read_rows(
        ["# header next", "distance_m,rpm,tof_s,fired,scored", "# a miss next",
         "2.0,2600,0.44,10,4"], 0.8)
    assert dropped == [(4, "4 of 10 scored")], dropped

    # Frames and a frame rate convert to seconds: 36 frames at 120 fps is 0.3 s.
    kept, _ = read_rows(["distance_m,rpm,tof_frames,fps", "2.0,3000,36,120"], 0.8)
    assert abs(kept[0][2] - 0.3) < 1e-12, kept

    # The warnings fire when they should.
    falling = fit([(2.0, 3000.0, 0.5), (3.0, 3200.0, 0.4)])
    assert any("falls" in w for w in warnings_for(falling, 1e9))
    assert any("only 2 distances" in w for w in warnings_for(falling, 1e9))
    outlier = fit([(2.0, 3000.0, 0.5), (3.0, 3200.0, 0.6), (4.0, 9000.0, 0.7)])
    assert any("off the fit" in w for w in warnings_for(outlier, 50.0))

    # And the refusals.
    for bad, why in (
        (["distance_m,rpm,tof_s", "2.0,3000,0"], "zero time of flight"),
        (["distance_m,rpm,tof_s", "2.0,3000,-0.4"], "negative time of flight"),
        (["distance_m,rpm", "2.0,3000"], "no time of flight at all"),
        (["rpm,tof_s", "3000,0.4"], "no distance"),
    ):
        try:
            read_rows(bad, 0.8)
        except ShotTableError:
            continue
        raise AssertionError(f"accepted a CSV with {why}")
    try:
        fit([(2.0, 3000.0, 0.5), (2.0, 3100.0, 0.5)])
    except ShotTableError:
        pass
    else:
        raise AssertionError("fitted a line through a single speed")

    print("self-test passed")
    return 0


def main(argv):
    """Command-line entry point.

    Args:
        argv: the arguments after the program name.

    Returns:
        The process exit status.
    """
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("csv", nargs="?", help="the measurements, one row per burst")
    parser.add_argument("--min-hit-rate", type=float, default=0.8,
                        help="scored / fired a row needs to be kept (default 0.8, 8 of 10)")
    parser.add_argument("--warn-residual-rpm", type=float, default=100.0,
                        help="name any row the fit misses by more than this (default 100)")
    parser.add_argument("--self-test", action="store_true", help="check this script and exit")
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()
    if not args.csv:
        parser.error("give a CSV file, or --self-test")

    try:
        with open(args.csv, newline="") as handle:
            kept, dropped = read_rows(handle.read().splitlines(), args.min_hit_rate)
        result = fit(kept)
    except (OSError, ShotTableError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    for number, reason in dropped:
        print(f"dropped row {number}: {reason}, below --min-hit-rate {args.min_hit_rate:g}")
    for note in warnings_for(result, args.warn_residual_rpm):
        print(f"warning: {note}")
    print(f"\n{len(kept)} rows kept, {len(result['distances'])} distances. Paste into "
          "Constants.Shot:\n")
    print(java(result))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
