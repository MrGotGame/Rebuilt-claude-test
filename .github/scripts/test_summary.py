#!/usr/bin/env python3
"""Render JUnit XML into the GitHub Actions job summary.

CI already ran the tests before this existed -- `gradle build` includes `test` -- but a failure
was only visible as a non-zero exit code buried in the log. This turns the run into a table on
the checks page naming the class, the case and the assertion message, so a red check is
actionable without downloading an artifact.

Deliberately dependency-free: no marketplace action, nothing to pin or audit, and it behaves the
same on a laptop as in CI.
"""

import glob
import os
import sys
import xml.etree.ElementTree as ET

ROOT = "build/test-results/test"
out = []
total = failed = skipped = 0
failures = []

for path in sorted(glob.glob(os.path.join(ROOT, "*.xml"))):
    suite = ET.parse(path).getroot()
    name = suite.get("name", os.path.basename(path))
    t = int(suite.get("tests", 0))
    f = int(suite.get("failures", 0)) + int(suite.get("errors", 0))
    s = int(suite.get("skipped", 0))
    total += t
    failed += f
    skipped += s
    if t - s > 0 or f > 0:
        out.append((name.split(".")[-1], t - s, f, s))
    for case in suite.iter("testcase"):
        for bad in list(case.findall("failure")) + list(case.findall("error")):
            failures.append(
                (name.split(".")[-1], case.get("name", "?"), (bad.get("message") or "").strip())
            )

ran = total - skipped
lines = ["## Test results", ""]

if not out and ran == 0:
    # Not a failure: several branches legitimately carry only @Disabled placeholders. It is still
    # worth saying loudly, because "all checks passed" on a branch that executed nothing is the
    # vacuous pass this whole setup exists to prevent.
    lines += ["> **No tests executed.** This branch has no runnable tests yet.", ""]
else:
    lines += [
        f"**{ran} executed, {failed} failed, {skipped} skipped**",
        "",
        "| Suite | Ran | Failed | Skipped |",
        "|---|---:|---:|---:|",
    ]
    for name, r, f, s in out:
        lines.append(f"| {name} | {r} | {f} | {s} |")
    lines.append("")

if failures:
    lines += ["### Failures", ""]
    for suite, case, msg in failures:
        msg = msg.replace("\n", " ")
        if len(msg) > 300:
            msg = msg[:300] + "…"
        lines += [f"**{suite} › {case}**", "", f"```\n{msg}\n```", ""]

summary = "\n".join(lines)
print(summary)

target = os.environ.get("GITHUB_STEP_SUMMARY")
if target:
    with open(target, "a", encoding="utf-8") as fh:
        fh.write(summary + "\n")

# Exit non-zero only on real failures. The `test` step has already failed the job in that case;
# this is belt and braces for a future workflow that runs the summary on its own.
sys.exit(1 if failed else 0)
