#!/usr/bin/env python3
"""Line coverage per source file from Unity Code Coverage results (OpenCover XML), as Markdown.

Unity writes one XML file per test run, so edit-mode and play-mode results come separately. They
are merged here: a line counts as covered if any run reached it.

    python3 tools/coverage_summary.py CodeCoverage              # table on stdout
    python3 tools/coverage_summary.py CodeCoverage --min 40     # and fail under 40% overall

CI appends the table to the test job's summary. Locally, run the tests with coverage from Window >
Analysis > Code Coverage (results land in CodeCoverage/ in the project folder).
"""
import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath


def find_results(paths):
    """(path, root element) of every OpenCover XML file in the given files and directories."""
    for p in map(Path, paths):
        for f in [p] if p.is_file() else sorted(p.rglob("*.xml")):
            try:
                root = ET.parse(f).getroot()
            except ET.ParseError:
                continue
            if root.tag == "CoverageSession":
                yield f, root


def relative(path):
    """Path from Assets/ or Packages/ on, wherever the project was checked out."""
    parts = PurePosixPath(path.replace("\\", "/")).parts
    for anchor in ("Assets", "Packages"):
        if anchor in parts:
            return "/".join(parts[parts.index(anchor):])
    return path.replace("\\", "/")


def line_hits(roots):
    """{file: {line: covered}} over all results."""
    lines = {}
    for root in roots:
        for module in root.iter("Module"):
            files = {f.get("uid"): relative(f.get("fullPath", "")) for f in module.iter("File")}
            for method in module.iter("Method"):
                ref = method.find("FileRef")
                default = ref.get("uid") if ref is not None else None
                for sp in method.iter("SequencePoint"):
                    name = files.get(sp.get("fileid", default))
                    if name is None:
                        continue
                    per_file = lines.setdefault(name, {})
                    line = int(sp.get("sl"))
                    per_file[line] = per_file.get(line, False) or int(sp.get("vc", "0")) > 0
    return lines


def percent(covered, total):
    return 100.0 * covered / total if total else 100.0


def markdown(lines, result_count):
    total = sum(len(v) for v in lines.values())
    covered = sum(sum(v.values()) for v in lines.values())
    out = [
        "## Code coverage",
        "",
        f"Line coverage {percent(covered, total):.0f}% ({covered} of {total} lines) over {len(lines)} files, "
        f"from {result_count} result file{'s' if result_count != 1 else ''}.",
        "",
        "| File | Lines covered | |",
        "|---|---:|---:|",
    ]
    for name in sorted(lines):
        n, c = len(lines[name]), sum(lines[name].values())
        out.append(f"| {name} | {c} / {n} | {percent(c, n):.0f}% |")
    return "\n".join(out) + "\n", percent(covered, total)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("paths", nargs="+", help="coverage results folder(s) or OpenCover XML file(s)")
    ap.add_argument("--min", type=float, help="fail if overall line coverage is below this percentage")
    args = ap.parse_args()

    results = list(find_results(args.paths))
    if not results:
        sys.exit(f"no OpenCover results under {', '.join(args.paths)}")
    text, overall = markdown(line_hits(root for _, root in results), len(results))
    sys.stdout.write(text)
    if args.min is not None and overall < args.min:
        sys.exit(f"line coverage {overall:.1f}% is below the minimum {args.min:g}%")


if __name__ == "__main__":
    main()
