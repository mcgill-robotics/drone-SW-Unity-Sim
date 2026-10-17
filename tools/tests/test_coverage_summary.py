"""Tests for tools/coverage_summary.py. From the repo root: python3 -m unittest discover -s tools/tests -v"""
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS))
import coverage_summary as cs  # noqa: E402

# Trimmed-down OpenCover output in the shape Unity's Code Coverage package writes.
EDIT_MODE = """<?xml version="1.0" encoding="utf-8"?>
<CoverageSession>
  <Summary numSequencePoints="5" visitedSequencePoints="3" />
  <Modules>
    <Module hash="A">
      <ModuleName>DroneSim</ModuleName>
      <Files>
        <File uid="1" fullPath="/github/workspace/Assets/_Project/Scripts/Bridge/MavlinkCodec.cs" />
        <File uid="2" fullPath="/github/workspace/Assets/_Project/Scripts/Vehicle/Wing.cs" />
      </Files>
      <Classes>
        <Class>
          <FullName>DroneSim.MavlinkCodec</FullName>
          <Methods>
            <Method visited="true">
              <FileRef uid="1" />
              <SequencePoints>
                <SequencePoint vc="4" sl="10" fileid="1" />
                <SequencePoint vc="4" sl="11" fileid="1" />
                <SequencePoint vc="0" sl="11" fileid="1" />
                <SequencePoint vc="0" sl="12" fileid="1" />
              </SequencePoints>
            </Method>
          </Methods>
        </Class>
        <Class>
          <FullName>DroneSim.Wing</FullName>
          <Methods>
            <Method visited="false">
              <FileRef uid="2" />
              <SequencePoints>
                <SequencePoint vc="0" sl="50" />
              </SequencePoints>
            </Method>
          </Methods>
        </Class>
      </Classes>
    </Module>
  </Modules>
</CoverageSession>
"""

# The play-mode run reaches line 12 and the wing, with the files numbered differently.
PLAY_MODE = """<?xml version="1.0" encoding="utf-8"?>
<CoverageSession>
  <Modules>
    <Module hash="A">
      <Files>
        <File uid="7" fullPath="C:\\work\\drone-sim\\Assets\\_Project\\Scripts\\Vehicle\\Wing.cs" />
        <File uid="8" fullPath="C:\\work\\drone-sim\\Assets\\_Project\\Scripts\\Bridge\\MavlinkCodec.cs" />
      </Files>
      <Classes>
        <Class>
          <Methods>
            <Method>
              <FileRef uid="8" />
              <SequencePoints><SequencePoint vc="1" sl="12" fileid="8" /></SequencePoints>
            </Method>
            <Method>
              <FileRef uid="7" />
              <SequencePoints><SequencePoint vc="2" sl="50" fileid="7" /></SequencePoints>
            </Method>
          </Methods>
        </Class>
      </Classes>
    </Module>
  </Modules>
</CoverageSession>
"""

CODEC = "Assets/_Project/Scripts/Bridge/MavlinkCodec.cs"
WING = "Assets/_Project/Scripts/Vehicle/Wing.cs"


class CoverageSummary(unittest.TestCase):
    def results(self, **files):
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        root = Path(d.name)
        for name, text in files.items():
            path = root / "drone-sim-opencov" / name / "TestCoverageResults_0000.xml"
            path.parent.mkdir(parents=True)
            path.write_text(text)
        (root / "Report").mkdir()
        (root / "Report" / "Summary.xml").write_text("<CoverageReport />")
        return root

    def hits(self, root):
        return cs.line_hits(r for _, r in cs.find_results([root]))

    def test_one_run(self):
        lines = self.hits(self.results(EditMode=EDIT_MODE))
        self.assertEqual({10: True, 11: True, 12: False}, lines[CODEC])
        self.assertEqual({50: False}, lines[WING], "falls back to the method's FileRef")

    def test_runs_are_merged_by_path(self):
        lines = self.hits(self.results(EditMode=EDIT_MODE, PlayMode=PLAY_MODE))
        self.assertEqual({10: True, 11: True, 12: True}, lines[CODEC])
        self.assertEqual({50: True}, lines[WING])

    def test_other_xml_is_ignored(self):
        self.assertEqual(1, len(list(cs.find_results([self.results(EditMode=EDIT_MODE)]))))

    def test_markdown_table(self):
        text, overall = cs.markdown(self.hits(self.results(EditMode=EDIT_MODE)), 1)
        self.assertAlmostEqual(50.0, overall)
        self.assertIn("Line coverage 50% (2 of 4 lines) over 2 files, from 1 result file.", text)
        self.assertIn(f"| {CODEC} | 2 / 3 | 67% |", text)
        self.assertIn(f"| {WING} | 0 / 1 | 0% |", text)

    def run_script(self, *args):
        return subprocess.run([sys.executable, str(TOOLS / "coverage_summary.py"), *map(str, args)],
                              capture_output=True, text=True)

    def test_minimum(self):
        root = self.results(EditMode=EDIT_MODE, PlayMode=PLAY_MODE)
        self.assertEqual(0, self.run_script(root, "--min", 100).returncode)
        root = self.results(EditMode=EDIT_MODE)
        result = self.run_script(root, "--min", 60)
        self.assertNotEqual(0, result.returncode)
        self.assertIn("below the minimum", result.stderr)
        self.assertIn("## Code coverage", result.stdout)

    def test_no_results_is_an_error(self):
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        result = self.run_script(d.name)
        self.assertNotEqual(0, result.returncode)
        self.assertIn("no OpenCover results", result.stderr)


if __name__ == "__main__":
    unittest.main()
