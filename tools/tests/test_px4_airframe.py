"""Tests for tools/px4_airframe.py. From the repo root: python3 -m unittest discover -s tools/tests -v

Several cases mirror VehicleSpecTests.cs so the script and Unity keep agreeing on the same spec.
"""
import json
import math
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(TOOLS))
import px4_airframe as af  # noqa: E402

VEHICLES = af.REPO / "Assets" / "StreamingAssets" / "vehicles"
SCRIPTS = af.REPO / "Assets" / "_Project" / "Scripts"


def shipped(name):
    path = VEHICLES / f"{name}.json"
    return json.loads(path.read_text(encoding="utf-8-sig")), path


def params(text):
    """The airframe's param set-default lines as name -> number."""
    out = {}
    for line in text.splitlines():
        parts = line.split()
        if parts[:2] == ["param", "set-default"]:
            out[parts[2]] = float(parts[3])
    return out


def temp_dir(test):
    d = tempfile.TemporaryDirectory()
    test.addCleanup(d.cleanup)
    return Path(d.name)


class ShippedSpecs(unittest.TestCase):
    def test_every_shipped_spec_generates_an_airframe(self):
        names = sorted(p.stem for p in VEHICLES.glob("*.json"))
        self.assertTrue(names)
        for name in names:
            with self.subTest(name):
                spec, path = shipped(name)
                self.assertEqual([], af.problems(spec, path.parent))
                text, hover = af.generate(spec, path)
                self.assertIsNotNone(hover, "the lift rotors can't carry the weight")
                self.assertIn(f". ${{R}}etc/init.d-posix/airframes/{spec['px4']['baseAirframe']}\n", text)
                self.assertEqual(len(spec["rotors"]), params(text)["CA_ROTOR_COUNT"])

    def test_autostart_ids_are_unique(self):
        ids = [shipped(p.stem)[0]["px4"]["autostartId"] for p in VEHICLES.glob("*.json")]
        self.assertEqual(len(ids), len(set(ids)))


class Geometry(unittest.TestCase):
    def test_rotors_are_relative_to_the_cg_and_km_sign_follows_direction(self):
        spec, path = shipped("x500")
        p = params(af.generate(spec, path)[0])
        cg = spec["massProperties"]["cg"]
        for i, r in enumerate(spec["rotors"]):
            for k, axis in enumerate("XYZ"):
                self.assertAlmostEqual(r["position"][k] - cg[k], p[f"CA_ROTOR{i}_P{axis}"], places=4)
            sign = 1 if r["direction"] == "ccw" else -1
            self.assertAlmostEqual(sign * r["momentRatio"], p[f"CA_ROTOR{i}_KM"], places=4)

    def test_axis_is_normalised(self):
        spec, path = shipped("quadplane")
        spec["rotors"][4]["axis"] = [3, 0, 0]
        spec["rotors"][0]["axis"] = [0, 0, -0.5]
        p = params(af.generate(spec, path)[0])
        self.assertEqual((1, 0, 0), (p["CA_ROTOR4_AX"], p["CA_ROTOR4_AY"], p["CA_ROTOR4_AZ"]))
        self.assertEqual((0, 0, -1), (p["CA_ROTOR0_AX"], p["CA_ROTOR0_AY"], p["CA_ROTOR0_AZ"]))

    def test_output_functions_follow_channels(self):
        # Motors are 101.., servos 201.., in spec order; unused channels are disabled.
        spec, path = shipped("quadplane")
        spec["rotors"][0]["channel"], spec["rotors"][1]["channel"] = 9, 0
        p = params(af.generate(spec, path)[0])
        expected = {r["channel"]: 101 + i for i, r in enumerate(spec["rotors"])}
        expected.update({s["channel"]: 201 + j for j, s in enumerate(spec["controlSurfaces"])})
        for ch in range(af.CHANNELS):
            self.assertEqual(expected.get(ch, 0), p[f"PWM_MAIN_FUNC{ch + 1}"], f"channel {ch}")

    def test_control_surfaces(self):
        spec, path = shipped("quadplane")
        p = params(af.generate(spec, path)[0])
        self.assertEqual(len(spec["controlSurfaces"]), p["CA_SV_CS_COUNT"])
        for j, s in enumerate(spec["controlSurfaces"]):
            self.assertEqual(af.SURFACE_TYPES.index(s["type"]) + 1, p[f"CA_SV_CS{j}_TYPE"])
            for k, axis in enumerate(("R", "P", "Y")):
                self.assertEqual(s["torque"][k], p[f"CA_SV_CS{j}_TRQ_{axis}"])


class HoverThrottle(unittest.TestCase):
    def test_x500_matches_the_quadratic_model_with_idle(self):
        # Same numbers as VehicleSpecTests.HoverThrottle_X500_MatchesQuadraticModelWithIdle.
        spec, path = shipped("x500")
        text, hover = af.generate(spec, path)
        speed = math.sqrt(2 * af.GRAVITY / 4 / 8.54858)
        self.assertAlmostEqual((speed - 0.15) / 0.85, hover, places=4)
        self.assertAlmostEqual(hover, params(text)["MPC_THR_HOVER"], places=4)

    def test_param_is_clamped(self):
        spec, path = shipped("x500")
        spec["massProperties"]["mass"] = 0.05
        text, hover = af.generate(spec, path)
        self.assertLess(hover, 0.1)
        self.assertEqual(0.1, params(text)["MPC_THR_HOVER"])

    def test_no_param_when_out_of_reach(self):
        spec, path = shipped("x500")
        spec["massProperties"]["mass"] = 10
        text, hover = af.generate(spec, path)
        self.assertIsNone(hover)
        self.assertNotIn("MPC_THR_HOVER", params(text))


class ThrustTables(unittest.TestCase):
    def table(self, text, encoding="utf-8"):
        d = temp_dir(self)
        (d / "motor.csv").write_text(text, encoding=encoding)
        return d

    def test_loads_and_inverts(self):
        # Same numbers as VehicleSpecTests.ThrustTable_LoadsAndInverts.
        d = self.table("throttle,thrust_n\n0,0\n0.5,6\n1.0,20\n")
        r = {"name": "r", "thrustTable": "motor.csv"}
        self.assertEqual(20, af.max_thrust(d, r))
        self.assertAlmostEqual(0.5, af.throttle_for(d, r, 6))
        self.assertAlmostEqual(0.75, af.throttle_for(d, r, 13))
        self.assertIsNone(af.throttle_for(d, r, 25))

    def test_moment_ratio_comes_from_the_torque_column(self):
        d = self.table("# throttle, thrust, torque\n0,0,0\n1,20,0.4\n")
        self.assertAlmostEqual(0.02, af.moment_ratio(d, {"thrustTable": "motor.csv", "momentRatio": 0.5}))

    def test_byte_order_mark_is_skipped(self):
        d = self.table("throttle,thrust\n0,0\n1,10\n", encoding="utf-8-sig")
        self.assertEqual(10, af.max_thrust(d, {"thrustTable": "motor.csv"}))

    def test_idle_with_a_table_is_a_problem(self):
        # Mirrors VehicleSpecTests.ThrustTable_WithIdle_IsAProblem.
        spec, _ = shipped("quadplane")
        d = self.table("0,0\n1,30\n")
        spec["rotors"][4]["thrustTable"] = "motor.csv"
        self.assertEqual([], af.problems(spec, d))
        spec["rotors"][4]["idle"] = 0.1
        self.assertTrue(any("idle must be 0 with a thrustTable" in s for s in af.problems(spec, d)))

    def test_throttle_must_increase(self):
        spec, _ = shipped("quadplane")
        d = self.table("0,0\n0.6,10\n0.5,12\n")
        spec["rotors"][0]["thrustTable"] = "motor.csv"
        self.assertTrue(any("throttle must increase" in s for s in af.problems(spec, d)))


class Problems(unittest.TestCase):
    def test_flags_bad_entries(self):
        # Mirrors VehicleSpecTests.Problems_FlagsBadEntries.
        spec, path = shipped("quadplane")
        spec["rotors"][1]["channel"] = spec["rotors"][0]["channel"]
        spec["rotors"][2]["direction"] = "clockwise"
        spec["controlSurfaces"][0]["type"] = "aileron"
        spec["massProperties"]["mass"] = 0
        p = af.problems(spec, path.parent)
        for expected in ("also used by", "direction", "unknown type", "mass"):
            self.assertTrue(any(expected in s for s in p), f"{expected!r} not in {p}")

    def test_channel_out_of_range(self):
        spec, path = shipped("x500")
        spec["rotors"][0]["channel"] = af.CHANNELS
        self.assertTrue(any("channel must be 0 to" in s for s in af.problems(spec, path.parent)))


class MatchesUnity(unittest.TestCase):
    """Constants the script copies from the C# code."""

    def test_surface_types_match_surface_spec(self):
        source = (SCRIPTS / "Vehicle" / "VehicleSpec.cs").read_text()
        block = re.search(r"string\[\] Types\s*=\s*\{(.*?)\};", source, re.S).group(1)
        self.assertEqual(af.SURFACE_TYPES, re.findall(r'"([^"]+)"', block))

    def test_channel_count_matches_vehicle_actuators(self):
        source = (SCRIPTS / "Vehicle" / "VehicleActuators.cs").read_text()
        self.assertEqual(af.CHANNELS, int(re.search(r"ChannelCount\s*=\s*(\d+)", source).group(1)))


class CommandLine(unittest.TestCase):
    def run_script(self, *args):
        return subprocess.run([sys.executable, str(TOOLS / "px4_airframe.py"), *args],
                              capture_output=True, text=True)

    def test_refuses_a_spec_with_problems(self):
        spec, _ = shipped("x500")
        spec["rotors"] = []
        path = temp_dir(self) / "bad.json"
        path.write_text(json.dumps(spec))
        result = self.run_script(str(path))
        self.assertNotEqual(0, result.returncode)
        self.assertIn("has problems", result.stderr)
        self.assertEqual("", result.stdout)

    def test_installs_into_a_px4_build(self):
        spec, path = shipped("x500")
        px4 = temp_dir(self)
        airframes = px4 / "build" / "px4_sitl_default" / "etc" / "init.d-posix" / "airframes"
        airframes.mkdir(parents=True)
        (airframes / spec["px4"]["baseAirframe"]).write_text("# stock\n")
        result = self.run_script("x500", "--px4", str(px4))
        self.assertEqual(0, result.returncode, result.stderr)
        installed = airframes / f"{spec['px4']['autostartId']}_unity_x500"
        self.assertEqual(af.generate(spec, path)[0], installed.read_text())

    def test_install_needs_a_built_px4(self):
        result = self.run_script("x500", "--px4", str(temp_dir(self)))
        self.assertNotEqual(0, result.returncode)
        self.assertIn("build PX4 SITL first", result.stderr)


if __name__ == "__main__":
    unittest.main()
