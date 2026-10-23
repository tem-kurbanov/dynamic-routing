"""Run with python3 -m unittest discover -s tests -v; requires a C++20 compiler.

Compile actual Router.cpp methods against a small Router state fixture and
the real Graph/Edge types. This isolates selection from MPI, SUMO and XGBoost;
it does not test MPI communication, prediction models or route replanning.
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RerouteSelectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = tempfile.TemporaryDirectory(prefix="reroute-selection-")
        cls.addClassCleanup(cls.work.cleanup)
        work = Path(cls.work.name)
        source = Path(os.environ.get("ROUTER_SOURCE", ROOT / "Router.cpp")).read_text()
        methods = []
        for signature in ("std::unordered_set<int> Router::selectForRerouting(",
                          "std::vector<routeData> Router::reactivelyReroute(",
                          "int Router::timeToInterval(double time)",
                          "int Router::timeToInterval(int time)"):
            start = source.index(signature)
            end = source.index("\n}\n", start) + 3
            methods.append(source[start:end])
        (work / "reroute_selection.inc").write_text("\n".join(methods))
        cls.binary = work / "reroute_selection"
        subprocess.run(
            shlex.split(os.environ.get("CXX", "c++"))
            + shlex.split(os.environ.get("CXXFLAGS", ""))
            + ["-std=c++20", "-D_GLIBCXX_ASSERTIONS", "-I", str(ROOT), "-I", str(work),
               str(ROOT / "tests/reroute_selection.cpp"), "-o", str(cls.binary)],
            check=True, capture_output=True, text=True,
        )

    def test_selection(self):
        for case in ("eligible", "no_congestion", "zero_portion", "unaffected_route",
                     "outside_horizon", "upstream_congested", "foreign_edge", "zero_length_cycle",
                     "reactive_local", "reactive_foreign", "reactive_expired_inflow"):
            with self.subTest(case=case):
                result = subprocess.run([str(self.binary), case], capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
