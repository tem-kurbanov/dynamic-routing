"""Exercise setup decisions with process/network/build boundaries replaced."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("build_dependencies", ROOT / "scripts/build_dependencies.py")
setup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(setup)


class SetupTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix="routing-setup-")
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name) / "checkout"
        self.root.mkdir()
        self.lock = json.loads((ROOT / "dependencies.json").read_text())
        (self.root / "dependencies.json").write_text(json.dumps(self.lock))
        self.sumo = self.root / "build/deps/sumo-src"
        self.sumo.mkdir(parents=True)
        self.xgb = self.root / "local-xgboost"
        self.xgb.mkdir()
        (self.xgb / "CMakeLists.txt").write_text("project(xgboost LANGUAGES CXX C VERSION 3.0.5)")

    def invoke(self, arguments, run_effect=None):
        def git_output(command, **kwargs):
            return self.lock["sumo"]["commit"] if "rev-parse" in command else ""

        with patch.object(setup, "ROOT", self.root), \
             patch.object(setup.subprocess, "check_output", side_effect=git_output), \
             patch.object(setup, "run", side_effect=run_effect) as run, \
             patch("sys.argv", ["build_dependencies.py"] + arguments), \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            setup.main()
        return run

    def test_reuses_fetched_sumo_without_fetch_flag(self):
        (self.sumo / "CMakeLists.txt").touch()
        run = self.invoke(["--xgboost-source", str(self.xgb)])
        self.assertTrue((self.root / "build/deps/paths.cmake").exists())
        self.assertFalse(any(call.args[0] == "git" for call in run.call_args_list))
        self.assertEqual(run.call_args_list[0].args[2], self.sumo)

    def test_retries_an_interrupted_sumo_checkout(self):
        def checkout_effect(*command):
            if command[0] == "git" and "checkout" in command:
                (self.sumo / "CMakeLists.txt").touch()

        run = self.invoke(["--fetch", "--xgboost-source", str(self.xgb)], checkout_effect)
        self.assertTrue(any("fetch" in call.args for call in run.call_args_list))
        self.assertTrue((self.root / "build/deps/paths.cmake").exists())

    def test_rejects_wrong_xgboost_version(self):
        (self.sumo / "CMakeLists.txt").touch()
        (self.xgb / "CMakeLists.txt").write_text("project(xgboost LANGUAGES CXX C VERSION 3.3.0)")
        with self.assertRaises(SystemExit) as error:
            self.invoke(["--xgboost-source", str(self.xgb)])
        self.assertEqual(error.exception.code, 2)

    def test_rejects_corrupt_archive_before_extraction(self):
        (self.sumo / "CMakeLists.txt").touch()
        (self.root / "build/deps/xgboost-source-3.0.5.tar.gz").write_bytes(b"not the pinned archive")
        with self.assertRaises(SystemExit) as error:
            self.invoke([])
        self.assertEqual(error.exception.code, 2)
        self.assertFalse((self.root / "build/deps/xgboost-3.0.5-src").exists())


if __name__ == "__main__":
    unittest.main()
