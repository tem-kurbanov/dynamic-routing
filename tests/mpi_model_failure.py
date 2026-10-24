"""A worker model error must terminate the entire MPI job promptly."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--binary", required=True)
parser.add_argument("--mpiexec", required=True)
parser.add_argument("--sumo", required=True)
parser.add_argument("--scenario", type=Path, required=True)
args = parser.parse_args()
with tempfile.TemporaryDirectory(prefix="routing-model-failure-") as temporary:
    root = Path(temporary)
    scenario = root / "scenario"
    scenario.mkdir()
    for item in args.scenario.resolve().iterdir():
        if item.name != "models":
            (scenario / item.name).symlink_to(item, target_is_directory=item.is_dir())
    models = scenario / "models/log"
    models.mkdir(parents=True)
    for item in (args.scenario.resolve() / "models/log").iterdir():
        if item.name == "speed_60.json":
            (models / item.name).write_text("invalid model\n")
        else:
            (models / item.name).symlink_to(item)
    env = dict(os.environ)
    env["PATH"] = str(Path(args.sumo).resolve().parent) + os.pathsep + env.get("PATH", "")
    env.setdefault("OMP_NUM_THREADS", "1")
    command = [args.mpiexec, "-n", "3", args.binary, "--agents", "2", "--path", str(scenario),
               "--sync", "20", "--end", "1"]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, env=env, cwd=root, start_new_session=True)
    try:
        output, _ = process.communicate(timeout=20)
        if process.returncode == 0 or "MPI rank " not in output or "Load model " not in output:
            raise RuntimeError("Expected coordinated model failure:\n" + output)
    finally:
        # Clean up SUMO children too, including when the MPI job exits before them.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()
print("PASS corrupt worker model: nonzero coordinated MPI termination within 20 seconds")
