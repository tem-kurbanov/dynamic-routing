#!/usr/bin/env python3
"""Run a bounded xgrid demo and check its output, without modifying scenarios."""
import argparse
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=ROOT / "build/MASHierarchicalSystem")
    parser.add_argument("--scenario", type=Path, default=ROOT / "scenarios/xgrid")
    parser.add_argument("--output", type=Path, default=ROOT / "build/example-output")
    parser.add_argument("--agents", type=int, choices=range(1, 5), default=1)
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--timeout", type=int, default=180)
    local_sumo = ROOT / "build/deps/sumo/bin/sumo"
    parser.add_argument("--sumo", default=str(local_sumo) if local_sumo.exists() else "sumo")
    parser.add_argument("--mpiexec", default="mpiexec")
    args = parser.parse_args()
    if args.steps <= 0 or args.timeout <= 0:
        parser.error("--steps and --timeout must be positive")
    scenario = args.scenario.resolve()
    binary = args.binary.resolve()
    for executable in (str(binary), args.sumo, args.mpiexec):
        if not shutil.which(executable):
            parser.error("Executable not found: " + executable)
    required = ["scenario.sumocfg", f"graphs/nodes{args.agents}.csv",
                f"graphs/edges{args.agents}.csv"]
    required += [f"models/log/{name}_60.json" for name in ("density", "speed", "fuel")]
    for name in required:
        if not (scenario / name).is_file():
            parser.error("Missing scenario input: " + str(scenario / name))
    output_root = args.output.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix="xgrid-", dir=output_root))
    # Each run gets fresh artifacts, including repeated CTest invocations.
    products = [output / name for name in ("tripinfo.xml", "edgedata.xml", "vehroutes.xml")]
    env = dict(os.environ)
    env["PATH"] = str(Path(shutil.which(args.sumo)).resolve().parent) + os.pathsep + env.get("PATH", "")
    env.setdefault("OMP_NUM_THREADS", "1")
    command = [args.mpiexec, "-n", str(args.agents + 1), str(binary),
               "--agents", str(args.agents), "--path", str(scenario), "--sync", "20",
               "--end", str(args.steps), "--epsilon", "0.99", "--k", "1", "--reroute", "0.85",
               "--tripinfo", str(products[0]), "--edgedata", str(products[1]),
               "--vehroutes", str(products[2]), "--logging"]
    print("Running:", " ".join(command), flush=True)
    with (output / "run.log").open("w") as log:
        process = subprocess.Popen(command, env=env, cwd=ROOT, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            result = process.wait(timeout=args.timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            raise RuntimeError("Example timed out; see " + str(output / "run.log"))
    if result:
        raise RuntimeError(f"MPI exited with status {result}; see {output / 'run.log'}")
    for path in products:
        ET.parse(path)
    routes = ET.parse(products[2]).getroot().findall("vehicle")
    if not routes:
        raise RuntimeError("No vehicle routes were recorded; use a longer --steps interval")
    print(f"Example passed: {len(routes)} vehicle routes recorded. Outputs: {output}")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, ET.ParseError) as error:
        print(error, file=sys.stderr)
        sys.exit(1)
