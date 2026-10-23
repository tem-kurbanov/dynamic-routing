#!/usr/bin/env python3
"""Build sibling sources, or fetch pinned dependencies with --fetch."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {".git", "lib", "build", "__pycache__"}


def fingerprint(root):
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root)
        if not path.is_file() or any(part in EXCLUDED for part in relative.parts):
            continue
        digest.update(relative.as_posix().encode() + b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def run(*command):
    print("+", " ".join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sumo-source", type=Path)
    parser.add_argument("--xgboost-source", type=Path, help="Optional local source tree; must be XGBoost 3.0.5")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--fetch", action="store_true", help="Fetch pinned sources into build/deps")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    lock = json.loads((ROOT / "dependencies.json").read_text())
    base = ROOT / "build/deps"
    base.mkdir(parents=True, exist_ok=True)
    sumo = args.sumo_source.resolve() if args.sumo_source else ROOT.parent / "sumo"
    if not (sumo / "CMakeLists.txt").exists() and not args.sumo_source:
        sumo = base / "sumo-src"
        if not (sumo / "CMakeLists.txt").exists() and args.fetch:
            run("git", "init", sumo)
            run("git", "-C", sumo, "fetch", "--depth=1", lock["sumo"]["repository"], lock["sumo"]["commit"])
            run("git", "-C", sumo, "checkout", "--detach", "FETCH_HEAD")
    if not (sumo / "CMakeLists.txt").exists():
        parser.error("Missing SUMO source; supply --sumo-source or use --fetch")
    commit = subprocess.check_output(["git", "-C", str(sumo), "rev-parse", "HEAD"], text=True).strip()
    dirty = subprocess.check_output(["git", "-C", str(sumo), "status", "--porcelain", "--untracked-files=no"], text=True)
    if commit != lock["sumo"]["commit"] or dirty:
        parser.error("SUMO source differs from the pinned commit in dependencies.json")

    release = lock["xgboost"]
    private_xgb = base / ("xgboost-" + release["version"] + "-src")
    xgb_build = base / ("xgboost-" + release["version"] + "-build")
    if args.xgboost_source:
        xgb = args.xgboost_source.resolve()
        cmake_file = xgb / "CMakeLists.txt"
        if not cmake_file.exists() or not re.search(r"project\(xgboost[^)]*VERSION\s+" + re.escape(release["version"]) + r"\s*\)", cmake_file.read_text()):
            parser.error("Local XGBoost source must identify itself as " + release["version"])
        if not private_xgb.exists():
            shutil.copytree(xgb, private_xgb, ignore=shutil.ignore_patterns(*EXCLUDED))
        elif fingerprint(private_xgb) != fingerprint(xgb):
            parser.error("Cached XGBoost source differs from --xgboost-source")
    else:
        archive = base / ("xgboost-source-" + release["version"] + ".tar.gz")
        if not archive.exists():
            if not args.fetch:
                parser.error("Missing pinned XGBoost archive; use --fetch to download it")
            temporary = archive.with_suffix(".download")
            urllib.request.urlretrieve(release["url"], temporary)
            temporary.rename(archive)
        if hashlib.sha256(archive.read_bytes()).hexdigest() != release["archive_sha256"]:
            parser.error("XGBoost archive checksum mismatch")
        if not private_xgb.exists():
            with tempfile.TemporaryDirectory(dir=base) as temporary:
                staging = Path(temporary)
                with tarfile.open(archive) as source:
                    for member in source.getmembers():
                        target = (staging / member.name).resolve()
                        if (not target.is_relative_to(staging.resolve()) or
                                Path(member.name).parts[0] != "xgboost" or
                                not (member.isfile() or member.isdir())):
                            parser.error("Unsafe member in XGBoost source archive")
                    source.extractall(staging)
                (staging / "xgboost").rename(private_xgb)
    run("cmake", "-S", sumo, "-B", base / "sumo", "-DCMAKE_BUILD_TYPE=Release",
        "-DISOLATED_BUILD=ON", "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=" + str(base / "sumo/bin"),
        "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=" + str(base / "sumo/lib"),
        "-DCHECK_OPTIONAL_LIBS=OFF", "-DNETEDIT=OFF", "-DENABLE_PYTHON_BINDINGS=OFF",
        "-DENABLE_JAVA_BINDINGS=OFF", "-DENABLE_CS_BINDINGS=OFF", "-DFMI=OFF", "-DPARQUET=OFF")
    run("cmake", "--build", base / "sumo", "--target", "sumo", "libtracicpp", "--parallel", args.jobs)
    run("cmake", "-S", private_xgb, "-B", xgb_build, "-DCMAKE_BUILD_TYPE=Release",
        "-DUSE_OPENMP=OFF", "-DUSE_CUDA=OFF", "-DKEEP_BUILD_ARTIFACTS_IN_BINARY_DIR=ON")
    run("cmake", "--build", xgb_build, "--parallel", args.jobs)
    values = {"SUMO_HOME": sumo, "SUMO_LIBRARY_DIR": base / "sumo/lib",
              "SUMO_EXECUTABLE": base / "sumo/bin/sumo", "XGBOOST_ROOT": private_xgb,
              "XGBOOST_LIBRARY_DIR": xgb_build / "lib"}
    with (base / "paths.cmake").open("w") as output:
        for name in ("SUMO_INCLUDE_DIR", "TRACICPP_LIBRARY", "XGBOOST_INCLUDE_DIR", "XGBOOST_LIBRARY"):
            output.write(f"unset({name} CACHE)\n")
        for name, value in values.items():
            escaped = str(value).replace("\\", "/").replace('"', '\\"')
            output.write(f'set({name} "{escaped}" CACHE PATH "Local dependency build" FORCE)\n')
    print("Next: cmake -S . -B build -C build/deps/paths.cmake -DENABLE_SUMO_SMOKE_TEST=ON")


if __name__ == "__main__":
    main()
