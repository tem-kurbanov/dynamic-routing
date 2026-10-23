# Cooperative Dynamic Traffic Routing

C++ research software for routing urban traffic with cooperating area-control agents. Each agent plans within its own part of a road network, predicts traffic conditions with XGBoost, and exchanges partial routes with neighboring agents over MPI.

The planner searches for Pareto alternatives across **distance, travel time, fuel consumption, and edge count**, then selects a route using ranked preferences. SUMO provides the traffic simulation through TraCI.

**Paper:** *Multiagent System for Dynamic Multicriteria Traffic Routing in Urban Environments*, AAMAS 2026 — Temirlan Kurbanov, Adéla Kubíková, Jiří Vokřínek, and Franziska Klügl. [Manuscript source](AAMAS_2026_Dynamic_Routing/main.tex) · [Citation metadata](CITATION.cff)

## How it works

```mermaid
flowchart LR
    SUMO[SUMO traffic simulation] <-->|TraCI| MA[Master process]
    MA <-->|Departures, positions, routes| A[Area agent 1]
    MA <-->|Departures, positions, routes| B[Area agent 2]
    A <-->|Partial routes and planning tokens| B
    A --> P[Hierarchical multicriteria search]
    B --> Q[Hierarchical multicriteria search]
    P --> X[Density, speed and fuel predictions]
    Q --> Y[Density, speed and fuel predictions]
```

The master advances SUMO and distributes vehicle requests. Area agents perform hierarchical label-setting search over path-cover graphs, cooperate across area boundaries, and record chosen routes to update future traffic expectations. Position synchronization runs every 20 seconds in the example; congestion-triggered rerouting is requested every 60 seconds.

## Build and run the xgrid example

Requirements: a C++20 compiler, CMake 3.26 or newer, Python 3.9 or newer, Git, Open MPI, Xerces-C, and zlib. Linux and macOS are the intended environments. Windows support is not configured.

On Ubuntu 24.04:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake python3 git libopenmpi-dev openmpi-bin libxerces-c-dev zlib1g-dev
```

On macOS, install the Xcode Command Line Tools and the dependencies:

```bash
brew install cmake open-mpi xerces-c
```

From the repository root:

```bash
python3 scripts/build_dependencies.py --fetch --jobs 4
cmake -S . -B build -C build/deps/paths.cmake \
  -DCMAKE_BUILD_TYPE=Release -DENABLE_SUMO_SMOKE_TEST=ON
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
python3 scripts/run_example.py
```

The helper uses an existing `../sumo` checkout if it matches the pinned commit, otherwise `--fetch` downloads the pinned fork. It downloads and verifies the **XGBoost 3.0.5** source archive, then builds CPU-only libraries locally under `build/deps/`. It leaves sibling source trees unchanged. Internet access is needed for missing sources; subsequent builds can omit `--fetch`.

The default example uses `scenarios/xgrid`, one area agent plus one master process, and 120 simulation steps. It is a bounded demonstration, not an experiment run. It writes each run to a fresh directory under `build/example-output/`:

| File | Contents |
|---|---|
| `run.log` | MPI/application logs and SUMO output |
| `tripinfo.xml` | Completed vehicle-trip statistics |
| `edgedata.xml` | Edge statistics produced by SUMO |
| `vehroutes.xml` | Vehicle routes, including unfinished trips at the stopping point |

The runner checks the process exit status, parses all three XML files, and requires recorded vehicle routes. For a cooperative example with two area agents:

```bash
python3 scripts/run_example.py --agents 2 --steps 30
```

The current application reserves a 1 GiB MPI send buffer per process. Allow at least 2 GiB plus simulation overhead for the default example, or 3 GiB plus overhead for two area agents. `--timeout` bounds the runner's wall-clock duration.

## Dependency versions

[dependencies.json](dependencies.json) records the SUMO commit, XGBoost release URL and SHA-256 checksum, and model-version metadata.

| Component | Pin / provenance |
|---|---|
| SUMO | [`tem-kurbanov/sumo`](https://github.com/tem-kurbanov/sumo), commit `0814800eceddd60f1fb6b41a7d8ec07589eee7ca` |
| XGBoost inference library | **3.0.5** |
| xgrid models | Model files record XGBoost **3.0.0** |
| InTAS models | Model files record XGBoost **3.0.5** |

The supplied scenarios use the fork's `time-to-teleport.disable-jam` option. A stock SUMO package may not accept it, so the source helper and CI use the pinned fork. Model-file versions identify the library that saved each model; a separate original training environment is not bundled. XGBoost documents backward compatibility for saved models in its [Model IO guide](https://xgboost.readthedocs.io/en/stable/tutorials/saving_model.html#a-note-on-backward-compatibility-of-models-and-memory-snapshots). Native tests load and predict with all nine bundled model files using 3.0.5.

An existing local XGBoost source tree can be supplied with `--xgboost-source /path/to/xgboost`; it must identify itself as 3.0.5. For a SUMO checkout in another location, use `--sumo-source /path/to/sumo`.

If the dependencies are already built, configure their locations explicitly:

```bash
cmake -S . -B build-custom -DCMAKE_BUILD_TYPE=Release \
  -DSUMO_HOME=/path/to/sumo-source \
  -DSUMO_LIBRARY_DIR=/path/to/sumo-library-directory \
  -DXGBOOST_ROOT=/path/to/xgboost-3.0.5-source \
  -DXGBOOST_LIBRARY_DIR=/path/to/xgboost-library-directory
cmake --build build-custom --parallel 4
```

Only the TraCI C++ library is needed by this executable; libsumo and GUI bindings are not linked. OpenMP and CUDA are disabled for the helper's XGBoost build. These settings support portable inference and are not a reconstruction of the paper's performance-testing environment.

## Scenarios and direct usage

Both `scenarios/xgrid` and `scenarios/intas` contain network partitions for one to four area agents, SUMO inputs, and pretrained predictors. The executable loads the **`models/log/`** models, whose transforms match the estimator code. The additional xgrid models directly under `models/` are retained as supplied; they are not substituted into the planner.

```text
scenarios/<name>/
  scenario.sumocfg
  graphs/nodes<N>.csv
  graphs/edges<N>.csv
  models/log/{density,speed,fuel}_60.json
  ... network, route and optional additional XML files
```

The graph files use contiguous integer node and edge IDs and cluster-membership bitmasks. Typed route files encode the driver's criterion priorities in the vehicle type ID. The bundled partitions were prepared in advance; graph partitioning and model training are not part of this repository's setup.

For direct use, add the pinned SUMO binary directory to `PATH` and launch exactly one more MPI process than the number of area agents:

```bash
export PATH="$PWD/build/deps/sumo/bin:$PATH"
mpiexec -n 2 build/MASHierarchicalSystem \
  --agents 1 --path "$PWD/scenarios/xgrid" --sync 20 \
  --end 120 --epsilon 0.99 --k 1 --reroute 0.85
```

`--help` lists all options. Direct invocation writes outputs under the scenario directory by default; `--tripinfo`, `--edgedata`, and `--vehroutes` also accept absolute paths to existing output directories. The Python runner handles directory creation and output isolation. Use `--scenario scenarios/intas` with the runner to select InTAS; it is substantially larger and may need a longer timeout.

For Slurm, submit `sbatch submit_jobs.sh` from the repository root after configuring the MPI/SUMO environment. The script defaults to a short xgrid run and two tasks. When overriding `AGENTS`, request `AGENTS + 1` tasks with `sbatch --ntasks=...`.

## Verification

The [GitHub Actions workflow](.github/workflows/ci.yml) defines two jobs: dependency-free regression checks under sanitizers, and a Linux build of the pinned dependencies followed by native model-loading, CLI, and xgrid smoke tests. Smoke artifacts are retained for inspection. Workflow configuration is not evidence of a successful hosted CI run; check the repository's Actions results.

Quick checks do not need SUMO, MPI, or XGBoost:

```bash
cmake -S . -B build-tests -DBUILD_ROUTER=OFF
ctest --test-dir build-tests --output-on-failure
```

These checks exercise the actual reroute-selection and reactive-orchestration method bodies against a small state fixture, plus scenario references, partition indices, model feature counts, saved-model versions, setup-cache reuse, interrupted-fetch retries, and dependency rejection. The fixture uses the real graph types but replaces prediction, replanning, and MPI boundaries. It is not an end-to-end correctness proof.

Full CTest runs additionally verify the XGBoost runtime version, model inference, CLI startup, and (when enabled) one- and two-agent simulations. The smoke cases use 30 simulation steps. The default example runs 120 steps and reaches periodic synchronization and rerouting calls; it does not guarantee that congestion triggers a reactive reroute.

## Research context and limits

The paper reports experiments on xgrid and InTAS, including improvements in travel time and fuel consumption. Its tables describe the published experiments. This repository includes subsequent fixes and setup improvements; the short checks here do not reproduce those tables, and no new benchmark results are claimed.

This is a research implementation. Further validation is needed for sustained congested operation, current-edge flow accounting during reactive rerouting, and recovery from worker failures. The code supports cooperative planning but does not implement production fault recovery or prove city-scale robustness through these smoke tests.

## Code map

| Files | Responsibility |
|---|---|
| `main.cpp` | CLI, master/worker startup, SUMO interface and synchronization |
| `Router.cpp`, `Router.h` | Cooperative search, route choice, prediction records and rerouting |
| `CoverGraph.*`, `CoverEdge.*` | Hierarchical path-cover construction and traversal |
| `Graph.h`, `Node.h`, `Edge.h` | Network representation |
| `*Estimator.*` | XGBoost predictors and output transforms |
| `serializations.h` | MPI message tags and data structures |
| `scripts/`, `tests/` | Setup, bounded example runner and verification |

## Citation and licensing

The project's own code, documentation, xgrid assets, and pretrained models are licensed under the [MIT License](LICENSE). The adapted InTAS scenario retains its GPL terms, and manuscript material and external dependencies retain their own notices. See [THIRD_PARTY.md](THIRD_PARTY.md) for scope and provenance, and [CITATION.cff](CITATION.cff) for the paper citation.
