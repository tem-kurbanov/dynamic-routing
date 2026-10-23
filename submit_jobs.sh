#!/usr/bin/env bash
#SBATCH --job-name=dynamic-routing-xgrid
#SBATCH --nodes=1
#SBATCH --ntasks=2
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=00:10:00
#SBATCH --output=slurm-%j.out
set -euo pipefail

# Submit from the repository root after building and configuring your MPI/SUMO environment.
# Match --ntasks to AGENTS + 1 when overriding AGENTS.
repo_root="${SLURM_SUBMIT_DIR:-$PWD}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
python3 "$repo_root/scripts/run_example.py" \
    --binary "${BINARY:-$repo_root/build/MASHierarchicalSystem}" \
    --scenario "${SCENARIO:-$repo_root/scenarios/xgrid}" \
    --sumo "${SUMO_BINARY:-sumo}" \
    --agents "${AGENTS:-1}" \
    --steps "${STEPS:-120}" \
    --timeout 540 \
    --output "$repo_root/build/slurm-output"
