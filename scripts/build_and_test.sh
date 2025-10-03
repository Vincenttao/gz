#!/usr/bin/env bash
# Lightweight helper to build and test the inspection stack with the expected environment.

set -euo pipefail

: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "${SCRIPT_DIR}")"

# Keep behaviour predictable regardless of where the script is invoked from.
cd "${REPO_ROOT}/ws_inspect"

source /opt/ros/jazzy/setup.bash

if [[ "${CONDA_DEFAULT_ENV:-}" != "ros2" ]]; then
    if ! command -v conda >/dev/null 2>&1; then
        echo "[build_and_test] conda not found; cannot activate ros2 environment." >&2
        exit 1
    fi
    # shellcheck disable=SC1091
    source "$(conda info --base)/etc/profile.d/conda.sh"
    conda activate ros2
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GZ_SIM_RESOURCE_PATH="${REPO_ROOT}/ws_inspect/src/sim_world_assets"

colcon build --symlink-install

if command -v ruff >/dev/null 2>&1; then
    ruff check
else
    echo "[build_and_test] ruff not found; skipping lint." >&2
fi

if command -v black >/dev/null 2>&1; then
    black --check .
else
    echo "[build_and_test] black not found; skipping formatting check." >&2
fi

colcon test
colcon test-result --verbose
