#!/usr/bin/env bash
# Read-only PX4 evidence; actual fusion flags and aid-source times, no flight I/O.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_DIR/scripts/setup_ros.bash"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
exec /usr/bin/python3 "$PROJECT_DIR/vio/ekf_monitor.py" --px4-directory "$PX4_DIR" "$@"
