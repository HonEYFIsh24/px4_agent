#!/usr/bin/env bash
# Own a single EV adapter child; shadow is the adapter's default mode.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_DIR/scripts/setup_ros.bash"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
exec /usr/bin/python3 "$PROJECT_DIR/vio/ev_bridge.py" "$@"
