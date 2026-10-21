#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PROJECT_DIR/logs/ros}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
VIO_LOG_DIR="${UAV_VIO_LOG_DIR:-$PROJECT_DIR/logs/vio}"
mkdir -p "$VIO_LOG_DIR"
if [[ ! -f "$PROJECT_DIR/vio_ws/devel/setup.bash" || ! -x "$PROJECT_DIR/vio_ws/devel/lib/vins/vins_node" ]]; then
    echo 'Build VINS-Fusion first using scripts/build_vio.sh.' >&2
    exit 1
fi
source "$PROJECT_DIR/vio_ws/devel/setup.bash"
/usr/bin/python3 "$PROJECT_DIR/vio/generate_assets.py"
vins_pid=''
cleanup() {
    if [[ -n "$vins_pid" ]]; then
        kill -INT "$vins_pid" 2>/dev/null || true
        wait "$vins_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
if [[ "${1:-}" == "--check" ]]; then
    /usr/bin/python3 "$PROJECT_DIR/vio/check_inputs.py"
    exit 0
fi
echo "VINS-Fusion log: $VIO_LOG_DIR/vins.log"
rosrun vins vins_node "$PROJECT_DIR/vio/config/stereo_imu.yaml" > "$VIO_LOG_DIR/vins.log" 2>&1 &
vins_pid=$!
# Keep VINS image subscriptions alive across the input check. Gazebo's
# multicamera plugin otherwise deactivates the sensor when the checker leaves.
# The GUI and camera publishers can still be warming up when the first images
# arrive. Startup diagnostics must not turn a temporary transport delay into
# termination of an otherwise running estimator. --check above remains the
# strict, standalone acceptance command and propagates any failed assertion.
echo 'Waiting 5 seconds for the stereo camera publishers to warm up.'
sleep 5
input_check_passed=0
for attempt in 1 2; do
    if ! kill -0 "$vins_pid" 2>/dev/null; then
        echo "VINS-Fusion exited during startup; see $VIO_LOG_DIR/vins.log." >&2
        wait "$vins_pid"
        exit 1
    fi
    echo "Strict VIO input startup check, attempt $attempt/2."
    if /usr/bin/python3 "$PROJECT_DIR/vio/check_inputs.py"; then
        input_check_passed=1
        break
    fi
    echo "WARNING: VIO input startup check $attempt/2 failed; estimator is still running." >&2
    if [[ "$attempt" == 1 ]]; then
        echo 'Waiting 5 seconds before retrying the same strict input check.'
        sleep 5
    fi
done
if [[ "$input_check_passed" != 1 ]]; then
    echo 'WARNING: VIO inputs did not pass startup acceptance. Keeping VINS-Fusion running; check tracking health and rerun scripts/start_vio.sh --check after camera warm-up.' >&2
fi
wait "$vins_pid"
