#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_FILE="${PX4_AGENT_WORLD:-$PROJECT_DIR/scene/worlds/obstacle_yard.world}"
# Strip world assignments from forwarded arguments so Gazebo receives one value.
# Explicit caller arguments override the manager environment; the last one wins.
SITL_ARGUMENTS=()
for argument in "$@"; do
    case "$argument" in
        world:=*) WORLD_FILE="${argument#world:=}" ;;
        gazebo_physics_library_path:=*)
            echo 'Private Gazebo library paths must come from the validated experimental backend.' >&2
            exit 1 ;;
        *) SITL_ARGUMENTS+=("$argument") ;;
    esac
done
# Do not export LD_LIBRARY_PATH here: start_sitl.sh sources ROS and PX4 paths.
# The launch group computes its server-only prepend after those setups complete.
case "${PX4_AGENT_GAZEBO_SYNC:-0}" in
    0|"") ;;
    1)
        [[ "${SITL_LAUNCH_FILE:-$PROJECT_DIR/launch/sitl_vio.launch}" == "$PROJECT_DIR/launch/sitl_vio.launch" ]] || {
            echo 'Experimental Gazebo synchronization requires the reviewed project SITL launch.' >&2
            exit 1
        }
        PRIVATE_PHYSICS_LIBRARY_PATH="$(/usr/bin/python3 "$PROJECT_DIR/scripts/validate_gazebo_sync.py" --verify)"
        [[ "$PRIVATE_PHYSICS_LIBRARY_PATH" == "$PROJECT_DIR/vio/gazebo_sync/lib" ]] || {
            echo 'Experimental Gazebo library validator returned an unexpected directory.' >&2
            exit 1
        }
        SITL_ARGUMENTS+=("gazebo_physics_library_path:=$PRIVATE_PHYSICS_LIBRARY_PATH")
        ;;
    *) echo 'PX4_AGENT_GAZEBO_SYNC must be 0 or 1.' >&2; exit 1 ;;
esac
[[ -n "$WORLD_FILE" ]] || { echo 'Gazebo world path cannot be empty.' >&2; exit 1; }
WORLD_FILE="$(readlink -f -- "$WORLD_FILE")"
/usr/bin/python3 "$PROJECT_DIR/vio/generate_assets.py"
if [[ "$WORLD_FILE" == "$PROJECT_DIR/scene/worlds/obstacle_yard.world" ]]; then
    /usr/bin/python3 "$PROJECT_DIR/scene/generate_obstacle_world.py"
fi
if [[ "$WORLD_FILE" == "$PROJECT_DIR/scene/worlds/logistics_yard.world" ]]; then
    /usr/bin/python3 "$PROJECT_DIR/scene/generate_world.py"
fi
[[ -r "$WORLD_FILE" ]] || { echo "Cannot read Gazebo world: $WORLD_FILE" >&2; exit 1; }
export PX4_AGENT_WORLD="$WORLD_FILE"
export GAZEBO_RESOURCE_PATH="$PROJECT_DIR/scene:$PROJECT_DIR/vio:${GAZEBO_RESOURCE_PATH:-}"
if [[ ! -f "$PROJECT_DIR/vio_ws/devel/lib/libuav_body_imu_plugin.so" ]]; then
    echo 'Build the original-IMU ROS plugin first using scripts/build_vio.sh.' >&2
    exit 1
fi
export GAZEBO_PLUGIN_PATH="$PROJECT_DIR/vio_ws/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export SITL_LAUNCH_FILE="${SITL_LAUNCH_FILE:-$PROJECT_DIR/launch/sitl_vio.launch}"
exec bash "$PROJECT_DIR/scripts/start_sitl.sh" \
    sdf:="$PROJECT_DIR/vio/models/iris_vio/iris_vio.sdf" \
    world:="$WORLD_FILE" "${SITL_ARGUMENTS[@]}"
