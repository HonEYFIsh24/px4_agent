#!/usr/bin/env bash
# Keep the validation attached to the desktop session if the caller disconnects.
set -u
if [[ $# != 2 && $# != 3 ]]; then
    echo 'usage: validate_yaw_terminal.sh OUTPUT_DIRECTORY LOG_FILE [success|cancel|reset|drop|pose|nbv]' >&2
    exit 2
fi
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir" || exit 2
output_dir="$1"
log_file="$2"
mode="${3:-success}"
case "$mode" in
    success) validator=scripts/validate_yaw_sitl.py ;;
    cancel) validator=scripts/validate_yaw_cancel_sitl.py ;;
    reset) validator=scripts/validate_yaw_reset_sitl.py ;;
    drop) validator=scripts/validate_yaw_reset_sitl.py ;;
    pose) validator=scripts/validate_yaw_reset_sitl.py ;;
    nbv) validator=scripts/validate_nbv_yaw_sitl.py ;;
    *) echo 'unknown validation mode' >&2; exit 2 ;;
esac
mkdir -p "$(dirname -- "$log_file")"
if [[ -e "$log_file" || -e "$log_file.exit" ]]; then
    echo 'refusing to overwrite existing validation log' >&2
    exit 2
fi
extra=()
[[ "$mode" != drop ]] || extra=(--fault drop)
[[ "$mode" != pose ]] || extra=(--fault pose)
.venv/bin/python "$validator" --execute --output "$output_dir" "${extra[@]}" >"$log_file" 2>&1
result=$?
printf '%s\n' "$result" >"$log_file.exit"
exit "$result"
