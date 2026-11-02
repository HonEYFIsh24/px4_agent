#!/usr/bin/env bash
# Keep an authorized flight validator attached to the desktop session.
set -u
if [[ $# -lt 3 ]]; then
    echo 'usage: validate_autonomous_terminal.sh OUTPUT_DIRECTORY LOG_FILE PROMPT [VALIDATOR_OPTIONS...]' >&2
    exit 2
fi
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir" || exit 2
output_dir="$1"
log_file="$2"
prompt="$3"
shift 3
mkdir -p "$(dirname -- "$log_file")"
if [[ -e "$output_dir" || -e "$log_file" || -e "$log_file.exit" ]]; then
    echo 'refusing to overwrite existing validation evidence' >&2
    exit 2
fi
.venv/bin/python scripts/validate_autonomous_search.py --execute --output "$output_dir" --prompt "$prompt" "$@" >"$log_file" 2>&1
result=$?
printf '%s\n' "$result" >"$log_file.exit"
exit "$result"
