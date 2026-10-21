#!/usr/bin/env bash
set -Eeuo pipefail
(($# == 2)) || exit 64
case "$2" in true|false) ;; *) exit 64 ;; esac
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$SCRIPT_DIR/gui_display.py" --run-directory "$1" --gui "$2"
