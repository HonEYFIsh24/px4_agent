#!/usr/bin/env bash
# Install the Agent interpreter locally; ROS keeps using /usr/bin/python3.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
REQUIREMENTS_FILE=requirements.txt
case "${1:-}" in
    "") ;;
    --desktop) REQUIREMENTS_FILE=requirements-desktop.txt ;;
    *) echo '用法：bash scripts/setup_agent.sh [--desktop]' >&2; exit 2 ;;
esac
if [[ ! -x .venv/bin/python ]]; then
    if command -v python3.11 >/dev/null 2>&1; then
        python3.11 -m venv .venv
    else
        # uv downloads a managed Python without sudo or changing system Python.
        /usr/bin/python3 -m pip install --target "$PROJECT_DIR/.tools" uv
        "$PROJECT_DIR/.tools/bin/uv" python install 3.11
        "$PROJECT_DIR/.tools/bin/uv" venv --python 3.11 .venv
    fi
fi
if [[ -x "$PROJECT_DIR/.tools/bin/uv" ]]; then
    "$PROJECT_DIR/.tools/bin/uv" pip install --python .venv/bin/python -r "$REQUIREMENTS_FILE"
    "$PROJECT_DIR/.tools/bin/uv" pip check --python .venv/bin/python
else
    .venv/bin/python -m pip install -r "$REQUIREMENTS_FILE"
    .venv/bin/python -m pip check
fi
.venv/bin/python -c 'from langchain.agents import create_agent; import agent, web_server'
if [[ "$REQUIREMENTS_FILE" == requirements-desktop.txt ]]; then
    .venv/bin/python -c 'from PySide6 import QtWidgets; import desktop'
fi
if [[ ! -f .env ]]; then
    (umask 077; cp .env.example .env)
fi
echo 'Agent environment ready. Configure the model API key in .env before sending language tasks.'
