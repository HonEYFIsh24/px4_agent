#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
AGENT_PYTHON="${AGENT_PYTHON:-$PROJECT_DIR/.venv/bin/python}"
if [[ ! -x "$AGENT_PYTHON" ]]; then
    AGENT_PYTHON=/home/dut/ros_agent/mcp-client-langchain/.venv/bin/python
fi
if [[ ! -x "$AGENT_PYTHON" ]]; then
    echo 'Create a Python 3.11 venv with requirements.txt, or set AGENT_PYTHON.' >&2
    exit 1
fi
exec "$AGENT_PYTHON" "$PROJECT_DIR/web_server.py" "$@"
