#!/usr/bin/env bash
# Launches trossen_live_monitor.py regardless of the caller's cwd.
# Usage: ./run.sh [--demo] [--host 0.0.0.0] [--port 5001]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$SCRIPT_DIR/.venv/bin/activate" ]; then
    source "$SCRIPT_DIR/.venv/bin/activate"
elif [ -f "$SCRIPT_DIR/venv/bin/activate" ]; then
    source "$SCRIPT_DIR/venv/bin/activate"
elif [ -f "$SCRIPT_DIR/../trossen_env/bin/activate" ]; then
    source "$SCRIPT_DIR/../trossen_env/bin/activate"
fi

cd "$SCRIPT_DIR"
exec python3 "$SCRIPT_DIR/scripts/trossen_live_monitor.py" "$@"
