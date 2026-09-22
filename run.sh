#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [ -n "${SENTINEL_PYTHON:-}" ]; then
  exec "$SENTINEL_PYTHON" -m sentinel "$@"
elif [ -x .venv/bin/python ]; then
  exec .venv/bin/python -m sentinel "$@"
elif python3 -c 'import numpy, PIL' >/dev/null 2>&1; then
  exec python3 -m sentinel "$@"
else
  sentinel_runtime="/Users/mark/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3"
  if [ -x "$sentinel_runtime" ]; then
    exec "$sentinel_runtime" -m sentinel "$@"
  fi
  printf '%s\n' 'Install Python 3.10+, then: python3 -m pip install -e .' >&2
  exit 1
fi
