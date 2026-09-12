#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "$HOME/codex/codex-jaka_mini_2/scripts/env.bash" ]; then
  source "$HOME/codex/codex-jaka_mini_2/scripts/env.bash" >/dev/null
fi

exec python3 "$ROOT/gemini_jaka_test.py" "$@"
