#!/usr/bin/env bash
# Regen pipeline driver for MetalWarriorsSNESRecomp.
#
# Regenerates src/gen/*.c from the recomp/*.cfg configs over a verified
# Metal Warriors (USA) ROM, then syncs recomp/funcs.h.
#
# ROM candidates (first match wins):
#   metalwarriors.sfc
#   Metal Warriors (USA).sfc
#
# Flags:
#   --no-tests             skip the framework test suite (default: run it).
#   --strict-idempotent    regenerate into a temporary directory and require
#                          byte-identical output.
#   -h | --help            this message.
#
# Run from anywhere — paths resolve relative to this script's location.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

RUN_TESTS=1
STRICT_IDEMPOTENT=0
for arg in "$@"; do
  case "$arg" in
    --no-tests) RUN_TESTS=0 ;;
    --strict-idempotent) STRICT_IDEMPOTENT=1 ;;
    -h|--help)  sed -n '2,/^set -euo/p' "$0" | sed -n '/^# /p' | sed 's/^# //'; exit 0 ;;
    *) echo "regen.sh: unknown flag: $arg (try --help)" >&2; exit 2 ;;
  esac
done

cd "$ROOT"

ROM=""
for cand in "metalwarriors.sfc" "Metal Warriors (USA).sfc"; do
  if [ -f "$cand" ]; then
    ROM="$cand"
    break
  fi
done

SNESRECOMP_ROOT="${SNESRECOMP_ROOT:-snesrecomp}"
TESTS="$SNESRECOMP_ROOT/tests/run_tests.py"

PYTHON="${PYTHON:-$(command -v python3 || command -v python || true)}"
if [ -z "$PYTHON" ]; then
  echo "regen.sh: no python3/python interpreter found on PATH" >&2
  exit 1
fi

if [ -z "$ROM" ]; then
  echo "regen.sh: no ROM found — stage metalwarriors.sfc or 'Metal Warriors (USA).sfc' at the repo root." >&2
  exit 1
fi

if [ ! -f "$SNESRECOMP_ROOT/tools/v2_emit.py" ]; then
  echo "regen.sh: snesrecomp is not initialized (missing $SNESRECOMP_ROOT/tools/v2_emit.py)." >&2
  exit 1
fi

step() { echo; echo "=== $* ==="; }

step "Regenerating banks from $ROM"
# --cfg-roots: every declared `func` seeds the analysis closure so the proven
# surface is materialized as AOT; the interpreter covers the unprovable remainder.
"$PYTHON" "$SNESRECOMP_ROOT/tools/v2_emit.py" --rom "$ROM" \
    --cfg-dir recomp --out-dir src/gen --cfg-roots

step "Syncing funcs.h"
"$PYTHON" "$SNESRECOMP_ROOT/tools/v2_sync_funcs_h.py" --cfg-dir recomp \
    --out recomp/funcs.h

if [ "$STRICT_IDEMPOTENT" -eq 1 ]; then
  step "Idempotency check: regen into temp dir + byte-compare"
  TMP_GEN="$(mktemp -d)"
  trap 'rm -rf "$TMP_GEN"' EXIT
  "$PYTHON" "$SNESRECOMP_ROOT/tools/v2_emit.py" --rom "$ROM" \
      --cfg-dir recomp --out-dir "$TMP_GEN" --cfg-roots
  # Match the tracked placeholder retained in the published output directory.
  : > "$TMP_GEN/.gitkeep"
  "$PYTHON" "$SNESRECOMP_ROOT/tools/v2_compare_output.py" \
      --expected src/gen --actual "$TMP_GEN"
fi

if [ "$RUN_TESTS" -eq 1 ]; then
  step "Framework tests"
  "$PYTHON" "$TESTS"
fi

step "Done"
