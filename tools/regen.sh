#!/usr/bin/env bash
# Regen pipeline driver for MetalWarriorsSNESRecomp.
#
# Regenerates src/gen/*.c from the recomp/*.cfg configs over a verified
# Metal Warriors (USA) ROM, then syncs recomp/funcs.h.
#
# Uses the snesrecomp local codegen SDK (snesrecomp_cli.py generate).
#
# ROM candidates (first match wins):
#   metalwarriors.sfc
#   Metal Warriors (USA).sfc
#
# Flags:
#   --no-tests             skip the framework test suite (default: run it).
#   --strict-idempotent    regenerate into a temporary directory and require
#                          byte-identical output.
#   --json-progress        forward SDK JSONL progress on stdout.
#   -h | --help            this message.
#
# Run from anywhere — paths resolve relative to this script's location.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Metal Warriors (USA) — keep in sync with README / catalog / main.c.
EXPECTED_CRC32="${SNESRECOMP_EXPECTED_CRC32:-f2ab92d4}"
EXPECTED_SHA256="${SNESRECOMP_EXPECTED_SHA256:-0d7f875877fe856066cfb39b4ecdbbe7d48393a75770720876c94419f809bb1c}"

RUN_TESTS=1
STRICT_IDEMPOTENT=0
JSON_PROGRESS=0
for arg in "$@"; do
  case "$arg" in
    --no-tests) RUN_TESTS=0 ;;
    --strict-idempotent) STRICT_IDEMPOTENT=1 ;;
    --json-progress) JSON_PROGRESS=1 ;;
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
CLI="$SNESRECOMP_ROOT/snesrecomp_cli.py"
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

if [ ! -f "$CLI" ]; then
  echo "regen.sh: snesrecomp CLI missing ($CLI). Init the snesrecomp submodule." >&2
  exit 1
fi

step() { echo; echo "=== $* ==="; }

sdk_generate() {
  local out_dir="$1"
  local -a args=(
    generate
    --project-root "$ROOT"
    --rom "$ROM"
    --cfg-dir recomp
    --out-dir "$out_dir"
    --funcs-h recomp/funcs.h
    --cfg-roots
    --expected-crc32 "$EXPECTED_CRC32"
    --expected-sha256 "$EXPECTED_SHA256"
  )
  if [ "$JSON_PROGRESS" -eq 1 ]; then
    args+=(--json-progress)
  fi
  "$PYTHON" "$CLI" "${args[@]}"
}

step "Regenerating banks from $ROM (snesrecomp generate)"
sdk_generate src/gen

if [ "$STRICT_IDEMPOTENT" -eq 1 ]; then
  step "Idempotency check: regen into temp dir + byte-compare"
  TMP_GEN="$(mktemp -d)"
  trap 'rm -rf "$TMP_GEN"' EXIT
  # Skip funcs.h rewrite on the temp pass — compare generated C only.
  JSON_FLAG=()
  if [ "$JSON_PROGRESS" -eq 1 ]; then
    JSON_FLAG=(--json-progress)
  fi
  "$PYTHON" "$CLI" generate \
      --project-root "$ROOT" \
      --rom "$ROM" \
      --cfg-dir recomp \
      --out-dir "$TMP_GEN" \
      --cfg-roots \
      --expected-crc32 "$EXPECTED_CRC32" \
      --expected-sha256 "$EXPECTED_SHA256" \
      "${JSON_FLAG[@]}"
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
