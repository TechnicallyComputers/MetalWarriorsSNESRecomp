#!/usr/bin/env bash
# Build src/gen into a zip and publish it to the private CI assets repo.
# Does NOT upload the SNES ROM — only recompiler output needed to link CI builds.
#
# Prereq: local src/gen/*.c (from bash tools/regen.sh --no-tests with your ROM).
#
# Usage:
#   bash tools/publish_ci_src_gen.sh           # zip + push to psxrecomp-ci-assets
#   bash tools/publish_ci_src_gen.sh --dry-run # zip only under dist/ci-assets/
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DRY_RUN=0
ASSETS_REPO="TechnicallyComputers/psxrecomp-ci-assets"
ASSETS_DIR="metalwarriors"

for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown arg: $arg" >&2; exit 2 ;;
  esac
done

cd "$ROOT"
if ! compgen -G 'src/gen/*.c' >/dev/null; then
  echo "error: src/gen/*.c missing — run: bash tools/regen.sh --no-tests" >&2
  exit 1
fi

OUT="$ROOT/dist/ci-assets/${ASSETS_DIR}"
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT
mkdir -p "$OUT" "$STAGE/src-gen"
cp -a src/gen/*.c "$STAGE/src-gen/"
[[ -f src/gen/program_manifest.json ]] && cp -a src/gen/program_manifest.json "$STAGE/src-gen/"
( cd "$STAGE/src-gen" && zip -9 -r "$OUT/src-gen.zip" . )
echo "Wrote $OUT/src-gen.zip ($(du -h "$OUT/src-gen.zip" | cut -f1))"

if [[ "$DRY_RUN" = "1" ]]; then
  echo "(--dry-run) not pushing"
  exit 0
fi

if ! command -v gh >/dev/null 2>&1; then
  echo "error: gh CLI required to push; zip is ready at $OUT/src-gen.zip" >&2
  exit 1
fi

CLONE="$(mktemp -d)"
trap 'rm -rf "$STAGE" "$CLONE"' EXIT
export GIT_LFS_SKIP_SMUDGE=1
TOKEN="$(gh auth token)"
git clone --depth 1 "https://x-access-token:${TOKEN}@github.com/${ASSETS_REPO}.git" "$CLONE"
mkdir -p "$CLONE/${ASSETS_DIR}"
cp -f "$OUT/src-gen.zip" "$CLONE/${ASSETS_DIR}/src-gen.zip"
if [[ ! -f "$CLONE/${ASSETS_DIR}/README.md" ]]; then
  cat > "$CLONE/${ASSETS_DIR}/README.md" <<'EOF'
# Metal Warriors CI assets

- `src-gen.zip` — prebuilt `src/gen/*.c` for GitHub Actions compile.
- **No ROM** is stored here. Refresh with `bash tools/publish_ci_src_gen.sh`
  from MetalWarriorsSNESRecomp after a local regen.
EOF
fi

cd "$CLONE"
git add "${ASSETS_DIR}/src-gen.zip" "${ASSETS_DIR}/README.md"
if git diff --cached --quiet; then
  echo "No changes to publish (remote already has this zip)."
  exit 0
fi
git -c user.email="ci@technicallycomputers.ca" -c user.name="CI assets" \
  commit -m "Update Metal Warriors src-gen.zip for CI (no ROM)."
git push origin HEAD
echo "Published to ${ASSETS_REPO}:${ASSETS_DIR}/src-gen.zip"
