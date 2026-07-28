#!/usr/bin/env bash
# Stage a Metal Warriors release zip next to the built runtime (no ROM).
#
# Usage:
#   scripts/package_release.sh <build-dir> <artifact-tag>
# Example:
#   scripts/package_release.sh build-ci linux-x64
#
# Writes: dist/metalwarriors-<VERSION>-<artifact-tag>.zip

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${1:-}"
ARTIFACT_TAG="${2:-}"

if [[ -z "${BUILD_DIR}" || -z "${ARTIFACT_TAG}" ]]; then
  echo "usage: $0 <build-dir> <artifact-tag>" >&2
  exit 2
fi

VERSION="$(tr -d '[:space:]' < "${ROOT}/VERSION")"
if [[ -z "${VERSION}" ]]; then
  echo "VERSION file is empty" >&2
  exit 1
fi

BUILD_DIR="$(cd "${BUILD_DIR}" && pwd)"
DIST="${ROOT}/dist"
STAGE="${DIST}/stage-${ARTIFACT_TAG}"
ZIP_NAME="metalwarriors-${VERSION}-${ARTIFACT_TAG}.zip"
EXE_NAME="MetalWarriorsSNESRecomp"

rm -rf "${STAGE}"
mkdir -p "${STAGE}" "${DIST}"
rm -f "${DIST}/${ZIP_NAME}"

EXE=""
for cand in \
  "${BUILD_DIR}/${EXE_NAME}" \
  "${BUILD_DIR}/${EXE_NAME}.exe" \
  "${BUILD_DIR}/Release/${EXE_NAME}.exe" \
  "${BUILD_DIR}/Debug/${EXE_NAME}.exe"
do
  if [[ -f "${cand}" ]]; then
    EXE="${cand}"
    break
  fi
done

if [[ -z "${EXE}" ]]; then
  echo "error: ${EXE_NAME} not found under ${BUILD_DIR}" >&2
  ls -la "${BUILD_DIR}" >&2 || true
  exit 1
fi

cp -a "${EXE}" "${STAGE}/"
STAGE_EXE="${STAGE}/$(basename "${EXE}")"
EXE_DIR="$(dirname "${EXE}")"

# recomp-ui POST_BUILD stages flat assets/fonts + assets/img next to the exe.
if [[ ! -d "${EXE_DIR}/assets/fonts" || ! -d "${EXE_DIR}/assets/img" ]]; then
  echo "error: ${EXE_DIR}/assets/{fonts,img} missing — rebuild ${EXE_NAME}" >&2
  exit 1
fi
mkdir -p "${STAGE}/assets"
cp -a "${EXE_DIR}/assets/fonts" "${STAGE}/assets/"
cp -a "${EXE_DIR}/assets/img" "${STAGE}/assets/"

if [[ ! -f "${STAGE}/assets/fonts/LatoLatin-Regular.ttf" ]]; then
  echo "error: assets/fonts incomplete (missing LatoLatin-Regular.ttf)" >&2
  exit 1
fi
if [[ ! -f "${STAGE}/assets/img/boxart.tga" ]]; then
  if [[ -f "${ROOT}/recomp/launcher/boxart.tga" ]]; then
    cp -a "${ROOT}/recomp/launcher/boxart.tga" "${STAGE}/assets/img/boxart.tga"
  else
    echo "error: assets/img/boxart.tga missing" >&2
    exit 1
  fi
fi

# Optional keybinds / README next to the binary.
[[ -f "${EXE_DIR}/keybinds.ini" ]] && cp -f "${EXE_DIR}/keybinds.ini" "${STAGE}/"
[[ -f "${ROOT}/README.md" ]] && cp -f "${ROOT}/README.md" "${STAGE}/"
cp -f "${ROOT}/VERSION" "${STAGE}/"

bundle_mingw_dlls() {
  local exe="$1"
  local objdump=""
  local dll src
  local -a needed=()

  if command -v x86_64-w64-mingw32-objdump >/dev/null 2>&1; then
    objdump="x86_64-w64-mingw32-objdump"
  elif command -v objdump >/dev/null 2>&1; then
    objdump="objdump"
  else
    echo "warning: no objdump; skipping MinGW DLL bundling" >&2
    return 0
  fi

  local runtime_bin="${MINGW_PREFIX:-/mingw64}/bin"
  mapfile -t needed < <(
    "${objdump}" -p "${exe}" 2>/dev/null \
      | awk '/DLL Name:/{print $3}' \
      | grep -viE '^(KERNEL32|USER32|GDI32|ADVAPI32|SHELL32|OLE32|OLEAUT32|WS2_32|WINMM|IMM32|SETUPAPI|VERSION|OPENGL32|COMCTL32|COMDLG32|RPCRT4|SHLWAPI|CRYPT32|BCRYPT|IPHLPAPI|NSI|DNSAPI|MSVCRT|UCRTBASE|VCRUNTIME|DBGHELP|API-MS-).*\.DLL$' \
      | sort -u
  ) || true

  needed+=(
    SDL2.dll
    libgcc_s_seh-1.dll
    libstdc++-6.dll
    libwinpthread-1.dll
    libssp-0.dll
  )

  local -A seen=()
  local -a unique=()
  for dll in "${needed[@]}"; do
    [[ -n "${dll}" ]] || continue
    local key
    key="$(printf '%s' "${dll}" | tr '[:upper:]' '[:lower:]')"
    [[ -n "${seen[$key]:-}" ]] && continue
    seen[$key]=1
    unique+=("${dll}")
  done

  for dll in "${unique[@]}"; do
    src=""
    if [[ -f "${EXE_DIR}/${dll}" ]]; then
      src="${EXE_DIR}/${dll}"
    elif [[ -f "${BUILD_DIR}/${dll}" ]]; then
      src="${BUILD_DIR}/${dll}"
    elif [[ -f "${runtime_bin}/${dll}" ]]; then
      src="${runtime_bin}/${dll}"
    else
      if "${objdump}" -p "${exe}" 2>/dev/null | grep -qi "DLL Name:[[:space:]]*${dll}"; then
        echo "error: required DLL missing: ${dll}" >&2
        echo "  looked in ${EXE_DIR}, ${BUILD_DIR}, ${runtime_bin}" >&2
        exit 1
      fi
      continue
    fi
    if ! "${objdump}" -p "${exe}" 2>/dev/null | grep -qi "DLL Name:[[:space:]]*${dll}"; then
      continue
    fi
    cp -f "${src}" "${STAGE}/"
    echo "bundled ${dll}"
  done
}

if [[ "$(basename "${STAGE_EXE}")" == *.exe ]]; then
  bundle_mingw_dlls "${STAGE_EXE}"
fi

# macOS: rewrite non-system dylibs next to the binary when dylibbundler exists.
if [[ "$(uname -s)" == "Darwin" ]] && command -v dylibbundler >/dev/null 2>&1; then
  mkdir -p "${STAGE}/libs"
  dylibbundler -od -b -x "${STAGE_EXE}" -d "${STAGE}/libs" -p "@executable_path/libs/" \
    || echo "warning: dylibbundler failed; macOS zip may need Homebrew SDL2" >&2
fi

cat > "${STAGE}/README.txt" <<EOF
Metal Warriors Recompiled ${VERSION}
Platform pack: ${ARTIFACT_TAG}

This build does NOT include the SNES ROM.
On first launch, select your legally obtained Metal Warriors (USA) ROM
(.sfc / .smc). Expected SHA-256:
  0d7f875877fe856066cfb39b4ecdbbe7d48393a75770720876c94419f809bb1c

Netplay lobbies match on game title + this VERSION string.
Online lobbies need ICE (libjuice) — this pack is built with
SNESRECOMP_NET_ICE=ON.
EOF

(
  cd "${STAGE}"
  if command -v zip >/dev/null 2>&1; then
    zip -r -q "${DIST}/${ZIP_NAME}" .
  else
    echo "error: zip not found; install zip to package releases" >&2
    exit 1
  fi
)

rm -rf "${STAGE}"
echo "Wrote ${DIST}/${ZIP_NAME}"
ls -la "${DIST}/${ZIP_NAME}"
