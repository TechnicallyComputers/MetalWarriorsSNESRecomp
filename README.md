# MetalWarriorsSNESRecomp

Private playtest scaffold: static recompilation of *Metal Warriors* (SNES,
USA) into native C using the [snesrecomp](https://github.com/mstan/snesrecomp)
framework.

This repo is **not** a public release. The ROM is never redistributed — you
supply your own legally dumped copy.

## Expected ROM

| Field | Value |
|-------|-------|
| Title | Metal Warriors (USA) |
| CRC32 | `0xf2ab92d4` |
| SHA-256 | `0d7f875877fe856066cfb39b4ecdbbe7d48393a75770720876c94419f809bb1c` |

Stage the ROM at the repo root as `metalwarriors.sfc` (or
`Metal Warriors (USA).sfc`). 512-byte SMC copier headers are auto-stripped
before hashing.

## Quick start (from source)

```bash
cd MetalWarriorsSNESRecomp
# snesrecomp must be present at ./snesrecomp (symlink to sibling checkout)

# 1. Stage ROM
ln -sf /path/to/Metal\ Warriors\ \(USA\).sfc metalwarriors.sfc

# 2. Regenerate banks (skip framework tests during bring-up)
bash tools/regen.sh --no-tests

# 3. Build (default: configure + link, no AppImage)
bash tools/build-linux.sh --config debug

# 4. Run (opens the RmlUi launcher by default; pass a ROM or --no-launcher to skip)
./build-linux-debug/MetalWarriorsSNESRecomp
# or: ./build-linux-debug/MetalWarriorsSNESRecomp --no-launcher metalwarriors.sfc
```

Debug TCP server listens on port **4380** (avoids collisions with SMW/Zelda/MMX).

## Layout

- `recomp/` — bank `.cfg` seeds + `funcs.h` (synced by regen)
- `src/` — game glue (`mw_rtl.c`, `main.c`, SPC stub, config/UI helpers)
- `src/gen/` — recompiler output (local only; gitignored)
- `snesrecomp/` — framework symlink
- `tools/regen.sh` / `tools/build-linux.sh` — regen + Linux build
- `docs/H2H_STAGE_PROPS.md` — H2H mover/platform identification & manipulation
  (coldump fields, meta whitelist, OAM sticky + BG1 brown)

## Status

LLE-first bring-up: `RunOneFrameOfGame` drives the CPU from the reset/NMI
vectors via `interp_bridge_run_until_quiescent` / `interp_bridge_run_interrupt`.
RmlUi launcher by default (`SkipLauncher=0` / `--no-launcher` to skip). No MSU-1. Opt-in widescreen via `Widescreen=71` in `config.ini`
(~16:9): menus stay centered; gameplay expands with DMA column staging widen plus
`WsShadow` (trailing margins use shadow/extend). Two gamepads enabled in `config.ini`.
