# Progressive symbol naming (partial decomp)

**Practice for every recomp we touch** (SNES, PSX, NES, …): when you identify a
function, hook site, or data object worth manipulating again, **label it in a
durable symbol map** before (or immediately after) using the address in host
code. Do not rely on chat memory, one-off comments, or raw hex sprinkled through
RTL.

Metal Warriors is the reference implementation of this practice. Port the same
shape to other titles; adapt paths/prefixes to the platform.

---

## Cross-recomp rules (always)

1. **One source of truth** — a checked-in map (prefer `recomp/symbols.toml` or
   game-root `symbols.toml`). Addresses are stable IDs; names are regenerated.
2. **Never hand-edit generated C** (`src/gen/`, `generated/`, overlay emit). Fix
   names in the map and regenerate.
3. **Discover → label → manipulate**
   - Discover via always-on rings / coldump / Ghidra (do not pause the runtime
     to “look around”).
   - Label in the map (`func` / `site` / `object` / `struct`+`field`).
   - Manipulate host code through generated macros / named cfg entries
     (`MW_FN_*`, `func Name pc`, …) — not bare `0x8086B6`.
4. **Prefer function entries over mid-function sites.** Sites are fine for
   opcode hooks; promote a real entry when you know the routine boundary.
5. **Gate AOT ownership.** Catalog freely (`emit = false` / comment-only /
   annotations). Only promote to named emitted codegen when the entry is safe
   to own (`emit = true`, or SNES `func` / PSX friendly-name emit).
6. **Structs and globals count.** Gameplay mods need WRAM/object layouts as
   much as function names — keep `[[object]]` / `[[struct]]` / `[[field]]` in
   the same map.
7. **Status is required** — `guessed` | `confirmed` | `hot`. Promote confidence
   when playtested; never delete a label that host code still uses.
8. **If the title lacks the toolchain yet**, add it (map + sync → header/cfg +
   regen hook) before piling on more raw hex. Do not invent a parallel naming
   scheme per session.

### Platform adapters

| Platform | Map → codegen | Host address surface |
|----------|---------------|----------------------|
| **SNES** (this repo) | `symbols.toml` → `bank*.cfg` `func` lines + `*_symbols.h` | `MW_FN_*` / `MW_SITE_*` / `MW_WRAM_*` |
| **SNES** (SMW-style) | Hand-maintained `bank*.cfg` `func` / `name` is acceptable if no toml yet; prefer converging on `symbols.toml` | Named C in `funcs.h` |
| **PSX** | Prefer `symbols.toml` (or promote annotations CSV) → friendly emit names + address aliases; keep `func_XXXXXXXX` as `#define` alias | `PSX_FN_*` / patch site tables |
| **Other** | Same loop: map → sync tool → header/cfg → regen | Prefixed macros, never raw hex in new hooks |

---

## Metal Warriors — reference layout

| Artifact | Role |
|----------|------|
| [`recomp/symbols.toml`](../recomp/symbols.toml) | **Author here** — funcs, hook sites, WRAM, structs |
| `python3 tools/sync_symbols.py` | Writes managed cfg blocks + header + catalog |
| `recomp/bank*.cfg` | snesrecomp seed; managed block is regenerated |
| `recomp/mw_symbols.h` | `MW_FN_*` / `MW_SITE_*` / `MW_WRAM_*` for `mw_rtl.c` |
| [`docs/SYMBOLS_CATALOG.md`](SYMBOLS_CATALOG.md) | Generated table (do not hand-edit) |
| `recomp/funcs.h` | Named AOT decls after `tools/regen.sh` |

`tools/regen.sh` runs `sync_symbols.py` before codegen.

## Workflow (MW)

1. **Discover** a PC or WRAM address (debug rings, coldump, Ghidra).
2. **Label** it in `recomp/symbols.toml`:
   - `[[func]]` — routine **entry** (prefer this over mid-function sites)
   - `[[site]]` / `[[site_group]]` — opcode hook PCs used by `mw_rtl.c`
   - `[[object]]` / `[[struct]]` / `[[field]]` — WRAM and layouts
3. **Sync** — `python3 tools/sync_symbols.py` (or just `bash tools/regen.sh`).
4. **Manipulate** host code via `MW_FN_MwObjectDrawer`, `MW_SITE_DmaSizeBg1`,
   `MW_WRAM_ObjectListHead`, etc. — not raw hex.

### Promoting a function to named AOT C

```toml
[[func]]
bank = 0x80
pc = 0x86B6
name = "MwObjectDrawer"
emit = true          # was false — now writes `func MwObjectDrawer 86B6`
status = "confirmed"
note = "…"
```

Then regen. snesrecomp emits `void MwObjectDrawer(CpuState*)` into
`src/gen/bank80_v2.c` and declares it in `funcs.h`.

Keep `emit = false` (default for new work) until you know the entry is safe
to own as AOT — LLE + opcode hooks still run unlabeled / non-emitted bodies.
Optional: add `end:XXXX` when you know the exclusive end PC.

### Status values

| Status | Meaning |
|--------|---------|
| `guessed` | Plausible; not playtested as a label |
| `confirmed` | Verified enough to trust in hooks/docs |
| `hot` | Touched often by H2H / widescreen / netplay work |

## What not to do

- Do not hand-edit `src/gen/*` or the managed `# >>> BEGIN symbols.toml` blocks.
- Do not rename only in chat or in `mw_rtl.c` comments — put it in
  `symbols.toml` or you will rediscover it next week.
- Do not `emit = true` an entire bank at once; promote entry points as you
  validate them (same discipline as SMW `bank*.cfg` promote-from-skip).
- Do not leave new hook PCs as magic numbers in host RTL when a symbol map
  exists — add the label in the same change.

## Related

- [`docs/H2H_STAGE_PROPS.md`](H2H_STAGE_PROPS.md) — mover / OAM / BG1 behavior
- snesrecomp `func` / `name` grammar — `snesrecomp/recompiler/v2/cfg_loader.py`
