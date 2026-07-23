# H2H stage props — identification & manipulation

Living notes for Metal Warriors dual-viewport / netplay **movers** (hazard
platforms, plates) and related bank-`$00B1` objects. Implementation lives in
`src/mw_rtl.c`. Update this file when coldumps or playtests change the model.

**Related env**

| Env | Purpose |
|-----|---------|
| `SNESRECOMP_MW_COLDUMP=path.jsonl` | Structured mover/column JSONL (preferred) |
| `SNESRECOMP_MW_ELEV=1` | Verbose stderr object + VRAM dump (floods terminal) |
| `SNESRECOMP_MW_COLS=1` | Older column / margin logs |

Truncate coldump files before each session (`: > /tmp/mw_a.jsonl`) — dumps append.

---

## 1. Object list

Live list head: WRAM `$1E14` → linked records (stride via `+$14` = next).

Heuristic layout (verified enough for present; Y is `+$04`):

| Off | Field | Notes |
|-----|--------|------|
| `+$00` | flags | bit15 = active |
| `+$02` | world X | |
| `+$04` | world Y | |
| `+$06` | dual / instance tag | **hi-byte** `$01xx`/`$02xx` = viewport owner; **lo-byte** `$02/$04/$06/$08` = instance id, *not* home |
| `+$08` | meta lo | sprite/meta id (e.g. `$C382`) |
| `+$0A` | meta hi / bank | movers use `$00B1` |
| `+$0C` | `c` | often `$0020` on platforms |
| `+$0E` | `e` | state / anim-ish (coldump) |
| `+$10` | `w10` | often `$FFFF` or small counter |
| `+$12` | `w12` | |
| `+$14` | next | |

Object pool addresses commonly fall in `$1934`…`$2000`.

---

## 2. Classification (present layer)

Three distinct classes — do **not** conflate:

| Class | Test | Present policy |
|-------|------|----------------|
| **Mover / stage prop** | bank `$00B1` **and** meta in whitelist | Home-isolated: OAM sticky + BG1 brown blank for foreign peer |
| **Shared item** | bank `$00B1`, meta ≠ `$D5B8`, **not** whitelist | Shared world OAM; multi-tile sticky for **both** peers |
| **Elevator** | meta `$D5B8` | BG2 / other path — **not** stage-prop whitelist |

### Mover meta whitelist (`mw_is_stage_prop_meta`)

Add new metas here only when a platform needs per-peer isolation:

- `$C382` — hazard stripe + brown body (most common)
- `$C39E`
- `$C6A4`
- `$C400`, `$C5F2`, `$C3EC`, `$C3C4`

Bare `meta ≠ 0 && meta ≠ $D5B8` is **wrong** for movers: it tagged pickups
(e.g. `$9FE2` crates) as `local_only` and culled them on the non-home peer.

---

## 3. Home camera (`mw_stage_prop_home_cam`)

Which peer may **draw** the mover OAM / keep BG1 brown.

1. If `+$06` hi-byte is `$01xx` → home **0**; `$02xx` → home **1** (`why=tag`).
2. Else lo-byte-only movers: **nearer dual-slot mech** each frame (`why=near`),
   with soft hysteresis when both mechs exist (`why=hyst`).
3. No mechs yet → home `-1` (`why=none`); blank on both until latch.
4. Single mech visible → `why=one`.

Coldump fields: `h`, `why`, `d0`/`d1` (squared dist to mech0/1), `mechs`.

**Do not** treat lo-byte `$0002/$0004/$0008` as viewport owners — that caused
ghost platforms on the wrong peer.

---

## 4. Two manipulation surfaces

Movers are split across **OAM** and **BG1**:

```
                    ┌─────────────────────┐
   dual drawer ───► │ cam capture (local) │
                    └──────────┬──────────┘
                               │ multi-tile sticky (home only)
                               ▼
                         present OAM
                               │
   $7F map (shared) ──► BG1 rebuild ──► blank foreign brown
                               │              (mw_present_align_stage_prop_bg1)
                               ▼
                         local framebuffer
```

### 4a. OAM (stripe / meta tiles)

- Dual drawer emits into cam0 then cam1; OAM pressure often keeps **1 of N** tiles.
- Capture stores tiles into **multi-tile sticky** (`tile`, `sz`, `mox`, `moy`).
- Present places the **full sticky set** for `home == local_slot` only, from live
  `+$02/+$04` + meta offsets.
- Foreign peer: `skip_own` (no OAM draw).

Coldump: `sn`, `tiles[]`, `draw` (home∧sn∧active), `present.n` / `skip_own`.

### 4b. BG1 (brown body)

- Shared `$7F` still carries every mover’s brown; local rebuild shows all of them.
- Ghosts appear in the **native 4:3 DMA window** (strip cols `0..view`), not the
  west history gutter — edge/widescreen mods do not apply. Object origin is
  often far off-screen (`sx≈−400`); pocket-at-`+$02` never hits those columns.
- **Native-col path:** when foreign origin is outside 4:3 but Y hits the strip,
  blank/filter cols `0..view` only where `$7F`/tile matches the ledge
  fingerprint (`mw_prop_blank_native_strip` + paint-time
  `mw_prop_foreign_ink_tile`). West `col < 0` untouched. Home-prop keepout.
- Also: pocket blank when origin is in-strip; `$42B3(scroll)` kept on
  full-frame (no sticky replace); void → sky `$0200`.

---

## 5. Observed catalog (H2H elev / hazard room)

From `SNESRECOMP_MW_COLDUMP` sessions (gm≈18 after match start). World XY was
**static** (`dwx/dwy=0`); only screen `sx/sy` moved with cameras.

| obj | meta | `+$06` | home | sticky | Observed tiles |
|-----|------|--------|------|--------|----------------|
| `$19F8` | `$C382` | `$0002` | 0 | **sn=0** | — (never captured) |
| `$1A4C` | `$C382` | `$0004` | 1 | **sn=0** | — |
| `$1AF4` | `$C382` | `$0008` | 1 | sn=1 | `$4E` only (incomplete vs stripe+brown) |
| `$1AD8` | `$C39E` | `$0006` | 0 | sn=1 | `$0A` |
| `$19A4` | `$C6A4` | `$0000` | 1 | sn=2 | `$E4`×2 (mox 0 and −15) |

Typical state words on these platforms: `c=$0020`, `e=$000D`/`$0019`/`$0020`,
`w10=$FFFF` or a small counter.

**Healthy reference:** `$C6A4` `$19A4` — multi-tile sticky + correct home draw.
**Weak targets:** `$C382` instances with `sn=0` (no OAM rebuild possible until
capture lands tiles).

---

## 6. Coldump field cheat-sheet

Top-level: `f`, `slot`, `master`, `cam0`/`cam1`, `loc`, `src1`, `src_loc`,
`mechs`, `strip.7f` / `7f_loc` / `vram`, `mism` / **`mism_local`**, `cap`,
`present`, `props`, `items`.

Per prop: `o`, `m`, `bank`, `t6`, `fl`, `h`, `why`, `d0`, `d1`, `sn`, `act`,
`draw`, `wx`, `wy`, `dwx`, `dwy`, `sx`, `sy`, `c`, `e`, `w10`, `w12`,
`bg_try`, `bg_hit`, `tiles[{t,sz,mox,moy}]`.

**Column work:** prefer `mism_local` + `7f_loc` vs `vram` (local cam). Raw
`mism` (world `src1` vs local VRAM) is misleading under full-frame H2H.

**Example filters**

```bash
# Mover identity
jq -c 'select(.props[0].why != null)|{f,slot,mechs,props:[.props[]|{o,m,t6,h,why,sn,draw,sx,sy,tiles}]}' /tmp/mw_a.jsonl

# Sticky-empty actives (manipulation gaps)
jq -c '.props[]?|select(.act==1 and .sn==0)|{o,m,t6,h,why,sx,sy}' /tmp/mw_a.jsonl

# Motion
jq -c '.props[]?|select((.dwx|fabs)+(.dwy|fabs)>0)|{o,m,dwx,dwy,wx,wy}' /tmp/mw_a.jsonl
```

---

## 7. How to manipulate (code map)

| Goal | Where | Notes |
|------|--------|------|
| Treat as mover | `mw_is_stage_prop_meta` | Whitelist meta only |
| Change home rule | `mw_stage_prop_home_cam` | Tag vs nearer-mech / hysteresis |
| Capture completeness | `mw_cam_oam_commit` + `mw_prop_sticky_store` | Widen X/Y for props; accumulate tiles |
| Draw on home peer | `mw_present_oam_from_cam_capture` sticky loop | Full `tiles[]` from live wx/wy |
| Hide on foreign peer | same + `skip_own`; BG1 `mw_present_align_stage_prop_bg1` | OAM skip + brown blank |
| Shared pickup (not mover) | `mw_is_shared_b1_item_meta` + item sticky | Both peers; do not home-isolate |
| Elevator | exclude `$D5B8` from mover path | BG2 / ROM strip |

**Present Y:** `draw=1` with `present.n=0` often means bottom cull (`sy ≳ 224`
after meta). Check `sx`/`sy` in coldump before assuming sticky failure.

---

## 8. Known gaps / open work

1. `$C382` upstairs (`$19F8` / `$1A4C` at wy≈106) — if the dual drawer never
   emits OAM while both cams sit far below, sticky still cannot seed; Y capture
   is widened (−512) and `$F0`-park recovery uses object world Y. Re-check after
   scrolling those platforms on-screen.
2. `$C382` multi-tile completeness (stripe + brown) — may still be 1 of N under
   heavy OAM pressure; sticky accumulates across frames once seeded.
3. Home props at `sy ≳ 224` still `skip_y` OAM (stripe) while BG1 brown remains —
   visual “slab without stripe” on the home peer, not a cross-peer ghost.
4. Widescreen DMA left pad clamps to 0 (VMADD col 0); left columns use shadow /
   `$7F`/snap west fill + VRAM west capture after rebuild — separate from
   mover OAM. H2H prefill ForceTile keys must use raw cam (same as
   `SetWorld`), not `mw_shadow_world(cam, scroll)`. Snap west reads allow
   column remap; snap capture merges prior west when dual stomps the stripe.
5. Items (`$9FE2` etc.) use the shared-item sticky path; despawn sync is
   present-side, not sim desync (netplay stays deterministic).

---

## 9. Changelog

| Date | Note |
|------|------|
| 2026-07-23 | Left gutter: H2H prefill world keys = raw cam (match `SetWorld`); VRAM west capture after rebuild; snap west merge on dual stomp. Earlier: west snap remap; native 4:3 foreign-ink filter; `$42B3` keep; sticky `$8000` wipe. |
| 2026-07-22 | Initial doc from coldump deep ID + H2H mover work: whitelist vs items, nearer-mech home, multi-tile sticky, BG1 blank, elev-room catalog (`$C382`/`$C39E`/`$C6A4`). |
