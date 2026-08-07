# H2H stage props — identification & manipulation

Living notes for Metal Warriors dual-viewport / netplay **movers** (hazard
platforms, plates) and related bank-`$00B1` objects. Implementation lives in
`src/mw_rtl.c`. Update this file when coldumps or playtests change the model.

**Naming:** durable PC / WRAM labels live in [`recomp/symbols.toml`](../recomp/symbols.toml)
(`MwObject` layout, `MW_WRAM_*`, drawer sites). Workflow: [`docs/SYMBOLS.md`](SYMBOLS.md).

**Related env**

| Env                                | Purpose                                             |
| ---------------------------------- | --------------------------------------------------- |
| `SNESRECOMP_MW_COLDUMP=path.jsonl` | Structured mover/column JSONL (preferred)           |
| `SNESRECOMP_MW_COLDUMP_EVERY=1`    | Emit every frame (1-frame snaps)                    |
| `SNESRECOMP_MW_H2H_OFFLINE_LOCAL_FULL` | Offline dual LocalFull: unset=auto with COLDUMP; `1` force; `0` keep split |
| `SNESRECOMP_MW_H2H_OFFLINE_SLOT=0\|1` | Which cam LocalFull/coldump presents offline (default 0) |
| `SNESRECOMP_MW_PROP_PIN_DY`        | Extra pixels on pinned `$C382` present `moy` — **leave unset** (step1 regress) |
| `SNESRECOMP_MW_PROP_C39E_DX`/`_DY` | List-prop probe — **skip** (step2 wrong meta / not stand) |
| `SNESRECOMP_MW_STAND_BG1`          | Map/BG1 stand probe: `dump` / `mark` / `blank` (see §8b step 3) |
| `SNESRECOMP_MW_STAND_BG1_AT`       | Origin: `auto` (default) / `mid` / `pres` / `focus` / `mech` |
| `SNESRECOMP_MW_STAND_BG1_WX`/`_WY` | Optional world origin override |
| `SNESRECOMP_MW_ELEV=1`             | Verbose stderr object + VRAM dump (floods terminal) |
| `SNESRECOMP_MW_COLS=1`             | Older column / margin logs                          |

Truncate before each session (`: > /tmp/mw_a.jsonl`) — dumps append. Trust
**`slot`**, not the filename. Full triage / jq: **§6**.

---

## 1. Object list

Live list head: WRAM `$1E14` → linked records (stride via `+$14` = next).

Heuristic layout (verified enough for present; Y is `+$04`):

| Off    | Field               | Notes                                                                                                 |
| ------ | ------------------- | ----------------------------------------------------------------------------------------------------- |
| `+$00` | flags               | bit15 = active                                                                                        |
| `+$02` | world X             |                                                                                                       |
| `+$04` | world Y             |                                                                                                       |
| `+$06` | dual / instance tag | **hi-byte** `$01xx`/`$02xx` = viewport owner; **lo-byte** `$02/$04/$06/$08` = instance id, _not_ home |
| `+$08` | meta lo             | sprite/meta id (e.g. `$C382`)                                                                         |
| `+$0A` | meta hi / bank      | movers use `$00B1`                                                                                    |
| `+$0C` | `c`                 | often `$0020` on platforms                                                                            |
| `+$0E` | `e`                 | state / anim-ish (coldump)                                                                            |
| `+$10` | `w10`               | often `$FFFF` or small counter                                                                        |
| `+$12` | `w12`               |                                                                                                       |
| `+$14` | next                |                                                                                                       |

Object pool addresses commonly fall in `$1934`…`$2000`.

---

## 2. Classification (present layer)

Three distinct classes — do **not** conflate:

| Class                  | Test                                            | Present policy                                                                                                                                        |
| ---------------------- | ----------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Mover / stage prop** | bank `$00B1` **and** meta in whitelist          | **Home-only OAM** (`skip_own` when `home != local`). BG1: always blank foreign brown; keepout is **home-only** (FOV keepout left P1 ledge dirt on P2) |
| **Shared item**        | bank `$00B1`, meta ≠ `$D5B8`, **not** whitelist | Shared world OAM; multi-tile sticky for **both** peers                                                                                                |
| **Elevator**           | meta `$D5B8`                                    | BG2 / other path — **not** stage-prop whitelist                                                                                                       |

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

1. **Nearer dual-slot mech** (soft until both mechs, then firm). **Ignore**
   mover `+$06` hi-byte — `$C6A4` often carries `$0200` (`why=tag` historically)
   and forced home=P2 while live `+$02` cam-glued to that peer. Mech finder
   still uses hi-byte via `mw_obj_stage_prop_owner`.
2. Mid ledge (637/832 @ 365): **exclusive cam FOV** beats nearer-mech and may
   reassign when firm (P2 was owning P1's view and drawing P1-clipped sticky).
   Handoff-lock catalog (`$C39E` @ 1118,490) locks forever. Other firm props:
   **no flip while still in the home peer's FOV**; else **16×** nearer.
3. No mechs yet → home `-1` (`why=none`); blank on both until latch.
4. Single mech, not yet firm → `why=one`.

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
  Match key is **tile id + mox/moy (±2)** — id-only merge half-culled `$C6A4`
  (`$E4`×2 → `sn=1` flicker).
- **`mox`** = drawer `sx − (owx − live $1E16/$1E1A)`; **latches once** for sane
  meta (`|mox|,|moy| ≤ 48`). Seed **only from home drawer** (`owner == draw_cam`)
  — empty+foreign-FOV seed latched P1-viewport-clipped tiles that P2 drew
  (same bottom cut as P1's screen). Polluted sticky scrubbed at present.
  Mid-ledge home flip clears sticky on both ends.
- Present: world XY − NMI local_cam + latched mox. When live `+$02` **cam-glues**
  to the home camera (`sx` stuck while `dwx≈dcam`), use a latched **anchor**
  instead of live WRAM (stops cam-following ghosts on fixed brown). Detect on
  **any** cam step; **world-lock wins** when both gates match (1px pans:
  `dwx=0` + `dsx=-dcam`). 2-frame clear hysteresis; non-`$C6A4` drop if
  `|live−anchor|>48`. **`$C6A4`:** if live is on a mech (`dx≈±4`, `dy≈−14`)
  it is a **back-carried pickup** — present follows **live** (sticky `$E4`×2
  only; no brown). Free-floating `$C6A4` (not near either mech) still
  world-locks X+Y. Do not treat on-mech `$C6A4` as the giant stand.
  Foreign blanks `use`+live+trail; VRAM residue FP only for free `$C6A4`.
- **Never CHR-allowlist untagged OAM as “platform.”** Coldump `$0A/$0C/$0E/
$20/$28…` under feet with `s:0` / `present.n=0` are **mech body CHR**
  (bank `$00AD`), not the stand. World-locking them by tile id freezes
  half the mech while `$06/$08/$E6…` still track — sliced presentation.
  Platform present = object path (`local_only` + whitelist meta / sticky
  `$E4`) or map-brown BG1/`$7F` when `near[]` is mechs-only (§6b).
- Foreign peer: always `skip_own` (no OAM). BG1 foreign brown **always** blanked
  at live, trail, and anchor.

Coldump: `sn`, `tiles[]`, `draw`, `present.n` / `skip_own`,
`scroll.{loc,snap,hs0,hs1,d01,dsnap}` (present loc vs BG1 snap cam; `$1E32`
is not P2 world cam).

### 4b. BG1 (brown body)

- Shared `$7F` still carries every mover’s brown; local rebuild shows all of them.
- **Full-frame H2H:** live `$7F` first; **snap fills voids** (disabling snap
  punched sky holes through terrain — do not use strip rebuild as a sway lever).
- No tile-ID body stamp / fingerprint paint (corrupts map).
- **Home brown latch:** free `$C6A4` (not on-mech) + **pinned `$C382`** —
  opaque `$7F` pocket when solid (≥4); paint at present `use_wx/wy`
  (**pins: catalog `+$02/+$04` only**). On-mech `$C6A4` backpack skips
  brown (origin sky). Scrub prior paint on XY move; clear on foreign home /
  sticky death. **Display-only** (VRAM) — not collision.
- **Foreign blank:** pin catalog uses wide east FP + VRAM residue match (global
  wide-$C382 FP stays off — wall scramble). Present BG1 scroll is already `cam`
  (`h0=loc`); coldump `hs0` is NMI-native pre-present, not the drawn scroll.
- Ghosts in native 4:3: foreign blank/filter + **home-only** keepout (no
  foreign-in-FOV keep). **Do not** blank home BG1 / ignore-keepout self-blank
  — 2026-08-04 experiment shattered floors/walls and missed `$D5B8` elevators
  (BG2 path; not stage-prop BG1).

---

## 5. Observed catalog (H2H elev / hazard room)

From `SNESRECOMP_MW_COLDUMP` sessions (gm≈18 after match start). World XY was
**static** (`dwx/dwy=0`); only screen `sx/sy` moved with cameras.

| obj     | meta    | `+$06`  | home | sticky   | Observed tiles                          |
| ------- | ------- | ------- | ---- | -------- | --------------------------------------- |
| `$19F8` | `$C382` | `$0002` | 0    | **sn=0** | — (never captured)                      |
| `$1A4C` | `$C382` | `$0004` | 1    | **sn=0** | —                                       |
| `$1AF4` | `$C382` | `$0008` | 1    | sn=1     | `$4E` only (incomplete vs stripe+brown) |
| `$1AD8` | `$C39E` | `$0006` | 0    | sn=1     | `$0A`                                   |
| `$19A4` | `$C6A4` | `$0000` | 1    | sn=2     | `$E4`×2 (mox 0 and −15)                 |

Typical state words on these platforms: `c=$0020`, `e=$000D`/`$0019`/`$0020`,
`w10=$FFFF` or a small counter.

**Healthy reference:** `$C6A4` `$19A4` — multi-tile sticky + correct home draw.
**Weak targets:** `$C382` instances with `sn=0` (no OAM rebuild possible until
capture lands tiles).

---

## 6. Coldump — session, triage, fields

Env: `SNESRECOMP_MW_COLDUMP=/tmp/mw_a.jsonl` (append-only JSONL). Prefer
`SNESRECOMP_MW_COLDUMP_EVERY=1` for 1-frame snaps. Binary:
`./build-linux-prod/MetalWarriorsSNESRecomp`.

### 6a. Session hygiene

| Rule                                  | Why                                                                                                                                                              |
| ------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `: > /tmp/mw_a.jsonl` before each run | Dumps **append**; mixed sessions lie                                                                                                                             |
| Trust **`slot`**, never the filename  | Peer A may write `mw_b.jsonl`; always `select(.slot==…)`                                                                                                         |
| Key props by `(m,wx,wy)`, not `o`     | Pool slots recycle (`o=$1A68` is not a stable id)                                                                                                                |
| `jq` meta is **decimal**              | `$C6A4` → `50852`, `$C382` → `50050`, `$C38C` → `50060`, `$C3EC` → `50156`, `$C39E` → `50078`, `$D5B8` → `54712`. Use `(.m\|tostring)` or `tostring\|sub("^";"0x")` if you want hex |

```bash
: > /tmp/mw_a.jsonl
# Offline dual H2H: COLDUMP auto-enables LocalFull present (map-col diag).
# P2 stand → OFFLINE_SLOT=1. Keep authentic split: OFFLINE_LOCAL_FULL=0.
SNESRECOMP_MW_COLDUMP=/tmp/mw_a.jsonl SNESRECOMP_MW_COLDUMP_EVERY=1 \
SNESRECOMP_MW_H2H_OFFLINE_SLOT=1 \
  ./build-linux-prod/MetalWarriorsSNESRecomp
```

### 6b. First triage — which surface is wrestling?

Cam-follow / snap under a peer’s feet is **two different bugs**. Classify the
dump before touching glue or BG1 strip code.

| Signal             | On-mech **backpack**                         | BG1 present stand (giant brown)                    |
| ------------------ | -------------------------------------------- | -------------------------------------------------- |
| `near[]`           | `cls=item`/`prop`, `dx≈±4`, **`dy≈-14`**     | Mechs ± backpack — **no** stand prop               |
| Meta               | Often `$C6A4` (`50852`); soak also `$C38C` (`50060`) on pool `$1A68` | Not in whitelist under feet                        |
| Sticky             | `$E4`×2                                      | `plat.stripe` never `$E4`; hazard `$62`/`$C8` only |
| Feet / `plat.feet` | —                                            | Often **sky** at collision (`weak=1`)              |
| Present            | Follow **live**                              | `plat.pres` FOV brown blob (`odx`/`ody` vs `coll`) |
| Misread            | World-lock backpack                          | Locating present at **feet** (collision ≠ present) |

**`plat` model (2026-08-05):** feet / near `$C382` is a **gate** (`coll`).
Present ink is a contiguous FOV run (`pres`, src `7f`|`vram`) scored by
**near-collision + on-screen** — not widest far-east shelf (false latch
`odx≈167` / `sx=342`). `odx`/`ody` = present − collision. `pres.ok=0` when
only far map brown exists in `$7F`/VRAM (stand ink missing from both). Lag:
`pres_world` vs `pres_lock`. `body` mirrors `pres`.

**Worked example:** P2 on stand, `feet.weak=1`, `pres.ok=1` with large
`odx`/`ody`, stripe `$E4` absent → map-brown present blob, not `$C6A4`.

### 6c. Field cheat-sheet

Top-level: `f`, **`slot`**, `master`, `cam0`/`cam1`, `loc`, `src1`, `src_loc`,
`mechs`, `strip.7f` / `7f_loc` / `vram`, `mism` / **`mism_local`**, `cap`,
`present`, **`pscroll`**, **`poam`**, `props`, `items`, `elevs`, **`near`**.

| Block                             | Read as                                                                                                                            |
| --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- | --- | ------------------------------------------------------------------------------------------------- |
| `mechs[0]`/`[1]`                  | Dual-slot mech world XY (finder skips stage props)                                                                                 |
| `loc`                             | Present local cam for this `slot`                                                                                                  |
| `present.{n,raw,skip_own,skip_y}` | Stage-prop OAM placed this frame. **`n=0` ≠ “nothing on screen”** — mech/UI still in `poam`; brown may still paint                 |
| `poam[]`                          | Presented OAM sample; `s:1` = sticky prop tile, `s:0` = other                                                                      |
| `props[]`                         | Whitelist movers only (`mw_is_stage_prop_meta`)                                                                                    |
| `elevs[]`                         | Meta `$D5B8` BG2 elevators — not in `props[]`                                                                                      |
| `near[]`                          | All `$1E14` records within **±160x / ±140y** of either mech                                                                        |
| `pscroll`                         | Scrolls **applied** for full-frame present (`h0` should track `loc`)                                                               |
| `strip.lag`                       | Cam-top row: `exp` = Δ(loc>>4); `stuck` only if `exp≠0`, strip has solid ink, and VRAM unshifted. Ignore all-sky cam-top (`sky≈n`) |
| `strip.feet`                      | Same probe at local mech Y (~40–75 below `loc`); **prefer `feet.lag.stuck`** for stand-surface swim                                |
| `plat`                            | Feet `$C382` (`                                                                                                                    | dy  | ≤120`) or `src:"feet"`. Body/stripe nearest **mech screen X**; brown at `my±32`; 22-col VRAM scan |

Per prop: `o`, `m`, `bank`, `t6`, `fl`, `h`, `why`, `d0`, `d1`, `sn`, `act`,
`draw`, `pin`, `wx`, `wy`, `dwx`, `dwy`, `sx`, `sy`, `glue`, `gh`, `use_wx`,
`use_wy`, `use_sx`, `use_sy`, `ax`, `ay`, `oam_x`, `c`, `e`, `w10`, `w12`,
`bg_try`, `bg_hit`, **`fov`**, **`use_fov`**, **`far_y`**,
`tiles[{t,sz,mox,moy}]`.

- `sx` = live `wx−loc`; `use_sx` = present XY (glue may freeze X)
- `draw:1` ≠ on-screen OAM — check `present.n` / `skip_y` / `far_y`
- `glue`/`gh` — cam-glue hits; `$C6A4` arms only on ghost follow (mech still)

**`near[]` fields**

| Field         | Meaning                                                                                                |
| ------------- | ------------------------------------------------------------------------------------------------------ |
| `cls`         | `prop` (whitelist) / `elev` (`$D5B8`) / `item` / `mech` (`+$06` hi `$01`/`$02` and not prop) / `other` |
| `n0`/`n1`     | Inside window of mech0 / mech1                                                                         |
| `dx*`/`dy*`   | `obj − mech*` world                                                                                    |
| `dwx`/`dwy`   | Frame Δ of this obj                                                                                    |
| `t7f`         | `$7F` word at origin (render ink; `$0200` = sky/empty sample)                                          |
| `prop`/`elev` | 1 if whitelist mover / `$D5B8`                                                                         |

**False negatives:** window is ±160/±140 — a collider just outside is omitted.
`cls=mech` wins on dual-tag hi-byte for non-whitelist `$B1` (meta 0 junk can
look like “mech”). Empty `near` non-mechs at feet ⇒ **no list object**; do not
invent a `$C6A4` fix.

**Column work:** prefer `mism_local` + `7f_loc` vs `vram` (local cam). Raw
`mism` (world `src1` vs local VRAM) is misleading under full-frame H2H.

### 6d. jq recipes

```bash
# --- Triage: rider vs BG1-only (P2) ---
jq -s 'map(select(.slot==1))|{
  frames:length,
  present_n_hist:([.[].present.n]|group_by(.)|map({n:.[0],c:length})),
  c6a4_frames:([.[]|select([.props[]?.m]|index(50852))]|length),
  near_prop_frames:([.[]|select([.near[]?|select(.n1==1 and .prop==1)]|length>0)]|length),
  prop_metas:([.[].props[]?.m]|group_by(.)|map({m:.[0],hex:(.[0]|tostring),c:length}))
}' /tmp/mw_a.jsonl

# Feet neighborhood (slot=1 → n1). Mechs-only ⇒ map stand, not mover glue.
jq -c 'select(.slot==1)|{f,loc,m1:.mechs[1],pn:.present.n,
  near:[.near[]?|select(.n1==1)|{o,m,bank,cls,wx,wy,dwx,dwy,dx1,dy1,t7f,prop}]}' \
  /tmp/mw_a.jsonl | head
jq -c 'select(.slot==1)|.near[]?|select(.n1==1 and .cls!="mech")|
  {o,m,cls,dx1,dy1,dwx,dwy,t7f}' /tmp/mw_a.jsonl | sort | uniq -c | sort -rn | head

# $C6A4 rider only (decimal 50852)
jq -c 'select(.slot==1)|select([.props[]?.m]|index(50852))|{f,loc,m1:.mechs[1],
  p:[.props[]|select(.m==50852)|{o,wx,wy,dwx,dwy,sx,sy,glue,gh,use_wx,use_sx,oam_x,sn}]}' \
  /tmp/mw_a.jsonl | head

# Props vs feet distance (catch “pins only, stand is map”)
jq -c 'select(.slot==1)|. as $r|{f,m1:$r.mechs[1],pn:$r.present.n,
  props:[.props[]|{o,m,wx,wy,dx:(.wx-$r.mechs[1][0]),dy:(.wy-$r.mechs[1][1]),
    sy,far_y,sn,glue,bg_try,bg_hit,draw}]}' /tmp/mw_a.jsonl | head

# Sticky-empty actives / moving props
jq -c 'select(.slot==0 or .slot==1)|.props[]?|select(.act==1 and .sn==0)|
  {f,slot,o,m,t6,h,why,sx,sy}' /tmp/mw_a.jsonl
jq -c 'select(.slot==0 or .slot==1)|.props[]?|select((.dwx|fabs)+(.dwy|fabs)>0)|
  {f,slot,o,m,dwx,dwy,wx,wy,glue,use_sx}' /tmp/mw_a.jsonl

# Present scrolls + sticky poam (BG1 swim / ride-then-snap)
jq -c 'select(.slot==1)|{f,loc,h0:.pscroll.h0,lag:.strip.lag,feet:.strip.feet,
  sticky:[.poam[]?|select(.s==1)|{x,y,t}]}' /tmp/mw_a.jsonl | head
# Prefer feet.lag (cam-top is often all $0200 sky while brown is at mech Y)
jq -c 'select(.slot==1 and .strip.feet.lag.stuck==1)|
  {f,loc,m1:.mechs[1],feet:.strip.feet}' /tmp/mw_a.jsonl | head

# Stand: collision gate vs present blob (FOV brown) — pan lock triage
jq -c 'select(.slot==1 and .plat.ok==1)|{f,loc,m1:.mechs[1],
  coll:.plat.coll,feet:.plat.feet,pres:.plat.pres,lag:.plat.lag}' \
  /tmp/mw_a.jsonl | head
jq -c 'select(.plat.lag.pres_lock==1 or .plat.lag.pres_world==1)|
  {f,loc,dloc:.plat.lag.dloc,d_pres_wx:.plat.lag.d_pres_wx,
   lock:.plat.lag.pres_lock,world:.plat.lag.pres_world,odx:.plat.pres.odx,
   ody:.plat.pres.ody,pres:.plat.pres}' /tmp/mw_a.jsonl | head

# Hitching paint ID — trust JSON slot. Prefer fix/spr (away-from-mech).
# Pan continuously. Lag flags need |dloc|≥3 or *.afire (accum ≥8).
jq -c 'select(.slot==1)|{f,lx:.loc[0],fix:.paint.fix,
  spr:(.paint.spr|{ok,n,t,sx,sy,lag})}' /tmp/mw_b.jsonl | head
# Fixed mid-screen VRAM: vstick=1 ⇒ view-column sticky at that slot
jq -c 'select(.slot==1 and ((.paint.fix.dloc|fabs)>=3 or .paint.fix.afire==1))|
  {f,lx:.loc[0],dloc:.paint.fix.dloc,
   v:[.paint.fix.v[]?|select(.vstick==1 or .scroll==1)|{sx,sy,t,vstick,scroll}],
   oam:.paint.fix.oam}' /tmp/mw_b.jsonl | head
# Away-from-mech non-mech OAM only
jq -c 'select(.slot==1 and .paint.spr.ok==1 and
  ((.paint.spr.lag.dloc|fabs)>=3 or .paint.spr.lag.afire==1))|
  {f,lx:.loc[0],spr:.paint.spr}' /tmp/mw_b.jsonl | head
jq -c 'select(.slot==1)|.paint.spr.tiles[]?' /tmp/mw_b.jsonl |
  jq -s 'group_by(.t)|map({t:.[0].t,c:length,sx:.[0].sx})|sort_by(-.c)|.[0:15]'
```

Snap / idle-BG2 probes (also under §8):

- `pscroll.{h0,v0,h1,v1,ploc,ok}` — applied present scrolls; `d_h0loc` / `d_h0hs0`
- Idle BG2: `pscroll.h1 == scroll.hs1` (PPU fine); never fill from WRAM/`h0`
- `strip.lag`: `stuck=1` when `|dloc|≥2` but VRAM ring unchanged
- prop `fov` / `use_fov` / `far_y` — blank FOV gate (`sy≈−200` → `far_y=1`)

---

## 7. How to manipulate (code map)

| Goal                      | Where                                                    | Notes                                                          |
| ------------------------- | -------------------------------------------------------- | -------------------------------------------------------------- | ------- | ------------------------------------------------- |
| Treat as mover            | `mw_is_stage_prop_meta`                                  | Whitelist meta only                                            |
| Change home rule          | `mw_stage_prop_home_cam`                                 | Tag vs nearer-mech / hysteresis                                |
| Capture completeness      | `mw_cam_oam_commit` + `mw_prop_sticky_store`             | Widen X/Y for props; accumulate tiles                          |
| Draw on home peer         | `mw_present_oam_from_cam_capture` sticky loop            | Full `tiles[]` from live wx/wy                                 |
| Hide on foreign peer      | same + `skip_own`; BG1 `mw_present_align_stage_prop_bg1` | OAM home-only; **always** blank foreign brown (no in-FOV keep) |
| Reject polluted sticky    | `mw_prop_meta_sane` / `mw_prop_sticky_scrub`             | Drop `                                                         | mox/moy | > 48` (mech steal); repair on next sane home seed |
| Shared pickup (not mover) | `mw_is_shared_b1_item_meta` + item sticky                | Both peers; do not home-isolate                                |
| Elevator                  | exclude `$D5B8` from mover path                          | Idle BG2: native peer scrolls (parallax). Dual: no ROM restamp |

**Present Y:** `draw=1` with `present.n=0` often means bottom cull (`sy ≳ 224`
after meta). Check `sx`/`sy` in coldump before assuming sticky failure.

---

## 8. Pin experiment (`$C382` catalog anchors)

World XY is stable; object pool slots recycle — key by `(wx,wy)`, not `o`.
Host match: `mw_prop_is_pinned_hazard` (meta `$C382` + catalog XY).

| Anchor                     | Role                                                 | Soak (slot1 stand)      |
| -------------------------- | ---------------------------------------------------- | ----------------------- |
| `(637,365)` / `(832,365)`  | Mid hazard ledge (**one home** via centroid 734,365) | In `props[]` every frame |
| `(220,106)` / `(1249,106)` | Upstairs                                             | `(220,106)` present     |
| `(220,621)`                | Floating crate                                       | Not in this soak        |

**Present:** pinned props force meta `mox/moy = −8/−8 + PIN_DY`
(`SNESRECOMP_MW_PROP_PIN_DY`). If sticky is empty (`sn=0`), seed synthetic
tile `$C8` so the live origin is visible.

**BG1:** foreign blank when `!in_fov` (wide `$C382` FP). Do **not** blank home
on mid-screen `!in_fov` (sky holes). **Far-Y** origins (`sy < −112` or
`≥ 288`, e.g. upstairs `sy≈−159`): FOV false; strip scan + **sky** `$0200`
fill for fingerprint ghosts (home or foreign). Synthetic pin seed only when
`sy ∈ [−48, 220)`. Mid-ledge ends share one home (centroid).

`draw:1` ≠ on-screen OAM — check `present.n` / `skip_y`. Coldump `pin=1`.

Full dump triage / jq live in **§6**. Pin-specific probes:

```bash
# Always filter by slot — filename is not peer identity
jq -c 'select(.slot==1)|{f,pscroll,poam:(.poam|length),
  props:[.props[]?|{m,wx,sx,sy,oam_x,fov,use_fov,far_y,bg_try,bg_hit,draw,pin}]}' \
  /tmp/mw_a.jsonl | head

# Catalog pins only (m=50050 = $C382)
jq -c 'select(.slot==1)|.props[]?|select(.m==50050)|
  {f,wx,wy,sx,sy,pin,sn,draw,use_sx,use_sy,bg_hit}' /tmp/mw_a.jsonl | head

jq -c 'select(.slot==1 and .pscroll.ok==1)|{f,d_h0loc:.pscroll.d_h0loc,
  h0:.pscroll.h0,hs0:.scroll.hs0}' /tmp/mw_a.jsonl

# Idle BG2: applied h1 must match NMI hs1 (filter slot; ignore filename)
jq -c 'select(.pscroll.ok==1 and .pscroll.h1 != .scroll.hs1)|
  {f,slot,h1:.pscroll.h1,hs1:.scroll.hs1,h0:.pscroll.h0}' /tmp/mw_a.jsonl
```

---

## 8a. Giant-stand soak suspects (2026-08-05)

Offline LocalFull coldump (`net:0`, `dual:1`, `OFFLINE_SLOT=1`, P2 on stand
~`wy>600`). List every candidate **before** adding hitch filters. Do not write
soak “wins” into `symbols.toml` until a lever moves the circled stand.

### Load-bearing / best current model

| Suspect | Soak evidence | Role vs stand |
| ------- | ------------- | ------------- |
| **Map/`$7F` ledge (`paint.band` `$1600`, mid `$1200`)** | P2: feet strip sky; `src_loc≢dma`; band SCREEN_FIXED | **Paint** — still open (not list props) |
| **Map collision (no list obj under feet)** | `near` = mechs only; feet `t7f=512` | **Collision** (separate) |
| **`plat.pres` east shelf** | `sx≥256` `odx≈200` | **False latch** — reject in coldump |

### List / OAM objects seen while on stand

| Suspect | Coldump id | Evidence | Working verdict |
| ------- | ---------- | -------- | --------------- |
| **`$C382` catalog pins** | `(637\|832,365)` / `(220,106)` `sy≪0` | `glue=0` `dwx=0`; far above cam | **Ruled out** — not under-feet stand |
| **All-`$C382` home-brown** | class latch experiment | Stand unchanged; backpack side risk | **Reverted** |
| **`$C39E` / `$C3EC` @ `(350,490)`** | meta recycles | Far from feet | Not stand |
| **Backpack `$E4` `poam s:0`** | t=228 near mech, often no props meta | Capture wx freezes → lags mech | **Fixed** — `mw_orphan_e4_backpack_xy` |
| **`$C6A4` / `$C38C` in props** | often `pack:[]` while `$E4` visible | Follow-live when listed near mech | Not stand |
| **Mech body** | `$E442` / bank `$AD` | Dominate `near` / `poam` | Mech — never CHR-allowlist |
| **Underfeet / foreign-DMA strip rewrite** | weak→snap, native merge, world-abs solid | Other BG hitch; stand unchanged | **Ruled out / reverted** |

### Absent this soak (still catalogued)

| Suspect | Note |
| ------- | ---- |
| **`$62` / `$C8` hazard stripe** | `paint.hzd.ok=0` |
| **`$D5B8` elevators** | empty `elevs[]` |
| **`$C6A4` as meta** | not in `props`; on-mech blob tagged `$C38C` instead |
| **`$C382` crate `(220,621)`** | not present in this elev-room soak |

Decimal cheat-sheet: `$C382=50050`, `$C38C=50060`, `$C39E=50078`,
`$C6A4=50852`, `$E442=58434`.

---

## 8b. Manipulation test queue

Goal: **perturb one suspect at a time** and watch the circled stand under the
mech. If the stand does not move / hitch differently, that suspect is not the
paint (or not the collision). Park the other cam off the column when possible.

### Pass / fail (per suspect)

| Result | Meaning |
| ------ | ------- |
| Stand under feet **moves / culls / recolors** with the lever | Married — keep digging that surface |
| Only far pin / backpack / mech changes; stand unchanged | **Ruled out** for giant stand — document and next |
| Map shatter / sliced mechs | Wrong class of lever — revert; do not widen |

### Step 1 — `$C382` pins `(637\|832,365)` and `(220,106)` — **DONE / ruled out**

**Outcome (2026-08-05):** stand under feet **did not** identify as these pins.
`PROP_PIN_DY` / pin path also **regressed** — corrupted map walls and items in
weird locations. **Leave `SNESRECOMP_MW_PROP_PIN_DY` unset.** Do not reuse pin
FP / synthetic seed as a stand probe.

Suspect other map objects / items next (queue below). Still prefer map/BG1 as
the long-term paint model.

### Step 2 — `$C39E` @ `(350,490)` — **DONE / skip (list props)**

**Outcome (2026-08-05):** probe keyed to meta `$C39E` but live object at
`(350,490)` was **`$C3EC` (`m=50156`)** — `use_sx==sx` always (DX/DY never
applied). Far from feet; `near` often mechs-only. **Skip further list-prop
manip** for the giant stand; leave `PROP_C39E_*` unset.

### Step 3 — Map / BG1 (split stand)  ← **current**

Best model after pins + empty/mechs-only `near`: collision = map; paint =
present BG1. Soak (`/tmp/mw_b.jsonl`, 2026-08-05) shows **two BG1 bands**,
same tile **`$1600` (5632)**:

| Band | Coldump | sy (approx) | wy (approx) | Pan behavior |
| ---- | ------- | ----------- | ----------- | ------------ |
| Top (stripe / upper brown) | `plat.pres` / `body` | ~59 | ~728 | **WORLD** (`d_sx≈−dloc`) — snaps with cam |
| Bottom (lower brown) | `paint.mid` (also `focus`/`vis`) | ~91–107 (sticky also ~43) | ~760–776 | **SCREEN_FIXED** (`d_sx≈0`) |

Hazard OAM absent (`hzd`/`spr` ok0). Feet: `$7F` sky, VRAM brown. Seam on
screen = these two rows disagreeing. **No** gap-fill / FP blank / CHR allowlist.

**Lever:** `SNESRECOMP_MW_STAND_BG1` (present-only, VRAM-restored):

| Value   | Effect |
| ------- | ------ |
| `dump`  | Sample `$7F` + VRAM grid (also with COLDUMP) |
| `mark`  | XOR solid VRAM; paint `$1600` into sky — band flickers ⇒ married |
| `blank` | Sky-fill solid VRAM — band vanishes ⇒ married |

**Origin** `SNESRECOMP_MW_STAND_BG1_AT` (default `auto`):

| AT | Uses |
| -- | ---- |
| `auto` | live `$1600` scan (raw cam Y) → per-slot mid/pres latch |
| `mid` | scan sy≈80–130 then mid latch (bottom / screen-fixed) |
| `pres` | scan sy≈40–80 then pres latch (top / world) |
| `focus` | scan sy≈20–60 |
| `mech` | local dual mech only (explicit; mark/blank never soft-fallback here) |

Scan uses **raw** `cam_y` for `sy` (not `v0`/y_bg). Tries present **VRAM**
then **`$7F`**. Per-slot latches survive the other peer’s coldump (on-screen
sx only — rejects east-shelf false latches). `src=miss` + `ok:0` when nothing
found (mark/blank no longer silent `mode:off`). `present.slot` is set by
LocalFull even when object-drawer OAM skips cam_capture.

Optional `STAND_BG1_WX`/`_WY`. Coldump: `stand_bg1.{mode,src,ox,oy,…}`.
Grid around origin covers both seam bands (`dy∈[-80,+64]`).

**Session — hit each band**

```bash
: > /tmp/mw_a.jsonl
# Do NOT set PROP_PIN_DY / PROP_C39E_*
# 1) screen-fixed bottom
SNESRECOMP_MW_COLDUMP=/tmp/mw_a.jsonl SNESRECOMP_MW_COLDUMP_EVERY=1 \
SNESRECOMP_MW_H2H_OFFLINE_SLOT=1 \
SNESRECOMP_MW_STAND_BG1=mark \
SNESRECOMP_MW_STAND_BG1_AT=mid \
  ./build-linux-prod/MetalWarriorsSNESRecomp

# 2) world top (after a few frames of mid so latches exist, or use auto)
SNESRECOMP_MW_STAND_BG1_AT=pres SNESRECOMP_MW_STAND_BG1=blank \
  ./build-linux-prod/MetalWarriorsSNESRecomp
```

Unset when done. Prefer truncate between runs.

**Watch**

- `stand_bg1.src` is `mid`/`pres`/`scan` (not stuck on `mech` with `solidv=0`).
- **mark/blank mid:** bottom circle should change; top may not.
- **mark/blank pres:** top/world band should change; bottom may stay glued.
- JSON must parse (`jq .` on a line) — bg2 close was fixed 2026-08-05.

```bash
jq -c 'select(.slot==1)|{f,mech1:.mechs[1],sb:.stand_bg1|
  {mode,src,ox,oy,solid7f,solidv,touched},
  mid:(.paint.mid|{ok,sx,sy,wx,wy,t}),
  pres:(.plat.pres|{ok,sx,sy,wx,wy,t})}' /tmp/mw_a.jsonl | head -20
```

**Log outcome here when done:** (pending) — mid band hit? Y/N; pres band hit?
Y/N; which matches each circle?

---

## 8c. Half→full playfield bounds (2026-08-03)

Sim lifetime gates (kept). **Did not fix whole-square whip sway** — coldump
still had `sx = wx−loc` while the square eased; that was present BG1/OAM.

| Lever                 | Native dual   | Full-frame fix                                      |
| --------------------- | ------------- | --------------------------------------------------- |
| `$1E9A` height        | `#$80`        | `$8095AF` → `#$F0` (bbox / Y distance; not `$1E9C`) |
| `$80A5AB` / `$809B43` | cam0 only     | Either dual cam (even without WS)                   |
| Mech finder           | first `$02xx` | Skip stage props                                    |

## 8d. Stabilize BG map (2026-08-03)

Uncapped snap X-remap + far-Y / wide fingerprint strip blank **shattered**
floors/walls (stair-step sky holes on both peers). Reverted to HEAD-safe BG:

- Snap native cols: `same_column` + `d_cols` ≤ west cap (no free X remap).
- Foreign blank / ink: narrow Y + `$7F` match only; **no** far-Y full-strip
  sky-fill; **no** home mid-screen blank. Same for `local_slot` 0 and 1.
- Prop OAM sticky / home / mid-ledge / dual bounds kept (slot-symmetric).

Map-brown stand hitch: full-frame BG1 must match game DMA (view-rel X + fine
HOFS) — see §9 #16. Snap native cols may remap **±1 tile column** only — do
not widen; no fingerprint paint.

---

## 9. Known gaps / open work

1. `$C382` upstairs (`$19F8` / `$1A4C` at wy≈106) — pin now seeds synthetic
   `$C8` when `sn=0`; confirm after scrolling those platforms on-screen. Dual
   drawer still preferred for real capture.
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
6. Map render props to true collision — run §6b triage first. If `near` at
   feet is mechs-only and `present.n=0`, the stand is **map/`$7F` brown**, not
   a missing whitelist meta. Only marry `cls=prop`/`other`/`elev` → present
   when a list object actually tracks the feet.
7. Residual BG bugs — snap ±1 col remap + `scroll.dsnap`; do **not**
   fingerprint-paint the strip or disable snap entirely (sky holes).
8. Home-only OAM + sane sticky: verify P1 no longer shows foreign jitter
   chunks; mid-ledge foreign brown still FOV-blanked (not OAM-drawn).
9. Firm home latch: mid-ledge cam wrestle / ~d01 doubles — confirm `why=firm`
   and near-zero home flips on `(637|832,365)` after both mechs spawn.
10. Idle BG2: `pscroll.h1 == scroll.hs1` (PPU fine; never WRAM/`h0`); keep WRAM
    `v1` parallax; dual `rst=0`. No full-strip FP blank / idle-BG2 wipe.
11. `$C39E` @ (1118,490) P1-bottom→P2-top rollover: home forever-lock + FOV
    hold on firm props — confirm no `h` flip at `sy≈206` on P1.
12. `$C6A4`: sticky `$E4` only; non-`$E4` never `prop_obj`/local_only (shots
    animate). On-mech `$C6A4` = backpack — present follows live.
13. Giant stand: use `plat.pres` (FOV blob) + `lag.pres_world`/`pres_lock`;
    `plat.feet.weak` is expected. Not `$C6A4`. Mid-ledge 1px false follow ~0.
14. Widescreen prop OAM: while panning past a home pin, `poam` sticky X must
    stay negative (left), never jump to `≈320` then `present.n→0`. Crate
    `(220,621)` was the repro (`oam=-186` → poam `+326` → cull at `-200`).
15. Both-peer BG1 flicker with `present.n=0`: P1 `bg_hit` on pinned `$C382`
    should rise well above ~10; P2 home pin brown stable at top edge (`sy≈-14`).
16. Mid-view stand “follows cam in grid snaps opposite pan”:
    **Paint = map/`$7F` ledge** (`paint.band` `$1600`, mid/vis `$1200`), not
    list `$C382`. P2: feet strip sky; live DMA foreign (`src_loc≢dma.aadr`).
    Keep view-rel X + `h0_ppu=cam&15`; reject `plat.pres` sx≥256 east shelf.
    **Still open** — next dig: per-cell coldump `stripe_t` vs `world_t` vs
    `vram_t` + lag class while panning (not another strip-wide rewrite).
17. **FALSE LEAD — strip rewrites that hitch other BG (2026-08-05, reverted):**
    Underfeet ownership/native snap merge, weak→snap on live sky, foreign-DMA
    **solid** world-abs / `prefer_snap` — other map edges tear while stand
    FOV-stuck unchanged. Rebuild keeps: live `$7F`, snap fills weak only,
    **void-only** world-abs. Trust `present.slot==slot` (dual `ps:-1` inherits
    peer `pscroll`). Do not revive strip-wide solid replace.
18. **Backpack lag (untagged `$E4` `poam s:0`, 2026-08-05):** Often absent from
    `props[]` as `$C6A4`/`$C38C`. Gameplay capture wx freezes while mech walks.
    Fix: `mw_orphan_e4_backpack_xy` — latch mech-relative offset when tight;
    free far `$E4` (platform sticky) unchanged. Verify: `e4.x` tracks `msx`.

Named guest symbols (`recomp/symbols.toml`): `MwObject`, `ObjectListHead`,
`ObjectPoolBase`, `Cam0_X`/`Cam1_X`, `Bg1MapPitch` (`$00B6`), `StripSrc_A/B`,
`ScrollH0_P2` (`$1E32` ≠ P2 world cam), `DualViewportFlag`, `TilePatchSta7F`,
`MwStageDirtyRect`, `MwDrawObject_H2H`. Meta catalog in TOML comments.

---

## 10. Changelog

| Date       | Note                                                                                                                                                                                                                                                                          |
| ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| 2026-08-05 | Revert foreign-DMA solid world-abs / `prefer_snap` (other BG hitch; stand unchanged). Keep void-only 42B3; backpack `$E4`; reject `plat.pres` east shelf.                                          |
| 2026-08-05 | Stand = map/`$7F` (not `$C382`); revert all-`$C382` home-brown; backpack `$E4` s:0 → `mw_orphan_e4_backpack_xy`.                                                                                        |
| 2026-08-05 | (reverted) all-`$C382` home-brown / wide capture — pins far-Y; stand unchanged.                                                                                                                      |
| 2026-08-05 | (reverted) underfeet sky strip merge — other BG hitch; stand unchanged. See #17.                                                                                                                    |
| 2026-08-05 | Stand hitch: prefer snap when live DMA foreign (void holes). Not the stand class fix.                                                                                                                 |
| 2026-08-05 | STAND_BG1: `src=miss` emit; `$7F` scan + stamp; latch sx gate; LocalFull always sets `present.slot`.                                                                                                |
| 2026-08-05 | STAND_BG1: live `$1600` scan with raw cam Y; per-slot mid/pres latch; no mech sky-stamp on mark/blank.                                                                                              |
| 2026-08-05 | Stand split: `$1600` WORLD `pres` (top) vs SCREEN_FIXED `mid` (bottom). `STAND_BG1_AT=mid\|pres`; fix coldump bg2/`stand_bg1` JSON.                                                                    |
| 2026-08-05 | Skip list props: step2 `$C39E` miss (`$C3EC` at XY). Step3 map/BG1 via `STAND_BG1=dump\|mark\|blank` + coldump `stand_bg1`.                                                                              |
| 2026-08-05 | Step1 `$C382` pins ruled out; `PROP_PIN_DY` left unset (wall/item regress). Step2 `$C39E`@350,490 via `PROP_C39E_DX`/`_DY`.                                                                                    |
| 2026-08-05 | §8a soak suspects + §8b manip queue; start with `$C382` pins + `PROP_PIN_DY`. `$C38C` backpack note.                                                                                                          |
| 2026-08-05 | Offline dual coldump/LocalFull: COLDUMP auto-enables present path; `OFFLINE_SLOT` / `OFFLINE_LOCAL_FULL`; coldump both slots.                                                                              |
| 2026-08-05 | Coldump `paint.fix` (fixed mid-screen VRAM/OAM lag); spr away-from-mech + wider mech CHR exclude.                                                                                              |
| 2026-08-05 | Coldump `paint.spr` (non-mech mid-view OAM lag) + `paint.mid` (screen-sticky BG1 cand).                                                                                                              |
| 2026-08-05 | Coldump `paint.hzd` ($62/$C8 OAM), `paint.band` (Y-band solid), scr `vstick_a`/apan (slow pan).                                                                                              |
| 2026-08-05 | Coldump `paint.vis` (screen-track near mech) + `paint.scr` (vstick/scroll). Gap-fill reverted — wrong lever.                                                                                                      |
| 2026-08-05 | Coldump `paint{}`: FOV paint candidates + lag (not feet-gated). Collision/paint are separate layers.                                                                                                                                                                          |
| 2026-08-05 | Stand ID: `plat.under[]` + under-feet `pres` (reject upper-shelf latch); fix VRAM world sample row. View-rel+fine HOFS kept.                                                                                                                                                  |
| 2026-08-05 | Stand grid-snap: BG1 view-rel strip + fine HOFS (`cam&15`); match game DMA. Orphan `$62` latch was wrong lever for map-brown stand. Snap ±1 col only.                                                                                                                         |
| 2026-08-05 | (superseded) orphan `$62` OAM world-latch for stand trim — did not move map-brown stand.                                                                                                                                                                                      |
| 2026-08-05 | `plat.pres`: score runs near `coll` / on-screen (`7f`+`vram`); reject far-east shelf latch.                                                                                                                                                                                   |
| 2026-08-05 | `plat`: feet/`$C382` = collision gate; `pres` = FOV brown blob (`odx`/`ody`). Stripe drops `$E4`. Lag `pres_world`/`pres_lock`.                                                                                                                                               |
| 2026-08-05 | `$C6A4` on-mech (`dy≈-14`, `$E4`×2, sky `t7f`) = backpack pickup — present follows live; brown latch skipped. Free `$C6A4` keeps world-lock. Giant stand ≠ `$C6A4`.                                                                                                           |
| 2026-08-05 | (superseded) `$C6A4` “rider” world-lock while under mech — tore backpack off / mis-IDd stand.                                                                                                                                                                                 |
| 2026-08-05 | Revert untagged stand-plat CHR world-lock (sliced mechs: `$0A/$0C/$20…` are mech body, not plat). `$C6A4` sticky `$E4` only.                                                                                                                                                  |
| 2026-08-05 | (bad) Untagged stand-plat OAM CHR world-lock at capture — reverted same day.                                                                                                                                                                                                  |
| 2026-08-05 | Mid-ledge home: neither-FOV → closer `                                                                                                                                                                                                                                        | sy          | `; pin paint skip `far_y`; gameplay OAM drop prop-sticky tiles; `plat.stripe`drop`$0E`.                                               |
| 2026-08-04 | `$C6A4` recycle: reset parked `                                                                                                                                                                                                                                               | anchor−live | >48`; skip sn=0 far paint; `plat.stripe`only`$62/$C8/$E4/$0E`.                                                                        |
| 2026-08-04 | Host brown paint: scrub prior paint origin on XY move; clear latch foreign/sticky-death; pins paint catalog XY only. Gameplay OAM present from capture `wx−loc` (not stale sx). symbols.toml paint/sim split.                                                                 |
| 2026-08-04 | `plat`: body/stripe nearest mech SX; brown `my±32`; 22-col scan (was first-left / far `wx=896`).                                                                                                                                                                              |
| 2026-08-04 | `plat`: feet-gate `$C382` (`                                                                                                                                                                                                                                                  | dy          | ≤120`) or `src:feet`; track `body.wx` (no fine-phase false snap).                                                                     |
| 2026-08-04 | Coldump `plat`: nearest `$C382` stripe+brown unit (`stripe`/`body`/`lag.snap`). Circled stand platform.                                                                                                                                                                       |
| 2026-08-04 | Coldump `strip.feet` + lag fix (`exp`=Δ tile col; stuck needs solid ink). symbols.toml: pitch/scrolls/`$1E32` trap, meta catalog, pool recycle notes. Map-brown swim ≠ `$C6A4` glue.                                                                                          |
| 2026-08-04 | `$C6A4`: both-peer glue tick; remote-step hold; feet resync when live under mech but anchor far (stale 1138). Foreign ghost blanks use+live.                                                                                                                                  |
| 2026-08-04 | `$C6A4` travel: retarget only if `dcam==0` and `                                                                                                                                                                                                                              | dwx         | ≥2` for 3 frames (blocks one-frame turnaround ±60 snaps).                                                                             |
| 2026-08-04 | `$C6A4`: drop world-step X retarget (was snapping `use` ±20 onto cam-poisoned live). Hold through follow/ride; true travel only.                                                                                                                                              |
| 2026-08-04 | `$C6A4`: world-lock present X through cam-correlated `+$02` (follow **and** ride); retarget only true travel. Decouples OAM/brown from viewport-stuck `sx`.                                                                                                                   |
| 2026-08-04 | Coldump §6 rewrite: session hygiene (trust `slot`), decimal meta table, rider-vs-BG1 triage (`near` mechs-only + `present.n=0` ⇒ map brown), worked example, expanded jq.                                                                                                     |
| 2026-08-04 | `$C6A4`: ghost-follow glue (mech still) vs live ride/walk; freeze X only; no `\|Δ\|>48`. `near[]` married feet to `$1A68`/`$C6A4` (`dy1≈-14`). symbols.toml: meta `$E442`/`$E30E` bank `$AD`, pool notes.                                                                     |
| 2026-08-04 | Coldump `near[]`: all `$1E14` objs within ±160/±140 of either mech (any meta/bank) + `t7f` + `cls`. For collision↔render marriage when `props[]` misses the stand surface.                                                                                                    |
| 2026-08-04 | `$C6A4`: disable cam-glue entirely (always live `+$02`). Freeze/clear still snapped OAM ±19px on strafe after brown-stale fix.                                                                                                                                                |
| 2026-08-04 | `$C6A4` glue: brown pin only if latch within 32px of live; `                                                                                                                                                                                                                  | anchor−live | >48`force-live (was C6A4-exempt); drop stale brown latch on failed recapture. Fixes elevator`$1A68`freeze at`(692,607)`/`use_sy≈-49`. |
| 2026-08-04 | **Revert** tall pin FP blank / native snap merge / tall brown latch (`bg_hit` 115–144 sky-holed floors). Same class as §8d — do not re-widen foreign strip Y.                                                                                                                 |
| 2026-08-04 | Prop OAM left cull: `x_lo=-extra` (was `-extra-64`) so sticky never enters widescreen ambiguous 9-bit band (ghost on right → snap). Home brown scrub-then-paint; skip `prop_obj` on other-cam reproj; poam unwrap matches PpuDecodeOamX. Foreign strip Y slop wider for pins. |
| 2026-08-04 | `$C6A4` world-lock: dsx-gated glue again; pin to `$7F` brown origin while `+$02` cam-follows; no `\|Δ\|>48` force-live. Follow detect: world-lock wins on 1px pans (fixes false glue on pins).                                                                                |
| 2026-08-04 | `$C6A4`: drop hold-glue (always live); demote non-`$E4` out of prop_obj/local_only so shots/items are not present-skipped. Keeps `$E4`-only sticky filter.                                                                                                                    |
| 2026-08-04 | `$C6A4` sticky: `$E4`-only + tighter list-recapture; scrub stolen shot/item tiles. Hold-glue kept; pickups/projectiles no longer freeze in air.                                                                                                                               |
| 2026-08-04 | `$C6A4` glue: hold world anchor through pan; retarget only on real travel. Stops ride-then-snap (`use−wx`→+17 then oam −15 on glue clear). Soften `strip.lag.stuck` to `                                                                                                      | dloc        | ≥2`.                                                                                                                                  |
| 2026-08-04 | Coldump: `strip.lag` (VRAM vs `dloc` stuck/snap) + `poam` prefers sticky prop tiles (`s:1`, max 32). For multi-frame platform ride-then-snap.                                                                                                                                 |
| 2026-08-04 | Idle BG2 present: always NMI PPU `h1` (`s_nmi_hscroll1`); keep WRAM `v1`. Kills cam-sized snaps and residual ±10–14 WRAM/PPU fine fight (`pscroll.h1 != hs1`).                                                                                                                |
| 2026-08-04 | Idle BG2 present: drop `h1==0→h0` cam fill; when WRAM h1 is 0/cam/world while NMI `hs1` fine, use PPU fine. Fixes 1-frame elevator/brown snap (`pscroll.h1` = loc).                                                                                                           |
| 2026-08-04 | Coldump: `pscroll` (applied present scrolls), `poam[]` (presented OAM), prop `fov`/`use_fov`/`far_y`. Catches 1-frame snaps that never touch prop `oam_x`.                                                                                                                    |
| 2026-08-04 | `$C6A4` dsx-gated glue: follow only if screen origin stuck (`dsx≈0` and `dwx≈dcam`); real travel → live; no 48px snap. Fixes left-slide from frozen use≈1000 and prior over-strip regression.                                                                                 |
| 2026-08-04 | `$C6A4`: disable cam-glue (`mw_prop_no_cam_glue`) — always live `use_wx` (interim; superseded by dsx-gated).                                                                                                                                                                  |
| 2026-08-04 | Hard-pin present cam: latch loc once for BG1+OAM; full-frame never peer `$1E36`/sticky src; skip raw-capture prop sx; snap same-column only. Coldump `oam_x`/`pcx`/`d01`; `COLDUMP_EVERY=1`.                                                                                  |
| 2026-08-04 | Foreign pin blank: wide east FP + VRAM residue (not global wide). Home brown latch for pinned `$C382` + `$C6A4`. `present.n=0` dumps were BG1-only flicker.                                                                                                                   |
| 2026-08-04 | Cam-glue: any-step follow detect + clear hysteresis + drop if live/anchor diverge >48; brown at `use_wx`; coldump `glue`/`gh`/`use_*`/`ax`. Fixes position teleport on cam move.                                                                                              |
| 2026-08-04 | `$C6A4` flicker: sticky match tile+mox (±2) so `sn=2`; home brown latch/paint at live XY (dual `$7F` stomps). No full-strip FP.                                                                                                                                               |
| 2026-08-04 | **Revert** foreign rider full-strip BG1 FP + idle-BG2 wipe (`$C39E` hit≈218 sky-holed map). Keep prior pocket/native blank only.                                                                                                                                              |
| 2026-08-04 | Foreign `$C6A4`/`$C39E`: visible-strip BG1 FP blank + dual idle-BG2 wipe — failed; see revert above.                                                                                                                                                                          |
| 2026-08-04 | **BG2 elevator:** revert cam-bind (dual `v1≈cam/2` is parallax). Dual: no ROM restamp (shared VRAM ghosts). Native peer scrolls + zero-mirror fallback only.                                                                                                                  |
| 2026-08-04 | BG2 restamp gated on peer DMA ownership (nearer cam/BG2 window); locate `own=-1`. Coldump `bg2.pv1`/`own`/`rst` + per-row `own`.                                                                                                                                              |
| 2026-08-04 | Idle BG2: always local-cam scroll on full-frame H2H; restamp visible pitch-run only; coldump before VRAM restore; pad shadow skips sky `$7F`.                                                                                                                                 |
| 2026-08-04 | BG2 restamp: frame-age live DMA rows only; locate seeds 8 rows (not 16). Coldump `elevs`/`bg2`. Targets cam-Y elevator ghost band.                                                                                                                                            |
| 2026-08-04 | **Revert** home BG1 blank / smooth-OAM catalog (map shatter; wrong path for `$D5B8`).                                                                                                                                                                                         |
| 2026-08-04 | Smooth movers experiment (blank home BG1 + OAM catalog) — failed; see revert above.                                                                                                                                                                                           |
| 2026-08-04 | Mid-ledge FOV home (not forever nearer); sticky home-drawer only; clear sticky on home flip.                                                                                                                                                                                  |
| 2026-08-04 | Home-only BG1 keepout (drop foreign-in-FOV keep) — P1 ledge ghost on P2 dual-cam X slide.                                                                                                                                                                                     |
| 2026-08-04 | Ignore mover `+$06` hi-byte home; present-anchor when live wx cam-glues; blank foreign at anchor.                                                                                                                                                                             |
| 2026-08-04 | Foreign BG1 always blank; prop OAM Y live-only (no cap-sy screen park / cam-follow).                                                                                                                                                                                          |
| 2026-08-04 | `$C39E`@1118,490 home forever-lock; firm props hold while in home FOV (no bottom→top handoff).                                                                                                                                                                                |
| 2026-08-04 | Revert BG2 expand/reframe restamp (map shear). Keep DMA-row-only restamp.                                                                                                                                                                                                     |
| 2026-08-04 | Firm home: lock mid ledge after both mechs; ignore single-mech stomps; others 16×. Fixes cam-wrestle doubles (~`d01`).                                                                                                                                                        |
| 2026-08-03 | Polluted sticky: `                                                                                                                                                                                                                                                            | mox/moy     | ≤48`, scrub, empty+FOV+sane seed only; prop recover `adx≤24`+`ady≤40`; **home-only OAM** (drop foreign FOV draw).                     |
| 2026-08-03 | P1 anti-jitter: seed sticky when empty from any drawer + present fallback (fix home_sn0) — later tightened (see above; unrestricted empty-seed stole mech tiles).                                                                                                             |
| 2026-08-03 | Stabilize: revert snap X-remap + far-Y fingerprint blank (map shatter). Keep slot-symmetric foreign blank / prop OAM.                                                                                                                                                         |
| 2026-08-03 | Restore dual `$1E9A` half→full + dual-cam on-screen/bbox + mech skip (boundary remapping; not strip paint).                                                                                                                                                                   |
| 2026-08-03 | Revert BG1 no-snap / no-RetainHistory (sky holes). Keep OAM mox live-cam latch-once + coldump `scroll.*`. Sway is not a strip-rebuild lever.                                                                                                                                  |
| 2026-08-03 | Far-Y FOV + strip sky-fill for upstairs ghosts; pin seed on-screen only. Earlier: revert home mid `!in_fov` blank; mid-ledge unified home; pin catalog / wide FP.                                                                                                             |
| 2026-07-23 | FOV exception uses body AABB (not origin alone) so `sx≈−300` ledges in the left margin stay visible. Confirm prod ELF is what you launch (`build-linux-prod/…`, not `build/` / debug). Earlier: origin FOV; left-gutter keys; native foreign-ink filter; sticky `$8000` wipe. |
| 2026-07-22 | Initial doc from coldump deep ID + H2H mover work: whitelist vs items, nearer-mech home, multi-tile sticky, BG1 blank, elev-room catalog (`$C382`/`$C39E`/`$C6A4`).                                                                                                           |
