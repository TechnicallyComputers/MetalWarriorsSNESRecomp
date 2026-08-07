#include "mw_rtl.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common_cpu_infra.h"
#include "cpu_state.h"
#include "funcs.h"
#include "mw_symbols.h"
#include "snes/interp_bridge.h"
#include "snes/snes.h"
#include "snes/dma.h"
#include "snes/ppu.h"
#include "snes/saveload.h"
#include "types.h"
#include "widescreen.h"
#include "snes/ws_shadow.h"
#include "snes_netplay.h"

extern Snes *g_snes;
extern Dma *g_dma;
extern Ppu *g_ppu;
extern uint8_t g_ram[0x20000];

/* NTSC: 1364 master clocks/scanline × 262 scanlines/frame. */
enum { kMwMasterClocksPerFrame = 1364u * 262u };

/*
 * BG1/BG2 stripe DMA ($80:92B3–$9338) — ROM findings (2026-07-18):
 *
 * Entry $8092B3: LDA/STZ $1E36 (BG1 src); $8092F7: LDA/STZ $1E38 (BG2).
 * Channel 0 setup (SEP #$10, A still 16-bit):
 *   $2115=VMAIN $80, $4301=$18 (VMDATAL), $4300=$01 (word A→B),
 *   $4304=$7F (A-bus bank), $4302=A from $1E36/$1E38.
 * Per-row loop:
 *   STA $4302 ← A; PHA; LDA DB:$856C,X → $2116; LDA #$0022 → $4305;
 *   PLA; CLC; ADC $B6; STY $420B; …  (row stride in A, not DASIZ)
 *   #$0022 = 34 bytes = 17 tilemap words (16 visible big-tile cols + 1).
 * X steps by 2 until $1E9C (BG1) or #$10 (BG2). Twin at $80A805.
 *
 * Source is a full decoded map in $7F (row bases $7E:42B3 + camX/8). Extra
 * columns exist on both sides of the 17-word window.
 *
 * Host pre-opcode at STA $4305 (PCs below), pad = ceil(g_ws_extra/16):
 *   pad_left  = min(pad, VMADD&31, aAdr/2)
 *   pad_right = min(pad, 32 - (col-pad_left) - 17)
 *   $4302  -= 2*pad_left     (A-bus word bytes; next row reloads from A)
 *   VMADD  -= pad_left       (tile columns; 32-col row clamp)
 *   DASIZ   = 0x22 + 2*(pad_left+pad_right)   (= 0x22+4*pad when unclamped)
 *
 * A/B (g_ws_extra=71 → pad=5): MW's stripe always uses VMADD col 0, so
 * pad_left clamps to 0 and DASIZ becomes 0x22+2*pad (=#$002C). Right margin
 * from DMA-pad VRAM; left from $7F/snap west of src — present rebuild paints
 * VRAM columns [-pad, …) and shadow prefill ForceTiles the gutter (snap
 * fallback when dual stomps $7F). SNESRECOMP_MW_DMA_WIDEN=0 disables widen.
 */
/* Stage "dirty rect" $1E72–$1E78 is NOT a $7F decode window. ROM only
 * CMP/STA's it at $80941E–$809452; the early-out path is dead (both
 * branches at $809417/$809419 fall into the rebuild). Rebuild only
 * refreshes $1E36 stripe src from $7E:42B3+cam. Full $7F map is decoded
 * once at stage load ($809BF1). STA widen kept as harmless A/B; it does
 * not fill pipe voids. SNESRECOMP_MW_WIDEN_AW=0 disables. */
enum { kMwWidenAfterWriteEnabled = 1 };
enum { kMwDmaWidenEnabled = 1 };
/* Set SNESRECOMP_MW_COLS=1 to log unique leading tile-column depth. */
enum { kMwUniqueColLogEnabled = 0 };
/*
 * SNESRECOMP_MW_COLDUMP=<path.jsonl> (or =1 → ./mw_coldump.jsonl):
 * compact one-line JSON records for offline column / mover analysis.
 * Prefer this over stderr — elev/COLS dumps overflow CLI scrollback.
 */

/* Native DASIZ and STA $4305 sites (LDA #$0022 is the prior opcode).
 * PCs: recomp/symbols.toml → mw_symbols.h (MW_SITE_DmaSize*). */
enum {
  kMwDmaSizeNative = 0x0022u, /* bytes */
  kMwDmaSizePcBg1 = MW_SITE_DmaSizeBg1,
  kMwDmaSizePcBg2 = MW_SITE_DmaSizeBg2,
  kMwDmaSizePcAlt1 = MW_SITE_DmaSizeAlt1,
  kMwDmaSizePcAlt2 = MW_SITE_DmaSizeAlt2,
};

static void mw_latch_nmi_camera(void);
static void mw_elev_dump(void);
static unsigned mw_ws_extra_u(void);
static int mw_h2h_vert_widen_armed(void);
static uint16_t mw_wram16(uint16_t addr);
static unsigned s_tile7f_hits;
static unsigned s_tile_helper_hits;
/* Whitelisted $00B1 movers → cam-capture local-only (skip reproject).
 * Latch at drawer STA $86 (X=object) for the whole sprite; post-meta must not
 * clear on a bad intermediate X. Shared $B1 items are mirrored into both cam
 * buffers instead (not local_only). 1P: $808714/$808721; H2H dual: $8087DE/
 * $808802/$80881F. Commit recovers list object via $96→$136E,Y. */
static int s_pending_stage_prop_local_only;
static uint16_t s_draw_obj_latched;
/* Armed at latch; cleared after first commit purges prior tiles for this obj. */
static uint16_t s_prop_purge_arm;
/* Lifetime counters for SNESRECOMP_MW_ELEV (non-saturating). */
static unsigned s_prop_stat_latch;
static unsigned s_prop_stat_list_rec;
static unsigned s_prop_stat_commit;
static unsigned s_prop_stat_convert;
/* Peak meta tile-count ($7E) seen while a stage prop is pending (elev). */
static unsigned s_prop_stat_meta7e_max;
/* Snapshot at present — elev dump may run after OAM clear zeros live buffers. */
static unsigned s_elev_cap_n0, s_elev_cap_n1;
static unsigned s_elev_cap_lo0, s_elev_cap_lo1;
/* Elev: last full-frame BG1 src path (0=none 1=42b3 2=raw_walk 3=skip_p1 4=legacy). */
static int s_elev_bg1_src_path;
/* Elev: count of foreign stage props blanked from local BG1 this present. */
static int s_elev_prop_bg_dy;

/* Last DMA-widen + present stats for SNESRECOMP_MW_COLDUMP JSONL. */
static int s_coldump_pad_l, s_coldump_pad_r;
static uint16_t s_coldump_dasiz, s_coldump_aadr0, s_coldump_aadr1;
static uint16_t s_coldump_vm0, s_coldump_vm1;
static uint32_t s_coldump_dma_pc;
static int s_coldump_dma_frame = -1;
static int s_coldump_dma_dirty;
static unsigned s_coldump_prop_n, s_coldump_prop_raw;
static unsigned s_coldump_skip_own, s_coldump_skip_y;
static int s_coldump_present_slot = -1;
static int s_coldump_present_frame = -1;
/* Scrolls actually applied for the last full-frame present (not NMI fine). */
static uint16_t s_coldump_ph0, s_coldump_pv0, s_coldump_ph1, s_coldump_pv1;
static uint16_t s_coldump_ploc_x, s_coldump_ploc_y;
static int s_coldump_pscroll_frame = -1;
/* Prior strip sample for cam-vs-VRAM lag (platform slide-then-snap). */
static int s_coldump_lag_slot = -1;
static uint16_t s_coldump_lag_locx;
static uint16_t s_coldump_lag_vram[22];
static uint16_t s_coldump_lag_feet_vram[22];
static uint8_t s_coldump_lag_have;
static uint8_t s_coldump_lag_feet_have;
/* Prior plat probe: collision gate + present brown blob (FOV band). */
static int s_coldump_plat_slot = -1;
static uint16_t s_coldump_plat_obj; /* $C382 near feet, else 0 */
static uint16_t s_coldump_plat_locx;
static int s_coldump_plat_use_sx;
static int s_coldump_plat_stripe_sx;
static int s_coldump_plat_pres_wx; /* present blob centroid world X */
static int s_coldump_plat_pres_wy;
static unsigned s_coldump_plat_pres_t;
static uint8_t s_coldump_plat_have;
static uint8_t s_coldump_plat_stripe_ok;
static uint8_t s_coldump_plat_pres_ok;
/* FOV paint-blob focus (NOT feet-gated) — ID hitching render layer. */
static int s_coldump_paint_slot = -1;
static int s_coldump_paint_wx;
static int s_coldump_paint_wy;
static uint16_t s_coldump_paint_locx;
static uint8_t s_coldump_paint_have;
/* Screen-tracked vis blob (near mech on screen) — world-sticky focus can
 * latch east shelves while the hitching stand stays view-column-fixed. */
static int s_coldump_vis_slot = -1;
static int s_coldump_vis_sx;
static int s_coldump_vis_sy;
static int s_coldump_vis_wx;
static int s_coldump_vis_wy;
static uint16_t s_coldump_vis_locx;
static uint8_t s_coldump_vis_have;
/* Fixed screen-slot VRAM probes (same sx/sy → tile stay = view-sticky). */
enum { kMwPaintScrN = 12 };
static int s_coldump_scr_slot = -1;
static uint16_t s_coldump_scr_locx;
static uint16_t s_coldump_scr_t[kMwPaintScrN];
static uint8_t s_coldump_scr_have;
static int16_t s_coldump_scr_apan[2]; /* accumulated Δloc since epoch */
static uint16_t s_coldump_scr_epoch_t[2][kMwPaintScrN];
static uint8_t s_coldump_scr_epoch_have[2];
/* Hazard OAM ($62/$C8) screen-tracked — stand trim may be sprites, not BG1. */
static int s_coldump_hzd_slot = -1;
static int s_coldump_hzd_sx;
static int s_coldump_hzd_sy;
static unsigned s_coldump_hzd_t;
static uint16_t s_coldump_hzd_locx;
static uint8_t s_coldump_hzd_have;
static int16_t s_coldump_hzd_apan[2];
/* Non-mech mid-view OAM screen-track (any CHR except known mech body). */
static int s_coldump_spr_slot = -1;
static int s_coldump_spr_sx;
static int s_coldump_spr_sy;
static unsigned s_coldump_spr_t;
static uint16_t s_coldump_spr_locx;
static uint8_t s_coldump_spr_have;
static int16_t s_coldump_spr_apan[2];
/* Mid-view BG1 cand — screen-sticky (stand paint XY), not feet/band nearest. */
static int s_coldump_mid_slot = -1;
static int s_coldump_mid_sx;
static int s_coldump_mid_sy;
static int s_coldump_mid_wx;
static int s_coldump_mid_wy;
static uint16_t s_coldump_mid_locx;
static uint8_t s_coldump_mid_have;
static int16_t s_coldump_mid_apan[2];
/* Fixed mid-screen probes — independent of mech / nearest-cand. */
enum { kMwPaintFixN = 4 };
static int s_coldump_fix_slot = -1;
static uint16_t s_coldump_fix_locx;
static uint16_t s_coldump_fix_vt[kMwPaintFixN];
static int s_coldump_fix_oam_sx;
static int s_coldump_fix_oam_sy;
static unsigned s_coldump_fix_oam_t;
static uint8_t s_coldump_fix_have;
static uint8_t s_coldump_fix_oam_have;
static int16_t s_coldump_fix_apan[2];
/* Per-prop BG1 blank results from last present align (foreign hide). */
static uint16_t s_coldump_bg_obj[24];
static uint16_t s_coldump_bg_hit[24];
static uint8_t s_coldump_bg_try[24];
static int s_coldump_bg_slot = -1;
/* Motion trail for coldump Δwx/Δwy. */
static uint16_t s_coldump_mot_obj[24];
static uint16_t s_coldump_mot_wx[24];
static uint16_t s_coldump_mot_wy[24];
static uint8_t s_coldump_mot_valid[24];
static unsigned s_lle_host_frames; /* also used by LLE session / coldump */
static void mw_coldump_tick(int local_slot);
static uint16_t mw_7f_off_from_world(uint16_t world_x, uint16_t world_y);
static void mw_stand_bg1_probe(int local_slot, uint16_t cam_x, uint16_t cam_y,
                               uint16_t scroll_x, uint16_t scroll_y);
/* Filled by mw_stand_bg1_probe (present); emitted in coldump stand_bg1. */
enum { kMwStandBg1Max = 96 };
static struct {
  int slot;
  char mode[8];
  char src[12]; /* mech|pres|mid|focus|env|scan|latch */
  int mx, my, ox, oy;
  int n_solid7f, n_solidv, n_touched, n_cells;
  int16_t dx[kMwStandBg1Max];
  int16_t dy[kMwStandBg1Max];
  uint16_t t7f[kMwStandBg1Max];
  uint16_t tv0[kMwStandBg1Max];
  uint16_t tv1[kMwStandBg1Max];
} s_stand_bg1;
/* Per-slot stand blob latches — survive the other peer's coldump clearing
 * global mid/pres have flags. Updated when paint.mid / plat.pres succeed. */
static int s_stand_latch_mid_wx[2], s_stand_latch_mid_wy[2];
static int s_stand_latch_pres_wx[2], s_stand_latch_pres_wy[2];
static uint8_t s_stand_latch_mid_have[2];
static uint8_t s_stand_latch_pres_have[2];
/* Near-mech motion trail for coldump `near[]` (any list meta, not just props). */
enum { kMwColdumpNearMot = 32 };
static uint16_t s_coldump_near_obj[kMwColdumpNearMot];
static uint16_t s_coldump_near_wx[kMwColdumpNearMot];
static uint16_t s_coldump_near_wy[kMwColdumpNearMot];
static uint8_t s_coldump_near_valid[kMwColdumpNearMot];
/* Defined with cam-capture buffers below; elev dump prefers present snapshot. */
static unsigned s_cam_n[2];
static uint8_t s_cam_local_only[2][128];
/* 1 = shared $B1 item tile — present via multi-tile sticky only (not raw). */
static uint8_t s_cam_shared_item[2][128];
/* Per tile: -1 = shared stage prop; 0/1 = dual-slot owner (object +$06 hi). */
static int8_t s_cam_prop_owner[2][128];

/*
 * Present-only: bank-$B1 *movers* that need per-peer home isolation.
 * Whitelist only — a bare `meta≠0 && meta≠$D5B8` tagged pickups/items
 * (e.g. `$9FE2` crates) as local_only; nearer-mech home then permanently
 * culled them on the non-home peer (same class of bug as mech OAM cull).
 * Elevators `$D5B8` stay out (BG2 path). Add metas here when a new mover
 * needs isolation.
 *
 * Living catalog / coldump / manipulation notes:
 *   docs/H2H_STAGE_PROPS.md
 */
static int mw_is_stage_prop_meta(uint16_t meta) {
  switch (meta) {
  case 0xC382u: /* hazard stripe + brown body */
  case 0xC39Eu:
  case 0xC6A4u:
  case 0xC400u:
  case 0xC5F2u:
  case 0xC3ECu:
  case 0xC3C4u:
    return 1;
  default:
    return 0;
  }
}

/* Bank-$B1 pickups/items: shared world OAM (not home-isolated movers). */
static int mw_is_shared_b1_item_meta(uint16_t meta) {
  return meta != 0 && meta != 0xD5B8u && !mw_is_stage_prop_meta(meta);
}

static int mw_obj_is_stage_prop(uint16_t obj) {
  if (obj < 0x1934u || obj >= 0x2000u)
    return 0;
  return mw_wram16((uint16_t)(obj + 0xAu)) == 0x00B1u &&
         mw_is_stage_prop_meta(mw_wram16((uint16_t)(obj + 8u)));
}

/*
 * Catalog $C382 hazard anchors (world XY is stable; pool slots recycle).
 * Pin present meta + widen foreign BG1 blank fingerprint — see
 * docs/H2H_STAGE_PROPS.md § pin experiment. Tune Y with
 * SNESRECOMP_MW_PROP_PIN_DY (pixels added to present moy; default 0).
 */
static int mw_prop_is_pinned_hazard(uint16_t obj) {
  if (!mw_obj_is_stage_prop(obj))
    return 0;
  if (mw_wram16((uint16_t)(obj + 8u)) != 0xC382u)
    return 0;
  const uint16_t wx = mw_wram16((uint16_t)(obj + 2u));
  const uint16_t wy = mw_wram16((uint16_t)(obj + 4u));
  if (wy == 365u && (wx == 637u || wx == 832u))
    return 1; /* mid hazard ledge (left/right extents) */
  if (wy == 621u && wx == 220u)
    return 1; /* floating crate */
  if (wy == 106u && (wx == 220u || wx == 1249u))
    return 1; /* upstairs */
  return 0;
}

static int mw_prop_pin_dy(void) {
  static int inited, dy;
  if (!inited) {
    inited = 1;
    const char *e = getenv("SNESRECOMP_MW_PROP_PIN_DY");
    dy = e ? atoi(e) : 0;
  }
  return dy;
}

/*
 * Soak manip probe: $C39E @ (350,490) only — present XY nudge + optional
 * synthetic $C8 seed. Does NOT use pin FP / PROP_PIN_DY (that corrupted
 * walls). Env: SNESRECOMP_MW_PROP_C39E_DX / _DY (pixels). Unset = off.
 * See docs/H2H_STAGE_PROPS.md §8b step 2.
 */
static int mw_prop_is_c39e_soak_probe(uint16_t obj) {
  if (!mw_obj_is_stage_prop(obj))
    return 0;
  if (mw_wram16((uint16_t)(obj + 8u)) != 0xC39Eu)
    return 0;
  return mw_wram16((uint16_t)(obj + 2u)) == 350u &&
         mw_wram16((uint16_t)(obj + 4u)) == 490u;
}

static int mw_prop_c39e_probe_dx(void) {
  static int inited, dx;
  if (!inited) {
    inited = 1;
    const char *e = getenv("SNESRECOMP_MW_PROP_C39E_DX");
    dx = e ? atoi(e) : 0;
  }
  return dx;
}

static int mw_prop_c39e_probe_dy(void) {
  static int inited, dy;
  if (!inited) {
    inited = 1;
    const char *e = getenv("SNESRECOMP_MW_PROP_C39E_DY");
    dy = e ? atoi(e) : 0;
  }
  return dy;
}

static int mw_prop_c39e_probe_active(void) {
  return mw_prop_c39e_probe_dx() != 0 || mw_prop_c39e_probe_dy() != 0;
}

static void mw_prop_apply_c39e_soak_probe(uint16_t obj, int *use_ox,
                                          int *use_oy) {
  if (!use_ox || !use_oy || !mw_prop_is_c39e_soak_probe(obj) ||
      !mw_prop_c39e_probe_active())
    return;
  *use_ox += mw_prop_c39e_probe_dx();
  *use_oy += mw_prop_c39e_probe_dy();
}

/*
 * Wide foreign BG1 fingerprint — pin catalog only. Global wide-$C382 FP
 * scrambled shared wall tiles; anchors have stable world XY and need the
 * longer east sample (origin sits west of the brown body).
 */
static int mw_prop_c382_wide_fp(uint16_t obj) {
  return mw_prop_is_pinned_hazard(obj);
}

/* Mid hazard ledge is two pool objs; nearer-mech home was splitting them. */
static int mw_prop_is_mid_hazard_ledge(uint16_t obj) {
  if (!mw_obj_is_stage_prop(obj))
    return 0;
  if (mw_wram16((uint16_t)(obj + 8u)) != 0xC382u)
    return 0;
  const uint16_t wx = mw_wram16((uint16_t)(obj + 2u));
  const uint16_t wy = mw_wram16((uint16_t)(obj + 4u));
  return wy == 365u && (wx == 637u || wx == 832u);
}

static void mw_prop_mid_ledge_centroid(int *ax, int *ay) {
  if (ax)
    *ax = (637 + 832) / 2; /* 734 */
  if (ay)
    *ay = 365;
}

/*
 * Forever home-lock once firm (no 16× handoff). $C39E @1118,490 sat on P1's
 * bottom then flipped onto P2's top. Mid ledge is NOT forever — exclusive
 * cam FOV may reassign (P1-view ledge was stuck h=P2 and ghosted on P2).
 */
static int mw_prop_home_lock_forever(uint16_t obj) {
  if (!mw_obj_is_stage_prop(obj))
    return 0;
  if (mw_wram16((uint16_t)(obj + 8u)) != 0xC39Eu)
    return 0;
  const uint16_t wx = mw_wram16((uint16_t)(obj + 2u));
  const uint16_t wy = mw_wram16((uint16_t)(obj + 4u));
  return wx == 1118u && wy == 490u;
}

/*
 * Dual-viewport slot on object +$06 — hi-byte only (mechs):
 *   $01xx → cam0, $02xx → cam1. -1 = no hi-byte tag.
 * Low-byte tags ($0002/$0004/$0008 on $C382 movers) are NOT viewport owners.
 * Stage-prop metas often carry $02xx too ($C6A4) — that is NOT used for
 * mover home (see mw_stage_prop_home_cam); mech finder still uses this.
 */
static int mw_obj_stage_prop_owner(uint16_t obj) {
  if (obj < 0x1934u || obj >= 0x2000u)
    return -1;
  const uint16_t t = mw_wram16((uint16_t)(obj + 6u));
  if ((t & 0xFF00u) == 0x0100u)
    return 0;
  if ((t & 0xFF00u) == 0x0200u)
    return 1;
  return -1;
}

/*
 * Home cam for stage props.
 * Do NOT trust +$06 hi-byte on movers — coldump `$C6A4` with `$0200` forced
 * home=P2 while live +$02 cam-glued to that peer (ghost follows P2; map
 * brown on P1 stayed put). Nearer dual-slot mech + firm latch instead.
 * Forever-latch from a one-shot nearer-cam / single-mech guess was wrong —
 * playtest `$1AD8` @`$045E` (+$06=$0006) latched owner=0 and converted into
 * cam0 (P1 ghost / P2 empty sky). No camera-center fallback: return -1
 * until a mech exists (present/BG1 suppress until then).
 * Soft home until both mechs seen, then firm. Mid ledge: exclusive cam FOV
 * beats nearer-mech (may reassign when firm). Handoff-lock catalog
 * ($C39E @1118,490) stays forever. Other firm props: no flip while still
 * in the home peer's FOV; else 16×. Single-mech flicker must not stomp firm.
 */
enum { kMwPropHomeMax = 24 };
static uint16_t s_prop_home_obj[kMwPropHomeMax];
static int8_t s_prop_home_cam[kMwPropHomeMax];
/* Firm = both mechs seen; stops single-mech flicker from flipping home. */
static uint8_t s_prop_home_firm[kMwPropHomeMax];
/* Last world pos per stage prop — blank trails when movers leave a peer. */
static uint16_t s_prop_trail_obj[kMwPropHomeMax];
static uint16_t s_prop_trail_wx[kMwPropHomeMax];
static uint16_t s_prop_trail_wy[kMwPropHomeMax];
static uint8_t s_prop_trail_valid[kMwPropHomeMax];
/*
 * Multi-tile sticky for home-isolated movers ($C382 etc). Dual-drawer OAM
 * pressure often keeps only 1 of N meta tiles per frame — single-tile sticky
 * permanently half-culled the sprite, and present skipped sticky once any raw
 * tile placed. Accumulate tiles (tile id + meta ox/oy) across frames; present
 * places the full set for the home peer only.
 */
enum { kMwPropStickyTiles = 8 };
static uint16_t s_prop_sticky_obj[kMwPropHomeMax];
static uint8_t s_prop_sticky_n[kMwPropHomeMax];
static uint8_t s_prop_sticky_spr[kMwPropHomeMax][kMwPropStickyTiles][4];
static uint8_t s_prop_sticky_sz[kMwPropHomeMax][kMwPropStickyTiles];
static int16_t s_prop_sticky_mox[kMwPropHomeMax][kMwPropStickyTiles];
static int16_t s_prop_sticky_moy[kMwPropHomeMax][kMwPropStickyTiles];
static uint8_t s_prop_sticky_valid[kMwPropHomeMax];
/*
 * Home $C6A4 brown body: dual $7F stomps leave the strip sky while OAM sticky
 * stays up → flicker. Latch opaque $7F cells when solid; paint at present
 * use_wx/wy (world-locked anchor when +$02 cam-follows). Not a full-strip FP.
 */
enum { kMwRiderBrownCells = 48 };
static uint16_t s_rider_brown_obj[kMwPropHomeMax];
static uint8_t s_rider_brown_valid[kMwPropHomeMax];
static uint8_t s_rider_brown_n[kMwPropHomeMax];
static uint16_t s_rider_brown_ox[kMwPropHomeMax]; /* world origin at latch */
static uint16_t s_rider_brown_oy[kMwPropHomeMax];
static int16_t s_rider_brown_dx[kMwPropHomeMax][kMwRiderBrownCells];
static int16_t s_rider_brown_dy[kMwPropHomeMax][kMwRiderBrownCells];
static uint16_t s_rider_brown_t[kMwPropHomeMax][kMwRiderBrownCells];
/* Last present paint origin — scrub when use XY moves (cam-follow ghost). */
static uint16_t s_rider_brown_paint_ox[kMwPropHomeMax];
static uint16_t s_rider_brown_paint_oy[kMwPropHomeMax];
static uint8_t s_rider_brown_paint_have[kMwPropHomeMax];
/*
 * Present/blank world XY when live +$02 tracks the home cam (sx glued while
 * cam/wx move together). Latch a stable anchor; keep it across glue streaks.
 */
static uint16_t s_prop_anchor_obj[kMwPropHomeMax];
static uint16_t s_prop_anchor_wx[kMwPropHomeMax];
static uint16_t s_prop_anchor_wy[kMwPropHomeMax];
static uint8_t s_prop_anchor_valid[kMwPropHomeMax];
static int16_t s_prop_glue_sx[kMwPropHomeMax];
static uint16_t s_prop_glue_locx[kMwPropHomeMax];
static uint16_t s_prop_glue_wx[kMwPropHomeMax];
static uint16_t s_prop_glue_mechx[kMwPropHomeMax];
static uint8_t s_prop_glue_mech_have[kMwPropHomeMax];
static uint8_t s_prop_glue_have[kMwPropHomeMax];
static uint8_t s_prop_glue_hits[kMwPropHomeMax];
static uint8_t s_prop_glue_clear[kMwPropHomeMax];
/* $C6A4: consecutive no-cam |dwx|≥2 frames before X retarget (N=3). */
static uint8_t s_prop_c6_travel_hits[kMwPropHomeMax];
static int s_prop_glue_tick_f[kMwPropHomeMax];
static int s_prop_glue_tick_slot[kMwPropHomeMax];
/* Last resolved present world XY — coldump + BG1 brown must match OAM. */
static uint16_t s_prop_use_obj[kMwPropHomeMax];
static uint16_t s_prop_use_wx[kMwPropHomeMax];
static uint16_t s_prop_use_wy[kMwPropHomeMax];
static uint8_t s_prop_use_glued[kMwPropHomeMax];
static uint8_t s_prop_use_valid[kMwPropHomeMax];

/*
 * Orphan hazard stripe ($62/$C8) not on prop sticky: capture wx=cam+sx
 * tracks the camera (coldump sx stuck at 83 while loc pans; BG1 brown
 * stays world-fixed). Latch world XY on first sight / true travel.
 */
enum { kMwOrphanStripeMax = 4 };
static uint8_t s_ostripe_tile[kMwOrphanStripeMax];
static uint16_t s_ostripe_wx[kMwOrphanStripeMax];
static uint16_t s_ostripe_wy[kMwOrphanStripeMax];
static uint16_t s_ostripe_live_wx[kMwOrphanStripeMax];
static uint16_t s_ostripe_locx[kMwOrphanStripeMax];
static int16_t s_ostripe_sx[kMwOrphanStripeMax];
static uint8_t s_ostripe_valid[kMwOrphanStripeMax];

/* Untagged gameplay $E4 on-mech backpack: capture wx freezes while the mech
 * walks (poam s:0 lag). Latch mech-relative offset when tight; hold through
 * stale capture. Free $C6A4 platform $E4 stays far from mech — untouched. */
enum { kMwE4PackMax = 4 };
static int16_t s_e4pack_ox[kMwE4PackMax];
static int16_t s_e4pack_oy[kMwE4PackMax];
static uint16_t s_e4pack_mechx[kMwE4PackMax];
static int8_t s_e4pack_cam[kMwE4PackMax];
static uint8_t s_e4pack_valid[kMwE4PackMax];

static void mw_prop_home_reset(void) {
  memset(s_prop_home_obj, 0, sizeof(s_prop_home_obj));
  memset(s_prop_home_cam, -1, sizeof(s_prop_home_cam));
  memset(s_prop_home_firm, 0, sizeof(s_prop_home_firm));
  memset(s_prop_trail_obj, 0, sizeof(s_prop_trail_obj));
  memset(s_prop_trail_valid, 0, sizeof(s_prop_trail_valid));
  memset(s_prop_sticky_obj, 0, sizeof(s_prop_sticky_obj));
  memset(s_prop_sticky_n, 0, sizeof(s_prop_sticky_n));
  memset(s_prop_sticky_valid, 0, sizeof(s_prop_sticky_valid));
  memset(s_rider_brown_obj, 0, sizeof(s_rider_brown_obj));
  memset(s_rider_brown_valid, 0, sizeof(s_rider_brown_valid));
  memset(s_rider_brown_n, 0, sizeof(s_rider_brown_n));
  memset(s_rider_brown_paint_have, 0, sizeof(s_rider_brown_paint_have));
  memset(s_prop_anchor_valid, 0, sizeof(s_prop_anchor_valid));
  memset(s_prop_glue_have, 0, sizeof(s_prop_glue_have));
  memset(s_prop_glue_mech_have, 0, sizeof(s_prop_glue_mech_have));
  memset(s_prop_glue_hits, 0, sizeof(s_prop_glue_hits));
  memset(s_prop_glue_clear, 0, sizeof(s_prop_glue_clear));
  memset(s_prop_c6_travel_hits, 0, sizeof(s_prop_c6_travel_hits));
  memset(s_prop_glue_tick_f, -1, sizeof(s_prop_glue_tick_f));
  memset(s_prop_glue_tick_slot, -1, sizeof(s_prop_glue_tick_slot));
  memset(s_prop_use_valid, 0, sizeof(s_prop_use_valid));
  memset(s_prop_use_glued, 0, sizeof(s_prop_use_glued));
  memset(s_ostripe_valid, 0, sizeof(s_ostripe_valid));
  memset(s_ostripe_tile, 0, sizeof(s_ostripe_tile));
  memset(s_e4pack_valid, 0, sizeof(s_e4pack_valid));
}

static int mw_prop_slot_for_obj(uint16_t obj) {
  int free_i = -1;
  int trail_i = -1;
  /* Prefer sticky match so trail/sticky never diverge across two slots
   * (trail-first lookup + prop_alive wipe was thrashing sn every few frames). */
  for (int i = 0; i < kMwPropHomeMax; i++) {
    if (s_prop_sticky_obj[i] == obj)
      return i;
    if (trail_i < 0 && s_prop_trail_obj[i] == obj)
      trail_i = i;
    if (free_i < 0 && !s_prop_trail_obj[i] && !s_prop_sticky_obj[i])
      free_i = i;
  }
  if (trail_i >= 0)
    return trail_i;
  if (free_i >= 0)
    return free_i;
  /* Last resort: only steal a slot that has no live sticky tiles. */
  for (int i = 0; i < kMwPropHomeMax; i++) {
    if (!s_prop_sticky_valid[i])
      return i;
  }
  return (int)((obj - 0x1934u) / 0x1Cu) % kMwPropHomeMax;
}

/* Platform meta tiles sit near the object origin — |mox|≈166 is mech steal. */
static int mw_prop_meta_sane(int mox, int moy) {
  return mox >= -48 && mox <= 48 && moy >= -48 && moy <= 48;
}

/* Declared early — sticky_store/scrub reject non-platform tiles under hold-glue. */
static int mw_prop_is_c6a4_rider(uint16_t obj);

/* $C6A4 sticky is $E4 only — low CHR ($0A/$0C/$20…) is mech body, not plat. */
static int mw_prop_c6a4_sticky_tile_ok(uint8_t tile) { return tile == 0xE4u; }

/* Coldump plat.stripe: hazard OAM only. Never $E4 — that is $C6A4 backpack. */
static int mw_plat_stripe_tile_ok(unsigned tile) {
  return tile == 0x62u || /* $C382 hazard stripe */
         tile == 0xC8u;   /* pin synthetic seed */
}

/* Mech-body CHR — exclude from paint.spr ($22/$26 hitch false positives). */
static int mw_paint_mech_oam_tile(unsigned tile) {
  if (tile <= 0x16u)
    return 1;
  if (tile >= 0x20u && tile <= 0x2Eu)
    return 1;
  if (tile >= 0x40u && tile <= 0x4Eu)
    return 1;
  if (tile >= 0x60u && tile <= 0x6Eu)
    return 1; /* $64/$66 were on-mech in soak */
  if (tile >= 0xE0u && tile <= 0xF0u)
    return 1;
  return 0;
}

static int mw_paint_midview_xy(int sx, int sy) {
  return sx >= 16 && sx < 256 && sy >= -16 && sy < 200;
}

/* Stand candidates must not sit on the mech sprite blob. */
static int mw_paint_away_from_mech(int sx, int sy, int mech_sx, int mech_sy,
                                   int have_mech) {
  if (!have_mech)
    return 1;
  const int dx = sx > mech_sx ? sx - mech_sx : mech_sx - sx;
  const int dy = sy > mech_sy ? sy - mech_sy : mech_sy - sy;
  return dx > 40 || dy > 48;
}

/*
 * World XY for untagged hazard stripe OAM. Capture often stores cam+sx with
 * sx FOV-stuck (dsx≈0, dwx≈dcam) while map brown world-locks — the orange/
 * black trim then rides the viewport and snaps when the drawer retargets.
 * Hold a world latch through cam-follow; retarget on true travel or >64px
 * teleport. Prefer prop sticky when the tile is already latched there.
 */
static void mw_orphan_stripe_world_xy(uint8_t tile, int live_wx, int live_wy,
                                      uint16_t loc_x, int *out_x, int *out_y) {
  int slot = -1;
  for (int i = 0; i < kMwOrphanStripeMax; i++) {
    if (s_ostripe_valid[i] && s_ostripe_tile[i] == tile) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    for (int i = 0; i < kMwOrphanStripeMax; i++) {
      if (!s_ostripe_valid[i]) {
        slot = i;
        break;
      }
    }
    if (slot < 0)
      slot = 0;
    s_ostripe_tile[slot] = tile;
    s_ostripe_wx[slot] = (uint16_t)live_wx;
    s_ostripe_wy[slot] = (uint16_t)live_wy;
    s_ostripe_live_wx[slot] = (uint16_t)live_wx;
    s_ostripe_locx[slot] = loc_x;
    s_ostripe_sx[slot] = (int16_t)(live_wx - (int)loc_x);
    s_ostripe_valid[slot] = 1;
  } else {
    const int sx = live_wx - (int)loc_x;
    const int dcam = (int)loc_x - (int)s_ostripe_locx[slot];
    const int dwx = live_wx - (int)s_ostripe_live_wx[slot];
    const int dsx = sx - (int)s_ostripe_sx[slot];
    int cam_follow = 0;
    int world_step = 0;
    int travel = 0;
    if (dcam != 0) {
      if (dsx <= 1 && dsx >= -1 && dwx - dcam <= 1 && dwx - dcam >= -1)
        cam_follow = 1;
      if (dsx + dcam <= 1 && dsx + dcam >= -1 && dwx <= 1 && dwx >= -1)
        world_step = 1;
      if (world_step)
        cam_follow = 0;
      if (!cam_follow && !world_step && (dwx > 1 || dwx < -1))
        travel = 1;
    } else if (dwx > 1 || dwx < -1) {
      travel = 1;
    }
    const int adx = live_wx - (int)s_ostripe_wx[slot];
    if (travel || world_step || adx > 64 || adx < -64) {
      s_ostripe_wx[slot] = (uint16_t)live_wx;
      s_ostripe_wy[slot] = (uint16_t)live_wy;
    } else if (cam_follow) {
      /* Hold world X; Y tracks in case the drawer lifts the trim. */
      s_ostripe_wy[slot] = (uint16_t)live_wy;
    } else {
      s_ostripe_wx[slot] = (uint16_t)live_wx;
      s_ostripe_wy[slot] = (uint16_t)live_wy;
    }
    s_ostripe_live_wx[slot] = (uint16_t)live_wx;
    s_ostripe_locx[slot] = loc_x;
    s_ostripe_sx[slot] = (int16_t)sx;
  }
  if (out_x)
    *out_x = (int)s_ostripe_wx[slot];
  if (out_y)
    *out_y = (int)s_ostripe_wy[slot];
}

/* True when tile is latched on any stage-prop sticky (home or foreign). */
static int mw_oam_tile_in_prop_sticky(uint8_t tile) {
  for (int s = 0; s < kMwPropHomeMax; s++) {
    if (!s_prop_sticky_valid[s] || !s_prop_sticky_obj[s])
      continue;
    if (!mw_obj_is_stage_prop(s_prop_sticky_obj[s]))
      continue;
    for (unsigned i = 0; i < s_prop_sticky_n[s]; i++) {
      if (s_prop_sticky_spr[s][i][2] == tile)
        return 1;
    }
  }
  return 0;
}

/* Drop polluted sticky tiles (empty-seed latched mech chunks / stolen shots). */
static void mw_prop_sticky_scrub(int s) {
  if (s < 0 || s >= kMwPropHomeMax || !s_prop_sticky_valid[s])
    return;
  const int c6a4 =
      s_prop_sticky_obj[s] && mw_prop_is_c6a4_rider(s_prop_sticky_obj[s]);
  unsigned w = 0;
  for (unsigned i = 0; i < s_prop_sticky_n[s]; i++) {
    if (!mw_prop_meta_sane((int)s_prop_sticky_mox[s][i],
                           (int)s_prop_sticky_moy[s][i]))
      continue;
    if (c6a4 && !mw_prop_c6a4_sticky_tile_ok(s_prop_sticky_spr[s][i][2]))
      continue;
    if (w != i) {
      memcpy(s_prop_sticky_spr[s][w], s_prop_sticky_spr[s][i], 4);
      s_prop_sticky_sz[s][w] = s_prop_sticky_sz[s][i];
      s_prop_sticky_mox[s][w] = s_prop_sticky_mox[s][i];
      s_prop_sticky_moy[s][w] = s_prop_sticky_moy[s][i];
    }
    w++;
  }
  s_prop_sticky_n[s] = (uint8_t)w;
  if (w == 0) {
    s_prop_sticky_valid[s] = 0;
    s_prop_sticky_obj[s] = 0;
  }
}

static void mw_prop_sticky_clear_obj(uint16_t obj) {
  if (obj < 0x1934u || obj >= 0x2000u)
    return;
  const int s = mw_prop_slot_for_obj(obj);
  if (s_prop_sticky_obj[s] != obj)
    return;
  s_prop_sticky_valid[s] = 0;
  s_prop_sticky_n[s] = 0;
  s_prop_sticky_obj[s] = 0;
  s_prop_anchor_valid[s] = 0;
  s_prop_glue_have[s] = 0;
  s_prop_glue_mech_have[s] = 0;
  s_prop_glue_hits[s] = 0;
  s_prop_glue_clear[s] = 0;
  s_prop_c6_travel_hits[s] = 0;
  s_prop_glue_tick_f[s] = -1;
  if (s_prop_use_obj[s] == obj)
    s_prop_use_valid[s] = 0;
  if (s_rider_brown_obj[s] == obj) {
    s_rider_brown_valid[s] = 0;
    s_rider_brown_n[s] = 0;
    s_rider_brown_obj[s] = 0;
    s_rider_brown_paint_have[s] = 0;
  }
}

static void mw_rider_brown_clear_slot(int s) {
  if (s < 0 || s >= kMwPropHomeMax)
    return;
  s_rider_brown_valid[s] = 0;
  s_rider_brown_n[s] = 0;
  s_rider_brown_obj[s] = 0;
  s_rider_brown_paint_have[s] = 0;
}

/* update_meta: 1 = seed new tile mox/moy; 0 = pixels only.
 * Existing sane tiles never rewrite meta — first latch wins (cam-whip safe).
 * Insane latched meta may be repaired by a later sane home seed.
 * Match tile id + meta offset — $C6A4 uses two $E4 tiles at mox 0 and −16;
 * id-only merge permanently half-culled the platform (sn=1 flicker). */
static void mw_prop_sticky_store(uint16_t obj, const uint8_t spr[4], uint8_t sz,
                                 int16_t mox, int16_t moy, int update_meta) {
  if (obj < 0x1934u || obj >= 0x2000u || !spr)
    return;
  if (mw_prop_is_c6a4_rider(obj) && !mw_prop_c6a4_sticky_tile_ok(spr[2]))
    return;
  const int s = mw_prop_slot_for_obj(obj);
  if (!s_prop_sticky_valid[s] || s_prop_sticky_obj[s] != obj) {
    s_prop_sticky_obj[s] = obj;
    s_prop_sticky_n[s] = 0;
    s_prop_sticky_valid[s] = 1;
  }
  for (unsigned i = 0; i < s_prop_sticky_n[s]; i++) {
    if (s_prop_sticky_spr[s][i][2] != spr[2])
      continue;
    const int dx = (int)s_prop_sticky_mox[s][i] - (int)mox;
    const int dy = (int)s_prop_sticky_moy[s][i] - (int)moy;
    if (dx > 2 || dx < -2 || dy > 2 || dy < -2)
      continue;
    memcpy(s_prop_sticky_spr[s][i], spr, 4);
    s_prop_sticky_sz[s][i] = sz;
    /* Repair polluted meta; never overwrite a sane latch. */
    if (update_meta && mw_prop_meta_sane((int)mox, (int)moy) &&
        !mw_prop_meta_sane((int)s_prop_sticky_mox[s][i],
                           (int)s_prop_sticky_moy[s][i])) {
      s_prop_sticky_mox[s][i] = mox;
      s_prop_sticky_moy[s][i] = moy;
    }
    return;
  }
  if (!update_meta || !mw_prop_meta_sane((int)mox, (int)moy))
    return;
  if (s_prop_sticky_n[s] >= kMwPropStickyTiles)
    return;
  const unsigned d = s_prop_sticky_n[s]++;
  memcpy(s_prop_sticky_spr[s][d], spr, 4);
  s_prop_sticky_sz[s][d] = sz;
  s_prop_sticky_mox[s][d] = mox;
  s_prop_sticky_moy[s][d] = moy;
}

/*
 * Shared bank-$B1 pickups/items (e.g. $9FE2): multi-tile sticky so dual-drawer
 * OAM pressure cannot leave a permanently half-culled sprite. Tiles are stored
 * with world meta offsets; present places the full set for BOTH peers from
 * live +$02/+$04 (not home-isolated).
 */
enum { kMwItemStickyMax = 16, kMwItemStickyTiles = 8 };
static uint16_t s_item_sticky_obj[kMwItemStickyMax];
static uint8_t s_item_sticky_n[kMwItemStickyMax];
static uint8_t s_item_sticky_spr[kMwItemStickyMax][kMwItemStickyTiles][4];
static uint8_t s_item_sticky_sz[kMwItemStickyMax][kMwItemStickyTiles];
static int16_t s_item_sticky_mox[kMwItemStickyMax][kMwItemStickyTiles];
static int16_t s_item_sticky_moy[kMwItemStickyMax][kMwItemStickyTiles];
static uint8_t s_item_sticky_valid[kMwItemStickyMax];

static void mw_item_sticky_reset(void) {
  memset(s_item_sticky_obj, 0, sizeof(s_item_sticky_obj));
  memset(s_item_sticky_n, 0, sizeof(s_item_sticky_n));
  memset(s_item_sticky_valid, 0, sizeof(s_item_sticky_valid));
}

static int mw_item_sticky_slot(uint16_t obj) {
  int free_i = -1;
  for (int i = 0; i < kMwItemStickyMax; i++) {
    if (s_item_sticky_valid[i] && s_item_sticky_obj[i] == obj)
      return i;
    if (free_i < 0 && !s_item_sticky_valid[i])
      free_i = i;
  }
  if (free_i >= 0)
    return free_i;
  return (int)((obj - 0x1934u) / 0x1Cu) % kMwItemStickyMax;
}

static void mw_item_sticky_store(uint16_t obj, const uint8_t spr[4], uint8_t sz,
                                 int16_t mox, int16_t moy) {
  if (obj < 0x1934u || obj >= 0x2000u || !spr)
    return;
  const int s = mw_item_sticky_slot(obj);
  if (!s_item_sticky_valid[s] || s_item_sticky_obj[s] != obj) {
    s_item_sticky_obj[s] = obj;
    s_item_sticky_n[s] = 0;
    s_item_sticky_valid[s] = 1;
  }
  for (unsigned i = 0; i < s_item_sticky_n[s]; i++) {
    if (s_item_sticky_spr[s][i][2] != spr[2])
      continue;
    const int dx = (int)s_item_sticky_mox[s][i] - (int)mox;
    const int dy = (int)s_item_sticky_moy[s][i] - (int)moy;
    if (dx <= 1 && dx >= -1 && dy <= 1 && dy >= -1) {
      memcpy(s_item_sticky_spr[s][i], spr, 4);
      s_item_sticky_sz[s][i] = sz;
      s_item_sticky_mox[s][i] = mox;
      s_item_sticky_moy[s][i] = moy;
      return;
    }
  }
  if (s_item_sticky_n[s] >= kMwItemStickyTiles)
    return;
  const unsigned d = s_item_sticky_n[s]++;
  memcpy(s_item_sticky_spr[s][d], spr, 4);
  s_item_sticky_sz[s][d] = sz;
  s_item_sticky_mox[s][d] = mox;
  s_item_sticky_moy[s][d] = moy;
}

static int mw_obj_is_shared_b1_item(uint16_t obj) {
  if (obj < 0x1934u || obj >= 0x2000u)
    return 0;
  if (mw_wram16((uint16_t)(obj + 0xAu)) != 0x00B1u)
    return 0;
  return mw_is_shared_b1_item_meta(mw_wram16((uint16_t)(obj + 8u)));
}

/* Find world X/Y of the dual-slot mech (hi-byte +$06). 0 if missing.
 * Skip stage props: $C3EC etc. can carry $02xx owner tags and would steal
 * "mech1" for home/hysteresis (coldump glued platforms to the prop). */
static int mw_find_dual_mech_xy(int slot, int *ox, int *oy) {
  const uint16_t want = (slot == 1) ? 0x0200u : 0x0100u;
  uint16_t idx = mw_wram16(0x1E14u);
  for (int guard = 0; guard < 64 && idx; guard++) {
    const uint16_t fl = mw_wram16(idx);
    const uint16_t t6 = mw_wram16((uint16_t)(idx + 6u));
    const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
    if ((fl & 0x8000u) && (t6 & 0xFF00u) == want &&
        !mw_obj_is_stage_prop(idx)) {
      if (ox)
        *ox = (int)mw_wram16((uint16_t)(idx + 2u));
      if (oy)
        *oy = (int)mw_wram16((uint16_t)(idx + 4u));
      return 1;
    }
    if (next == 0 || next == idx)
      break;
    idx = next;
  }
  return 0;
}

/*
 * Gameplay-path $E4 backpack (not prop sticky). Capture wx freezes while the
 * mech walks → poam s:0 falls behind. Latch mech-relative offset when tight
 * (≤16×28); hold while capture stays near the expected seat. Far $E4 (free
 * platform / unrelated) keeps capture world XY — never CHR-allowlist lock.
 */
static void mw_orphan_e4_backpack_xy(int cap_wx, int cap_wy, int local_slot,
                                     int *out_x, int *out_y) {
  int mx = 0, my = 0;
  if (!mw_find_dual_mech_xy(local_slot, &mx, &my)) {
    if (out_x)
      *out_x = cap_wx;
    if (out_y)
      *out_y = cap_wy;
    return;
  }
  const int dx = cap_wx - mx;
  const int dy = cap_wy - my;
  const int adx = dx < 0 ? -dx : dx;
  const int ady = dy < 0 ? -dy : dy;

  int slot = -1;
  for (int i = 0; i < kMwE4PackMax; i++) {
    if (!s_e4pack_valid[i] || (int)s_e4pack_cam[i] != local_slot)
      continue;
    const int expect_wx = mx + (int)s_e4pack_ox[i];
    const int expect_wy = my + (int)s_e4pack_oy[i];
    const int aedx = cap_wx > expect_wx ? cap_wx - expect_wx : expect_wx - cap_wx;
    const int aedy = cap_wy > expect_wy ? cap_wy - expect_wy : expect_wy - cap_wy;
    const int odx = dx - (int)s_e4pack_ox[i];
    const int ody = dy - (int)s_e4pack_oy[i];
    if (adx <= 48 && ady <= 40 && odx <= 8 && odx >= -8 && ody <= 8 &&
        ody >= -8) {
      slot = i;
      break;
    }
    if (aedx <= 48 && aedy <= 40) {
      slot = i;
      break;
    }
  }

  if (adx <= 16 && ady <= 28) {
    if (slot < 0) {
      for (int i = 0; i < kMwE4PackMax; i++) {
        if (!s_e4pack_valid[i]) {
          slot = i;
          break;
        }
      }
      if (slot < 0)
        slot = 0;
    }
    s_e4pack_ox[slot] = (int16_t)dx;
    s_e4pack_oy[slot] = (int16_t)dy;
    s_e4pack_mechx[slot] = (uint16_t)mx;
    s_e4pack_cam[slot] = (int8_t)local_slot;
    s_e4pack_valid[slot] = 1;
    if (out_x)
      *out_x = mx + dx;
    if (out_y)
      *out_y = my + dy;
    return;
  }

  if (slot >= 0) {
    const int dmx = mx - (int)s_e4pack_mechx[slot];
    if (dmx > 80 || dmx < -80) {
      s_e4pack_valid[slot] = 0;
      if (out_x)
        *out_x = cap_wx;
      if (out_y)
        *out_y = cap_wy;
      return;
    }
    s_e4pack_mechx[slot] = (uint16_t)mx;
    if (out_x)
      *out_x = mx + (int)s_e4pack_ox[slot];
    if (out_y)
      *out_y = my + (int)s_e4pack_oy[slot];
    return;
  }

  if (out_x)
    *out_x = cap_wx;
  if (out_y)
    *out_y = cap_wy;
}

/* Store / update soft home hint. firm=1 once both mechs have been seen. */
static void mw_prop_home_store(uint16_t obj, int spat, int firm) {
  int slot = -1;
  for (int i = 0; i < kMwPropHomeMax; i++) {
    if (s_prop_home_obj[i] == obj) {
      slot = i;
      break;
    }
    if (slot < 0 && !s_prop_home_obj[i])
      slot = i;
  }
  if (slot < 0)
    slot = (int)((obj - 0x1934u) / 0x1Cu) % kMwPropHomeMax;
  s_prop_home_obj[slot] = obj;
  s_prop_home_cam[slot] = (int8_t)spat;
  if (firm)
    s_prop_home_firm[slot] = 1;
}

static int mw_prop_home_lookup(uint16_t obj) {
  for (int i = 0; i < kMwPropHomeMax; i++) {
    if (s_prop_home_obj[i] == obj)
      return (int)s_prop_home_cam[i];
  }
  return -1;
}

static int mw_prop_home_is_firm(uint16_t obj) {
  for (int i = 0; i < kMwPropHomeMax; i++) {
    if (s_prop_home_obj[i] == obj)
      return s_prop_home_firm[i] ? 1 : 0;
  }
  return 0;
}

/* Keep both mid-ledge ends on the same home cam. Wipe sticky on flip so
 * P1-viewport-clipped tiles are not drawn by the new home peer. */
static void mw_prop_mid_ledge_sync_home(uint16_t obj, int spat, int firm) {
  const int prev = mw_prop_home_lookup(obj);
  mw_prop_home_store(obj, spat, firm);
  const uint16_t wx = mw_wram16((uint16_t)(obj + 2u));
  const uint16_t want = (wx == 637u) ? 832u : 637u;
  uint16_t other = 0;
  uint16_t idx = mw_wram16(0x1E14u);
  for (int guard = 0; guard < 64 && idx; guard++) {
    const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
    if (mw_prop_is_mid_hazard_ledge(idx) &&
        mw_wram16((uint16_t)(idx + 2u)) == want) {
      mw_prop_home_store(idx, spat, firm);
      other = idx;
    }
    if (next == 0 || next == idx)
      break;
    idx = next;
  }
  if (prev >= 0 && prev != spat) {
    mw_prop_sticky_clear_obj(obj);
    if (other)
      mw_prop_sticky_clear_obj(other);
  }
}

/*
 * Mover *body* intersects the local full-frame view (not origin alone).
 * Used for BG1 foreign blank / ink + capture empty-seed gate — not OAM
 * present (OAM is home-only). Origins often sit far left (sx≈−300) while
 * brown still fills the margin; origin-only FOV blanked those wrongly.
 */
/*
 * Origin far above/below the local frame (upstairs $C382 sy≈−159). Body Y
 * slop alone must not count as in-FOV — that kept home brown + skipped blank.
 */
static int mw_prop_origin_far_y(int owy, uint16_t loc_y) {
  const int sy = owy - (int)loc_y;
  return sy < -112 || sy >= 288;
}

static int mw_prop_in_local_fov(int owx, int owy, uint16_t loc_x,
                                uint16_t loc_y) {
  const int extra =
      (g_ws_active && g_ws_extra > 0) ? IntMin(g_ws_extra, kWsExtraMax) : 0;
  const int sx = owx - (int)loc_x;
  const int sy = owy - (int)loc_y;
  if (mw_prop_origin_far_y(owy, loc_y))
    return 0;
  /* Pocket / keepout-ish body: origin can be west of the visible ledge. */
  const int body_l = sx - 48;
  const int body_r = sx + 256;
  const int body_t = sy - 64;
  const int body_b = sy + 128;
  const int view_l = -extra - 16;
  const int view_r = 256 + extra + 16;
  const int view_t = -96;
  const int view_b = 224 + 112;
  if (body_r < view_l || body_l > view_r)
    return 0;
  if (body_b < view_t || body_t > view_b)
    return 0;
  return 1;
}

static int mw_prop_is_c6a4_rider(uint16_t obj) {
  if (obj < 0x1934u || obj >= 0x2000u)
    return 0;
  return mw_wram16((uint16_t)(obj + 8u)) == 0xC6A4u;
}

/* Live near either dual mech (carry / backpack). Coldump $C6A4: dx≈±4 dy≈−14. */
static int mw_prop_live_near_mech(int live_ox, int live_oy) {
  for (int slot = 0; slot < 2; slot++) {
    int mx = 0, my = 0;
    if (!mw_find_dual_mech_xy(slot, &mx, &my))
      continue;
    const int ldx = live_ox - mx;
    const int ldy = live_oy - my;
    const int aldx = ldx < 0 ? -ldx : ldx;
    const int aldy = ldy < 0 ? -ldy : ldy;
    if (aldx <= 48 && aldy <= 32)
      return 1;
  }
  return 0;
}

/*
 * World XY for prop OAM / foreign blank. Live +$02 on tagged movers can
 * track the home camera (sx glued, dwx≈dcam) while $7F brown stays fixed —
 * that reads as a cam-following ghost. Prefer a latched world anchor.
 * Glue ticks only once per (frame, local_slot) so BG1 brown + OAM share XY.
 *
 * Detect on any cam step; 1px pans satisfy both follow and world gates —
 * world-lock wins. Clear with 2-frame hysteresis. Drop glue if
 * |live−anchor|>48 (non-$C6A4).
 *
 * $C6A4 (2026-08-05 coldump): when live is on a mech (dx≈±4, dy≈−14) it is
 * a back-carried pickup — sticky $E4×2 only, origin t7f=$0200 (no brown).
 * Present MUST follow live so OAM stays on the mech; world-lock left it at
 * a stale X (use≪live / off-screen). Free-floating $C6A4 (not near a mech)
 * still uses the dedicated world-lock path. Giant hazard stand under feet
 * with stripe/brown is map/$7F — not this meta.
 */
static void mw_prop_present_world_xy(uint16_t obj, int live_ox, int live_oy,
                                     uint16_t loc_x, int home, int local_slot,
                                     int *out_x, int *out_y) {
  const int ps = mw_prop_slot_for_obj(obj);
  const int c6 = mw_prop_is_c6a4_rider(obj);

  extern int snes_frame_counter;

  /* $C6A4 on-mech = carried pickup: stick present to live. */
  if (c6 && mw_prop_live_near_mech(live_ox, live_oy)) {
    s_prop_glue_hits[ps] = 0;
    s_prop_glue_clear[ps] = 0;
    s_prop_c6_travel_hits[ps] = 0;
    s_prop_glue_have[ps] = 0;
    s_prop_anchor_obj[ps] = obj;
    s_prop_anchor_wx[ps] = (uint16_t)live_ox;
    s_prop_anchor_wy[ps] = (uint16_t)live_oy;
    s_prop_anchor_valid[ps] = 1;
    mw_rider_brown_clear_slot(ps);
    if (out_x)
      *out_x = live_ox;
    if (out_y)
      *out_y = live_oy;
    s_prop_use_obj[ps] = obj;
    s_prop_use_wx[ps] = (uint16_t)live_ox;
    s_prop_use_wy[ps] = (uint16_t)live_oy;
    s_prop_use_glued[ps] = 0;
    s_prop_use_valid[ps] = 1;
    return;
  }

  /* Free-floating $C6A4: dedicated world lock (home and foreign). */
  if (c6) {
    enum { kC6TravelNeed = 3 };
    const int already =
        s_prop_glue_tick_f[ps] == snes_frame_counter &&
        s_prop_glue_tick_slot[ps] == local_slot;
    int cam_corr = 0;
    int remote_step = 0;
    if (!already) {
      const int sx = live_ox - (int)loc_x;
      const int is_home = (home == local_slot);
      int travel_step = 0;
      int commit_travel = 0;
      int feet_resync = 0;
      if (s_prop_glue_have[ps] && s_prop_anchor_obj[ps] == obj) {
        const int dcam = (int)loc_x - (int)s_prop_glue_locx[ps];
        const int dwx = live_ox - (int)s_prop_glue_wx[ps];
        if (dcam != 0 && dwx - dcam <= 1 && dwx - dcam >= -1)
          cam_corr = 1;
        /* Local cam still, live moved → other peer / shared sim. Hold. */
        if (dcam == 0 && dwx != 0)
          remote_step = 1;
        /* Home scripted X only (not remote cam-follow catch-up). */
        if (is_home && dcam == 0 && (dwx > 1 || dwx < -1))
          travel_step = 1;
        if (cam_corr || remote_step) {
          if (s_prop_glue_hits[ps] < 250u)
            s_prop_glue_hits[ps]++;
          s_prop_glue_clear[ps] = 0;
          if (cam_corr || !is_home)
            s_prop_c6_travel_hits[ps] = 0;
        }
        if (is_home && travel_step && !cam_corr) {
          if (s_prop_c6_travel_hits[ps] < 250u)
            s_prop_c6_travel_hits[ps]++;
          if (s_prop_c6_travel_hits[ps] >= kC6TravelNeed) {
            commit_travel = 1;
            s_prop_glue_hits[ps] = 0;
            s_prop_glue_clear[ps] = 0;
            s_prop_c6_travel_hits[ps] = 0;
          }
        } else if (!travel_step) {
          s_prop_c6_travel_hits[ps] = 0;
        }
      }

      /*
       * Stale-anchor "feet resync" ONLY before hold, and NEVER while live is
       * cam-poisoned. Once glue_hits≥1, following live-under-mech stair-steps
       * the world lock onto the viewport (~33px snaps in coldump).
       */
      if (!cam_corr && !remote_step && s_prop_glue_hits[ps] == 0 &&
          s_prop_anchor_valid[ps] && s_prop_anchor_obj[ps] == obj) {
        int mx = 0, my = 0;
        if (mw_find_dual_mech_xy(local_slot, &mx, &my)) {
          const int ldx = live_ox - mx;
          const int ldy = live_oy - my;
          const int adx = (int)s_prop_anchor_wx[ps] - mx;
          const int aldx = ldx < 0 ? -ldx : ldx;
          const int aldy = ldy < 0 ? -ldy : ldy;
          const int aadx = adx < 0 ? -adx : adx;
          if (aldx <= 48 && aldy <= 32 && aadx > aldx + 32)
            feet_resync = 1;
        }
      }

      s_prop_glue_sx[ps] = (int16_t)sx;
      s_prop_glue_locx[ps] = loc_x;
      s_prop_glue_wx[ps] = (uint16_t)live_ox;
      s_prop_glue_have[ps] = 1;
      s_prop_glue_tick_f[ps] = snes_frame_counter;
      s_prop_glue_tick_slot[ps] = local_slot;

      if (!s_prop_anchor_valid[ps] || s_prop_anchor_obj[ps] != obj) {
        s_prop_anchor_obj[ps] = obj;
        s_prop_anchor_wx[ps] = (uint16_t)live_ox;
        s_prop_anchor_wy[ps] = (uint16_t)live_oy;
        s_prop_anchor_valid[ps] = 1;
        s_prop_c6_travel_hits[ps] = 0;
      } else if (commit_travel) {
        s_prop_anchor_wx[ps] = (uint16_t)live_ox;
        s_prop_anchor_wy[ps] = (uint16_t)live_oy;
      } else if (feet_resync) {
        /* Pre-hold only (gated above). */
        s_prop_anchor_wx[ps] = (uint16_t)live_ox;
        s_prop_anchor_wy[ps] = (uint16_t)live_oy;
        s_prop_glue_hits[ps] = 1;
        s_prop_c6_travel_hits[ps] = 0;
      }
      /* While held: freeze both X and Y (live Y also cam-tracks). */
    } else if (s_prop_glue_have[ps] && s_prop_anchor_obj[ps] == obj) {
      const int dcam = (int)loc_x - (int)s_prop_glue_locx[ps];
      const int dwx = live_ox - (int)s_prop_glue_wx[ps];
      if (dcam != 0 && dwx - dcam <= 1 && dwx - dcam >= -1)
        cam_corr = 1;
      if (dcam == 0 && dwx != 0)
        remote_step = 1;
    }

    const int have = s_prop_anchor_valid[ps] && s_prop_anchor_obj[ps] == obj;
    int ux = have ? (int)s_prop_anchor_wx[ps] : live_ox;
    int uy = have ? (int)s_prop_anchor_wy[ps] : live_oy;
    /*
     * Parked live + far anchor ⇒ pool recycle / stolen pin X.
     * Do NOT run after a cam-chase hold: live sits under the mech at the
     * viewport-poisoned X while use stayed world-locked (|use−live| grows
     * past 48). Resetting there stair-steps the platform onto the camera.
     * Recycle signature: sticky empty, or live not under local feet.
     */
    if (have && s_prop_glue_hits[ps] == 0) {
      const int adx = ux - live_ox;
      const int aadx = adx < 0 ? -adx : adx;
      if (aadx > 48 && !cam_corr && !remote_step) {
        const int sn =
            (s_prop_sticky_valid[ps] && s_prop_sticky_obj[ps] == obj)
                ? (int)s_prop_sticky_n[ps]
                : 0;
        int under_feet = 0;
        int mx = 0, my = 0;
        if (mw_find_dual_mech_xy(local_slot, &mx, &my)) {
          const int ldx = live_ox - mx;
          const int ldy = live_oy - my;
          const int aldx = ldx < 0 ? -ldx : ldx;
          const int aldy = ldy < 0 ? -ldy : ldy;
          if (aldx <= 48 && aldy <= 32)
            under_feet = 1;
        }
        if (sn == 0 || !under_feet) {
          s_prop_anchor_wx[ps] = (uint16_t)live_ox;
          s_prop_anchor_wy[ps] = (uint16_t)live_oy;
          s_prop_glue_hits[ps] = 0;
          s_prop_glue_clear[ps] = 0;
          s_prop_c6_travel_hits[ps] = 0;
          ux = live_ox;
          uy = live_oy;
          mw_rider_brown_clear_slot(ps);
        }
      }
    }
    const int locked = have && (s_prop_glue_hits[ps] >= 1 ||
                                (int)s_prop_anchor_wx[ps] != live_ox ||
                                (int)s_prop_anchor_wy[ps] != live_oy);
    if (!locked) {
      ux = live_ox;
      uy = live_oy;
      if (have) {
        s_prop_anchor_wx[ps] = (uint16_t)live_ox;
        s_prop_anchor_wy[ps] = (uint16_t)live_oy;
      }
    }
    if (out_x)
      *out_x = ux;
    if (out_y)
      *out_y = uy;
    s_prop_use_obj[ps] = obj;
    s_prop_use_wx[ps] = (uint16_t)ux;
    s_prop_use_wy[ps] = (uint16_t)uy;
    s_prop_use_glued[ps] = locked ? 1u : 0u;
    s_prop_use_valid[ps] = 1;
    return;
  }

  const int glued = s_prop_glue_hits[ps] >= 2 && s_prop_anchor_valid[ps] &&
                    s_prop_anchor_obj[ps] == obj;

  if (home != local_slot) {
    const int ux = glued ? (int)s_prop_anchor_wx[ps] : live_ox;
    const int uy = glued ? (int)s_prop_anchor_wy[ps] : live_oy;
    if (out_x)
      *out_x = ux;
    if (out_y)
      *out_y = uy;
    s_prop_use_obj[ps] = obj;
    s_prop_use_wx[ps] = (uint16_t)ux;
    s_prop_use_wy[ps] = (uint16_t)uy;
    s_prop_use_glued[ps] = glued ? 1u : 0u;
    s_prop_use_valid[ps] = 1;
    return;
  }

  const int already =
      s_prop_glue_tick_f[ps] == snes_frame_counter &&
      s_prop_glue_tick_slot[ps] == local_slot;
  if (!already) {
    const int sx = live_ox - (int)loc_x;
    int follow_step = 0;
    int world_step = 0;
    int travel_step = 0;

    if (s_prop_glue_have[ps] && s_prop_anchor_obj[ps] == obj) {
      const int dcam = (int)loc_x - (int)s_prop_glue_locx[ps];
      const int dwx = live_ox - (int)s_prop_glue_wx[ps];
      const int dsx = sx - (int)s_prop_glue_sx[ps];
      if (dcam != 0) {
        /* sx stuck + wx tracks cam → cam-glue; sx mirrors −dcam → world-lock. */
        if (dsx <= 1 && dsx >= -1 && dwx - dcam <= 1 && dwx - dcam >= -1)
          follow_step = 1;
        if (dsx + dcam <= 1 && dsx + dcam >= -1 && dwx <= 1 && dwx >= -1)
          world_step = 1;
        /* |dcam|==1 + world-lock matches both; never arm glue on that. */
        if (world_step)
          follow_step = 0;
        if (!follow_step && !world_step && (dwx > 1 || dwx < -1))
          travel_step = 1;
      }
    }
    if (follow_step) {
      if (s_prop_glue_hits[ps] < 250u)
        s_prop_glue_hits[ps]++;
      s_prop_glue_clear[ps] = 0;
    } else if (world_step || travel_step) {
      s_prop_glue_hits[ps] = 0;
      s_prop_glue_clear[ps] = 0;
    } else if (s_prop_glue_have[ps]) {
      const int dcam = (int)loc_x - (int)s_prop_glue_locx[ps];
      if (dcam != 0) {
        if (s_prop_glue_clear[ps] < 250u)
          s_prop_glue_clear[ps]++;
        if (s_prop_glue_clear[ps] >= 2u)
          s_prop_glue_hits[ps] = 0;
      }
    }
    s_prop_glue_sx[ps] = (int16_t)sx;
    s_prop_glue_locx[ps] = loc_x;
    s_prop_glue_wx[ps] = (uint16_t)live_ox;
    s_prop_glue_have[ps] = 1;
    s_prop_glue_tick_f[ps] = snes_frame_counter;
    s_prop_glue_tick_slot[ps] = local_slot;

    int now_glued = s_prop_glue_hits[ps] >= 2;

    if (!s_prop_anchor_valid[ps] || s_prop_anchor_obj[ps] != obj) {
      s_prop_anchor_obj[ps] = obj;
      s_prop_anchor_wx[ps] = (uint16_t)live_ox;
      s_prop_anchor_wy[ps] = (uint16_t)live_oy;
      s_prop_anchor_valid[ps] = 1;
    } else if (world_step || travel_step) {
      s_prop_anchor_wx[ps] = (uint16_t)live_ox;
      s_prop_anchor_wy[ps] = (uint16_t)live_oy;
    } else if (follow_step) {
      const int adx = (int)s_prop_anchor_wx[ps] - live_ox;
      const int ady = (int)s_prop_anchor_wy[ps] - live_oy;
      if (adx > 48 || adx < -48 || ady > 48 || ady < -48) {
        s_prop_anchor_wx[ps] = (uint16_t)live_ox;
        s_prop_anchor_wy[ps] = (uint16_t)live_oy;
        s_prop_glue_hits[ps] = 0;
        s_prop_glue_clear[ps] = 0;
        now_glued = 0;
      }
    } else if (!now_glued && !follow_step) {
      s_prop_anchor_wx[ps] = (uint16_t)live_ox;
      s_prop_anchor_wy[ps] = (uint16_t)live_oy;
    }

    if (now_glued && s_prop_anchor_valid[ps] &&
        s_prop_anchor_obj[ps] == obj) {
      const int adx = (int)s_prop_anchor_wx[ps] - live_ox;
      const int ady = (int)s_prop_anchor_wy[ps] - live_oy;
      if (adx > 48 || adx < -48 || ady > 48 || ady < -48) {
        s_prop_glue_hits[ps] = 0;
        s_prop_glue_clear[ps] = 0;
        s_prop_anchor_wx[ps] = (uint16_t)live_ox;
        s_prop_anchor_wy[ps] = (uint16_t)live_oy;
        now_glued = 0;
      }
    }
  }

  const int now_glued = s_prop_glue_hits[ps] >= 2 &&
                        s_prop_anchor_valid[ps] &&
                        s_prop_anchor_obj[ps] == obj;
  const int ux = now_glued ? (int)s_prop_anchor_wx[ps] : live_ox;
  const int uy = now_glued ? (int)s_prop_anchor_wy[ps] : live_oy;
  if (out_x)
    *out_x = ux;
  if (out_y)
    *out_y = uy;
  s_prop_use_obj[ps] = obj;
  s_prop_use_wx[ps] = (uint16_t)ux;
  s_prop_use_wy[ps] = (uint16_t)uy;
  s_prop_use_glued[ps] = now_glued ? 1u : 0u;
  s_prop_use_valid[ps] = 1;
}

static int mw_prop_anchor_xy(uint16_t obj, int *out_x, int *out_y) {
  const int ps = mw_prop_slot_for_obj(obj);
  if (!s_prop_anchor_valid[ps] || s_prop_anchor_obj[ps] != obj)
    return 0;
  if (out_x)
    *out_x = (int)s_prop_anchor_wx[ps];
  if (out_y)
    *out_y = (int)s_prop_anchor_wy[ps];
  return 1;
}

static int mw_stage_prop_home_cam(uint16_t obj, uint16_t cam0x, uint16_t cam0y,
                                  uint16_t cam1x, uint16_t cam1y) {
  /* Ignore +$06 hi-byte on movers — see comment above. */
  if (obj < 0x1934u || obj >= 0x2000u)
    return -1;

  int mx0 = 0, my0 = 0, mx1 = 0, my1 = 0;
  const int have0 = mw_find_dual_mech_xy(0, &mx0, &my0);
  const int have1 = mw_find_dual_mech_xy(1, &mx1, &my1);
  if (!have0 && !have1)
    return -1;

  /* Mid ledge (637/832 @ y=365): one home from centroid, both ends synced. */
  int owx = (int)mw_wram16((uint16_t)(obj + 2u));
  int owy = (int)mw_wram16((uint16_t)(obj + 4u));
  const int mid_ledge = mw_prop_is_mid_hazard_ledge(obj);
  const int lock_forever = mw_prop_home_lock_forever(obj);
  if (mid_ledge)
    mw_prop_mid_ledge_centroid(&owx, &owy);
  int spat;
  int32_t d0 = 0, d1 = 0;
  const int both = have0 && have1;
  if (both) {
    const int32_t ax0 = (int32_t)owx - mx0;
    const int32_t ay0 = (int32_t)owy - my0;
    const int32_t ax1 = (int32_t)owx - mx1;
    const int32_t ay1 = (int32_t)owy - my1;
    d0 = ax0 * ax0 + ay0 * ay0;
    d1 = ax1 * ax1 + ay1 * ay1;
    spat = (d1 < d0) ? 1 : 0;
  } else if (have1) {
    spat = 1;
  } else {
    spat = 0;
  }
  /* Mid ledge: exclusive cam FOV beats nearer-mech (stops P2 owning P1 view).
   * When neither FOV (both far_y — P2 at y≈554 vs ledge y=365): nearer-mech
   * wrongly hands home to P2 (dump why=near h=1). Prefer the cam with smaller
   * |sy|; only then fall back to nearer mech. */
  const int in0 = mw_prop_in_local_fov(owx, owy, cam0x, cam0y);
  const int in1 = mw_prop_in_local_fov(owx, owy, cam1x, cam1y);
  if (mid_ledge) {
    if (in0 && !in1)
      spat = 0;
    else if (in1 && !in0)
      spat = 1;
    else if (!in0 && !in1) {
      const int sy0 = owy - (int)cam0y;
      const int sy1 = owy - (int)cam1y;
      const int a0 = sy0 < 0 ? -sy0 : sy0;
      const int a1 = sy1 < 0 ? -sy1 : sy1;
      if (a0 + 32 < a1)
        spat = 0;
      else if (a1 + 32 < a0)
        spat = 1;
    }
  }

  const int cur = mw_prop_home_lookup(obj);
  const int firm = mw_prop_home_is_firm(obj);
  if (cur < 0) {
    if (mid_ledge)
      mw_prop_mid_ledge_sync_home(obj, spat, both);
    else
      mw_prop_home_store(obj, spat, both);
    return spat;
  }

  /*
   * Firm homes: no single-mech stomp. Catalog locks forever. Mid ledge may
   * reassign when only the other peer's FOV contains it. Otherwise keep
   * home while still in that peer's FOV; only then allow a 16× nearer flip.
   */
  if (firm) {
    if (mid_ledge) {
      int want = cur;
      if (in0 && !in1)
        want = 0;
      else if (in1 && !in0)
        want = 1;
      else if (!in0 && !in1)
        want = spat; /* Y-proximity / nearer — do not hold a far-Y owner */
      /* both in FOV: hold firm home (no cam wrestle). */
      mw_prop_mid_ledge_sync_home(obj, want, 1);
      return want;
    }
    if (lock_forever) {
      mw_prop_home_store(obj, cur, 1);
      return cur;
    }
    if (!both) {
      mw_prop_home_store(obj, cur, 1);
      return cur;
    }
    if (cur == spat) {
      mw_prop_home_store(obj, cur, 1);
      return cur;
    }
    {
      const uint16_t hx = cur ? cam1x : cam0x;
      const uint16_t hy = cur ? cam1y : cam0y;
      if (mw_prop_in_local_fov(owx, owy, hx, hy)) {
        mw_prop_home_store(obj, cur, 1);
        return cur;
      }
    }
    const int32_t d_cur = (cur == 0) ? d0 : d1;
    const int32_t d_new = (spat == 0) ? d0 : d1;
    if (d_new * 16 < d_cur) {
      mw_prop_home_store(obj, spat, 1);
      return spat;
    }
    mw_prop_home_store(obj, cur, 1);
    return cur;
  }

  /* Soft (not yet firm): single-mech may update; both → firm. */
  if (!both) {
    if (cur != spat) {
      if (mid_ledge)
        mw_prop_mid_ledge_sync_home(obj, spat, 0);
      else
        mw_prop_home_store(obj, spat, 0);
    } else if (mid_ledge) {
      mw_prop_mid_ledge_sync_home(obj, cur, 0);
    }
    return spat;
  }
  /* First both-mech sighting: take nearer and firm (ends cam wrestle). */
  if (mid_ledge) {
    mw_prop_mid_ledge_sync_home(obj, spat, 1);
    return spat;
  }
  mw_prop_home_store(obj, spat, 1);
  return spat;
}

/* Dual drawer list index $96 → object at $136E,Y (word table). */
static uint16_t mw_dual_draw_list_obj(void) {
  const uint16_t yi = mw_wram16(0x0096u);
  if (yi >= 0x80u || (yi & 1u))
    return 0;
  const uint16_t obj = mw_wram16((uint16_t)(0x136Eu + yi));
  if (obj < 0x1934u || obj >= 0x2000u)
    return 0;
  return obj;
}

static void mw_latch_stage_prop_obj(uint16_t obj) {
  if (mw_obj_is_stage_prop(obj)) {
    if (!s_pending_stage_prop_local_only)
      s_prop_stat_latch++;
    s_pending_stage_prop_local_only = 1;
    s_draw_obj_latched = obj;
    /* New sprite pass — next commit replaces any prior tiles for this obj. */
    s_prop_purge_arm = obj;
    return;
  }
  /* Confirmed object that isn't a stage prop → new sprite, clear.
   * Garbage / mid-sprite X: keep pending so multi-tile metas stay tagged. */
  if (obj >= 0x1934u && obj < 0x2000u) {
    s_pending_stage_prop_local_only = 0;
    s_draw_obj_latched = obj;
    s_prop_purge_arm = 0;
  }
}

/* Dual $80882F: LDA $00,X before TAX←flags&6 — reinforce only, never clear.
 * $7E (meta tile count) is already stored by the prior LDA [$82]/STA $7E. */
static void mw_draw_prop_reinforce_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!MwIsDualViewport() || !mw_h2h_vert_widen_armed())
    return;
  const uint16_t obj = (uint16_t)cpu->X;
  if (mw_obj_is_stage_prop(obj)) {
    /* Do not re-arm purge mid-sprite — that would drop tiles already committed. */
    if (!s_pending_stage_prop_local_only || s_draw_obj_latched != obj)
      s_prop_purge_arm = obj;
    if (!s_pending_stage_prop_local_only)
      s_prop_stat_latch++;
    s_pending_stage_prop_local_only = 1;
    s_draw_obj_latched = obj;
  }
  if (s_pending_stage_prop_local_only) {
    const unsigned n = (unsigned)mw_wram16(0x007Eu);
    if (n > s_prop_stat_meta7e_max)
      s_prop_stat_meta7e_max = n;
  }
}

/* Read a 16-bit CPU vector from bank $00 (emulation/native vector table). */
static uint32_t mw_read_vector_pc24(uint16_t vec_addr) {
  uint8 lo = cpu_read8(&g_cpu, 0x00, vec_addr);
  uint8 hi = cpu_read8(&g_cpu, 0x00, (uint16)(vec_addr + 1));
  return ((uint32_t)hi << 8) | lo; /* bank $00 */
}

/* IRQ stub / generic vector path. */
static void mw_run_interrupt(uint16_t vec_addr, uint64_t deadline) {
  cpu_push_interrupt_frame(&g_cpu);
  interp_bridge_set_master_deadline(deadline);
  (void)interp_bridge_run_interrupt(&g_cpu, mw_read_vector_pc24(vec_addr));
  interp_bridge_set_master_deadline(0);
}

/*
 * Metal Warriors NMI entry is a trampoline at $00/$80:8182:
 *   PHK / PER $0002 / JML [$0000] / RTI
 *
 * [$0000] normally holds $80:818A. Cutscenes (LucasArts logo) retarget it
 * to $80:B7E0 around a WAI. Boot can also smash [$0000] into WRAM — heal
 * those to $818A before entering so JML doesn't run stack garbage.
 */
static void mw_run_nmi(uint64_t deadline) {
  const uint16 s_entry = g_cpu.S;
  const uint16 hook_pc = (uint16)(g_ram[0] | (g_ram[1] << 8));
  const uint8 hook_bank = g_ram[2];

  if (hook_pc < 0x8000u) {
    static unsigned reports;
    if (reports < 8) {
      reports++;
      fprintf(stderr,
              "[mw_rtl] NMI hook smashed $%02X:%04X — healing to $80:818A\n",
              hook_bank, hook_pc);
    }
    g_ram[0] = (uint8_t)(MW_FN_MwNmiBody & 0xFFu);
    g_ram[1] = (uint8_t)((MW_FN_MwNmiBody >> 8) & 0xFFu);
    g_ram[2] = (uint8_t)((MW_FN_MwNmiBody >> 16) & 0xFFu);
  } else {
    static uint32_t last = 0xFFFFFFFFu;
    static unsigned reports;
    const uint32_t hook = ((uint32_t)hook_bank << 16) | hook_pc;
    if (hook != last && reports < 24) {
      reports++;
      last = hook;
      fprintf(stderr, "[mw_rtl] NMI hook $%06X\n", (unsigned)hook);
    }
  }

  cpu_push_interrupt_frame(&g_cpu);
  interp_bridge_set_master_deadline(deadline);
  /* Real trampoline so PHK/PER/JML[$0000]/RTL/RTI match hardware. */
  const int ok = interp_bridge_run_interrupt(&g_cpu, MW_FN_I_NMI);
  interp_bridge_set_master_deadline(0);
  /* NMI just built this frame's scroll/strip DMA — snapshot the camera it
   * used so presentation keys don't drift ahead of the PPU state. */
  mw_latch_nmi_camera();
  if (!ok) {
    static unsigned reports;
    if (reports < 8) {
      reports++;
      fprintf(stderr,
              "[mw_rtl] NMI bail S=$%04X (restore $%04X) hook=$%02X:%04X\n",
              (unsigned)g_cpu.S, (unsigned)s_entry, g_ram[2],
              (unsigned)(g_ram[0] | (g_ram[1] << 8)));
    }
    g_cpu.S = s_entry;
    g_ram[0x13] = (uint8)(g_ram[0x13] & ~0x80u);
  }
}

static uint16_t mw_wram16(uint16_t addr) {
  return (uint16_t)(g_ram[addr] | ((uint16_t)g_ram[addr + 1] << 8));
}

static void mw_wram16_write(uint16_t addr, uint16_t v) {
  g_ram[addr] = (uint8_t)(v & 0xff);
  g_ram[addr + 1] = (uint8_t)(v >> 8);
}

bool MwIsDualViewport(void) {
  /* $82F6F7: LDA $1EB2 / BNE selects dual-cam spawn + P2 camera $1E1A/$1E1C. */
  return mw_wram16(0x1EB2) != 0;
}

/* Same expand gate as MwConfigureWidescreen (Mode 1/2/3 stage / H2H only). */
static bool mw_can_expand_gameplay(void) {
  if (!g_ppu)
    return false;
  const uint8_t mode = (uint8_t)(g_ppu->bgmode & 7);
  const uint8_t gm = g_ram[0x10];
  const bool mode_ok = (mode == 1 || mode == 2 || mode == 3);
  const bool menuish = (gm == 0x2a || gm == 0x48 || gm == 0x4e || gm == 0x54 ||
                        gm == 0x5a || gm == 0x00);
  if (!mode_ok || menuish)
    return false;
  /* Title / options / briefings often set H-mirror on a 32-col map without a
   * real offscreen world — do not expand from wide_map alone (that stretched
   * menus). Stage side-scroll is gm $18; H2H dual also needs margins. */
  if (gm == 0x18)
    return true;
  return MwIsDualViewport();
}

/* Extra big-tile columns for the current widescreen budget. */
static int mw_ws_tile_pad(void) {
  if (!g_ws_active || g_ws_extra <= 0)
    return 0;
  return (IntMin(g_ws_extra, kWsExtraMax) + 15) / 16;
}

/*
 * Relative viewport-window policy (default on): keep/kill BG2 strip history
 * and reject east wrap echoes using the same ±extra band as entity hooks.
 * SNESRECOMP_MW_VIEWPORT_REL=0 disables (legacy sticky west keep / no echo
 * filter — left chains linger, right door phantoms return).
 */
static int mw_viewport_rel_armed(void) {
  static int armed = -1;
  if (armed < 0) {
    const char *e = getenv("SNESRECOMP_MW_VIEWPORT_REL");
    if (e && e[0] == '0')
      armed = 0;
    else
      armed = 1;
  }
  return armed;
}

/* West history depth in tiles: widescreen margin + native 56px spawn slop. */
static int mw_ws_west_keep_tiles(void) {
  const int extra = IntMin(g_ws_extra, kWsExtraMax);
  if (extra <= 0)
    return 0;
  return (extra + 56 + 15) / 16 + 1;
}

static uint16_t mw_read7f16(uint16_t addr) {
  const uint32_t o = 0x10000u + (uint32_t)addr;
  if (o + 1u >= 0x20000u)
    return 0;
  return (uint16_t)(g_ram[o] | ((uint16_t)g_ram[o + 1u] << 8));
}

/*
 * Camera snapshot from the NMI that built the current PPU state. $1E16/$1E18
 * keep advancing in the main loop after NMI, so sampling them at present time
 * can be a few pixels ahead of the strip scroll/DMA — that mismatch shifted
 * margin tile keys and drew seam lines at the 4:3 boundary while moving.
 */
static uint16_t s_nmi_cam_x, s_nmi_cam_y, s_nmi_src_bg1, s_nmi_src_bg2;
static uint16_t s_nmi_cam2_x, s_nmi_cam2_y; /* BG2 camera $1E1A/$1E1C */
static uint16_t s_nmi_hscroll, s_nmi_vscroll;
static uint16_t s_nmi_hscroll1, s_nmi_vscroll1;
/* Full scroll mirrors (not PPU regs — those are often fine-phase 0..15 only). */
static uint16_t s_nmi_wram_h0, s_nmi_wram_v0, s_nmi_wram_h1, s_nmi_wram_v1;
static uint16_t s_nmi_wram_h0_p2, s_nmi_wram_v0_p2, s_nmi_wram_h1_p2,
    s_nmi_wram_v1_p2;
static bool s_nmi_latched;
/* Sticky stripe sources: $1E36/$1E38 are STZ'd at the start of the upload
 * routine, so NMI-time WRAM often reads 0 for BG2. Capture A-bus from the
 * DMA widen hook while the channel still holds the real pointer.
 * BG2 sticky only when DMA bank is $7F (map stream); ROM-bank strips clear
 * it. Frame stamps expire after one frame. */
static uint16_t s_sticky_src_bg1, s_sticky_src_bg2;
static int s_sticky_src_bg1_frame = -2;
static int s_sticky_src_bg2_frame = -2;
/* Per dual-cam BG1 DMA sticky — shared s_sticky_src_bg1 is whichever cam
 * last uploaded; slot sticky keeps each peer's last attributed strip. */
static uint16_t s_sticky_src_bg1_slot[2];
static int s_sticky_src_bg1_slot_frame[2] = {-2, -2};
/* Set while MwDrawPpuFrameLocalFull is presenting — idle BG2 must not use
 * 1P west-ROM / history extend (fills space with repeating tile trash). */
static int s_present_h2h_full_frame;
/* Presenting peer (0/1); -1 outside MwDrawPpuFrameLocalFull. */
static int s_present_h2h_local_slot = -1;
/*
 * Latched once per MwDrawPpuFrameLocalFull — BG1 rebuild/blank/paint and prop
 * OAM must share this cam. Re-reading $1E16/$1E1A mid-present (or mixing with
 * peer) shifts stripe+brown far left by ~d01 for one frame then snaps back.
 */
static uint16_t s_present_loc_x, s_present_loc_y;
static uint16_t s_present_oth_x, s_present_oth_y;
static int s_present_cam_valid;
/*
 * Per-cam snapshot of the BG1 $7F strip. Dual only DMA's one window per
 * frame; full-frame present rebuilds both peers from these caches so the
 * non-streamed cam does not repaint empty/void $7F into its viewport.
 */
/* Snap stores west+east around the stripe base. DMA pad_left is always 0
 * (VMADD col 0), so the left widescreen gutter must come from snap/$7F west
 * of src — not from the 17-word DMA window alone. */
enum { kMwBg1SnapWest = 8, kMwBg1SnapRows = 32, kMwBg1SnapCols = 40 };
typedef struct MwBg1Snap {
  uint16_t words[kMwBg1SnapRows][kMwBg1SnapCols];
  uint16_t src;
  uint16_t pitch;
  uint16_t cam_x;
  uint16_t cam_y;
  int frame;
  uint8_t valid;
} MwBg1Snap;
static MwBg1Snap s_bg1_snap[2];
/* Room uses decorative BG2 from ROM (bank $BB etc.), not $7F map stream.
 * Set by MwNotifyBg2MapDma; sticky/$1E38 blips from terrain dirty frames
 * must not flip present into a $7F BG2 rebuild. Cleared on map-base change
 * / session reset — not on transient bank-$7F strip DMAs. */
static int s_bg2_rom_idle;
/* Same origin passed to WsShadowSetWorld — margin capture must key to this. */
static uint32_t s_shadow_world_x, s_shadow_world_y;
static bool s_shadow_world_valid;

/*
 * Stripe A-bus base: $7E:42B3[row] + (camX & ~15)>>3
 * (see $809449..$809471 BG1 / $809512.. STA $1E38 BG2). Prefer this over
 * sticky/$1E36 — those are STZ'd during upload and go 0 between frames,
 * which blanked left BG1 and flickered the gutters.
 */
static uint16_t mw_map_src_from_42b3(uint16_t cam_x, uint16_t cam_y) {
  const uint16_t row_i = (uint16_t)((cam_y & (uint16_t)~15u) >> 3);
  const uint16_t col_i = (uint16_t)((cam_x & (uint16_t)~15u) >> 3);
  const uint16_t base = mw_wram16((uint16_t)(0x42B3u + row_i));
  if (!base)
    return 0;
  return (uint16_t)(base + col_i);
}

static uint16_t mw_bg1_pitch(void) {
  uint16_t pitch = mw_wram16(0x00B6);
  if (pitch < 32)
    pitch = 0x0290;
  return pitch;
}

/* True when two $7F stripe bases share a column (differ by whole pitch rows). */
static int mw_bg1_src_same_column(uint16_t a, uint16_t b, uint16_t pitch) {
  if (!a || !b || pitch < 32)
    return 0;
  const int d = (int)a - (int)b;
  return (d % (int)pitch) == 0;
}

/* Same column or within ±max_cols tile columns (careful snap X remap). */
static int mw_bg1_src_near_column(uint16_t a, uint16_t b, uint16_t pitch,
                                  int max_cols) {
  if (!a || !b || pitch < 32 || max_cols < 0)
    return 0;
  const int d = (int)a - (int)b;
  const int d_rows = d / (int)pitch;
  int rem = d - d_rows * (int)pitch;
  if (rem & 1)
    return 0;
  int d_cols = rem / 2;
  if (d_cols > max_cols || d_cols < -max_cols)
    return 0;
  return 1;
}

static int mw_bg1_tile_void(uint16_t t) {
  /* Undecoded gutter / missing map word — do not paint over live VRAM. */
  return t == 0 || t == 0x0DAEu;
}

/* Weak/empty strip content for snap quality (includes coldump sky $0200). */
static int mw_bg1_tile_weak(uint16_t t) {
  return mw_bg1_tile_void(t) || t == 0x0200u;
}

/* BG1 VRAM word at world pixel (present addressing). Full-frame: X is
 * view-relative to scroll; Y is world-absolute row. Do NOT use
 * (scroll>>4)+(sy>>4) — fine Y breaks tile-row identity (599 vs 552 →
 * row 4 instead of 5) and made feet samples lie. */
static uint16_t mw_bg1_vram_at_world(uint16_t world_x, uint16_t world_y,
                                     uint16_t scroll_x, uint16_t scroll_y) {
  if (!g_ppu)
    return 0;
  const unsigned sh = PPU_bigTiles(g_ppu, 0) ? 4u : 3u;
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, 0);
  const int map_row = (int)((world_y >> sh) & 31u);
  int map_col;
  (void)scroll_y; /* Y addressing is world-absolute; scroll unused. */
  if (s_present_h2h_full_frame) {
    map_col = (int)(world_x >> sh) - (int)(scroll_x >> sh);
    map_col &= 31;
  } else {
    map_col = (int)((world_x >> sh) & 31u);
  }
  return g_ppu->vram[(map_base + (map_row << 5) + (map_col & 31)) & 0x7fffu];
}

/* Snapshot one cam's BG1 strip out of $7F (survives dual VRAM stomps).
 * Columns are relative to src: [-kMwBg1SnapWest, +span).
 * All-weak captures keep the prior solid snap. West-only history merge —
 * native-col merge froze terrain / shattered floors (2026-08-04) and the
 * 2026-08-05 foreign-live native merge misbehaved other BG while panning. */
static void mw_bg1_snap_capture(int slot, uint16_t cam_x, uint16_t cam_y,
                                uint16_t src_opt) {
  uint16_t words[kMwBg1SnapRows][kMwBg1SnapCols];
  unsigned solid = 0;
  if (slot != 0 && slot != 1)
    return;
  const uint16_t pitch = mw_bg1_pitch();
  uint16_t src = src_opt;
  if (!src)
    src = mw_map_src_from_42b3(cam_x, cam_y);
  if (!src)
    return;
  const int col_lo = -kMwBg1SnapWest;
  const int col_hi = kMwBg1SnapCols - kMwBg1SnapWest;
  memset(words, 0, sizeof(words));
  for (int row = 0; row < kMwBg1SnapRows; row++) {
    for (int col = col_lo; col < col_hi; col++) {
      const int byte_off =
          (int)src + row * (int)pitch + col * 2;
      uint16_t t = 0;
      if (byte_off >= 0 && byte_off + 1 < 0x10000)
        t = mw_read7f16((uint16_t)byte_off);
      words[row][col + kMwBg1SnapWest] = t;
      if (!mw_bg1_tile_weak(t))
        solid++;
    }
  }
  /* Keep a prior solid snap when this capture is all weak — dual only
   * streamed the other cam into $7F this frame. */
  if (solid == 0 && s_bg1_snap[slot].valid)
    return;
  MwBg1Snap *snap = &s_bg1_snap[slot];
  /* Dual often stomps only the 17-col stripe; west of src goes void while
   * the strip stays solid. Merging keeps prior west history for the left
   * widescreen gutter (DMA pad_left is always 0).
   * Do NOT merge native-strip cells — that froze stale terrain. */
  if (snap->valid && snap->src &&
      mw_bg1_src_same_column(src, snap->src, pitch)) {
    for (int row = 0; row < kMwBg1SnapRows; row++) {
      for (int c = 0; c < kMwBg1SnapWest; c++) {
        if (mw_bg1_tile_weak(words[row][c]) &&
            !mw_bg1_tile_weak(snap->words[row][c]))
          words[row][c] = snap->words[row][c];
      }
    }
  }
  memcpy(snap->words, words, sizeof(words));
  snap->src = src;
  snap->pitch = pitch;
  snap->cam_x = cam_x;
  snap->cam_y = cam_y;
  {
    extern int snes_frame_counter;
    snap->frame = snes_frame_counter;
  }
  snap->valid = 1;
}

static void mw_bg1_snap_both_cams(void) {
  if (!MwIsDualViewport())
    return;
  const uint16_t c0x = s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16);
  const uint16_t c0y = s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18);
  const uint16_t c1x = s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1A);
  const uint16_t c1y = s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1C);
  mw_bg1_snap_capture(0, c0x, c0y, 0);
  mw_bg1_snap_capture(1, c1x, c1y, 0);
}

/* Attribute a BG1 DMA A-bus base to the nearer dual cam and refresh that
 * slot's sticky + $7F snapshot. */
static void mw_bg1_note_dma_src(uint16_t src) {
  if (!src)
    return;
  extern int snes_frame_counter;
  s_sticky_src_bg1 = src;
  s_sticky_src_bg1_frame = snes_frame_counter;

  int slot = 0;
  uint16_t cam_x = s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16);
  uint16_t cam_y = s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18);
  if (MwIsDualViewport()) {
    const uint16_t c0x = cam_x;
    const uint16_t c0y = cam_y;
    const uint16_t c1x = s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1A);
    const uint16_t c1y = s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1C);
    const uint16_t s0 = mw_map_src_from_42b3(c0x, c0y);
    const uint16_t s1 = mw_map_src_from_42b3(c1x, c1y);
    int d0 = s0 ? (int)src - (int)s0 : 0x7fffffff;
    int d1 = s1 ? (int)src - (int)s1 : 0x7fffffff;
    if (d0 < 0)
      d0 = -d0;
    if (d1 < 0)
      d1 = -d1;
    if (d1 < d0) {
      slot = 1;
      cam_x = c1x;
      cam_y = c1y;
    }
  }
  s_sticky_src_bg1_slot[slot] = src;
  s_sticky_src_bg1_slot_frame[slot] = snes_frame_counter;
  mw_bg1_snap_capture(slot, cam_x, cam_y, src);
}

/* `col` is relative to `src` (may be negative for west gutter). Maps through
 * snap->src allowing both row and column deltas — DMA sticky often sits a
 * few tile-columns east of the present $42B3 base.
 *
 * Native strip (col >= 0): allow at most ±1 tile-column remap (dual DMA
 * one-col lag / fine→coarse hitch). Uncapped X remap shattered floors —
 * do not widen past 1. West history (col < 0): full column remap for the
 * left widescreen gutter (VMADD col 0 ⇒ pad_left=0).
 */
static uint16_t mw_bg1_snap_word(int slot, uint16_t src, uint16_t pitch,
                                 int row, int col) {
  if (slot != 0 && slot != 1)
    return 0;
  const MwBg1Snap *snap = &s_bg1_snap[slot];
  if (!snap->valid || row < 0 || row >= kMwBg1SnapRows)
    return 0;
  int p = pitch ? (int)pitch : (int)snap->pitch;
  if (p < 32)
    p = 0x290;
  const int west = col < 0;
  if (!west && s_present_h2h_full_frame && src && snap->src &&
      !mw_bg1_src_near_column(src, snap->src, (uint16_t)p, 1))
    return 0;
  const int delta = (int)src - (int)snap->src;
  const int d_rows = delta / p;
  const int rem = delta - d_rows * p;
  if (rem & 1)
    return 0;
  const int d_cols = rem / 2;
  /* Native: ±1 col only. West: snap buffer bounds only. */
  if (!west && (d_cols > 1 || d_cols < -1))
    return 0;
  const int rr = row + d_rows;
  const int cc = col + d_cols + kMwBg1SnapWest;
  if (rr < 0 || rr >= kMwBg1SnapRows || cc < 0 || cc >= kMwBg1SnapCols)
    return 0;
  return snap->words[rr][cc];
}

/* Score a $7F stripe base against live BG1 VRAM column 0 (first 12 rows). */
static int mw_bg1_src_match(uint16_t src, uint16_t scroll_x, uint16_t scroll_y) {
  if (!g_ppu || !src)
    return -1;
  uint16_t pitch = mw_wram16(0x00B6);
  if (pitch < 32)
    pitch = 0x0290;
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, 0);
  const uint32_t buf_tx0 = (uint32_t)scroll_x >> 4;
  const uint32_t buf_ty0 = (uint32_t)scroll_y >> 4;
  int match = 0;
  for (int row = 0; row < 12; row++) {
    const uint32_t row_ptr = (uint32_t)src + (uint32_t)row * (uint32_t)pitch;
    const int map_row = (int)((buf_ty0 + (uint32_t)row) & 31u);
    const int map_col = (int)(buf_tx0 & 31u);
    const uint16_t vword =
        (uint16_t)(map_base + (map_row << 5) + map_col);
    if (g_ppu->vram[vword & 0x7fff] == mw_read7f16((uint16_t)row_ptr))
      match++;
  }
  return match;
}

/* Pick the $7F base that best matches the live strip (handles ±1 row drift). */
static uint16_t mw_best_bg1_src(uint16_t src_world, uint16_t src_sticky,
                                uint16_t src_fallback, uint16_t scroll_x,
                                uint16_t scroll_y) {
  uint16_t cands[5];
  int n = 0;
  if (src_world)
    cands[n++] = src_world;
  if (src_sticky)
    cands[n++] = src_sticky;
  if (src_world) {
    cands[n++] = (uint16_t)(src_world + 0x0290u);
    cands[n++] = (uint16_t)(src_world - 0x0290u);
  }
  if (src_fallback)
    cands[n++] = src_fallback;
  uint16_t best = src_world ? src_world : (src_sticky ? src_sticky : src_fallback);
  int best_m = -1;
  for (int i = 0; i < n; i++) {
    const uint16_t s = cands[i];
    if (!s)
      continue;
    int dup = 0;
    for (int j = 0; j < i; j++)
      if (cands[j] == s) {
        dup = 1;
        break;
      }
    if (dup)
      continue;
    const int m = mw_bg1_src_match(s, scroll_x, scroll_y);
    if (m > best_m) {
      best_m = m;
      best = s;
    }
  }
  return best;
}

static uint16_t mw_stage_src_bg1(void) {
  const uint16_t cam_x = s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16);
  const uint16_t cam_y = s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18);
  const uint16_t from_map = mw_map_src_from_42b3(cam_x, cam_y);
  if (from_map)
    return from_map;
  if (s_sticky_src_bg1)
    return s_sticky_src_bg1;
  return s_nmi_latched ? s_nmi_src_bg1 : mw_wram16(0x1E36);
}
/* True when BG2 DMA'd a bank-$7F map strip this/last frame (or NMI still
 * holds $1E38). Idle/space BG2 uses bank-$BB ROM strips at the same STA
 * $4305 PCs — those must not count as streaming. ROM-idle rooms stay idle
 * even when wall/floor damage briefly rebuilds $1E38 / $7F strips.
 * Full-frame H2H present always treats BG2 as idle (dual dirty frames
 * false-arm $1E38 / $7F and flash space garbage). */
static int mw_bg2_streaming_now(void) {
  if (s_present_h2h_full_frame || s_bg2_rom_idle)
    return 0;
  extern int snes_frame_counter;
  const uint16_t live = s_nmi_latched ? s_nmi_src_bg2 : mw_wram16(0x1E38);
  if (live)
    return 1;
  if (s_sticky_src_bg2 &&
      s_sticky_src_bg2_frame >= snes_frame_counter - 1)
    return 1;
  return 0;
}

static uint16_t mw_stage_src_bg2(void) {
  /* BG2 only streams in some rooms. When it does not ($1E38 never written,
   * no fresh sticky DMA src), return 0 so margin prefill skips BG2 —
   * deriving a src from $7E:42B3 anyway filled BG2 gutters with level-origin
   * panels that drew OVER BG1 (user-visible "BG1 missing" + wrong backdrop). */
  const uint16_t live = s_nmi_latched ? s_nmi_src_bg2 : mw_wram16(0x1E38);
  if (!mw_bg2_streaming_now())
    return 0;
  const uint16_t cam_x = s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1A);
  const uint16_t cam_y = s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1C);
  const uint16_t from_map = mw_map_src_from_42b3(cam_x, cam_y);
  if (from_map)
    return from_map;
  if (s_sticky_src_bg2)
    return s_sticky_src_bg2;
  return live;
}

/*
 * Right-margin OAM hints: a 9-bit OAM X in [256, 256+extra) is ambiguous —
 * either a sprite we intentionally placed in the right widescreen margin,
 * or a sprite the game parked off-screen-left at x−512 (invisible on real
 * hardware; MW does this constantly for culled/idle objects, which used to
 * ghost into the right margin as phantom door pieces). Collect a bit per
 * staging slot whenever a hook emits a genuine right-margin X and publish
 * the set to the PPU at NMI, when the staging buffer becomes live OAM.
 */
static uint8_t s_oam_right_hints[16];

static void mw_oam_hint_right(unsigned slot) {
  if (slot < 128u)
    s_oam_right_hints[slot >> 3] |= (uint8_t)(1u << (slot & 7u));
}

static void mw_latch_nmi_camera(void) {
  /* The NMI just DMA'd the staging buffer built by the previous logic
   * frame into OAM; the hints collected alongside it become live now. */
  if (g_ppu)
    PpuWsSetOamRightHints(g_ppu, s_oam_right_hints);
  memset(s_oam_right_hints, 0, sizeof(s_oam_right_hints));

  s_nmi_cam_x = mw_wram16(0x1E16);
  s_nmi_cam_y = mw_wram16(0x1E18);
  s_nmi_cam2_x = mw_wram16(0x1E1A);
  s_nmi_cam2_y = mw_wram16(0x1E1C);
  s_nmi_src_bg1 = mw_wram16(0x1E36);
  s_nmi_src_bg2 = mw_wram16(0x1E38);
  /* Latch BG scroll at the same instant: IRQ/HUD code can rewrite the
   * scroll registers between NMI and present, and a mismatched scroll key
   * shifts every captured viewport tile — a moving seam at the 4:3 edge. */
  if (g_ppu) {
    s_nmi_hscroll = (uint16_t)g_ppu->hScroll[0];
    s_nmi_vscroll = (uint16_t)g_ppu->vScroll[0];
    s_nmi_hscroll1 = (uint16_t)g_ppu->hScroll[1];
    s_nmi_vscroll1 = (uint16_t)g_ppu->vScroll[1];
  }
  /* Full dual scroll mirrors — PPU regs are often fine phase only (0..15)
   * while $1E2E/$1E5E (P1) and $1E32/$1E62 (P2) hold the real camera scroll. */
  s_nmi_wram_h0 = mw_wram16(0x1E2E);
  s_nmi_wram_v0 = mw_wram16(0x1E5E);
  s_nmi_wram_h1 = mw_wram16(0x1E42);
  s_nmi_wram_v1 = mw_wram16(0x1E60);
  s_nmi_wram_h0_p2 = mw_wram16(0x1E32);
  s_nmi_wram_v0_p2 = mw_wram16(0x1E62);
  s_nmi_wram_h1_p2 = mw_wram16(0x1E46);
  s_nmi_wram_v1_p2 = mw_wram16(0x1E64);
  s_nmi_latched = true;
  /* Dual: snapshot each cam's $7F strip so full-frame present can rebuild
   * from a cached window when the game only DMA'd the other cam this frame. */
  mw_bg1_snap_both_cams();
  {
    static unsigned logs;
    const char *e = getenv("SNESRECOMP_MW_SRC");
    if (e && e[0] == '1' && logs < 200) {
      logs++;
      extern int snes_frame_counter;
      fprintf(stderr,
              "[mw_latch] f=%d cam=%04X/%04X cam2=%04X/%04X src=%04X/%04X "
              "hs=%u/%u vs=%u/%u phase_dxy=%d/%d xsc=%u/%u big=%d/%d\n",
              snes_frame_counter, s_nmi_cam_x, s_nmi_cam_y, s_nmi_cam2_x,
              s_nmi_cam2_y, s_nmi_src_bg1, s_nmi_src_bg2,
              (unsigned)s_nmi_hscroll, (unsigned)s_nmi_hscroll1,
              (unsigned)s_nmi_vscroll, (unsigned)s_nmi_vscroll1,
              (int)(s_nmi_cam_x & 15) - (int)(s_nmi_hscroll & 15),
              (int)(s_nmi_cam_y & 15) - (int)(s_nmi_vscroll & 15),
              g_ppu ? (unsigned)(g_ppu->bgXsc[0] & 3) : 0u,
              g_ppu ? (unsigned)(g_ppu->bgXsc[1] & 3) : 0u,
              g_ppu ? PPU_bigTiles(g_ppu, 0) : 0,
              g_ppu ? PPU_bigTiles(g_ppu, 1) : 0);
    }
  }

  /* Occasional object + OAM margin dump. */
  if (getenv("SNESRECOMP_MW_COLS")) {
    static unsigned dumps;
    extern int snes_frame_counter;
    if (dumps < 8 && (snes_frame_counter % 60) == 0) {
      dumps++;
      const uint16_t cam = s_nmi_cam_x;
      const int extra = (int)mw_ws_extra_u();
      unsigned n_left = 0, n_mid = 0, n_right = 0, n_far = 0;
      uint16_t idx = mw_wram16(0x1E14);
      for (int guard = 0; guard < 64 && idx; guard++) {
        const uint16_t flags = mw_wram16(idx);
        if (flags & 0x8000u) {
          const int16_t sx =
              (int16_t)(mw_wram16((uint16_t)(idx + 2u)) - cam);
          if (sx < -extra)
            n_far++;
          else if (sx < 0)
            n_left++;
          else if (sx < 256)
            n_mid++;
          else if (sx < 256 + extra)
            n_right++;
          else
            n_far++;
        }
        const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
        if (next == idx)
          break;
        idx = next;
      }
      /* Staging OAM at $14C4 / high at $16C4 — count left-margin slots. */
      unsigned oam_left = 0, oam_right = 0, oam_on = 0;
      for (unsigned s = 0; s < 128u; s++) {
        const unsigned o = 0x14C4u + s * 4u;
        const uint8_t y = g_ram[o + 1u];
        if (y >= 0xE0u)
          continue; /* offscreen sentinel */
        oam_on++;
        const uint8_t xlo = g_ram[o];
        const unsigned hi = 0x16C4u + (s >> 2);
        const unsigned shift = (s & 3u) * 2u;
        const bool xhi = (g_ram[hi] & (uint8_t)(1u << shift)) != 0;
        const int sx = xhi ? (int)xlo - 256 : (int)xlo;
        if (sx < 0 && sx >= -extra)
          oam_left++;
        else if (sx >= 256 && sx < 256 + extra)
          oam_right++;
      }
      fprintf(stderr,
              "[mw_objs] f=%d cam=%04X objs far/left/mid/right %u/%u/%u/%u "
              "oam on/L/R %u/%u/%u extra=%d tile7f=%u helper=%u\n",
              snes_frame_counter, (unsigned)cam, n_far, n_left, n_mid, n_right,
              oam_on, oam_left, oam_right, extra, s_tile7f_hits,
              s_tile_helper_hits);
    }
  }

  /* Elevator / platform attribution — SNESRECOMP_MW_ELEV=1. */
  mw_elev_dump();
}

static uint16_t mw_stage_cam_x(void) {
  return s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16);
}
static uint16_t mw_stage_cam_y(void) {
  return s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18);
}
static uint16_t mw_stage_cam2_x(void) {
  /* $809520 builds BG2's strip from $1E1A; when dual-cam is idle those
   * words stay 0 — fall back to the main camera so shadow keys don't
   * collapse onto the fine-scroll phase alone (breaks both margins). */
  const uint16_t c =
      s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1A);
  return c ? c : mw_stage_cam_x();
}
static uint16_t mw_stage_cam2_y(void) {
  const uint16_t c =
      s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1C);
  return c ? c : mw_stage_cam_y();
}

/* Shadow world origin: coarse from the layer's WRAM camera, fine phase from
 * the latched PPU scroll that actually addresses the strip. Replacing cam's
 * fine bits with scroll keeps tile keys matched to the pixels the 4:3 view
 * shows when cam&15 != scroll&15 (common while the main loop advances cam
 * ahead of the NMI-latched strip). */
static uint32_t mw_shadow_world(uint16_t cam, uint16_t scroll) {
  /* Of the values congruent to scroll (mod 16), take the one nearest cam.
   * Always keeping cam's cell was off by 16 for a frame whenever cam had
   * crossed a 16px boundary the strip hadn't caught up with (worst when
   * scrolling left) — the margins jumped a tile against the view (tearing
   * at the 4:3 seams). */
  int32_t w = (int32_t)((cam & ~15u) | (scroll & 15u));
  const int32_t d = (int32_t)cam - w;
  if (d > 8)
    w += 16;
  else if (d < -8)
    w -= 16;
  return w < 0 ? 0u : (uint32_t)w;
}

/* Shared gate for entity/sprite widescreen hooks (gameplay modes only). */
static bool mw_ws_entity_hooks_armed(void) {
  if (!g_ppu || !g_ws_active || g_ws_extra <= 0)
    return false;
  const uint8_t mode = (uint8_t)(g_ppu->bgmode & 7);
  return mode == 1 || mode == 2 || mode == 3;
}

static unsigned mw_ws_extra_u(void) {
  return (unsigned)IntMin(g_ws_extra, kWsExtraMax);
}

/*
 * Meta-sprite OAM draw-window (bank $80, 12 size variants):
 *   STA $14C4,X            ; X low byte already stored
 *   CMP #$0100 / BCC nohi  ; 0..255: store without the OAM x high bit
 *   CMP #$FFF1 / BCC cull  ; -15..-1: fall through and OR the high bit
 * Do NOT widen the immediates — that sends [256,256+extra) down the no-high-bit
 * path and draws them at x-256 on the left. Remap margin X onto the high-bit
 * path by rewriting A; the stored low byte is already correct.
 *
 * Left band must match mw_active ([-64-extra, 320+extra)): the active list
 * already keeps anchors at sx≈−122, but the native #$FFF1 cull drops anything
 * < −15, so chain hooks on the left margin never reached OAM.
 */
static const uint32_t kMwSpriteWinCmpPc[] = {MW_SITE_LIST_SpriteWinCmp};

static void mw_oam_set_x_high(uint16_t oam_y);

static void mw_sprite_win_hook(CpuState *cpu, uint32_t pc24) {
  if (!mw_ws_entity_hooks_armed())
    return;
  const int extra = (int)mw_ws_extra_u();
  const uint16_t a = cpu->A;
  const int16_t sx = (int16_t)a;
  /* Right: only [256, 256+extra) — PPU wrap still forces x≥256+extra negative. */
  const bool right_margin = sx >= 256 && sx < 256 + extra;
  /* Left: anchor window is [-64-extra, 320); parts can sit ~48px left of
   * the anchor, and the visible margin is [-extra, 0). Keep a part slop
   * band so wide meta-sprites still emit OAM in the left margin. */
  const int left_min = -(64 + extra + 48);
  const bool left_margin = sx < 0 && sx >= left_min;
  if (!right_margin && !left_margin)
    return;
  cpu->A = 0xFFF8u;
  /* STA $14C4,X already wrote the X low byte. Force the staging high-OAM
   * X bit here too: the ROM's follow-up LDY $836C,X / ORA path can miss
   * for some size variants when A was rewritten, leaving left-margin
   * sprites at +low (visible as a wrong-side ghost) or X without bit 8. */
  mw_oam_set_x_high(cpu->X);
  if (right_margin)
    mw_oam_hint_right((unsigned)cpu->X / 4u);
  static unsigned logs;
  if (logs < 24 && getenv("SNESRECOMP_MW_COLS")) {
    logs++;
    fprintf(stderr, "[mw_sprite_win] pc=$%06X margin x=%d kept\n",
            (unsigned)pc24, (int)sx);
  }
}

/*
 * Particle / bullet OAM path ($80DBD3 / $80DC54):
 *   screenX = worldX − cam
 *   BMI skip / CMP #$0100 / BCS skip / STA $14C4,Y
 * Native window is hard [0,256) with no high-bit path. Widen by clearing N/C
 * on the branch sites, then set the $16C4 high-OAM X bit after STA so right
 * margin particles don't wrap to the left.
 *
 * Right keep must match the PPU non-wrap band [256, 256+extra) exactly —
 * past that the PPU does x-=512 and the shot reappears on the left edge.
 * (An older extra+16 BCS window caused that wrap ghost.)
 */
static const uint32_t kMwParticleBmiPc[] = {MW_SITE_LIST_ParticleBmi};
static const uint32_t kMwParticleBcsPc[] = {MW_SITE_LIST_ParticleBcs};
static const uint32_t kMwParticleAfterStaPc[] = {MW_SITE_LIST_ParticleAfterSta};

static void mw_oam_set_x_high(uint16_t oam_y) {
  /* Y is the byte offset into the $14C4 staging buffer (slot*4). High OAM
   * lives at $16C4 (see $80916A clear: DP walks $14C4 → $16C4). */
  if (oam_y & 3u)
    return;
  const unsigned slot = (unsigned)oam_y / 4u;
  if (slot >= 128u)
    return;
  const unsigned byte = 0x16C4u + (slot >> 2);
  const unsigned shift = (slot & 3u) * 2u;
  g_ram[byte] = (uint8_t)(g_ram[byte] | (uint8_t)(1u << shift));
}

static void mw_particle_bmi_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_ws_entity_hooks_armed())
    return;
  /* BMI skip if screenX < 0 — match meta-sprite left slop so bullets
   * "behind" the player stay as far as mech pieces. */
  const int16_t x = (int16_t)cpu->A;
  const int left_min = -(64 + (int)mw_ws_extra_u() + 48);
  if (x < 0 && x >= left_min)
    cpu->_flag_N = 0;
}

static void mw_particle_bcs_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_ws_entity_hooks_armed())
    return;
  /* After CMP #$0100: BCS skip if A ≥ 256 unsigned. Right-margin values
   * sit in [256, 256+extra) — same as mw_sprite_win / PPU wrap boundary.
   * Left-margin negatives are $FFxx and also look ≥ 256 unsigned — clear C
   * for both so STA runs with the real screen X. Past 256+extra, leave C set
   * so the ROM skips (no OAM → no left-edge wrap ghost). */
  const uint16_t a = cpu->A;
  const int16_t sx = (int16_t)a;
  const int extra = (int)mw_ws_extra_u();
  const int left_min = -(64 + extra + 48);
  const bool right = a >= 0x0100u && a < (uint16_t)(0x0100u + (unsigned)extra);
  const bool left = sx < 0 && sx >= left_min;
  if (right || left)
    cpu->_flag_C = 0;
}

static void mw_particle_after_sta_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_ws_entity_hooks_armed())
    return;
  /* A still holds screenX; Y is the OAM byte index. Only mark high-bit /
   * right-hint for the visible margin bands — never for x ≥ 256+extra. */
  const uint16_t a = cpu->A;
  const int16_t x = (int16_t)a;
  const int extra = (int)mw_ws_extra_u();
  const int left_min = -(64 + extra + 48);
  const bool right = a >= 0x0100u && a < (uint16_t)(0x0100u + (unsigned)extra);
  const bool left = x < 0 && x >= left_min;
  if (!right && !left)
    return;
  mw_oam_set_x_high(cpu->Y);
  if (right)
    mw_oam_hint_right((unsigned)cpu->Y / 4u);
  static unsigned logs;
  if (logs < 24 && getenv("SNESRECOMP_MW_COLS")) {
    logs++;
    fprintf(stderr, "[mw_particle] margin x=%d oamY=$%04X\n", (int)x,
            (unsigned)cpu->Y);
  }
}

/*
 * Active draw-list builder ($809219 / $809256): native screen window is
 * [-64, 320). Clamp extended-margin coordinates back into that window so
 * doors/bridges in [-extra, -64) and [320, 256+extra) stay on the list.
 */
/* X only — Y CMPs share #$FFC0/#$0140 immediates; widening them kept
 * off-screen vertical positions (e.g. sy≈−117) on the active list. */
/* CMP #$FFC0 / #$0140 after screen-X — see symbols.toml ActiveLo/HiCmp. */
static const uint32_t kMwActiveLoCmpPc[] = {MW_SITE_LIST_ActiveLoCmp};
static const uint32_t kMwActiveHiCmpPc[] = {MW_SITE_LIST_ActiveHiCmp};

static void mw_active_lo_hook(CpuState *cpu, uint32_t pc24) {
  if (!mw_ws_entity_hooks_armed())
    return;
  const int extra = (int)mw_ws_extra_u();
  const int16_t sx = (int16_t)cpu->A;
  if (sx < -64 && sx >= -64 - extra) {
    cpu->A = 0xFFC0u; /* unsigned ≥ #$FFC0 → pass low edge */
    static unsigned logs;
    if (logs < 12 && getenv("SNESRECOMP_MW_COLS")) {
      logs++;
      fprintf(stderr, "[mw_active] left sx=%d kept pc=$%06X X=%04X\n",
              (int)sx, (unsigned)pc24, (unsigned)cpu->X);
    }
  }
}

/* After screen-X → $86 (1P $808704/$808714; dual $8087DE/$808802).
 * X is the object — latch stage-prop ownership for all tiles of this sprite. */
static void mw_draw_sx_hook(CpuState *cpu, uint32_t pc24) {
  if (MwIsDualViewport() && mw_h2h_vert_widen_armed())
    mw_latch_stage_prop_obj((uint16_t)cpu->X);
  if (!mw_ws_entity_hooks_armed())
    return;
  const int16_t sx = (int16_t)cpu->A;
  const int extra = (int)mw_ws_extra_u();
  if (sx >= -64 - extra - 48 && sx < 256 + extra) {
    static unsigned logs;
    if (logs < 20 && getenv("SNESRECOMP_MW_COLS")) {
      const int16_t sy = (int16_t)mw_wram16(0x0088);
      logs++;
      fprintf(stderr,
              "[mw_draw] sx=%d sy=%d pc=$%06X obj=%04X flags=%04X prop=%d\n",
              (int)sx, (int)sy, (unsigned)pc24, (unsigned)cpu->X,
              (unsigned)mw_wram16((uint16_t)cpu->X),
              s_pending_stage_prop_local_only);
    }
  }
}

/* After meta resolve returns (1P $808721 post-$8759; dual $80881F post-$8857)
 * — $82/$84 are the meta-sprite pointer. Only reinforce stage-prop latch;
 * never clear it here (a mid-sprite non-prop X used to drop tags so only a
 * few tiles stayed local_only / world-presented). Cleared at next STA $86. */
static void mw_draw_meta_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  const uint16_t p_lo = mw_wram16(0x0082);
  const uint16_t p_hi = mw_wram16(0x0084);
  if (MwIsDualViewport() && mw_h2h_vert_widen_armed()) {
    const uint16_t obj = (uint16_t)cpu->X;
    if (mw_obj_is_stage_prop(obj)) {
      s_draw_obj_latched = obj;
      s_pending_stage_prop_local_only = 1;
    } else if (p_hi == 0x00B1u && mw_is_stage_prop_meta(p_lo)) {
      s_pending_stage_prop_local_only = 1;
      /* keep s_draw_obj_latched from STA $86 */
    }
    /* else: keep s_pending / latched object for the rest of this sprite */
  }
  if (!mw_ws_entity_hooks_armed())
    return;
  const int16_t sx = (int16_t)mw_wram16(0x0086);
  const int16_t sy = (int16_t)mw_wram16(0x0088);
  const int extra = (int)mw_ws_extra_u();
  if (sx < -64 - extra - 48 || sx >= 256 + extra)
    return;
  static unsigned logs;
  if (logs < 20 && getenv("SNESRECOMP_MW_COLS")) {
    logs++;
    fprintf(stderr,
            "[mw_draw] meta sx=%d sy=%d ptr=%04X/%04X %s prop=%d obj=%04X\n",
            (int)sx, (int)sy, (unsigned)p_lo, (unsigned)p_hi,
            (p_lo | p_hi) ? "ok" : "NULL-skip",
            s_pending_stage_prop_local_only, (unsigned)s_draw_obj_latched);
  }
}

static void mw_active_hi_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_ws_entity_hooks_armed())
    return;
  const int extra = (int)mw_ws_extra_u();
  const int16_t sx = (int16_t)cpu->A;
  if (sx >= 320 && sx < 320 + extra)
    cpu->A = 0x013Fu; /* keep below #$0140 */
}

/*
 * Map object spawn scan ($82F62E): window [cam−56, cam+312) via $34/$38,
 * BMI if worldX<$34, then CMP #$0170. $34 and $38 are independent left
 * bounds (primary vs dual cam) — tracking one shared value let dual-cam
 * $38 overwrite $34 and either spawn props past the true right margin or
 * reject valid primary-window objects.
 */
static const uint32_t kMwSpawnLeftStaPc[] = {MW_SITE_LIST_SpawnLeftSta};
static const uint32_t kMwSpawnHiCmpPc[] = {MW_SITE_LIST_SpawnHiCmp};
static const uint32_t kMwSpawnLeftBmiPc[] = {MW_SITE_LIST_SpawnLeftBmi};
static uint16_t s_spawn_left_34 = 0;
static uint16_t s_spawn_left_38 = 0;
static bool s_spawn_left_34_valid = false;
static bool s_spawn_left_38_valid = false;

static void mw_spawn_left_sta_hook(CpuState *cpu, uint32_t pc24) {
  if (!mw_ws_entity_hooks_armed())
    return;
  {
    static unsigned hits;
    hits++;
    if ((hits == 1 || hits == 64) && getenv("SNESRECOMP_MW_COLS"))
      fprintf(stderr, "[mw_spawn] left_sta hits=%u pc=$%06X A=%04X\n", hits,
              (unsigned)pc24, (unsigned)cpu->A);
  }
  /* Native $34 = cam−56: the 56px is object-width slop past the visible
   * edge, not part of the viewport. Keep that slop beyond the widescreen
   * margin: $34 = cam−56−extra. (Subtracting only extra−56 put the bound
   * at the margin edge itself, so wide objects straddling it were never
   * TTL-refreshed and despawned exactly at the 4:3 left line.) */
  const unsigned extra = mw_ws_extra_u();
  cpu->A = (uint16_t)(cpu->A - (uint16_t)extra);
  if (pc24 == 0x82F72Du) {
    s_spawn_left_38 = cpu->A;
    s_spawn_left_38_valid = true;
  } else {
    s_spawn_left_34 = cpu->A;
    s_spawn_left_34_valid = true;
  }
}

static void mw_spawn_left_bmi_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_ws_entity_hooks_armed())
    return;
  const unsigned extra = mw_ws_extra_u();
  const int16_t d = (int16_t)cpu->A; /* worldX − $34/$38 */
  if (d < 0 && d >= -(int)extra)
    cpu->_flag_N = 0;
}

static void mw_spawn_hi_hook(CpuState *cpu, uint32_t pc24) {
  if (!mw_ws_entity_hooks_armed())
    return;
  const bool dual = (pc24 == 0x82F67Bu);
  if (dual ? !s_spawn_left_38_valid : !s_spawn_left_34_valid)
    return;
  const unsigned extra = mw_ws_extra_u();
  const int16_t rel = (int16_t)cpu->A; /* worldX − left */
  if (rel < 0)
    return;
  const uint16_t left = dual ? s_spawn_left_38 : s_spawn_left_34;
  const uint16_t cam = dual ? mw_wram16(0x1E1A) : mw_wram16(0x1E16);
  const uint32_t world_x = (uint32_t)left + (uint32_t)(uint16_t)rel;
  /* Mirror the native window's 56px right slop past the extended edge. */
  const uint32_t max_world =
      (uint32_t)cam + 256u + (uint32_t)extra + 56u;
  /* Native BCC/BCS threshold is #$0170; only rewrite the extended band. */
  if ((uint16_t)rel >= 0x0170u && world_x < max_world)
    cpu->A = 0x016Fu;
}

/*
 * Bank-$84 object "on screen?" helper ($80A5AB), used by doors/bridges:
 *   screenX = worldX − cam
 *   if screenX < 0:  pass only if screenX + $1F0A >= 0   (≈ −size)
 *   else:            pass only if screenX < #$0120        (288)
 * That truncates both widescreen margins. Widen the BMI/CMP sites, and
 * also override the caller's ROL→$60 result so a failed Y pad or stale
 * $1F0A cannot hide a horizontally on-screen margin object.
 */
/*
 * Bank-$84 object "on screen?" helper ($80A5AB), used by doors/bridges:
 *   screenX = worldX − cam ($1E16 only)
 *   if screenX < 0:  pass only if screenX + $1F0A >= 0   (≈ −size)
 *   else:            pass only if screenX < #$0120        (288)
 * Widescreen: widen BMI/CMP margins. Dual full-frame H2H: also pass when
 * the object sits in P2's cam window — native helper is P1-cam only, so
 * movers near P2 froze / chased the wrong bounds (lost bearings on X).
 * Dual-cam accept must NOT require g_ws_extra (4:3 full-frame still needs it).
 */
static int mw_h2h_dual_bound_armed(void) {
  return MwIsDualViewport() &&
         (MwH2hFullFrameLocalArmed() || mw_h2h_vert_widen_armed());
}

static int mw_world_x_in_cam_band(uint16_t world_x, uint16_t cam, int extra) {
  const int sx = (int)world_x - (int)cam;
  return sx >= -extra && sx < 256 + extra;
}

static int mw_world_x_in_any_dual_cam(uint16_t world_x, int extra) {
  if (mw_world_x_in_cam_band(world_x, mw_wram16(0x1E16u), extra))
    return 1;
  if (MwIsDualViewport() &&
      mw_world_x_in_cam_band(world_x, mw_wram16(0x1E1Au), extra))
    return 1;
  return 0;
}

static int mw_obj_onscreen_extra(void) {
  int extra = 32;
  if (mw_ws_entity_hooks_armed())
    extra += (int)mw_ws_extra_u();
  return extra;
}

static void mw_obj_onscreen_entry_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_ws_entity_hooks_armed() && !mw_h2h_dual_bound_armed())
    return;
  /* TXA at $80A5AB — X holds world X. */
  static unsigned logs;
  if (logs < 16 && getenv("SNESRECOMP_MW_COLS")) {
    logs++;
    const int16_t sx =
        (int16_t)((uint16_t)cpu->X - mw_wram16(0x1E16));
    fprintf(stderr, "[mw_obj_onscr] enter worldX=%04X cam=%04X sx=%d\n",
            (unsigned)cpu->X, (unsigned)mw_wram16(0x1E16), (int)sx);
  }
}

static void mw_obj_onscreen_bmi_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_ws_entity_hooks_armed() && !mw_h2h_dual_bound_armed())
    return;
  /* After CLC / ADC $1F0A — A = screenX + pad; BMI fail if A < 0. */
  const int extra = mw_obj_onscreen_extra();
  const int16_t a = (int16_t)cpu->A;
  const int16_t pad = (int16_t)mw_wram16(0x1F0A);
  const int16_t sx = (int16_t)(a - pad);
  if (a < 0 && sx >= -extra) {
    cpu->_flag_N = 0;
    static unsigned logs;
    if (logs < 12 && getenv("SNESRECOMP_MW_COLS")) {
      logs++;
      fprintf(stderr, "[mw_obj_onscr] left sx=%d pad=%d kept\n", (int)sx,
              (int)pad);
    }
    return;
  }
  /* Dual: failed P1-left test, but object is in P2's horizontal band. */
  if (a < 0 && mw_h2h_dual_bound_armed()) {
    const uint16_t world =
        (uint16_t)(mw_wram16(0x1E16u) + (uint16_t)sx);
    if (mw_world_x_in_cam_band(world, mw_wram16(0x1E1Au), extra))
      cpu->_flag_N = 0;
  }
}

static void mw_obj_onscreen_cmp_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_ws_entity_hooks_armed() && !mw_h2h_dual_bound_armed())
    return;
  /* CMP #$0120 / BCS fail — keep [288, 256+extra), or either dual cam. */
  const int extra = mw_obj_onscreen_extra();
  const int16_t sx = (int16_t)cpu->A;
  if (sx >= 0x0120 && sx < 256 + extra) {
    static unsigned logs;
    if (logs < 12 && getenv("SNESRECOMP_MW_COLS")) {
      logs++;
      fprintf(stderr, "[mw_obj_onscr] right sx=%d kept\n", (int)sx);
    }
    cpu->A = 0x011Fu;
    return;
  }
  if (sx >= 0x0120 && mw_h2h_dual_bound_armed()) {
    const uint16_t world =
        (uint16_t)(mw_wram16(0x1E16u) + (uint16_t)sx);
    if (mw_world_x_in_any_dual_cam(world, extra))
      cpu->A = 0x011Fu;
  }
}

/* $809A31 — STA $7F0000,X used by doors/platforms (entry $809A18).
 * Dest is world-absolute via 42B3[(Y&~15)>>3]+((X&~15)>>3); X/Y are the
 * pixel args (hook sees computed $7F offset in cpu->X). */
static void mw_tile_patch_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_ws_entity_hooks_armed())
    return;
  s_tile7f_hits++;
  static unsigned logs;
  if (logs < 16 && getenv("SNESRECOMP_MW_COLS")) {
    logs++;
    fprintf(stderr, "[mw_tile7f] hits=%u A=%04X X=%04X cam=%04X\n",
            s_tile7f_hits, (unsigned)cpu->A, (unsigned)cpu->X,
            (unsigned)mw_wram16(0x1E16));
  }
}

static int mw_h2h_spawn_y_widen_armed(void);

/* Extra |obj−cam| radius for vertical lifetime (elevators / tall shafts).
 * Offline 1P default 192 — native radius + ws_extra alone is too short when
 * the camera pans the shaft. SNESRECOMP_MW_Y_SLOP=0 disables; =N overrides.
 * Does not touch OAM/CMP vert-widen (that culled $D5B8 elevators). */
static unsigned mw_ws_y_lifetime_slop(void) {
  static int cached = -2; /* -2 unset, -1 off, else pixels */
  if (cached == -2) {
    const char *e = getenv("SNESRECOMP_MW_Y_SLOP");
    if (e && e[0] == '0' && e[1] == '\0')
      cached = -1;
    else if (e && e[0] >= '1' && e[0] <= '9')
      cached = atoi(e);
    else
      cached = 192;
  }
  if (cached < 0)
    return 0;
  if (cached > 512)
    return 512u;
  return (unsigned)cached;
}

/*
 * $8283AC: after computing |obj−(cam+80)|, PLA restores the caller's radius
 * and STA $3C compares X/Y against it. Widen that radius by g_ws_extra so
 * every bank-$02 door/platform/prop update gate keeps margin objects alive
 * without per-call-site BCS patches.
 * (Do NOT touch $028883/$02888A player-proximity compares — those caused
 * phantom BG uploads when forced earlier.)
 */
static void mw_dist_limit_sta_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_ws_entity_hooks_armed())
    return;
  /* Widescreen X slop + vertical lifetime slop (1P shafts / opt-in H2H).
   * Not OAM/CMP vert-widen — that path culled $D5B8 elevators. */
  unsigned extra = mw_ws_extra_u();
  if (mw_h2h_spawn_y_widen_armed() && MwIsDualViewport())
    extra += 160u;
  else if (!MwIsDualViewport() && mw_can_expand_gameplay())
    extra += mw_ws_y_lifetime_slop();
  if (extra == 0)
    return;
  const uint32_t wide = (uint32_t)cpu->A + (uint32_t)extra;
  cpu->A = (wide > 0xFFFFu) ? 0xFFFFu : (uint16_t)wide;
  static unsigned logs;
  if (logs < 8 && getenv("SNESRECOMP_MW_COLS")) {
    logs++;
    fprintf(stderr, "[mw_dist] limit %u→%u (dx=%u dy=%u)\n",
            (unsigned)(cpu->A - (uint16_t)extra), (unsigned)cpu->A,
            (unsigned)cpu->X, (unsigned)cpu->Y);
  }
}

/* $809A61 — common bank-$02 door/platform tile helper (calls $809A18). */
static void mw_tile_helper_hook(CpuState *cpu, uint32_t pc24) {
  if (!mw_ws_entity_hooks_armed())
    return;
  s_tile_helper_hits++;
  static unsigned logs;
  if (logs < 16 && getenv("SNESRECOMP_MW_COLS")) {
    logs++;
    fprintf(stderr, "[mw_tile] pc=$%06X hits=%u A=%04X X=%04X Y=%04X cam=%04X\n",
            (unsigned)pc24, s_tile_helper_hits, (unsigned)cpu->A,
            (unsigned)cpu->X, (unsigned)cpu->Y, (unsigned)mw_wram16(0x1E16));
  }
}

/*
 * REMOVED (2026-07-18, playtest regression): forcing the bank-$84 update
 * gate ($8483DC BCS / $848408 CMP), the $8481D2 "$60" visibility result,
 * the $828883/$82888A DP compares, the $848BF5/$848C32 abs-distance
 * compares, and the $8283C6 distance-shrink ($828424). Those fabricated
 * "on screen" states for objects the engine had not set up, which
 * triggered graphics uploads/BG patches for unspawned objects (phantom
 * door column in the right margin, black tile corruption inside the live
 * door) without fixing left-margin visibility.
 */

/* Debug attribution (SNESRECOMP_MW_COLS=1): which engine bank drives the
 * live map objects. $80A5D7 is the per-object graphics uploader; its JSL
 * return address on the stack names the caller. */
static void mw_objgfx_caller_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!getenv("SNESRECOMP_MW_COLS"))
    return;
  static uint32_t seen[16];
  static unsigned nseen;
  const uint16_t s = cpu->S;
  const uint32_t ret = (uint32_t)g_ram[(uint16_t)(s + 1)] |
                       ((uint32_t)g_ram[(uint16_t)(s + 2)] << 8) |
                       ((uint32_t)g_ram[(uint16_t)(s + 3)] << 16);
  for (unsigned i = 0; i < nseen; i++)
    if (seen[i] == ret)
      return;
  if (nseen < 16) {
    seen[nseen++] = ret;
    fprintf(stderr, "[mw_objgfx] caller=$%06X\n", ret);
  }
}

static void mw_obj_gate_heartbeat_hook(CpuState *cpu, uint32_t pc24) {
  (void)cpu;
  (void)pc24;
  if (!getenv("SNESRECOMP_MW_COLS"))
    return;
  static unsigned hits;
  hits++;
  if (hits == 1 || hits == 64 || hits == 1024)
    fprintf(stderr, "[mw_obj_gate] $8483C6 dispatcher hits=%u\n", hits);
}

/*
 * Camera bbox $82837B: CPX $1E16 / BCC fail / CPX $1E26 / BCS fail.
 * Do NOT add extra into A at STA $1E26 — that also corrupts the following
 * AND #$000F → $1E1E scroll-phase mirror and jitters every actor.
 * Widen by patching $1E26 in WRAM after the store (at the AND), and by
 * forcing the left/right branch flags for the margin band.
 */
static void mw_bbox_right_and_hook(CpuState *cpu, uint32_t pc24) {
  (void)cpu;
  if (!mw_ws_entity_hooks_armed())
    return;
  /* AND #$000F at $8093D8 / $8094A3 — $1E26/$1E2A already hold cam+256. */
  const uint16_t addr = (pc24 == 0x8094A3u) ? 0x1E2Au : 0x1E26u;
  const uint16_t v = mw_wram16(addr);
  const uint16_t wide = (uint16_t)(v + (uint16_t)mw_ws_extra_u());
  if (wide != v)
    mw_wram16_write(addr, wide);
}

static void mw_bbox_left_bcc_hook(CpuState *cpu, uint32_t pc24) {
  if (!mw_ws_entity_hooks_armed())
    return;
  if (cpu->_flag_C)
    return; /* already ≥ cam */
  const unsigned extra = mw_ws_extra_u();
  const uint16_t cam =
      (pc24 == 0x828397u) ? mw_wram16(0x1E1A) : mw_wram16(0x1E16);
  const uint16_t x = cpu->X;
  if ((uint16_t)(cam - x) <= (uint16_t)extra && x < cam)
    cpu->_flag_C = 1;
}

static void mw_bbox_right_bcs_hook(CpuState *cpu, uint32_t pc24) {
  if (!mw_ws_entity_hooks_armed())
    return;
  if (!cpu->_flag_C)
    return; /* already < right bound */
  const unsigned extra = mw_ws_extra_u();
  const uint16_t cam =
      (pc24 == 0x82839Cu) ? mw_wram16(0x1E1A) : mw_wram16(0x1E16);
  const uint16_t x = cpu->X;
  const uint32_t right = (uint32_t)cam + 0x100u + extra;
  if ((uint32_t)x < right)
    cpu->_flag_C = 0;
}

/* $809B43: after ADC #$0040 / CMP $1E16 — A is coord+64; native rejects
 * coord < cam−64. Accept widescreen left band, and P2 cam under dual H2H. */
static void mw_bbox_padded_left_bcc_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_ws_entity_hooks_armed() && !mw_h2h_dual_bound_armed())
    return;
  if (cpu->_flag_C)
    return;
  const unsigned extra =
      mw_ws_entity_hooks_armed() ? mw_ws_extra_u() : 0u;
  const uint16_t orig = (uint16_t)(cpu->A - 0x0040u);
  const uint16_t cam0 = mw_wram16(0x1E16u);
  if ((uint16_t)(cam0 - orig) <= (uint16_t)extra && orig < cam0) {
    cpu->_flag_C = 1;
    return;
  }
  if (mw_h2h_dual_bound_armed()) {
    const uint16_t cam1 = mw_wram16(0x1E1Au);
    if ((uint16_t)(cam1 - orig) <= (uint16_t)extra && orig < cam1)
      cpu->_flag_C = 1;
  }
}

/*
 * Non-streaming BG2 (elevator room): the live strip is DMA'd from a wide
 * ROM tilemap (bank $BB, ~100-word row pitch). Each row is 22 words
 * (16 view + 6 east headroom) into VRAM $6C00+. West of that window exists
 * in ROM but is never DMA'd — capture the row A-bus bases here and prefill
 * the left gutter from ROM at present.
 */
enum { kMwBg2RomRows = 16, kMwBg2RomCols = 22 };
/* Native idle-BG2 DMA is 8 map rows (CPX #$10). Locate/restamp must not
 * invent a taller strip — that painted a cam-Y ghost band for both peers. */
enum { kMwBg2RomDmaRows = 8 };
/* Restamp only rows DMA'd (or locate-seeded) within this many frames. */
enum { kMwBg2RomRowMaxAge = 3 };
static uint8_t s_bg2_rom_bank[kMwBg2RomRows];
static uint16_t s_bg2_rom_addr[kMwBg2RomRows];
static uint16_t s_bg2_rom_valid_mask;
static uint16_t s_bg2_rom_map_base;
static bool s_bg2_rom_locate_failed;
/* Last ROM strip words (present restamp). Survives dual $7F VRAM stomps when
 * locate-from-VRAM would fail on garbage. */
static uint16_t s_bg2_rom_words[kMwBg2RomRows][kMwBg2RomCols];
static uint16_t s_bg2_rom_words_mask;
static int s_bg2_rom_row_frame[kMwBg2RomRows];
/* Which dual peer's BG2 scroll window the DMA row was aimed at (−1 = none /
 * locate-only). Full-frame restamp only uses rows owned by local_slot. */
static int8_t s_bg2_rom_row_owner[kMwBg2RomRows];
static uint16_t s_coldump_bg2_present_v1;
static int s_coldump_bg2_restamp;
static uint16_t s_coldump_bg2_own_mask;

static uint16_t mw_cart_read16(uint8_t bank, uint16_t addr);

static int mw_bg2_rom_row_fresh(unsigned row) {
  extern int snes_frame_counter;
  if (row >= (unsigned)kMwBg2RomRows)
    return 0;
  if (!(s_bg2_rom_valid_mask & (uint16_t)(1u << row)))
    return 0;
  const int stamped = s_bg2_rom_row_frame[row];
  if (stamped <= 0)
    return 0;
  const int age = snes_frame_counter - stamped;
  return age >= 0 && age <= kMwBg2RomRowMaxAge;
}

/* Map-row distance into a peer's idle-BG2 view [ty0, ty0+dma_rows). */
static int mw_bg2_row_dist_to_scroll(unsigned row, uint16_t scroll_y,
                                     unsigned sh) {
  const unsigned ty0 = (unsigned)scroll_y >> sh;
  int best = 64;
  for (int i = 0; i < kMwBg2RomDmaRows; i++) {
    const unsigned mr = (ty0 + (unsigned)i) & 31u;
    int d = (int)row - (int)mr;
    if (d < 0)
      d = -d;
    if (d > 16)
      d = 32 - d;
    if (d < best)
      best = d;
  }
  return best;
}

/*
 * Attribute a BG2 map-row DMA to the nearer dual cam (coldump / non-dual
 * restamp). Prefer each peer's BG2 WRAM scroll; if mirrors are 0, fall back
 * to main cams. Dual present does not restamp from these rows.
 */
static int mw_bg2_dma_owner_for_row(unsigned row) {
  if (!g_ppu || !MwIsDualViewport())
    return -1;
  const unsigned sh = PPU_bigTiles(g_ppu, 1) ? 4u : 3u;
  uint16_t v0 = s_nmi_latched ? s_nmi_wram_v1 : mw_wram16(0x1E60u);
  uint16_t v1 = s_nmi_latched ? s_nmi_wram_v1_p2 : mw_wram16(0x1E64u);
  const uint16_t c0y = s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18u);
  const uint16_t c1y = s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1Cu);
  if (v0 == 0)
    v0 = c0y;
  if (v1 == 0)
    v1 = c1y;
  const int d0 = mw_bg2_row_dist_to_scroll(row, v0, sh);
  const int d1 = mw_bg2_row_dist_to_scroll(row, v1, sh);
  /* Must land in/near that peer's native 8-row window. */
  if (d0 > kMwBg2RomDmaRows && d1 > kMwBg2RomDmaRows)
    return -1;
  if (d0 <= d1)
    return 0;
  return 1;
}

void MwNotifyBg2MapDma(uint8_t aBank, uint16_t aAdr, uint16_t vmadd,
                       uint16_t size) {
  if (!g_ppu || size < 0x22u)
    return;
  /* Word transfer into VRAM; reject tiny / non-strip uploads. */
  if (size > 0x40u)
    return;
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, 1);
  if (vmadd < map_base || vmadd >= (uint16_t)(map_base + 0x400u))
    return;
  if ((uint16_t)(vmadd - map_base) & 0x1Fu)
    return; /* must start at column 0 of a map row */
  const unsigned row = (unsigned)(vmadd - map_base) >> 5;
  if (row >= (unsigned)kMwBg2RomRows)
    return;
  if (s_bg2_rom_map_base != map_base) {
    s_bg2_rom_map_base = map_base;
    s_bg2_rom_valid_mask = 0;
    s_bg2_rom_words_mask = 0;
    s_bg2_rom_locate_failed = false;
    s_bg2_rom_idle = 0;
    memset(s_bg2_rom_row_frame, 0, sizeof(s_bg2_rom_row_frame));
    memset(s_bg2_rom_row_owner, -1, sizeof(s_bg2_rom_row_owner));
  }
  /* Only stamp ROM-idle from non-$7F strips. Terrain dirty frames may DMA
   * bank $7F into the same map — do not clear idle or overwrite ROM row
   * bases with $7F pointers. */
  if (aBank != 0x7Fu && aBank != 0) {
    extern int snes_frame_counter;
    s_bg2_rom_idle = 1;
    s_bg2_rom_bank[row] = aBank;
    s_bg2_rom_addr[row] = aAdr;
    s_bg2_rom_valid_mask |= (uint16_t)(1u << row);
    s_bg2_rom_row_frame[row] = snes_frame_counter;
    s_bg2_rom_row_owner[row] = (int8_t)mw_bg2_dma_owner_for_row(row);
    if (g_snes) {
      for (int col = 0; col < kMwBg2RomCols; col++)
        s_bg2_rom_words[row][col] =
            mw_cart_read16(aBank, (uint16_t)(aAdr + (uint32_t)col * 2u));
      s_bg2_rom_words_mask |= (uint16_t)(1u << row);
    }
  }
  if (getenv("SNESRECOMP_MW_BG2STAMP")) {
    static unsigned logs;
    if (logs < 24u) {
      logs++;
      fprintf(stderr,
              "[mw_bg2dma] row=%u a=%02X:%04X size=%04X vmadd=%04X own=%d\n",
              row, (unsigned)aBank, (unsigned)aAdr, (unsigned)size,
              (unsigned)vmadd, (int)s_bg2_rom_row_owner[row]);
    }
  }
}

static uint16_t mw_cart_read16(uint8_t bank, uint16_t addr) {
  const uint32_t lo = snes_read(g_snes, ((uint32_t)bank << 16) | addr);
  const uint32_t hi =
      snes_read(g_snes, ((uint32_t)bank << 16) | (uint16_t)(addr + 1u));
  return (uint16_t)(lo | (hi << 8));
}

enum { kMwBg2RomPitch = 0xC8u }; /* bytes between strip rows (100 words) */

/*
 * After loadstate the strip is already in VRAM but no DMA has fired yet.
 * Match VRAM row 0 against bank $BB (the observed strip source) and recover
 * per-row A-bus bases so west prefetch works before the first pan.
 */
static void mw_bg2_rom_locate_from_vram(void) {
  if (!g_ppu || !g_snes || s_bg2_rom_valid_mask || s_bg2_rom_locate_failed)
    return;
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, 1);
  uint16_t needle[8];
  int nonzero = 0;
  for (int c = 0; c < 8; c++) {
    needle[c] = g_ppu->vram[(uint16_t)(map_base + c) & 0x7fff];
    if (needle[c] != 0 && needle[c] != 0x0200u)
      nonzero++;
  }
  if (nonzero < 2) {
    s_bg2_rom_locate_failed = true;
    return;
  }

  /* Scan bank $BB once; elevator-room strips live there. */
  const uint8_t bank = 0xBBu;
  uint16_t hit = 0;
  for (uint32_t addr = 0x8000u; addr + 16u < 0x10000u; addr += 2u) {
    int ok = 1;
    for (int c = 0; c < 8; c++) {
      if (mw_cart_read16(bank, (uint16_t)(addr + (uint32_t)c * 2u)) !=
          needle[c]) {
        ok = 0;
        break;
      }
    }
    if (ok) {
      hit = (uint16_t)addr;
      break;
    }
  }
  if (!hit) {
    s_bg2_rom_locate_failed = true;
    return;
  }

  extern int snes_frame_counter;
  s_bg2_rom_map_base = map_base;
  s_bg2_rom_valid_mask = 0;
  s_bg2_rom_words_mask = 0;
  s_bg2_rom_idle = 1;
  memset(s_bg2_rom_row_frame, 0, sizeof(s_bg2_rom_row_frame));
  memset(s_bg2_rom_row_owner, -1, sizeof(s_bg2_rom_row_owner));
  /* Seed only the native 8-row DMA height. Pitch-extrapolating to 16 rows
   * left a second elevator band that both peers saw at a fixed world-Y.
   * Locate is not peer DMA ownership — restamp stays off until a real strip
   * upload attributes a row to local_slot. */
  for (unsigned row = 0; row < (unsigned)kMwBg2RomDmaRows; row++) {
    const uint32_t a = (uint32_t)hit + (uint32_t)row * kMwBg2RomPitch;
    if (a > 0xffffu)
      break;
    s_bg2_rom_bank[row] = bank;
    s_bg2_rom_addr[row] = (uint16_t)a;
    s_bg2_rom_valid_mask |= (uint16_t)(1u << row);
    s_bg2_rom_row_frame[row] = snes_frame_counter;
    s_bg2_rom_row_owner[row] = -1;
    for (int col = 0; col < kMwBg2RomCols; col++)
      s_bg2_rom_words[row][col] =
          mw_cart_read16(bank, (uint16_t)(a + (uint32_t)col * 2u));
    s_bg2_rom_words_mask |= (uint16_t)(1u << row);
  }
  if (getenv("SNESRECOMP_MW_BG2STAMP"))
    fprintf(stderr, "[mw_bg2rom] located strip at $%02X:%04X mask=%04X\n",
            (unsigned)bank, (unsigned)hit, (unsigned)s_bg2_rom_valid_mask);
}

/* Left gutter for retainHistory BG2: force from ROM west of the 22-col DMA
 * every present (same model as live VRAM / right headroom — not history).
 *
 * ROM bases are indexed by absolute VRAM map row (DMA vmadd). Live capture
 * keys viewport row 0 as VRAM row buf_ty0 — so west must use
 * rom_row = (buf_ty0 + vrow) & 31, not rom_row = vrow (that shifted chains
 * down whenever BG2 tile-scroll / look was non-zero).
 *
 * Viewport-relative ownership:
 * - Prefill when the strip A-bus base is stable (Force+static base drags in X).
 * - Force when buf_ty0 or ROM addr[0] changes so pan/look rewrites the margin
 *   instead of stacking Prefill ghosts beside history.
 * - Skip void Prefills/Forces so $0200 does not occupy cells or wipe
 *   live gap-fill / history that ROM does not describe. */
static uint32_t s_bg2_west_last_buf_ty0 = 0xffffffffu;
static uint16_t s_bg2_west_last_addr0 = 0xffffu;

static bool mw_bg2_rom_tile_opaque(uint16_t t) {
  return t != 0 && t != 0x0200u && t != 0x0DAEu;
}

static void mw_prefill_bg2_west_from_rom_at(uint16_t cam_x, uint16_t hs1,
                                            uint16_t vs1) {
  if (!g_ppu || !g_snes)
    return;
  if (!s_bg2_rom_valid_mask)
    mw_bg2_rom_locate_from_vram();
  if (!s_bg2_rom_valid_mask)
    return;
  /* Streaming BG2 uses $7F prefill; wide maps are native. Caller skips when
   * BG2 streams — do not gate on mw_stage_src_bg2() (dual cam2!=0 false +). */
  if ((g_ppu->bgXsc[1] & 1) != 0)
    return;

  const unsigned sh = PPU_bigTiles(g_ppu, 1) ? 4u : 3u;
  const int rows = sh == 4u ? 16 : 29;
  const uint32_t world_x = mw_shadow_world(cam_x, hs1);
  const uint32_t tx0 = world_x >> sh;
  const unsigned phase = (unsigned)world_x & ((1u << sh) - 1u);
  const int margin_tiles =
      (int)((phase + (unsigned)IntMin(g_ws_extra, kWsExtraMax) + (1u << sh) -
             1u) >>
            sh) +
      1;
  if (margin_tiles <= 0 || tx0 == 0)
    return;

  const int fill_rows = rows < kMwBg2RomRows ? rows : kMwBg2RomRows;
  const int view_cols = 256 >> (int)sh;
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, 1);
  const uint32_t buf_tx0 = (uint32_t)hs1 >> sh;
  const uint32_t buf_ty0 = (uint32_t)vs1 >> sh;
  const bool viewport_rel = mw_viewport_rel_armed() != 0;
  const uint16_t addr0 =
      (s_bg2_rom_valid_mask & 1u) ? s_bg2_rom_addr[0] : (uint16_t)0xffffu;
  const bool ty0_changed = (s_bg2_west_last_buf_ty0 != buf_ty0);
  const bool addr_changed = (s_bg2_west_last_addr0 != addr0);
  s_bg2_west_last_buf_ty0 = buf_ty0;
  s_bg2_west_last_addr0 = addr0;
  const bool force_west =
      !viewport_rel || ty0_changed || (viewport_rel && addr_changed);
  unsigned filled = 0;
  for (int d = 1; d <= margin_tiles; d++) {
    if (tx0 < (uint32_t)d)
      break;
    /* Skip west columns that are just a shifted copy of the live strip
     * (period/wrap) — same idea as east rejectEastEcho. */
    if (viewport_rel) {
      int echo = 0;
      for (int vc = 0; vc < view_cols && !echo; vc++) {
        int same = 1;
        int compared = 0;
        for (int vrow = 0; vrow < fill_rows; vrow++) {
          const unsigned rom_row = (buf_ty0 + (uint32_t)vrow) & 31u;
          if (rom_row >= (unsigned)kMwBg2RomRows ||
              !(s_bg2_rom_valid_mask & (uint16_t)(1u << rom_row)))
            continue;
          const uint16_t base = s_bg2_rom_addr[rom_row];
          const uint32_t byte_off = (uint32_t)d * 2u;
          if (base < byte_off) {
            same = 0;
            break;
          }
          const uint16_t wt = mw_cart_read16(s_bg2_rom_bank[rom_row],
                                             (uint16_t)(base - (uint16_t)byte_off));
          const int map_col = (int)((buf_tx0 + (uint32_t)vc) & 31u);
          const int map_row = (int)rom_row;
          const uint16_t vword =
              (uint16_t)(map_base + (map_row << 5) + map_col);
          compared++;
          if (wt != g_ppu->vram[vword & 0x7fff]) {
            same = 0;
            break;
          }
        }
        if (same && compared > 0)
          echo = 1;
      }
      if (echo)
        continue;
    }
    for (int vrow = 0; vrow < fill_rows; vrow++) {
      const unsigned rom_row = (buf_ty0 + (uint32_t)vrow) & 31u;
      if (rom_row >= (unsigned)kMwBg2RomRows ||
          !(s_bg2_rom_valid_mask & (uint16_t)(1u << rom_row)))
        continue;
      const uint8_t bank = s_bg2_rom_bank[rom_row];
      const uint16_t base = s_bg2_rom_addr[rom_row];
      const uint32_t byte_off = (uint32_t)d * 2u;
      if (base < byte_off)
        continue;
      const uint16_t t =
          mw_cart_read16(bank, (uint16_t)(base - (uint16_t)byte_off));
      if (viewport_rel && !mw_bg2_rom_tile_opaque(t))
        continue;
      if (force_west)
        WsShadowForceWestViewportTile(1, tx0 - (uint32_t)d, (uint32_t)vrow, t);
      else
        WsShadowPrefillWestViewportTile(1, tx0 - (uint32_t)d, (uint32_t)vrow,
                                        t);
      filled++;
    }
  }
  if (filled && getenv("SNESRECOMP_MW_BG2STAMP")) {
    static unsigned logs;
    if (logs < 12u) {
      logs++;
      fprintf(stderr,
              "[mw_bg2west] filled=%u margin=%d tx0=%u buf_ty0=%u "
              "addr0=%04X force=%d mask=%04X\n",
              filled, margin_tiles, (unsigned)tx0, (unsigned)buf_ty0,
              (unsigned)addr0, (int)force_west, (unsigned)s_bg2_rom_valid_mask);
    }
  }
}

/*
 * BG1: reject margin columns whose full-row signature matches anything in
 * the local strip neighborhood [−margin, view_span). DMA/$7F east of the
 * window often holds a shifted copy of west/strip columns (e.g. BA06/398C
 * at d=18 matching d=−2) — those became the orange-panel cascade.
 */
static bool mw_7f_cols_equal(uint16_t src0, uint16_t pitch, int widx_a,
                             int widx_b, int rows) {
  for (int row = 0; row < rows; row++) {
    const uint32_t base =
        (uint32_t)src0 + (uint32_t)row * (uint32_t)pitch;
    const uint16_t a =
        mw_read7f16((uint16_t)(base + (uint32_t)widx_a * 2u));
    const uint16_t b =
        mw_read7f16((uint16_t)(base + (uint32_t)widx_b * 2u));
    if (a != b)
      return false;
  }
  return true;
}

static bool mw_7f_col_is_void(uint16_t src0, uint16_t pitch, int widx,
                              int rows) {
  for (int row = 0; row < rows; row++) {
    const uint32_t off = (uint32_t)src0 + (uint32_t)row * (uint32_t)pitch +
                         (uint32_t)widx * 2u;
    if (off + 1u >= 0x10000u)
      return true;
    const uint16_t t = mw_read7f16((uint16_t)off);
    if (t != 0 && t != 0x0DAEu)
      return false;
  }
  return true;
}

static bool mw_7f_col_echoes_neighborhood(uint16_t src0, uint16_t pitch,
                                          int widx, int view_span,
                                          int margin_tiles, int rows) {
  if (rows <= 0 || view_span <= 0)
    return true;
  /* Void $7F columns match every other void — that must not block VRAM pad
   * capture (DMA widen already uploaded real tiles there). */
  if (mw_7f_col_is_void(src0, pitch, widx, rows))
    return false;
  const int lo = -margin_tiles;
  for (int vc = lo; vc < view_span; vc++) {
    if (vc == widx)
      continue;
    if (mw_7f_cols_equal(src0, pitch, widx, vc, rows))
      return true;
  }
  return false;
}

static void mw_prefill_layer_from_map(int layer, uint16_t src0, uint32_t cam_x,
                                      uint32_t cam_y, bool fill_left,
                                      bool fill_right, int right_skip) {
  if (!g_ppu || !g_ws_active || g_ws_extra <= 0 || src0 == 0)
    return;

  const unsigned sh = PPU_bigTiles(g_ppu, layer) ? 4u : 3u;
  const int view_cols = 256 >> (int)sh;
  const int rows = sh == 4u ? 16 : 29;
  const uint32_t tx0 = cam_x >> sh;
  const uint32_t ty0 = cam_y >> sh;
  const unsigned phase = (unsigned)cam_x & ((1u << sh) - 1u);
  const int view_span = view_cols + (phase ? 1 : 0);
  const int margin_tiles =
      (int)((phase + (unsigned)IntMin(g_ws_extra, kWsExtraMax) + (1u << sh) -
             1u) >>
            sh) +
      1;
  if (margin_tiles <= 0)
    return;
  if (right_skip < 0)
    right_skip = 0;

  uint16_t pitch = mw_wram16(0x00B6);
  if (pitch < (uint16_t)(view_cols * 2))
    pitch = 0x0290;

  for (int d = 1; d <= margin_tiles; d++) {
    if (fill_left) {
      const int widx = -d;
      if (tx0 >= (uint32_t)d) {
        for (int row = 0; row < rows; row++) {
          const uint32_t row_ptr =
              (uint32_t)src0 + (uint32_t)row * (uint32_t)pitch;
          const uint32_t left_bytes = (uint32_t)d * 2u;
          if (row_ptr < left_bytes)
            continue;
          uint16_t t =
              mw_read7f16((uint16_t)(row_ptr - left_bytes));
          /* Dual often stomps $7F; fall back to the per-cam snap west. */
          if ((t == 0 || t == 0x0DAEu) && layer == 0 &&
              (s_present_h2h_local_slot == 0 ||
               s_present_h2h_local_slot == 1)) {
            const uint16_t st =
                mw_bg1_snap_word(s_present_h2h_local_slot, src0, pitch, row,
                                 widx);
            if (st != 0 && st != 0x0DAEu)
              t = st;
          }
          /* $0DAE is MW's decoded-map void (fine-scroll overhang / empty).
           * Painting it into the gutter drew a false lattice over the left
           * margin; skip so the shadow miss (transparent) shows through. */
          if (t == 0 || t == 0x0DAEu)
            continue;
          WsShadowForceTile(layer, tx0 - (uint32_t)d, ty0 + (uint32_t)row, t);
        }
      }
    }
    if (fill_right && d > right_skip) {
      /* World-east columns in the decoded $7F map (src from $7E:42B3).
       * Do not neighborhood-filter — moving walls reuse strip tile IDs and
       * that despawned real collision geometry. Cascade came from BG2 edge
       * smear / VRAM DMA-pad, not from these map columns. */
      const int widx = view_span + d - 1;
      for (int row = 0; row < rows; row++) {
        const uint32_t right_off =
            (uint32_t)src0 + (uint32_t)row * (uint32_t)pitch +
            (uint32_t)widx * 2u;
        if (right_off + 1u >= 0x10000u)
          continue;
        const uint16_t t = mw_read7f16((uint16_t)right_off);
        if (t == 0 || t == 0x0DAEu)
          continue;
        WsShadowForceTile(layer, tx0 + (uint32_t)widx, ty0 + (uint32_t)row, t);
      }
    }
  }
}

/* DMA-pad VRAM → shadow (right). MW stripe VMADD col 0 clamps pad_left to 0
 * and widens DASIZ for pad_right only — those words land in VRAM past the
 * 17-col strip. Prefer non-void VRAM when $7F east is still $0DAE.
 * Never ForceTile void/0 over a cell: $7F prefill often already has the
 * real pipe/duct tile, and blanking it from a stale/void pad word was the
 * right-gutter black gap. */
static void mw_capture_vram_pad_cols(int layer, uint16_t src0, uint32_t world_x,
                                     uint32_t world_y, uint16_t scroll_x,
                                     uint16_t scroll_y) {
  if (!g_ppu || !g_ws_active || !s_shadow_world_valid)
    return;
  const int pad = mw_ws_tile_pad();
  if (pad <= 0)
    return;
  const unsigned sh = PPU_bigTiles(g_ppu, layer) ? 4u : 3u;
  const int view_cols = 256 >> (int)sh;
  const unsigned phase = (unsigned)world_x & ((1u << sh) - 1u);
  const int view_span = view_cols + (phase ? 1 : 0);
  const int rows = sh == 4u ? 16 : 29;
  const uint32_t tx0 = world_x >> sh;
  const uint32_t ty0 = world_y >> sh;
  const uint32_t buf_tx0 = (uint32_t)scroll_x >> sh;
  const uint32_t buf_ty0 = (uint32_t)scroll_y >> sh;
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, layer);
  const int x_mask = PPU_bgTilemapWider(g_ppu, layer) ? 63 : 31;
  uint16_t pitch = mw_wram16(0x00B6);
  if (pitch < (uint16_t)(view_cols * 2))
    pitch = 0x0290;
  const int margin_tiles =
      (int)((phase + (unsigned)IntMin(g_ws_extra, kWsExtraMax) + (1u << sh) -
             1u) >>
            sh) +
      1;

  unsigned filled = 0;
  for (int d = 0; d < pad; d++) {
    const int map_col =
        (int)((buf_tx0 + (uint32_t)(view_span + d)) & (uint32_t)x_mask);
    const int half = (x_mask > 31 && map_col >= 32) ? 0x400 : 0;
    const int widx = view_span + d;
    /* Reject shifted strip echoes in $7F; never reject void $7F (see above). */
    if (src0 &&
        mw_7f_col_echoes_neighborhood(src0, pitch, widx, view_span,
                                      margin_tiles, rows))
      continue;
    for (int row = 0; row < rows; row++) {
      const int map_row = (int)((buf_ty0 + (uint32_t)row) & 31u);
      const uint16_t word =
          (uint16_t)(map_base + half + (map_row << 5) + (map_col & 31));
      const uint16_t t = g_ppu->vram[word & 0x7fff];
      if (t == 0 || t == 0x0DAEu)
        continue;
      /* Local map says sky/void — do not promote dual-stale pad VRAM into
       * the widescreen shadow (P1 east-col ghosts while $7F is $0200). */
      if (src0) {
        const uint32_t off = (uint32_t)src0 + (uint32_t)row * (uint32_t)pitch +
                             (uint32_t)widx * 2u;
        if (off + 1u < 0x10000u) {
          const uint16_t t7 = mw_read7f16((uint16_t)off);
          if (t7 == 0x0200u || t7 == 0 || t7 == 0x0DAEu)
            continue;
        }
      }
      WsShadowForceTile(layer, tx0 + (uint32_t)(view_span + d),
                        ty0 + (uint32_t)row, t);
      filled++;
    }
  }
  if (filled && getenv("SNESRECOMP_MW_COLS")) {
    static unsigned logs;
    if (logs < 12u) {
      logs++;
      fprintf(stderr, "[mw_vpad] layer=%d pad=%d filled=%u span=%d src=%04X\n",
              layer, pad, filled, view_span, (unsigned)src0);
    }
  }
}

/* Rebuild paints west into wrapped VRAM (col < 0); Frame only captures the
 * native view. Copy those VRAM words into shadow — $7F may already be dual-
 * stomped by present time while rebuild still had snap west. */
static void mw_capture_vram_west_cols(int layer, uint32_t world_x,
                                      uint32_t world_y, uint16_t scroll_x,
                                      uint16_t scroll_y) {
  if (!g_ppu || !g_ws_active || !s_shadow_world_valid)
    return;
  const int pad = mw_ws_tile_pad();
  if (pad <= 0)
    return;
  const unsigned sh = PPU_bigTiles(g_ppu, layer) ? 4u : 3u;
  const int rows = sh == 4u ? 16 : 29;
  const uint32_t tx0 = world_x >> sh;
  const uint32_t ty0 = world_y >> sh;
  if (tx0 == 0)
    return;
  const uint32_t buf_tx0 = (uint32_t)scroll_x >> sh;
  const uint32_t buf_ty0 = (uint32_t)scroll_y >> sh;
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, layer);
  const int x_mask = PPU_bgTilemapWider(g_ppu, layer) ? 63 : 31;
  const int map_period = x_mask + 1;

  for (int d = 1; d <= pad; d++) {
    if (tx0 < (uint32_t)d)
      break;
    int map_col = ((int)buf_tx0 - d) % map_period;
    if (map_col < 0)
      map_col += map_period;
    const int half = (x_mask > 31 && map_col >= 32) ? 0x400 : 0;
    for (int row = 0; row < rows; row++) {
      const int map_row = (int)((buf_ty0 + (uint32_t)row) & 31u);
      const uint16_t word =
          (uint16_t)(map_base + half + (map_row << 5) + (map_col & 31));
      const uint16_t t = g_ppu->vram[word & 0x7fffu];
      if (t == 0 || t == 0x0DAEu || t == 0x0200u)
        continue;
      WsShadowForceTile(layer, tx0 - (uint32_t)d, ty0 + (uint32_t)row, t);
    }
  }
}

static void mw_log_7f_vram_align(void) {
  static unsigned logs;
  extern int snes_frame_counter;
  if (!getenv("SNESRECOMP_MW_ALIGN") || !g_ppu || logs >= 8)
    return;
  if ((snes_frame_counter % 60) != 0)
    return;
  logs++;
  const uint16_t src = mw_stage_src_bg1();
  uint16_t pitch = mw_wram16(0x00B6);
  if (pitch < 32)
    pitch = 0x0290;
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, 0);
  const uint32_t buf_tx0 =
      (uint32_t)(s_nmi_latched ? s_nmi_hscroll : g_ppu->hScroll[0]) >> 4;
  const uint32_t buf_ty0 =
      (uint32_t)(s_nmi_latched ? s_nmi_vscroll : g_ppu->vScroll[0]) >> 4;
  int match = 0, total = 0;
  /* Row 0: dump west + strip + DMA-pad east from $7F vs VRAM. */
  {
    const uint32_t row_ptr = (uint32_t)src;
    fprintf(stderr, "[mw_align] f=%d src=%04X pitch=%04X cam=%04X hs=%u 7fX:",
            snes_frame_counter, (unsigned)src, (unsigned)pitch,
            (unsigned)mw_stage_cam_x(),
            (unsigned)(s_nmi_latched ? s_nmi_hscroll : g_ppu->hScroll[0]));
    for (int d = -5; d <= 21; d++) {
      const int off = (int)row_ptr + d * 2;
      uint16_t t = 0;
      if (off >= 0 && off + 1 < 0x10000)
        t = mw_read7f16((uint16_t)off);
      fprintf(stderr, " %04X", (unsigned)t);
    }
    fprintf(stderr, "\n[mw_align] vramX:");
    for (int d = 0; d <= 21; d++) {
      const int map_col = (int)((buf_tx0 + (uint32_t)d) & 31u);
      const int map_row = (int)(buf_ty0 & 31u);
      const uint16_t vword =
          (uint16_t)(map_base + (map_row << 5) + map_col);
      fprintf(stderr, " %04X", (unsigned)g_ppu->vram[vword & 0x7fff]);
    }
    fprintf(stderr, "\n");
  }
  for (int row = 0; row < 12; row++) {
    const uint32_t row_ptr = (uint32_t)src + (uint32_t)row * (uint32_t)pitch;
    const int map_row = (int)((buf_ty0 + (uint32_t)row) & 31u);
    const int map_col = (int)(buf_tx0 & 31u);
    const uint16_t vword =
        (uint16_t)(map_base + (map_row << 5) + map_col);
    total++;
    if (g_ppu->vram[vword & 0x7fff] == mw_read7f16((uint16_t)row_ptr))
      match++;
  }
  fprintf(stderr,
          "[mw_align] f=%d match_col0=%d/%d cam16=%04X cam18=%04X "
          "map_src=%04X sticky=%04X base42=%04X\n",
          snes_frame_counter, match, total, (unsigned)mw_wram16(0x1E16),
          (unsigned)mw_wram16(0x1E18),
          (unsigned)mw_map_src_from_42b3(mw_wram16(0x1E16), mw_wram16(0x1E18)),
          (unsigned)s_sticky_src_bg1,
          (unsigned)mw_wram16(
              (uint16_t)(0x42B3u + ((mw_wram16(0x1E18) & ~15u) >> 3))));
  /* 2D dump: $7F rows 0..11, cols -8..23 rel src (pitch rows) + VRAM block. */
  for (int row = 0; row < 12; row++) {
    fprintf(stderr, "[mw_7fmap] r%02d:", row);
    for (int d = -8; d < 24; d++) {
      const int off = (int)src + row * (int)pitch + d * 2;
      uint16_t t = 0;
      if (off >= 0 && off + 1 < 0x10000)
        t = mw_read7f16((uint16_t)off);
      fprintf(stderr, " %04X", (unsigned)t);
    }
    fprintf(stderr, "\n");
  }
  for (int row = 0; row < 12; row++) {
    fprintf(stderr, "[mw_vram] r%02d:", row);
    for (int d = 0; d < 24; d++) {
      const int map_col = (int)((buf_tx0 + (uint32_t)d) & 31u);
      const int map_row = (int)((buf_ty0 + (uint32_t)row) & 31u);
      const uint16_t vword =
          (uint16_t)(map_base + (map_row << 5) + map_col);
      fprintf(stderr, " %04X", (unsigned)g_ppu->vram[vword & 0x7fff]);
    }
    fprintf(stderr, "\n");
  }
  /* BG2 map dump: all 32 columns x 12 rows (512px-wide big-tile map). The
   * view uses ~17 cols; check whether off-screen objects (doors/elevators)
   * are drawn into the wrapped half. */
  {
    const uint16_t base2 = (uint16_t)PPU_bgTilemapAdr(g_ppu, 1);
    const uint32_t b2_tx0 =
        (uint32_t)(s_nmi_latched ? s_nmi_hscroll1 : g_ppu->hScroll[1]) >> 4;
    const uint32_t b2_ty0 =
        (uint32_t)(s_nmi_latched ? s_nmi_vscroll1 : g_ppu->vScroll[1]) >> 4;
    fprintf(stderr, "[mw_bg2map] base=%04X tx0=%u ty0=%u rom_mask=%02X\n",
            (unsigned)base2, (unsigned)b2_tx0, (unsigned)b2_ty0,
            (unsigned)s_bg2_rom_valid_mask);
    for (int row = 0; row < 12; row++) {
      fprintf(stderr, "[mw_bg2map] r%02d:", row);
      for (int c = 0; c < 32; c++) {
        const int map_row = (int)((b2_ty0 + (uint32_t)row) & 31u);
        const uint16_t vword = (uint16_t)(base2 + (map_row << 5) + c);
        fprintf(stderr, " %04X", (unsigned)g_ppu->vram[vword & 0x7fff]);
      }
      fprintf(stderr, "\n");
    }
    if (s_bg2_rom_valid_mask & 1u) {
      fprintf(stderr, "[mw_bg2rom] r00 west/live:");
      const uint8_t bank = s_bg2_rom_bank[0];
      const uint16_t base = s_bg2_rom_addr[0];
      for (int d = -8; d < 22; d++) {
        uint16_t t = 0;
        if (d >= 0 || base >= (uint16_t)(-d * 2))
          t = mw_cart_read16(bank, (uint16_t)(base + (uint16_t)(d * 2)));
        fprintf(stderr, " %04X", (unsigned)t);
      }
      fprintf(stderr, "\n");
    }
  }
}

static void mw_log_seam_trace(void) {
  static int armed = -1;
  if (armed < 0)
    armed = getenv("SNESRECOMP_MW_SEAM") ? 1 : 0;
  if (!armed || !g_ppu)
    return;
  extern int snes_frame_counter;
  /* Whole-window checksums (22 cols x 16 rows) to see WHEN maps change. */
  const uint16_t base2 = (uint16_t)PPU_bgTilemapAdr(g_ppu, 1);
  uint32_t h = 2166136261u;
  for (int r = 0; r < 16; r++)
    for (int c = 0; c < 22; c++) {
      h ^= g_ppu->vram[(uint16_t)(base2 + (r << 5) + c) & 0x7fff];
      h *= 16777619u;
    }
  uint32_t h1 = 2166136261u;
  const uint16_t base1 = (uint16_t)PPU_bgTilemapAdr(g_ppu, 0);
  for (int r = 0; r < 16; r++)
    for (int c = 0; c < 22; c++) {
      h1 ^= g_ppu->vram[(uint16_t)(base1 + (r << 5) + c) & 0x7fff];
      h1 *= 16777619u;
    }
  fprintf(stderr,
          "[mw_seam] f=%d cam=%04X hs0=%u hs1=%u bg1map=%08X bg2map=%08X\n",
          snes_frame_counter, (unsigned)mw_stage_cam_x(),
          (unsigned)(s_nmi_latched ? s_nmi_hscroll : g_ppu->hScroll[0]),
          (unsigned)(s_nmi_latched ? s_nmi_hscroll1 : g_ppu->hScroll[1]),
          (unsigned)h1, (unsigned)h);
}

/*
 * Margin fill shared by 1P and H2H full-frame local present.
 * When cam_x_override != NULL, both layers key to that single camera (1P
 * recipe) — required for netplay local FOV so P2 is not filled from P1's
 * $7F/cam. When NULL, keep the classic split path (BG1=cam1, BG2=cam2).
 */
static void mw_prefill_margins_from_map_ex(const uint16_t *cam_x_override,
                                           const uint16_t *cam_y_override,
                                           uint16_t hs0, uint16_t vs0,
                                           uint16_t hs1, uint16_t vs1) {
  if (!g_ppu || !g_ws_active || g_ws_extra <= 0 || !mw_can_expand_gameplay())
    return;
  mw_log_7f_vram_align();
  mw_log_seam_trace();

  const bool single_cam = (cam_x_override != NULL && cam_y_override != NULL);
  const uint16_t cam1_x =
      single_cam ? *cam_x_override
                 : (s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16));
  const uint16_t cam1_y =
      single_cam ? *cam_y_override
                 : (s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18));
  const uint16_t cam2_x =
      single_cam ? *cam_x_override
                 : (s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1A));
  const uint16_t cam2_y =
      single_cam ? *cam_y_override
                 : (s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1C));

  /* H2H full-frame SetWorld uses raw cam (see MwDrawPpuFrameLocalFull).
   * ForceTile keys must match that origin — mw_shadow_world(cam, scroll)
   * drifts by up to a tile when the strip lags, so left-gutter ink lands
   * on keys the margin sampler never reads (empty history side). */
  const uint32_t world1_x =
      single_cam ? (uint32_t)cam1_x : mw_shadow_world(cam1_x, hs0);
  const uint32_t world1_y =
      single_cam ? (uint32_t)cam1_y : mw_shadow_world(cam1_y, vs0);
  const uint32_t world2_x =
      single_cam ? (uint32_t)cam2_x : mw_shadow_world(cam2_x, hs1);
  const uint32_t world2_y =
      single_cam ? (uint32_t)cam2_y : mw_shadow_world(cam2_y, vs1);

  const uint16_t src1_live =
      s_nmi_latched ? s_nmi_src_bg1 : mw_wram16(0x1E36);
  const uint16_t src2_live =
      s_nmi_latched ? s_nmi_src_bg2 : mw_wram16(0x1E38);
  const uint16_t src1w =
      mw_map_src_from_42b3((uint16_t)world1_x, (uint16_t)world1_y);
  /* BG2 $7F only when the game actually streams BG2 — do not treat dual
   * cam2!=0 as stream (that painted level-origin panels over BG1). */
  const bool bg2_stream = mw_bg2_streaming_now() != 0;
  const uint16_t src2w =
      bg2_stream ? mw_map_src_from_42b3((uint16_t)world2_x, (uint16_t)world2_y)
                 : 0;
  /* Full-frame local: never let the other cam's DMA sticky win over $42B3
   * for this camera — that painted P1 walls into P2's gutters. */
  uint16_t use_src1;
  if (single_cam) {
    const int slot = (s_present_h2h_local_slot == 0 ||
                      s_present_h2h_local_slot == 1)
                         ? s_present_h2h_local_slot
                         : 0;
    const uint16_t slot_sticky = s_sticky_src_bg1_slot[slot];
    use_src1 = src1w ? src1w
                     : (slot_sticky ? slot_sticky
                                    : mw_best_bg1_src(0, s_sticky_src_bg1,
                                                      src1_live, hs0, vs0));
  } else {
    use_src1 =
        mw_best_bg1_src(src1w, s_sticky_src_bg1, src1_live, hs0, vs0);
  }
  const uint16_t use_src2 =
      bg2_stream ? (src2w ? src2w : (s_sticky_src_bg2 ? s_sticky_src_bg2
                                                     : src2_live))
                 : 0;

  {
    static int left7f = -1, right7f = -1;
    if (left7f < 0) {
      const char *e = getenv("SNESRECOMP_MW_7F_LEFT");
      left7f = (e && e[0] == '0') ? 0 : 1;
    }
    if (right7f < 0) {
      const char *e = getenv("SNESRECOMP_MW_7F_RIGHT");
      right7f = (e && e[0] == '0') ? 0 : 1;
    }
    if (left7f) {
      mw_prefill_layer_from_map(0, use_src1, world1_x, world1_y, true, false,
                                0);
      if (use_src2)
        mw_prefill_layer_from_map(1, use_src2, world2_x, world2_y, true, false,
                                  0);
      /* Rebuild→VRAM west beats dual-stomped $7F at present time. */
      mw_capture_vram_west_cols(0, world1_x, world1_y, hs0, vs0);
    }
    if (right7f) {
      mw_prefill_layer_from_map(0, use_src1, world1_x, world1_y, false, true,
                                0);
      if (use_src2)
        mw_prefill_layer_from_map(1, use_src2, world2_x, world2_y, false, true,
                                  0);
    }
    if (right7f && use_src1)
      mw_capture_vram_pad_cols(0, use_src1, world1_x, world1_y, hs0, vs0);
  }

  /* Idle narrow BG2: ROM west of the 22-col DMA strip → left gutter.
   * Same 1P recipe under H2H full-frame (elevators); do not gate on a $BB
   * DMA stamp — dual H2H rarely fires that notify. */
  if (!bg2_stream && (g_ppu->bgXsc[1] & 1) == 0)
    mw_prefill_bg2_west_from_rom_at(cam1_x, hs1, vs1);

  if (WsShadowLayerActive(1)) {
    static int bg2_ext = -1; /* 0=off, 1=full, 2=seam */
    if (bg2_ext < 0) {
      const char *e = getenv("SNESRECOMP_MW_BG2_EXTEND");
      if (e && e[0] == '1')
        bg2_ext = 1;
      else if (e && (e[0] == 's' || e[0] == 'S'))
        bg2_ext = 2;
      else
        bg2_ext = 0;
    }
    if (bg2_ext == 1)
      WsShadowExtendSolidEdges(1, IntMin(g_ws_extra, kWsExtraMax));
    else if (bg2_ext == 2)
      WsShadowContinueSeam(1);
  }
}

static void mw_prefill_margins_from_map(void) {
  const uint16_t hs0 =
      s_nmi_latched ? s_nmi_hscroll : (uint16_t)g_ppu->hScroll[0];
  const uint16_t vs0 =
      s_nmi_latched ? s_nmi_vscroll : (uint16_t)g_ppu->vScroll[0];
  const uint16_t hs1 =
      s_nmi_latched ? s_nmi_hscroll1 : (uint16_t)g_ppu->hScroll[1];
  const uint16_t vs1 =
      s_nmi_latched ? s_nmi_vscroll1 : (uint16_t)g_ppu->vScroll[1];
  mw_prefill_margins_from_map_ex(NULL, NULL, hs0, vs0, hs1, vs1);
}

static int mw_dma_widen_armed(void) {
  static int v = -1;
  if (v < 0) {
    const char *e = getenv("SNESRECOMP_MW_DMA_WIDEN");
    /* Default on; SNESRECOMP_MW_DMA_WIDEN=0 disables (A/B). */
    if (e && e[0] == '0')
      v = 0;
    else if (e && e[0] == '1')
      v = 1;
    else
      v = kMwDmaWidenEnabled ? 1 : 0;
  }
  return v;
}

/*
 * Gate for DMA widen. These STA $4305 sites are stage stripe uploaders only.
 * During NMI, WRAM $10 is often still a cutscene/dialogue code ($4E) even when
 * the present frame is gameplay ($18), and cam≠hScroll until $80933A — so
 * mw_can_expand_gameplay is the wrong gate. Mode 1/2/3 + ws is enough.
 */
static bool mw_can_widen_dma(void) {
  if (!g_ppu || !g_ws_active || g_ws_extra <= 0)
    return false;
  const uint8_t mode = (uint8_t)(g_ppu->bgmode & 7);
  return mode == 1 || mode == 2 || mode == 3;
}

/*
 * Pre-opcode at STA $4305: A holds #$0022; $4302 and $2116 are already set.
 * Widen both margins: shift source/VMADD left by pad_left when col allows,
 * grow DASIZ for pad_left + 17 + pad_right words. MW's stripe uses col 0 so
 * pad_left becomes 0; left margin is filled at present via edge extend.
 */
static void mw_dma_size_hook(CpuState *cpu, uint32_t pc24) {
  /* Sticky A-bus base: only on the real 17-word strip STA $4305 (A==#$0022),
   * row 0 (X==0). Always capture even when widen is off — present-time $7F
   * Prefill and BG2 stream detection need it. Capturing before the size
   * check used to poison sticky (e.g. $DAE6 → all-zero $7F reads).
   * BG2: only bank $7F is map streaming. Bank $BB (etc.) is the idle/space
   * ROM strip — hit/damage frames re-DMA that and used to false-arm sticky,
   * flickering H2H full-frame backdrops into $7F garbage for 1–2 frames. */
  if (cpu->A == kMwDmaSizeNative && g_dma && cpu->X == 0) {
    const uint16_t src = g_dma->channel[0].aAdr;
    const uint8_t bank = g_dma->channel[0].aBank;
    if (src) {
      extern int snes_frame_counter;
      if (pc24 == kMwDmaSizePcBg1 || pc24 == kMwDmaSizePcAlt1) {
        mw_bg1_note_dma_src(src);
      } else if (pc24 == kMwDmaSizePcBg2 || pc24 == kMwDmaSizePcAlt2) {
        if (bank == 0x7Fu) {
          s_sticky_src_bg2 = src;
          s_sticky_src_bg2_frame = snes_frame_counter;
        } else {
          s_sticky_src_bg2 = 0;
          s_sticky_src_bg2_frame = -2;
        }
      }
    }
  }

  if (!mw_dma_widen_armed())
    return;
  if (!mw_can_widen_dma())
    return;
  if (cpu->A != kMwDmaSizeNative)
    return;

  const int pad = mw_ws_tile_pad();
  if (pad <= 0)
    return;

  const int native_words = (int)(kMwDmaSizeNative / 2u); /* 17 */

  int pad_left = pad;
  int pad_right = pad;

  const uint16_t aAdr0 = g_dma ? g_dma->channel[0].aAdr : 0;
  /* Do not read before A-bus address 0 (word-aligned tilemap words). */
  if (pad_left > (int)(aAdr0 / 2u))
    pad_left = (int)(aAdr0 / 2u);

  const uint16_t vmadd0 = g_ppu ? g_ppu->vramPointer : 0;
  if (g_ppu) {
    const int col = (int)(vmadd0 & 31u);
    if (pad_left > col)
      pad_left = col;
    const int new_col = col - pad_left;
    const int room_right = 32 - new_col - native_words;
    if (room_right < 0)
      return;
    if (pad_right > room_right)
      pad_right = room_right;
  }

  if (pad_left <= 0 && pad_right <= 0)
    return;

  if (pad_left > 0) {
    if (g_dma)
      g_dma->channel[0].aAdr = (uint16_t)(aAdr0 - (uint16_t)(pad_left * 2));
    if (g_ppu)
      g_ppu->vramPointer = (uint16_t)(vmadd0 - (uint16_t)pad_left);
  }

  /* bytes = 2*(17 + pad_left + pad_right) = 0x22 + 2*(pad_left+pad_right). */
  cpu->A = (uint16_t)(kMwDmaSizeNative +
                      (uint16_t)(2 * (pad_left + pad_right)));

  /* Latch for SNESRECOMP_MW_COLDUMP (last successful widen this frame). */
  {
    extern int snes_frame_counter;
    s_coldump_pad_l = pad_left;
    s_coldump_pad_r = pad_right;
    s_coldump_dasiz = cpu->A;
    s_coldump_aadr0 = aAdr0;
    s_coldump_aadr1 = g_dma ? g_dma->channel[0].aAdr : aAdr0;
    s_coldump_vm0 = vmadd0;
    s_coldump_vm1 = g_ppu ? g_ppu->vramPointer : vmadd0;
    s_coldump_dma_pc = pc24;
    s_coldump_dma_frame = snes_frame_counter;
    s_coldump_dma_dirty = 1;
  }

  static unsigned log_left = 8;
  if (log_left > 0) {
    log_left--;
    fprintf(stderr,
            "[mw_dma_widen] pc=$%06X size=$%04X→$%04X pad=%d/%d "
            "aAdr=$%04X→$%04X vmadd=$%04X→$%04X\n",
            (unsigned)pc24, (unsigned)kMwDmaSizeNative, (unsigned)cpu->A,
            pad_left, pad_right, (unsigned)aAdr0,
            g_dma ? (unsigned)g_dma->channel[0].aAdr : 0u, (unsigned)vmadd0,
            g_ppu ? (unsigned)g_ppu->vramPointer : 0u);
  }
}

/*
 * Pre-STA widen for $80943B..$809452:
 *   lo = cam & ~15; hi = lo + #$10   (pixels; one 16px column)
 * Harmless: these words are not a decode window (see kMwWidenAfterWrite).
 */
static int mw_stage_window_pad_px(void) {
  static int armed = -1;
  if (armed < 0) {
    const char *e = getenv("SNESRECOMP_MW_WIDEN_AW");
    if (e && e[0] == '0')
      armed = 0;
    else if (e && e[0] == '1')
      armed = 1;
    else
      armed = kMwWidenAfterWriteEnabled ? 1 : 0;
  }
  if (!armed || !g_ws_active || g_ws_extra <= 0 || !mw_can_expand_gameplay())
    return 0;
  return mw_ws_tile_pad() * 16;
}

static void mw_stage_window_lo_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  const int pad = mw_stage_window_pad_px();
  if (pad <= 0)
    return;
  if (cpu->A > (uint16_t)pad)
    cpu->A = (uint16_t)(cpu->A - (uint16_t)pad);
  else
    cpu->A = 0;
}

static void mw_stage_window_hi_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  const int pad = mw_stage_window_pad_px();
  if (pad <= 0)
    return;
  const uint32_t sum = (uint32_t)cpu->A + (uint32_t)pad;
  cpu->A = (sum > 0xffffu) ? 0xffffu : (uint16_t)sum;
  static unsigned logs;
  if (logs < 8u && getenv("SNESRECOMP_MW_COLS")) {
    logs++;
    fprintf(stderr, "[mw_widen_sta] pc=$%06X hi→%04X pad_px=%d\n",
            (unsigned)pc24, (unsigned)cpu->A, pad);
  }
}

static void mw_install_hooks(uint32_t const *pcs, size_t n,
                             InterpPreOpcodeHook hook) {
  for (size_t i = 0; i < n; i++)
    interp_bridge_set_pre_opcode_hook(pcs[i], hook);
}

/*
 * Phase 2b — H2H full-frame local present.
 *
 * Sim taller hooks (HDMA seam steal / stripe bump / Y-bias cancel) default
 * OFF — they break dual split and fight the shared VRAM strip. Opt in with
 * SNESRECOMP_MW_H2H_TALLER=1 (both peers).
 *
 * Full-frame local (default ON for netplay present): rebuild local FOV.
 * Opt out: SNESRECOMP_MW_H2H_FULL_FRAME=0 (legacy half-crop).
 *
 * Present OAM object-list redraw (solution 2) is opt-in only
 * (SNESRECOMP_MW_H2H_OBJ_OAM=1) — guest drawer during present caused black
 * screen.
 *
 * Dual draw writes each tile twice (P1 then P2) into 128 staging slots and
 * stops at CPX #$0200 — when cameras diverge the union active list overflows
 * and late meta-sprites arrive half-written ("sliced" mechs). With vert-widen
 * capture, wrap the staging index at full so ROM keeps emitting; per-cam
 * buffers already hold completed tiles. SNESRECOMP_MW_H2H_OAM_WRAP=0 disables.
 *
 * Vertical FOV OAM widen (netplay default ON): ROM-poke dual Y CMP (top
 * #$FF70, bottom #$E0) + active-list #$A8→#$E0. Staging uses +$78 for
 * negative sy; capture stores pre-bias screen Y so bottom sprites past +$78
 * still present. Opt out: SNESRECOMP_MW_H2H_VERT_WIDEN=0. Spawn/update Y is
 * separate (SNESRECOMP_MW_H2H_SPAWN_Y_WIDEN=1, default OFF).
 */
static int mw_h2h_taller_armed(void) {
  static int v = -1;
  if (v < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_TALLER");
    if (e && e[0] == '1')
      v = 1;
    else
      v = 0; /* default off — keep authentic dual strip/HDMA/OAM */
  }
  return v;
}

bool MwH2hFullFrameLocalArmed(void) {
  static int v = -1;
  if (v < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_FULL_FRAME");
    if (e && e[0] == '0')
      v = 0;
    else
      v = 1; /* default on — netplay H2H local FOV / top bar */
  }
  return v != 0;
}

/* Forward — coldump arming also gates offline LocalFull auto-enable. */
static int mw_coldump_armed(void);

int MwH2hPresentLocalSlot(void) {
  if (snes_netplay_active()) {
    const int s = snes_netplay_local_slot();
    if (s == 0 || s == 1)
      return s;
  }
  static int slot = -1;
  if (slot < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_OFFLINE_SLOT");
    slot = (e && e[0] == '1') ? 1 : 0;
  }
  return slot;
}

bool MwH2hShouldLocalFullPresent(void) {
  if (!MwIsDualViewport() || !MwH2hFullFrameLocalArmed())
    return false;
  if (snes_netplay_active())
    return true;
  /* Offline dual: LocalFull for column/stand diag. Auto-on with COLDUMP;
   * force with OFFLINE_LOCAL_FULL=1; keep split with =0. */
  static int offline = -1;
  if (offline < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_OFFLINE_LOCAL_FULL");
    if (e && e[0] == '0')
      offline = 0;
    else if (e && e[0] == '1')
      offline = 1;
    else
      offline = mw_coldump_armed() ? 1 : 0;
  }
  return offline != 0;
}

/* OAM +$78 bias / Y-CMP ROM patches for netplay full-frame present.
 * Default ON (elevators + full-frame Y). Offline hard-off. Opt out: =0. */
static int mw_h2h_vert_widen_armed(void) {
  if (!snes_netplay_active())
    return 0;
  static int v = -1;
  if (v < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_VERT_WIDEN");
    if (e && e[0] == '0')
      v = 0;
    else
      v = 1; /* default on — full-frame object Y + elevator draw */
  }
  return v != 0;
}

/* Dual spawn/update Y window + $8283AC radius slop. Does NOT touch OAM
 * bias / drawer CMP (those are VERT_WIDEN). Default OFF — playtest with
 * full-frame default ON was regressive (wrong object lifetime) and still
 * failed to spawn $D5B8 elevators. Opt in: SPAWN_Y_WIDEN=1. */
static int mw_h2h_spawn_y_widen_armed(void) {
  if (!snes_netplay_active())
    return 0;
  static int v = -1;
  if (v < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_SPAWN_Y_WIDEN");
    if (e && e[0] == '1')
      v = 1;
    else
      v = 0;
  }
  return v != 0;
}

/* Full-frame present BG1 $7F strip repaint. Default ON (needed for FOV
 * expand). Opt out: SNESRECOMP_MW_H2H_BG1_REBUILD=0 — playtest: skip was
 * purely regressive and did not move this stage's misaligned plates. */
static int mw_h2h_bg1_rebuild_armed(void) {
  static int v = -1;
  if (v < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_BG1_REBUILD");
    if (e && e[0] == '0')
      v = 0;
    else
      v = 1;
  }
  return v != 0;
}

/*
 * Elevator / platform probe (SNESRECOMP_MW_ELEV=1).
 *
 * Walks the live object list at $1E14 and dumps each record plus layer
 * signals (BG1 $7F tile patches, BG2 ROM-idle, OAM occupancy, dual cams).
 * Compare offline H2H (elevator present) vs netplay to see whether the
 * object is missing from the list or only failing present.
 *
 * Object layout (from existing COLS walker; Y/type not fully confirmed):
 *   +$00 flags (bit15 = active)  +$02 world X  +$14 next
 * Heuristic: +$04 treated as world Y for sx/sy columns (verify offline).
 */
/*
 * Compact JSONL column dump. Opt-in via SNESRECOMP_MW_COLDUMP.
 * One object per emit; fields stay stable for jq / peer A↔B diff.
 */
static FILE *s_coldump_fp;
static int s_coldump_armed = -1; /* -1 unknown, 0 off, 1 on */
static int s_coldump_last_f[2] = {-1, -1}; /* per-slot — offline dumps both */
static uint32_t s_coldump_last_hash[2];
static unsigned s_coldump_lines;

static int mw_coldump_armed(void) {
  if (s_coldump_armed >= 0)
    return s_coldump_armed;
  const char *e = getenv("SNESRECOMP_MW_COLDUMP");
  if (!e || !e[0] || e[0] == '0') {
    s_coldump_armed = 0;
    return 0;
  }
  const char *path = (e[0] == '1' && e[1] == '\0') ? "mw_coldump.jsonl" : e;
  s_coldump_fp = fopen(path, "ab");
  if (!s_coldump_fp) {
    fprintf(stderr, "[mw_coldump] failed to open '%s'\n", path);
    s_coldump_armed = 0;
    return 0;
  }
  setvbuf(s_coldump_fp, NULL, _IOLBF, 0);
  fprintf(stderr,
          "[mw_coldump] armed → %s (JSONL mover ID: why/d0/d1/tiles/"
          "bg_hit/dwx; strip.mism_local for local-cam columns). "
          "Offline dual: LocalFull auto-on (OFFLINE_LOCAL_FULL=0 keeps "
          "split; OFFLINE_SLOT=0|1 picks cam). "
          "Leave SNESRECOMP_MW_ELEV off for a quiet terminal.\n",
          path);
  s_coldump_armed = 1;
  return 1;
}

static uint32_t mw_coldump_fnv1a_u16(uint32_t h, uint16_t v) {
  h ^= (uint8_t)(v & 0xffu);
  h *= 16777619u;
  h ^= (uint8_t)(v >> 8);
  h *= 16777619u;
  return h;
}

/*
 * Home-reason probe for coldump (does not change latch — calls the real
 * home cam then classifies how that answer was produced).
 * why: "tag" | "near" | "hyst" | "one" | "none"
 */
static const char *mw_coldump_home_why(uint16_t obj, int home, int *d0_out,
                                       int *d1_out, int *mx0_out, int *my0_out,
                                       int *mx1_out, int *my1_out) {
  if (d0_out)
    *d0_out = -1;
  if (d1_out)
    *d1_out = -1;
  int mx0 = 0, my0 = 0, mx1 = 0, my1 = 0;
  const int have0 = mw_find_dual_mech_xy(0, &mx0, &my0);
  const int have1 = mw_find_dual_mech_xy(1, &mx1, &my1);
  if (mx0_out)
    *mx0_out = have0 ? mx0 : -1;
  if (my0_out)
    *my0_out = have0 ? my0 : -1;
  if (mx1_out)
    *mx1_out = have1 ? mx1 : -1;
  if (my1_out)
    *my1_out = have1 ? my1 : -1;
  /* Mover home ignores +$06 hi-byte (no why=tag). */
  if (home < 0 || (!have0 && !have1))
    return "none";
  if (!(have0 && have1))
    return "one";
  int owx = (int)mw_wram16((uint16_t)(obj + 2u));
  int owy = (int)mw_wram16((uint16_t)(obj + 4u));
  if (mw_prop_is_mid_hazard_ledge(obj))
    mw_prop_mid_ledge_centroid(&owx, &owy);
  const int32_t ax0 = (int32_t)owx - mx0;
  const int32_t ay0 = (int32_t)owy - my0;
  const int32_t ax1 = (int32_t)owx - mx1;
  const int32_t ay1 = (int32_t)owy - my1;
  const int32_t d0 = ax0 * ax0 + ay0 * ay0;
  const int32_t d1 = ax1 * ax1 + ay1 * ay1;
  if (d0_out)
    *d0_out = (int)(d0 > 0x7fffffff ? 0x7fffffff : d0);
  if (d1_out)
    *d1_out = (int)(d1 > 0x7fffffff ? 0x7fffffff : d1);
  const int spat = (d1 < d0) ? 1 : 0;
  if (mw_prop_home_is_firm(obj) && home != spat)
    return mw_prop_home_lock_forever(obj) ? "lock" : "firm";
  return (home == spat) ? "near" : "hyst";
}

static void mw_coldump_tick(int local_slot) {
  if (!mw_coldump_armed() || !s_coldump_fp)
    return;

  extern int snes_frame_counter;
  if (snes_frame_counter < 120)
    return;

  int slot = local_slot;
  if (slot != 0 && slot != 1)
    slot = MwH2hPresentLocalSlot();
  if (slot != 0 && slot != 1)
    slot = 0;
  if (snes_frame_counter == s_coldump_last_f[slot])
    return;

  const uint16_t cam0x = s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16);
  const uint16_t cam0y = s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18);
  const uint16_t cam1x = s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1A);
  const uint16_t cam1y = s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1C);
  const uint16_t src1 = mw_stage_src_bg1();
  uint16_t pitch = mw_wram16(0x00B6);
  if (pitch < 32)
    pitch = 0x0290;

  const uint16_t loc_x = (slot == 1) ? cam1x : cam0x;
  const uint16_t loc_y = (slot == 1) ? cam1y : cam0y;
  const uint16_t src_local = mw_map_src_from_42b3(loc_x, loc_y);

  enum { kStripN = 22 }; /* view 17 + small east pad sample */
  uint16_t row7f[kStripN];
  uint16_t row7f_loc[kStripN];
  uint16_t rowvram[kStripN];
  int mism = 0;      /* world src1 vs VRAM — often misleading under full-frame */
  int mism_local = 0; /* local-cam $7F vs VRAM — useful for column work */
  uint32_t hash = 2166136261u;
  memset(row7f, 0, sizeof(row7f));
  memset(row7f_loc, 0, sizeof(row7f_loc));
  memset(rowvram, 0, sizeof(rowvram));
  if (g_ppu) {
    const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, 0);
    const uint32_t buf_tx0 = (uint32_t)loc_x >> 4;
    const uint32_t buf_ty0 = (uint32_t)loc_y >> 4;
    const int map_row = (int)(buf_ty0 & 31u);
    /* Full-frame present: BG1 strip is view-relative at VRAM col 0. */
    const int view_rel = s_present_h2h_full_frame;
    for (int d = 0; d < kStripN; d++) {
      if (src1) {
        const int off = (int)src1 + d * 2;
        if (off >= 0 && off + 1 < 0x10000)
          row7f[d] = mw_read7f16((uint16_t)off);
      }
      if (src_local) {
        const int off = (int)src_local + d * 2;
        if (off >= 0 && off + 1 < 0x10000)
          row7f_loc[d] = mw_read7f16((uint16_t)off);
      }
      const int map_col =
          view_rel ? (d & 31) : (int)((buf_tx0 + (uint32_t)d) & 31u);
      const uint16_t vword =
          (uint16_t)(map_base + (map_row << 5) + map_col);
      rowvram[d] = g_ppu->vram[vword & 0x7fffu];
      if (row7f[d] != rowvram[d])
        mism++;
      if (row7f_loc[d] != rowvram[d])
        mism_local++;
      hash = mw_coldump_fnv1a_u16(hash, row7f_loc[d]);
      hash = mw_coldump_fnv1a_u16(hash, rowvram[d]);
    }
  }
  hash = mw_coldump_fnv1a_u16(hash, src1);
  hash = mw_coldump_fnv1a_u16(hash, src_local);
  hash = mw_coldump_fnv1a_u16(hash, (uint16_t)s_coldump_pad_l);
  hash = mw_coldump_fnv1a_u16(hash, (uint16_t)s_coldump_pad_r);
  hash = mw_coldump_fnv1a_u16(hash, s_coldump_dasiz);
  if (s_coldump_pscroll_frame == snes_frame_counter) {
    hash = mw_coldump_fnv1a_u16(hash, s_coldump_ph0);
    hash = mw_coldump_fnv1a_u16(hash, s_coldump_ph1);
    hash = mw_coldump_fnv1a_u16(hash, s_coldump_ploc_x);
  }
  {
    unsigned sticky_bits = 0;
    for (int i = 0; i < kMwPropHomeMax; i++) {
      if (s_prop_sticky_valid[i])
        sticky_bits += (unsigned)s_prop_sticky_n[i] + 1u;
    }
    hash = mw_coldump_fnv1a_u16(hash, (uint16_t)sticky_bits);
    hash = mw_coldump_fnv1a_u16(hash, (uint16_t)s_coldump_prop_n);
  }
  hash = mw_coldump_fnv1a_u16(hash, s_bg2_rom_valid_mask);
  hash = mw_coldump_fnv1a_u16(hash, s_bg2_rom_words_mask);
  hash = mw_coldump_fnv1a_u16(hash, (uint16_t)s_bg2_rom_idle);
  {
    uint16_t fresh = 0;
    for (unsigned r = 0; r < (unsigned)kMwBg2RomRows; r++) {
      if (mw_bg2_rom_row_fresh(r))
        fresh |= (uint16_t)(1u << r);
    }
    hash = mw_coldump_fnv1a_u16(hash, fresh);
  }

  /* SNESRECOMP_MW_COLDUMP_EVERY=1 — keep every frame (catch 1-frame X pops). */
  static int s_coldump_every = -1;
  if (s_coldump_every < 0)
    s_coldump_every = getenv("SNESRECOMP_MW_COLDUMP_EVERY") ? 1 : 0;
  const int period = MwIsDualViewport() ? 15 : 30;
  const int hash_changed = (hash != s_coldump_last_hash[slot]);
  const int dma_hit = s_coldump_dma_dirty &&
                      s_coldump_dma_frame == snes_frame_counter;
  if (!s_coldump_every && !dma_hit && !hash_changed &&
      (snes_frame_counter % period) != 0)
    return;

  s_coldump_last_f[slot] = snes_frame_counter;
  s_coldump_last_hash[slot] = hash;
  s_coldump_dma_dirty = 0;
  s_coldump_lines++;

  int mx0 = -1, my0 = -1, mx1 = -1, my1 = -1;
  {
    int t0 = 0, u0 = 0, t1 = 0, u1 = 0;
    if (mw_find_dual_mech_xy(0, &t0, &u0)) {
      mx0 = t0;
      my0 = u0;
    }
    if (mw_find_dual_mech_xy(1, &t1, &u1)) {
      mx1 = t1;
      my1 = u1;
    }
  }

  /* Scroll-authority probe: present loc vs per-slot BG1 snap cam (whip lag).
   * Note: $1E32 is NOT P2 world cam (coldump d1h was always huge) — use snap.
   * hs0/hs1 are NMI/PPU samples (often fine-phase 0..15 for BG2) — not the
   * scrolls applied during full-frame present. Those are pscroll.h0/h1. */
  const unsigned snap_cx =
      (slot == 0 || slot == 1) && s_bg1_snap[slot].valid
          ? (unsigned)s_bg1_snap[slot].cam_x
          : 0u;
  const unsigned hs0 =
      (unsigned)(s_nmi_latched ? s_nmi_hscroll
                               : (g_ppu ? (uint16_t)g_ppu->hScroll[0] : 0));
  const unsigned hs1 =
      (unsigned)(s_nmi_latched ? s_nmi_hscroll1
                               : (g_ppu ? (uint16_t)g_ppu->hScroll[1] : 0));
  const int pscroll_ok = (s_coldump_pscroll_frame == snes_frame_counter);
  /* Prefer latched present scrolls; fall back to live PPU if mid-present. */
  const unsigned ph0 =
      pscroll_ok ? (unsigned)s_coldump_ph0
                 : (g_ppu ? (unsigned)g_ppu->hScroll[0] : 0u);
  const unsigned pv0 =
      pscroll_ok ? (unsigned)s_coldump_pv0
                 : (g_ppu ? (unsigned)g_ppu->vScroll[0] : 0u);
  const unsigned ph1 =
      pscroll_ok ? (unsigned)s_coldump_ph1
                 : (g_ppu ? (unsigned)g_ppu->hScroll[1] : 0u);
  const unsigned pv1 =
      pscroll_ok ? (unsigned)s_coldump_pv1
                 : (g_ppu ? (unsigned)g_ppu->vScroll[1] : 0u);
  const unsigned plx =
      pscroll_ok ? (unsigned)s_coldump_ploc_x : (unsigned)loc_x;
  const unsigned ply =
      pscroll_ok ? (unsigned)s_coldump_ploc_y : (unsigned)loc_y;
  fprintf(s_coldump_fp,
          "{\"f\":%d,\"slot\":%d,\"net\":%d,\"dual\":%d,\"master\":%llu,"
          "\"host\":%u,\"gm\":%u,\"cam0\":[%u,%u],\"cam1\":[%u,%u],"
          "\"loc\":[%u,%u],\"src1\":%u,\"src_loc\":%u,\"pitch\":%u,"
          "\"scroll\":{\"loc\":%u,\"snap\":%u,\"hs0\":%u,\"hs1\":%u,"
          "\"d01\":%d,\"dsnap\":%d},"
          "\"pscroll\":{\"ok\":%d,\"h0\":%u,\"v0\":%u,\"h1\":%u,\"v1\":%u,"
          "\"h0_ppu\":%u,\"ploc\":[%u,%u],\"d_h0loc\":%d,\"d_h0hs0\":%d,"
          "\"d_h0ppu\":%d},"
          "\"bg1src\":%d,\"bg_dy\":%d,\"bg_slot\":%d,"
          "\"mechs\":[[%d,%d],[%d,%d]],"
          "\"dma\":{\"pc\":%u,\"pad\":[%d,%d],\"dasiz\":%u,"
          "\"aadr\":[%u,%u],\"vm\":[%u,%u],\"f\":%d},"
          "\"strip\":{\"n\":%d,\"mism\":%d,\"mism_local\":%d,\"hash\":%u,"
          "\"7f\":[",
          snes_frame_counter, slot, snes_netplay_active() ? 1 : 0,
          MwIsDualViewport() ? 1 : 0, (unsigned long long)g_cpu.master_cycles,
          s_lle_host_frames, (unsigned)g_ram[0x10], (unsigned)cam0x,
          (unsigned)cam0y, (unsigned)cam1x, (unsigned)cam1y, (unsigned)loc_x,
          (unsigned)loc_y, (unsigned)src1, (unsigned)src_local,
          (unsigned)pitch, (unsigned)loc_x, snap_cx, hs0, hs1,
          (int)cam1x - (int)cam0x, (int)loc_x - (int)snap_cx, pscroll_ok ? 1 : 0,
          ph0, pv0, ph1, pv1, (unsigned)(ph0 & 15u), plx, ply,
          (int)ph0 - (int)plx, (int)ph0 - (int)hs0,
          (int)(ph0 & 15u) -
              (int)(g_ppu ? (uint16_t)g_ppu->hScroll[0] : 0),
          s_elev_bg1_src_path, s_elev_prop_bg_dy,
          s_coldump_bg_slot, mx0, my0, mx1, my1, (unsigned)s_coldump_dma_pc,
          s_coldump_pad_l, s_coldump_pad_r, (unsigned)s_coldump_dasiz,
          (unsigned)s_coldump_aadr0, (unsigned)s_coldump_aadr1,
          (unsigned)s_coldump_vm0, (unsigned)s_coldump_vm1,
          s_coldump_dma_frame, kStripN, mism, mism_local, (unsigned)hash);
  for (int d = 0; d < kStripN; d++) {
    fprintf(s_coldump_fp, "%s%u", d ? "," : "", (unsigned)row7f[d]);
  }
  fprintf(s_coldump_fp, "],\"7f_loc\":[");
  for (int d = 0; d < kStripN; d++) {
    fprintf(s_coldump_fp, "%s%u", d ? "," : "", (unsigned)row7f_loc[d]);
  }
  fprintf(s_coldump_fp, "],\"vram\":[");
  for (int d = 0; d < kStripN; d++) {
    fprintf(s_coldump_fp, "%s%u", d ? "," : "", (unsigned)rowvram[d]);
  }
  /* Feet-row strip: cam-top row is often sky while the stand surface is
   * ~40–75px below (mech Y). Sample that row for swim/snap triage. */
  uint16_t row7f_feet[kStripN];
  uint16_t rowvram_feet[kStripN];
  memset(row7f_feet, 0, sizeof(row7f_feet));
  memset(rowvram_feet, 0, sizeof(rowvram_feet));
  const int feet_y = (slot == 1) ? my1 : my0;
  const uint16_t src_feet =
      (feet_y >= 0) ? mw_map_src_from_42b3(loc_x, (uint16_t)feet_y) : 0;
  if (g_ppu && feet_y >= 0) {
    const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, 0);
    const uint32_t buf_tx0 = (uint32_t)loc_x >> 4;
    const uint32_t buf_ty0 = (uint32_t)(uint16_t)feet_y >> 4;
    const int map_row = (int)(buf_ty0 & 31u);
    const int view_rel = s_present_h2h_full_frame;
    for (int d = 0; d < kStripN; d++) {
      if (src_feet) {
        const int off = (int)src_feet + d * 2;
        if (off >= 0 && off + 1 < 0x10000)
          row7f_feet[d] = mw_read7f16((uint16_t)off);
      }
      const int map_col =
          view_rel ? (d & 31) : (int)((buf_tx0 + (uint32_t)d) & 31u);
      const uint16_t vword =
          (uint16_t)(map_base + (map_row << 5) + map_col);
      rowvram_feet[d] = g_ppu->vram[vword & 0x7fffu];
    }
  }
  /* Strip lag: tile-column cam step but VRAM unshifted → ride-then-snap.
   * Sub-tile pans and all-sky cam-top rows must not set stuck. */
  {
    int lag_ok = 0, dloc = 0, exp = 0, best = 0, m0 = 0, mb = 0, stuck = 0;
    int sky = 0, solid = 0;
    int feet_ok = 0, f_exp = 0, f_best = 0, f_m0 = 0, f_mb = 0, f_stuck = 0;
    int f_sky = 0, f_solid = 0;
    const int phase = (int)(loc_x & 15u);
    for (int d = 0; d < kStripN; d++) {
      if (mw_bg1_tile_weak(rowvram[d]))
        sky++;
      else
        solid++;
      if (mw_bg1_tile_weak(rowvram_feet[d]))
        f_sky++;
      else
        f_solid++;
    }
    if (s_coldump_lag_have && s_coldump_lag_slot == slot) {
      lag_ok = 1;
      dloc = (int)loc_x - (int)s_coldump_lag_locx;
      exp = (int)(loc_x >> 4) - (int)(s_coldump_lag_locx >> 4);
      for (int d = 0; d < kStripN; d++) {
        if (rowvram[d] == s_coldump_lag_vram[d])
          m0++;
      }
      mb = -1;
      for (int sh = -8; sh <= 8; sh++) {
        int m = 0;
        for (int d = 0; d < kStripN; d++) {
          const int s = d + sh;
          if (s < 0 || s >= kStripN)
            continue;
          if (rowvram[d] == s_coldump_lag_vram[s])
            m++;
        }
        if (m > mb) {
          mb = m;
          best = sh;
        }
      }
      stuck = (exp != 0) && solid >= 4 && best == 0 && m0 >= (kStripN - 4);
    }
    if (s_coldump_lag_feet_have && s_coldump_lag_have &&
        s_coldump_lag_slot == slot && feet_y >= 0) {
      feet_ok = 1;
      f_exp = exp;
      for (int d = 0; d < kStripN; d++) {
        if (rowvram_feet[d] == s_coldump_lag_feet_vram[d])
          f_m0++;
      }
      f_mb = -1;
      for (int sh = -8; sh <= 8; sh++) {
        int m = 0;
        for (int d = 0; d < kStripN; d++) {
          const int s = d + sh;
          if (s < 0 || s >= kStripN)
            continue;
          if (rowvram_feet[d] == s_coldump_lag_feet_vram[s])
            m++;
        }
        if (m > f_mb) {
          f_mb = m;
          f_best = sh;
        }
      }
      f_stuck =
          (f_exp != 0) && f_solid >= 4 && f_best == 0 && f_m0 >= (kStripN - 4);
    }
    fprintf(s_coldump_fp,
            "],\"lag\":{\"ok\":%d,\"dloc\":%d,\"exp\":%d,\"best\":%d,"
            "\"m0\":%d,\"mb\":%d,\"stuck\":%d,\"phase\":%d,"
            "\"sky\":%d,\"solid\":%d},"
            "\"feet\":{\"y\":%d,\"src\":%u,\"sky\":%d,\"solid\":%d,"
            "\"lag\":{\"ok\":%d,\"exp\":%d,\"best\":%d,\"m0\":%d,\"mb\":%d,"
            "\"stuck\":%d},\"7f\":[",
            lag_ok, dloc, exp, best, m0, mb, stuck, phase, sky, solid, feet_y,
            (unsigned)src_feet, f_sky, f_solid, feet_ok, f_exp, f_best, f_m0,
            f_mb, f_stuck);
    for (int d = 0; d < kStripN; d++) {
      fprintf(s_coldump_fp, "%s%u", d ? "," : "", (unsigned)row7f_feet[d]);
    }
    fprintf(s_coldump_fp, "],\"vram\":[");
    for (int d = 0; d < kStripN; d++) {
      fprintf(s_coldump_fp, "%s%u", d ? "," : "", (unsigned)rowvram_feet[d]);
    }
    fprintf(s_coldump_fp, "]}}");
    s_coldump_lag_slot = slot;
    s_coldump_lag_locx = loc_x;
    memcpy(s_coldump_lag_vram, rowvram, sizeof(s_coldump_lag_vram));
    memcpy(s_coldump_lag_feet_vram, rowvram_feet,
           sizeof(s_coldump_lag_feet_vram));
    s_coldump_lag_have = 1;
    s_coldump_lag_feet_have = (feet_y >= 0) ? 1 : 0;
  }
  /* Presented OAM still live here (dump is before OAM/VRAM restore).
   * Prefer sticky prop tiles (platforms) — first-16 HUD-only missed them. */
  const int poam_extra =
      (g_ws_active && g_ws_extra > 0) ? IntMin(g_ws_extra, kWsExtraMax) : 0;
  enum { kPoamMax = 32 };
  int poam_x[kPoamMax], poam_y[kPoamMax];
  unsigned poam_t[kPoamMax];
  uint8_t poam_s[kPoamMax];
  unsigned poam_n = 0;
  uint8_t sticky_tile[64];
  unsigned sticky_tile_n = 0;
  {
    uint8_t seen[256];
    memset(seen, 0, sizeof(seen));
    for (int i = 0; i < kMwPropHomeMax && sticky_tile_n < 64u; i++) {
      if (!s_prop_sticky_valid[i])
        continue;
      for (unsigned t = 0; t < s_prop_sticky_n[i] && sticky_tile_n < 64u;
           t++) {
        const unsigned tile = (unsigned)s_prop_sticky_spr[i][t][2];
        if (tile < 256u && !seen[tile]) {
          seen[tile] = 1;
          sticky_tile[sticky_tile_n++] = (uint8_t)tile;
        }
      }
    }
  }
  if (g_ppu) {
    /* Ambiguous-band unwrap must match PpuDecodeOamX (right-hint strict).
     * Bare `x >= 256+extra` left sticky props as +326 while oam_x=-186. */
    const uint8_t *poam_rh = g_ppu->wsOamRightHintStrict ? g_ppu->wsOamRightHint
                                                         : NULL;
    /* Pass 1: sticky prop tiles anywhere in OAM (even off-screen Y). */
    for (int s = 0; s < 128 && poam_n < (unsigned)kPoamMax; s++) {
      const unsigned tile = (unsigned)(g_ppu->oam[s * 2 + 1] & 0xffu);
      int is_sticky = 0;
      for (unsigned k = 0; k < sticky_tile_n; k++) {
        if ((unsigned)sticky_tile[k] == tile) {
          is_sticky = 1;
          break;
        }
      }
      if (!is_sticky)
        continue;
      const int y = (int)(uint8_t)(g_ppu->oam[s * 2] >> 8);
      int x = (int)(g_ppu->oam[s * 2] & 0xff);
      x |= ((int)(g_ppu->highOam[s >> 2] >> ((s & 3) * 2)) & 1) << 8;
      if (x >= 256 + poam_extra) {
        x -= 512;
      } else if (x >= 256 && poam_rh &&
                 !(poam_rh[s >> 3] & (uint8_t)(1u << (s & 7)))) {
        x -= 512;
      }
      poam_x[poam_n] = x;
      poam_y[poam_n] = y;
      poam_t[poam_n] = tile;
      poam_s[poam_n] = 1;
      poam_n++;
    }
    /* Pass 2: fill with on-screen non-sticky (HUD / mechs). */
    for (int s = 0; s < 128 && poam_n < (unsigned)kPoamMax; s++) {
      const int y = (int)(uint8_t)(g_ppu->oam[s * 2] >> 8);
      if (y >= 0xf0)
        continue;
      int x = (int)(g_ppu->oam[s * 2] & 0xff);
      x |= ((int)(g_ppu->highOam[s >> 2] >> ((s & 3) * 2)) & 1) << 8;
      if (x >= 256 + poam_extra) {
        x -= 512;
      } else if (x >= 256 && poam_rh &&
                 !(poam_rh[s >> 3] & (uint8_t)(1u << (s & 7)))) {
        x -= 512;
      }
      if (x + 64 < -poam_extra || x > 256 + poam_extra)
        continue;
      const unsigned tile = (unsigned)(g_ppu->oam[s * 2 + 1] & 0xffu);
      int is_sticky = 0;
      for (unsigned k = 0; k < sticky_tile_n; k++) {
        if ((unsigned)sticky_tile[k] == tile) {
          is_sticky = 1;
          break;
        }
      }
      if (is_sticky)
        continue; /* already in pass 1 */
      poam_x[poam_n] = x;
      poam_y[poam_n] = y;
      poam_t[poam_n] = tile;
      poam_s[poam_n] = 0;
      poam_n++;
    }
  }
  /* Only attribute present.* to the slot LocalFull just drew — elev dual-tick
   * of the other peer must not inherit slot/f/n from the presented cam. */
  {
    const int present_here = (s_coldump_present_slot == slot) ? 1 : 0;
    fprintf(s_coldump_fp,
            ",\"cap\":{\"n\":[%u,%u],\"lo\":[%u,%u]},"
            "\"present\":{\"slot\":%d,\"f\":%d,\"n\":%u,\"raw\":%u,"
            "\"skip_own\":%u,\"skip_y\":%u},\"poam\":[",
            s_elev_cap_n0, s_elev_cap_n1, s_elev_cap_lo0, s_elev_cap_lo1,
            present_here ? s_coldump_present_slot : -1,
            present_here ? s_coldump_present_frame : -1,
            present_here ? s_coldump_prop_n : 0u,
            present_here ? s_coldump_prop_raw : 0u,
            present_here ? s_coldump_skip_own : 0u,
            present_here ? s_coldump_skip_y : 0u);
  }
  for (unsigned i = 0; i < poam_n; i++) {
    fprintf(s_coldump_fp, "%s{\"x\":%d,\"y\":%d,\"t\":%u,\"s\":%u}",
            i ? "," : "", poam_x[i], poam_y[i], poam_t[i],
            (unsigned)poam_s[i]);
  }
  fprintf(s_coldump_fp, "],\"props\":[");

  int prop_out = 0;
  uint16_t idx = mw_wram16(0x1E14);
  for (int guard = 0; guard < 64 && idx && prop_out < 12; guard++) {
    const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
    if (mw_obj_is_stage_prop(idx)) {
      const int home = mw_stage_prop_home_cam(idx, cam0x, cam0y, cam1x, cam1y);
      const int ps = mw_prop_slot_for_obj(idx);
      const unsigned sn =
          (s_prop_sticky_valid[ps] && s_prop_sticky_obj[ps] == idx)
              ? (unsigned)s_prop_sticky_n[ps]
              : 0u;
      const unsigned meta = (unsigned)mw_wram16((uint16_t)(idx + 8u));
      const unsigned bank = (unsigned)mw_wram16((uint16_t)(idx + 0xAu));
      const unsigned t6 = (unsigned)mw_wram16((uint16_t)(idx + 6u));
      const unsigned fl = (unsigned)mw_wram16(idx);
      const unsigned act = (fl & 0x8000u) ? 1u : 0u;
      const unsigned wx = (unsigned)mw_wram16((uint16_t)(idx + 2u));
      const unsigned wy = (unsigned)mw_wram16((uint16_t)(idx + 4u));
      const unsigned c = (unsigned)mw_wram16((uint16_t)(idx + 0xCu));
      const unsigned e = (unsigned)mw_wram16((uint16_t)(idx + 0xEu));
      const unsigned w10 = (unsigned)mw_wram16((uint16_t)(idx + 0x10u));
      const unsigned w12 = (unsigned)mw_wram16((uint16_t)(idx + 0x12u));
      int d0 = -1, d1 = -1, unused0, unused1, unused2, unused3;
      const char *why =
          mw_coldump_home_why(idx, home, &d0, &d1, &unused0, &unused1,
                              &unused2, &unused3);
      int dwx = 0, dwy = 0;
      if (s_coldump_mot_valid[ps] && s_coldump_mot_obj[ps] == idx) {
        dwx = (int)wx - (int)s_coldump_mot_wx[ps];
        dwy = (int)wy - (int)s_coldump_mot_wy[ps];
      }
      s_coldump_mot_obj[ps] = idx;
      s_coldump_mot_wx[ps] = (uint16_t)wx;
      s_coldump_mot_wy[ps] = (uint16_t)wy;
      s_coldump_mot_valid[ps] = 1;
      const unsigned bg_hit =
          (s_coldump_bg_obj[ps] == idx) ? (unsigned)s_coldump_bg_hit[ps] : 0u;
      const unsigned bg_try =
          (s_coldump_bg_obj[ps] == idx) ? (unsigned)s_coldump_bg_try[ps] : 0u;
      const int sx = (int)wx - (int)loc_x;
      const int sy = (int)wy - (int)loc_y;
      /* Home sticky only — matches present (no foreign FOV OAM). */
      const int draw = (sn > 0 && home == slot) ? 1 : 0;
      const int glue = (s_prop_use_valid[ps] && s_prop_use_obj[ps] == idx)
                           ? (int)s_prop_use_glued[ps]
                           : ((s_prop_glue_hits[ps] >= 2 &&
                               s_prop_anchor_valid[ps] &&
                               s_prop_anchor_obj[ps] == idx)
                                  ? 1
                                  : 0);
      int use_wx_i =
          (s_prop_use_valid[ps] && s_prop_use_obj[ps] == idx)
              ? (int)s_prop_use_wx[ps]
              : (glue && s_prop_anchor_valid[ps] &&
                         s_prop_anchor_obj[ps] == idx
                     ? (int)s_prop_anchor_wx[ps]
                     : (int)wx);
      int use_wy_i =
          (s_prop_use_valid[ps] && s_prop_use_obj[ps] == idx)
              ? (int)s_prop_use_wy[ps]
              : (glue && s_prop_anchor_valid[ps] &&
                         s_prop_anchor_obj[ps] == idx
                     ? (int)s_prop_anchor_wy[ps]
                     : (int)wy);
      mw_prop_apply_c39e_soak_probe(idx, &use_wx_i, &use_wy_i);
      const unsigned use_wx = (unsigned)use_wx_i;
      const unsigned use_wy = (unsigned)use_wy_i;
      const int ax =
          (s_prop_anchor_valid[ps] && s_prop_anchor_obj[ps] == idx)
              ? (int)s_prop_anchor_wx[ps]
              : -1;
      const int ay =
          (s_prop_anchor_valid[ps] && s_prop_anchor_obj[ps] == idx)
              ? (int)s_prop_anchor_wy[ps]
              : -1;
      const int use_sx = use_wx_i - (int)loc_x;
      const int use_sy = use_wy_i - (int)loc_y;
      int mox0 = 0;
      if (sn > 0) {
        mox0 = mw_prop_is_pinned_hazard(idx)
                   ? -8
                   : (int)s_prop_sticky_mox[ps][0];
      }
      const int oam_x = use_sx + mox0;
      const int d01 = (int)cam1x - (int)cam0x;
      const int fov = mw_prop_in_local_fov((int)wx, (int)wy, loc_x, loc_y);
      const int far_y = mw_prop_origin_far_y((int)wy, loc_y);
      const int use_fov =
          mw_prop_in_local_fov((int)use_wx, (int)use_wy, loc_x, loc_y);

      fprintf(s_coldump_fp,
              "%s{\"o\":%u,\"m\":%u,\"bank\":%u,\"t6\":%u,\"fl\":%u,"
              "\"h\":%d,\"why\":\"%s\",\"d0\":%d,\"d1\":%d,"
              "\"sn\":%u,\"act\":%u,\"draw\":%d,\"pin\":%d,"
              "\"wx\":%u,\"wy\":%u,\"dwx\":%d,\"dwy\":%d,"
              "\"sx\":%d,\"sy\":%d,"
              "\"glue\":%d,\"gh\":%u,\"use_wx\":%u,\"use_wy\":%u,"
              "\"use_sx\":%d,\"use_sy\":%d,\"oam_x\":%d,\"ax\":%d,\"ay\":%d,"
              "\"pcx\":%u,\"d01\":%d,"
              "\"c\":%u,\"e\":%u,\"w10\":%u,\"w12\":%u,"
              "\"bg_try\":%u,\"bg_hit\":%u,\"fov\":%d,\"use_fov\":%d,"
              "\"far_y\":%d,\"tiles\":[",
              prop_out ? "," : "", (unsigned)idx, meta, bank, t6, fl, home,
              why, d0, d1, sn, act, draw, mw_prop_is_pinned_hazard(idx) ? 1 : 0,
              wx, wy, dwx, dwy, sx, sy, glue, (unsigned)s_prop_glue_hits[ps],
              use_wx, use_wy, use_sx, use_sy, oam_x, ax, ay, (unsigned)loc_x,
              d01, c, e, w10, w12, bg_try, bg_hit, fov, use_fov, far_y);
      if (sn > 0) {
        for (unsigned t = 0; t < sn; t++) {
          fprintf(s_coldump_fp,
                  "%s{\"t\":%u,\"sz\":%u,\"mox\":%d,\"moy\":%d}", t ? "," : "",
                  (unsigned)s_prop_sticky_spr[ps][t][2],
                  (unsigned)s_prop_sticky_sz[ps][t],
                  (int)s_prop_sticky_mox[ps][t],
                  (int)s_prop_sticky_moy[ps][t]);
        }
      }
      fprintf(s_coldump_fp, "]}");
      prop_out++;
    }
    if (next == 0 || next == idx)
      break;
    idx = next;
  }

  /* Shared $B1 item sticky counts (despawn / half-cull checks). */
  fprintf(s_coldump_fp, "],\"items\":[");
  int item_out = 0;
  for (int i = 0; i < kMwItemStickyMax && item_out < 8; i++) {
    if (!s_item_sticky_valid[i] || !s_item_sticky_obj[i])
      continue;
    fprintf(s_coldump_fp, "%s{\"o\":%u,\"sn\":%u,\"m\":%u}",
            item_out ? "," : "", (unsigned)s_item_sticky_obj[i],
            (unsigned)s_item_sticky_n[i],
            (unsigned)mw_wram16((uint16_t)(s_item_sticky_obj[i] + 8u)));
    item_out++;
  }
  /* Elevators ($D5B8) are BG2 — not in props[]. Emit separately. */
  fprintf(s_coldump_fp, "],\"elevs\":[");
  int elev_out = 0;
  idx = mw_wram16(0x1E14);
  for (int guard = 0; guard < 64 && idx && elev_out < 8; guard++) {
    const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
    const uint16_t meta = mw_wram16((uint16_t)(idx + 8u));
    if (meta == 0xD5B8u) {
      const unsigned fl = (unsigned)mw_wram16(idx);
      const unsigned wx = (unsigned)mw_wram16((uint16_t)(idx + 2u));
      const unsigned wy = (unsigned)mw_wram16((uint16_t)(idx + 4u));
      const int sx = (int)wx - (int)loc_x;
      const int sy = (int)wy - (int)loc_y;
      fprintf(s_coldump_fp,
              "%s{\"o\":%u,\"m\":%u,\"act\":%u,\"wx\":%u,\"wy\":%u,"
              "\"sx\":%d,\"sy\":%d}",
              elev_out ? "," : "", (unsigned)idx, (unsigned)meta,
              (fl & 0x8000u) ? 1u : 0u, wx, wy, sx, sy);
      elev_out++;
    }
    if (next == 0 || next == idx)
      break;
    idx = next;
  }

  /*
   * All ObjectListHead ($1E14) records near either mech — any bank/meta.
   * Separates collision authority (tracks feet) from render movers
   * (props[] / $7F) that may not share the same MwObject.
   */
  fprintf(s_coldump_fp, "],\"near\":[");
  {
    enum { kNearMax = 24, kNearDx = 160, kNearDy = 140 };
    uint8_t near_seen[kMwColdumpNearMot];
    memset(near_seen, 0, sizeof(near_seen));
    int near_out = 0;
    idx = mw_wram16(0x1E14u);
    for (int guard = 0; guard < 64 && idx && near_out < kNearMax; guard++) {
      const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
      const unsigned fl = (unsigned)mw_wram16(idx);
      const unsigned wx = (unsigned)mw_wram16((uint16_t)(idx + 2u));
      const unsigned wy = (unsigned)mw_wram16((uint16_t)(idx + 4u));
      const unsigned t6 = (unsigned)mw_wram16((uint16_t)(idx + 6u));
      const unsigned meta = (unsigned)mw_wram16((uint16_t)(idx + 8u));
      const unsigned bank = (unsigned)mw_wram16((uint16_t)(idx + 0xAu));
      const unsigned act = (fl & 0x8000u) ? 1u : 0u;
      int n0 = 0, n1 = 0;
      int dx0 = 0, dy0 = 0, dx1 = 0, dy1 = 0;
      if (mx0 >= 0) {
        dx0 = (int)wx - mx0;
        dy0 = (int)wy - my0;
        if (dx0 <= kNearDx && dx0 >= -kNearDx && dy0 <= kNearDy &&
            dy0 >= -kNearDy)
          n0 = 1;
      }
      if (mx1 >= 0) {
        dx1 = (int)wx - mx1;
        dy1 = (int)wy - my1;
        if (dx1 <= kNearDx && dx1 >= -kNearDx && dy1 <= kNearDy &&
            dy1 >= -kNearDy)
          n1 = 1;
      }
      if (n0 || n1) {
        const char *cls = "other";
        if (mw_obj_is_stage_prop(idx))
          cls = "prop";
        else if (meta == 0xD5B8u)
          cls = "elev";
        else if (mw_obj_is_shared_b1_item(idx))
          cls = "item";
        else if ((t6 & 0xFF00u) == 0x0100u || (t6 & 0xFF00u) == 0x0200u)
          cls = "mech";
        int dwx = 0, dwy = 0;
        int mot_i = -1;
        for (int i = 0; i < kMwColdumpNearMot; i++) {
          if (s_coldump_near_valid[i] && s_coldump_near_obj[i] == idx) {
            mot_i = i;
            break;
          }
        }
        if (mot_i < 0) {
          for (int i = 0; i < kMwColdumpNearMot; i++) {
            if (!s_coldump_near_valid[i]) {
              mot_i = i;
              break;
            }
          }
        }
        if (mot_i < 0)
          mot_i = (int)((idx - 0x1934u) / 0x1Cu) % kMwColdumpNearMot;
        if (s_coldump_near_valid[mot_i] && s_coldump_near_obj[mot_i] == idx) {
          dwx = (int)wx - (int)s_coldump_near_wx[mot_i];
          dwy = (int)wy - (int)s_coldump_near_wy[mot_i];
        }
        s_coldump_near_obj[mot_i] = idx;
        s_coldump_near_wx[mot_i] = (uint16_t)wx;
        s_coldump_near_wy[mot_i] = (uint16_t)wy;
        s_coldump_near_valid[mot_i] = 1;
        near_seen[mot_i] = 1;
        unsigned t7f = 0;
        const uint16_t off7 =
            mw_7f_off_from_world((uint16_t)wx, (uint16_t)wy);
        if (off7)
          t7f = (unsigned)mw_read7f16(off7);
        const int sx = (int)wx - (int)loc_x;
        const int sy = (int)wy - (int)loc_y;
        fprintf(s_coldump_fp,
                "%s{\"o\":%u,\"m\":%u,\"bank\":%u,\"t6\":%u,\"fl\":%u,"
                "\"act\":%u,\"cls\":\"%s\",\"wx\":%u,\"wy\":%u,"
                "\"dwx\":%d,\"dwy\":%d,\"sx\":%d,\"sy\":%d,"
                "\"n0\":%d,\"n1\":%d,\"dx0\":%d,\"dy0\":%d,"
                "\"dx1\":%d,\"dy1\":%d,\"t7f\":%u,"
                "\"prop\":%d,\"elev\":%d}",
                near_out ? "," : "", (unsigned)idx, meta, bank, t6, fl, act,
                cls, wx, wy, dwx, dwy, sx, sy, n0, n1, dx0, dy0, dx1, dy1, t7f,
                mw_obj_is_stage_prop(idx) ? 1 : 0,
                meta == 0xD5B8u ? 1 : 0);
        near_out++;
      }
      if (next == 0 || next == idx)
        break;
      idx = next;
    }
    /* Drop trail slots not seen this frame so recycled pool objs don't lie. */
    for (int i = 0; i < kMwColdumpNearMot; i++) {
      if (s_coldump_near_valid[i] && !near_seen[i])
        s_coldump_near_valid[i] = 0;
    }
  }

  /*
   * plat: collision GATE (feet / near $C382) vs present BLOB under feet.
   * Circled hazard stand sits at/below mech Y — FOV-widest shelf above
   * (ody≪0) is a DIFFERENT map blob. Prefer runs covering coll X with
   * ody in [-16,+112]; dump under[] rows for ID when feet lip is sky.
   */
  fprintf(s_coldump_fp, "],\"plat\":");
  {
    enum { kPlatFeetDy = 120, kPlatFeetDx = 160 };
    const int mx = (slot == 1) ? mx1 : mx0;
    const int my = (slot == 1) ? my1 : my0;
    uint16_t best = 0;
    int best_d2 = 0x7fffffff;
    if (mx >= 0 && my >= 0) {
      uint16_t scan = mw_wram16(0x1E14u);
      for (int guard = 0; guard < 64 && scan; guard++) {
        const uint16_t next = mw_wram16((uint16_t)(scan + 0x14u));
        if (mw_wram16((uint16_t)(scan + 8u)) == 0xC382u &&
            (mw_wram16(scan) & 0x8000u)) {
          const int owx = (int)mw_wram16((uint16_t)(scan + 2u));
          const int owy = (int)mw_wram16((uint16_t)(scan + 4u));
          const int dx = owx - mx;
          const int dy = owy - my;
          if (dx <= kPlatFeetDx && dx >= -kPlatFeetDx && dy <= kPlatFeetDy &&
              dy >= -kPlatFeetDy) {
            const int d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
              best_d2 = d2;
              best = scan;
            }
          }
        }
        if (next == 0 || next == scan)
          break;
        scan = next;
      }
    }
    const char *src = best ? "obj" : "feet";
    unsigned wx = 0, wy = 0, use_wx = 0, use_wy = 0, meta = 0, sn = 0;
    int home = -1, glue = 0, draw = 0, pin = 0;
    int sx = 0, sy = 0, use_sx = 0, use_sy = 0;
    int mox0 = 0, moy0 = 0;
    /* Collision contact: $C382 near feet, else mech XY. */
    int coll_wx = mx >= 0 ? mx : (int)loc_x + 128;
    int coll_wy = my >= 0 ? my : (int)loc_y + 112;
    if (best) {
      const int ps = mw_prop_slot_for_obj(best);
      meta = (unsigned)mw_wram16((uint16_t)(best + 8u));
      wx = (unsigned)mw_wram16((uint16_t)(best + 2u));
      wy = (unsigned)mw_wram16((uint16_t)(best + 4u));
      sn = (s_prop_sticky_valid[ps] && s_prop_sticky_obj[ps] == best)
               ? (unsigned)s_prop_sticky_n[ps]
               : 0u;
      home = mw_stage_prop_home_cam(best, cam0x, cam0y, cam1x, cam1y);
      glue = (s_prop_use_valid[ps] && s_prop_use_obj[ps] == best)
                 ? (int)s_prop_use_glued[ps]
                 : 0;
      use_wx = (s_prop_use_valid[ps] && s_prop_use_obj[ps] == best)
                   ? (unsigned)s_prop_use_wx[ps]
                   : wx;
      use_wy = (s_prop_use_valid[ps] && s_prop_use_obj[ps] == best)
                   ? (unsigned)s_prop_use_wy[ps]
                   : wy;
      use_sx = (int)use_wx - (int)loc_x;
      use_sy = (int)use_wy - (int)loc_y;
      sx = (int)wx - (int)loc_x;
      sy = (int)wy - (int)loc_y;
      pin = mw_prop_is_pinned_hazard(best) ? 1 : 0;
      draw = (sn > 0 && home == slot) ? 1 : 0;
      coll_wx = (int)use_wx;
      coll_wy = (int)use_wy;
      if (sn > 0) {
        mox0 = pin ? -8 : (int)s_prop_sticky_mox[ps][0];
        moy0 = (int)s_prop_sticky_moy[ps][0] + mw_prop_pin_dy();
      }
    } else if (my >= 0) {
      use_wy = (unsigned)my;
      use_wx = (mx >= 0) ? (unsigned)mx : loc_x + 128u;
      wx = use_wx;
      wy = use_wy;
      use_sx = (int)use_wx - (int)loc_x;
      use_sy = (int)use_wy - (int)loc_y;
      sx = use_sx;
      sy = use_sy;
    }

    const int mech_sx = (mx >= 0) ? (mx - (int)loc_x) : 128;
    const int feet_sy = (my >= 0) ? (my - (int)loc_y) : 112;

    /* Hazard stripe OAM only ($62/$C8) — never $E4 backpack. */
    int stripe_ok = 0;
    int stripe_sx = 0, stripe_sy = 0;
    unsigned tile0 = 0;
    int stripe_sticky = 0;
    if (best && sn > 0) {
      tile0 = (unsigned)s_prop_sticky_spr[mw_prop_slot_for_obj(best)][0][2];
      if (mw_plat_stripe_tile_ok(tile0)) {
        stripe_sx = use_sx + mox0;
        stripe_sy = use_sy + moy0;
        stripe_ok = 1;
        stripe_sticky = 1;
      } else {
        tile0 = 0;
      }
    } else {
      int best_score = 0x7fffffff;
      for (int pass = 0; pass < 2 && !stripe_ok; pass++) {
        best_score = 0x7fffffff;
        for (unsigned i = 0; i < poam_n; i++) {
          if (pass == 0 && !poam_s[i])
            continue;
          if (!mw_plat_stripe_tile_ok(poam_t[i]))
            continue;
          if (poam_x[i] <= -80 || poam_x[i] >= 340)
            continue;
          const int adx = poam_x[i] - mech_sx;
          const int ady = poam_y[i] - feet_sy;
          const int aady = ady < 0 ? -ady : ady;
          if (aady > 64)
            continue;
          int score = adx * adx + ady * ady;
          if (poam_s[i])
            score -= 5000;
          if (score < best_score) {
            best_score = score;
            stripe_sx = poam_x[i];
            stripe_sy = poam_y[i];
            tile0 = poam_t[i];
            stripe_sticky = poam_s[i] ? 1 : 0;
            stripe_ok = 1;
          }
        }
      }
    }

    /* Feet collision sample (lip often sky; body is under[] below). */
    int feet_bx = coll_wx;
    int feet_by = coll_wy;
    unsigned feet_t7f = 0, feet_tvram = 0;
    int feet_vok = 0;
    {
      const uint16_t off =
          mw_7f_off_from_world((uint16_t)feet_bx, (uint16_t)feet_by);
      if (off)
        feet_t7f = (unsigned)mw_read7f16(off);
      if (g_ppu) {
        feet_tvram = (unsigned)mw_bg1_vram_at_world(
            (uint16_t)feet_bx, (uint16_t)feet_by, loc_x, loc_y);
        feet_vok = 1;
      }
    }

    /*
     * under[]: BG1 rows at coll_wy + 0..+96 (circled stand body). Each row:
     * solid count, best run covering coll X (or nearest), 7f+vram samples.
     */
    enum { kUnderN = 7, kUnderCols = 22, kMinRun = 2 };
    int under_solid[kUnderN];
    int under_n[kUnderN];
    int under_wx[kUnderN];
    int under_west[kUnderN];
    int under_east[kUnderN];
    unsigned under_t7f[kUnderN];
    unsigned under_tv[kUnderN];
    unsigned under_t[kUnderN];
    memset(under_solid, 0, sizeof(under_solid));
    memset(under_n, 0, sizeof(under_n));
    memset(under_wx, 0, sizeof(under_wx));
    memset(under_west, 0, sizeof(under_west));
    memset(under_east, 0, sizeof(under_east));
    memset(under_t7f, 0, sizeof(under_t7f));
    memset(under_tv, 0, sizeof(under_tv));
    memset(under_t, 0, sizeof(under_t));
    if (my >= 0 || best) {
      const int x0 = (coll_wx - (kUnderCols / 2) * 16) & ~15;
      for (int ui = 0; ui < kUnderN; ui++) {
        const int ty = (coll_wy + ui * 16) & ~15;
        uint8_t solid[kUnderCols];
        uint16_t tiles[kUnderCols];
        memset(solid, 0, sizeof(solid));
        memset(tiles, 0, sizeof(tiles));
        int nsolid = 0;
        for (int c = 0; c < kUnderCols; c++) {
          const int tx = x0 + c * 16;
          uint16_t t7 = 0, tv = 0;
          const uint16_t off =
              mw_7f_off_from_world((uint16_t)tx, (uint16_t)ty);
          if (off)
            t7 = mw_read7f16(off);
          if (g_ppu)
            tv = mw_bg1_vram_at_world((uint16_t)tx, (uint16_t)ty, loc_x,
                                      loc_y);
          /* Prefer VRAM (what you see); fall back to $7F. */
          const uint16_t t = !mw_bg1_tile_weak(tv) ? tv : t7;
          if (tx == (coll_wx & ~15)) {
            under_t7f[ui] = (unsigned)t7;
            under_tv[ui] = (unsigned)tv;
          }
          if (mw_bg1_tile_weak(t))
            continue;
          solid[c] = 1;
          tiles[c] = t;
          nsolid++;
        }
        under_solid[ui] = nsolid;
        /* Best contiguous run: prefer covering coll_wx. */
        int br_n = 0, br_wx = 0, br_west = 0, br_east = 0;
        unsigned br_t = 0;
        int br_score = -0x7fffffff;
        int c = 0;
        while (c < kUnderCols) {
          while (c < kUnderCols && !solid[c])
            c++;
          if (c >= kUnderCols)
            break;
          const int c0 = c;
          unsigned t0 = tiles[c];
          int sumx = 0, n = 0;
          while (c < kUnderCols && solid[c]) {
            sumx += x0 + c * 16 + 8;
            n++;
            c++;
          }
          if (n < kMinRun)
            continue;
          const int west = x0 + c0 * 16;
          const int east = x0 + (c0 + n) * 16 - 1;
          const int cx = sumx / n;
          int adx = cx - coll_wx;
          if (adx < 0)
            adx = -adx;
          int score = n * 10 - adx;
          if (west <= coll_wx && east >= coll_wx)
            score += 200;
          if (score > br_score) {
            br_score = score;
            br_n = n;
            br_wx = cx;
            br_west = west;
            br_east = east;
            br_t = t0;
          }
        }
        under_n[ui] = br_n;
        under_wx[ui] = br_wx;
        under_west[ui] = br_west;
        under_east[ui] = br_east;
        under_t[ui] = br_t;
      }
    }

    /*
     * Present blob = under-feet stand body (not FOV-widest upper shelf).
     * Scan only coll_wy-16 .. coll_wy+112; heavy bonus for ody>=0.
     */
    int pres_ok = 0, pres_n = 0, pres_wx = 0, pres_wy = 0;
    int pres_west = 0, pres_east = 0, pres_top = 0;
    unsigned pres_t = 0;
    const char *pres_src = "none";
    if (my >= 0 || best) {
      enum { kPad = 48, kCols = 28 };
      int x0 = ((int)loc_x - kPad) & ~15;
      int y0 = (coll_wy - 16) & ~15;
      int y1 = (coll_wy + 112 + 15) & ~15;
      int best_score = -0x7fffffff;
      int best_n = 0, best_cx = 0, best_cy = 0;
      int best_west = 0, best_east = 0;
      unsigned best_t = 0;
      int best_from_vram = 0;

      for (int pass = 0; pass < 2; pass++) {
        if (pass == 1 && !g_ppu)
          continue;
        for (int ty = y0; ty < y1; ty += 16) {
          const int ody = (ty + 8) - coll_wy;
          if (ody < -16 || ody > 112)
            continue;

          uint8_t solid[kCols];
          uint16_t tiles[kCols];
          memset(solid, 0, sizeof(solid));
          memset(tiles, 0, sizeof(tiles));
          for (int c = 0; c < kCols; c++) {
            const int tx = x0 + c * 16;
            uint16_t t = 0;
            if (pass == 0) {
              const uint16_t off =
                  mw_7f_off_from_world((uint16_t)tx, (uint16_t)ty);
              if (!off)
                continue;
              t = mw_read7f16(off);
            } else {
              const int sx = tx - (int)loc_x;
              if (sx < -48 || sx >= 360)
                continue;
              t = mw_bg1_vram_at_world((uint16_t)tx, (uint16_t)ty, loc_x,
                                       loc_y);
            }
            if (mw_bg1_tile_weak(t))
              continue;
            solid[c] = 1;
            tiles[c] = t;
          }

          int c = 0;
          while (c < kCols) {
            while (c < kCols && !solid[c])
              c++;
            if (c >= kCols)
              break;
            const int c0 = c;
            unsigned t0 = tiles[c];
            int sumx = 0, n = 0;
            while (c < kCols && solid[c]) {
              sumx += x0 + c * 16 + 8;
              n++;
              c++;
            }
            if (n < kMinRun)
              continue;
            const int west = x0 + c0 * 16;
            const int east = x0 + (c0 + n) * 16 - 1;
            const int cx = sumx / n;
            const int cy = ty + 8;
            const int sx = cx - (int)loc_x;
            int adx = cx - coll_wx;
            if (adx < 0)
              adx = -adx;

            /* Under/at feet wins; upper shelves (ody<0) heavily penalized. */
            int score = n * 8 - adx * 3;
            if (ody >= 0 && ody <= 80)
              score += 200 - ody;
            else if (ody >= -16)
              score += 40;
            else
              score -= 200;
            if (west <= coll_wx && east >= coll_wx)
              score += 150;
            else if (adx <= 64)
              score += 80;
            else if (adx > 200)
              score -= 100;
            if (sx >= 0 && sx < 320)
              score += 40;
            else
              score -= 60;
            if (pass == 1 && adx <= 96)
              score += 30;
            if (s_coldump_plat_have && s_coldump_plat_slot == slot &&
                s_coldump_plat_pres_ok) {
              int pdy = cy - s_coldump_plat_pres_wy;
              if (pdy < 0)
                pdy = -pdy;
              int pdx = cx - s_coldump_plat_pres_wx;
              if (pdx < 0)
                pdx = -pdx;
              if (pdy <= 24 && pdx <= 48)
                score += 50;
            }
            if (score > best_score) {
              best_score = score;
              best_n = n;
              best_cx = cx;
              best_cy = cy;
              best_west = west;
              best_east = east;
              best_t = t0;
              best_from_vram = pass;
            }
          }
        }
      }

      /* Prefer densest under[] row if FOV scan still empty/weak. */
      if (best_n < kMinRun || best_score < 40) {
        for (int ui = 0; ui < kUnderN; ui++) {
          if (under_n[ui] < kMinRun)
            continue;
          const int cy = ((coll_wy + ui * 16) & ~15) + 8;
          const int ody = cy - coll_wy;
          int score = under_n[ui] * 8 + 180 - (ody < 0 ? -ody : ody);
          if (under_west[ui] <= coll_wx && under_east[ui] >= coll_wx)
            score += 150;
          if (score > best_score) {
            best_score = score;
            best_n = under_n[ui];
            best_cx = under_wx[ui];
            best_cy = cy;
            best_west = under_west[ui];
            best_east = under_east[ui];
            best_t = under_t[ui];
            best_from_vram = !mw_bg1_tile_weak((uint16_t)under_tv[ui]);
          }
        }
      }

      if (best_n >= kMinRun && best_score > -0x100000) {
        int adx = best_cx - coll_wx;
        if (adx < 0)
          adx = -adx;
        const int sx = best_cx - (int)loc_x;
        /* Reject far-east shelf (sx≥256) — dump false latch odx≈200 sx=289. */
        const int near_ok =
            adx <= 160 || (sx >= 0 && sx < 256 && adx <= 180);
        if (near_ok || (best_score >= 40 && sx < 256)) {
          pres_ok = 1;
          pres_n = best_n;
          pres_wx = best_cx;
          pres_wy = best_cy;
          pres_west = best_west;
          pres_east = best_east;
          pres_t = best_t;
          pres_src = best_from_vram ? "vram" : "7f";
          if (s_coldump_plat_have && s_coldump_plat_slot == slot &&
              s_coldump_plat_pres_ok) {
            int dwy = pres_wy - s_coldump_plat_pres_wy;
            if (dwy < 0)
              dwy = -dwy;
            if (dwy <= 24) {
              const int prev = s_coldump_plat_pres_wx;
              if (prev >= pres_west && prev <= pres_east)
                pres_wx = prev;
            }
          }
        }
      }
    }
    const int pres_sx = pres_ok ? (pres_wx - (int)loc_x) : 0;
    const int pres_sy = pres_ok ? (pres_wy - (int)loc_y) : 0;
    const int odx = pres_ok ? (pres_wx - coll_wx) : 0;
    const int ody = pres_ok ? (pres_wy - coll_wy) : 0;

    int lag_ok = 0, dloc = 0, d_use = 0, d_stripe = 0;
    int d_pres_wx = 0, d_pres_sx = 0;
    int stripe_lock = 0, pres_lock = 0, pres_world = 0, unit = 0, snap = 0;
    int use_lock = 0;
    const int same_id =
        s_coldump_plat_have && s_coldump_plat_slot == slot &&
        s_coldump_plat_obj == best;
    if (same_id && (best || my >= 0)) {
      lag_ok = 1;
      dloc = (int)loc_x - (int)s_coldump_plat_locx;
      if (best) {
        d_use = use_sx - s_coldump_plat_use_sx;
        use_lock =
            (dloc <= -2 || dloc >= 2) && (d_use <= 1 && d_use >= -1);
      }
      if (stripe_ok && s_coldump_plat_stripe_ok) {
        d_stripe = stripe_sx - s_coldump_plat_stripe_sx;
        stripe_lock = (dloc <= -2 || dloc >= 2) &&
                      (d_stripe <= 1 && d_stripe >= -1);
      }
      if (pres_ok && s_coldump_plat_pres_ok) {
        d_pres_wx = pres_wx - s_coldump_plat_pres_wx;
        d_pres_sx = (pres_wx - (int)loc_x) -
                    (s_coldump_plat_pres_wx - (int)s_coldump_plat_locx);
        /* Cam-follow present: d_pres_wx ≈ dloc. World-lock: d_pres_wx ≈ 0. */
        pres_lock = (dloc <= -2 || dloc >= 2) &&
                    (d_pres_wx - dloc <= 1 && d_pres_wx - dloc >= -1);
        pres_world = (dloc <= -2 || dloc >= 2) &&
                     (d_pres_wx <= 1 && d_pres_wx >= -1);
        snap = pres_lock ||
               ((d_pres_wx <= -8 || d_pres_wx >= 8) && !pres_world);
        unit = (stripe_lock && pres_world) || (use_lock && pres_world);
      }
    }

    if (my < 0 && !best) {
      fprintf(s_coldump_fp, "{\"ok\":0}");
      s_coldump_plat_have = 0;
    } else {
      fprintf(
          s_coldump_fp,
          "{\"ok\":1,\"src\":\"%s\",\"o\":%u,\"m\":%u,\"h\":%d,"
          "\"pin\":%d,\"sn\":%u,\"glue\":%d,\"draw\":%d,"
          "\"wx\":%u,\"wy\":%u,\"use_wx\":%u,\"use_wy\":%u,"
          "\"sx\":%d,\"sy\":%d,\"use_sx\":%d,\"use_sy\":%d,"
          "\"mech_sx\":%d,\"feet_sy\":%d,"
          "\"coll\":{\"wx\":%d,\"wy\":%d,\"sx\":%d,\"sy\":%d},"
          "\"stripe\":{\"ok\":%d,\"sx\":%d,\"sy\":%d,\"t\":%u,"
          "\"mox\":%d,\"moy\":%d,\"sticky\":%d},"
          "\"feet\":{\"wx\":%d,\"wy\":%d,\"sx\":%d,\"sy\":%d,"
          "\"t7f\":%u,\"tvram\":%u,\"vok\":%d,\"weak\":%d},"
          "\"pres\":{\"ok\":%d,\"src\":\"%s\",\"n\":%d,\"wx\":%d,\"wy\":%d,"
          "\"sx\":%d,\"sy\":%d,\"west\":%d,\"east\":%d,\"top\":%d,"
          "\"t\":%u,\"odx\":%d,\"ody\":%d},"
          "\"body\":{\"ok\":%d,\"idx\":%d,\"sx\":%d,\"wx\":%d,\"t\":%u},"
          "\"under\":[",
          src, (unsigned)best, meta, home, pin, sn, glue, draw, wx, wy, use_wx,
          use_wy, sx, sy, use_sx, use_sy, mech_sx, feet_sy, coll_wx, coll_wy,
          coll_wx - (int)loc_x, coll_wy - (int)loc_y, stripe_ok, stripe_sx,
          stripe_sy, tile0, mox0, moy0, stripe_sticky, feet_bx, feet_by,
          feet_bx - (int)loc_x, feet_by - (int)loc_y, feet_t7f, feet_tvram,
          feet_vok, mw_bg1_tile_weak((uint16_t)feet_t7f) ? 1 : 0, pres_ok,
          pres_src, pres_n, pres_wx, pres_wy, pres_sx, pres_sy, pres_west,
          pres_east, pres_ok ? (pres_wy - 8) : 0, pres_t, odx, ody, pres_ok, -1,
          pres_sx, pres_wx, pres_t);
      for (int ui = 0; ui < kUnderN; ui++) {
        const int uy = (coll_wy + ui * 16) & ~15;
        fprintf(s_coldump_fp,
                "%s{\"dy\":%d,\"y\":%d,\"solid\":%d,\"n\":%d,\"wx\":%d,"
                "\"west\":%d,\"east\":%d,\"t\":%u,\"t7f\":%u,\"tv\":%u,"
                "\"sx\":%d}",
                ui ? "," : "", ui * 16, uy, under_solid[ui], under_n[ui],
                under_wx[ui], under_west[ui], under_east[ui], under_t[ui],
                under_t7f[ui], under_tv[ui],
                under_n[ui] ? under_wx[ui] - (int)loc_x : 0);
      }
      fprintf(s_coldump_fp,
              "],\"lag\":{\"ok\":%d,\"dloc\":%d,\"d_use\":%d,\"d_stripe\":%d,"
              "\"d_pres_wx\":%d,\"d_pres_sx\":%d,\"use_lock\":%d,"
              "\"stripe_lock\":%d,\"pres_lock\":%d,\"pres_world\":%d,"
              "\"unit\":%d,\"snap\":%d}}",
              lag_ok, dloc, d_use, d_stripe, d_pres_wx, d_pres_sx, use_lock,
              stripe_lock, pres_lock, pres_world, unit, snap);
      s_coldump_plat_slot = slot;
      s_coldump_plat_obj = best;
      s_coldump_plat_locx = loc_x;
      s_coldump_plat_use_sx = use_sx;
      s_coldump_plat_stripe_sx = stripe_sx;
      s_coldump_plat_pres_wx = pres_wx;
      s_coldump_plat_pres_wy = pres_wy;
      s_coldump_plat_pres_t = pres_t;
      s_coldump_plat_stripe_ok = (uint8_t)stripe_ok;
      s_coldump_plat_pres_ok = (uint8_t)pres_ok;
      s_coldump_plat_have = 1;
      if (pres_ok && (slot == 0 || slot == 1)) {
        const int psx = pres_wx - (int)loc_x;
        /* Skip far-east shelf false wins (soak wx≈912 / sx≈314). */
        if (psx >= -64 && psx < 320) {
          s_stand_latch_pres_wx[slot] = pres_wx;
          s_stand_latch_pres_wy[slot] = pres_wy;
          s_stand_latch_pres_have[slot] = 1;
        }
      }
    }
  }

  /*
   * paint[] — FOV solid BG1 runs + on-screen OAM. NOT feet/coll gated.
   * Collision and paint are separate; find where paint actually is and
   * whether that blob FOV-locks (d_wx≈dloc) or world-locks (d_wx≈0).
   */
  {
    enum {
      kPaintPad = 48,
      kPaintCols = 28,
      kPaintMinRun = 2,
      kPaintMaxCand = 8,
      kPaintMaxOam = 24
    };
    typedef struct {
      int n, wx, wy, west, east, sx, sy, score, from_vram;
      unsigned t;
    } MwPaintCand;
    MwPaintCand cands[kPaintMaxCand];
    int ncand = 0;
    int vram_solid = 0, f7_solid = 0;

    int x0 = ((int)loc_x - kPaintPad) & ~15;
    int y0 = ((int)loc_y - kPaintPad) & ~15;
    int y1 = ((int)loc_y + 224 + kPaintPad + 15) & ~15;

    for (int pass = 0; pass < 2; pass++) {
      if (pass == 1 && !g_ppu)
        continue;
      for (int ty = y0; ty < y1; ty += 16) {
        const int sy0 = ty - (int)loc_y;
        if (sy0 < -64 || sy0 >= 260)
          continue;
        uint8_t solid[kPaintCols];
        uint16_t tiles[kPaintCols];
        memset(solid, 0, sizeof(solid));
        memset(tiles, 0, sizeof(tiles));
        for (int c = 0; c < kPaintCols; c++) {
          const int tx = x0 + c * 16;
          const int sx = tx - (int)loc_x;
          if (sx < -64 || sx >= 360)
            continue;
          uint16_t t = 0;
          if (pass == 0) {
            const uint16_t off =
                mw_7f_off_from_world((uint16_t)tx, (uint16_t)ty);
            if (!off)
              continue;
            t = mw_read7f16(off);
            if (!mw_bg1_tile_weak(t))
              f7_solid++;
          } else {
            t = mw_bg1_vram_at_world((uint16_t)tx, (uint16_t)ty, loc_x,
                                     loc_y);
            if (!mw_bg1_tile_weak(t))
              vram_solid++;
          }
          if (mw_bg1_tile_weak(t))
            continue;
          solid[c] = 1;
          tiles[c] = t;
        }
        int c = 0;
        while (c < kPaintCols) {
          while (c < kPaintCols && !solid[c])
            c++;
          if (c >= kPaintCols)
            break;
          const int c0 = c;
          unsigned t0 = tiles[c];
          int sumx = 0, n = 0;
          while (c < kPaintCols && solid[c]) {
            sumx += x0 + c * 16 + 8;
            n++;
            c++;
          }
          if (n < kPaintMinRun)
            continue;
          const int west = x0 + c0 * 16;
          const int east = x0 + (c0 + n) * 16 - 1;
          const int cx = sumx / n;
          const int cy = ty + 8;
          const int sx = cx - (int)loc_x;
          const int sy = cy - (int)loc_y;
          /* Prefer large on-screen runs — never feet/coll distance. */
          int score = n * 20;
          if (sx >= 0 && sx < 320 && sy >= 0 && sy < 224)
            score += 100;
          else if (sx >= -32 && sx < 352 && sy >= -48 && sy < 260)
            score += 40;
          else
            score -= 80;
          if (pass == 1)
            score += 15; /* VRAM = what you see */
          /* Insert/replace into top-N by score. */
          int slot = ncand;
          if (ncand < kPaintMaxCand) {
            ncand++;
          } else {
            slot = 0;
            for (int i = 1; i < kPaintMaxCand; i++) {
              if (cands[i].score < cands[slot].score)
                slot = i;
            }
            if (score <= cands[slot].score)
              continue;
          }
          cands[slot].n = n;
          cands[slot].wx = cx;
          cands[slot].wy = cy;
          cands[slot].west = west;
          cands[slot].east = east;
          cands[slot].sx = sx;
          cands[slot].sy = sy;
          cands[slot].score = score;
          cands[slot].from_vram = pass;
          cands[slot].t = t0;
        }
      }
    }

    /* Sort cands by score desc (small N). */
    for (int i = 0; i < ncand; i++) {
      for (int j = i + 1; j < ncand; j++) {
        if (cands[j].score > cands[i].score) {
          MwPaintCand tmp = cands[i];
          cands[i] = cands[j];
          cands[j] = tmp;
        }
      }
    }

    /* Focus: stick to prior FOV blob by proximity; else best on-screen cand. */
    int focus = -1;
    if (s_coldump_paint_have && s_coldump_paint_slot == slot && ncand > 0) {
      int best_d2 = 0x7fffffff;
      for (int i = 0; i < ncand; i++) {
        int dx = cands[i].wx - s_coldump_paint_wx;
        int dy = cands[i].wy - s_coldump_paint_wy;
        const int d2 = dx * dx + dy * dy;
        if (d2 < best_d2 && d2 < 80 * 80) {
          best_d2 = d2;
          focus = i;
        }
      }
    }
    if (focus < 0) {
      for (int i = 0; i < ncand; i++) {
        if (cands[i].sx >= -32 && cands[i].sx < 352 && cands[i].sy >= -48 &&
            cands[i].sy < 260) {
          focus = i;
          break;
        }
      }
      if (focus < 0 && ncand > 0)
        focus = 0;
    }

    int dloc = 0, d_wx = 0, d_sx = 0, lock = 0, world = 0, snap = 0, lag_ok = 0;
    if (focus >= 0 && s_coldump_paint_have && s_coldump_paint_slot == slot) {
      lag_ok = 1;
      dloc = (int)loc_x - (int)s_coldump_paint_locx;
      d_wx = cands[focus].wx - s_coldump_paint_wx;
      d_sx = cands[focus].sx -
             (s_coldump_paint_wx - (int)s_coldump_paint_locx);
      if (dloc <= -2 || dloc >= 2) {
        lock = (d_wx - dloc <= 1 && d_wx - dloc >= -1);
        world = (d_wx <= 1 && d_wx >= -1);
        snap = lock || ((d_wx <= -8 || d_wx >= 8) && !world);
      }
    }

    /* Local mech screen XY — vis tracks paint near what you see. */
    int mech_sx = 0, mech_sy = 0, have_mech = 0;
    {
      int mmx = 0, mmy = 0;
      if (mw_find_dual_mech_xy(slot, &mmx, &mmy)) {
        mech_sx = mmx - (int)loc_x;
        mech_sy = mmy - (int)loc_y;
        have_mech = 1;
      }
    }

    /*
     * vis: cand nearest mech screen (stand lip often slightly above feet).
     * Sticky-track by SCREEN proximity so view-column–fixed ink is followed
     * even when its world X rides the cam (focus's world stick misses that).
     */
    int vis = -1;
    const int vis_tx = mech_sx;
    const int vis_ty = mech_sy - 24;
    if (have_mech && ncand > 0) {
      if (s_coldump_vis_have && s_coldump_vis_slot == slot) {
        int best_d2 = 0x7fffffff;
        for (int i = 0; i < ncand; i++) {
          const int dx = cands[i].sx - s_coldump_vis_sx;
          const int dy = cands[i].sy - s_coldump_vis_sy;
          const int d2 = dx * dx + dy * dy;
          if (d2 < best_d2 && d2 < 64 * 64) {
            best_d2 = d2;
            vis = i;
          }
        }
      }
      if (vis < 0) {
        int best_d2 = 0x7fffffff;
        for (int i = 0; i < ncand; i++) {
          const int dx = cands[i].sx - vis_tx;
          const int dy = cands[i].sy - vis_ty;
          const int d2 = dx * dx + dy * dy;
          if (d2 < best_d2) {
            best_d2 = d2;
            vis = i;
          }
        }
        if (best_d2 > 120 * 120)
          vis = -1; /* nothing near the mech on screen */
      }
    }
    int v_dloc = 0, v_dwx = 0, v_dsx = 0, v_lock = 0, v_world = 0, v_snap = 0,
        v_lag = 0;
    if (vis >= 0 && s_coldump_vis_have && s_coldump_vis_slot == slot) {
      v_lag = 1;
      v_dloc = (int)loc_x - (int)s_coldump_vis_locx;
      v_dwx = cands[vis].wx - s_coldump_vis_wx;
      v_dsx = cands[vis].sx - s_coldump_vis_sx;
      if (v_dloc <= -2 || v_dloc >= 2) {
        v_lock = (v_dwx - v_dloc <= 1 && v_dwx - v_dloc >= -1);
        v_world = (v_dwx <= 1 && v_dwx >= -1);
        /* Screen-fixed while cam moves = the hitch pattern. */
        v_snap = (v_dsx <= 1 && v_dsx >= -1) || v_lock ||
                 ((v_dwx <= -8 || v_dwx >= 8) && !v_world);
      }
    }

    /* Screen-slot VRAM: same sx/sy tile across pan ⇒ view-column sticky.
     * Threshold |dloc|≥2 (slow pans never hit 4). Also accumulate pan. */
    typedef struct {
      int sx, sy;
      unsigned t;
      int weak, same_t, view_stick, scroll, vstick_a;
    } MwPaintScr;
    MwPaintScr scr[kMwPaintScrN];
    int nscr = 0;
    int scr_dloc = 0;
    int scr_lag = 0;
    int scr_apan = 0;
    int scr_apan_fire = 0;
    if (have_mech && g_ppu) {
      /* Mech-relative + absolute mid-view (stand often not at feet sx). */
      static const int kOff[kMwPaintScrN][2] = {
          {0, 0},    {0, -16},  {0, -32}, {0, -48}, {0, 16},   {-32, -16},
          {32, -16}, {-48, -32}, {48, -32}, {128, 100}, {160, 80}, {96, 120}};
      /* First 9 are mech+offset; last 3 are absolute screen XY. */
      scr_lag = (s_coldump_scr_have && s_coldump_scr_slot == slot) ? 1 : 0;
      if (scr_lag)
        scr_dloc = (int)loc_x - (int)s_coldump_scr_locx;
      if (slot == 0 || slot == 1) {
        if (scr_lag)
          s_coldump_scr_apan[slot] =
              (int16_t)((int)s_coldump_scr_apan[slot] + scr_dloc);
        scr_apan = (int)s_coldump_scr_apan[slot];
        if (scr_apan <= -8 || scr_apan >= 8)
          scr_apan_fire = 1;
      }
      for (int i = 0; i < kMwPaintScrN; i++) {
        int sx, sy;
        if (i < 9) {
          sx = mech_sx + kOff[i][0];
          sy = mech_sy + kOff[i][1];
        } else {
          sx = kOff[i][0];
          sy = kOff[i][1];
        }
        const int wx = (int)loc_x + sx;
        const int wy = (int)loc_y + sy;
        const uint16_t t = mw_bg1_vram_at_world((uint16_t)wx, (uint16_t)wy,
                                                loc_x, loc_y);
        const int weak = mw_bg1_tile_weak(t) ? 1 : 0;
        int same_t = 0, view_stick = 0, scroll = 0, vstick_a = 0;
        if (scr_lag) {
          same_t = (t == s_coldump_scr_t[i]) ? 1 : 0;
          if (scr_dloc <= -2 || scr_dloc >= 2) {
            if (!weak && same_t)
              view_stick = 1;
            if (!weak && !same_t)
              scroll = 1;
          }
          if (scr_apan_fire && s_coldump_scr_epoch_have[slot]) {
            if (!weak && t == s_coldump_scr_epoch_t[slot][i])
              vstick_a = 1;
          }
        }
        scr[nscr].sx = sx;
        scr[nscr].sy = sy;
        scr[nscr].t = (unsigned)t;
        scr[nscr].weak = weak;
        scr[nscr].same_t = same_t;
        scr[nscr].view_stick = view_stick;
        scr[nscr].scroll = scroll;
        scr[nscr].vstick_a = vstick_a;
        nscr++;
      }
      if (scr_apan_fire && (slot == 0 || slot == 1)) {
        for (int i = 0; i < nscr; i++)
          s_coldump_scr_epoch_t[slot][i] = (uint16_t)scr[i].t;
        s_coldump_scr_epoch_have[slot] = 1;
        s_coldump_scr_apan[slot] = 0;
      } else if ((slot == 0 || slot == 1) && !s_coldump_scr_epoch_have[slot] &&
                 nscr > 0) {
        for (int i = 0; i < nscr; i++)
          s_coldump_scr_epoch_t[slot][i] = (uint16_t)scr[i].t;
        s_coldump_scr_epoch_have[slot] = 1;
      }
    }

    /* band: nearest solid VRAM in a Y band around mech (offset stand ink). */
    int band_ok = 0, band_sx = 0, band_sy = 0, band_d2 = 0;
    unsigned band_t = 0;
    if (have_mech && g_ppu) {
      int best_d2 = 0x7fffffff;
      for (int sy = mech_sy - 64; sy <= mech_sy + 32; sy += 8) {
        if (sy < -48 || sy >= 240)
          continue;
        for (int sx = 0; sx < 256; sx += 8) {
          const int wx = (int)loc_x + sx;
          const int wy = (int)loc_y + sy;
          const uint16_t t = mw_bg1_vram_at_world((uint16_t)wx, (uint16_t)wy,
                                                  loc_x, loc_y);
          if (mw_bg1_tile_weak(t))
            continue;
          const int dx = sx - mech_sx;
          const int dy = sy - mech_sy;
          const int d2 = dx * dx + dy * dy;
          if (d2 < best_d2) {
            best_d2 = d2;
            band_ok = 1;
            band_sx = sx;
            band_sy = sy;
            band_t = (unsigned)t;
            band_d2 = d2;
          }
        }
      }
    }

    /* hzd: hazard stripe OAM ($62/$C8) near mech — not mech-body CHRs. */
    int hzd_ok = 0, hzd_sx = 0, hzd_sy = 0, hzd_sticky = 0;
    unsigned hzd_t = 0;
    int h_dloc = 0, h_dsx = 0, h_lock = 0, h_world = 0, h_snap = 0, h_lag = 0;
    int h_apan = 0, h_apan_fire = 0;
    if (have_mech) {
      int best_d2 = 0x7fffffff;
      int pick = -1;
      if (s_coldump_hzd_have && s_coldump_hzd_slot == slot) {
        for (unsigned i = 0; i < poam_n; i++) {
          if (!mw_plat_stripe_tile_ok(poam_t[i]))
            continue;
          const int dx = poam_x[i] - s_coldump_hzd_sx;
          const int dy = poam_y[i] - s_coldump_hzd_sy;
          const int d2 = dx * dx + dy * dy;
          if (d2 < best_d2 && d2 < 80 * 80) {
            best_d2 = d2;
            pick = (int)i;
          }
        }
      }
      if (pick < 0) {
        best_d2 = 0x7fffffff;
        for (unsigned i = 0; i < poam_n; i++) {
          if (!mw_plat_stripe_tile_ok(poam_t[i]))
            continue;
          const int dx = poam_x[i] - mech_sx;
          const int dy = poam_y[i] - (mech_sy - 16);
          const int d2 = dx * dx + dy * dy;
          if (d2 < best_d2 && d2 < 140 * 140) {
            best_d2 = d2;
            pick = (int)i;
          }
        }
      }
      if (pick >= 0) {
        hzd_ok = 1;
        hzd_sx = poam_x[pick];
        hzd_sy = poam_y[pick];
        hzd_t = poam_t[pick];
        hzd_sticky = poam_s[pick] ? 1 : 0;
        if (s_coldump_hzd_have && s_coldump_hzd_slot == slot) {
          h_lag = 1;
          h_dloc = (int)loc_x - (int)s_coldump_hzd_locx;
          h_dsx = hzd_sx - s_coldump_hzd_sx;
          if (slot == 0 || slot == 1) {
            s_coldump_hzd_apan[slot] =
                (int16_t)((int)s_coldump_hzd_apan[slot] + h_dloc);
            h_apan = (int)s_coldump_hzd_apan[slot];
            if (h_apan <= -8 || h_apan >= 8)
              h_apan_fire = 1;
          }
          if (h_dloc <= -2 || h_dloc >= 2 || h_apan_fire) {
            /* FOV-lock: sx tracks cam (dsx≈dloc). World: sx≈-dloc. */
            h_lock = (h_dsx - h_dloc <= 2 && h_dsx - h_dloc >= -2);
            h_world = (h_dsx + h_dloc <= 2 && h_dsx + h_dloc >= -2);
            /* Screen-fixed (stand hitch): dsx≈0 while cam moves. */
            h_snap = (h_dsx <= 2 && h_dsx >= -2) || h_lock;
          }
          if (h_apan_fire && (slot == 0 || slot == 1))
            s_coldump_hzd_apan[slot] = 0;
        }
      } else if (s_coldump_hzd_slot == slot) {
        s_coldump_hzd_have = 0;
        if (slot == 0 || slot == 1)
          s_coldump_hzd_apan[slot] = 0;
      }
    }

    /*
     * spr: any non-mech mid-view OAM (not $62-only). Screen-track + lag.
     * Stand trim/pillars may use CHRs outside the hazard allowlist.
     */
    enum { kMwPaintSprTiles = 12 };
    int spr_ok = 0, spr_sx = 0, spr_sy = 0, spr_sticky = 0, spr_n = 0;
    unsigned spr_t = 0;
    int s_dloc = 0, s_dsx = 0, s_lock = 0, s_world = 0, s_snap = 0, s_lag = 0;
    int s_apan = 0, s_apan_fire = 0;
    int spr_tile_t[kMwPaintSprTiles];
    int spr_tile_n[kMwPaintSprTiles];
    int spr_tile_sx[kMwPaintSprTiles];
    int spr_tile_sy[kMwPaintSprTiles];
    int spr_nt = 0;
    {
      int pick = -1;
      int best_d2 = 0x7fffffff;
      const int aim_sx = 128;
      const int aim_sy = 100; /* fixed mid-screen — not mech lip */
      if (s_coldump_spr_have && s_coldump_spr_slot == slot) {
        for (unsigned i = 0; i < poam_n; i++) {
          if (mw_paint_mech_oam_tile(poam_t[i]))
            continue;
          if (!mw_paint_midview_xy(poam_x[i], poam_y[i]))
            continue;
          if (!mw_paint_away_from_mech(poam_x[i], poam_y[i], mech_sx, mech_sy,
                                       have_mech))
            continue;
          if (poam_t[i] != s_coldump_spr_t)
            continue;
          const int dx = poam_x[i] - s_coldump_spr_sx;
          const int dy = poam_y[i] - s_coldump_spr_sy;
          const int d2 = dx * dx + dy * dy;
          if (d2 < best_d2 && d2 < 96 * 96) {
            best_d2 = d2;
            pick = (int)i;
          }
        }
      }
      if (pick < 0) {
        best_d2 = 0x7fffffff;
        for (unsigned i = 0; i < poam_n; i++) {
          if (mw_paint_mech_oam_tile(poam_t[i]))
            continue;
          if (!mw_paint_midview_xy(poam_x[i], poam_y[i]))
            continue;
          if (!mw_paint_away_from_mech(poam_x[i], poam_y[i], mech_sx, mech_sy,
                                       have_mech))
            continue;
          const int dx = poam_x[i] - aim_sx;
          const int dy = poam_y[i] - aim_sy;
          int d2 = dx * dx + dy * dy;
          if (poam_s[i])
            d2 -= 2000; /* prefer sticky prop tiles */
          if (d2 < best_d2) {
            best_d2 = d2;
            pick = (int)i;
          }
        }
      }
      /* Histogram: non-mech + away-from-mech mid-view only. */
      for (unsigned i = 0; i < poam_n; i++) {
        if (mw_paint_mech_oam_tile(poam_t[i]))
          continue;
        if (!mw_paint_midview_xy(poam_x[i], poam_y[i]))
          continue;
        if (!mw_paint_away_from_mech(poam_x[i], poam_y[i], mech_sx, mech_sy,
                                     have_mech))
          continue;
        spr_n++;
        int ti = -1;
        for (int k = 0; k < spr_nt; k++) {
          if (spr_tile_t[k] == (int)poam_t[i]) {
            ti = k;
            break;
          }
        }
        if (ti < 0 && spr_nt < kMwPaintSprTiles) {
          ti = spr_nt++;
          spr_tile_t[ti] = (int)poam_t[i];
          spr_tile_n[ti] = 0;
          spr_tile_sx[ti] = 0;
          spr_tile_sy[ti] = 0;
        }
        if (ti >= 0) {
          spr_tile_n[ti]++;
          spr_tile_sx[ti] += poam_x[i];
          spr_tile_sy[ti] += poam_y[i];
        }
      }
      for (int k = 0; k < spr_nt; k++) {
        if (spr_tile_n[k] > 0) {
          spr_tile_sx[k] /= spr_tile_n[k];
          spr_tile_sy[k] /= spr_tile_n[k];
        }
      }
      if (pick >= 0) {
        spr_ok = 1;
        spr_sx = poam_x[pick];
        spr_sy = poam_y[pick];
        spr_t = poam_t[pick];
        spr_sticky = poam_s[pick] ? 1 : 0;
        if (s_coldump_spr_have && s_coldump_spr_slot == slot) {
          s_lag = 1;
          s_dloc = (int)loc_x - (int)s_coldump_spr_locx;
          s_dsx = spr_sx - s_coldump_spr_sx;
          if (slot == 0 || slot == 1) {
            s_coldump_spr_apan[slot] =
                (int16_t)((int)s_coldump_spr_apan[slot] + s_dloc);
            s_apan = (int)s_coldump_spr_apan[slot];
            if (s_apan <= -8 || s_apan >= 8)
              s_apan_fire = 1;
          }
          /* |dloc|≥3 — at ±2 world/lock/snap all true when dsx≈0. */
          if (s_dloc <= -3 || s_dloc >= 3 || s_apan_fire) {
            s_lock = (s_dsx - s_dloc <= 1 && s_dsx - s_dloc >= -1);
            s_world = (s_dsx + s_dloc <= 1 && s_dsx + s_dloc >= -1);
            s_snap = (s_dsx <= 1 && s_dsx >= -1);
          }
          if (s_apan_fire && (slot == 0 || slot == 1))
            s_coldump_spr_apan[slot] = 0;
        }
      } else if (s_coldump_spr_slot == slot) {
        s_coldump_spr_have = 0;
        if (slot == 0 || slot == 1)
          s_coldump_spr_apan[slot] = 0;
      }
    }

    /* fix: fixed screen XY VRAM + nearest non-mech OAM (mech-independent). */
    typedef struct {
      int sx, sy;
      unsigned t;
      int weak, same_t, vstick, scroll;
    } MwPaintFixV;
    MwPaintFixV fixv[kMwPaintFixN];
    int nfixv = 0;
    int fix_dloc = 0, fix_lag = 0, fix_apan = 0, fix_afire = 0;
    int fix_oam_ok = 0, fix_oam_sx = 0, fix_oam_sy = 0, fix_oam_s = 0;
    unsigned fix_oam_t = 0;
    int fo_dsx = 0, fo_lock = 0, fo_world = 0, fo_snap = 0, fo_lag = 0;
    static const int kFixXY[kMwPaintFixN][2] = {
        {128, 100}, {128, 80}, {160, 100}, {96, 100}};
    if (g_ppu) {
      fix_lag = (s_coldump_fix_have && s_coldump_fix_slot == slot) ? 1 : 0;
      if (fix_lag)
        fix_dloc = (int)loc_x - (int)s_coldump_fix_locx;
      if ((slot == 0 || slot == 1) && fix_lag) {
        s_coldump_fix_apan[slot] =
            (int16_t)((int)s_coldump_fix_apan[slot] + fix_dloc);
        fix_apan = (int)s_coldump_fix_apan[slot];
        if (fix_apan <= -8 || fix_apan >= 8)
          fix_afire = 1;
      }
      for (int i = 0; i < kMwPaintFixN; i++) {
        const int sx = kFixXY[i][0];
        const int sy = kFixXY[i][1];
        const uint16_t t = mw_bg1_vram_at_world(
            (uint16_t)((int)loc_x + sx), (uint16_t)((int)loc_y + sy), loc_x,
            loc_y);
        const int weak = mw_bg1_tile_weak(t) ? 1 : 0;
        int same_t = 0, vstick = 0, scroll = 0;
        if (fix_lag) {
          same_t = (t == s_coldump_fix_vt[i]) ? 1 : 0;
          if ((fix_dloc <= -3 || fix_dloc >= 3 || fix_afire) && !weak) {
            if (same_t)
              vstick = 1;
            else
              scroll = 1;
          }
        }
        fixv[nfixv].sx = sx;
        fixv[nfixv].sy = sy;
        fixv[nfixv].t = (unsigned)t;
        fixv[nfixv].weak = weak;
        fixv[nfixv].same_t = same_t;
        fixv[nfixv].vstick = vstick;
        fixv[nfixv].scroll = scroll;
        nfixv++;
      }
      /* Nearest non-mech OAM to screen center (128,100). */
      {
        int best_d2 = 0x7fffffff;
        int pick = -1;
        for (unsigned i = 0; i < poam_n; i++) {
          if (mw_paint_mech_oam_tile(poam_t[i]))
            continue;
          if (!mw_paint_midview_xy(poam_x[i], poam_y[i]))
            continue;
          if (!mw_paint_away_from_mech(poam_x[i], poam_y[i], mech_sx, mech_sy,
                                       have_mech))
            continue;
          const int dx = poam_x[i] - 128;
          const int dy = poam_y[i] - 100;
          const int d2 = dx * dx + dy * dy;
          if (d2 < best_d2 && d2 < 100 * 100) {
            best_d2 = d2;
            pick = (int)i;
          }
        }
        if (pick >= 0) {
          fix_oam_ok = 1;
          fix_oam_sx = poam_x[pick];
          fix_oam_sy = poam_y[pick];
          fix_oam_t = poam_t[pick];
          fix_oam_s = poam_s[pick] ? 1 : 0;
          if (s_coldump_fix_oam_have && s_coldump_fix_slot == slot &&
              s_coldump_fix_oam_t == fix_oam_t) {
            fo_lag = 1;
            fo_dsx = fix_oam_sx - s_coldump_fix_oam_sx;
            if (fix_dloc <= -3 || fix_dloc >= 3 || fix_afire) {
              fo_lock = (fo_dsx - fix_dloc <= 1 && fo_dsx - fix_dloc >= -1);
              fo_world = (fo_dsx + fix_dloc <= 1 && fo_dsx + fix_dloc >= -1);
              fo_snap = (fo_dsx <= 1 && fo_dsx >= -1);
            }
          }
        }
      }
      if (fix_afire && (slot == 0 || slot == 1))
        s_coldump_fix_apan[slot] = 0;
    }

    /*
     * mid: BG1 cand in mid-view, screen-sticky tracked. Prefer large runs
     * near screen center / prior mid — NOT nearest-to-feet (band false hit).
     */
    int mid = -1;
    int m_dloc = 0, m_dwx = 0, m_dsx = 0, m_lock = 0, m_world = 0, m_snap = 0,
        m_lag = 0, m_apan = 0, m_apan_fire = 0;
    if (ncand > 0) {
      if (s_coldump_mid_have && s_coldump_mid_slot == slot) {
        int best_d2 = 0x7fffffff;
        for (int i = 0; i < ncand; i++) {
          if (!mw_paint_midview_xy(cands[i].sx, cands[i].sy))
            continue;
          const int dx = cands[i].sx - s_coldump_mid_sx;
          const int dy = cands[i].sy - s_coldump_mid_sy;
          const int d2 = dx * dx + dy * dy;
          if (d2 < best_d2 && d2 < 80 * 80) {
            best_d2 = d2;
            mid = i;
          }
        }
      }
      if (mid < 0) {
        int best_score = -0x7fffffff;
        for (int i = 0; i < ncand; i++) {
          if (!mw_paint_midview_xy(cands[i].sx, cands[i].sy))
            continue;
          const int dx = cands[i].sx - 128;
          const int dy = cands[i].sy - 100;
          int score = cands[i].n * 30 - (dx * dx + dy * dy) / 64;
          if (cands[i].from_vram)
            score += 10;
          if (score > best_score) {
            best_score = score;
            mid = i;
          }
        }
      }
      if (mid >= 0 && s_coldump_mid_have && s_coldump_mid_slot == slot) {
        m_lag = 1;
        m_dloc = (int)loc_x - (int)s_coldump_mid_locx;
        m_dwx = cands[mid].wx - s_coldump_mid_wx;
        m_dsx = cands[mid].sx - s_coldump_mid_sx;
        if (slot == 0 || slot == 1) {
          s_coldump_mid_apan[slot] =
              (int16_t)((int)s_coldump_mid_apan[slot] + m_dloc);
          m_apan = (int)s_coldump_mid_apan[slot];
          if (m_apan <= -8 || m_apan >= 8)
            m_apan_fire = 1;
        }
        if (m_dloc <= -3 || m_dloc >= 3 || m_apan_fire) {
          m_lock = (m_dwx - m_dloc <= 1 && m_dwx - m_dloc >= -1);
          m_world = (m_dwx <= 1 && m_dwx >= -1);
          m_snap = (m_dsx <= 1 && m_dsx >= -1);
        }
        if (m_apan_fire && (slot == 0 || slot == 1))
          s_coldump_mid_apan[slot] = 0;
      }
    }

    fprintf(s_coldump_fp,
            ",\"paint\":{\"vram_solid\":%d,\"f7_solid\":%d,\"n\":%d,"
            "\"mech_sx\":%d,\"mech_sy\":%d,\"focus\":",
            vram_solid, f7_solid, ncand, have_mech ? mech_sx : 0,
            have_mech ? mech_sy : 0);
    if (focus < 0) {
      fprintf(s_coldump_fp, "{\"ok\":0}");
      s_coldump_paint_have = 0;
    } else {
      fprintf(s_coldump_fp,
              "{\"ok\":1,\"src\":\"%s\",\"n\":%d,\"wx\":%d,\"wy\":%d,"
              "\"sx\":%d,\"sy\":%d,\"west\":%d,\"east\":%d,\"t\":%u,"
              "\"score\":%d,\"lag\":{\"ok\":%d,\"dloc\":%d,\"d_wx\":%d,"
              "\"d_sx\":%d,\"lock\":%d,\"world\":%d,\"snap\":%d}}",
              cands[focus].from_vram ? "vram" : "7f", cands[focus].n,
              cands[focus].wx, cands[focus].wy, cands[focus].sx,
              cands[focus].sy, cands[focus].west, cands[focus].east,
              cands[focus].t, cands[focus].score, lag_ok, dloc, d_wx, d_sx,
              lock, world, snap);
      s_coldump_paint_slot = slot;
      s_coldump_paint_wx = cands[focus].wx;
      s_coldump_paint_wy = cands[focus].wy;
      s_coldump_paint_locx = loc_x;
      s_coldump_paint_have = 1;
    }

    fprintf(s_coldump_fp, ",\"vis\":");
    if (vis < 0) {
      fprintf(s_coldump_fp, "{\"ok\":0}");
      if (s_coldump_vis_slot == slot)
        s_coldump_vis_have = 0;
    } else {
      fprintf(s_coldump_fp,
              "{\"ok\":1,\"src\":\"%s\",\"n\":%d,\"wx\":%d,\"wy\":%d,"
              "\"sx\":%d,\"sy\":%d,\"west\":%d,\"east\":%d,\"t\":%u,"
              "\"ds_mech\":%d,\"lag\":{\"ok\":%d,\"dloc\":%d,\"d_wx\":%d,"
              "\"d_sx\":%d,\"lock\":%d,\"world\":%d,\"snap\":%d}}",
              cands[vis].from_vram ? "vram" : "7f", cands[vis].n,
              cands[vis].wx, cands[vis].wy, cands[vis].sx, cands[vis].sy,
              cands[vis].west, cands[vis].east, cands[vis].t,
              (cands[vis].sx - vis_tx) * (cands[vis].sx - vis_tx) +
                  (cands[vis].sy - vis_ty) * (cands[vis].sy - vis_ty),
              v_lag, v_dloc, v_dwx, v_dsx, v_lock, v_world, v_snap);
      s_coldump_vis_slot = slot;
      s_coldump_vis_sx = cands[vis].sx;
      s_coldump_vis_sy = cands[vis].sy;
      s_coldump_vis_wx = cands[vis].wx;
      s_coldump_vis_wy = cands[vis].wy;
      s_coldump_vis_locx = loc_x;
      s_coldump_vis_have = 1;
    }

    fprintf(s_coldump_fp,
            ",\"scr\":{\"lag\":%d,\"dloc\":%d,\"apan\":%d,\"afire\":%d,\"p\":[",
            scr_lag, scr_dloc, scr_apan, scr_apan_fire);
    for (int i = 0; i < nscr; i++) {
      fprintf(s_coldump_fp,
              "%s{\"sx\":%d,\"sy\":%d,\"t\":%u,\"weak\":%d,\"same\":%d,"
              "\"vstick\":%d,\"vstick_a\":%d,\"scroll\":%d}",
              i ? "," : "", scr[i].sx, scr[i].sy, scr[i].t, scr[i].weak,
              scr[i].same_t, scr[i].view_stick, scr[i].vstick_a,
              scr[i].scroll);
      if (have_mech && g_ppu)
        s_coldump_scr_t[i] = (uint16_t)scr[i].t;
    }
    fprintf(s_coldump_fp, "]}");
    if (nscr > 0) {
      s_coldump_scr_slot = slot;
      s_coldump_scr_locx = loc_x;
      s_coldump_scr_have = 1;
    }

    fprintf(s_coldump_fp,
            ",\"band\":{\"ok\":%d,\"sx\":%d,\"sy\":%d,\"t\":%u,\"d2\":%d}",
            band_ok, band_sx, band_sy, band_t, band_d2);

    fprintf(s_coldump_fp, ",\"hzd\":");
    if (!hzd_ok) {
      fprintf(s_coldump_fp, "{\"ok\":0}");
    } else {
      fprintf(s_coldump_fp,
              "{\"ok\":1,\"sx\":%d,\"sy\":%d,\"t\":%u,\"s\":%d,"
              "\"lag\":{\"ok\":%d,\"dloc\":%d,\"d_sx\":%d,\"lock\":%d,"
              "\"world\":%d,\"snap\":%d,\"apan\":%d,\"afire\":%d}}",
              hzd_sx, hzd_sy, hzd_t, hzd_sticky, h_lag, h_dloc, h_dsx, h_lock,
              h_world, h_snap, h_apan, h_apan_fire);
      s_coldump_hzd_slot = slot;
      s_coldump_hzd_sx = hzd_sx;
      s_coldump_hzd_sy = hzd_sy;
      s_coldump_hzd_t = hzd_t;
      s_coldump_hzd_locx = loc_x;
      s_coldump_hzd_have = 1;
    }

    fprintf(s_coldump_fp, ",\"spr\":");
    if (!spr_ok) {
      fprintf(s_coldump_fp, "{\"ok\":0,\"n\":%d,\"tiles\":[", spr_n);
      for (int k = 0; k < spr_nt; k++) {
        fprintf(s_coldump_fp, "%s{\"t\":%d,\"n\":%d,\"sx\":%d,\"sy\":%d}",
                k ? "," : "", spr_tile_t[k], spr_tile_n[k], spr_tile_sx[k],
                spr_tile_sy[k]);
      }
      fprintf(s_coldump_fp, "]}");
    } else {
      fprintf(s_coldump_fp,
              "{\"ok\":1,\"n\":%d,\"sx\":%d,\"sy\":%d,\"t\":%u,\"s\":%d,"
              "\"lag\":{\"ok\":%d,\"dloc\":%d,\"d_sx\":%d,\"lock\":%d,"
              "\"world\":%d,\"snap\":%d,\"apan\":%d,\"afire\":%d},\"tiles\":[",
              spr_n, spr_sx, spr_sy, spr_t, spr_sticky, s_lag, s_dloc, s_dsx,
              s_lock, s_world, s_snap, s_apan, s_apan_fire);
      for (int k = 0; k < spr_nt; k++) {
        fprintf(s_coldump_fp, "%s{\"t\":%d,\"n\":%d,\"sx\":%d,\"sy\":%d}",
                k ? "," : "", spr_tile_t[k], spr_tile_n[k], spr_tile_sx[k],
                spr_tile_sy[k]);
      }
      fprintf(s_coldump_fp, "]}");
      s_coldump_spr_slot = slot;
      s_coldump_spr_sx = spr_sx;
      s_coldump_spr_sy = spr_sy;
      s_coldump_spr_t = spr_t;
      s_coldump_spr_locx = loc_x;
      s_coldump_spr_have = 1;
    }

    fprintf(s_coldump_fp, ",\"mid\":");
    if (mid < 0) {
      fprintf(s_coldump_fp, "{\"ok\":0}");
      if (s_coldump_mid_slot == slot) {
        s_coldump_mid_have = 0;
        if (slot == 0 || slot == 1)
          s_coldump_mid_apan[slot] = 0;
      }
    } else {
      fprintf(s_coldump_fp,
              "{\"ok\":1,\"src\":\"%s\",\"n\":%d,\"wx\":%d,\"wy\":%d,"
              "\"sx\":%d,\"sy\":%d,\"west\":%d,\"east\":%d,\"t\":%u,"
              "\"lag\":{\"ok\":%d,\"dloc\":%d,\"d_wx\":%d,\"d_sx\":%d,"
              "\"lock\":%d,\"world\":%d,\"snap\":%d,\"apan\":%d,\"afire\":%d}}",
              cands[mid].from_vram ? "vram" : "7f", cands[mid].n,
              cands[mid].wx, cands[mid].wy, cands[mid].sx, cands[mid].sy,
              cands[mid].west, cands[mid].east, cands[mid].t, m_lag, m_dloc,
              m_dwx, m_dsx, m_lock, m_world, m_snap, m_apan, m_apan_fire);
      s_coldump_mid_slot = slot;
      s_coldump_mid_sx = cands[mid].sx;
      s_coldump_mid_sy = cands[mid].sy;
      s_coldump_mid_wx = cands[mid].wx;
      s_coldump_mid_wy = cands[mid].wy;
      s_coldump_mid_locx = loc_x;
      s_coldump_mid_have = 1;
      if (slot == 0 || slot == 1) {
        s_stand_latch_mid_wx[slot] = cands[mid].wx;
        s_stand_latch_mid_wy[slot] = cands[mid].wy;
        s_stand_latch_mid_have[slot] = 1;
      }
    }

    fprintf(s_coldump_fp,
            ",\"fix\":{\"lag\":%d,\"dloc\":%d,\"apan\":%d,\"afire\":%d,\"v\":[",
            fix_lag, fix_dloc, fix_apan, fix_afire);
    for (int i = 0; i < nfixv; i++) {
      fprintf(s_coldump_fp,
              "%s{\"sx\":%d,\"sy\":%d,\"t\":%u,\"weak\":%d,\"same\":%d,"
              "\"vstick\":%d,\"scroll\":%d}",
              i ? "," : "", fixv[i].sx, fixv[i].sy, fixv[i].t, fixv[i].weak,
              fixv[i].same_t, fixv[i].vstick, fixv[i].scroll);
      s_coldump_fix_vt[i] = (uint16_t)fixv[i].t;
    }
    fprintf(s_coldump_fp, "],\"oam\":");
    if (!fix_oam_ok) {
      fprintf(s_coldump_fp, "{\"ok\":0}");
      s_coldump_fix_oam_have = 0;
    } else {
      fprintf(s_coldump_fp,
              "{\"ok\":1,\"sx\":%d,\"sy\":%d,\"t\":%u,\"s\":%d,"
              "\"lag\":{\"ok\":%d,\"d_sx\":%d,\"lock\":%d,\"world\":%d,"
              "\"snap\":%d}}",
              fix_oam_sx, fix_oam_sy, fix_oam_t, fix_oam_s, fo_lag, fo_dsx,
              fo_lock, fo_world, fo_snap);
      s_coldump_fix_oam_sx = fix_oam_sx;
      s_coldump_fix_oam_sy = fix_oam_sy;
      s_coldump_fix_oam_t = fix_oam_t;
      s_coldump_fix_oam_have = 1;
    }
    fprintf(s_coldump_fp, "}");
    if (nfixv > 0) {
      s_coldump_fix_slot = slot;
      s_coldump_fix_locx = loc_x;
      s_coldump_fix_have = 1;
    }

    fprintf(s_coldump_fp, ",\"cands\":[");
    for (int i = 0; i < ncand; i++) {
      fprintf(s_coldump_fp,
              "%s{\"src\":\"%s\",\"n\":%d,\"wx\":%d,\"wy\":%d,\"sx\":%d,"
              "\"sy\":%d,\"west\":%d,\"east\":%d,\"t\":%u,\"score\":%d}",
              i ? "," : "", cands[i].from_vram ? "vram" : "7f", cands[i].n,
              cands[i].wx, cands[i].wy, cands[i].sx, cands[i].sy,
              cands[i].west, cands[i].east, cands[i].t, cands[i].score);
    }
    /* On-screen OAM — paint may be sprites; do not assume feet Y. */
    fprintf(s_coldump_fp, "],\"oam\":[");
    int oam_out = 0;
    for (unsigned i = 0; i < poam_n && oam_out < kPaintMaxOam; i++) {
      if (poam_x[i] < -64 || poam_x[i] >= 360 || poam_y[i] < -64 ||
          poam_y[i] >= 240)
        continue;
      fprintf(s_coldump_fp, "%s{\"x\":%d,\"y\":%d,\"t\":%u,\"s\":%u}",
              oam_out ? "," : "", poam_x[i], poam_y[i], poam_t[i],
              (unsigned)poam_s[i]);
      oam_out++;
    }
    fprintf(s_coldump_fp, "],\"oam_n\":%d}", oam_out);
  }

  uint16_t bg2_fresh = 0;
  for (unsigned r = 0; r < (unsigned)kMwBg2RomRows; r++) {
    if (mw_bg2_rom_row_fresh(r))
      bg2_fresh |= (uint16_t)(1u << r);
  }
  fprintf(s_coldump_fp,
          ",\"bg2\":{\"idle\":%d,\"mask\":%u,\"words\":%u,\"fresh\":%u,"
          "\"v1\":%u,\"pv1\":%u,\"own\":%u,\"rst\":%d,\"rows\":[",
          s_bg2_rom_idle, (unsigned)s_bg2_rom_valid_mask,
          (unsigned)s_bg2_rom_words_mask, (unsigned)bg2_fresh,
          (unsigned)(s_nmi_latched ? (slot == 1 ? s_nmi_wram_v1_p2
                                               : s_nmi_wram_v1)
                                   : mw_wram16(slot == 1 ? 0x1E64u
                                                         : 0x1E60u)),
          (unsigned)s_coldump_bg2_present_v1,
          (unsigned)s_coldump_bg2_own_mask, s_coldump_bg2_restamp);
  int bg2_row_out = 0;
  for (unsigned r = 0; r < (unsigned)kMwBg2RomRows; r++) {
    if (!(s_bg2_rom_valid_mask & (uint16_t)(1u << r)))
      continue;
    const int age = (s_bg2_rom_row_frame[r] > 0)
                        ? (snes_frame_counter - s_bg2_rom_row_frame[r])
                        : -1;
    fprintf(s_coldump_fp,
            "%s{\"r\":%u,\"bank\":%u,\"addr\":%u,\"age\":%d,\"fresh\":%d,"
            "\"own\":%d}",
            bg2_row_out ? "," : "", r, (unsigned)s_bg2_rom_bank[r],
            (unsigned)s_bg2_rom_addr[r], age, mw_bg2_rom_row_fresh(r) ? 1 : 0,
            (int)s_bg2_rom_row_owner[r]);
    bg2_row_out++;
  }
  /* Close bg2.rows + bg2 object before sibling keys (stand_bg1 / meta7e). */
  fprintf(s_coldump_fp, "]}");
  /* Emit for the presented slot even on src=miss (n_cells==0). */
  if (s_stand_bg1.mode[0] && s_stand_bg1.slot == slot) {
    fprintf(s_coldump_fp,
            ",\"stand_bg1\":{\"mode\":\"%s\",\"src\":\"%s\",\"slot\":%d,"
            "\"ok\":%d,\"mx\":%d,\"my\":%d,\"ox\":%d,\"oy\":%d,"
            "\"solid7f\":%d,\"solidv\":%d,\"touched\":%d,\"cells\":[",
            s_stand_bg1.mode, s_stand_bg1.src[0] ? s_stand_bg1.src : "?",
            s_stand_bg1.slot, s_stand_bg1.n_cells > 0 ? 1 : 0, s_stand_bg1.mx,
            s_stand_bg1.my, s_stand_bg1.ox, s_stand_bg1.oy,
            s_stand_bg1.n_solid7f, s_stand_bg1.n_solidv,
            s_stand_bg1.n_touched);
    for (int i = 0; i < s_stand_bg1.n_cells; i++) {
      fprintf(s_coldump_fp,
              "%s{\"dx\":%d,\"dy\":%d,\"t7f\":%u,\"tv0\":%u,\"tv1\":%u}",
              i ? "," : "", (int)s_stand_bg1.dx[i], (int)s_stand_bg1.dy[i],
              (unsigned)s_stand_bg1.t7f[i], (unsigned)s_stand_bg1.tv0[i],
              (unsigned)s_stand_bg1.tv1[i]);
    }
    fprintf(s_coldump_fp, "]}");
  } else {
    fprintf(s_coldump_fp, ",\"stand_bg1\":{\"mode\":\"off\",\"ok\":0}");
  }
  fprintf(s_coldump_fp, ",\"meta7e_max\":%u,\"lines\":%u}\n",
          s_prop_stat_meta7e_max, s_coldump_lines);
}

static void mw_elev_dump(void) {
  static int armed = -1;
  if (armed < 0) {
    const char *e = getenv("SNESRECOMP_MW_ELEV");
    armed = (e && e[0] && e[0] != '0') ? 1 : 0;
    if (armed)
      fprintf(stderr,
              "[mw_elev] armed — dump $1E14 objects + BG1/BG2/OAM layer "
              "signals (~2 Hz in dual H2H). Compare offline vs netplay.\n");
  }
  /* Present-time coldump runs inside LocalFull. Always fill gaps offline /
   * half-crop: emit any slot not already dumped this frame (both peers). */
  if (MwIsDualViewport()) {
    mw_coldump_tick(0);
    mw_coldump_tick(1);
  } else {
    mw_coldump_tick(0);
  }
  if (!armed)
    return;

  extern int snes_frame_counter;
  static int last_f = -1;
  static unsigned last_tile7f, last_helper;
  /* ~2 Hz once dual/stage is up; every 30 frames otherwise for boot. */
  const int period = MwIsDualViewport() ? 30 : 60;
  if (snes_frame_counter < 120 || (snes_frame_counter % period) != 0)
    return;
  if (snes_frame_counter == last_f)
    return;
  last_f = snes_frame_counter;

  const uint16_t cam0x = s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16);
  const uint16_t cam0y = s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18);
  const uint16_t cam1x = s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1A);
  const uint16_t cam1y = s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1C);
  const uint16_t src1 = s_nmi_latched ? s_nmi_src_bg1 : mw_wram16(0x1E36);
  const uint16_t src2 = s_nmi_latched ? s_nmi_src_bg2 : mw_wram16(0x1E38);
  const unsigned d_tile = s_tile7f_hits - last_tile7f;
  const unsigned d_help = s_tile_helper_hits - last_helper;
  last_tile7f = s_tile7f_hits;
  last_helper = s_tile_helper_hits;

  uint16_t bg2_banks = 0;
  for (unsigned r = 0; r < (unsigned)kMwBg2RomRows; r++) {
    if (s_bg2_rom_valid_mask & (uint16_t)(1u << r))
      bg2_banks |= (uint16_t)(1u << (s_bg2_rom_bank[r] & 15u));
  }

  unsigned oam_on = 0, oam_mid = 0;
  for (unsigned s = 0; s < 128u; s++) {
    const unsigned o = 0x14C4u + s * 4u;
    const uint8_t y = g_ram[o + 1u];
    if (y >= 0xE0u)
      continue;
    oam_on++;
    if (y >= 0x40u && y < 0xC0u)
      oam_mid++;
  }

  /* Prefer present-time snapshot — live buffers are often cleared at NMI. */
  const unsigned prop_lo0 = s_elev_cap_lo0;
  const unsigned prop_lo1 = s_elev_cap_lo1;

  fprintf(stderr,
          "[mw_elev] f=%d dual=%d net=%d vw=%d syw=%d ff=%d gm=%02X mode=%u "
          "cam0=%04X/%04X cam1=%04X/%04X src=%04X/%04X "
          "bg2idle=%d rom_mask=%04X bg2bank_lo=%04X "
          "hs=%04X/%04X vs=%04X/%04X "
          "tile7f=%u(+%u) helper=%u(+%u) oam_on/mid=%u/%u "
          "prop_lo=%u+%u cap=%u+%u "
          "prop_stat=latch:%u list:%u commit:%u conv:%u meta7e:%u "
          "bg1src=%d bg_dy=%d list=$%04X 1EB2=%04X\n",
          snes_frame_counter, MwIsDualViewport() ? 1 : 0,
          snes_netplay_active() ? 1 : 0, mw_h2h_vert_widen_armed() ? 1 : 0,
          mw_h2h_spawn_y_widen_armed() ? 1 : 0,
          MwH2hFullFrameLocalArmed() ? 1 : 0, (unsigned)g_ram[0x10],
          g_ppu ? (unsigned)(g_ppu->bgmode & 7) : 0u, (unsigned)cam0x,
          (unsigned)cam0y, (unsigned)cam1x, (unsigned)cam1y, (unsigned)src1,
          (unsigned)src2, s_bg2_rom_idle, (unsigned)s_bg2_rom_valid_mask,
          (unsigned)bg2_banks,
          (unsigned)(s_nmi_latched ? s_nmi_wram_h0 : mw_wram16(0x1E2E)),
          (unsigned)(s_nmi_latched ? s_nmi_wram_h0_p2 : mw_wram16(0x1E32)),
          (unsigned)(s_nmi_latched ? s_nmi_wram_v0 : mw_wram16(0x1E5E)),
          (unsigned)(s_nmi_latched ? s_nmi_wram_v0_p2 : mw_wram16(0x1E62)),
          s_tile7f_hits, d_tile, s_tile_helper_hits, d_help, oam_on, oam_mid,
          prop_lo0, prop_lo1, s_elev_cap_n0, s_elev_cap_n1, s_prop_stat_latch,
          s_prop_stat_list_rec, s_prop_stat_commit, s_prop_stat_convert,
          s_prop_stat_meta7e_max, s_elev_bg1_src_path, s_elev_prop_bg_dy,
          (unsigned)mw_wram16(0x1E14), (unsigned)mw_wram16(0x1EB2));

  uint16_t idx = mw_wram16(0x1E14);
  unsigned n = 0, n_active = 0;
  for (int guard = 0; guard < 64 && idx; guard++) {
    const uint16_t fl = mw_wram16(idx);
    const uint16_t wx = mw_wram16((uint16_t)(idx + 2u));
    const uint16_t wy = mw_wram16((uint16_t)(idx + 4u)); /* heuristic */
    const uint16_t w6 = mw_wram16((uint16_t)(idx + 6u));
    const uint16_t w8 = mw_wram16((uint16_t)(idx + 8u));
    const uint16_t wa = mw_wram16((uint16_t)(idx + 0xAu));
    const uint16_t wc = mw_wram16((uint16_t)(idx + 0xCu));
    const uint16_t we = mw_wram16((uint16_t)(idx + 0xEu));
    const uint16_t w10 = mw_wram16((uint16_t)(idx + 0x10u));
    const uint16_t w12 = mw_wram16((uint16_t)(idx + 0x12u));
    const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
    const int active = (fl & 0x8000u) != 0;
    if (active)
      n_active++;

    const int16_t sx0 = (int16_t)(wx - cam0x);
    const int16_t sy0 = (int16_t)(wy - cam0y);
    const int16_t sx1 = (int16_t)(wx - cam1x);
    const int16_t sy1 = (int16_t)(wy - cam1y);

    /* Flag candidates near either camera's playfield. */
    const char *tag = "";
    if (active && sx0 >= -64 && sx0 < 320 && sy0 >= -64 && sy0 < 288)
      tag = " near0";
    else if (active && sx1 >= -64 && sx1 < 320 && sy1 >= -64 && sy1 < 288)
      tag = " near1";

    fprintf(stderr,
            "[mw_elev] obj[%u] @$%04X fl=%04X%s wx=%04X wy?=%04X "
            "sx0=%d sy0?=%d sx1=%d sy1?=%d "
            "+6=%04X +8=%04X +A=%04X +C=%04X +E=%04X +10=%04X +12=%04X "
            "next=%04X\n",
            n, (unsigned)idx, (unsigned)fl, active ? " act" : "",
            (unsigned)wx, (unsigned)wy, (int)sx0, (int)sy0, (int)sx1,
            (int)sy1, (unsigned)w6, (unsigned)w8, (unsigned)wa, (unsigned)wc,
            (unsigned)we, (unsigned)w10, (unsigned)w12, (unsigned)next);

    /* Raw 0x20 bytes so type/bank pointers are visible even if offsets shift. */
    fprintf(stderr, "[mw_elev] obj[%u] raw:", n);
    for (unsigned b = 0; b < 0x20u; b++)
      fprintf(stderr, " %02X", (unsigned)g_ram[(uint16_t)(idx + b)]);
    fprintf(stderr, "%s\n", tag);

    n++;
    if (next == 0 || next == idx)
      break;
    idx = next;
  }
  fprintf(stderr, "[mw_elev] list_n=%u active=%u (wy? = +$04 heuristic)\n", n,
          n_active);

  /* BG2 map row 0 sample — elevators-as-BG2 would show non-void here. */
  if (g_ppu) {
    const uint16_t base2 = (uint16_t)PPU_bgTilemapAdr(g_ppu, 1);
    const uint16_t base1 = (uint16_t)PPU_bgTilemapAdr(g_ppu, 0);
    fprintf(stderr, "[mw_elev] vram bg1base=%04X bg2base=%04X xsc=%u/%u big=%d/%d "
                    "row0_bg2:",
            (unsigned)base1, (unsigned)base2,
            g_ppu ? (unsigned)(g_ppu->bgXsc[0] & 3) : 0u,
            g_ppu ? (unsigned)(g_ppu->bgXsc[1] & 3) : 0u,
            g_ppu ? PPU_bigTiles(g_ppu, 0) : 0,
            g_ppu ? PPU_bigTiles(g_ppu, 1) : 0);
    for (int c = 0; c < 16; c++)
      fprintf(stderr, " %04X",
              (unsigned)g_ppu->vram[(uint16_t)(base2 + c) & 0x7fffu]);
    fprintf(stderr, "\n[mw_elev] vram row0_bg1:");
    for (int c = 0; c < 16; c++)
      fprintf(stderr, " %04X",
              (unsigned)g_ppu->vram[(uint16_t)(base1 + c) & 0x7fffu]);
    fprintf(stderr, "\n");
  }
}

/* Staging OAM cam tag: 0=P1 cam path, 1=P2 cam path, 0xFF=unknown. */
static uint8_t s_oam_cam_tag[128];
static int s_oam_draw_cam;
/* s_oam_draw_cam: which capture buffer. Source of truth is the drawer ADC of
 * cam screen regs — $86/$88 → cam0, $8A/$8C → cam1 — because those are the
 * values baked into staging X/Y. Y-CMP / ADC #$78 tags alone could leave a
 * $8A tile committed as cam0 (P1 saw platforms track P2's camera).
 * s_oam_saw_adc78: one-shot "ROM just ADCed Y" for bias undo; cleared after
 * Y STA apply so later tiles that skip ADC still use raw A. */
static int s_oam_saw_adc78;
static int s_oam_cam_tag_init;

/* Per-camera compact OAM captured during dual draw (present-only).
 * s_cam_sy holds pre-bias signed screen Y so bottom sprites with sy≥$78
 * (biased OAM Y would hit $F0) still present correctly after −bias/+shift.
 * s_cam_sx holds signed screen X — needed because 9-bit OAM X in
 * [256, 256+extra) is ambiguous (right margin vs left wrap); decoding via
 * mw_oam_screen_x_from_9bit alone mistreats left-offscreen mechs as right
 * margin (P2 saw P1 ghost on the right while approaching from the left). */
static uint8_t s_cam_spr[2][128][4];
static int16_t s_cam_sx[2][128];
static int16_t s_cam_sy[2][128];
static uint8_t s_cam_high[2][32];
/* s_cam_local_only / s_cam_n declared above (elev dump). 1 = stage prop. */
/* World pixel of each captured tile; stage props present as world−local_cam. */
static uint16_t s_cam_wx[2][128];
static uint16_t s_cam_wy[2][128];
/* Stage-prop commit: object + meta offsets so present rebuilds screen from
 * live +$02/+$04 (avoids capture lag snap). Dual drawer emits into both cams;
 * purge-before-add keeps a single tile set in the home buffer. */
static uint16_t s_cam_prop_obj[2][128];
static int16_t s_cam_prop_meta_ox[2][128];
static int16_t s_cam_prop_meta_oy[2][128];
static int s_cam_capture_frame;
/* Set in STA $14C5 hooks; consumed at tile/attr commit. */
static int16_t s_pending_sy;
static int s_pending_sy_valid;

static int mw_i32_abs(int32_t v) { return v < 0 ? (int)(-v) : (int)v; }

static void mw_oam_cam_tag_ensure_init(void) {
  if (!s_oam_cam_tag_init) {
    memset(s_oam_cam_tag, 0xff, sizeof(s_oam_cam_tag));
    s_oam_cam_tag_init = 1;
  }
}

static void mw_cam_oam_reset(void) {
  memset(s_cam_spr, 0, sizeof(s_cam_spr));
  memset(s_cam_sx, 0, sizeof(s_cam_sx));
  memset(s_cam_sy, 0, sizeof(s_cam_sy));
  memset(s_cam_high, 0, sizeof(s_cam_high));
  memset(s_cam_local_only, 0, sizeof(s_cam_local_only));
  memset(s_cam_shared_item, 0, sizeof(s_cam_shared_item));
  memset(s_cam_prop_owner, -1, sizeof(s_cam_prop_owner));
  memset(s_cam_wx, 0, sizeof(s_cam_wx));
  memset(s_cam_wy, 0, sizeof(s_cam_wy));
  memset(s_cam_prop_obj, 0, sizeof(s_cam_prop_obj));
  memset(s_cam_prop_meta_ox, 0, sizeof(s_cam_prop_meta_ox));
  memset(s_cam_prop_meta_oy, 0, sizeof(s_cam_prop_meta_oy));
  s_cam_n[0] = s_cam_n[1] = 0;
  for (int c = 0; c < 2; c++)
    for (int s = 0; s < 128; s++)
      s_cam_spr[c][s][1] = 0xf0u;
  s_cam_capture_frame = 0;
  s_pending_stage_prop_local_only = 0;
  s_draw_obj_latched = 0;
  s_prop_purge_arm = 0;
  s_pending_sy_valid = 0;
}

/* Drop prior capture tiles for a stage prop (both cams). Compact in place. */
static void mw_cam_purge_prop_obj(uint16_t prop_obj) {
  if (!prop_obj)
    return;
  for (int c = 0; c < 2; c++) {
    unsigned w = 0;
    for (unsigned i = 0; i < s_cam_n[c]; i++) {
      if (s_cam_local_only[c][i] && s_cam_prop_obj[c][i] == prop_obj)
        continue;
      if (w != i) {
        memcpy(s_cam_spr[c][w], s_cam_spr[c][i], 4);
        s_cam_sx[c][w] = s_cam_sx[c][i];
        s_cam_sy[c][w] = s_cam_sy[c][i];
        s_cam_wx[c][w] = s_cam_wx[c][i];
        s_cam_wy[c][w] = s_cam_wy[c][i];
        s_cam_local_only[c][w] = s_cam_local_only[c][i];
        s_cam_shared_item[c][w] = s_cam_shared_item[c][i];
        s_cam_prop_owner[c][w] = s_cam_prop_owner[c][i];
        s_cam_prop_obj[c][w] = s_cam_prop_obj[c][i];
        s_cam_prop_meta_ox[c][w] = s_cam_prop_meta_ox[c][i];
        s_cam_prop_meta_oy[c][w] = s_cam_prop_meta_oy[c][i];
        {
          const uint8_t h =
              (uint8_t)((s_cam_high[c][i >> 2] >> ((i & 3u) * 2u)) & 3u);
          const unsigned wb = w >> 2;
          const unsigned ws = (w & 3u) * 2u;
          s_cam_high[c][wb] = (uint8_t)(
              (s_cam_high[c][wb] & ~(uint8_t)(3u << ws)) | (uint8_t)(h << ws));
        }
      }
      w++;
    }
    for (unsigned i = w; i < s_cam_n[c]; i++) {
      s_cam_spr[c][i][1] = 0xf0u;
      s_cam_local_only[c][i] = 0;
      s_cam_shared_item[c][i] = 0;
      s_cam_prop_obj[c][i] = 0;
      s_cam_prop_owner[c][i] = (int8_t)-1;
    }
    s_cam_n[c] = w;
  }
}

/* Present-only VRAM word backup so local strip rebuild cannot desync sim.
 * Worst case 32×32 BG1 + 32×32 BG2 (8px tiles, full map wrap). */
enum { kMwVramSaveMax = 2048 };
static struct {
  uint16_t addr;
  uint16_t val;
} s_vram_save[kMwVramSaveMax];
static int s_vram_save_n;

static void mw_vram_save_reset(void) { s_vram_save_n = 0; }

static void mw_vram_save_word(uint16_t addr) {
  addr &= 0x7fffu;
  if (s_vram_save_n >= kMwVramSaveMax)
    return;
  s_vram_save[s_vram_save_n].addr = addr;
  s_vram_save[s_vram_save_n].val = g_ppu->vram[addr];
  s_vram_save_n++;
}

static void mw_vram_restore(void) {
  for (int i = 0; i < s_vram_save_n; i++)
    g_ppu->vram[s_vram_save[i].addr & 0x7fffu] = s_vram_save[i].val;
  s_vram_save_n = 0;
}

/* Defined with stage-prop blank — used while painting native 4:3 cols. */
static int mw_prop_foreign_ink_tile(int wx, int wy, uint16_t tile,
                                    int local_slot, uint16_t c0x, uint16_t c0y,
                                    uint16_t c1x, uint16_t c1y,
                                    uint16_t scroll_x);
static uint16_t mw_bg1_under_at_world(int wx, int wy);

/*
 * Rebuild one layer's streaming strip from $7F into VRAM for present only
 * (matches game DMA: view cols + fine overhang + widescreen pad_right).
 *
 * Full-frame H2H BG1 matches the game stripe uploader (VMADD col 0, 17
 * words from $42B3): paint **view-relative** into VRAM columns starting at
 * 0, while the caller sets BG1HOFS to the fine phase only (cam & 15).
 * World-absolute VRAM + full HOFS fought that DMA model — within a 16px
 * phase the stand looked FOV-stuck, then snapped opposite on coarse step.
 *
 * Half-frame / non-BG1: still key VRAM to scroll>>tile (world-absolute).
 * raw_cam_y = scroll before Y recenter (0 → no sticky walk).
 */
static void mw_present_rebuild_layer_strip(int layer, uint16_t cam_x,
                                          uint16_t cam_y, uint16_t scroll_x,
                                          uint16_t scroll_y, int n_rows,
                                          uint16_t raw_cam_y) {
  if (!g_ppu || n_rows <= 0)
    return;

  const uint16_t pitch = mw_bg1_pitch();
  const int local_slot = s_present_h2h_local_slot;

  /* Prefer scroll (= latched local cam on full-frame) so src == PPU sample. */
  uint16_t src_w = mw_map_src_from_42b3(scroll_x, scroll_y);
  if (!src_w)
    src_w = mw_map_src_from_42b3(cam_x, cam_y);

  uint16_t sticky = (layer == 0) ? s_sticky_src_bg1 : s_sticky_src_bg2;
  if (layer == 0 && (local_slot == 0 || local_slot == 1) &&
      s_sticky_src_bg1_slot[local_slot])
    sticky = s_sticky_src_bg1_slot[local_slot];
  /* Drop sticky from another dual-cam column — same_column false means the
   * last DMA stripe is the peer's stream; using it phase-shifts mover dirt
   * into this peer's top rows. */
  if (layer == 0 && sticky && src_w &&
      !mw_bg1_src_same_column(src_w, sticky, pitch))
    sticky = 0;
  const uint16_t live =
      (layer == 0)
          ? (s_nmi_latched ? s_nmi_src_bg1 : mw_wram16(0x1E36))
          : (s_nmi_latched ? s_nmi_src_bg2 : mw_wram16(0x1E38));
  uint16_t src = src_w ? src_w : (sticky ? sticky : live);
  int src_path = src_w ? 1 : 0;

  /*
   * Full-frame H2H: ONLY local 42B3 walk (scroll/cam). Never sticky/live
   * $1E36 — that is the dual DMA stream and shifts mover brown by ~d01
   * (stripe+ghost jump far left for one frame). Half-frame may Y-adjust.
   */
  if (layer == 0 && s_present_h2h_full_frame) {
    src = src_w;
    src_path = src_w ? 1 : 3;
    if (!src)
      return;
  } else if (layer == 0 && raw_cam_y != 0) {
    const uint16_t live_src = sticky ? sticky : live;
    const int allow_live =
        live_src &&
        (!src_w || mw_bg1_src_same_column(src_w, live_src, pitch));
    if (allow_live) {
      const unsigned sh = PPU_bigTiles(g_ppu, 0) ? 4u : 3u;
      const int tile_px = 1 << (int)sh;
      const int dy = (int)raw_cam_y - (int)scroll_y; /* usually y_bg */
      const int d_tiles = dy / tile_px;
      const int adj = (int)live_src - d_tiles * (int)pitch;
      if (adj > 0 && adj < 0x10000) {
        src = (uint16_t)adj;
        if (!src_w)
          src_path = 4;
      }
    }
  }
  if (layer == 0)
    s_elev_bg1_src_path = src_path;
  if (!src)
    return;

  const unsigned sh = PPU_bigTiles(g_ppu, layer) ? 4u : 3u;
  const int view_cols = 256 >> (int)sh;
  const uint32_t buf_tx0 = (uint32_t)scroll_x >> sh;
  const uint32_t buf_ty0 = (uint32_t)scroll_y >> sh;
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, layer);
  const int x_mask = PPU_bgTilemapWider(g_ppu, layer) ? 63 : 31;
  int pad = (g_ws_active && g_ws_extra > 0) ? mw_ws_tile_pad() : 0;
  if (pad < 0)
    pad = 0;
  /* Native stripe = view + 1 overhang; DMA widen only adds pad_right
   * (VMADD col 0 → pad_left=0). Paint [-pad, view+1+pad) so the left
   * widescreen gutter lands in VRAM / is available to the shadow. */
  const int col_lo = -pad;
  int col_hi = view_cols + 1 + pad;
  if (col_hi > 32)
    col_hi = 32;
  if (n_rows > 32)
    n_rows = 32;
  const int map_period = x_mask + 1;
  /* Full-frame BG1: view-relative X (game DMA col 0); Y stays world row. */
  const int view_rel_x = (layer == 0 && s_present_h2h_full_frame);

  /* Prefer live $7F; snap fills dual-stomped voids/sky (±1 tile column).
   * Do NOT prefer_snap / world-abs-replace solid live when DMA is foreign —
   * that hitch other BG while panning and left the stand FOV-stuck
   * (2026-08-05). Void-only world-abs below is the only 42B3 rescue. */
  const int use_snap =
      (layer == 0 && (local_slot == 0 || local_slot == 1) &&
       s_bg1_snap[local_slot].valid &&
       mw_bg1_src_near_column(src, s_bg1_snap[local_slot].src, pitch, 1) &&
       ((int)s_bg1_snap[local_slot].cam_x - (int)scroll_x <= 24 &&
        (int)s_bg1_snap[local_slot].cam_x - (int)scroll_x >= -24));
  const int tile_px = 1 << (int)sh;

  /* Foreign-ink filter: latched present cams only (never re-read WRAM). */
  const uint16_t filt_c0x =
      s_present_cam_valid
          ? (local_slot == 0 ? s_present_loc_x : s_present_oth_x)
          : (s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16u));
  const uint16_t filt_c0y =
      s_present_cam_valid
          ? (local_slot == 0 ? s_present_loc_y : s_present_oth_y)
          : (s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18u));
  const uint16_t filt_c1x =
      s_present_cam_valid
          ? (local_slot == 1 ? s_present_loc_x : s_present_oth_x)
          : (s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1Au));
  const uint16_t filt_c1y =
      s_present_cam_valid
          ? (local_slot == 1 ? s_present_loc_y : s_present_oth_y)
          : (s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1Cu));

  /* Full-frame: also refresh 2 rows above cam (paint.focus lived at
   * coldump sy≈−37 — above strip top; stale view-cols FOV-snapped). */
  const int row_lo =
      (layer == 0 && s_present_h2h_full_frame) ? -2 : 0;
  for (int row = row_lo; row < n_rows; row++) {
    int map_row = ((int)buf_ty0 + row) & 31;
    if (map_row < 0)
      map_row += 32;
    for (int col = col_lo; col < col_hi; col++) {
      int map_col;
      if (view_rel_x) {
        map_col = col % map_period;
        if (map_col < 0)
          map_col += map_period;
      } else {
        map_col = ((int)buf_tx0 + col) % map_period;
        if (map_col < 0)
          map_col += map_period;
      }
      const int half = (x_mask > 31 && map_col >= 32) ? 0x400 : 0;
      const uint16_t vaddr =
          (uint16_t)(map_base + half + (map_row << 5) + (map_col & 31));
      mw_vram_save_word(vaddr);
      uint16_t t = 0;
      const int byte_off = (int)src + row * (int)pitch + col * 2;
      if (byte_off >= 0 && byte_off + 1 < 0x10000)
        t = mw_read7f16((uint16_t)byte_off);
      if (use_snap) {
        const uint16_t ts =
            mw_bg1_snap_word(local_slot, src, pitch, row, col);
        /* Solid snap only fills void/sky live — never replace solid. */
        if (!mw_bg1_tile_weak(ts) && mw_bg1_tile_weak(t))
          t = ts;
      }
      /* Void-only world-abs: stripe miss that 42B3 still holds. Do not
       * replace solid or sky live (foreign-DMA world-abs hitch). */
      if (layer == 0 && s_present_h2h_full_frame && mw_bg1_tile_void(t) &&
          col >= 0) {
        const int wx = (int)scroll_x + col * tile_px + tile_px / 2;
        const int wy = (int)scroll_y + row * tile_px + tile_px / 2;
        if (wx >= 0 && wy >= 0) {
          const uint16_t woff =
              mw_7f_off_from_world((uint16_t)wx, (uint16_t)wy);
          if (woff) {
            const uint16_t tw = mw_read7f16(woff);
            if (!mw_bg1_tile_weak(tw))
              t = tw;
          }
        }
      }
      if (mw_bg1_tile_void(t)) {
        if (col < 0) {
          /* West gutter still undecoded — leave miss/transparent. */
          continue;
        } else if (s_present_h2h_full_frame) {
          /*
           * Do NOT keep prior VRAM. Dual $7F often has void for this
           * column while the 32-row tilemap still holds the other cam's
           * / prior-scroll brown (P1 low-Y wall → mid-screen band on
           * P2). Sky only when snap also missed.
           */
          t = 0x0200u;
        } else {
          continue; /* non-full-frame: leave prior VRAM */
        }
      }
      /*
       * Native 4:3 cols only: drop foreign ledge fingerprint. West/history
       * cols (col < 0) are a different path — ghosts live in the DMA window.
       */
      if (layer == 0 && s_present_h2h_full_frame && col >= 0 &&
          col < view_cols && !mw_bg1_tile_void(t) && t != 0x0200u) {
        const int wx = (int)scroll_x + col * tile_px + tile_px / 2;
        const int wy = (int)scroll_y + row * tile_px + tile_px / 2;
        if (mw_prop_foreign_ink_tile(wx, wy, t, local_slot, filt_c0x, filt_c0y,
                                     filt_c1x, filt_c1y, scroll_x))
          t = mw_bg1_under_at_world(wx, wy);
      }
      g_ppu->vram[vaddr & 0x7fffu] = t;
    }
  }
}

static int mw_present_strip_rows(int layer) {
  if (!g_ppu)
    return 15;
  const unsigned sh = PPU_bigTiles(g_ppu, layer) ? 4u : 3u;
  const int tile_px = 1 << (int)sh;
  /* Full framebuffer + 1 overhang row; clamp to 32-row tilemap. */
  int rows = (224 + tile_px - 1) / tile_px + 1;
  if (rows < 8)
    rows = 8;
  if (rows > 32)
    rows = 32;
  return rows;
}

/* $809A18 addressing: $7F byte offset from world pixels. */
static uint16_t mw_7f_off_from_world(uint16_t world_x, uint16_t world_y) {
  const uint16_t row_i = (uint16_t)((world_y & (uint16_t)~15u) >> 3);
  const uint16_t col_i = (uint16_t)((world_x & (uint16_t)~15u) >> 3);
  const uint16_t base = mw_wram16((uint16_t)(0x42B3u + row_i));
  if (!base)
    return 0;
  return (uint16_t)(base + col_i);
}

/* Map world pixel to BG1 VRAM word for the present strip, or -1 if outside.
 * row_slop: ±1 for normal; larger for foreign-mover blank (trail ghosts).
 * X range matches rebuild ([-pad, view+1+pad)) so widescreen west cells
 * of foreign brown are blankable too. */
static int mw_bg1_world_to_vaddr_slop(uint16_t world_x, uint16_t world_y,
                                      uint16_t scroll_x, uint16_t scroll_y,
                                      int row_slop) {
  if (!g_ppu)
    return -1;
  if (row_slop < 1)
    row_slop = 1;
  const unsigned sh = PPU_bigTiles(g_ppu, 0) ? 4u : 3u;
  const int view_cols = 256 >> (int)sh;
  const int n_rows = mw_present_strip_rows(0);
  const int32_t tx =
      (int32_t)(world_x >> sh) - (int32_t)(scroll_x >> sh);
  const int32_t ty =
      (int32_t)(world_y >> sh) - (int32_t)(scroll_y >> sh);
  int pad = (g_ws_active && g_ws_extra > 0) ? mw_ws_tile_pad() : 0;
  if (pad < 0)
    pad = 0;
  if (tx < -pad || tx >= view_cols + 1 + pad)
    return -1;
  if (ty < -row_slop || ty >= n_rows + row_slop)
    return -1;
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, 0);
  const int x_mask = PPU_bgTilemapWider(g_ppu, 0) ? 63 : 31;
  int map_col;
  if (s_present_h2h_full_frame) {
    /* View-relative strip (VRAM col = view index). scroll_* is full cam. */
    map_col = (int)tx;
    map_col %= (x_mask + 1);
    if (map_col < 0)
      map_col += x_mask + 1;
  } else {
    map_col =
        (int)(((uint32_t)(scroll_x >> sh) + (uint32_t)tx) & (uint32_t)x_mask);
  }
  const int map_row =
      (int)(((uint32_t)(scroll_y >> sh) + (uint32_t)ty) & 31u);
  const int half = (x_mask > 31 && map_col >= 32) ? 0x400 : 0;
  return (int)(map_base + half + (map_row << 5) + (map_col & 31));
}

static int mw_bg1_world_to_vaddr(uint16_t world_x, uint16_t world_y,
                                 uint16_t scroll_x, uint16_t scroll_y) {
  return mw_bg1_world_to_vaddr_slop(world_x, world_y, scroll_x, scroll_y, 1);
}

/* Blank one BG1 cell. fill_sky=1 → $0200 (far-Y ghosts); else terrain above. */
static void mw_bg1_blank_world_ex(uint16_t wx, uint16_t wy, uint16_t scroll_x,
                                  uint16_t scroll_y, int fill_sky) {
  const int va = mw_bg1_world_to_vaddr_slop(wx, wy, scroll_x, scroll_y, 3);
  if (va < 0)
    return;
  uint16_t under = 0x0200u;
  if (!fill_sky) {
    under = 0;
    for (int up = 32; up <= 80; up += 16) {
      const uint16_t off = mw_7f_off_from_world(wx, (uint16_t)((int)wy - up));
      if (!off)
        continue;
      const uint16_t t = mw_read7f16(off);
      if (t != 0 && t != 0x0DAEu) {
        under = t;
        break;
      }
    }
  }
  mw_vram_save_word((uint16_t)va);
  g_ppu->vram[(unsigned)va & 0x7fffu] = under;
}

static void mw_bg1_blank_world(uint16_t wx, uint16_t wy, uint16_t scroll_x,
                               uint16_t scroll_y) {
  mw_bg1_blank_world_ex(wx, wy, scroll_x, scroll_y, 0);
}

/* Present-only: write one BG1 word at world XY (rider brown latch paint). */
static void mw_bg1_paint_world(uint16_t wx, uint16_t wy, uint16_t scroll_x,
                               uint16_t scroll_y, uint16_t t) {
  if (!g_ppu || mw_bg1_tile_weak(t))
    return;
  const int va = mw_bg1_world_to_vaddr_slop(wx, wy, scroll_x, scroll_y, 3);
  if (va < 0)
    return;
  mw_vram_save_word((uint16_t)va);
  g_ppu->vram[(unsigned)va & 0x7fffu] = t;
}

/* Present-only write (allows sky/marker — stand BG1 probe). */
static void mw_bg1_write_world(uint16_t wx, uint16_t wy, uint16_t scroll_x,
                               uint16_t scroll_y, uint16_t t) {
  if (!g_ppu)
    return;
  const int va = mw_bg1_world_to_vaddr_slop(wx, wy, scroll_x, scroll_y, 3);
  if (va < 0)
    return;
  mw_vram_save_word((uint16_t)va);
  g_ppu->vram[(unsigned)va & 0x7fffu] = t;
}

/*
 * Giant-stand map/BG1 soak probe (list props skipped — see H2H_STAGE_PROPS §8b).
 * Present-only; restored with VRAM save. Env SNESRECOMP_MW_STAND_BG1=
 *   dump / mark / blank
 * Origin SNESRECOMP_MW_STAND_BG1_AT (default auto):
 *   auto|mid|pres|focus — live $1600 scan (raw cam Y for sy) → per-slot latch
 *   mech — local dual mech only (explicit)
 * Optional STAND_BG1_WX/_WY. mark/blank never fall back to mech (sky stamp).
 * sy uses loc_y=cam_y_raw; VRAM X uses scroll_x=h0 (view-rel).
 */
static int mw_stand_bg1_mode(void) {
  static int inited, mode;
  if (!inited) {
    inited = 1;
    const char *e = getenv("SNESRECOMP_MW_STAND_BG1");
    if (!e || !e[0] || e[0] == '0')
      mode = 0;
    else if (!strcmp(e, "mark"))
      mode = 2;
    else if (!strcmp(e, "blank"))
      mode = 3;
    else
      mode = 1;
  }
  return mode;
}

/*
 * Prefer solid $1600 near mech with screen Y in [sy_lo,sy_hi].
 * from_7f=0 → present VRAM; =1 → $7F map (stand often sky in VRAM).
 * loc_y = raw present cam Y. scroll_x = h0 for VRAM / sx score.
 */
static int mw_stand_bg1_scan_blob(int mx, int my, uint16_t scroll_x,
                                  uint16_t loc_y, int sy_lo, int sy_hi,
                                  int from_7f, int *ox, int *oy) {
  if (!ox || !oy)
    return 0;
  if (!from_7f && !g_ppu)
    return 0;
  int best = 0x7fffffff;
  int bx = 0, by = 0, found = 0;
  for (int dy = -80; dy <= 128; dy += 16) {
    for (int dx = -96; dx <= 96; dx += 16) {
      const int wx = mx + dx;
      const int wy = my + dy;
      const int sy = wy - (int)loc_y;
      if (sy < sy_lo || sy > sy_hi)
        continue;
      uint16_t t = 0;
      if (from_7f) {
        const uint16_t off =
            mw_7f_off_from_world((uint16_t)wx, (uint16_t)wy);
        if (!off)
          continue;
        t = mw_read7f16(off);
      } else {
        t = mw_bg1_vram_at_world((uint16_t)wx, (uint16_t)wy, scroll_x,
                                 loc_y);
      }
      if (mw_bg1_tile_weak(t))
        continue;
      int score = dx * dx / 4 + dy * dy / 4;
      if (t == 0x1600u)
        score -= 50000;
      /* Prefer on-screen; reject far-east shelf latch (sx≫256). */
      {
        const int sx = wx - (int)scroll_x;
        if (sx < -64 || sx >= 320)
          score += 20000;
        const int adx = sx - 160;
        score += (adx < 0 ? -adx : adx);
      }
      if (score < best) {
        best = score;
        bx = wx;
        by = wy;
        found = 1;
      }
    }
  }
  if (!found)
    return 0;
  *ox = bx;
  *oy = by;
  return 1;
}

static void mw_stand_bg1_fill_grid(int ox, int oy, int mode, uint16_t scroll_x,
                                   uint16_t loc_y, int dy0, int dy1) {
  for (int dy = dy0; dy <= dy1; dy += 16) {
    for (int dx = -64; dx <= 64; dx += 16) {
      if (s_stand_bg1.n_cells >= kMwStandBg1Max)
        return;
      const int wx = ox + dx;
      const int wy = oy + dy;
      uint16_t t7 = 0, tv = 0;
      const uint16_t off =
          mw_7f_off_from_world((uint16_t)wx, (uint16_t)wy);
      if (off)
        t7 = mw_read7f16(off);
      if (g_ppu)
        tv = mw_bg1_vram_at_world((uint16_t)wx, (uint16_t)wy, scroll_x,
                                  loc_y);
      if (!mw_bg1_tile_weak(t7))
        s_stand_bg1.n_solid7f++;
      if (!mw_bg1_tile_weak(tv))
        s_stand_bg1.n_solidv++;
      uint16_t tv1 = tv;
      if (mode == 3) {
        if (!mw_bg1_tile_weak(tv) || !mw_bg1_tile_weak(t7)) {
          mw_bg1_blank_world_ex((uint16_t)wx, (uint16_t)wy, scroll_x, loc_y,
                                1);
          tv1 = 0x0200u;
          s_stand_bg1.n_touched++;
        }
      } else if (mode == 2) {
        if (!mw_bg1_tile_weak(tv)) {
          tv1 = (uint16_t)(tv ^ 0x0018u);
          mw_bg1_write_world((uint16_t)wx, (uint16_t)wy, scroll_x, loc_y,
                             tv1);
          s_stand_bg1.n_touched++;
        } else if (!mw_bg1_tile_weak(t7)) {
          /* VRAM sky, $7F has stand — stamp map ink into present VRAM. */
          mw_bg1_write_world((uint16_t)wx, (uint16_t)wy, scroll_x, loc_y, t7);
          tv1 = t7;
          s_stand_bg1.n_touched++;
        }
      }
      const int i = s_stand_bg1.n_cells++;
      s_stand_bg1.dx[i] = (int16_t)dx;
      s_stand_bg1.dy[i] = (int16_t)dy;
      s_stand_bg1.t7f[i] = t7;
      s_stand_bg1.tv0[i] = tv;
      s_stand_bg1.tv1[i] = tv1;
    }
  }
}

static int mw_stand_bg1_try_scans(int mx, int my, uint16_t scroll_x,
                                  uint16_t loc_y, const char *at, int *ox,
                                  int *oy, const char **src_out) {
  int sy0 = 80, sy1 = 130;
  if (!strcmp(at, "pres")) {
    sy0 = 40;
    sy1 = 80;
  } else if (!strcmp(at, "focus")) {
    sy0 = 20;
    sy1 = 60;
  }
  /* VRAM first, then $7F (stand often missing from present VRAM). */
  if (mw_stand_bg1_scan_blob(mx, my, scroll_x, loc_y, sy0, sy1, 0, ox, oy)) {
    *src_out = "scan";
    return 1;
  }
  if (!strcmp(at, "auto") &&
      mw_stand_bg1_scan_blob(mx, my, scroll_x, loc_y, 40, 80, 0, ox, oy)) {
    *src_out = "scan";
    return 1;
  }
  if (mw_stand_bg1_scan_blob(mx, my, scroll_x, loc_y, 20, 160, 0, ox, oy)) {
    *src_out = "scan";
    return 1;
  }
  if (mw_stand_bg1_scan_blob(mx, my, scroll_x, loc_y, sy0, sy1, 1, ox, oy)) {
    *src_out = "scan7f";
    return 1;
  }
  if (!strcmp(at, "auto") &&
      mw_stand_bg1_scan_blob(mx, my, scroll_x, loc_y, 40, 80, 1, ox, oy)) {
    *src_out = "scan7f";
    return 1;
  }
  if (mw_stand_bg1_scan_blob(mx, my, scroll_x, loc_y, 20, 160, 1, ox, oy)) {
    *src_out = "scan7f";
    return 1;
  }
  return 0;
}

static void mw_stand_bg1_probe(int local_slot, uint16_t cam_x, uint16_t cam_y,
                               uint16_t scroll_x, uint16_t scroll_y) {
  memset(&s_stand_bg1, 0, sizeof(s_stand_bg1));
  s_stand_bg1.slot = local_slot;
  s_stand_bg1.ox = -1;
  s_stand_bg1.oy = -1;
  const int mode = mw_stand_bg1_mode();
  if (!mw_coldump_armed() && mode < 2)
    return;
  if (mode == 2)
    snprintf(s_stand_bg1.mode, sizeof(s_stand_bg1.mode), "mark");
  else if (mode == 3)
    snprintf(s_stand_bg1.mode, sizeof(s_stand_bg1.mode), "blank");
  else
    snprintf(s_stand_bg1.mode, sizeof(s_stand_bg1.mode), "dump");

  (void)scroll_y;
  const uint16_t loc_y = cam_y;

  int mx = 0, my = 0;
  const int have_mech = mw_find_dual_mech_xy(local_slot, &mx, &my);
  s_stand_bg1.mx = have_mech ? mx : -1;
  s_stand_bg1.my = have_mech ? my : -1;

  const char *at = getenv("SNESRECOMP_MW_STAND_BG1_AT");
  if (!at || !at[0])
    at = "auto";
  const char *ex = getenv("SNESRECOMP_MW_STAND_BG1_WX");
  const char *ey = getenv("SNESRECOMP_MW_STAND_BG1_WY");

  int ox = -1, oy = -1;
  const char *src = "miss";

  if (ex && ex[0] && ey && ey[0]) {
    ox = atoi(ex);
    oy = atoi(ey);
    src = "env";
  } else if (!strcmp(at, "mech")) {
    if (have_mech) {
      ox = mx;
      oy = my;
      src = "mech";
    }
  } else {
    /* Per-slot latch first (survives other peer coldump + VRAM sky frames).
     * Reject far-east shelf false latches (sx≫playfield). */
    if ((local_slot == 0 || local_slot == 1)) {
      if ((!strcmp(at, "mid") || !strcmp(at, "auto") || !strcmp(at, "focus")) &&
          s_stand_latch_mid_have[local_slot]) {
        const int lsx = s_stand_latch_mid_wx[local_slot] - (int)scroll_x;
        if (lsx >= -64 && lsx < 320) {
          ox = s_stand_latch_mid_wx[local_slot];
          oy = s_stand_latch_mid_wy[local_slot];
          src = "latch";
        }
      }
      if (ox < 0 &&
          (!strcmp(at, "pres") || !strcmp(at, "auto") || !strcmp(at, "mid")) &&
          s_stand_latch_pres_have[local_slot]) {
        const int lsx = s_stand_latch_pres_wx[local_slot] - (int)scroll_x;
        if (lsx >= -64 && lsx < 320) {
          ox = s_stand_latch_pres_wx[local_slot];
          oy = s_stand_latch_pres_wy[local_slot];
          src = "latch";
        }
      }
    }
    if (ox < 0 && have_mech) {
      const char *scan_src = "scan";
      if (mw_stand_bg1_try_scans(mx, my, scroll_x, loc_y, at, &ox, &oy,
                                 &scan_src))
        src = scan_src;
    }
  }

  snprintf(s_stand_bg1.src, sizeof(s_stand_bg1.src), "%s", src);
  s_stand_bg1.ox = ox;
  s_stand_bg1.oy = oy;

  static int logged;
  if (!logged && mode >= 1) {
    logged = 1;
    fprintf(stderr,
            "[mw_rtl] STAND_BG1=%s at=%s src=%s origin=(%d,%d) slot=%d — "
            "present-only; unset when done\n",
            s_stand_bg1.mode, at, src, ox, oy, local_slot);
  }

  if (ox < 0 || oy < 0) {
    /* Keep mode+src=miss for coldump; no mech sky stamp. */
    return;
  }

  mw_stand_bg1_fill_grid(ox, oy, mode, scroll_x, loc_y, -80, 64);
  (void)cam_x;
}

/* Home brown latch: free $C6A4 + pinned $C382 (not on-mech backpack).
 * All-stage-prop $C382 home-brown was a false lead (2026-08-05): elev-room
 * stand is plat.pres map/$7F; catalog $C382 pins sit far-Y (sy≪0). */
static int mw_home_brown_eligible(uint16_t obj) {
  const uint16_t meta = mw_wram16((uint16_t)(obj + 8u));
  if (mw_prop_is_pinned_hazard(obj))
    return 1;
  if (meta != 0xC6A4u)
    return 0;
  /* Backpack $C6A4 has sky at origin — never latch/paint brown on the mech. */
  const int ox = (int)mw_wram16((uint16_t)(obj + 2u));
  const int oy = (int)mw_wram16((uint16_t)(obj + 4u));
  return !mw_prop_live_near_mech(ox, oy);
}

/*
 * Capture / paint home mover brown from $7F. Refresh latch only when the
 * pocket is solid (dual stomps leave void — keep prior latch).
 */
static void mw_rider_brown_capture(uint16_t obj, int owx, int owy) {
  if (!mw_home_brown_eligible(obj))
    return;
  const int s = mw_prop_slot_for_obj(obj);
  const int wide = mw_prop_c382_wide_fp(obj);
  static const int kDxN[7] = {-0x30, -0x20, -0x10, 0, 0x10, 0x20, 0x30};
  static const int kDxW[12] = {-0x30, -0x20, -0x10, 0,    0x10, 0x20,
                               0x30,  0x40,  0x50, 0x60, 0x80, 0xA0};
  const int *dxs = wide ? kDxW : kDxN;
  const int ndx = wide ? 12 : 7;
  uint16_t tmp_t[kMwRiderBrownCells];
  int16_t tmp_dx[kMwRiderBrownCells];
  int16_t tmp_dy[kMwRiderBrownCells];
  unsigned n = 0;
  for (int dy = -0x20; dy <= 0x40 && n < (unsigned)kMwRiderBrownCells;
       dy += 0x10) {
    for (int k = 0; k < ndx && n < (unsigned)kMwRiderBrownCells; k++) {
      const int bx = owx + dxs[k];
      const int by = owy + dy;
      const uint16_t off =
          mw_7f_off_from_world((uint16_t)bx, (uint16_t)by);
      if (!off)
        continue;
      const uint16_t t = mw_read7f16(off);
      if (mw_bg1_tile_weak(t))
        continue;
      tmp_dx[n] = (int16_t)dxs[k];
      tmp_dy[n] = (int16_t)dy;
      tmp_t[n] = t;
      n++;
    }
  }
  if (n < 4u) {
    /* Keep prior tiles only if origin still near the mover — else glue can
     * pin $C6A4 to a stale Y (elevator rose; dual $7F blocked recapture). */
    if (s_rider_brown_valid[s] && s_rider_brown_obj[s] == obj) {
      const int dox = (int)s_rider_brown_ox[s] - owx;
      const int doy = (int)s_rider_brown_oy[s] - owy;
      if (dox > 48 || dox < -48 || doy > 48 || doy < -48)
        s_rider_brown_valid[s] = 0;
    }
    return;
  }
  s_rider_brown_obj[s] = obj;
  s_rider_brown_valid[s] = 1;
  s_rider_brown_n[s] = (uint8_t)n;
  s_rider_brown_ox[s] = (uint16_t)owx;
  s_rider_brown_oy[s] = (uint16_t)owy;
  memcpy(s_rider_brown_dx[s], tmp_dx, n * sizeof(tmp_dx[0]));
  memcpy(s_rider_brown_dy[s], tmp_dy, n * sizeof(tmp_dy[0]));
  memcpy(s_rider_brown_t[s], tmp_t, n * sizeof(tmp_t[0]));
}

/*
 * Scrub then paint home brown. Blanking the latched footprint first clears
 * dual-stomp / strip residue so a second cam-following ghost can't sit
 * beside the world-locked latch. When present XY moves (live cam-glued then
 * anchor snap), also scrub the *previous* paint origin — otherwise the
 * follow-frame ink stays on screen and wrestles with the world-locked body.
 */
static void mw_rider_brown_paint(uint16_t obj, int owx, int owy,
                                 uint16_t scroll_x, uint16_t scroll_y) {
  if (!mw_home_brown_eligible(obj))
    return;
  const int s = mw_prop_slot_for_obj(obj);
  if (!s_rider_brown_valid[s] || s_rider_brown_obj[s] != obj ||
      s_rider_brown_n[s] == 0)
    return;
  /*
   * Refuse far present XY when sticky is empty. Cam-follow keeps sn>0 and
   * paints at the world anchor while live walks with the cam; pool-recycle
   * ghosts had sn=0 + use≪live (P2 $C6A4 use=637 live=350) and stamped
   * display-only brown at the wrong world X.
   */
  {
    const int live_ox = (int)mw_wram16((uint16_t)(obj + 2u));
    const int live_oy = (int)mw_wram16((uint16_t)(obj + 4u));
    const int adx = owx - live_ox;
    const int ady = owy - live_oy;
    const int sn =
        (s_prop_sticky_valid[s] && s_prop_sticky_obj[s] == obj)
            ? (int)s_prop_sticky_n[s]
            : 0;
    if (sn == 0 &&
        (adx > 48 || adx < -48 || ady > 48 || ady < -48))
      return;
  }
  if (s_rider_brown_paint_have[s] &&
      (s_rider_brown_paint_ox[s] != (uint16_t)owx ||
       s_rider_brown_paint_oy[s] != (uint16_t)owy)) {
    const int pox = (int)s_rider_brown_paint_ox[s];
    const int poy = (int)s_rider_brown_paint_oy[s];
    for (unsigned i = 0; i < s_rider_brown_n[s]; i++) {
      const uint16_t bx =
          (uint16_t)(pox + (int)s_rider_brown_dx[s][i]);
      const uint16_t by =
          (uint16_t)(poy + (int)s_rider_brown_dy[s][i]);
      mw_bg1_blank_world(bx, by, scroll_x, scroll_y);
    }
  }
  for (unsigned i = 0; i < s_rider_brown_n[s]; i++) {
    const uint16_t bx =
        (uint16_t)(owx + (int)s_rider_brown_dx[s][i]);
    const uint16_t by =
        (uint16_t)(owy + (int)s_rider_brown_dy[s][i]);
    mw_bg1_blank_world(bx, by, scroll_x, scroll_y);
    mw_bg1_paint_world(bx, by, scroll_x, scroll_y, s_rider_brown_t[s][i]);
  }
  s_rider_brown_paint_ox[s] = (uint16_t)owx;
  s_rider_brown_paint_oy[s] = (uint16_t)owy;
  s_rider_brown_paint_have[s] = 1;
}

/*
 * True when map word `t` matches foreign ledge ink near (owx,owy).
 * wide=1 ($C382): sample farther east + ±1 tile Y — origin sits west of body.
 */
static int mw_prop_tile_matches_origin(uint16_t t, int owx, int owy, int wide) {
  if (mw_bg1_tile_void(t) || t == 0x0200u)
    return 0;
  const int dx_hi = wide ? 0x100 : 0x20;
  const int dy_lo = wide ? -0x10 : 0;
  const int dy_hi = wide ? 0x20 : 0;
  for (int ody = dy_lo; ody <= dy_hi; ody += 0x10) {
    for (int dx = -0x20; dx <= dx_hi; dx += 0x10) {
      const uint16_t po = mw_7f_off_from_world((uint16_t)(owx + dx),
                                               (uint16_t)(owy + ody));
      if (po && mw_read7f16(po) == t)
        return 1;
    }
  }
  return 0;
}

/* True when $7F at (bx,by) matches foreign ledge ink near (owx,owy). */
static int mw_prop_7f_matches_origin(int bx, int by, int owx, int owy,
                                     int wide) {
  const uint16_t off = mw_7f_off_from_world((uint16_t)bx, (uint16_t)by);
  if (!off)
    return 0;
  return mw_prop_tile_matches_origin(mw_read7f16(off), owx, owy, wide);
}

/* VRAM residue: $7F at cell may already be terrain while ghost tile remains.
 * wide pins + $C6A4 riders (sticky-only present still leaves strip ink). */
static int mw_prop_vram_matches_origin(int bx, int by, int owx, int owy,
                                       int vram_fp, uint16_t scroll_x,
                                       uint16_t scroll_y) {
  if (!g_ppu || !vram_fp)
    return 0;
  const int va =
      mw_bg1_world_to_vaddr_slop((uint16_t)bx, (uint16_t)by, scroll_x,
                                 scroll_y, 3);
  if (va < 0)
    return 0;
  return mw_prop_tile_matches_origin(g_ppu->vram[(unsigned)va & 0x7fffu], owx,
                                     owy, 1);
}

/*
 * Home prop owns this strip column at world Y — keep ledge.
 * Do NOT keepout foreign props merely because they intersect local FOV:
 * that preserved P1's brown on P2 (dual-cam X slide/snap, map-correct Y).
 */
static int mw_prop_home_keepout_x(int bx, int by, int local_slot, uint16_t c0x,
                                  uint16_t c0y, uint16_t c1x, uint16_t c1y) {
  uint16_t idx = mw_wram16(0x1E14u);
  for (int guard = 0; guard < 64 && idx; guard++) {
    const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
    if (mw_obj_is_stage_prop(idx)) {
      const int home = mw_stage_prop_home_cam(idx, c0x, c0y, c1x, c1y);
      if (home == local_slot) {
        const int pwx = (int)mw_wram16((uint16_t)(idx + 2u));
        const int pwy = (int)mw_wram16((uint16_t)(idx + 4u));
        const int ady = pwy > by ? pwy - by : by - pwy;
        const int adx = pwx > bx ? pwx - bx : bx - pwx;
        if (ady <= 0x30 && adx <= 0x70)
          return 1;
      }
    }
    if (next == 0 || next == idx)
      break;
    idx = next;
  }
  return 0;
}

/* Pocket blank around object origin (origin inside / near 4:3). */
static unsigned mw_prop_blank_band_pocket(int owx, int owy, int wide,
                                          uint16_t scroll_x,
                                          uint16_t scroll_y) {
  /* Pin/wide: body extends east of origin — sample farther (+ denser Y). */
  static const int kDxN[7] = {-0x30, -0x20, -0x10, 0, 0x10, 0x20, 0x30};
  static const int kDxW[12] = {-0x30, -0x20, -0x10, 0,    0x10, 0x20,
                               0x30,  0x40,  0x50, 0x60, 0x80, 0xA0};
  const int *dxs = wide ? kDxW : kDxN;
  const int ndx = wide ? 12 : 7;
  const int dy_step = wide ? 8 : 8;
  const int dy_lo = wide ? -0x30 : -64;
  const int dy_hi = wide ? 0x60 : 0x80;
  unsigned hit = 0;
  for (int dy = dy_lo; dy <= dy_hi; dy += dy_step) {
    for (int k = 0; k < ndx; k++) {
      const uint16_t bx = (uint16_t)(owx + dxs[k]);
      const uint16_t by = (uint16_t)(owy + dy);
      if (mw_bg1_world_to_vaddr_slop(bx, by, scroll_x, scroll_y, 3) >= 0) {
        mw_bg1_blank_world(bx, by, scroll_x, scroll_y);
        hit++;
      }
    }
  }
  return hit;
}

/*
 * Foreign ledge ink in the native 4:3 stripe (cols 0..view), not west gutter.
 * Origin often far off-screen in X (sx≈−400) while brown still fills the DMA
 * window. Narrow Y band + $7F fingerprint only — no full-strip / far-Y scan
 * (shared brown tile IDs punched stair-step holes through walls/floors).
 * Pin-wide: longer Y + VRAM residue match (dual $7F stomps leave ghosts).
 * Same policy for local_slot 0 and 1 (P1/P2 mirrored).
 * Do NOT blank home mid-screen / ignore-keepout self-blank — that shattered
 * floors/walls and missed $D5B8 elevators (BG2 path).
 */
static unsigned mw_prop_blank_native_strip(int owx, int owy, int local_slot,
                                           uint16_t c0x, uint16_t c0y,
                                           uint16_t c1x, uint16_t c1y,
                                           int wide, int vram_fp,
                                           uint16_t scroll_x,
                                           uint16_t scroll_y) {
  if (!g_ppu)
    return 0;
  const unsigned sh = PPU_bigTiles(g_ppu, 0) ? 4u : 3u;
  const int tile_px = 1 << (int)sh;
  const int view_cols = 256 >> (int)sh;
  const int n_rows = mw_present_strip_rows(0);
  const int32_t ty =
      (int32_t)(owy >> sh) - (int32_t)(scroll_y >> sh);
  /* Body extends ± a few tiles from origin (pins / tall crates). Origin-only
   * ±2 skipped foreign crate brown when sy was just outside the strip.
   * Do NOT widen to far-Y / dy≥$100 — shared brown tile IDs punch stair-step
   * sky holes through walls/floors (reverted 2026-08-03 and again 2026-08-04). */
  const int ty_slop = wide ? 6 : 4;
  if (ty < -ty_slop || ty >= n_rows + ty_slop)
    return 0;
  const int32_t tx_o =
      (int32_t)(owx >> sh) - (int32_t)(scroll_x >> sh);
  /* Origin already in native 4:3 — pocket path handles it. */
  if (tx_o >= 0 && tx_o < view_cols)
    return 0;

  const int dy_lo = wide ? -0x20 : -0x10;
  const int dy_hi = wide ? 0x50 : 0x30;
  unsigned hit = 0;
  for (int dy = dy_lo; dy <= dy_hi; dy += tile_px) {
    const int by = owy + dy;
    for (int col = 0; col < view_cols; col++) {
      const int bx = (int)scroll_x + col * tile_px + tile_px / 2;
      if (mw_prop_home_keepout_x(bx, by, local_slot, c0x, c0y, c1x, c1y))
        continue;
      if (!mw_prop_7f_matches_origin(bx, by, owx, owy, wide) &&
          !mw_prop_vram_matches_origin(bx, by, owx, owy, vram_fp, scroll_x,
                                       scroll_y))
        continue;
      if (mw_bg1_world_to_vaddr_slop((uint16_t)bx, (uint16_t)by, scroll_x,
                                     scroll_y, 3) < 0)
        continue;
      mw_bg1_blank_world((uint16_t)bx, (uint16_t)by, scroll_x, scroll_y);
      hit++;
    }
  }
  return hit;
}

static unsigned mw_prop_blank_band_ex(int owx, int owy, int local_slot,
                                      uint16_t c0x, uint16_t c0y, uint16_t c1x,
                                      uint16_t c1y, int wide, int vram_fp,
                                      uint16_t scroll_x, uint16_t scroll_y) {
  unsigned hit =
      mw_prop_blank_native_strip(owx, owy, local_slot, c0x, c0y, c1x, c1y,
                                 wide, vram_fp, scroll_x, scroll_y);
  hit += mw_prop_blank_band_pocket(owx, owy, wide, scroll_x, scroll_y);
  return hit;
}

static unsigned mw_prop_blank_band(int owx, int owy, int local_slot,
                                   uint16_t c0x, uint16_t c0y, uint16_t c1x,
                                   uint16_t c1y, uint16_t scroll_x,
                                   uint16_t scroll_y) {
  return mw_prop_blank_band_ex(owx, owy, local_slot, c0x, c0y, c1x, c1y, 0, 0,
                               scroll_x, scroll_y);
}

/*
 * During native-col rebuild: suppress foreign ledge ink only (never home /
 * far-Y full-strip). Match tile vs foreign origin $7F — west gutter skipped.
 */
static int mw_prop_foreign_ink_tile(int wx, int wy, uint16_t tile,
                                    int local_slot, uint16_t c0x, uint16_t c0y,
                                    uint16_t c1x, uint16_t c1y,
                                    uint16_t scroll_x) {
  (void)scroll_x;
  if (!g_ppu || !s_present_h2h_full_frame)
    return 0;
  if (mw_bg1_tile_void(tile) || tile == 0x0200u)
    return 0;
  if (mw_prop_home_keepout_x(wx, wy, local_slot, c0x, c0y, c1x, c1y))
    return 0;
  uint16_t idx = mw_wram16(0x1E14u);
  for (int guard = 0; guard < 64 && idx; guard++) {
    const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
    if (mw_obj_is_stage_prop(idx)) {
      const int home = mw_stage_prop_home_cam(idx, c0x, c0y, c1x, c1y);
      const int owx = (int)mw_wram16((uint16_t)(idx + 2u));
      const int owy = (int)mw_wram16((uint16_t)(idx + 4u));
      /* Foreign movers: always suppress ink (no in-FOV keep — that left a
       * static brown ghost on the non-home peer). */
      if (home != local_slot) {
        const int wide = mw_prop_c382_wide_fp(idx);
        const int ady = owy > wy ? owy - wy : wy - owy;
        const int ady_lim = wide ? 0x50 : 0x30;
        if (ady <= ady_lim) {
          const int dx_hi = wide ? 0xA0 : 0x20;
          for (int dx = -0x20; dx <= dx_hi; dx += 0x10) {
            const uint16_t po =
                mw_7f_off_from_world((uint16_t)(owx + dx), (uint16_t)owy);
            if (po && mw_read7f16(po) == tile)
              return 1;
          }
          if (mw_prop_7f_matches_origin(wx, wy, owx, owy, wide))
            return 1;
        }
      }
    }
    if (next == 0 || next == idx)
      break;
    idx = next;
  }
  return 0;
}

static uint16_t mw_bg1_under_at_world(int wx, int wy) {
  for (int up = 32; up <= 80; up += 16) {
    const uint16_t off =
        mw_7f_off_from_world((uint16_t)wx, (uint16_t)((int)wy - up));
    if (!off)
      continue;
    const uint16_t t = mw_read7f16(off);
    if (t != 0 && t != 0x0DAEu)
      return t;
  }
  return 0x0200u;
}

/*
 * Shared $7F still carries every mover's brown body. After local BG1 rebuild
 * (and again after margin prefill): blank foreign movers at live pos AND the
 * previous trail cell (shared $7F leaves ghosts when a platform rides away).
 */
static void mw_present_align_stage_prop_bg1(int local_slot, uint16_t loc_x,
                                            uint16_t loc_y, uint16_t oth_x,
                                            uint16_t oth_y, uint16_t scroll_x,
                                            uint16_t scroll_y) {
  s_elev_prop_bg_dy = 0;
  if (!g_ppu || !s_present_h2h_full_frame || !MwIsDualViewport())
    return;
  if (local_slot != 0 && local_slot != 1)
    return;

  const uint16_t c0x = (local_slot == 0) ? loc_x : oth_x;
  const uint16_t c0y = (local_slot == 0) ? loc_y : oth_y;
  const uint16_t c1x = (local_slot == 1) ? loc_x : oth_x;
  const uint16_t c1y = (local_slot == 1) ? loc_y : oth_y;
  unsigned n_hide = 0;
  unsigned n_try = 0;
  memset(s_coldump_bg_obj, 0, sizeof(s_coldump_bg_obj));
  memset(s_coldump_bg_hit, 0, sizeof(s_coldump_bg_hit));
  memset(s_coldump_bg_try, 0, sizeof(s_coldump_bg_try));
  s_coldump_bg_slot = local_slot;

  uint16_t idx = mw_wram16(0x1E14u);
  for (int guard = 0; guard < 64 && idx; guard++) {
    const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
    if (mw_obj_is_stage_prop(idx)) {
      const int home = mw_stage_prop_home_cam(idx, c0x, c0y, c1x, c1y);
      const int owx = (int)mw_wram16((uint16_t)(idx + 2u));
      const int owy = (int)mw_wram16((uint16_t)(idx + 4u));
      const int ts = mw_prop_slot_for_obj(idx);
      const int had = s_prop_trail_valid[ts] && s_prop_trail_obj[ts] == idx;
      const int pwx = had ? (int)s_prop_trail_wx[ts] : owx;
      const int pwy = had ? (int)s_prop_trail_wy[ts] : owy;
      /* Foreign home → always blank brown. Do not blank home (map shatter). */
      if (home != local_slot) {
        n_try++;
        const int wide = mw_prop_c382_wide_fp(idx);
        const int vram_fp = wide || mw_prop_is_c6a4_rider(idx);
        int use_ox = owx, use_oy = owy;
        mw_prop_present_world_xy(idx, owx, owy, loc_x, home, local_slot,
                                 &use_ox, &use_oy);
        /* Drop any prior home-paint latch — display-only ink must not linger
         * on the non-home peer after a home flip / pool recycle. */
        if (ts >= 0 && ts < kMwPropHomeMax && s_rider_brown_obj[ts] == idx) {
          if (s_rider_brown_paint_have[ts] && s_rider_brown_n[ts] > 0) {
            const int pox = (int)s_rider_brown_paint_ox[ts];
            const int poy = (int)s_rider_brown_paint_oy[ts];
            for (unsigned i = 0; i < s_rider_brown_n[ts]; i++) {
              mw_bg1_blank_world(
                  (uint16_t)(pox + (int)s_rider_brown_dx[ts][i]),
                  (uint16_t)(poy + (int)s_rider_brown_dy[ts][i]), scroll_x,
                  scroll_y);
            }
          }
          mw_rider_brown_clear_slot(ts);
        }
        unsigned hit =
            mw_prop_blank_band_ex(use_ox, use_oy, local_slot, c0x, c0y, c1x,
                                  c1y, wide, vram_fp, scroll_x, scroll_y);
        if (had && (pwx != use_ox || pwy != use_oy))
          hit += mw_prop_blank_band_ex(pwx, pwy, local_slot, c0x, c0y, c1x,
                                       c1y, wide, vram_fp, scroll_x,
                                       scroll_y);
        if (use_ox != owx || use_oy != owy)
          hit += mw_prop_blank_band_ex(owx, owy, local_slot, c0x, c0y, c1x,
                                       c1y, wide, vram_fp, scroll_x,
                                       scroll_y);
        if (ts >= 0 && ts < 24) {
          s_coldump_bg_obj[ts] = idx;
          s_coldump_bg_hit[ts] = (uint16_t)(hit > 0xffffu ? 0xffffu : hit);
          s_coldump_bg_try[ts] = 1;
        }
        if (hit)
          n_hide++;
        else if (getenv("SNESRECOMP_MW_ELEV")) {
          static unsigned miss_logs;
          if (miss_logs < 16u) {
            miss_logs++;
            const int sx = use_ox - (int)scroll_x;
            const int sy = use_oy - (int)scroll_y;
            fprintf(stderr,
                    "[mw_prop] bg_miss obj=$%04X home=%d slot=%d "
                    "ow=$%04X/$%04X scrn=%d,%d scroll=$%04X/$%04X\n",
                    (unsigned)idx, home, local_slot, (unsigned)use_ox,
                    (unsigned)use_oy, sx, sy, (unsigned)scroll_x,
                    (unsigned)scroll_y);
          }
        }
        s_prop_trail_obj[ts] = idx;
        s_prop_trail_wx[ts] = (uint16_t)use_ox;
        s_prop_trail_wy[ts] = (uint16_t)use_oy;
        s_prop_trail_valid[ts] = 1;
      } else if (mw_home_brown_eligible(idx)) {
        /* Home brown latch/paint at present XY (anchor when cam-glued).
         * Pins: catalog +$02/+$04 is stable — never paint at a glued live
         * that tracked the home cam (display-only wrestle). Skip pin paint
         * when origin is far_y from local cam (wrong home + upstairs ink). */
        int use_ox = owx, use_oy = owy;
        int do_paint = 1;
        if (mw_prop_is_pinned_hazard(idx)) {
          if (mw_prop_origin_far_y(owy, loc_y))
            do_paint = 0;
          use_ox = owx;
          use_oy = owy;
        } else {
          mw_prop_present_world_xy(idx, owx, owy, loc_x, home, local_slot,
                                   &use_ox, &use_oy);
          /* Capture at live when use was rejected as recycle junk — paint
           * gate also skips sn=0+|use−live|>48. */
          {
            const int adx = use_ox - owx;
            const int ady = use_oy - owy;
            const int ps = mw_prop_slot_for_obj(idx);
            const int sn =
                (ps >= 0 && s_prop_sticky_valid[ps] &&
                 s_prop_sticky_obj[ps] == idx)
                    ? (int)s_prop_sticky_n[ps]
                    : 0;
            if (sn == 0 &&
                (adx > 48 || adx < -48 || ady > 48 || ady < -48)) {
              use_ox = owx;
              use_oy = owy;
            }
          }
        }
        if (do_paint) {
          mw_rider_brown_capture(idx, use_ox, use_oy);
          mw_rider_brown_paint(idx, use_ox, use_oy, scroll_x, scroll_y);
        }
        s_prop_trail_obj[ts] = idx;
        s_prop_trail_wx[ts] = (uint16_t)use_ox;
        s_prop_trail_wy[ts] = (uint16_t)use_oy;
        s_prop_trail_valid[ts] = 1;
      } else {
        s_prop_trail_obj[ts] = idx;
        s_prop_trail_wx[ts] = (uint16_t)owx;
        s_prop_trail_wy[ts] = (uint16_t)owy;
        s_prop_trail_valid[ts] = 1;
      }
    }
    if (next == 0 || next == idx)
      break;
    idx = next;
  }

  s_elev_prop_bg_dy = (int)n_hide;
  if (getenv("SNESRECOMP_MW_ELEV")) {
    static int log_div;
    if ((log_div++ % 30) == 0)
      fprintf(stderr,
              "[mw_rtl] H2H stage-prop BG1: hide_foreign=%u try=%u slot=%d\n",
              n_hide, n_try, local_slot);
  }
}

static void mw_present_rebuild_local_strips(uint16_t cam_x, uint16_t cam_y,
                                           uint16_t h0, uint16_t v0,
                                           uint16_t h1, uint16_t v1,
                                           int rebuild_bg1, int rebuild_bg2,
                                           uint16_t raw_cam_y) {
  mw_vram_save_reset();
  if (rebuild_bg1)
    mw_present_rebuild_layer_strip(0, cam_x, cam_y, h0, v0,
                                   mw_present_strip_rows(0), raw_cam_y);
  /* Only rebuild BG2 when the game streams it from $7F. Decorative /
   * idle BG2 keeps native VRAM (map fill was painting over the backdrop). */
  if (rebuild_bg2)
    mw_present_rebuild_layer_strip(1, cam_x, cam_y, h1, v1,
                                   mw_present_strip_rows(1), 0);
}

/* Present-only: rewrite narrow BG2's DMA strip from ROM (save/restore).
 * Dual-sim $7F dirty uploads can stomp the decorative nametable — but
 * per-peer restamp under full-frame present painted elevator ghosts (shared
 * VRAM + foreign DMA row cache). Dual: never restamp; keep live DMA + native
 * parallax scrolls. Offline/non-dual may still repair from local-owned rows.
 * Do not expand/pitch-extrapolate or Y-rewindow — that sheared PORT/terrain. */
static void mw_present_restamp_bg2_from_rom(int local_slot, uint16_t scroll_y) {
  s_coldump_bg2_present_v1 = scroll_y;
  s_coldump_bg2_restamp = 0;
  s_coldump_bg2_own_mask = 0;
  if (!g_ppu || !g_snes)
    return;
  if ((g_ppu->bgXsc[1] & 1) != 0)
    return;

  const int dual = MwIsDualViewport();
  uint16_t own_mask = 0;
  for (unsigned r = 0; r < (unsigned)kMwBg2RomRows; r++) {
    if (!mw_bg2_rom_row_fresh(r))
      continue;
    const int own = (int)s_bg2_rom_row_owner[r];
    /* Dual coldump: local-owned only. Non-dual restamp: local or locate. */
    if (own == local_slot || (!dual && own < 0))
      own_mask |= (uint16_t)(1u << r);
  }
  s_coldump_bg2_own_mask = own_mask;
  if (dual)
    return; /* shared nametable — restamp ghosts the elevator */

  if (!s_bg2_rom_valid_mask)
    mw_bg2_rom_locate_from_vram();
  if (!s_bg2_rom_valid_mask)
    return;
  if (!own_mask)
    return;

  const unsigned sh = PPU_bigTiles(g_ppu, 1) ? 4u : 3u;
  const int n_vis = sh == 4u ? 16 : 29;
  const unsigned ty0 = (unsigned)scroll_y >> sh;
  uint16_t vis_mask = 0;
  for (int v = 0; v < n_vis; v++) {
    const unsigned mr = (ty0 + (unsigned)v) & 31u;
    if (mr < (unsigned)kMwBg2RomRows)
      vis_mask |= (uint16_t)(1u << mr);
  }

  /* Newest local-owned fresh row on-screen; pitch-continuous run ∩ visible. */
  int seed = -1;
  int seed_frame = -1;
  for (unsigned r = 0; r < (unsigned)kMwBg2RomRows; r++) {
    if (!(own_mask & (uint16_t)(1u << r)))
      continue;
    if (!(vis_mask & (uint16_t)(1u << r)))
      continue;
    if (s_bg2_rom_row_frame[r] >= seed_frame) {
      seed_frame = s_bg2_rom_row_frame[r];
      seed = (int)r;
    }
  }
  if (seed < 0)
    return;

  const uint8_t seed_bank = s_bg2_rom_bank[seed];
  const uint16_t seed_addr = s_bg2_rom_addr[seed];
  uint16_t run_mask = 0;
  for (int d = -kMwBg2RomDmaRows; d <= kMwBg2RomDmaRows; d++) {
    const int r = seed + d;
    if (r < 0 || r >= kMwBg2RomRows)
      continue;
    if (!(own_mask & (uint16_t)(1u << r)))
      continue;
    if (!mw_bg2_rom_row_fresh((unsigned)r))
      continue;
    if (s_bg2_rom_bank[r] != seed_bank)
      continue;
    const int expect = (int)seed_addr + d * (int)kMwBg2RomPitch;
    if (expect < 0 || expect > 0xffff)
      continue;
    if (s_bg2_rom_addr[r] != (uint16_t)expect)
      continue;
    if (!(vis_mask & (uint16_t)(1u << r)))
      continue;
    run_mask |= (uint16_t)(1u << r);
  }
  if (!run_mask)
    return;

  /* Idle strip = 22 words at column 0 of each map row (see MwNotifyBg2MapDma). */
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, 1);
  for (unsigned rom_row = 0; rom_row < (unsigned)kMwBg2RomRows; rom_row++) {
    if (!(run_mask & (uint16_t)(1u << rom_row)))
      continue;
    const uint8_t bank = s_bg2_rom_bank[rom_row];
    const uint16_t base = s_bg2_rom_addr[rom_row];
    const int have_words =
        (s_bg2_rom_words_mask & (uint16_t)(1u << rom_row)) != 0;
    for (int col = 0; col < kMwBg2RomCols; col++) {
      const uint16_t t =
          have_words ? s_bg2_rom_words[rom_row][col]
                     : mw_cart_read16(bank,
                                      (uint16_t)(base + (uint32_t)col * 2u));
      const uint16_t vaddr =
          (uint16_t)(map_base + ((int)rom_row << 5) + col);
      mw_vram_save_word(vaddr);
      g_ppu->vram[vaddr & 0x7fffu] = t;
    }
  }
  s_coldump_bg2_restamp = 1;
}

/* Present-only: adjust OAM Y. Skip already-culled (Y≥$F0).
 * Do NOT wrap small negative Y into 192..255 — the PPU's uint8 (line−Y) then
 * draws those sprites on the bottom scanlines (P1 mechs/items "rising" from
 * the bottom of P2's full-frame view). Park off-screen instead. */
static void mw_oam_y_add(int delta) {
  if (!g_ppu || delta == 0)
    return;
  for (int i = 0; i < 0x100; i += 2) {
    const uint8_t x = (uint8_t)(g_ppu->oam[i] & 0xff);
    const uint8_t y = (uint8_t)(g_ppu->oam[i] >> 8);
    if (y >= 0xf0u)
      continue;
    int ny = (int)y + delta;
    /* Full-frame present is 224 lines — hide anything above or past the bottom. */
    if (ny < 0 || ny >= 224)
      ny = 0xf0;
    g_ppu->oam[i] = (uint16_t)x | ((uint16_t)(uint8_t)ny << 8);
  }
}

static uint16_t mw_u16_sub_sat(uint16_t v, int amount) {
  if (amount <= 0)
    return v;
  if ((int)v <= amount)
    return 0;
  return (uint16_t)((int)v - amount);
}

/* Dual cams frame the mech for a ~112-line half; full 224 needs +(224-half)/2
 * screen Y (subtract from cam/vscroll). Present-only — sim unchanged. */
static int mw_h2h_full_frame_y_shift(void) {
  int split_y = (int)mw_wram16(0x1EA0);
  if (split_y <= 0 || split_y >= 224)
    split_y = 0x70;
  return (224 - split_y) / 2;
}

/* Dual emit Y bias (+$78). Present undoes this for whichever local buffer
 * is shown (both cams store biased Y under vert-widen). */
static int mw_h2h_oam_y_bias(void) {
  return mw_h2h_vert_widen_armed() ? 0x78 : 0;
}

static int mw_h2h_oam_cull_armed(void) {
  static int v = -1;
  if (v < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_OAM_CULL");
    if (e && e[0] == '0')
      v = 0;
    else
      v = 1; /* default on — hard-cull other dual half */
  }
  return v;
}

/* Present-only 1P object-list OAM (solution 2). Default OFF — earlier
 * incomplete WRAM restore after $8086B6 blanked the screen. Opt in:
 * SNESRECOMP_MW_H2H_OBJ_OAM=1 (use to A/B cam-capture/reproject bugs). */
static int mw_h2h_obj_oam_armed(void) {
  static int v = -1;
  if (v < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_OBJ_OAM");
    if (e && e[0] == '1')
      v = 1;
    else
      v = 0;
  }
  return v;
}

/* Staging $14C4 (4 bytes/sprite) + $16C4 highOam → PPU OAM. */
static void mw_staging_to_ppu_oam(void) {
  if (!g_ppu)
    return;
  for (int slot = 0; slot < 128; slot++) {
    const unsigned b = 0x14C4u + (unsigned)slot * 4u;
    const uint8_t x = g_ram[b];
    const uint8_t y = g_ram[b + 1u];
    const uint8_t tile = g_ram[b + 2u];
    const uint8_t attr = g_ram[b + 3u];
    g_ppu->oam[slot * 2] = (uint16_t)x | ((uint16_t)y << 8);
    g_ppu->oam[slot * 2 + 1] = (uint16_t)tile | ((uint16_t)attr << 8);
  }
  memcpy(g_ppu->highOam, &g_ram[0x16C4], 0x20);
}

/*
 * Present-only: run 1P active-list builder+drawer ($8086B6) against the local
 * camera so meta-sprites use the full [-64,320) Y window (not dual #$A8).
 * Backs up / restores WRAM + CPU so sim stays dual-deterministic.
 * Returns 1 if PPU OAM was replaced from staging.
 */
static int mw_present_oam_from_objects(uint16_t cam_x, uint16_t cam_y_raw) {
  if (!mw_h2h_obj_oam_armed() || !g_ppu || !MwIsDualViewport())
    return 0;

  CpuState cpu_bak = g_cpu;
  /* $8086B6: DP scratch, $136E active-list, $14C4..$18FF staging/dual-aux,
   * $1934 object pool, $1E00 camera/stripe mirrors. $8B01 also walks $17C4
   * when DP $78 ≠ 0 — force $78=0 so the dual merge is skipped. */
  uint8_t low_bak[0x200];
  uint8_t list_bak[0x1C4]; /* $1300..$14C3 incl. active list at $136E */
  uint8_t stage_bak[0x43C]; /* $14C4..$18FF */
  uint8_t pool_bak[24 * 0x1C];
  uint8_t mid_bak[0x200];
  memcpy(low_bak, g_ram, sizeof(low_bak));
  memcpy(list_bak, &g_ram[0x1300], sizeof(list_bak));
  memcpy(stage_bak, &g_ram[0x14C4], sizeof(stage_bak));
  memcpy(pool_bak, &g_ram[0x1934], sizeof(pool_bak));
  memcpy(mid_bak, &g_ram[0x1E00], sizeof(mid_bak));

  mw_wram16_write(0x1E16u, cam_x);
  mw_wram16_write(0x1E18u, cam_y_raw);
  /* Keep dual mirrors coherent if any path peeks P2 cam during the draw. */
  mw_wram16_write(0x1E1Au, cam_x);
  mw_wram16_write(0x1E1Cu, cam_y_raw);
  g_ram[0x78] = 0;
  g_ram[0x79] = 0;

  cpu_push_jsr_return_frame(&g_cpu);
  g_cpu.PB = 0x80;
  const int ok = interp_bridge_run(&g_cpu, MW_FN_MwObjectDrawer);
  if (ok)
    mw_staging_to_ppu_oam();

  memcpy(g_ram, low_bak, sizeof(low_bak));
  memcpy(&g_ram[0x1300], list_bak, sizeof(list_bak));
  memcpy(&g_ram[0x14C4], stage_bak, sizeof(stage_bak));
  memcpy(&g_ram[0x1934], pool_bak, sizeof(pool_bak));
  memcpy(&g_ram[0x1E00], mid_bak, sizeof(mid_bak));
  g_cpu = cpu_bak;

  if (ok) {
    extern int snes_frame_counter;
    /* Object-drawer path skips cam_capture — still latch present for coldump. */
    s_coldump_present_slot = s_present_h2h_local_slot >= 0
                                 ? s_present_h2h_local_slot
                                 : 0;
    s_coldump_present_frame = snes_frame_counter;
    static int logged;
    if (!logged) {
      logged = 1;
      fprintf(stderr,
              "[mw_rtl] H2H present OAM from 1P object drawer ($8086B6) "
              "cam=$%04X/$%04X (SNESRECOMP_MW_H2H_OBJ_OAM=0 to disable)\n",
              (unsigned)cam_x, (unsigned)cam_y_raw);
    }
  }
  return ok;
}

/* Decode 9-bit OAM X to screen X. [256, 256+extra) stays positive (right
 * widescreen margin); [256+extra, 512) wraps to the left straddle band. */
static int mw_oam_screen_x_from_9bit(int x9, int extra) {
  if (extra < 0)
    extra = 0;
  if (x9 >= 256 + extra)
    return x9 - 512;
  return x9;
}

/* Ambiguous-band-aware signed X for cam-capture. Uses staging right-hints
 * collected by sprite/particle hooks; unmarked [256, 256+extra) → left wrap. */
static int mw_oam_signed_sx_from_9bit(int x9, int extra, unsigned staging_slot) {
  if (extra < 0)
    extra = 0;
  if (x9 < 256)
    return x9;
  if (x9 >= 256 + extra)
    return x9 - 512;
  if (staging_slot < 128u &&
      (s_oam_right_hints[staging_slot >> 3] & (uint8_t)(1u << (staging_slot & 7u))))
    return x9; /* genuine right margin */
  return x9 - 512; /* parked / left-wrap */
}

static int mw_ppu_oam_read_x9(int slot) {
  const int idx = slot * 2;
  int x = (int)(g_ppu->oam[idx] & 0xff);
  x |= ((int)(g_ppu->highOam[slot >> 2] >> ((slot & 3) * 2)) & 1) << 8;
  return x;
}

static int mw_ppu_oam_read_x(int slot, int extra) {
  return mw_oam_screen_x_from_9bit(mw_ppu_oam_read_x9(slot), extra);
}

static int mw_cam_find_xy(int cam, int sx, int sy, uint8_t tile, int tol);

/* Append one finished staging sprite (X/Y already stored; A = tile|attr).
 * Camera = s_oam_draw_cam from ADC $86/$88 vs $8A/$8C; stage props may
 * re-pick cam by matching object world to cam+screen (mistag repair).
 * Prefers pending pre-bias sy from the STA $14C5 hook so bottom tiles that
 * park staging Y at $F0 (biased ≥ $F0) still reach full-frame present. */
static void mw_cam_oam_commit(unsigned src_off, uint16_t tile_attr) {
  if (!mw_h2h_vert_widen_armed() || !MwIsDualViewport())
    return;
  if (src_off & 3u || src_off >= 128u * 4u)
    return;

  const uint16_t y_meta_early = mw_wram16(0x0082u);
  const uint16_t y_bank_early = mw_wram16(0x0084u);
  const int shared_item_early =
      mw_obj_is_shared_b1_item(s_draw_obj_latched) ||
      (y_bank_early == 0x00B1u && mw_is_shared_b1_item_meta(y_meta_early));
  const int b1_wide_early =
      (y_bank_early == 0x00B1u && y_meta_early != 0 &&
       y_meta_early != 0xD5B8u);
  const int prop_cand_early =
      mw_obj_is_stage_prop(s_draw_obj_latched) ||
      s_pending_stage_prop_local_only ||
      (y_bank_early == 0x00B1u && mw_is_stage_prop_meta(y_meta_early)) ||
      b1_wide_early;

  int16_t sy;
  int sy_from_obj = 0;
  if (s_pending_sy_valid) {
    sy = s_pending_sy;
    s_pending_sy_valid = 0;
  } else {
    /* Fallback: staging byte is biased. */
    const uint8_t y = g_ram[0x14C5u + src_off];
    if (y >= 0xf0u) {
      /* Parked $F0 — still capture movers/items from object world Y once
       * cams + prop_obj are known (pending_sy miss used to drop them). */
      if (!prop_cand_early && !shared_item_early)
        return;
      sy_from_obj = 1;
      sy = 0;
    } else {
      sy = (int16_t)((int)y - 0x78);
    }
  }
  /*
   * Vert-widen window is −144…#$E0 for gameplay. Stage props / shared $B1
   * items may convert or sit far off the half-frame — still capture so
   * present can rebuild (movers: live +$02/+$04; items: multi-tile sticky).
   * Movers above the dual cams (sy≈−300) need the same −512 floor as items.
   */
  if (!sy_from_obj) {
    const int y_lo =
        (shared_item_early || prop_cand_early) ? -512 : -144;
    const int y_hi =
        (prop_cand_early || shared_item_early) ? 0x120 : 0xE0;
    if (sy < y_lo || sy >= y_hi)
      return;
  }

  const unsigned src_slot = src_off / 4u;
  const unsigned src_byte = 0x16C4u + (src_slot >> 2);
  const unsigned src_sh = (src_slot & 3u) * 2u;
  int x9 = (int)g_ram[0x14C4u + src_off];
  x9 |= ((int)(g_ram[src_byte] >> src_sh) & 1) << 8;
  const int extra =
      (g_ws_active && g_ws_extra > 0) ? IntMin(g_ws_extra, kWsExtraMax) : 0;
  const int sx = mw_oam_signed_sx_from_9bit(x9, extra, src_slot);
  /*
   * Drop draw-cam offscreen X so far shots aren't kept as wrap ghosts.
   * Stage props / shared $B1 items: widen so sibling meta tiles that sit
   * just past the 256px edge still reach multi-tile sticky (half-off movers
   * at sx≈261 used to keep one tile and permanently drop the rest).
   */
  {
    const uint16_t y_meta = mw_wram16(0x0082u);
    const uint16_t y_bank = mw_wram16(0x0084u);
    const int wide_x =
        mw_obj_is_stage_prop(s_draw_obj_latched) ||
        s_pending_stage_prop_local_only ||
        mw_obj_is_shared_b1_item(s_draw_obj_latched) ||
        (y_bank == 0x00B1u && y_meta != 0 && y_meta != 0xD5B8u);
    const int x_lo = wide_x ? (-extra - 256) : (-extra - 64);
    const int x_hi = wide_x ? (256 + extra + 256) : (256 + extra + 64);
    if (sx + 64 < x_lo || sx > x_hi)
      return;
  }

  const uint16_t cam0x = s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16u);
  const uint16_t cam0y = s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18u);
  const uint16_t cam1x = s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1Au);
  const uint16_t cam1y = s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1Cu);

  const int draw_cam = (s_oam_draw_cam == 1) ? 1 : 0;
  int cam = draw_cam;
  int sx_i = sx;
  int sy_i = (int)sy;
  /* Drawer screen before home convert — mox keys off this + live WRAM cam. */
  const int sx_draw = sx_i;
  const int sy_draw = sy_i;
  uint16_t wx = (uint16_t)((int32_t)(cam ? cam1x : cam0x) + sx_i);
  uint16_t wy = (uint16_t)((int32_t)(cam ? cam1y : cam0y) + sy_i);
  /*
   * Stage props ($00B1≠$D5B8): recover object from latch and/or dual-list
   * $96→$136E (tile emit replaces X with flags&6). Home cam = hi-byte +$06
   * or nearer dual camera — convert sx/sy into that buffer so the far peer
   * does not present a ghost (dual drawer emits into both cams).
   */
  int prop_owner = -1;
  uint16_t prop_obj = s_draw_obj_latched;
  if (!mw_obj_is_stage_prop(prop_obj)) {
    const uint16_t list_obj = mw_dual_draw_list_obj();
    if (mw_obj_is_stage_prop(list_obj)) {
      prop_obj = list_obj;
      s_draw_obj_latched = list_obj;
      s_prop_stat_list_rec++;
    }
  }
  const uint16_t meta_lo = mw_wram16(0x0082u);
  const uint16_t meta_hi = mw_wram16(0x0084u);
  int local_only = 0;
  if (mw_obj_is_stage_prop(prop_obj)) {
    local_only = 1;
    s_pending_stage_prop_local_only = 1;
  } else if (s_pending_stage_prop_local_only ||
             (meta_hi == 0x00B1u && mw_is_stage_prop_meta(meta_lo))) {
    /* Require real stage-prop meta — do not tag every $00B1 bank draw. */
    local_only = 1;
    s_pending_stage_prop_local_only = 1;
  }
  /*
   * Recover mover by object +$08 == live meta $82 only. Require both X and Y
   * near the object origin — X-only (adx≤48) stole mech tiles on $C6A4 and
   * latched mox≈166 into sticky (jitter chunks that followed P1).
   */
  /* Never re-home shared $B1 pickups onto a nearby mover (steals item OAM). */
  if (!mw_obj_is_stage_prop(prop_obj) && meta_hi == 0x00B1u &&
      mw_is_stage_prop_meta(meta_lo) &&
      !mw_obj_is_shared_b1_item(s_draw_obj_latched) &&
      !mw_is_shared_b1_item_meta(meta_lo)) {
    const uint16_t dcx = draw_cam ? cam1x : cam0x;
    const uint16_t dcy = draw_cam ? cam1y : cam0y;
    uint16_t best = 0;
    int best_score = 0x7fffffff;
    uint16_t idx = mw_wram16(0x1E14u);
    for (int guard = 0; guard < 64 && idx; guard++) {
      const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
      if (mw_obj_is_stage_prop(idx)) {
        const uint16_t om = mw_wram16((uint16_t)(idx + 8u));
        if (om == meta_lo) {
          const int psx = (int)mw_wram16((uint16_t)(idx + 2u)) - (int)dcx;
          const int psy = (int)mw_wram16((uint16_t)(idx + 4u)) - (int)dcy;
          const int adx = mw_i32_abs(psx - sx_i);
          const int ady = mw_i32_abs(psy - sy_i);
          /* $C6A4: tight box + $E4-only (shots/items sit inside the old 24×40). */
          const int c6 = (om == 0xC6A4u);
          const int lim_x = c6 ? 16 : 24;
          const int lim_y = c6 ? 20 : 40;
          if (c6 && !mw_prop_c6a4_sticky_tile_ok((uint8_t)(tile_attr & 0xffu)))
            continue;
          if (adx <= lim_x && ady <= lim_y) {
            const int score = adx + ady;
            if (score < best_score) {
              best_score = score;
              best = idx;
            }
          }
        }
      }
      if (next == 0 || next == idx)
        break;
      idx = next;
    }
    if (best) {
      prop_obj = best;
      s_draw_obj_latched = best;
      local_only = 1;
      s_pending_stage_prop_local_only = 1;
      s_prop_stat_list_rec++;
    }
  }
  /*
   * $C6A4 + non-$E4 tile: demote out of prop local_only. Sticky rejects the
   * tile while present skips prop_obj≠0 → shots/items vanished or froze.
   */
  {
    const uint8_t draw_tile = (uint8_t)(tile_attr & 0xffu);
    const int c6_context =
        mw_prop_is_c6a4_rider(prop_obj) ||
        (meta_hi == 0x00B1u && meta_lo == 0xC6A4u);
    if (c6_context && !mw_prop_c6a4_sticky_tile_ok(draw_tile)) {
      if (mw_obj_is_shared_b1_item(s_draw_obj_latched))
        prop_obj = s_draw_obj_latched;
      else
        prop_obj = 0;
      local_only = 0;
      s_pending_stage_prop_local_only = 0;
    }
  }
  /*
   * Staging Y was $F0 (no pending_sy): rebuild screen Y from object world
   * vs the drawer cam so far-above / far-below movers still sticky-capture.
   */
  if (sy_from_obj) {
    if (!mw_obj_is_stage_prop(prop_obj) && !mw_obj_is_shared_b1_item(prop_obj))
      return;
    const uint16_t owy = mw_wram16((uint16_t)(prop_obj + 4u));
    const uint16_t dcy = draw_cam ? cam1y : cam0y;
    sy = (int16_t)((int)owy - (int)dcy);
    sy_i = (int)sy;
    if (sy < -512 || sy >= 0x120)
      return;
    wx = (uint16_t)((int32_t)(draw_cam ? cam1x : cam0x) + sx_i);
    wy = (uint16_t)((int32_t)dcy + sy_i);
  }
  if (local_only) {
    const int have_obj = mw_obj_is_stage_prop(prop_obj);
    prop_owner =
        have_obj ? mw_stage_prop_home_cam(prop_obj, cam0x, cam0y, cam1x, cam1y)
                 : -1;
    int want = cam;
    uint16_t owx = 0;
    if (have_obj)
      owx = mw_wram16((uint16_t)(prop_obj + 2u));
    /*
     * Only rebucket when home is known. Unassigned (-1) keeps drawer cam
     * but present/BG will suppress until sticky latches — avoids dual-ghost
     * and convert thrash from cam-distance heuristics.
     */
    if (prop_owner >= 0)
      want = prop_owner;
    if (want != draw_cam) {
      const uint16_t ocx = draw_cam ? cam1x : cam0x;
      const uint16_t ocy = draw_cam ? cam1y : cam0y;
      const uint16_t ncx = want ? cam1x : cam0x;
      const uint16_t ncy = want ? cam1y : cam0y;
      sx_i += (int)ocx - (int)ncx;
      sy_i += (int)ocy - (int)ncy;
      s_prop_stat_convert++;
    }
    cam = want;
    if (have_obj) {
      const int32_t offx =
          (int32_t)sx_i - ((int32_t)owx - (int32_t)(cam ? cam1x : cam0x));
      wx = (uint16_t)((int32_t)owx + offx);
    } else {
      wx = (uint16_t)((int32_t)(cam ? cam1x : cam0x) + sx_i);
    }
    wy = (uint16_t)((int32_t)(cam ? cam1y : cam0y) + sy_i);
    s_prop_stat_commit++;
    if (getenv("SNESRECOMP_MW_ELEV") && (s_prop_stat_commit <= 48u ||
                                         (s_prop_stat_commit % 120u) == 0u)) {
      fprintf(stderr,
              "[mw_prop] commit obj=$%04X owx=$%04X +6=$%04X owner=%d "
              "draw=%d sx=%d sy=%d → cam%d sx'=%d sy'=%d wx=$%04X wy=$%04X\n",
              (unsigned)prop_obj, (unsigned)owx,
              have_obj ? (unsigned)mw_wram16((uint16_t)(prop_obj + 6u)) : 0u,
              prop_owner, draw_cam, sx, (int)sy, cam, sx_i, sy_i, (unsigned)wx,
              (unsigned)wy);
    }
  }

  /* Dual drawer emits the same prop into both cams; purge so home keeps one. */
  if (local_only && mw_obj_is_stage_prop(prop_obj) &&
      s_prop_purge_arm == prop_obj) {
    mw_cam_purge_prop_obj(prop_obj);
    s_prop_purge_arm = 0;
  }

  if (s_cam_n[cam] >= 128u)
    return;

  const unsigned dst = s_cam_n[cam]++;
  s_cam_spr[cam][dst][0] = g_ram[0x14C4u + src_off];
  s_cam_spr[cam][dst][1] = g_ram[0x14C5u + src_off]; /* biased / parked */
  s_cam_spr[cam][dst][2] = (uint8_t)(tile_attr & 0xffu);
  s_cam_spr[cam][dst][3] = (uint8_t)(tile_attr >> 8);
  s_cam_sx[cam][dst] = (int16_t)sx_i;
  s_cam_sy[cam][dst] = (int16_t)sy_i;
  s_cam_wx[cam][dst] = wx;
  s_cam_wy[cam][dst] = wy;
  s_cam_local_only[cam][dst] = local_only ? 1u : 0u;
  s_cam_shared_item[cam][dst] = 0;
  s_cam_prop_owner[cam][dst] = local_only ? (int8_t)prop_owner : (int8_t)-1;
  if (local_only && mw_obj_is_stage_prop(prop_obj) &&
      !(mw_prop_is_c6a4_rider(prop_obj) &&
        !mw_prop_c6a4_sticky_tile_ok((uint8_t)(tile_attr & 0xffu)))) {
    const uint16_t owx = mw_wram16((uint16_t)(prop_obj + 2u));
    const uint16_t owy = mw_wram16((uint16_t)(prop_obj + 4u));
    /*
     * Object-local meta vs *live* WRAM drawer cam ($1E16/$1E1A). Never use
     * post-convert sx_i. Seed only home drawer, or empty+in-FOV+sane meta —
     * unrestricted empty-seed latched mech tiles (mox≈166) that followed P1.
     */
    const uint16_t live_cx =
        draw_cam ? mw_wram16(0x1E1Au) : mw_wram16(0x1E16u);
    const uint16_t live_cy =
        draw_cam ? mw_wram16(0x1E1Cu) : mw_wram16(0x1E18u);
    const int16_t mox =
        (int16_t)(sx_draw - ((int)owx - (int)live_cx));
    int16_t moy = (int16_t)(sy_draw - ((int)owy - (int)live_cy));
    s_cam_prop_obj[cam][dst] = prop_obj;
    s_cam_prop_meta_ox[cam][dst] = mox;
    s_cam_prop_meta_oy[cam][dst] = moy;
    {
      const uint8_t spr[4] = {
          g_ram[0x14C4u + src_off], g_ram[0x14C5u + src_off],
          (uint8_t)(tile_attr & 0xffu), (uint8_t)(tile_attr >> 8)};
      const uint8_t sz = (uint8_t)((g_ram[src_byte] >> src_sh) & 2u);
      if (moy == 0)
        moy = -10;
      /*
       * Meta seed only from the true home drawer. Empty+foreign-FOV seed
       * latched P1-viewport-clipped tiles that P2 then drew (same bottom
       * cut as P1's screen edge).
       */
      const int seed = mw_prop_meta_sane((int)mox, (int)moy) &&
                       (prop_owner >= 0 && prop_owner == draw_cam);
      /* sticky_store no-ops non-$E4 for $C6A4. */
      if (seed)
        mw_prop_sticky_store(prop_obj, spr, sz, mox, moy, 1);
      else
        mw_prop_sticky_store(prop_obj, spr, sz, mox, moy, 0);
    }
  } else {
    s_cam_prop_obj[cam][dst] = 0;
    s_cam_prop_meta_ox[cam][dst] = 0;
    s_cam_prop_meta_oy[cam][dst] = 0;
    /* Demoted $C6A4 non-$E4 / non-prop: must not skip gameplay present. */
    if (!mw_obj_is_stage_prop(prop_obj) ||
        (mw_prop_is_c6a4_rider(prop_obj) &&
         !mw_prop_c6a4_sticky_tile_ok((uint8_t)(tile_attr & 0xffu))))
      s_cam_local_only[cam][dst] = 0;
  }
  s_cam_capture_frame = 1;

  const uint8_t src_h = (uint8_t)((g_ram[src_byte] >> src_sh) & 3u);
  const unsigned dst_byte = dst >> 2;
  const unsigned dst_sh = (dst & 3u) * 2u;
  s_cam_high[cam][dst_byte] = (uint8_t)(
      (s_cam_high[cam][dst_byte] & ~(uint8_t)(3u << dst_sh)) |
      (uint8_t)(src_h << dst_sh));

  mw_oam_cam_tag_ensure_init();
  if (src_slot < 128u)
    s_oam_cam_tag[src_slot] = (uint8_t)cam;

  /*
   * Shared bank-$B1 items (pickups/crates, not movers): dual drawer emits
   * each tile into cam0 then cam1; OAM pressure often drops half the
   * sprite. Mirror into the other cam AND accumulate multi-tile sticky
   * with world meta offsets — present places the full set for both peers.
   */
  if (!local_only && meta_hi == 0x00B1u &&
      mw_is_shared_b1_item_meta(meta_lo)) {
    uint16_t item_obj = prop_obj;
    if (!mw_obj_is_shared_b1_item(item_obj)) {
      const uint16_t list_obj = mw_dual_draw_list_obj();
      if (mw_obj_is_shared_b1_item(list_obj))
        item_obj = list_obj;
      else {
        uint16_t idx = mw_wram16(0x1E14u);
        uint16_t best = 0;
        int best_score = 0x7fffffff;
        const uint16_t dcx = cam ? cam1x : cam0x;
        for (int guard = 0; guard < 64 && idx; guard++) {
          const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
          if (mw_obj_is_shared_b1_item(idx) &&
              mw_wram16((uint16_t)(idx + 8u)) == meta_lo) {
            const int psx =
                (int)mw_wram16((uint16_t)(idx + 2u)) - (int)dcx;
            const int adx = mw_i32_abs(psx - sx_i);
            if (adx < best_score) {
              best_score = adx;
              best = idx;
            }
          }
          if (next == 0 || next == idx)
            break;
          idx = next;
        }
        if (best)
          item_obj = best;
      }
    }
    if (mw_obj_is_shared_b1_item(item_obj)) {
      const uint16_t owx = mw_wram16((uint16_t)(item_obj + 2u));
      const uint16_t owy = mw_wram16((uint16_t)(item_obj + 4u));
      const uint16_t cx = cam ? cam1x : cam0x;
      const uint16_t cy = cam ? cam1y : cam0y;
      const int16_t mox = (int16_t)(sx_i - ((int)owx - (int)cx));
      const int16_t moy = (int16_t)(sy_i - ((int)owy - (int)cy));
      const uint8_t spr[4] = {
          g_ram[0x14C4u + src_off], g_ram[0x14C5u + src_off],
          (uint8_t)(tile_attr & 0xffu), (uint8_t)(tile_attr >> 8)};
      const uint8_t sz = (uint8_t)((g_ram[src_byte] >> src_sh) & 2u);
      mw_item_sticky_store(item_obj, spr, sz, mox, moy);
      /* Tag capture tiles so present skips raw halves — sticky only. */
      s_cam_shared_item[cam][dst] = 1;
      s_cam_prop_obj[cam][dst] = item_obj;
    }
    const int ocam = cam ^ 1;
    if (s_cam_n[ocam] < 128u) {
      const uint16_t ccx = cam ? cam1x : cam0x;
      const uint16_t ccy = cam ? cam1y : cam0y;
      const uint16_t ocx = ocam ? cam1x : cam0x;
      const uint16_t ocy = ocam ? cam1y : cam0y;
      const int osx = sx_i + (int)ccx - (int)ocx;
      const int osy = sy_i + (int)ccy - (int)ocy;
      const uint8_t tile = (uint8_t)(tile_attr & 0xffu);
      if (!mw_cam_find_xy(ocam, osx, osy, tile, 2)) {
        const unsigned odst = s_cam_n[ocam]++;
        s_cam_spr[ocam][odst][0] = g_ram[0x14C4u + src_off];
        s_cam_spr[ocam][odst][1] = g_ram[0x14C5u + src_off];
        s_cam_spr[ocam][odst][2] = tile;
        s_cam_spr[ocam][odst][3] = (uint8_t)(tile_attr >> 8);
        s_cam_sx[ocam][odst] = (int16_t)osx;
        s_cam_sy[ocam][odst] = (int16_t)osy;
        s_cam_wx[ocam][odst] =
            (uint16_t)((int32_t)ocx + osx);
        s_cam_wy[ocam][odst] =
            (uint16_t)((int32_t)ocy + osy);
        s_cam_local_only[ocam][odst] = 0;
        s_cam_shared_item[ocam][odst] =
            mw_obj_is_shared_b1_item(item_obj) ? 1u : 0u;
        s_cam_prop_owner[ocam][odst] = (int8_t)-1;
        s_cam_prop_obj[ocam][odst] =
            mw_obj_is_shared_b1_item(item_obj) ? item_obj : 0;
        s_cam_prop_meta_ox[ocam][odst] = 0;
        s_cam_prop_meta_oy[ocam][odst] = 0;
        {
          const unsigned ob = odst >> 2;
          const unsigned osh = (odst & 3u) * 2u;
          s_cam_high[ocam][ob] = (uint8_t)(
              (s_cam_high[ocam][ob] & ~(uint8_t)(3u << osh)) |
              (uint8_t)(src_h << osh));
        }
      }
    }
  }
}

static void mw_h2h_vw_oam_commit_x_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_h2h_vert_widen_armed() || !MwIsDualViewport())
    return;
  mw_cam_oam_commit((unsigned)cpu->X, cpu->A);
}

static void mw_h2h_vw_oam_commit_y_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_h2h_vert_widen_armed() || !MwIsDualViewport())
    return;
  mw_cam_oam_commit((unsigned)cpu->Y, cpu->A);
}

/* Encode screen X to 9-bit OAM (inverse of mw_oam_screen_x_from_9bit).
 * Only valid for sx in [- (512-(256+extra)), 256+extra). Callers must cull
 * past the non-wrap band or the PPU will show a left-edge wrap ghost. */
static int mw_oam_x9_from_screen(int sx, int extra) {
  if (extra < 0)
    extra = 0;
  if (sx < 0)
    return sx + 512;
  return sx;
}

static int mw_present_oam_dup_tol(const int *xs, const int *ys,
                                  const uint8_t *tiles, unsigned n, int x,
                                  int y, uint8_t tile, int tol) {
  for (unsigned i = 0; i < n; i++) {
    if (tiles[i] != tile)
      continue;
    const int dx = xs[i] - x;
    const int dy = ys[i] - y;
    if (dx <= tol && dx >= -tol && dy <= tol && dy >= -tol)
      return 1;
  }
  return 0;
}

static int mw_present_oam_dup(const int *xs, const int *ys, const uint8_t *tiles,
                              unsigned n, int x, int y, uint8_t tile) {
  return mw_present_oam_dup_tol(xs, ys, tiles, n, x, y, tile, 6);
}

/* End-match results OAM: OBJ prio 3 (dumps show a34/a35). Gameplay uses
 * prio 0–2 (a20/a22/a2C/…). Do NOT use mutual screen-XY — results split
 * across cam0/cam1 at different positions (mutual=0). */
static int mw_oam_is_results_ui(uint8_t attr) {
  return (attr & 0x30u) == 0x30u;
}

/*
 * Stage-prop OAM Y — world-lock only: live_oy − loc + moy + y_shift.
 * Do NOT fall back to capture sy (that is already screen-space for the
 * drawer cam): when live_ny left [0,224) the stripe stayed parked on screen
 * and "followed" the local camera while BG1/world moved.
 * Returns −1 if off-screen.
 */
static int mw_prop_present_ny(int have_cap, int cap_sy, int live_oy, int loc_y,
                              int moy, int y_shift) {
  (void)have_cap;
  (void)cap_sy;
  const int live_ny = (live_oy - loc_y) + moy + y_shift;
  if (live_ny >= 0 && live_ny < 224)
    return live_ny;
  return -1;
}

static int mw_present_oam_place(int x, int ny, const uint8_t *sp,
                                uint8_t size_bit, int extra,
                                uint8_t *right_hints, int *kept_x, int *kept_y,
                                uint8_t *kept_tile, unsigned *kept, int tol) {
  if (!g_ppu || !sp || !kept || *kept >= 128u)
    return 0;
  if (ny < 0 || ny >= 224)
    return 0;
  if (mw_present_oam_dup_tol(kept_x, kept_y, kept_tile, *kept, x, ny, sp[2],
                             tol))
    return 0;
  const int lx9 = mw_oam_x9_from_screen(x, extra);
  const unsigned dst = *kept;
  const int idx = (int)dst * 2;
  g_ppu->oam[idx] =
      (uint16_t)(lx9 & 0xff) | ((uint16_t)(uint8_t)ny << 8);
  g_ppu->oam[idx + 1] = (uint16_t)sp[2] | ((uint16_t)sp[3] << 8);
  const uint8_t hbits =
      (uint8_t)((size_bit & 2u) | (uint8_t)((lx9 >> 8) & 1));
  const unsigned dst_byte = dst >> 2;
  const unsigned dst_sh = (dst & 3u) * 2u;
  g_ppu->highOam[dst_byte] = (uint8_t)(
      (g_ppu->highOam[dst_byte] & ~(uint8_t)(3u << dst_sh)) |
      (uint8_t)(hbits << dst_sh));
  if (x >= 256 && x < 256 + extra)
    right_hints[dst >> 3] |= (uint8_t)(1u << (dst & 7u));
  kept_x[dst] = x;
  kept_y[dst] = ny;
  kept_tile[dst] = sp[2];
  (*kept)++;
  return 1;
}

/* Match tile in cam buffer at (sx,sy) within tol. Returns 1 if found. */
static int mw_cam_find_xy(int cam, int sx, int sy, uint8_t tile, int tol) {
  if (cam != 0 && cam != 1)
    return 0;
  for (unsigned i = 0; i < s_cam_n[cam]; i++) {
    if (s_cam_spr[cam][i][2] != tile)
      continue;
    const int dx = (int)s_cam_sx[cam][i] - sx;
    const int dy = (int)s_cam_sy[cam][i] - sy;
    if (dx <= tol && dx >= -tol && dy <= tol && dy >= -tol)
      return 1;
  }
  return 0;
}

/*
 * End-match / screen-fixed UI probe (SNESRECOMP_MW_RESULTS=1).
 * Present-only — no behavior change. Dumps cam-capture layout so we can see
 * whether results OAM is mutual raw-XY, one-cam-only, or only matches after
 * cam-delta reproject (world-like). Run through an end-of-match screen and
 * compare lines tagged [mw_results].
 */
static void mw_results_dump(int local_slot, int y_shift, uint16_t loc_x,
                            uint16_t loc_y, uint16_t oth_x, uint16_t oth_y) {
  static int armed = -1;
  if (armed < 0) {
    const char *e = getenv("SNESRECOMP_MW_RESULTS");
    armed = (e && e[0] && e[0] != '0') ? 1 : 0;
    if (armed)
      fprintf(stderr,
              "[mw_results] armed — cam-capture UI probe (~2 Hz). Look for "
              "mutual_raw vs match_reproj vs one-cam-only on end-match.\n");
  }
  if (!armed)
    return;

  extern int snes_frame_counter;
  static int last_f = -1;
  if (!MwIsDualViewport())
    return;
  if ((snes_frame_counter % 30) != 0 || snes_frame_counter == last_f)
    return;
  last_f = snes_frame_counter;

  const int other = local_slot ^ 1;
  unsigned mutual0 = 0, mutual6 = 0, match_reproj = 0, ui_n = 0;
  int b0x0 = 512, b0x1 = -512, b0y0 = 512, b0y1 = -512;
  int b1x0 = 512, b1x1 = -512, b1y0 = 512, b1y1 = -512;
  long sum0x = 0, sum0y = 0, sum1x = 0, sum1y = 0;

  for (unsigned i = 0; i < s_cam_n[0]; i++) {
    const int sx = (int)s_cam_sx[0][i];
    const int sy = (int)s_cam_sy[0][i];
    const uint8_t tile = s_cam_spr[0][i][2];
    if (mw_oam_is_results_ui(s_cam_spr[0][i][3]))
      ui_n++;
    sum0x += sx;
    sum0y += sy;
    if (sx < b0x0)
      b0x0 = sx;
    if (sx > b0x1)
      b0x1 = sx;
    if (sy < b0y0)
      b0y0 = sy;
    if (sy > b0y1)
      b0y1 = sy;
    if (mw_cam_find_xy(1, sx, sy, tile, 0))
      mutual0++;
    if (mw_cam_find_xy(1, sx, sy, tile, 6))
      mutual6++;
  }
  for (unsigned i = 0; i < s_cam_n[1]; i++) {
    const int sx = (int)s_cam_sx[1][i];
    const int sy = (int)s_cam_sy[1][i];
    if (mw_oam_is_results_ui(s_cam_spr[1][i][3]))
      ui_n++;
    sum1x += sx;
    sum1y += sy;
    if (sx < b1x0)
      b1x0 = sx;
    if (sx > b1x1)
      b1x1 = sx;
    if (sy < b1y0)
      b1y0 = sy;
    if (sy > b1y1)
      b1y1 = sy;
  }
  /* Reproject test: other → local. Count tiles that match a local sprite
   * only after cam delta (not at raw XY). */
  for (unsigned i = 0; i < s_cam_n[other]; i++) {
    const int ox = (int)s_cam_sx[other][i];
    const int oy = (int)s_cam_sy[other][i];
    const uint8_t tile = s_cam_spr[other][i][2];
    if (mw_cam_find_xy(local_slot, ox, oy, tile, 6))
      continue; /* raw match — screen-fixed / dual half */
    const int32_t world_x = (int32_t)oth_x + ox;
    const int32_t world_y = (int32_t)oth_y + oy;
    const int rx = (int)(world_x - (int32_t)loc_x);
    const int ry = (int)(world_y - (int32_t)loc_y);
    if (mw_cam_find_xy(local_slot, rx, ry, tile, 6))
      match_reproj++;
  }

  const int med0x =
      s_cam_n[0] ? (int)(sum0x / (long)s_cam_n[0]) : 0;
  const int med0y =
      s_cam_n[0] ? (int)(sum0y / (long)s_cam_n[0]) : 0;
  const int med1x =
      s_cam_n[1] ? (int)(sum1x / (long)s_cam_n[1]) : 0;
  const int med1y =
      s_cam_n[1] ? (int)(sum1y / (long)s_cam_n[1]) : 0;

  /* Top tile IDs in local buffer (crude histogram). */
  unsigned hist[256];
  memset(hist, 0, sizeof(hist));
  for (unsigned i = 0; i < s_cam_n[local_slot]; i++)
    hist[s_cam_spr[local_slot][i][2]]++;
  uint8_t top_t[5];
  unsigned top_c[5];
  memset(top_t, 0, sizeof(top_t));
  memset(top_c, 0, sizeof(top_c));
  for (unsigned t = 0; t < 256u; t++) {
    const unsigned c = hist[t];
    if (c == 0)
      continue;
    for (int k = 0; k < 5; k++) {
      if (c > top_c[k]) {
        for (int m = 4; m > k; m--) {
          top_c[m] = top_c[m - 1];
          top_t[m] = top_t[m - 1];
        }
        top_c[k] = c;
        top_t[k] = (uint8_t)t;
        break;
      }
    }
  }

  /* Live PPU OAM still on-screen (pre-present wipe happens after this call). */
  unsigned live_oam = 0;
  if (g_ppu) {
    for (int s = 0; s < 128; s++) {
      const uint8_t y = (uint8_t)(g_ppu->oam[s * 2] >> 8);
      if (y < 0xf0u)
        live_oam++;
    }
  }

  fprintf(stderr,
          "[mw_results] f=%d slot=%d gm=%02X ysh=%d n0=%u n1=%u "
          "mutual0=%u mutual6=%u match_reproj=%u ui_prio3=%u live_oam=%u\n",
          snes_frame_counter, local_slot, (unsigned)g_ram[0x10], y_shift,
          s_cam_n[0], s_cam_n[1], mutual0, mutual6, match_reproj, ui_n,
          live_oam);
  fprintf(stderr,
          "[mw_results] cam0 bbox=(%d..%d,%d..%d) mean=(%d,%d)  "
          "cam1 bbox=(%d..%d,%d..%d) mean=(%d,%d)  "
          "dcam=(%d,%d)\n",
          s_cam_n[0] ? b0x0 : 0, s_cam_n[0] ? b0x1 : 0,
          s_cam_n[0] ? b0y0 : 0, s_cam_n[0] ? b0y1 : 0, med0x, med0y,
          s_cam_n[1] ? b1x0 : 0, s_cam_n[1] ? b1x1 : 0,
          s_cam_n[1] ? b1y0 : 0, s_cam_n[1] ? b1y1 : 0, med1x, med1y,
          (int)loc_x - (int)oth_x, (int)loc_y - (int)oth_y);
  fprintf(stderr,
          "[mw_results] slot%d top tiles: %02X×%u %02X×%u %02X×%u %02X×%u "
          "%02X×%u\n",
          local_slot, top_t[0], top_c[0], top_t[1], top_c[1], top_t[2],
          top_c[2], top_t[3], top_c[3], top_t[4], top_c[4]);

  /* Sample up to 8 sprites from each cam (sx,sy,tile,attr). */
  for (int cam = 0; cam < 2; cam++) {
    const unsigned n = s_cam_n[cam] < 8u ? s_cam_n[cam] : 8u;
    fprintf(stderr, "[mw_results] cam%d sample:", cam);
    for (unsigned i = 0; i < n; i++) {
      fprintf(stderr, " (%d,%d t%02X a%02X)", (int)s_cam_sx[cam][i],
              (int)s_cam_sy[cam][i], s_cam_spr[cam][i][2],
              s_cam_spr[cam][i][3]);
    }
    fprintf(stderr, "%s\n", s_cam_n[cam] > 8u ? " …" : "");
  }
}

/*
 * Present OAM from dual cam-capture buffers.
 * Primary: local buffer (cam-relative sy already matches this view).
 * Also reproject the other buffer into local camera space — dual drawers
 * emit P1-half then P2-half per tile, and vert-widen + OAM pressure often
 * drops the second half for small props (crates), so items near P2 only
 * land in cam0. Naive merge (no reproject) tore mechs; this transforms
 * X/Y by dual cam deltas and dedups against already-placed tiles.
 *
 * End-match results (OBJ prio 3 / a34·a35): dual draw splits wins vs menu
 * across cam0/cam1 (mutual raw=0; both clusters center-stacked so X-span
 * split mis-groups glyphs). Stamp each cam's UI as a group: wins-like
 * tile IDs (else lower mean-X) → top, menu → bottom. Never cam-delta
 * reproject those tiles.
 * Returns 1 if applied (Y already final — caller must NOT apply oam_delta).
 */
static int mw_present_oam_from_cam_capture(int local_slot, int extra,
                                           int y_shift) {
  if (!g_ppu || (local_slot != 0 && local_slot != 1))
    return 0;
  if (!s_cam_capture_frame || (s_cam_n[0] == 0 && s_cam_n[1] == 0))
    return 0;

  /*
   * Left cull must NOT reach the widescreen ambiguous 9-bit band
   * [256, 256+extra) ↔ screen ≈[-256, -extra). Placing props there
   * (old x_lo=-extra-64 → keep down to ≈-extra-128) ghosts them onto the
   * right margin when left-wrap fails, then they vanish at cull — cam wrestle.
   * Keep only sprites that can still touch the visible left margin:
   * x + 64 >= -extra  ⇒  reject when x + 64 < -extra.
   */
  const int x_lo = -extra;
  const int x_hi = 256 + extra + 64;
  const int other = local_slot ^ 1;

  /* Same latched cams as BG1 present — never re-read WRAM mid-frame. */
  uint16_t loc_x, loc_y, oth_x, oth_y;
  if (s_present_cam_valid) {
    loc_x = s_present_loc_x;
    loc_y = s_present_loc_y;
    oth_x = s_present_oth_x;
    oth_y = s_present_oth_y;
  } else {
    loc_x = local_slot == 0
                ? (s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16u))
                : (s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1Au));
    loc_y = local_slot == 0
                ? (s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18u))
                : (s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1Cu));
    oth_x = other == 0 ? (s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16u))
                       : (s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1Au));
    oth_y = other == 0 ? (s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18u))
                       : (s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1Cu));
  }

  mw_results_dump(local_slot, y_shift, loc_x, loc_y, oth_x, oth_y);

  for (int slot = 0; slot < 128; slot++) {
    const int idx = slot * 2;
    g_ppu->oam[idx] = 0x00f0u; /* X=0 Y=$F0 */
    g_ppu->oam[idx + 1] = 0;
  }
  memset(g_ppu->highOam, 0, sizeof(g_ppu->highOam));

  uint8_t right_hints[16];
  memset(right_hints, 0, sizeof(right_hints));

  int kept_x[128], kept_y[128];
  uint8_t kept_tile[128];
  unsigned kept = 0;

  unsigned ui_n = 0;
  for (int cam = 0; cam < 2; cam++) {
    for (unsigned i = 0; i < s_cam_n[cam]; i++) {
      if (mw_oam_is_results_ui(s_cam_spr[cam][i][3]))
        ui_n++;
    }
  }
  /* Enough prio-3 glyphs → results overlay (gameplay stays prio≤2). */
  const int results_ui = (ui_n >= 8u);

  /*
   * ---- owned results overlay FIRST ----
   * Dual drawers put wins/W·L on one cam buffer and the menu on the other;
   * under full-frame both land mid-screen with overlapping X (so a glyph
   * X-split tears strings). Group by source cam; native L/R → mean-X order
   * picks top (wins) vs bottom (menu). Raw captured XY only (no y_shift).
   */
  if (results_ui) {
    int cx0[2] = {512, 512}, cx1[2] = {-512, -512};
    int cy0[2] = {512, 512}, cy1[2] = {-512, -512};
    int64_t xsum[2] = {0, 0};
    unsigned cn[2] = {0, 0};
    /* Dumps: wins/W·L glyphs ~t40–t5F; menu options ~t60–t9F. */
    int wins_score[2] = {0, 0}, menu_score[2] = {0, 0};
    for (int cam = 0; cam < 2; cam++) {
      for (unsigned i = 0; i < s_cam_n[cam]; i++) {
        if (!mw_oam_is_results_ui(s_cam_spr[cam][i][3]))
          continue;
        const int x = (int)s_cam_sx[cam][i];
        const int y = (int)s_cam_sy[cam][i];
        if (x + 64 < x_lo || x > x_hi)
          continue;
        if (x < cx0[cam])
          cx0[cam] = x;
        if (x > cx1[cam])
          cx1[cam] = x;
        if (y < cy0[cam])
          cy0[cam] = y;
        if (y > cy1[cam])
          cy1[cam] = y;
        xsum[cam] += x;
        cn[cam]++;
        {
          const uint8_t t = s_cam_spr[cam][i][2];
          if (t >= 0x40u && t < 0x60u)
            wins_score[cam]++;
          else if (t >= 0x60u && t < 0xA0u)
            menu_score[cam]++;
        }
      }
    }

    /*
     * Prefer tile-ID scores (stable under center-stack); fall back to
     * mean-X (native L/R) when scores are tied.
     */
    int cam_top = -1, cam_bot = -1;
    if (cn[0] > 0 && cn[1] > 0) {
      const int s0 = wins_score[0] - menu_score[0];
      const int s1 = wins_score[1] - menu_score[1];
      if (s0 != s1) {
        cam_top = (s0 > s1) ? 0 : 1;
        cam_bot = cam_top ^ 1;
      } else {
        const int mean0 = (int)(xsum[0] / (int64_t)cn[0]);
        const int mean1 = (int)(xsum[1] / (int64_t)cn[1]);
        cam_top = (mean0 <= mean1) ? 0 : 1;
        cam_bot = cam_top ^ 1;
      }
      /* Playtest: bands were inverted (menu top / wins bottom) — swap. */
      {
        const int t = cam_top;
        cam_top = cam_bot;
        cam_bot = t;
      }
    } else if (cn[0] > 0) {
      cam_top = 0;
    } else if (cn[1] > 0) {
      cam_top = 1;
    }

    const int content_w = 256;
    int dx[2] = {0, 0}, dy[2] = {0, 0};
    if (cam_top >= 0) {
      const int c = cam_top;
      const int gw = cx1[c] - cx0[c];
      const int tx = (content_w - gw) / 2;
      /* Below 16px top bar; was 24 — pull toward mid so it isn't glued up. */
      const int ty = 48;
      dx[c] = tx - cx0[c];
      dy[c] = ty - cy0[c];
    }
    if (cam_bot >= 0) {
      const int c = cam_bot;
      const int gw = cx1[c] - cx0[c];
      const int gh = cy1[c] - cy0[c];
      const int tx = (content_w - gw) / 2;
      /* Raise from bottom margin (was 16) so menu sits nearer wins/W·L. */
      int ty = 224 - 48 - gh;
      if (ty < 104)
        ty = 104; /* keep clear of top band */
      dx[c] = tx - cx0[c];
      dy[c] = ty - cy0[c];
    }

    for (int cam = 0; cam < 2; cam++) {
      if (cn[cam] == 0)
        continue;
      for (unsigned i = 0; i < s_cam_n[cam] && kept < 128u; i++) {
        const uint8_t *sp = s_cam_spr[cam][i];
        if (!mw_oam_is_results_ui(sp[3]))
          continue;
        const int sx = (int)s_cam_sx[cam][i];
        const int sy = (int)s_cam_sy[cam][i];
        if (sx + 64 < x_lo || sx > x_hi)
          continue;
        const int x = sx + dx[cam];
        const int ny = sy + dy[cam];
        const uint8_t src_h = (uint8_t)(
            (s_cam_high[cam][i >> 2] >> ((i & 3u) * 2u)) & 2u);
        mw_present_oam_place(x, ny, sp, src_h, extra, right_hints, kept_x,
                             kept_y, kept_tile, &kept, 0);
      }
    }

    static int reflow_logged;
    if (!reflow_logged && cam_top >= 0 && cam_bot >= 0) {
      reflow_logged = 1;
      fprintf(stderr,
              "[mw_rtl] H2H results UI reflow: wins/W·L top cam%d (dy=%d) "
              "menu bottom cam%d (dy=%d)\n",
              cam_top, dy[cam_top], cam_bot, dy[cam_bot]);
    }
  }

  const uint16_t c0x = (local_slot == 0) ? loc_x : oth_x;
  const uint16_t c0y = (local_slot == 0) ? loc_y : oth_y;
  const uint16_t c1x = (local_slot == 1) ? loc_x : oth_x;
  const uint16_t c1y = (local_slot == 1) ? loc_y : oth_y;

  /* ---- local gameplay (skip results UI + stage props; props handled next) ---- */
  for (unsigned i = 0; i < s_cam_n[local_slot] && kept < 128u; i++) {
    const uint8_t *sp = s_cam_spr[local_slot][i];
    if (results_ui && mw_oam_is_results_ui(sp[3]))
      continue;
    if (s_cam_local_only[local_slot][i])
      continue;
    /* Belt: never present props from stale capture sx (screen-space at
     * capture cam). That jumps left by Δcam when the local cam moves. */
    if (s_cam_prop_obj[local_slot][i] != 0)
      continue;
    if (s_cam_shared_item[local_slot][i])
      continue; /* full sprite from item sticky below */
    /* Prop sticky tiles must come from the home sticky path only. Untagged
     * capture copies (e.g. $0E) screen-lock beside map brown under feet. */
    if (mw_oam_tile_in_prop_sticky(sp[2]))
      continue;
    /* World-reproject from capture wx/wy vs present loc — capture sx is
     * screen-space at drawer cam and screen-locks when loc advances
     * (platform/mech chunks follow the viewport then snap). */
    int use_wx = (int)s_cam_wx[local_slot][i];
    int use_wy = (int)s_cam_wy[local_slot][i];
    /* Orphan $62/$C8: capture wx tracks cam (trim FOV-locks beside world
     * brown). Latch world XY through follow. */
    if (mw_plat_stripe_tile_ok(sp[2]))
      mw_orphan_stripe_world_xy(sp[2], use_wx, use_wy, loc_x, &use_wx,
                                &use_wy);
    /* Untagged $E4 on-mech backpack: capture wx freezes → falls behind. */
    else if (sp[2] == 0xE4u)
      mw_orphan_e4_backpack_xy(use_wx, use_wy, local_slot, &use_wx, &use_wy);
    const int x = use_wx - (int)loc_x;
    if (x + 64 < x_lo || x > x_hi)
      continue;
    /* local_only already skipped above. Do NOT XY-cull near stage props —
     * mechs on $C6A4/$C382 sit inside that box and vanished (A/B: OAM_CULL=0). */
    const int ny = use_wy - (int)loc_y + y_shift;
    const uint8_t src_h = (uint8_t)(
        (s_cam_high[local_slot][i >> 2] >> ((i & 3u) * 2u)) & 2u);
    mw_present_oam_place(x, ny, sp, src_h, extra, right_hints, kept_x, kept_y,
                         kept_tile, &kept, 6);
  }

  /* ---- stage props ($00B1≠$D5B8) ----
   * Home-only OAM (no foreign FOV draw — that felt like blocks following P1).
   * Foreign ghosts stay a BG1 blank problem. Scrub polluted sticky first. */
  unsigned prop_n = 0, prop_skip_own = 0, prop_skip_y = 0, prop_raw = 0;
  uint8_t prop_alive[kMwPropHomeMax];
  memset(prop_alive, 0, sizeof(prop_alive));
  if (!results_ui) {
    uint16_t idx = mw_wram16(0x1E14u);
    for (int guard = 0; guard < 64 && idx && kept < 128u; guard++) {
      const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
      /* Keep sticky while the object remains a stage prop in the list.
       * Object +$00 bit $8000 flickers on $C39E/$C6A4 every few frames;
       * requiring it here wiped sn→0 and blanked the home peer. */
      if (mw_obj_is_stage_prop(idx)) {
        const int ps = mw_prop_slot_for_obj(idx);
        mw_prop_sticky_scrub(ps);
        /* Mark alive on every peer so foreign present does not wipe home. */
        if (s_prop_sticky_valid[ps] && s_prop_sticky_obj[ps] == idx)
          prop_alive[ps] = 1;
        const int home = mw_stage_prop_home_cam(idx, c0x, c0y, c1x, c1y);
        const int live_ox = (int)mw_wram16((uint16_t)(idx + 2u));
        const int live_oy = (int)mw_wram16((uint16_t)(idx + 4u));
        int use_ox = live_ox, use_oy = live_oy;
        mw_prop_present_world_xy(idx, live_ox, live_oy, loc_x, home,
                                 local_slot, &use_ox, &use_oy);
        mw_prop_apply_c39e_soak_probe(idx, &use_ox, &use_oy);
        if (home != local_slot) {
          prop_skip_own++;
        } else {
          const int pinned = mw_prop_is_pinned_hazard(idx);
          const int c39e_probe =
              mw_prop_is_c39e_soak_probe(idx) && mw_prop_c39e_probe_active();
          const int pin_sy = use_oy - (int)loc_y;
          /* sn=0 pin: seed only when roughly on-screen (not upstairs sy≈−159). */
          if (pinned && pin_sy >= -48 && pin_sy < 220 &&
              !(s_prop_sticky_valid[ps] && s_prop_sticky_obj[ps] == idx &&
                s_prop_sticky_n[ps] > 0)) {
            static const uint8_t kPinC382[4] = {0, 0, 0xC8, 0x30};
            mw_prop_sticky_store(idx, kPinC382, 2, -8, -8, 1);
            prop_alive[ps] = 1;
          }
          /* Soak probe: seed marker so C39E@350,490 is visible when sn=0.
           * Wider Y than pins — object often sits far above stand cam. */
          if (c39e_probe &&
              !(s_prop_sticky_valid[ps] && s_prop_sticky_obj[ps] == idx &&
                s_prop_sticky_n[ps] > 0)) {
            static const uint8_t kProbeC39E[4] = {0, 0, 0xC8, 0x30};
            mw_prop_sticky_store(idx, kProbeC39E, 2, -8, -8, 1);
            prop_alive[ps] = 1;
          }
          if (s_prop_sticky_valid[ps] && s_prop_sticky_obj[ps] == idx &&
              s_prop_sticky_n[ps] > 0) {
            for (unsigned t = 0; t < s_prop_sticky_n[ps] && kept < 128u; t++) {
              const uint8_t *spr = s_prop_sticky_spr[ps][t];
              int mox = (int)s_prop_sticky_mox[ps][t];
              int moy = (int)s_prop_sticky_moy[ps][t];
              if (pinned) {
                mox = -8;
                moy = -8 + mw_prop_pin_dy();
              } else if (c39e_probe) {
                mox = -8;
                moy = -8;
              } else if (!mw_prop_meta_sane(mox, moy)) {
                continue;
              }
              const int x = (use_ox - (int)loc_x) + mox;
              const int ny =
                  mw_prop_present_ny(0, 0, use_oy, (int)loc_y, moy, y_shift);
              if (x + 64 < x_lo || x > x_hi) {
                continue;
              }
              if (ny < 0) {
                prop_skip_y++;
                continue;
              }
              prop_raw++;
              if (mw_present_oam_dup(kept_x, kept_y, kept_tile, kept, x, ny,
                                     spr[2]))
                continue;
              if (mw_present_oam_place(x, ny, spr, s_prop_sticky_sz[ps][t],
                                       extra, right_hints, kept_x, kept_y,
                                       kept_tile, &kept, 6))
                prop_n++;
            }
          }
        }
      }
      if (next == 0 || next == idx)
        break;
      idx = next;
    }
    for (int s = 0; s < kMwPropHomeMax; s++) {
      if (s_prop_sticky_valid[s] && !prop_alive[s]) {
        s_prop_sticky_valid[s] = 0;
        s_prop_sticky_n[s] = 0;
        s_prop_sticky_obj[s] = 0;
        s_prop_anchor_valid[s] = 0;
        s_prop_glue_have[s] = 0;
        s_prop_glue_mech_have[s] = 0;
        s_prop_glue_hits[s] = 0;
        s_prop_glue_clear[s] = 0;
        s_prop_c6_travel_hits[s] = 0;
        s_prop_use_valid[s] = 0;
        mw_rider_brown_clear_slot(s);
      }
    }
  }
  /* Fallback: home-only raw local_only when sticky empty; seed only if sane. */
  for (int src = 0; src < 2 && kept < 128u; src++) {
    for (unsigned i = 0; i < s_cam_n[src] && kept < 128u; i++) {
      if (!s_cam_local_only[src][i])
        continue;
      const uint8_t *sp = s_cam_spr[src][i];
      if (results_ui && mw_oam_is_results_ui(sp[3]))
        continue;
      const uint16_t pobj = s_cam_prop_obj[src][i];
      if (!mw_obj_is_stage_prop(pobj))
        continue;
      const int ps = mw_prop_slot_for_obj(pobj);
      mw_prop_sticky_scrub(ps);
      if (s_prop_sticky_valid[ps] && s_prop_sticky_obj[ps] == pobj &&
          s_prop_sticky_n[ps] > 0)
        continue; /* already presented full sticky set */
      int home = mw_stage_prop_home_cam(pobj, c0x, c0y, c1x, c1y);
      const int live_ox = (int)mw_wram16((uint16_t)(pobj + 2u));
      const int live_oy = (int)mw_wram16((uint16_t)(pobj + 4u));
      int use_ox = live_ox, use_oy = live_oy;
      mw_prop_present_world_xy(pobj, live_ox, live_oy, loc_x, home,
                               local_slot, &use_ox, &use_oy);
      mw_prop_apply_c39e_soak_probe(pobj, &use_ox, &use_oy);
      if (home != local_slot) {
        prop_skip_own++;
        continue;
      }
      int mox = (int)s_cam_prop_meta_ox[src][i];
      int moy = (int)s_cam_prop_meta_oy[src][i];
      if (moy == 0)
        moy = -10;
      if (mw_prop_is_pinned_hazard(pobj)) {
        mox = -8;
        moy = -8 + mw_prop_pin_dy();
      } else if (mw_prop_is_c39e_soak_probe(pobj) &&
                 mw_prop_c39e_probe_active()) {
        mox = -8;
        moy = -8;
      } else if (!mw_prop_meta_sane(mox, moy)) {
        continue;
      }
      const uint8_t src_h = (uint8_t)(
          (s_cam_high[src][i >> 2] >> ((i & 3u) * 2u)) & 2u);
      mw_prop_sticky_store(pobj, sp, src_h, (int16_t)mox, (int16_t)moy, 1);
      const int cap_sy = (int)s_cam_sy[src][i];
      const int x = (use_ox - (int)loc_x) + mox;
      const int ny =
          mw_prop_present_ny(1, cap_sy, use_oy, (int)loc_y, moy, y_shift);
      if (x + 64 < x_lo || x > x_hi)
        continue;
      if (ny < 0) {
        prop_skip_y++;
        continue;
      }
      prop_raw++;
      if (mw_present_oam_dup(kept_x, kept_y, kept_tile, kept, x, ny, sp[2]))
        continue;
      if (mw_present_oam_place(x, ny, sp, src_h, extra, right_hints, kept_x,
                               kept_y, kept_tile, &kept, 6))
        prop_n++;
    }
  }
  /*
   * Shared $B1 pickups/items: place full multi-tile sticky for BOTH peers
   * from live +$02/+$04. Capture/reproject alone leave half-sprites when
   * dual OAM pressure drops tiles or far-cam Y culls the emit.
   */
  if (!results_ui) {
    uint8_t item_alive[kMwItemStickyMax];
    memset(item_alive, 0, sizeof(item_alive));
    uint16_t idx = mw_wram16(0x1E14u);
    for (int guard = 0; guard < 64 && idx && kept < 128u; guard++) {
      const uint16_t next = mw_wram16((uint16_t)(idx + 0x14u));
      if (mw_obj_is_shared_b1_item(idx)) {
        int s = -1;
        for (int i = 0; i < kMwItemStickyMax; i++) {
          if (s_item_sticky_valid[i] && s_item_sticky_obj[i] == idx) {
            s = i;
            break;
          }
        }
        if (s >= 0) {
          item_alive[s] = 1;
          const int live_ox = (int)mw_wram16((uint16_t)(idx + 2u));
          const int live_oy = (int)mw_wram16((uint16_t)(idx + 4u));
          for (unsigned t = 0; t < s_item_sticky_n[s] && kept < 128u; t++) {
            const uint8_t *spr = s_item_sticky_spr[s][t];
            const int x =
                (live_ox - (int)loc_x) + (int)s_item_sticky_mox[s][t];
            const int ny = mw_prop_present_ny(
                0, 0, live_oy, (int)loc_y, (int)s_item_sticky_moy[s][t],
                y_shift);
            if (x + 64 < x_lo || x > x_hi || ny < 0)
              continue;
            if (mw_present_oam_dup(kept_x, kept_y, kept_tile, kept, x, ny,
                                   spr[2]))
              continue;
            mw_present_oam_place(x, ny, spr, s_item_sticky_sz[s][t], extra,
                                 right_hints, kept_x, kept_y, kept_tile,
                                 &kept, 6);
          }
        }
      }
      if (next == 0 || next == idx)
        break;
      idx = next;
    }
    for (int s = 0; s < kMwItemStickyMax; s++) {
      if (s_item_sticky_valid[s] && !item_alive[s]) {
        s_item_sticky_valid[s] = 0;
        s_item_sticky_n[s] = 0;
        s_item_sticky_obj[s] = 0;
      }
    }
  }
  {
    unsigned lo0 = 0, lo1 = 0;
    for (unsigned i = 0; i < s_cam_n[0]; i++)
      if (s_cam_local_only[0][i])
        lo0++;
    for (unsigned i = 0; i < s_cam_n[1]; i++)
      if (s_cam_local_only[1][i])
        lo1++;
    s_elev_cap_n0 = s_cam_n[0];
    s_elev_cap_n1 = s_cam_n[1];
    s_elev_cap_lo0 = lo0;
    s_elev_cap_lo1 = lo1;
    {
      extern int snes_frame_counter;
      s_coldump_prop_n = prop_n;
      s_coldump_prop_raw = prop_raw;
      s_coldump_skip_own = prop_skip_own;
      s_coldump_skip_y = prop_skip_y;
      s_coldump_present_slot = local_slot;
      s_coldump_present_frame = snes_frame_counter;
    }
    if (getenv("SNESRECOMP_MW_ELEV")) {
      static int prop_log_div;
      if ((prop_log_div++ % 30) == 0) {
        fprintf(stderr,
                "[mw_rtl] H2H stage-prop OAM ($00B1≠$D5B8): present "
                "(%u tiles raw=%u; capture local_only=%u+%u slot=%d "
                "skip_own=%u skip_y=%u "
                "stat=latch:%u list:%u commit:%u conv:%u meta7e_max:%u "
                "bg1src=%d bg_dy=%d)\n",
                prop_n, prop_raw, lo0, lo1, local_slot, prop_skip_own,
                prop_skip_y, s_prop_stat_latch, s_prop_stat_list_rec,
                s_prop_stat_commit, s_prop_stat_convert,
                s_prop_stat_meta7e_max, s_elev_bg1_src_path,
                s_elev_prop_bg_dy);
      }
    }
  }

  /* ---- other buffer → reproject world sprites only (never results UI /
   * stage props — sticky/home path above; also skip prop_obj if demoted
   * out of local_only). Shared $B1 items: sticky present already full. ---- */
  const int reproj_x_lo = -extra;
  const int reproj_x_hi = 256 + extra; /* exclusive: [lo, hi) stays unwrapped */
  for (unsigned i = 0; i < s_cam_n[other] && kept < 128u; i++) {
    const uint8_t *sp = s_cam_spr[other][i];
    if (results_ui && mw_oam_is_results_ui(sp[3]))
      continue;
    if (s_cam_local_only[other][i])
      continue;
    if (s_cam_prop_obj[other][i] != 0)
      continue; /* stage prop — never cam-delta ghost from far buffer */
    if (s_cam_shared_item[other][i])
      continue; /* sticky present already placed full item */
    if (mw_oam_tile_in_prop_sticky(sp[2]))
      continue;
    const int ox = (int)s_cam_sx[other][i];
    const int oy = (int)s_cam_sy[other][i];
    const int raw_ny = oy + y_shift;
    int world_x = (int)oth_x + ox;
    int world_y = (int)oth_y + oy;
    /* Far-buffer orphan stripe: same cam+sx FOV-lock as local path. */
    if (mw_plat_stripe_tile_ok(sp[2]))
      mw_orphan_stripe_world_xy(sp[2], world_x, world_y, loc_x, &world_x,
                                &world_y);
    else if (sp[2] == 0xE4u)
      mw_orphan_e4_backpack_xy(world_x, world_y, local_slot, &world_x,
                               &world_y);
    const int x = world_x - (int)loc_x;
    const int sy = world_y - (int)loc_y;
    /* Screen-fixed dual half: same raw XY in both cams — skip. Do NOT
     * skip on raw XY alone before reproject: that dropped the far half of
     * sliced crates when the near half already occupied a nearby slot. */
    if (x == ox && sy == oy &&
        mw_present_oam_dup(kept_x, kept_y, kept_tile, kept, ox, raw_ny,
                           sp[2]))
      continue;
    if (x < reproj_x_lo || x >= reproj_x_hi)
      continue;
    const int ny = sy + y_shift;
    /* Size+Xhi — keep bit0 (X9) and bit1 (size) so 16×16 items don't
     * collapse to 8×8 (looks like a culled right half). */
    const uint8_t src_h = (uint8_t)(
        (s_cam_high[other][i >> 2] >> ((i & 3u) * 2u)) & 3u);
    mw_present_oam_place(x, ny, sp, src_h, extra, right_hints, kept_x, kept_y,
                         kept_tile, &kept, 6);
  }

  PpuWsSetOamRightHints(g_ppu, right_hints);

  static int logged;
  if (!logged) {
    logged = 1;
    fprintf(stderr,
            "[mw_rtl] H2H present OAM from cam capture (slot=%d n=%u+%u "
            "kept=%u, y_shift=%d; results_ui=%d ui_prio3=%u)\n",
            local_slot, s_cam_n[local_slot], s_cam_n[other], kept, y_shift,
            results_ui, ui_n);
  } else if (results_ui) {
    static int ui_logged;
    if (!ui_logged) {
      ui_logged = 1;
      fprintf(stderr,
              "[mw_rtl] H2H results UI ownership: stamp prio3 per-cam "
              "top/bottom (no y_shift / no cam-delta reproject), ui_n=%u\n",
              ui_n);
    }
  }
  (void)kept;
  return 1;
}

/* Present-only: keep sprites tagged for local_slot; hide other/untagged. */
static void mw_present_oam_cull_by_tag(int local_slot, int extra) {
  if (!g_ppu || (local_slot != 0 && local_slot != 1))
    return;
  mw_oam_cam_tag_ensure_init();
  const int x_lo = -extra - 64;
  const int x_hi = 256 + extra + 64;

  for (int slot = 0; slot < 128; slot++) {
    const int idx = slot * 2;
    const uint8_t y = (uint8_t)(g_ppu->oam[idx] >> 8);
    if (y >= 0xf0u)
      continue;
    int x = mw_ppu_oam_read_x(slot, extra);
    int hide = 0;
    const uint8_t tag = s_oam_cam_tag[slot];
    if (tag == 0xffu || (int)tag != local_slot)
      hide = 1;
    if (!hide && (x + 64 < x_lo || x > x_hi))
      hide = 1;
    if (hide)
      g_ppu->oam[idx] = (uint16_t)(g_ppu->oam[idx] & 0x00ffu) | 0xf000u;
  }
}

/* Present-only OAM cull for full-frame local (legacy / non–vert-widen). */
static void mw_present_oam_cull_other_half(int local_slot, int extra) {
  if (!g_ppu || (local_slot != 0 && local_slot != 1))
    return;
  int split_y = (int)mw_wram16(0x1EA0);
  if (split_y <= 0 || split_y >= 224)
    split_y = 0x70;
  const int y_pad = 8;
  const int x_lo = -extra - 64;
  const int x_hi = 256 + extra + 64;

  for (int slot = 0; slot < 128; slot++) {
    const int idx = slot * 2;
    const uint8_t y = (uint8_t)(g_ppu->oam[idx] >> 8);
    if (y >= 0xf0u)
      continue;
    int x = mw_ppu_oam_read_x(slot, extra);

    int hide = 0;
    if (local_slot == 0) {
      if ((int)y >= split_y + y_pad)
        hide = 1;
    } else {
      if ((int)y < split_y - y_pad)
        hide = 1;
    }
    if (!hide && (x + 64 < x_lo || x > x_hi))
      hide = 1;
    if (hide) {
      g_ppu->oam[idx] = (uint16_t)(g_ppu->oam[idx] & 0x00ffu) | 0xf000u;
    }
  }
}

/* ROM HDMA split tables at $80:9670 — three 104/16/104 bands (BG1 H/V, BG2 H). */
enum {
  kMwHdmaSplitNative0 = 0x68u, /* 104 */
  kMwHdmaSplitNative1 = 0x10u, /* 16 */
  kMwHdmaSplitTall = 0x70u,    /* 112 */
};

static void mw_h2h_taller_patch_hdma(void) {
  if (!mw_h2h_taller_armed() || !g_snes || !g_snes->cart)
    return;
  uint8_t *base = cart_getRomPtr(g_snes->cart, 0x80, 0x9670);
  if (!base)
    return;

  /* Each band is 10 bytes: cnt,ptr ×3 + 00. Native 104/16/104 → 112/112/end. */
  static const int kBandOff[3] = {0x00, 0x0a, 0x1b};
  if (base[0] == kMwHdmaSplitTall)
    return; /* already patched (same ROM image) */

  for (int b = 0; b < 3; b++) {
    uint8_t *t = base + kBandOff[b];
    if (t[0] != kMwHdmaSplitNative0 || t[3] != kMwHdmaSplitNative1 ||
        t[6] != kMwHdmaSplitNative0 || t[9] != 0x00)
      continue;
    const uint8_t p1lo = t[7];
    const uint8_t p1hi = t[8];
    t[0] = kMwHdmaSplitTall;
    /* t[1], t[2] keep first scroll mirror */
    t[3] = kMwHdmaSplitTall;
    t[4] = p1lo;
    t[5] = p1hi;
    t[6] = 0x00;
    t[7] = t[8] = t[9] = 0x00;
  }
  static int logged;
  if (!logged) {
    logged = 1;
    fprintf(stderr,
            "[mw_rtl] H2H Phase 2b: HDMA 112/112, stripe #$001E, spawn Y −$70, "
            "OAM Y bias cancelled (SNESRECOMP_MW_H2H_TALLER=0 to disable)\n");
  }
}

/* Dual path STA $1E9C at $8095A9 (LDA #$0010). Bump to single-player #$001E.
 * Opt-in only (H2H_TALLER) — raising row count fights the dual strip. */
static void mw_h2h_taller_stripe_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_h2h_taller_armed() || !MwIsDualViewport())
    return;
  if (cpu->A == 0x0010u)
    cpu->A = 0x001Eu;
}

/*
 * Dual stores $1E9C=#$10 then $1E9A=$1E9C<<3 (=#$80, half-frame height).
 * $1E9A feeds ONLY bbox bottoms ($1E28/$1E2C) and $8283AC Y distance — not
 * stripe row count ($1E9C). Full-frame H2H present is ~224px, so movers in
 * the lower half sit outside sim bounds and chase cam deltas relative to a
 * 128px playfield (lost Y bearings). Raise $1E9A to 1P height (#$F0) while
 * leaving $1E9C at #$10. Gate: full-frame present and/or vert-widen.
 */
static void mw_h2h_dual_view_height_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!MwIsDualViewport())
    return;
  if (!MwH2hFullFrameLocalArmed() && !mw_h2h_vert_widen_armed())
    return;
  /* Dual path only (#$10<<3). 1P path already stores #$F0. */
  if (cpu->A == 0x0080u)
    cpu->A = 0x00F0u;
}

/* Spawn window top: after SBC #$0028, A → STA $36/$3A (camY−40).
 * Dual/taller: extra −$80. Expand gameplay: also −Y_SLOP (tall shafts) so
 * an elevator on another floor stays inside [top, top+$3E). Y-only — does
 * not touch the shared $8283AC X/Y radius (that regresses object lifetime). */
static void mw_h2h_taller_spawn_y_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  unsigned sub = 0;
  if ((mw_h2h_taller_armed() || mw_h2h_spawn_y_widen_armed()) &&
      MwIsDualViewport())
    sub = 0x80u;
  else if (mw_can_expand_gameplay()) {
    sub = mw_ws_y_lifetime_slop();
    if (sub > 0xC0u)
      sub = 0xC0u;
  }
  if (sub == 0)
    return;
  if (cpu->A >= (uint16_t)sub)
    cpu->A = (uint16_t)(cpu->A - (uint16_t)sub);
  else
    cpu->A = 0;
}

/*
 * $82F62A STA $3E — spawn Y window height after:
 *   LDA #$0160 / LDX $1EB2 / BEQ +1 / LSR
 * Dual halves that to #$00B0 (~176px). Elevators on another floor never enter
 * the map-object scan, so they neither draw nor run until the camera reaches
 * them. Restore ≥1P height under expand, then add Y_SLOP for tall shafts.
 */
static void mw_spawn_y_height_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_can_expand_gameplay())
    return;
  uint32_t h = cpu->A;
  if (h < 0x0160u)
    h = 0x0160u;
  h += (uint32_t)mw_ws_y_lifetime_slop();
  if (h > 0xFFFFu)
    h = 0xFFFFu;
  cpu->A = (uint16_t)h;
  static int logged;
  if (!logged) {
    logged = 1;
    fprintf(stderr,
            "[mw_rtl] spawn Y height $3E → $%04X (dual native #$00B0; "
            "SNESRECOMP_MW_Y_SLOP=0 for 1P #$0160 only)\n",
            (unsigned)cpu->A);
  }
}

/* Pre-ADC #$0070 at $80B99B / $80B9EF: cancel dual half-screen Y bias so OAM
 * stays camera-relative for full-frame local present. */
static void mw_h2h_taller_y_bias_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_h2h_taller_armed() || !MwIsDualViewport())
    return;
  cpu->A = (uint16_t)(cpu->A - 0x0070u);
}

/* Offline 1P: widen meta-sprite Y CMPs without dual OAM +$78 bias.
 * Native #$68/#$70 is a ~half-screen draw clip — elevators/mechs vanish
 * outside that band. SNESRECOMP_MW_1P_Y_WIDEN=0 disables. */
static int mw_1p_y_draw_widen_armed(void) {
  static int v = -1;
  if (v < 0) {
    const char *e = getenv("SNESRECOMP_MW_1P_Y_WIDEN");
    if (e && e[0] == '0')
      v = 0;
    else
      v = 1; /* default on */
  }
  return v != 0;
}

/*
 * Dual active-list Y gate (CMP at $809280 / $8092A0): native #$A8 (or
 * widen #$E0) drops stage props whose anchor +$04 is at sy≈225 while the
 * stripe tiles sit ~10px higher. Force A under the threshold for stage
 * props only — a global list widen to #$0100/#$0140 leaked P2-floor into
 * cam0.
 */
static void mw_prop_list_y_gate_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_h2h_vert_widen_armed() || !MwIsDualViewport())
    return;
  const uint16_t obj = (uint16_t)cpu->X;
  if (!mw_obj_is_stage_prop(obj))
    return;
  const int16_t sy = (int16_t)cpu->A;
  /* Match commit prop window (−144…$120). */
  if (sy >= 0x00A0 && sy < 0x0120)
    cpu->A = (uint16_t)0x0090; /* < #$A8 and < #$E0 → list-add path */
}

/*
 * Patch Y-window CMP immediates in ROM (interp fetches these). Never
 * rewrite A at the CMP — meta paths STA $14C5 from the same A.
 *   Bottom: #$0068 / #$0070 → #$00E0 (full-frame down; was #$78 with bias)
 *           native #$00E0 sites stay #$E0 (do NOT narrow — that clipped mechs)
 *   Top:    #$FFF1 → #$FF70 (−144)
 *   List:   #$00A8 → #$00E0 under widen (gameplay tall sprites). Stage props
 *           with sy≥$E0 enter via mw_prop_list_y_gate_hook (prop-only).
 * Modes: 0 = native, 1 = offline 1P draw widen (no OAM bias), 4 = dual
 * H2H vert-widen (+ bias hooks elsewhere). $8283AC radius is X-heavy and
 * does not fix this clip.
 */
static void mw_h2h_vert_widen_patch_imm(void) {
  if (!g_snes || !g_snes->cart)
    return;
  int want = 0;
  if (mw_h2h_vert_widen_armed() && MwIsDualViewport())
    want = 4;
  else if (mw_1p_y_draw_widen_armed() && !MwIsDualViewport() &&
           mw_can_expand_gameplay())
    want = 1;
  /* Gen bump: force re-apply after list #$0100 → #$E0 policy change. */
  enum { kVertWidenPatchGen = 7 };
  static int applied = -1; /* -1 unknown; else want + gen*10 */
  if (applied == want + kVertWidenPatchGen * 10)
    return;

  static const uint16_t kCmp68[] = {
      0x8C71u, 0x8CB6u, 0x8DD3u, 0x8E1Fu, 0x8F4Au, 0x8F96u, 0x90C1u, 0x9114u,
  };
  static const uint16_t kCmp70[] = {0xDCACu, 0xDCE4u};
  static const uint16_t kCmpE0[] = {
      0x8BC1u, 0x8D13u, 0x8E8Au, 0x8FFAu, 0xDC28u,
  };
  static const uint16_t kCmpF1[] = {
      0x8BBCu, 0x8C6Cu, 0x8CB1u, 0x8D0Eu, 0x8DCEu, 0x8E1Au,
      0x8E85u, 0x8F45u, 0x8F91u, 0x8FF5u, 0x90BCu, 0x910Fu,
  };
  static const uint16_t kCmpA8[] = {0x9280u, 0x92A0u};

  const int widen = (want != 0);
  int n = 0;
  for (size_t i = 0; i < sizeof(kCmp68) / sizeof(kCmp68[0]); i++) {
    uint8_t *p = cart_getRomPtr(g_snes->cart, 0x80, kCmp68[i]);
    if (p && p[0] == 0xC9u && p[2] == 0x00u &&
        (p[1] == 0x68u || p[1] == 0x78u || p[1] == 0xE0u)) {
      const uint8_t v = widen ? 0xE0u : 0x68u;
      if (p[1] != v) {
        p[1] = v;
        n++;
      }
    }
  }
  for (size_t i = 0; i < sizeof(kCmp70) / sizeof(kCmp70[0]); i++) {
    uint8_t *p = cart_getRomPtr(g_snes->cart, 0x80, kCmp70[i]);
    if (p && p[0] == 0xC9u && p[2] == 0x00u &&
        (p[1] == 0x70u || p[1] == 0x78u || p[1] == 0xE0u)) {
      const uint8_t v = widen ? 0xE0u : 0x70u;
      if (p[1] != v) {
        p[1] = v;
        n++;
      }
    }
  }
  for (size_t i = 0; i < sizeof(kCmpE0) / sizeof(kCmpE0[0]); i++) {
    uint8_t *p = cart_getRomPtr(g_snes->cart, 0x80, kCmpE0[i]);
    /* Restore #$E0 if an older build narrowed to #$78; never narrow again. */
    if (p && p[0] == 0xC9u && p[2] == 0x00u &&
        (p[1] == 0xE0u || p[1] == 0x78u)) {
      const uint8_t v = 0xE0u;
      if (p[1] != v) {
        p[1] = v;
        n++;
      }
    }
  }
  for (size_t i = 0; i < sizeof(kCmpF1) / sizeof(kCmpF1[0]); i++) {
    uint8_t *p = cart_getRomPtr(g_snes->cart, 0x80, kCmpF1[i]);
    /* Native #$FFF1; widen #$FF90 (old) or #$FF70 (current). */
    if (p && p[0] == 0xC9u && p[2] == 0xFFu && p[3] == 0xB0u &&
        (p[1] == 0xF1u || p[1] == 0x90u || p[1] == 0x70u)) {
      const uint8_t v = widen ? 0x70u : 0xF1u;
      if (p[1] != v) {
        p[1] = v;
        n++;
      }
    }
  }
  /* Active-list Y: #$A8 → #$E0 when widening (undo #$0100/#$0140 if present). */
  for (size_t i = 0; i < sizeof(kCmpA8) / sizeof(kCmpA8[0]); i++) {
    uint8_t *p = cart_getRomPtr(g_snes->cart, 0x80, kCmpA8[i]);
    if (!p || p[0] != 0xC9u)
      continue;
    const uint8_t want_lo = widen ? 0xE0u : 0xA8u;
    const uint8_t want_hi = 0x00u;
    const int cur_ok = (p[1] == want_lo && p[2] == want_hi);
    const int cur_known =
        (p[2] == 0x00u &&
         (p[1] == 0xA8u || p[1] == 0xE0u || p[1] == 0xF0u)) ||
        (p[2] == 0x01u && (p[1] == 0x00u || p[1] == 0x40u));
    if (!cur_ok && cur_known) {
      p[1] = want_lo;
      p[2] = want_hi;
      n++;
    }
  }
  applied = want + kVertWidenPatchGen * 10;
  if (n)
    fprintf(stderr,
            "[mw_rtl] Y-draw widen: %s %d CMP immediates "
            "(mode=%d top=FF70 bot=E0 list=%s)\n",
            widen ? "patched" : "restored", n, want,
            widen ? "E0+prop-hook" : "A8");
}

/* Dual bottom-Y CMP on P1 ($86/$88) drawers only — not P2 ($8A/$8C) CMPs
 * (those used to force cam0 and let $8A tiles leak into the cam0 buffer). */
static void mw_h2h_vw_y_hi_tag_hook(CpuState *cpu, uint32_t pc24) {
  (void)cpu;
  (void)pc24;
  if (!mw_h2h_vert_widen_armed() || !MwIsDualViewport())
    return;
  s_oam_saw_adc78 = 0;
  s_oam_draw_cam = 0;
}

/* P2 path: ADC #$0078 for this sprite (unsigned OAM Y). */
static void mw_h2h_vw_adc78_tag_hook(CpuState *cpu, uint32_t pc24) {
  (void)cpu;
  (void)pc24;
  if (!mw_h2h_vert_widen_armed() || !MwIsDualViewport())
    return;
  s_oam_saw_adc78 = 1;
  s_oam_draw_cam = 1;
}

/* Native CMP #$E0 size variants — start of a P1 sprite. */
static void mw_h2h_vw_y_e0_tag_hook(CpuState *cpu, uint32_t pc24) {
  (void)cpu;
  (void)pc24;
  if (!mw_h2h_vert_widen_armed() || !MwIsDualViewport())
    return;
  s_oam_saw_adc78 = 0;
  s_oam_draw_cam = 0;
}

/* Drawer ADC $86/$88 — screen pos is cam0-relative. */
static void mw_h2h_vw_pos_cam0_hook(CpuState *cpu, uint32_t pc24) {
  (void)cpu;
  (void)pc24;
  if (!mw_h2h_vert_widen_armed() || !MwIsDualViewport())
    return;
  s_oam_draw_cam = 0;
}

/* Drawer ADC $8A/$8C — screen pos is cam1-relative. */
static void mw_h2h_vw_pos_cam1_hook(CpuState *cpu, uint32_t pc24) {
  (void)cpu;
  (void)pc24;
  if (!mw_h2h_vert_widen_armed() || !MwIsDualViewport())
    return;
  s_oam_draw_cam = 1;
}

/* Record pre-bias sy; keep dual staging OAM in 0..$EF (park $F0 if not). */
static void mw_h2h_vw_apply_y_bias_for_store(CpuState *cpu) {
  int16_t sy;
  if (s_oam_saw_adc78) {
    /* ROM just ADC #$78 — undo for pre-bias sy, then re-bias staging. */
    sy = (int16_t)(cpu->A - 0x0078);
    s_oam_saw_adc78 = 0;
    s_oam_draw_cam = 1; /* reinforce — ADC path is always P2 half */
  } else {
    sy = (int16_t)cpu->A;
    /* Do not force cam0 here: P2 tiles ADC $8C before STA Y; a forced
     * cam0 was the leak that put $8A X into the cam0 capture buffer. Cam
     * comes from ADC $86/$88/$8A/$8C hooks (+ ADC #$78 above). */
  }
  s_pending_sy = sy;
  s_pending_sy_valid = 1;
  const int biased = (int)sy + 0x78;
  if (biased < 0 || biased >= 0xf0)
    cpu->A = (uint16_t)((cpu->A & 0xff00u) | 0x00f0u);
  else
    cpu->A = (uint16_t)((cpu->A & 0xff00u) | (unsigned)(biased & 0xff));
}

static void mw_h2h_vw_oam_y_sta_x_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_h2h_vert_widen_armed() || !MwIsDualViewport())
    return;
  mw_h2h_vw_apply_y_bias_for_store(cpu);
  mw_oam_cam_tag_ensure_init();
  if (cpu->X & 3u)
    return;
  const unsigned slot = (unsigned)cpu->X / 4u;
  const int cam = (s_oam_draw_cam == 1) ? 1 : 0;
  if (slot < 128u)
    s_oam_cam_tag[slot] = (uint8_t)cam;
}

static void mw_h2h_vw_oam_y_sta_y_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_h2h_vert_widen_armed() || !MwIsDualViewport())
    return;
  mw_h2h_vw_apply_y_bias_for_store(cpu);
  mw_oam_cam_tag_ensure_init();
  if (cpu->Y & 3u)
    return;
  const unsigned slot = (unsigned)cpu->Y / 4u;
  const int cam = (s_oam_draw_cam == 1) ? 1 : 0;
  if (slot < 128u)
    s_oam_cam_tag[slot] = (uint8_t)cam;
}

static void mw_h2h_vw_oam_clear_hook(CpuState *cpu, uint32_t pc24) {
  (void)cpu;
  (void)pc24;
  if (!mw_h2h_vert_widen_armed())
    return;
  memset(s_oam_cam_tag, 0xff, sizeof(s_oam_cam_tag));
  s_oam_cam_tag_init = 1;
  s_oam_draw_cam = 0;
  s_oam_saw_adc78 = 0;
  mw_cam_oam_reset();
}

static int mw_h2h_oam_wrap_armed(void) {
  static int v = -1;
  if (v < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_OAM_WRAP");
    if (e && e[0] == '0')
      v = 0;
    else
      v = 1; /* default on with vert-widen capture */
  }
  return v;
}

/*
 * Pre-CPX #$0200 in dual/1P meta drawers: staging OAM is full (X = byte index
 * 512 = 128 sprites). Native BEQ aborts the rest of the object list — with
 * dual P1+P2 tile pairs that happens quickly when cams diverge. Reset X to 0
 * so CPX does not match and drawing continues; cam-capture already copied
 * earlier tiles into s_cam_spr[] (staging becomes a scratch ring).
 */
static void mw_h2h_oam_full_wrap_hook(CpuState *cpu, uint32_t pc24) {
  (void)pc24;
  if (!mw_h2h_oam_wrap_armed() || !mw_h2h_vert_widen_armed() ||
      !MwIsDualViewport())
    return;
  if (cpu->X < 0x0200u)
    return;
  cpu->X = 0;
  static unsigned logs;
  if (logs < 8 && getenv("SNESRECOMP_MW_COLS")) {
    logs++;
    fprintf(stderr,
            "[mw_rtl] H2H OAM staging wrap (continue draw; n0=%u n1=%u)\n",
            s_cam_n[0], s_cam_n[1]);
  }
}

/*
 * Native BG2 stripe loop: INX/INX / CPX #$10 / BNE — only 8 map rows per
 * frame. With 16×16 tiles that is 128px, so elevators/platforms on idle BG2
 * vanish outside a half-screen Y band while BG1 (full $1E9C rows) still
 * draws the shaft. VMADD tables at $83:857C / $83:864C hold 12 valid row
 * words before junk — widen to CPX #$18 (12 rows ≈ 192px). CPX sites:
 * $809334, $80A88C.
 *
 * Only for netplay or active widescreen expand. Offline dual ($1EB2) still
 * makes mw_can_expand_gameplay() true, but a 12-row DMA stomps the native
 * 16-line HDMA seam HUD (direction arrows) — bar only flashes when the
 * arrow state is rewritten. SNESRECOMP_MW_BG2_ROWS=0 disables; =16 tries 16.
 *
 */
static void mw_bg2_stripe_rows_patch(void) {
  if (!g_snes || !g_snes->cart)
    return;
  static int rows = -2; /* -2 unset, 0 off, else row count */
  if (rows == -2) {
    const char *e = getenv("SNESRECOMP_MW_BG2_ROWS");
    if (e && e[0] == '0' && e[1] == '\0')
      rows = 0;
    else if (e && e[0] >= '1' && e[0] <= '9') {
      rows = atoi(e);
      if (rows < 8)
        rows = 8;
      if (rows > 16)
        rows = 16;
    } else
      rows = 12; /* default: full 857C table when expand allowed */
  }
  /* Dual offline has WS hard-off — keep native 8 so the center seam HUD
   * stays intact. Netplay / 1P widescreen may widen. */
  const int expand_ok =
      mw_can_expand_gameplay() &&
      (snes_netplay_active() || (g_ws_active && g_ws_extra > 0));
  const int want = (rows > 8 && expand_ok) ? rows : 8;
  const uint8_t imm = (uint8_t)(want * 2); /* X steps by 2 */
  static int applied = -1;
  if (applied == want)
    return;
  static const uint16_t kCpx[] = {0x9334u, 0xA88Cu};
  int n = 0;
  for (size_t i = 0; i < sizeof(kCpx) / sizeof(kCpx[0]); i++) {
    uint8_t *p = cart_getRomPtr(g_snes->cart, 0x80, kCpx[i]);
    /* CPX #imm8 : E0 nn  (SEP #$10 → 8-bit X) */
    if (!p || p[0] != 0xE0u)
      continue;
    if (p[1] != 0x10u && p[1] != 0x18u && p[1] != 0x20u)
      continue;
    if (p[1] != imm) {
      p[1] = imm;
      n++;
    }
  }
  applied = want;
  if (n)
    fprintf(stderr, "[mw_rtl] BG2 stripe rows: %d (CPX #$%02X, %d site(s))\n",
            want, (unsigned)imm, n);
}

static void mw_install_dma_widen_hooks(void) {
  mw_h2h_vert_widen_patch_imm();
  mw_bg2_stripe_rows_patch();
  static int done;
  if (done)
    return;
  done = 1;
  if (kMwDmaWidenEnabled) {
    interp_bridge_set_pre_opcode_hook(kMwDmaSizePcBg1, mw_dma_size_hook);
    interp_bridge_set_pre_opcode_hook(kMwDmaSizePcBg2, mw_dma_size_hook);
    interp_bridge_set_pre_opcode_hook(kMwDmaSizePcAlt1, mw_dma_size_hook);
    interp_bridge_set_pre_opcode_hook(kMwDmaSizePcAlt2, mw_dma_size_hook);
  }
  mw_install_hooks(kMwSpriteWinCmpPc,
                   sizeof(kMwSpriteWinCmpPc) / sizeof(kMwSpriteWinCmpPc[0]),
                   mw_sprite_win_hook);
  mw_install_hooks(kMwParticleBmiPc,
                   sizeof(kMwParticleBmiPc) / sizeof(kMwParticleBmiPc[0]),
                   mw_particle_bmi_hook);
  mw_install_hooks(kMwParticleBcsPc,
                   sizeof(kMwParticleBcsPc) / sizeof(kMwParticleBcsPc[0]),
                   mw_particle_bcs_hook);
  mw_install_hooks(kMwParticleAfterStaPc,
                   sizeof(kMwParticleAfterStaPc) /
                       sizeof(kMwParticleAfterStaPc[0]),
                   mw_particle_after_sta_hook);
  mw_install_hooks(kMwActiveLoCmpPc,
                   sizeof(kMwActiveLoCmpPc) / sizeof(kMwActiveLoCmpPc[0]),
                   mw_active_lo_hook);
  mw_install_hooks(kMwActiveHiCmpPc,
                   sizeof(kMwActiveHiCmpPc) / sizeof(kMwActiveHiCmpPc[0]),
                   mw_active_hi_hook);
  mw_install_hooks(kMwSpawnLeftStaPc,
                   sizeof(kMwSpawnLeftStaPc) / sizeof(kMwSpawnLeftStaPc[0]),
                   mw_spawn_left_sta_hook);
  mw_install_hooks(kMwSpawnLeftBmiPc,
                   sizeof(kMwSpawnLeftBmiPc) / sizeof(kMwSpawnLeftBmiPc[0]),
                   mw_spawn_left_bmi_hook);
  mw_install_hooks(kMwSpawnHiCmpPc,
                   sizeof(kMwSpawnHiCmpPc) / sizeof(kMwSpawnHiCmpPc[0]),
                   mw_spawn_hi_hook);
  /* Stage dirty-rect builders: widen lo/hi in A before STA $1E72..$1E78. */
  interp_bridge_set_pre_opcode_hook(MW_SITE_StageWindowLo_A, mw_stage_window_lo_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_StageWindowHi_A, mw_stage_window_hi_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_StageWindowLo_B, mw_stage_window_lo_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_StageWindowHi_B, mw_stage_window_hi_hook);
  /* After STA $1E26/$1E2A — widen WRAM only; leave A for AND → $1E1E. */
  interp_bridge_set_pre_opcode_hook(MW_SITE_BBoxRightAnd_A, mw_bbox_right_and_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_BBoxRightAnd_B, mw_bbox_right_and_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_BBoxLeftBcc_Cam0, mw_bbox_left_bcc_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_BBoxLeftBcc_Cam1, mw_bbox_left_bcc_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_BBoxRightBcs_Cam0, mw_bbox_right_bcs_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_BBoxRightBcs_Cam1, mw_bbox_right_bcs_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_BBoxPaddedLeftBcc, mw_bbox_padded_left_bcc_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_ObjOnscreenEntry, mw_obj_onscreen_entry_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_ObjOnscreenBmi, mw_obj_onscreen_bmi_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_ObjOnscreenCmp, mw_obj_onscreen_cmp_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_ObjGfxCaller, mw_objgfx_caller_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_ObjGateHeartbeat, mw_obj_gate_heartbeat_hook);
  /* Widen $8283AC radius once for all bank-$02 update/despawn callers. */
  interp_bridge_set_pre_opcode_hook(MW_SITE_DistLimitSta, mw_dist_limit_sta_hook);
  /* STA $7F0000,X — door/platform tile patches (debug traffic). */
  interp_bridge_set_pre_opcode_hook(MW_SITE_TilePatchSta7F, mw_tile_patch_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_TileHelper_A, mw_tile_helper_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_TileHelper_B, mw_tile_helper_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_TileHelper_C, mw_tile_helper_hook);
  /* Active-list Y CMP — stage props with sy≥$A8/$E0 still enter the list. */
  interp_bridge_set_pre_opcode_hook(MW_SITE_PropListYGate_A, mw_prop_list_y_gate_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_PropListYGate_B, mw_prop_list_y_gate_hook);
  /* Active-list drawer STA $86 + post-meta (1P / H2H dual). */
  interp_bridge_set_pre_opcode_hook(MW_SITE_DrawSx_1P_A, mw_draw_sx_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_DrawSx_1P_B, mw_draw_sx_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_DrawMeta_1P, mw_draw_meta_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_DrawSx_H2H_A, mw_draw_sx_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_DrawSx_H2H_B, mw_draw_sx_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_DrawMeta_H2H, mw_draw_meta_hook);
  /* Dual drawer: LDA $00,X before TAX←flags&6 — reinforce, never clear. */
  interp_bridge_set_pre_opcode_hook(MW_SITE_DrawPropReinforce, mw_draw_prop_reinforce_hook);
  /* Phase 2b taller H2H: dual stripe rows + spawn Y window + OAM Y bias. */
  interp_bridge_set_pre_opcode_hook(MW_SITE_H2hTallerStripe, mw_h2h_taller_stripe_hook);
  /* Dual $1E9A half-frame → full-frame for mover bbox/distance (not $1E9C). */
  interp_bridge_set_pre_opcode_hook(MW_SITE_H2hDualViewHeight, mw_h2h_dual_view_height_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_SpawnYHeight, mw_spawn_y_height_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_H2hTallerSpawnY_A, mw_h2h_taller_spawn_y_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_H2hTallerSpawnY_B, mw_h2h_taller_spawn_y_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_H2hTallerSpawnY_C, mw_h2h_taller_spawn_y_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_H2hYBias_A, mw_h2h_taller_y_bias_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_H2hYBias_B, mw_h2h_taller_y_bias_hook);
  interp_bridge_set_pre_opcode_hook(MW_SITE_H2hYBias_C, mw_h2h_taller_y_bias_hook);

  /* Full-frame vertical widen: cam tags / bias cancel (CMP imm via ROM poke). */
  {
    /* P1 drawer Y-CMP only ($86/$88). P2 dual CMPs ($8CB6/$8E1F/…) must not
     * force cam0 — that leaked ADC $8A tiles into the cam0 capture buffer. */
    static const uint32_t kYHiTag[] = {
        0x808C71u, 0x808DD3u, 0x808F4Au, 0x8090C1u, 0x80DCACu, 0x80DCE4u,
    };
    static const uint32_t kYE0Tag[] = {
        0x808BC1u, 0x808D13u, 0x808E8Au, 0x808FFAu, 0x80DC28u,
    };
    static const uint32_t kAdc78Tag[] = {
        0x808CBCu, 0x808E25u, 0x808F9Cu, 0x80911Au, 0x80DCEAu,
    };
    /* Cam screen-pos ADCs — definitive capture-buffer tag. */
    static const uint32_t kPosCam0[] = {
        0x808BBAu, 0x808BCCu, 0x808C6Au, 0x808C7Cu, 0x808D0Cu, 0x808D25u,
        0x808DCCu, 0x808DE5u, 0x808E83u, 0x808E95u, 0x808F43u, 0x808F55u,
        0x808FF3u, 0x80900Cu, 0x8090BAu, 0x8090D3u,
    };
    static const uint32_t kPosCam1[] = {
        0x808CAFu, 0x808CC5u, 0x808E18u, 0x808E35u, 0x808F8Fu, 0x808FA5u,
        0x80910Du, 0x80912Au,
    };
    static const uint32_t kOamYStaX[] = {
        0x808BC6u, 0x808C76u, 0x808CBFu, 0x808D18u, 0x808DD8u, 0x808E28u,
        0x808E8Fu, 0x808F4Fu, 0x808F9Fu, 0x808FFFu, 0x8090C6u, 0x80911Du,
    };
    static const uint32_t kOamYStaY[] = {
        0x80DC2Du, 0x80DCB1u, 0x80DCEDu,
    };
    /* STA $14C6 — tile/attr word; staging X/Y already written. */
    static const uint32_t kOamCommitX[] = {
        0x808C18u, 0x808C9Du, 0x808CE6u, 0x808D74u, 0x808E06u, 0x808E56u,
        0x808EE4u, 0x808F76u, 0x808FC6u, 0x80905Bu, 0x8090F4u, 0x80914Bu,
    };
    static const uint32_t kOamCommitY[] = {
        0x80DC32u, 0x80DCB6u, 0x80DCF2u,
    };
    mw_install_hooks(kYHiTag, sizeof(kYHiTag) / sizeof(kYHiTag[0]),
                     mw_h2h_vw_y_hi_tag_hook);
    mw_install_hooks(kYE0Tag, sizeof(kYE0Tag) / sizeof(kYE0Tag[0]),
                     mw_h2h_vw_y_e0_tag_hook);
    mw_install_hooks(kAdc78Tag, sizeof(kAdc78Tag) / sizeof(kAdc78Tag[0]),
                     mw_h2h_vw_adc78_tag_hook);
    mw_install_hooks(kPosCam0, sizeof(kPosCam0) / sizeof(kPosCam0[0]),
                     mw_h2h_vw_pos_cam0_hook);
    mw_install_hooks(kPosCam1, sizeof(kPosCam1) / sizeof(kPosCam1[0]),
                     mw_h2h_vw_pos_cam1_hook);
    mw_install_hooks(kOamYStaX, sizeof(kOamYStaX) / sizeof(kOamYStaX[0]),
                     mw_h2h_vw_oam_y_sta_x_hook);
    mw_install_hooks(kOamYStaY, sizeof(kOamYStaY) / sizeof(kOamYStaY[0]),
                     mw_h2h_vw_oam_y_sta_y_hook);
    mw_install_hooks(kOamCommitX, sizeof(kOamCommitX) / sizeof(kOamCommitX[0]),
                     mw_h2h_vw_oam_commit_x_hook);
    mw_install_hooks(kOamCommitY, sizeof(kOamCommitY) / sizeof(kOamCommitY[0]),
                     mw_h2h_vw_oam_commit_y_hook);
    /* Staging OAM clear at $809169 — reset cam tags + capture buffers. */
    interp_bridge_set_pre_opcode_hook(MW_SITE_H2hOamClear, mw_h2h_vw_oam_clear_hook);
    /* Dual tile×2 overflows 128 slots — wrap index so capture can finish. */
    {
      static const uint32_t kOamFullCpx[] = {
          0x80873Au, 0x808838u, 0x808C1Fu, 0x808CA4u, 0x808CEDu, 0x808D7Bu,
          0x808E0Du, 0x808E5Du, 0x808EEBu, 0x808F7Du, 0x808FCDu, 0x809062u,
          0x8090FBu, 0x809152u,
      };
      mw_install_hooks(kOamFullCpx, sizeof(kOamFullCpx) / sizeof(kOamFullCpx[0]),
                       mw_h2h_oam_full_wrap_hook);
    }
  }
}

/* Widen one lo/hi pair in place. Returns true if modified. */
static bool mw_widen_pair(uint16_t lo_addr, uint16_t hi_addr, int pad) {
  const uint16_t lo = mw_wram16(lo_addr);
  const uint16_t hi = mw_wram16(hi_addr);
  if (hi <= lo)
    return false;
  const unsigned span = (unsigned)(hi - lo);
  /* Native window is exactly +$10 (see ADC #$0010 at $80943F). */
  if (span < 12u || span > 20u)
    return false;

  const uint16_t new_lo = (lo > (uint16_t)pad) ? (uint16_t)(lo - pad) : 0;
  uint32_t new_hi32 = (uint32_t)hi + (uint32_t)pad;
  if (new_hi32 > 0xffffu)
    new_hi32 = 0xffffu;
  const uint16_t new_hi = (uint16_t)new_hi32;
  if (new_lo == lo && new_hi == hi)
    return false;

  mw_wram16_write(lo_addr, new_lo);
  mw_wram16_write(hi_addr, new_hi);
  return true;
}

void MwOnStageWindowStore(uint32_t ram_off, uint32_t pc24) {
  {
    static int armed = -1;
    if (armed < 0) {
      const char *e = getenv("SNESRECOMP_MW_WIDEN_AW");
      if (e && e[0] == '0')
        armed = 0;
      else if (e && e[0] == '1')
        armed = 1;
      else
        armed = kMwWidenAfterWriteEnabled ? 1 : 0;
    }
    if (!armed)
      return;
  }
  if (!g_ws_active || g_ws_extra <= 0 || !mw_can_expand_gameplay())
    return;

  /* Fire once the HI word's last byte lands (16-bit STA = two byte writes).
   * Writer PCs from LLE watch: $809442 ($1E74), $809452 ($1E78). */
  const bool xy_hi =
      (pc24 == 0x809442u && (ram_off == 0x1E74u || ram_off == 0x1E75u));
  const bool y_hi =
      (pc24 == 0x809452u && (ram_off == 0x1E78u || ram_off == 0x1E79u));
  if (!xy_hi && !y_hi)
    return;
  /* Only act on the high byte of the store to avoid double-widen. */
  if ((ram_off & 1u) == 0u)
    return;

  /* $1E72.. are cam-snapped pixels with native span #$0010 (one 16px tile
   * column). Widen by whole tile columns of widescreen budget, in pixels. */
  const int pad_px = mw_ws_tile_pad() * 16;
  if (pad_px <= 0)
    return;

  static unsigned log_left = 12;
  if (xy_hi) {
    const uint16_t lo = mw_wram16(0x1E72);
    const uint16_t hi = mw_wram16(0x1E74);
    if (mw_widen_pair(0x1E72, 0x1E74, pad_px) && log_left > 0) {
      log_left--;
      fprintf(stderr,
              "[mw_widen_aw] pc=$%06X $1E72/$1E74 %u-%u -> %u-%u pad_px=%d\n",
              (unsigned)pc24, (unsigned)lo, (unsigned)hi,
              (unsigned)mw_wram16(0x1E72), (unsigned)mw_wram16(0x1E74),
              pad_px);
    }
  }
  if (y_hi) {
    const uint16_t lo = mw_wram16(0x1E76);
    const uint16_t hi = mw_wram16(0x1E78);
    if (mw_widen_pair(0x1E76, 0x1E78, pad_px) && log_left > 0) {
      log_left--;
      fprintf(stderr,
              "[mw_widen_aw] pc=$%06X $1E76/$1E78 %u-%u -> %u-%u pad_px=%d\n",
              (unsigned)pc24, (unsigned)lo, (unsigned)hi,
              (unsigned)mw_wram16(0x1E76), (unsigned)mw_wram16(0x1E78),
              pad_px);
    }
  }
}

/*
 * Unique tile-column depth beyond the native 16-wide staging strip.
 * Leading = right when scrolling right; trailing = left. A column counts as
 * unique when its 16-row VRAM signature differs from the native edge column
 * (i.e. not an ExtendEdges repeat) and is non-empty.
 */
static void mw_log_unique_leading_cols(void) {
  static int armed = -1;
  if (armed < 0) {
    const char *e = getenv("SNESRECOMP_MW_COLS");
    armed = (kMwUniqueColLogEnabled || (e && e[0] && e[0] != '0')) ? 1 : 0;
  }
  if (!armed || !g_ppu || !mw_can_expand_gameplay())
    return;
  static unsigned last_frame;
  extern int snes_frame_counter;
  if (snes_frame_counter < 2100 || (snes_frame_counter % 60) != 0)
    return;
  if (snes_frame_counter == (int)last_frame)
    return;
  last_frame = (unsigned)snes_frame_counter;

  const int layer = 0;
  const bool big = PPU_bigTiles(g_ppu, layer) != 0;
  const unsigned sh = big ? 4u : 3u;
  const int view_cols = 256 >> (int)sh; /* 16 big / 32 small */
  const int map_cols = PPU_bgTilemapWider(g_ppu, layer) ? 64 : 32;
  const int rows = big ? 16 : 32;
  const uint16_t map_base = (uint16_t)PPU_bgTilemapAdr(g_ppu, layer);
  const uint16_t cam = (uint16_t)(g_ppu->hScroll[layer] & 0x3ff);
  const uint32_t tx0 = (uint32_t)cam >> sh;
  const int x_mask = map_cols - 1;

  uint32_t col_hash[64];
  int populated = 0;
  for (int c = 0; c < map_cols; c++) {
    uint32_t h = 2166136261u;
    int nonzero = 0;
    for (int r = 0; r < rows; r++) {
      int half = (map_cols > 32 && c >= 32) ? 0x400 : 0;
      uint16_t word =
          (uint16_t)(map_base + half + (r << 5) + (c & 31));
      uint16_t t = g_ppu->vram[word & 0x7fff];
      if (t)
        nonzero = 1;
      h ^= t;
      h *= 16777619u;
    }
    col_hash[c] = h;
    if (nonzero)
      populated++;
  }

  const int left_map = (int)(tx0 & (uint32_t)x_mask);
  const int right_map =
      (int)((tx0 + (uint32_t)view_cols - 1u) & (uint32_t)x_mask);
  const uint32_t right_h = col_hash[right_map];
  const uint32_t left_h = col_hash[left_map];

  int unique_lead = 0, repeat_lead = 0;
  int unique_trail = 0, repeat_trail = 0;
  for (int d = 1; d <= 16; d++) {
    int c = (right_map + d) & x_mask;
    if (col_hash[c] == right_h) {
      repeat_lead++;
      continue;
    }
    int nz = 0;
    for (int r = 0; r < rows && !nz; r++) {
      int half = (map_cols > 32 && c >= 32) ? 0x400 : 0;
      uint16_t word =
          (uint16_t)(map_base + half + (r << 5) + (c & 31));
      if (g_ppu->vram[word & 0x7fff])
        nz = 1;
    }
    if (!nz)
      break;
    unique_lead++;
  }
  for (int d = 1; d <= 16; d++) {
    int c = (left_map - d) & x_mask;
    if (col_hash[c] == left_h) {
      repeat_trail++;
      continue;
    }
    int nz = 0;
    for (int r = 0; r < rows && !nz; r++) {
      int half = (map_cols > 32 && c >= 32) ? 0x400 : 0;
      uint16_t word =
          (uint16_t)(map_base + half + (r << 5) + (c & 31));
      if (g_ppu->vram[word & 0x7fff])
        nz = 1;
    }
    if (!nz)
      break;
    unique_trail++;
  }

  fprintf(stderr,
          "[mw_cols] f=%d cam=%u tx0=%u view=%d map_cols=%d populated=%d "
          "unique_lead=%d repeat_lead=%d unique_trail=%d repeat_trail=%d "
          "win=%u-%u/%u-%u\n",
          snes_frame_counter, (unsigned)cam, (unsigned)tx0, view_cols, map_cols,
          populated, unique_lead, repeat_lead, unique_trail, repeat_trail,
          (unsigned)mw_wram16(0x1E72), (unsigned)mw_wram16(0x1E74),
          (unsigned)mw_wram16(0x1E76), (unsigned)mw_wram16(0x1E78));
}

void MwConfigureWidescreen(void) {
  /* Opt-in 16:9 policy. Re-applied every present because ppu_reset / load-state
   * zeroes the margin fields.
   *
   * Expand stage side-scroll (WRAM $10 == $18) and dual-viewport H2H ($1EB2)
   * on Mode 1/2/3 so margins show world tiles (Phase 2a). Known boot/menu/
   * dialogue modes ($2A/$48/$4E/$54/$5A/$00) and other non-stage screens stay
   * centered 4:3 with black pillarbox — title/options must not expand just
   * because a BG H-mirror bit is set.
   *
   * Camera X mirrors (gameplay): $1E2E (primary), $1E1E. $1E72–$1E78 only
   * track last stripe-src rebuild (writers $80943B..); $7F is full-map at
   * stage load. DMA widen at STA $4305 ($8092E4/$809328/$80A838/$80A87F).
   * Margins: $7F prefill + non-void DMA-pad VRAM on BG1 right.
   */
  if (!g_ws_active || !g_ppu || g_ws_extra <= 0)
    return;

  mw_install_dma_widen_hooks();
  mw_h2h_taller_patch_hdma();

  const uint8_t extra = (uint8_t)IntMin(g_ws_extra, kWsExtraMax);

  if (mw_can_expand_gameplay()) {
    PpuSetExtraSpace(g_ppu, extra);
    const uint16_t scroll0_x =
        s_nmi_latched ? s_nmi_hscroll : (uint16_t)g_ppu->hScroll[0];
    const uint16_t scroll0_y =
        s_nmi_latched ? s_nmi_vscroll : (uint16_t)g_ppu->vScroll[0];
    const uint16_t scroll1_x =
        s_nmi_latched ? s_nmi_hscroll1 : (uint16_t)g_ppu->hScroll[1];
    const uint16_t scroll1_y =
        s_nmi_latched ? s_nmi_vscroll1 : (uint16_t)g_ppu->vScroll[1];
    const uint16_t src2 = mw_stage_src_bg2();
    const bool bg2_stream = src2 != 0;
    const bool bg2_wide = (g_ppu->bgXsc[1] & 1) != 0;
    /* BG2 must draw in margins under transparent BG1 — clamping left black
     * holes. Streaming: cam2-keyed shadow. Narrow idle: scroll-keyed shadow
     * + solid extend (not main-cam; that sheared the gutters). Wide: native. */
    PpuSetWidescreenLayerClamp(g_ppu, 0);
    WsShadowSetBlankTile(0, 0);
    const uint32_t world0_x = mw_shadow_world(mw_stage_cam_x(), scroll0_x);
    const uint32_t world0_y = mw_shadow_world(mw_stage_cam_y(), scroll0_y);
    s_shadow_world_x = world0_x;
    s_shadow_world_y = world0_y;
    s_shadow_world_valid = true;
    WsShadowSetWorld(0, world0_x, world0_y);
    WsShadowSetScroll(0, scroll0_x, scroll0_y);
    if (bg2_stream) {
      WsShadowSetBlankTile(1, 0);
      WsShadowSetRetainHistory(1, false);
      WsShadowSetCaptureCols(1, 0);
      WsShadowSetWestKeep(1, 0);
      WsShadowSetRejectEastEcho(1, false);
      const uint32_t world1_x = mw_shadow_world(mw_stage_cam2_x(), scroll1_x);
      const uint32_t world1_y = mw_shadow_world(mw_stage_cam2_y(), scroll1_y);
      WsShadowSetWorld(1, world1_x, world1_y);
      WsShadowSetScroll(1, scroll1_x, scroll1_y);
    } else if (!bg2_wide) {
      /* Non-streaming BG2 (cam2/$1E38 idle) tracks the main camera 1:1.
       * RIGHT: live capture of 22 cols (16 view + 6 east headroom) — the
       * game already DMA's that headroom from the bank-$BB ROM map.
       * LEFT: force ROM west-of-DMA every present (same as live/right).
       * Viewport-relative: west keep = margin+56px slop; reject east wrap
       * echoes so a second door/gate does not appear in the right gutter. */
      WsShadowSetBlankTile(1, 0);
      const uint32_t world1_x = mw_shadow_world(mw_stage_cam_x(), scroll1_x);
      const uint32_t world1_y = mw_shadow_world(mw_stage_cam_y(), scroll1_y);
      WsShadowSetWorld(1, world1_x, world1_y);
      WsShadowSetScroll(1, scroll1_x, scroll1_y);
      WsShadowSetRetainHistory(1, true);
      if (mw_viewport_rel_armed()) {
        /* Native view only — DMA headroom (cols 17..21) is an offscreen
         * buffer; drawing it in the right gutter creates phantom doors. */
        const unsigned sh1 = PPU_bigTiles(g_ppu, 1) ? 4u : 3u;
        const int view_cols = 256 >> (int)sh1;
        const unsigned phase1 =
            (unsigned)world1_x & ((1u << sh1) - 1u);
        WsShadowSetCaptureCols(1, view_cols + (phase1 ? 1 : 0));
        WsShadowSetWestKeep(1, mw_ws_west_keep_tiles());
        WsShadowSetRejectEastEcho(1, true);
      } else {
        WsShadowSetCaptureCols(1, 22);
        WsShadowSetWestKeep(1, 0); /* default kWsWestKeep */
        WsShadowSetRejectEastEcho(1, false);
      }
    }
  } else {
    PpuSetExtraSpaceCentered(g_ppu, extra);
    PpuSetWidescreenLayerClamp(g_ppu, 0);
    s_shadow_world_valid = false;
    WsShadowReset();
  }
}

void MwDrawPpuFrame(void) {
  /* Presentation-only. IRQs are serviced inside RunOneFrameOfGame while the
   * LLE bridge advances the beam — do NOT mutate g_cpu here (that was leaving
   * the guest mid-handler / with a corrupt stack and a permanently blank
   * screen).
   *
   * Run every HDMA channel $420C enables — MW uses ch4/ch6 ($50) for the
   * LucasArts logo, not only the SMW-style 5/6/7 set. */
  /* Turbo / disableRender skips RtlDrawPpuFrame, which normally calls this.
   * Without it WsShadow never registers and left/right margins fall back to
   * VRAM wrap — doors/platforms appear clipped at the 4:3 edge. Idempotent. */
  if (g_ws_active)
    MwConfigureWidescreen();
  else
    mw_install_dma_widen_hooks();
  /* HDMA ROM patch is independent of widescreen margins. */
  mw_h2h_taller_patch_hdma();

  SimpleHdma hdma_chans[8];
  Dma *dma = g_dma;
  /* Prefer live $420C latch; MW also mirrors enable in $1358 (NMI source). */
  uint8 hdmaen = g_snesrecomp_last_hdmaen;
  if (hdmaen == 0)
    hdmaen = g_ram[0x1358];

  dma_startDma(dma, hdmaen, true);
  for (int ch = 0; ch < 8; ch++)
    SimpleHdma_Init(&hdma_chans[ch], &dma->channel[ch]);

  /* Capture VRAM view, then live margin cols ($7F left / DMA-pad right). */
  WsShadowFrame(g_ppu);
  mw_prefill_margins_from_map();
  mw_log_unique_leading_cols();

  for (int i = 0; i <= 224; i++) {
    ppu_runLine(g_ppu, i);
    for (int ch = 0; ch < 8; ch++)
      SimpleHdma_DoLine(&hdma_chans[ch]);
  }
}

/*
 * Present-only top bar for full-frame H2H. Replaces the native dual-seam HUD
 * (HDMA middle 16 lines) that full-frame local skips. Solid fill masks the
 * top FOV transition; marker shows where the other player's camera sits.
 */
static int mw_h2h_top_bar_armed(void) {
  static int v = -1;
  if (v < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_TOP_BAR");
    if (e && e[0] == '0')
      v = 0;
    else
      v = 1;
  }
  return v;
}

static void mw_put_px32(uint8 *row, int x, int w, uint32_t bgra) {
  if (x < 0 || x >= w)
    return;
  memcpy(row + (size_t)x * 4u, &bgra, 4);
}

static void mw_fill_rect32(uint8 *pixels, size_t pitch, int width, int height,
                           int x0, int y0, int x1, int y1, uint32_t bgra) {
  if (x0 < 0)
    x0 = 0;
  if (y0 < 0)
    y0 = 0;
  if (x1 > width)
    x1 = width;
  if (y1 > height)
    y1 = height;
  for (int y = y0; y < y1; y++) {
    uint8 *row = pixels + (size_t)y * pitch;
    for (int x = x0; x < x1; x++)
      mw_put_px32(row, x, width, bgra);
  }
}

/* Down-pointing chevron (into the playfield) centered at (cx, cy). */
static void mw_draw_chevron_down(uint8 *pixels, size_t pitch, int width,
                                 int height, int cx, int cy, int half_w,
                                 int half_h, uint32_t bgra) {
  for (int dy = -half_h; dy <= half_h; dy++) {
    const int y = cy + dy;
    if (y < 0 || y >= height)
      continue;
    /* Taper: full width at top of chevron, point at bottom. */
    const int t = half_h > 0 ? (half_h - dy) : 0;
    const int span = (half_w * (t + 1)) / (half_h + 1);
    uint8 *row = pixels + (size_t)y * pitch;
    for (int dx = -span; dx <= span; dx++)
      mw_put_px32(row, cx + dx, width, bgra);
  }
}

void MwPresentH2hTopBar(uint8 *pixels, size_t pitch, int width, int height,
                        int local_slot) {
  if (!pixels || width <= 0 || height <= 0)
    return;
  if (local_slot != 0 && local_slot != 1)
    return;
  if (!mw_h2h_top_bar_armed() || !MwIsDualViewport())
    return;

  /* Native seam is 16 lines ($80:9670 middle band). */
  enum { kBarH = 16 };
  const int bar_h = height < kBarH ? height : kBarH;

  /* Opaque dark bar + thin bottom edge (BGRA). */
  const uint32_t col_bar = 0xff12161fu;
  const uint32_t col_edge = 0xff3a4558u;
  const uint32_t col_track = 0xff2a3344u;
  const uint32_t col_mark = 0xffe8c547u;
  const uint32_t col_mark_dim = 0xff8a7028u;

  mw_fill_rect32(pixels, pitch, width, height, 0, 0, width, bar_h, col_bar);
  mw_fill_rect32(pixels, pitch, width, height, 0, bar_h - 1, width, bar_h,
                 col_edge);

  /* Horizontal track through the bar. */
  const int track_y0 = bar_h / 2 - 1;
  const int track_y1 = track_y0 + 2;
  const int pad_x = width > 40 ? 12 : 4;
  mw_fill_rect32(pixels, pitch, width, height, pad_x, track_y0, width - pad_x,
                 track_y1, col_track);

  /* Center tick. */
  const int mid = width / 2;
  mw_fill_rect32(pixels, pitch, width, height, mid - 1, 3, mid + 1, bar_h - 3,
                 col_edge);

  /* Opponent direction from dual cams (follow each mech). */
  uint16_t loc_x, loc_y, oth_x, oth_y;
  if (local_slot == 0) {
    loc_x = s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16u);
    loc_y = s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18u);
    oth_x = s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1Au);
    oth_y = s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1Cu);
  } else {
    loc_x = s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1Au);
    loc_y = s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1Cu);
    oth_x = s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16u);
    oth_y = s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18u);
  }
  const int dx = (int)oth_x - (int)loc_x;
  const int dy = (int)oth_y - (int)loc_y;

  /* Map ΔX into the track; ±512 world ≈ full bar travel. */
  const int travel = (width - 2 * pad_x) / 2;
  int mark_off = (dx * travel) / 512;
  if (mark_off < -travel)
    mark_off = -travel;
  if (mark_off > travel)
    mark_off = travel;
  const int mark_x = mid + mark_off;

  /* Vertical cue: brighter / taller when opponent is far below; dimmer +
   * inset when above (still on the bar). */
  int half_h = 4;
  uint32_t mark_col = col_mark;
  if (dy > 64) {
    half_h = 5;
    mark_col = col_mark;
  } else if (dy < -64) {
    half_h = 3;
    mark_col = col_mark_dim;
  }
  mw_draw_chevron_down(pixels, pitch, width, height, mark_x, bar_h / 2, 6,
                       half_h, mark_col);

  /* End-cap arrows when opponent is off the ends of the track. */
  if (mark_off <= -travel + 2) {
    mw_fill_rect32(pixels, pitch, width, height, pad_x, 4, pad_x + 3, bar_h - 4,
                   col_mark);
  } else if (mark_off >= travel - 2) {
    mw_fill_rect32(pixels, pitch, width, height, width - pad_x - 3, 4,
                   width - pad_x, bar_h - 4, col_mark);
  }

  static int logged;
  if (!logged) {
    logged = 1;
    fprintf(stderr,
            "[mw_rtl] H2H top bar armed (h=%d) — opponent locator on full-frame "
            "local (SNESRECOMP_MW_H2H_TOP_BAR=0 to disable)\n",
            bar_h);
  }
}

void MwDrawPpuFrameLocalFull(int local_slot) {
  /* Present-only full-frame H2H: rebuild local cam strips into VRAM (then
   * restore), re-center half-framed cam to 1P mid-screen, 1P-style margins,
   * 224-line draw. Sim dual state is unchanged. */
  if (!g_ppu || (local_slot != 0 && local_slot != 1)) {
    MwDrawPpuFrame();
    return;
  }

  s_present_h2h_local_slot = local_slot;
  mw_install_dma_widen_hooks();
  if (mw_h2h_taller_armed())
    mw_h2h_taller_patch_hdma();

  /* Present origin = NMI-latched dual cameras (full world, not fine-phase
   * scroll mirrors). P1: $1E16/$1E18. P2: $1E1A/$1E1C. Using $1E32/$1E62 for
   * P2 pinned the view near map origin and broke Y follow.
   * Latch once for the whole present — BG1 + prop OAM must not re-read. */
  uint16_t cam_x, cam_y_raw;
  if (local_slot == 0) {
    cam_x = s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16u);
    cam_y_raw = s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18u);
    s_present_oth_x = s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1Au);
    s_present_oth_y = s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1Cu);
  } else {
    cam_x = s_nmi_latched ? s_nmi_cam2_x : mw_wram16(0x1E1Au);
    cam_y_raw = s_nmi_latched ? s_nmi_cam2_y : mw_wram16(0x1E1Cu);
    s_present_oth_x = s_nmi_latched ? s_nmi_cam_x : mw_wram16(0x1E16u);
    s_present_oth_y = s_nmi_latched ? s_nmi_cam_y : mw_wram16(0x1E18u);
  }
  s_present_loc_x = cam_x;
  s_present_loc_y = cam_y_raw;
  s_present_cam_valid = 1;

  /*
   * Half→full Y recenter (no OAM focus) — only when vert-widen is OFF.
   * Vert-widen already emits a ~224-tall window vs cam_raw (CMP→#$E0 /
   * #$FF70 + staging +$78). Adding y_bg≈64 then pushes dual-bottom stage
   * props (capture sy≈210–225) to ny≥224: brown leaves the strip, OAM
   * either vanishes (skip_y) or an unshifted hack floats the stripe ~64px
   * above the BG1 body. With vert-widen on, y_bg=y_oam=0 so OAM and BG1
   * share cam_raw (aligned, platforms stay on-screen).
   * Non-VW path: snap to the $7E:42B3 16px row bucket (not merely PPU
   * tile height) so cam_y−y_bg stays on map phase — 8px-only snap left
   * $C382 stripe vs brown ~1 tile apart.
   */
  enum { kMwH2hBgNudge = 8 };
  enum { kMwH2hBgPhasePx = 16 }; /* match 42B3 (cam_y & ~15) */
  const int oam_bias = mw_h2h_oam_y_bias();
  int y_bg = 0;
  if (!mw_h2h_vert_widen_armed()) {
    y_bg = mw_h2h_full_frame_y_shift() + kMwH2hBgNudge;
    y_bg -= y_bg % kMwH2hBgPhasePx;
  }
  const int y_oam = y_bg;
  const uint16_t cam_y = mw_u16_sub_sat(cam_y_raw, y_bg);
  const int oam_delta = -oam_bias + y_oam;
  /* BG1: recenter + $7F strip rebuild. BG2: never $7F-rebuild on full-frame
   * H2H — dual dirty frames false-arm $1E38 / $7F and flash space garbage.
   * Narrow idle BG2 uses the 1P cam-track + retainHistory path (elevators). */
  s_present_h2h_full_frame = 1;
  const int bg2_stream = 0;
  /* Full world cam for $7F/42B3 rebuild, prop blank, OAM, coldump. */
  const uint16_t h0 = cam_x;
  const uint16_t v0 = cam_y;
  /* PPU BG1HOFS: fine phase only — strip is view-relative at VRAM col 0
   * (matches game DMA). Full HOFS + world-absolute VRAM caused the stand
   * to FOV-follow within a tile then snap opposite on coarse step. */
  const uint16_t h0_ppu = (uint16_t)(cam_x & 15u);
  const uint16_t v0_ppu = v0; /* Y stays world-absolute rows + full VOFS */
  uint16_t h1, v1;
  if (local_slot == 0) {
    h1 = s_nmi_latched ? s_nmi_wram_h1 : mw_wram16(0x1E42u);
    v1 = s_nmi_latched ? s_nmi_wram_v1 : mw_wram16(0x1E60u);
  } else {
    h1 = s_nmi_latched ? s_nmi_wram_h1_p2 : mw_wram16(0x1E46u);
    v1 = s_nmi_latched ? s_nmi_wram_v1_p2 : mw_wram16(0x1E64u);
  }
  /* Idle narrow BG2 ($D5B8 elevators): H from NMI PPU fine (shared layer).
   * Dual WRAM $1E42/$1E46 disagrees with hs1 inside 0..15 (~±10–14) and used
   * to also spike to cam via h1==0→h0 — both read as left elevator/brown twitch.
   * Keep WRAM v1 (parallax ≈cam/2); only fill zero from PPU. Never h1=cam. */
  const int bg2_wide = (g_ppu->bgXsc[1] & 1) != 0;
  if (!bg2_wide) {
    h1 = s_nmi_latched ? s_nmi_hscroll1 : (uint16_t)g_ppu->hScroll[1];
    if (v1 == 0) {
      v1 = s_nmi_latched ? s_nmi_vscroll1 : (uint16_t)g_ppu->vScroll[1];
    }
  }

  if (!MwIsDualViewport())
    mw_prop_home_reset();

  /* Refresh this slot's snap from local 42B3 $7F (src_opt bypasses any
   * residual gate). Keep prior if this window is all weak. */
  {
    uint16_t snap_src = mw_map_src_from_42b3(cam_x, cam_y);
    if (!snap_src)
      snap_src = mw_map_src_from_42b3(h0, v0);
    mw_bg1_snap_capture(local_slot, cam_x, cam_y, snap_src);
  }
  const int bg1_rebuild = mw_h2h_bg1_rebuild_armed();
  mw_present_rebuild_local_strips(cam_x, cam_y, h0, v0, h1, v1, bg1_rebuild,
                                  bg2_stream, cam_y_raw);
  const uint16_t oth_x = s_present_oth_x;
  const uint16_t oth_y = s_present_oth_y;
  /* Always suppress foreign movers (rebuild off still leaves $7F ghosts). */
  mw_present_align_stage_prop_bg1(local_slot, cam_x, cam_y_raw, oth_x, oth_y,
                                  h0, v0);
  mw_present_restamp_bg2_from_rom(local_slot, v1);

  uint16_t oam_backup[0x100];
  uint8_t high_oam_backup[0x20];
  memcpy(oam_backup, g_ppu->oam, sizeof(oam_backup));
  memcpy(high_oam_backup, g_ppu->highOam, sizeof(high_oam_backup));
  const int extra_px =
      (g_ws_active && g_ws_extra > 0) ? IntMin(g_ws_extra, kWsExtraMax) : 0;
  /* Opt-in guest 1P drawer; else per-cam OAM capture / tag cull / Y-half. */
  const int obj_oam =
      mw_present_oam_from_objects(cam_x, cam_y_raw);
  if (!obj_oam) {
    int from_cap = 0;
    if (mw_h2h_oam_cull_armed()) {
      if (mw_h2h_vert_widen_armed()) {
        from_cap =
            mw_present_oam_from_cam_capture(local_slot, extra_px, y_oam);
        if (!from_cap)
          mw_present_oam_cull_by_tag(local_slot, extra_px);
      } else {
        mw_present_oam_cull_other_half(local_slot, extra_px);
      }
    }
    /* Capture already wrote final Y; only bias-undo fallback paths. */
    if (!from_cap && oam_delta)
      mw_oam_y_add(oam_delta);
  } else if (y_oam) {
    /* 1P OAM has no +$70 bias — only half→full Y recentering. */
    mw_oam_y_add(y_oam);
  }

  if (g_ws_active && g_ws_extra > 0) {
    const uint8_t extra = (uint8_t)IntMin(g_ws_extra, kWsExtraMax);
    if (mw_can_expand_gameplay()) {
      PpuSetExtraSpace(g_ppu, extra);
      PpuSetWidescreenLayerClamp(g_ppu, 0);
      const uint32_t world0_x = (uint32_t)(cam_x & (uint16_t)~15u);
      const uint32_t world0_y = (uint32_t)cam_y;
      /* BG2 shadow/world keyed to its present scroll (native idle, cam when
       * streaming) — never force idle world1_x = cam_x. */
      const uint32_t world1_x = (uint32_t)h1;
      const uint32_t world1_y = (uint32_t)v1;
      s_shadow_world_x = (uint32_t)cam_x;
      s_shadow_world_y = world0_y;
      s_shadow_world_valid = true;
      WsShadowSetBlankTile(0, 0);
      /* Coarse world + fine HOFS matches view-relative BG1 strip. */
      WsShadowSetWorld(0, world0_x, world0_y);
      WsShadowSetScroll(0, h0_ppu, v0_ppu);
      /* BG1: never retainHistory on full-frame H2H — margin lag reads as
       * platform/terrain momentum during cam whips. 4:3 body is VRAM. */
      WsShadowSetRetainHistory(0, false);
      WsShadowSetBlankTile(1, 0);
      if (!bg2_wide) {
        /* Narrow idle BG2 (elevators): 1P retainHistory + west ROM. Do not
         * require a $BB DMA stamp — that gate left elevators invisible. */
        const uint32_t idle_x = mw_shadow_world(cam_x, h1);
        const uint32_t idle_y = mw_shadow_world(cam_y, v1);
        WsShadowSetWorld(1, idle_x, idle_y);
        WsShadowSetScroll(1, h1, v1);
        WsShadowSetRetainHistory(1, true);
        if (mw_viewport_rel_armed()) {
          const unsigned sh1 = PPU_bigTiles(g_ppu, 1) ? 4u : 3u;
          const int view_cols = 256 >> (int)sh1;
          const unsigned phase1 =
              (unsigned)idle_x & ((1u << sh1) - 1u);
          WsShadowSetCaptureCols(1, view_cols + (phase1 ? 1 : 0));
          WsShadowSetWestKeep(1, mw_ws_west_keep_tiles());
          WsShadowSetRejectEastEcho(1, true);
        } else {
          WsShadowSetCaptureCols(1, 22);
          WsShadowSetWestKeep(1, 0);
          WsShadowSetRejectEastEcho(1, false);
        }
      } else {
        /* Wide idle BG2: native scrolls, no 1P history gutters. */
        WsShadowSetWorld(1, world1_x, world1_y);
        WsShadowSetScroll(1, h1, v1);
        WsShadowSetRetainHistory(1, false);
        WsShadowSetCaptureCols(1, 0);
        WsShadowSetWestKeep(1, 0);
        WsShadowSetRejectEastEcho(1, false);
      }
    } else {
      PpuSetExtraSpaceCentered(g_ppu, extra);
      PpuSetWidescreenLayerClamp(g_ppu, 0);
      s_shadow_world_valid = false;
      WsShadowReset();
    }
  }

  g_ppu->hScroll[0] = h0_ppu;
  g_ppu->vScroll[0] = v0_ppu;
  g_ppu->hScroll[1] = h1;
  g_ppu->vScroll[1] = v1;
  /* Coldump: world cam for rebuild (h0) + fine HOFS actually applied. */
  {
    extern int snes_frame_counter;
    s_coldump_ph0 = h0;
    s_coldump_pv0 = v0;
    s_coldump_ph1 = h1;
    s_coldump_pv1 = v1;
    s_coldump_ploc_x = cam_x;
    s_coldump_ploc_y = cam_y_raw;
    s_coldump_pscroll_frame = snes_frame_counter;
  }

  WsShadowFrame(g_ppu);
  mw_prefill_margins_from_map_ex(&cam_x, &cam_y, h0, v0, h1, v1);
  /* Prefill/`$7F` east can repaint mover brown — re-hide after margins. */
  mw_present_align_stage_prop_bg1(local_slot, cam_x, cam_y_raw, oth_x, oth_y,
                                  h0, v0);
  /* After final align — stand map/BG1 probe must not be wiped by prefill. */
  mw_stand_bg1_probe(local_slot, cam_x, cam_y_raw, h0, v0);

  static int logged;
  if (!logged) {
    logged = 1;
    fprintf(stderr,
            "[mw_rtl] H2H full-frame local (slot=%d) y_bg=%d y_oam=%d "
            "oam_delta=%d bg2_stream=%d cam=$%04X/$%04X bg2v=$%04X "
            "vram_words=%d\n",
            local_slot, y_bg, y_oam, oam_delta, bg2_stream, (unsigned)cam_x,
            (unsigned)cam_y, (unsigned)v1, s_vram_save_n);
  }

  /* No dual HDMA — it rewrites scrolls mid-frame for the other half and
   * shears the local full-frame view even if we poke scrolls each line. */
  for (int i = 0; i <= 224; i++) {
    g_ppu->hScroll[0] = h0_ppu;
    g_ppu->vScroll[0] = v0_ppu;
    g_ppu->hScroll[1] = h1;
    g_ppu->vScroll[1] = v1;
    ppu_runLine(g_ppu, i);
  }

  /* Always tag which slot LocalFull presented — cam_capture only did this
   * before; object-drawer OAM left present.slot=-1 in coldump. */
  {
    extern int snes_frame_counter;
    s_coldump_present_slot = local_slot;
    s_coldump_present_frame = snes_frame_counter;
  }

  /* Dump before restore — mism_local must reflect presented VRAM, not sim. */
  mw_coldump_tick(local_slot);

  memcpy(g_ppu->oam, oam_backup, sizeof(oam_backup));
  memcpy(g_ppu->highOam, high_oam_backup, sizeof(high_oam_backup));
  mw_vram_restore();
  s_present_h2h_full_frame = 0;
  s_present_h2h_local_slot = -1;
  s_present_cam_valid = 0;
}

/* LLE host execution cursor — not in snes_saveload (that blob holds the
 * unused snes->cpu). Persist via RtlGameInfo.state_save_extra so Shift+F1
 * / script loadstate can resume past menus instead of cold-booting.
 * Cleared by MwSessionReset on rematch (see soft-return sticky latch). */
static bool s_lle_did_reset = false;
static uint32_t s_lle_resume_pc = 0;
/* s_lle_host_frames declared with coldump latch (above). */
static bool s_lle_extra_loaded = false; /* set by state_load_extra */

void MwSessionReset(void) {
  s_lle_did_reset = false;
  s_lle_resume_pc = 0;
  s_lle_host_frames = 0;
  s_lle_extra_loaded = false;

  s_nmi_latched = false;
  s_nmi_cam_x = s_nmi_cam_y = 0;
  s_nmi_src_bg1 = s_nmi_src_bg2 = 0;
  s_nmi_cam2_x = s_nmi_cam2_y = 0;
  s_nmi_hscroll = s_nmi_vscroll = 0;
  s_nmi_hscroll1 = s_nmi_vscroll1 = 0;
  s_sticky_src_bg1 = s_sticky_src_bg2 = 0;
  s_sticky_src_bg1_frame = s_sticky_src_bg2_frame = -2;
  s_sticky_src_bg1_slot[0] = s_sticky_src_bg1_slot[1] = 0;
  s_sticky_src_bg1_slot_frame[0] = s_sticky_src_bg1_slot_frame[1] = -2;
  memset(s_bg1_snap, 0, sizeof(s_bg1_snap));
  s_present_h2h_full_frame = 0;
  s_present_h2h_local_slot = -1;
  s_present_cam_valid = 0;
  s_shadow_world_x = s_shadow_world_y = 0;
  s_shadow_world_valid = false;
  memset(s_oam_right_hints, 0, sizeof(s_oam_right_hints));
  memset(s_oam_cam_tag, 0xff, sizeof(s_oam_cam_tag));
  s_oam_cam_tag_init = 1;
  s_oam_draw_cam = 0;
  s_oam_saw_adc78 = 0;
  mw_cam_oam_reset();
  mw_prop_home_reset();
  memset(s_coldump_mot_valid, 0, sizeof(s_coldump_mot_valid));
  memset(s_coldump_near_valid, 0, sizeof(s_coldump_near_valid));
  memset(s_coldump_bg_try, 0, sizeof(s_coldump_bg_try));
  memset(s_coldump_bg_hit, 0, sizeof(s_coldump_bg_hit));
  memset(s_stand_latch_mid_have, 0, sizeof(s_stand_latch_mid_have));
  memset(s_stand_latch_pres_have, 0, sizeof(s_stand_latch_pres_have));
  s_coldump_bg_slot = -1;
  s_coldump_last_f[0] = s_coldump_last_f[1] = -1;
  s_coldump_last_hash[0] = s_coldump_last_hash[1] = 0;
  s_spawn_left_34 = s_spawn_left_38 = 0;
  s_spawn_left_34_valid = s_spawn_left_38_valid = false;
  s_bg2_rom_valid_mask = 0;
  s_bg2_rom_words_mask = 0;
  s_bg2_rom_map_base = 0;
  s_bg2_rom_locate_failed = false;
  s_bg2_rom_idle = 0;
  memset(s_bg2_rom_row_frame, 0, sizeof(s_bg2_rom_row_frame));
  memset(s_bg2_rom_row_owner, -1, sizeof(s_bg2_rom_row_owner));
  s_coldump_bg2_present_v1 = 0;
  s_coldump_bg2_restamp = 0;
  s_coldump_bg2_own_mask = 0;
  s_bg2_west_last_buf_ty0 = 0xffffffffu;
  s_bg2_west_last_addr0 = 0xffffu;
  s_tile7f_hits = 0;
  s_tile_helper_hits = 0;

  /* g_cpu / frame counters survive snes_free — zero so SnesInit beam sync and
   * the next RunOneFrameOfGame cold-boot from RESET, not a stale WAI. */
  cpu_state_init(&g_cpu, g_ram);
  snes_frame_counter = 0;
  g_apu_last_sync_master = 0;
  WsShadowReset();

  fprintf(stderr, "[mw_rtl] session reset — next frame will LLE cold boot\n");
}

enum { kMwLleSaveMagic = 0x314C574Du }; /* "MWL1" little-endian */

typedef struct MwLleSaveChunk {
  uint32_t magic;
  uint32_t resume_pc24;
  uint16_t A, X, Y, S, D;
  uint8_t DB, PB, P;
  uint8_t m_flag, x_flag, emulation;
  uint8_t flag_N, flag_V, flag_Z, flag_C, flag_I, flag_D;
  uint8_t _pad;
  uint64_t cycles;
  uint64_t master_cycles;
  uint32_t host_frames;
} MwLleSaveChunk;

void MwStateSaveExtra(SaveLoadInfo *sli) {
  MwLleSaveChunk c;
  memset(&c, 0, sizeof(c));
  c.magic = kMwLleSaveMagic;
  c.resume_pc24 = s_lle_resume_pc;
  c.A = g_cpu.A;
  c.X = g_cpu.X;
  c.Y = g_cpu.Y;
  c.S = g_cpu.S;
  c.D = g_cpu.D;
  c.DB = g_cpu.DB;
  c.PB = g_cpu.PB;
  c.P = g_cpu.P;
  c.m_flag = g_cpu.m_flag;
  c.x_flag = g_cpu.x_flag;
  c.emulation = g_cpu.emulation;
  c.flag_N = g_cpu._flag_N;
  c.flag_V = g_cpu._flag_V;
  c.flag_Z = g_cpu._flag_Z;
  c.flag_C = g_cpu._flag_C;
  c.flag_I = g_cpu._flag_I;
  c.flag_D = g_cpu._flag_D;
  c.cycles = g_cpu.cycles;
  c.master_cycles = g_cpu.master_cycles;
  c.host_frames = (uint32_t)s_lle_host_frames;
  sli->func(sli, &c, sizeof(c));
}

void MwStateLoadExtra(SaveLoadInfo *sli, uint32_t version) {
  (void)version;
  MwLleSaveChunk c;
  memset(&c, 0, sizeof(c));
  sli->func(sli, &c, sizeof(c));
  if (c.magic != kMwLleSaveMagic) {
    fprintf(stderr, "[mw_rtl] save extra: bad magic $%08X — ignoring LLE chunk\n",
            (unsigned)c.magic);
    s_lle_extra_loaded = false;
    return;
  }
  g_cpu.A = c.A;
  g_cpu.X = c.X;
  g_cpu.Y = c.Y;
  g_cpu.S = c.S;
  g_cpu.D = c.D;
  g_cpu.DB = c.DB;
  g_cpu.PB = c.PB;
  g_cpu.P = c.P;
  g_cpu.m_flag = c.m_flag;
  g_cpu.x_flag = c.x_flag;
  g_cpu.emulation = c.emulation;
  g_cpu._flag_N = c.flag_N;
  g_cpu._flag_V = c.flag_V;
  g_cpu._flag_Z = c.flag_Z;
  g_cpu._flag_C = c.flag_C;
  g_cpu._flag_I = c.flag_I;
  g_cpu._flag_D = c.flag_D;
  g_cpu.host_return_valid = 0;
  g_cpu.cycles = c.cycles;
  g_cpu.master_cycles = c.master_cycles;
  g_cpu.ram = g_ram;
  s_lle_resume_pc = c.resume_pc24 & 0xFFFFFFu;
  s_lle_host_frames = c.host_frames ? c.host_frames : 1u;
  s_lle_extra_loaded = true;
  fprintf(stderr, "[mw_rtl] LLE load extra resume=$%06X master=%llu frames=%u\n",
          (unsigned)s_lle_resume_pc,
          (unsigned long long)g_cpu.master_cycles, s_lle_host_frames);
}

void MwOnStateLoaded(uint32_t version) {
  (void)version;
  /* Re-arm presentation hooks; they are process-local, not in the snapshot. */
  mw_install_dma_widen_hooks();
  s_nmi_latched = false;
  s_bg2_rom_valid_mask = 0;
  s_bg2_rom_words_mask = 0;
  s_bg2_rom_map_base = 0;
  s_bg2_rom_locate_failed = false;
  s_bg2_rom_idle = 0;
  memset(s_bg2_rom_row_frame, 0, sizeof(s_bg2_rom_row_frame));
  memset(s_bg2_rom_row_owner, -1, sizeof(s_bg2_rom_row_owner));
  if (s_lle_extra_loaded) {
    s_lle_did_reset = true; /* skip cold RESET — resume mid-game */
    fprintf(stderr, "[mw_rtl] state loaded — skipping LLE cold boot\n");
  } else {
    /* Snapshot has WRAM/PPU but no LLE cursor (pre-fix Shift+F1). Force a
     * clean cold boot rather than hybrid mid-game WRAM + RESET PC. */
    s_lle_did_reset = false;
    s_lle_resume_pc = 0;
    s_lle_host_frames = 0;
    fprintf(stderr,
            "[mw_rtl] state loaded without LLE chunk — will cold boot "
            "(re-save with Shift+F1 after this build)\n");
  }
  s_lle_extra_loaded = false;
}

void RunOneFrameOfGame(void) {
  /* Whole-program LLE bring-up:
   *  - boot from RESET vector
   *  - inject NMI only after the game enables it ($4200 bit 7)
   *  - service IRQs when the bridge yields with inIrq set
   *  - hard-cap each host frame to one NTSC frame of master clocks so boot
   *    clear loops can't monopolize the quiescent bridge */
  if (!s_lle_did_reset) {
    cpu_state_init(&g_cpu, g_ram);
    s_lle_resume_pc = mw_read_vector_pc24(0xFFFC);
    fprintf(stderr, "[mw_rtl] LLE boot from RESET vector $%06X\n",
            (unsigned)s_lle_resume_pc);
    mw_install_dma_widen_hooks();
    s_lle_did_reset = true;
    s_lle_host_frames = 0;
  }

  const uint64_t frame_end =
      g_cpu.master_cycles + (uint64_t)kMwMasterClocksPerFrame;

  /* Hardware NMI is gated by NMITIMEN — firing earlier corrupts SEI boot. */
  if (s_lle_host_frames > 0 && g_snes->nmiEnabled) {
    g_snes->inNmi = true;
    mw_run_nmi(frame_end);
  }

  /* Run until one frame of master time elapses, servicing IRQs as they latch.
   *
   * WAI means "sleep until the next interrupt". End the host frame so the
   * leading NMI on the next RtlRunFrame wakes it — that is one vblank later.
   * Firing NMI immediately and continuing used to pump many WAI→NMI loops
   * inside a single present (logo/menu ran uncapped-fast). */
  int slices = 0;
  while (g_cpu.master_cycles < frame_end && slices < 64) {
    slices++;
    interp_bridge_set_master_deadline(frame_end);
    (void)interp_bridge_run_until_quiescent(&g_cpu, s_lle_resume_pc);
    interp_bridge_set_master_deadline(0);
    s_lle_resume_pc = interp_bridge_lle_resume_pc();

    if (g_snes->inIrq) {
      mw_run_interrupt(0xFFEE, frame_end);
      continue;
    }
    if (interp_bridge_lle_took_wai())
      break;
    /* Quiescent wait (for next NMI) or deadline hit. */
    break;
  }

  s_lle_host_frames++;

  if (s_lle_host_frames <= 10 || (s_lle_host_frames % 60) == 0) {
    const uint8 inidisp = g_ppu ? g_ppu->inidisp : 0;
    const uint8 bright_mirror = g_ram[0x1356]; /* NMI restores $2100 from here */
    const uint16 hook = (uint16)(g_ram[0] | (g_ram[1] << 8));
    fprintf(stderr,
            "[mw_rtl] host_frame=%u resume=$%06X master=%llu slices=%d "
            "nmiEn=%d irqEn=%d/%d inidisp=$%02X($1356=$%02X) "
            "hdmaen=$%02X($1358=$%02X) hook=$%02X:%04X\n",
            s_lle_host_frames, (unsigned)s_lle_resume_pc,
            (unsigned long long)g_cpu.master_cycles, slices,
            (int)g_snes->nmiEnabled, (int)g_snes->vIrqEnabled,
            (int)g_snes->hIrqEnabled, (unsigned)inidisp,
            (unsigned)bright_mirror, (unsigned)g_snesrecomp_last_hdmaen,
            (unsigned)g_ram[0x1358], g_ram[2], hook);
  }
}
