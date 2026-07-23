#ifndef MW_RTL_H_
#define MW_RTL_H_
#include "common_rtl.h"
#include "common_cpu_infra.h"

struct SaveLoadInfo;

void RunOneFrameOfGame(void);
void MwDrawPpuFrame(void);

/* Present-only: full 224-line framebuffer from one H2H camera (local slot
 * 0 = P1 / $1E2E,$1E5E; 1 = P2 / $1E32,$1E62). Default ON for netplay.
 * Opt out: SNESRECOMP_MW_H2H_FULL_FRAME=0 (legacy half-crop).
 * Column/mover offline dump: SNESRECOMP_MW_COLDUMP=path.jsonl (or =1). */
void MwDrawPpuFrameLocalFull(int local_slot);
bool MwH2hFullFrameLocalArmed(void);

/* Present-only: solid top bar (masks top FOV transition) + opponent-direction
 * marker for full-frame H2H. Call after RtlWidescreenPresent. Opt out:
 * SNESRECOMP_MW_H2H_TOP_BAR=0. */
void MwPresentH2hTopBar(uint8 *pixels, size_t pitch, int width, int height,
                        int local_slot);

/* Per-frame widescreen policy. Call from RtlDrawPpuFrame when g_ws_active. */
void MwConfigureWidescreen(void);

/* True when MW has dual viewports armed ($1EB2 != 0) — H2H split-screen
 * battle uses top = P1 / bottom = P2. Present-only; does not alter sim. */
bool MwIsDualViewport(void);

/* Called from cpu_write8/16 after a store that may touch stage-window WRAM.
 * pc24 is g_interp816_cur_pc (opcode address). Dirty-window only — stripe DMA
 * length/source are adjusted in mw_dma_size_hook (mw_rtl.c). */
void MwOnStageWindowStore(uint32_t ram_off, uint32_t pc24);

/* Called from dma_startDma when a channel targets VRAM. Captures BG2
 * ROM-strip row bases (bank $BB map) for left-margin prefetch. */
void MwNotifyBg2MapDma(uint8_t aBank, uint16_t aAdr, uint16_t vmadd,
                       uint16_t size);

/* LLE save-state extras: CpuState + resume PC (not in snes->cpu blob). */
void MwStateSaveExtra(struct SaveLoadInfo *sli);
void MwStateLoadExtra(struct SaveLoadInfo *sli, uint32_t version);
void MwOnStateLoaded(uint32_t version);

/* Clear process-lifetime LLE / present session state before rematch SnesInit.
 * Soft-return leaves s_lle_did_reset set; without this the next session resumes
 * a stale WAI PC on a wiped chip (blank screen, nmiEn=0). Safe on first boot. */
void MwSessionReset(void);

#endif  /* MW_RTL_H_ */
