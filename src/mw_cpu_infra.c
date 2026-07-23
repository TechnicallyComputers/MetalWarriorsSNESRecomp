#include "common_cpu_infra.h"
#include "mw_rtl.h"

const RtlGameInfo kSmwGameInfo = {
  .title = "metalwarriors",
  .initialize = NULL,
  .run_frame = &RunOneFrameOfGame,
  .draw_ppu_frame = &MwDrawPpuFrame,
  .save_name_prefix = "save",
  .state_save_extra = &MwStateSaveExtra,
  .state_load_extra = &MwStateLoadExtra,
  .on_state_loaded = &MwOnStateLoaded,
  .session_reset = &MwSessionReset,
};
