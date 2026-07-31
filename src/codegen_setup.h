/* Local ROM → C generate setup for the recomp-ui first-run wizard. */
#ifndef MW_CODEGEN_SETUP_H
#define MW_CODEGEN_SETUP_H

#include "recomp_launcher.h"

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fill prepare_* / needs_setup on gi when a source tree + snesrecomp SDK
 * are discoverable. No-op for player zips without tools. */
void mw_codegen_setup_apply(RecompLauncherCGameInfo* gi);

/* True when project_root/src/gen/dispatch_v2.c is missing. */
int mw_codegen_sources_missing(void);

/* After recomp_launcher_run_window returns RELAUNCH: persist rom.cfg and
 * exec the rebuilt binary. Does not return on success. */
void mw_codegen_relaunch_or_exit(const char* rom_path);

#ifdef __cplusplus
}
#endif

#endif /* MW_CODEGEN_SETUP_H */
