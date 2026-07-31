/* Metal Warriors thin wrapper around snesrecomp portable codegen host. */
#ifndef MW_CODEGEN_SETUP_H
#define MW_CODEGEN_SETUP_H

#include "recomp_launcher.h"

#ifdef __cplusplus
extern "C" {
#endif

void mw_codegen_setup_apply(RecompLauncherCGameInfo* gi);
void mw_codegen_relaunch_or_exit(const char* rom_path);

#ifdef __cplusplus
}
#endif

#endif /* MW_CODEGEN_SETUP_H */
