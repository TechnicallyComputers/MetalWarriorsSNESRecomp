/* Metal Warriors config for the portable snesrecomp codegen host. */

#include "codegen_setup.h"

#include "snesrecomp_codegen_host.h"

/* Keep digests in sync with README / tools/regen.sh / main.c. */
static const SnesrecompCodegenHostConfig kMwCodegenConfig = {
    .display_name = "Metal Warriors",
    .project_root_env = "METALWARRIORS_PROJECT_ROOT",
    .build_dir_env = "METALWARRIORS_BUILD_DIR",
    .force_setup_env = "METALWARRIORS_FORCE_SETUP",
    .snesrecomp_cli_relpath = "snesrecomp/snesrecomp_cli.py",
    .seed_cfg_relpath = "recomp/bank00.cfg",
    .cfg_dir = "recomp",
    .out_dir = "src/gen",
    .funcs_h = "recomp/funcs.h",
    .gen_marker_relpath = "src/gen/dispatch_v2.c",
    .build_dir_name = "build",
    .cmake_target = "MetalWarriorsSNESRecomp",
    .exe_basename = "MetalWarriorsSNESRecomp",
    .expected_crc32 = "f2ab92d4",
    .expected_sha256 =
        "0d7f875877fe856066cfb39b4ecdbbe7d48393a75770720876c94419f809bb1c",
    .cfg_roots = 1,
    .prepare_note =
        "Uses your verified Metal Warriors (USA) ROM with the local snesrecomp "
        "SDK to regenerate src/gen, then runs cmake --build and restarts into "
        "the new binary. You must legally own this ROM.",
    .prepare_note_windows =
        "Uses your verified Metal Warriors (USA) ROM with the local snesrecomp "
        "SDK to regenerate src/gen, then quits and rebuilds via a helper so "
        "the running .exe is not locked. You must legally own this ROM.",
    .prepare_note_no_cmake =
        "Uses your verified Metal Warriors (USA) ROM with the local snesrecomp "
        "SDK to regenerate src/gen. CMake/build dir not found — rebuild "
        "manually: cmake --build build && relaunch.",
};

void mw_codegen_setup_apply(RecompLauncherCGameInfo* gi) {
    snesrecomp_codegen_host_apply(gi, &kMwCodegenConfig);
}

void mw_codegen_relaunch_or_exit(const char* rom_path) {
    snesrecomp_codegen_host_relaunch_or_exit(rom_path);
}
