cmake_minimum_required(VERSION 3.16)
project(MetalWarriorsSNESRecomp C CXX)

# Release pin for lobby matching (SNESRECOMP_BUILD_VERSION / SNES_GAME_VERSION).
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" _MW_VER_RAW)
string(STRIP "${_MW_VER_RAW}" MW_RELEASE_VERSION)
set(SNESRECOMP_BUILD_VERSION "" CACHE STRING
    "Lobby release pin (empty = Release uses VERSION file, else dev)")
if(SNESRECOMP_BUILD_VERSION STREQUAL "")
    get_property(_mw_is_multi GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
    if(_mw_is_multi)
        set(SNESRECOMP_BUILD_VERSION "$<IF:$<CONFIG:Release>,${MW_RELEASE_VERSION},dev>")
    elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
        set(SNESRECOMP_BUILD_VERSION "${MW_RELEASE_VERSION}")
    else()
        set(SNESRECOMP_BUILD_VERSION "dev")
    endif()
endif()
message(STATUS "Metal Warriors lobby game_version: ${SNESRECOMP_BUILD_VERSION} (VERSION=${MW_RELEASE_VERSION})")

# Cross-platform (macOS/Linux) build for MetalWarriorsSNESRecomp.
#
# Prerequisites: a snesrecomp checkout at ./snesrecomp (symlink/clone of the
# sibling repo), the generated C in src/gen/ (run `bash tools/regen.sh` with a
# verified metalwarriors.sfc at the repo root), SDL2 (e.g. `apt install libsdl2-dev`).

set(CMAKE_C_STANDARD 11)

set(SNESRECOMP_ROOT "${CMAKE_SOURCE_DIR}/snesrecomp" CACHE PATH
    "Path to the snesrecomp checkout used by this game")
option(SNESRECOMP_NET_ICE
    "Enable ICE/libjuice for Metal Warriors internet netplay" OFF)
include(${SNESRECOMP_ROOT}/runner/runner.cmake)

find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)

# MinGW's SDL2 package includes SDL2main by default, but this game provides a
# plain main() and uses the CRT entry point. Linking the forwarding shim would
# instead require SDL_main().
if(WIN32)
    list(FILTER SDL2_LIBRARIES EXCLUDE REGEX "SDL2main")
endif()

# Generated recompiler output (produced by tools/regen.sh; not committed).
file(GLOB MW_GEN_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/gen/*.c)
if(NOT MW_GEN_SOURCES)
    message(FATAL_ERROR
        "src/gen/ is empty — run `bash tools/regen.sh` with a verified "
        "metalwarriors.sfc at the repo root before building.")
endif()

add_executable(MetalWarriorsSNESRecomp
    ${SNESRECOMP_RUNNER_SOURCES}
    # game-specific runtime
    src/config.c
    src/glsl_shader.c
    src/main.c
    src/post_mortem.c
    src/opengl.c
    src/gen_stubs.c
    src/mw_cpu_infra.c
    src/mw_rtl.c
    src/mw_spc_player.c

    # OpenGL function loader
    third_party/gl_core/gl_core_3_1.c
    # recompiled banks (generated)
    ${MW_GEN_SOURCES}
)

target_include_directories(MetalWarriorsSNESRecomp PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/recomp
    ${SNESRECOMP_RUNNER_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
)

# prod-vs-debug via the single SNESRECOMP_ENABLE_TRACE switch (runner.cmake,
# default OFF). Prod: no debug_server.c / TCP server / rings. Debug:
# -DSNESRECOMP_ENABLE_TRACE=ON. tools/build-linux.sh --config {prod,debug} sets it.
if(SNESRECOMP_ENABLE_TRACE)
    set(_MW_TRACE 1)
else()
    set(_MW_TRACE 0)
endif()
target_compile_definitions(MetalWarriorsSNESRecomp PRIVATE
    SMW_DISABLE_LM=1
    SYSTEM_VOLUME_MIXER_AVAILABLE=0   # Win32 WASAPI volume control excluded
    SNESRECOMP_TRACE=${_MW_TRACE}     # TCP debug server: 0 in prod, 1 in debug
    SNESRECOMP_REVERSE_DEBUG=0        # raw WRAM stores; matches non-debug generated C
    SNESRECOMP_BUILD_VERSION="${SNESRECOMP_BUILD_VERSION}"
    SNES_GAME_VERSION="${SNESRECOMP_BUILD_VERSION}"
)

# The recompiled C is machine-generated; quiet its warnings and tolerate the
# K&R-style implicit declarations clang rejects by default. Keep the
# implicit-decl flag C-only so the launcher C++ TUs stay clean.
if(NOT MSVC)
    target_compile_options(MetalWarriorsSNESRecomp PRIVATE -w)
    target_compile_options(MetalWarriorsSNESRecomp PRIVATE
        $<$<COMPILE_LANGUAGE:C>:-Wno-implicit-function-declaration>)
endif()

target_link_libraries(MetalWarriorsSNESRecomp PRIVATE
    ${SDL2_LIBRARIES}
    OpenGL::GL
    m                                 # libm: glsl_shader.c uses pow() (implicit on macOS)
)

if(WIN32)
    # Trace/debug-server sockets and post-mortem symbolization.
    target_link_libraries(MetalWarriorsSNESRecomp PRIVATE ws2_32 dbghelp)
endif()

# Shared Dear ImGui pre-boot launcher from snesrecomp's lib/recomp-ui submodule.
# Presentation only; lobby/netcode are supplied through callbacks in main.c.
snesrecomp_enable_launcher_ui(MetalWarriorsSNESRecomp)

# Delay-sync netplay + MotK lobby client (LAN by default; opt into ICE with
# -DSNESRECOMP_NET_ICE=ON when libjuice is available).
snesrecomp_enable_recomp_net(MetalWarriorsSNESRecomp)
