# MetalWarriorsSNESRecomp

Private playtest scaffold: static recompilation of *Metal Warriors* (SNES,
USA) into native C using the [snesrecomp](https://github.com/mstan/snesrecomp)
framework. The pre-boot launcher and netplay screens come from
[recomp-ui](https://github.com/mstan/recomp-ui), vendored as a **repo-root**
submodule (`./recomp-ui`). snesrecomp keeps only `lib/recomp-net`.

This repo is **not** a public release. The ROM is never redistributed; supply
your own legally dumped copy.

## Expected ROM

| Field | Value |
|-------|-------|
| Title | Metal Warriors (USA) |
| CRC32 | `0xf2ab92d4` |
| SHA-256 | `0d7f875877fe856066cfb39b4ecdbbe7d48393a75770720876c94419f809bb1c` |

Stage the ROM at the repo root as `metalwarriors.sfc` (or
`Metal Warriors (USA).sfc`). A 512-byte SMC copier header is stripped before
hashing.

## Source setup

The game needs a snesrecomp checkout (default `./snesrecomp`, clone or symlink)
and the repo-root `recomp-ui` submodule. Initialize both after cloning:

```bash
git submodule update --init --recursive
git -C snesrecomp submodule update --init --recursive   # lib/recomp-net
```

To build against a feature worktree instead, set `SNESRECOMP_ROOT` explicitly:

```bash
SNESRECOMP_ROOT=/path/to/snesrecomp-worktree \
  bash tools/regen.sh --no-tests

cmake -S . -B build \
  -DSNESRECOMP_ROOT=/path/to/snesrecomp-worktree \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
```

On PowerShell, the equivalent configuration is:

```powershell
$env:SNESRECOMP_ROOT = 'F:\path\to\snesrecomp-worktree'
bash tools/regen.sh --no-tests
cmake -S . -B build -DSNESRECOMP_ROOT="$env:SNESRECOMP_ROOT"
cmake --build build --parallel
```

The ROM must be staged before running `tools/regen.sh`. SDL2 and OpenGL
development libraries are also required. For the standard Linux build wrapper,
`bash tools/build-linux.sh --config debug` configures and builds the game after
generation.

## Run

Launch without arguments to open recomp-ui, or provide the verified ROM for a
direct offline boot:

```bash
./build/MetalWarriorsSNESRecomp
./build/MetalWarriorsSNESRecomp --no-launcher metalwarriors.sfc
```

The trace-enabled debug TCP server listens on port **4380**.

## LAN netplay

Both players must use matching builds and the same verified ROM.

1. Start the game on the host, select the ROM, and open **Netplay**.
2. Choose **Host Lobby**. For LAN, check **LAN/Direct IP**, pick the host
   address and port (default `7777`), then create. LAN rooms advertise only on
   the local registry; online rooms advertise only on the lobby server.
3. On the second instance, open **Netplay**, select the matching row, and
   choose **Join**. Both players remain in the lobby
   modal and can see its occupied slots.
4. Allow the selected UDP port through the host firewall. Once the guest is
   visible, choose **Start Lobby** on the host; only then do both instances
   launch the game together.

For two instances on one machine, use `127.0.0.1` as the host IP. Metal
Warriors netplay uses two player slots and locks the shared match configuration
for deterministic simulation.

### Headless LAN smoke test

`SNES_NET_*` environment variables bypass recomp-ui but exercise the same
runtime and netcode. The following PowerShell commands run two loopback peers;
start them in separate terminals with the same ROM and session ID.

Host terminal:

```powershell
$env:SNES_NETPLAY = '1'
$env:SNES_NET_SLOT = '0'
$env:SNES_NET_INPUT_PLAYER = '0'
$env:SNES_NET_SESSION_ID = '4242'
$env:SNES_NET_BIND = '0.0.0.0:7777'
$env:SNES_NET_PEER = '127.0.0.1:7778'
$env:SNES_NET_TRANSPORT = 'lan'
$env:SNES_NET_TEST_TICKS = '600'
.\build\MetalWarriorsSNESRecomp.exe --no-launcher metalwarriors.sfc
```

Guest terminal:

```powershell
$env:SNES_NETPLAY = '1'
$env:SNES_NET_SLOT = '1'
$env:SNES_NET_INPUT_PLAYER = '0'
$env:SNES_NET_SESSION_ID = '4242'
$env:SNES_NET_BIND = '0.0.0.0:7778'
$env:SNES_NET_PEER = '127.0.0.1:7777'
$env:SNES_NET_TRANSPORT = 'lan'
$env:SNES_NET_TEST_TICKS = '600'
.\build\MetalWarriorsSNESRecomp.exe --no-launcher metalwarriors.sfc
```

`SNES_NET_TEST_TICKS=N` is optional. During environment-driven netplay, it
exits cleanly after `N` admitted synchronized ticks and prints a pass marker,
making the pair suitable for bounded CI or local smoke tests. Omit it for a
normal interactive session.

## Layout

- `recomp/` - bank `.cfg` seeds and `funcs.h` (synced by regeneration)
- `src/` - game glue, runtime integration, configuration, and generated stubs
- `src/gen/` - recompiler output (local only; gitignored)
- `recomp-ui/` - Dear ImGui launcher submodule (game-owned pin)
- `snesrecomp/` - default framework checkout or symlink (includes
  `lib/recomp-net`); override with `SNESRECOMP_ROOT` when using a worktree
- `tools/regen.sh` and `tools/build-linux.sh` - regeneration and Linux build
- `docs/H2H_STAGE_PROPS.md` - H2H mover/platform identification & manipulation
  (coldump fields, meta whitelist, OAM sticky + BG1 brown)

## Status

LLE-first bring-up: `RunOneFrameOfGame` drives the CPU from the reset/NMI
vectors through `interp_bridge_run_until_quiescent` and
`interp_bridge_run_interrupt`. The recomp-ui launcher is enabled by default;
use `--no-launcher` for scripts or direct ROM boot. There is no MSU-1 support.
Netplay is opt-in and supports two-player LAN sessions through recomp-net.
