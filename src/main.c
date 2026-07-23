#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "debug_server.h"
#include <SDL.h>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#if SYSTEM_VOLUME_MIXER_AVAILABLE
#include "platform/win32/volume_control.h"
#endif
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#endif

#include "snes/ppu.h"
#include "snes/dma.h"

#include "types.h"
#include "mw_rtl.h"
#include "cpu_state.h"
#include "common_cpu_infra.h"
#include "framedump.h"
#include "config.h"
#include "util.h"
#include "mw_spc_player.h"

#include "snes/snes.h"
#ifdef __SWITCH__
#include "switch_impl.h"
#endif

#include "launcher.h"
#include "keybinds.h"
#include "host_report.h"
#include "widescreen.h"  // g_ws_active, g_ws_extra, kWsExtraMax, RtlWidescreenPresent
#include "snes/color_lut.h"  // opt-in present-time CRT color LUT (SNESRECOMP_SCREEN)
#if defined(RECOMP_LAUNCHER)
#include "recomp_launcher.h"
#include "launcher_profile.h"
#include "snes_lobby_client.h"
#endif
#include "snes_netplay.h"
#include "recomp_net/address.h"
#include "recomp_net/lan_lobby.h"

typedef struct GamepadInfo {
  uint32 modifiers;
  SDL_JoystickID joystick_id;
  uint8 index;
  uint8 axis_buttons;
  uint16 last_cmd[kGamepadBtn_Count];
  Sint16 last_axis_x, last_axis_y;
} GamepadInfo;


static void SDLCALL AudioCallback(void *userdata, Uint8 *stream, int len);
static void EnsureConfigIni(void);
#if defined(RECOMP_LAUNCHER)
static int FindLauncherAssetsDir(char *out, size_t cap);
static void LauncherSettingsFromConfig(RecompLauncherCSettings *s);
static void ConfigFromLauncherSettings(const RecompLauncherCSettings *s);
#endif
static void RenderNumber(uint8 *dst, size_t pitch, int n, uint8 big);
static void OpenOneGamepad(int i);

static void CaptureNetplaySyncBytes(uint8_t out[2]) {
  out[0] = g_ram[0x001A];
  out[1] = g_ram[0x001B];
}

static void ApplyNetplaySyncBytes(const uint8_t in[2]) {
  g_ram[0x001A] = in[0];
  g_ram[0x001B] = in[1];
}
static uint32 GetActiveControllers(void);
static void RefreshKeybindControllerBits(void);
static int ResolveNetplayInputPlayer(int local_slot);
static uint16_t CaptureLocalNetplayButtons(void);
static int NetplayBarrierAdmit(bool *running);
static void netplay_soft_exit(const char *origin);
static void HandleVolumeAdjustment(int volume_adjustment);
static void HandleGamepadAxisInput(GamepadInfo *gi, int axis, Sint16 value);
static int RemapSdlButton(int button);
static void HandleGamepadInput(GamepadInfo *gi, int button, bool pressed);
static void HandleInput(int keyCode, int keyMod, bool pressed);
static void HandleCommand(uint32 j, bool pressed);
void OpenGLRenderer_Create(struct RendererFuncs *funcs);

bool g_new_ppu = true;

// Widescreen master switch — storage lives per-game (declared extern in the
// runner's widescreen.h). g_ws_extra = extra columns per side (0 = off);
// g_ws_active = (g_ws_extra > 0). Set once from config in the setup path below.
bool g_ws_active = false;
int g_ws_extra = 0;

struct SpcPlayer *g_spc_player;

// Sized for the widescreen capacity (256 + 2*kPpuExtraLeftRight per row). With
// widescreen off (g_ws_extra == 0) the PPU only writes the leading 256 columns,
// so this is authentic-identical; the extra capacity is just unused tail.
static uint8_t g_my_pixels[kPpuBufWidth * 4 * 240];


enum {
  kDefaultFullscreen = 0,
  kMaxWindowScale = 10,
  kDefaultFreq = 44100,
  kDefaultChannels = 2,
  kDefaultSamples = 2048,
};

/* Release stamp baked in by make_release.ps1 via
 * /p:SnesRecompBuildVersion=<ver> (vcxproj turns the MSBuild property
 * into this define). Local/IDE builds report "dev"; the post-mortem
 * report's build.pe_timestamp still uniquely identifies those. */
#ifndef SNESRECOMP_BUILD_VERSION
#define SNESRECOMP_BUILD_VERSION "dev"
#endif

static const char kWindowTitle[] = "Metal Warriors (Recompiled)";
static uint32 g_win_flags = SDL_WINDOW_RESIZABLE;
static SDL_Window *g_window;

static uint8 g_paused, g_turbo, g_cursor = true;
static uint8 g_current_window_scale;
static uint32 g_input_state;
/* Gamepad-driven SNES controller bits, kept separate from g_input_state
 * (keyboard) so the per-frame keybinds.ini polling at the top of the
 * main loop doesn't clear bits the gamepad just set. OR'd into `inputs`
 * once per frame alongside g_input_state and axis_buttons. */
static uint32 g_pad_buttons;
static bool g_display_perf;
static int g_curr_fps;
static int g_ppu_render_flags = 0;
static int g_snes_width, g_snes_height;
static int g_sdl_audio_mixer_volume = SDL_MIX_MAXVOLUME;
static struct RendererFuncs g_renderer_funcs;

static GamepadInfo g_gamepad[2];
static SnesNetplayConfig g_netplay_cfg;
static int g_netplay_pending;
/* From lobby match_caps: -1 = unset (legacy force), else host ws_extra (0=off). */
static int g_netplay_caps_ws_extra = -1;
#if defined(RECOMP_LAUNCHER)
/* Set when this process started netplay from the lobby room — soft-exit
 * returns there instead of continuing offline or killing the process. */
static int g_netplay_from_lobby;
#endif

extern Snes *g_snes;

#if defined(RECOMP_LAUNCHER)
static char g_launcher_lobby_url[256];
static int g_launcher_hosting_lan;
static int g_launcher_joined_lan;
static RecompLauncherCNetplayLaunch g_launcher_lan_launch;
#define LAUNCHER_MAX_LOCAL_ADDRESSES 32
static RNetIpv4Address
    g_launcher_local_addresses[LAUNCHER_MAX_LOCAL_ADDRESSES];
static int g_launcher_local_address_count;
static char g_launcher_external_ip[RNET_IPV4_ADDRESS_TEXT_MAX];

static const char *LauncherLanLobbyPath(void) {
  return "netplay_lan_lobby.txt";
}

static int LauncherReadLanState(RNetLanLobby *state) {
  return rnet_lan_lobby_read(LauncherLanLobbyPath(), "Metal Warriors",
                             SNES_GAME_VERSION, state) == RNET_LAN_LOBBY_OK;
}

static int LauncherCreateLanState(const char *name, const char *endpoint,
                                  const char *password) {
  RNetLanLobby state;
  memset(&state, 0, sizeof(state));
  snprintf(state.name, sizeof(state.name), "%s",
           name && name[0] ? name : "LAN Lobby");
  snprintf(state.game, sizeof(state.game), "Metal Warriors");
  snprintf(state.game_version, sizeof(state.game_version), "%s",
           SNES_GAME_VERSION);
  snprintf(state.endpoint, sizeof(state.endpoint), "%s",
           endpoint && endpoint[0] ? endpoint : "127.0.0.1:7777");
  snprintf(state.host_name, sizeof(state.host_name), "%s",
           snes_lobby_display_name()[0] ? snes_lobby_display_name() : "Host");
  snprintf(state.password, sizeof(state.password), "%s", password ? password : "");
  state.host_slot = 0;
  if (rnet_lan_lobby_publish(LauncherLanLobbyPath(), &state) !=
      RNET_LAN_LOBBY_OK) return 0;
  g_launcher_hosting_lan = 1;
  g_launcher_joined_lan = 0;
  memset(&g_launcher_lan_launch, 0, sizeof(g_launcher_lan_launch));
  return 1;
}

static int LauncherFillLanLobbyRow(RecompLauncherCNetplayLobby *out) {
  RNetLanLobby state;
  if (!out || !LauncherReadLanState(&state)) return 0;
  memset(out, 0, sizeof(*out));
  snprintf(out->lobby_id, sizeof(out->lobby_id), "lan:%s", state.endpoint);
  snprintf(out->name, sizeof(out->name), "LAN - %s",
           state.name[0] ? state.name : "Lobby");
  snprintf(out->game_name, sizeof(out->game_name), "%s", state.game);
  snprintf(out->game_version, sizeof(out->game_version), "%s",
           state.game_version);
  out->player_count = state.joiner_name[0] ? 2 : 1;
  out->max_slots = 2;
  out->has_password = state.password[0] != '\0';
  return 1;
}

static void LauncherClearLanJoinerSeat(void) {
  g_launcher_joined_lan = 0;
  memset(&g_launcher_lan_launch, 0, sizeof(g_launcher_lan_launch));
}

/* Joiner watches the file registry: empty/missing guest seat means kicked. */
static void LauncherSyncLanJoinerSeat(void) {
  RNetLanLobby state;
  const char *name;
  if (!g_launcher_joined_lan) return;
  if (!LauncherReadLanState(&state)) {
    LauncherClearLanJoinerSeat();
    return;
  }
  name = snes_lobby_display_name();
  if (!state.joiner_name[0] ||
      (name && name[0] && strcmp(state.joiner_name, name) != 0))
    LauncherClearLanJoinerSeat();
}

static int LauncherUseLanMembers(RNetLanLobby *state) {
  RNetLanLobby local;
  if (!state) state = &local;
  LauncherSyncLanJoinerSeat();
  if (!LauncherReadLanState(state)) return 0;
  if (g_launcher_joined_lan) return 1;
  if (!g_launcher_hosting_lan) return 0;
  return state->joiner_name[0] || snes_lobby_member_count() < 2;
}

static SnesLobbyMatchCaps LauncherNetplayCaps(
    const RecompLauncherCSettings *settings) {
  SnesLobbyMatchCaps caps;
  memset(&caps, 0, sizeof(caps));
  caps.valid = 1;
  caps.widescreen = 1;
  caps.widescreen_hud = 1;
  caps.ignore_aspect = settings ? settings->ignore_aspect != 0 : 0;
  caps.input_delay = 2;
  caps.ws_extra = 71;
  return caps;
}

static const char *LauncherNpDefaultUrl(void *ctx) {
  (void)ctx;
  return g_launcher_lobby_url[0] ? g_launcher_lobby_url
                                 : snes_lobby_default_url();
}

static void LauncherNpSetUrl(void *ctx, const char *url) {
  (void)ctx;
  snprintf(g_launcher_lobby_url, sizeof(g_launcher_lobby_url), "%s",
           url && url[0] ? url : snes_lobby_default_url());
}

static int LauncherNpConnect(void *ctx) {
  (void)ctx;
  snes_lobby_set_game_identity("Metal Warriors", SNES_GAME_VERSION);
  return snes_lobby_connect(LauncherNpDefaultUrl(NULL));
}

static int LauncherNpConnected(void *ctx) {
  (void)ctx;
  return snes_lobby_connected();
}

static void LauncherNpPump(void *ctx) {
  (void)ctx;
  snes_lobby_pump();
  LauncherSyncLanJoinerSeat();
}

static void LauncherNpSetPlayerName(void *ctx, const char *name) {
  (void)ctx;
  snes_lobby_set_display_name(name && name[0] ? name : "Player");
}

static const char *LauncherNpPlayerName(void *ctx) {
  (void)ctx;
  return snes_lobby_display_name();
}

static void LauncherNpRequestList(void *ctx) {
  (void)ctx;
  snes_lobby_request_list();
}

static int LauncherNpListCount(void *ctx) {
  RecompLauncherCNetplayLobby lan;
  (void)ctx;
  return snes_lobby_list_count() + (LauncherFillLanLobbyRow(&lan) ? 1 : 0);
}

static int LauncherNpListGet(void *ctx, int index,
                             RecompLauncherCNetplayLobby *out) {
  SnesLobbyRow row;
  int remote_count;
  (void)ctx;
  if (!out || index < 0) return 0;
  remote_count = snes_lobby_list_count();
  if (index >= remote_count)
    return index == remote_count ? LauncherFillLanLobbyRow(out) : 0;
  if (!snes_lobby_list_get(index, &row)) return 0;
  memset(out, 0, sizeof(*out));
  snprintf(out->lobby_id, sizeof(out->lobby_id), "%s", row.lobby_id);
  snprintf(out->name, sizeof(out->name), "%s", row.name);
  snprintf(out->game_name, sizeof(out->game_name), "%s", row.game_name);
  snprintf(out->game_version, sizeof(out->game_version), "%s",
           row.game_version);
  out->player_count = row.player_count;
  out->max_slots = row.max_slots;
  out->has_password = row.has_password;
  return 1;
}

static int LauncherRefreshLocalAddresses(void) {
  int count = rnet_ipv4_enumerate(
      g_launcher_local_addresses, LAUNCHER_MAX_LOCAL_ADDRESSES);
  if (count < 0) count = 0;
  if (count > LAUNCHER_MAX_LOCAL_ADDRESSES)
    count = LAUNCHER_MAX_LOCAL_ADDRESSES;
  g_launcher_local_address_count = count;
  return count;
}

static int LauncherNpLocalAddressGet(
    void *ctx, int index, RecompLauncherCNetplayLocalAddress *out) {
  (void)ctx;
  if (!out || index < 0) return 0;
  if (index == 0) LauncherRefreshLocalAddresses();
  if (index >= g_launcher_local_address_count) return 0;
  memset(out, 0, sizeof(*out));
  snprintf(out->address, sizeof(out->address), "%s",
           g_launcher_local_addresses[index].address);
  snprintf(out->label, sizeof(out->label), "%s",
           g_launcher_local_addresses[index].interface_label);
  return 1;
}

static int LauncherNpLocalIp(void *ctx, char *out, size_t out_len) {
  RecompLauncherCNetplayLocalAddress address;
  if (!out || !out_len || !LauncherNpLocalAddressGet(ctx, 0, &address))
    return 0;
  snprintf(out, out_len, "%s", address.address);
  return out[0] != '\0';
}

static int LauncherNpExternalIp(void *ctx, char *out, size_t out_len) {
  RNetExternalIpv4Config config;
  int rc;
  (void)ctx;
  if (!out || !out_len) return 0;
  if (!g_launcher_external_ip[0]) {
    rnet_external_ipv4_config_init(&config);
    /* This callback currently runs on the launcher render thread. Keep the
     * STUN wait short; a later recomp-ui API can make discovery asynchronous. */
    config.timeout_ms = 900;
    rc = rnet_external_ipv4_discover(&config, g_launcher_external_ip,
                                     sizeof(g_launcher_external_ip));
    if (rc != RNET_EXTERNAL_IPV4_OK) {
      fprintf(stderr, "metalwarriors: external IPv4 discovery failed (%d)\n",
              rc);
      snprintf(out, out_len, "Unavailable");
      return 0;
    }
  }
  snprintf(out, out_len, "%s", g_launcher_external_ip);
  return out[0] != '\0';
}

static int LauncherNpCreate(void *ctx, const char *lobby_name,
                            char *host_endpoint, const char *password,
                            const RecompLauncherCSettings *settings,
                            int lan_only) {
  SnesLobbyMatchCaps caps = LauncherNetplayCaps(settings);
  (void)ctx;
  /* host_endpoint is in/out (capacity >= 64). recomp-ui already applied the
   * universal UDP port policy (LAN exact / online auto-pick) before create. */
  if (!host_endpoint) return -1;
  if (!host_endpoint[0])
    snprintf(host_endpoint, 64, lan_only ? "127.0.0.1:7777" : "0.0.0.0:7777");
  /* Advertise on exactly one channel. LAN and server lobbies use different
   * join/member/start paths; dual-publish produced duplicate list rows. */
  if (lan_only) {
    if (!LauncherCreateLanState(lobby_name, host_endpoint, password)) return -1;
    return 0;
  }
  /* Online: clear any stale same-machine LAN row so it cannot appear beside
   * the server lobby in the merged list. */
  (void)rnet_lan_lobby_leave(LauncherLanLobbyPath(), 1);
  g_launcher_hosting_lan = 0;
  g_launcher_joined_lan = 0;
  memset(&g_launcher_lan_launch, 0, sizeof(g_launcher_lan_launch));
  return snes_lobby_create(
      lobby_name && lobby_name[0] ? lobby_name : "Netplay Lobby",
      "Metal Warriors", SNES_GAME_VERSION, password ? password : "",
      host_endpoint, &caps);
}

static int LauncherNpJoin(void *ctx, const char *lobby_id,
                          const char *password, char *guest_bind) {
  RNetLanLobby state;
  const char *name;
  (void)ctx;
  memset(&g_launcher_lan_launch, 0, sizeof(g_launcher_lan_launch));
  if (lobby_id && strncmp(lobby_id, "lan:", 4) == 0) {
    (void)guest_bind;
    name = snes_lobby_display_name();
    if (rnet_lan_lobby_join(LauncherLanLobbyPath(), "Metal Warriors",
                            SNES_GAME_VERSION, password ? password : "",
                            name && name[0] ? name : "Player", &state) !=
        RNET_LAN_LOBBY_OK) return -1;
    g_launcher_hosting_lan = 0;
    g_launcher_joined_lan = 1;
    return 0;
  }
  g_launcher_hosting_lan = 0;
  g_launcher_joined_lan = 0;
  return snes_lobby_join(lobby_id, password ? password : "", guest_bind);
}

static int LauncherNpLeave(void *ctx) {
  int rc;
  (void)ctx;
  if (g_launcher_hosting_lan)
    (void)rnet_lan_lobby_leave(LauncherLanLobbyPath(), 1);
  else if (g_launcher_joined_lan)
    (void)rnet_lan_lobby_leave(LauncherLanLobbyPath(), 0);
  g_launcher_hosting_lan = 0;
  g_launcher_joined_lan = 0;
  memset(&g_launcher_lan_launch, 0, sizeof(g_launcher_lan_launch));
  rc = snes_lobby_leave();
  return rc;
}

static int LauncherNpInLobby(void *ctx) {
  (void)ctx;
  LauncherSyncLanJoinerSeat();
  return g_launcher_hosting_lan || g_launcher_joined_lan ||
         snes_lobby_in_lobby();
}

static int LauncherNpIsHost(void *ctx) {
  (void)ctx;
  if (g_launcher_hosting_lan || g_launcher_joined_lan)
    return g_launcher_hosting_lan ? 1 : 0;
  return snes_lobby_is_host();
}

static int LauncherNpMemberCount(void *ctx) {
  (void)ctx;
  RNetLanLobby state;
  return LauncherUseLanMembers(&state) ? 2 : snes_lobby_member_count();
}

static int LauncherNpMemberGet(void *ctx, int index,
                               RecompLauncherCNetplayMember *out) {
  SnesLobbyMember member;
  RNetLanLobby state;
  (void)ctx;
  if (!out) return 0;
  memset(out, 0, sizeof(*out));
  if (LauncherUseLanMembers(&state)) {
    if (index < 0 || index > 1) return 0;
    out->slot = index == 0 ? state.host_slot : 1 - state.host_slot;
    out->ready = index == 0 || state.joiner_name[0] != '\0';
    out->is_host = index == 0;
    snprintf(out->display_name, sizeof(out->display_name), "%s",
             index == 0 ? state.host_name : state.joiner_name);
    return 1;
  }
  if (!snes_lobby_member_get(index, &member)) return 0;
  out->slot = member.slot;
  out->ready = member.ready;
  {
    const char *host_id = snes_lobby_host_player_id();
    out->is_host = host_id && host_id[0] &&
                   strcmp(member.player_id, host_id) == 0;
  }
  snprintf(out->display_name, sizeof(out->display_name), "%s",
           member.display_name);
  return 1;
}

static int LauncherNpMoveMember(void *ctx, int from_slot, int to_slot) {
  RNetLanLobby state;
  (void)ctx;
  if (g_launcher_hosting_lan && from_slot >= 0 && from_slot <= 1 &&
      to_slot >= 0 && to_slot <= 1 && from_slot != to_slot &&
      LauncherReadLanState(&state))
    return rnet_lan_lobby_set_host_slot(LauncherLanLobbyPath(),
                                        1 - state.host_slot);
  if (g_launcher_joined_lan) return -1;
  return snes_lobby_move(from_slot, to_slot);
}

static int LauncherNpKickMember(void *ctx, int slot) {
  RNetLanLobby state;
  int guest_slot;
  (void)ctx;
  if (g_launcher_hosting_lan) {
    if (slot < 0 || slot > 1 || !LauncherReadLanState(&state)) return -1;
    guest_slot = 1 - state.host_slot;
    if (slot != guest_slot || !state.joiner_name[0]) return -1;
    return rnet_lan_lobby_kick(LauncherLanLobbyPath()) == RNET_LAN_LOBBY_OK
               ? 0
               : -1;
  }
  if (g_launcher_joined_lan) return -1;
  return snes_lobby_kick(slot);
}

static int LauncherNpLocalReady(void *ctx) {
  (void)ctx;
  if (g_launcher_hosting_lan || g_launcher_joined_lan) return 1;
  return snes_lobby_local_ready();
}

static int LauncherNpAllReady(void *ctx) {
  (void)ctx;
  RNetLanLobby state;
  if (LauncherUseLanMembers(&state)) return state.joiner_name[0] != '\0';
  return snes_lobby_all_ready();
}

static int LauncherNpSetReady(void *ctx, int ready) {
  (void)ctx;
  if (g_launcher_hosting_lan || g_launcher_joined_lan) return 0;
  return snes_lobby_set_ready(ready);
}

static void LauncherArmLanLaunchFromState(const RNetLanLobby *state) {
  const char *colon;
  const char *port;
  if (!state) return;
  memset(&g_launcher_lan_launch, 0, sizeof(g_launcher_lan_launch));
  g_launcher_lan_launch.enabled = 1;
  g_launcher_lan_launch.local_slot =
      g_launcher_hosting_lan ? state->host_slot : 1 - state->host_slot;
  g_launcher_lan_launch.input_player = 0;
  g_launcher_lan_launch.session_id = 1;
  g_launcher_lan_launch.input_delay = 2;
  if (g_launcher_hosting_lan) {
    colon = strrchr(state->endpoint, ':');
    port = colon ? colon + 1 : "7777";
    snprintf(g_launcher_lan_launch.bind_hostport,
             sizeof(g_launcher_lan_launch.bind_hostport), "0.0.0.0:%s", port);
  } else {
    snprintf(g_launcher_lan_launch.bind_hostport,
             sizeof(g_launcher_lan_launch.bind_hostport), "0.0.0.0:0");
    snprintf(g_launcher_lan_launch.peer_hostport,
             sizeof(g_launcher_lan_launch.peer_hostport), "%s", state->endpoint);
  }
}

static int LauncherNpRequestStart(void *ctx,
                                  const RecompLauncherCSettings *settings) {
  SnesLobbyMatchCaps caps = LauncherNetplayCaps(settings);
  RNetLanLobby state;
  (void)ctx;
  /* LAN host start is file-backed — never fall through to the WS client. */
  if (g_launcher_hosting_lan) {
    if (!LauncherReadLanState(&state) || !state.joiner_name[0]) return -1;
    if (rnet_lan_lobby_set_started(LauncherLanLobbyPath(), 1) !=
        RNET_LAN_LOBBY_OK)
      return -1;
    state.started = 1;
    LauncherArmLanLaunchFromState(&state);
    return 0;
  }
  return snes_lobby_request_start(&caps);
}

static int LauncherNpLaunchPending(void *ctx) {
  RNetLanLobby state;
  (void)ctx;
  if ((g_launcher_hosting_lan || g_launcher_joined_lan) &&
      !g_launcher_lan_launch.enabled && LauncherReadLanState(&state) &&
      state.started)
    LauncherArmLanLaunchFromState(&state);
  return g_launcher_lan_launch.enabled || snes_lobby_launch_pending();
}

static void LauncherNpClearLaunchPending(void *ctx) {
  (void)ctx;
  memset(&g_launcher_lan_launch, 0, sizeof(g_launcher_lan_launch));
  snes_lobby_clear_launch_pending();
}

static const char *LauncherNpLastError(void *ctx) {
  const SnesLobbyJoinInfo *join;
  (void)ctx;
  join = snes_lobby_join_info();
  return (join && join->last_error[0]) ? join->last_error : "";
}

static void LauncherNpClearLastError(void *ctx) {
  (void)ctx;
  snes_lobby_clear_last_error();
}

static int LauncherNpFillLaunch(void *ctx,
                                RecompLauncherCNetplayLaunch *out) {
  SnesLobbyJoinInfo join;
  const SnesLobbyMatchCaps *caps;
  (void)ctx;
  if (!out) return 0;
  if (g_launcher_lan_launch.enabled) {
    *out = g_launcher_lan_launch;
    return 1;
  }
  /* Online: snes_lobby_try_fill_launch gates on op:launch + usable endpoints. */
  if (!snes_lobby_try_fill_launch(&join)) return 0;
  caps = snes_lobby_match_caps();
  memset(out, 0, sizeof(*out));
  out->enabled = 1;
  out->local_slot = join.local_slot;
  out->input_player = 0;
  out->session_id = join.session_id;
  out->input_delay = caps && caps->valid ? caps->input_delay : 2;
  snprintf(out->bind_hostport, sizeof(out->bind_hostport), "%s",
           join.bind_hostport);
  snprintf(out->peer_hostport, sizeof(out->peer_hostport), "%s",
           join.peer_hostport);
  return 1;
}

static RecompLauncherCNetplayCallbacks g_launcher_netplay_callbacks = {
  NULL,
  LauncherNpDefaultUrl,
  LauncherNpSetUrl,
  LauncherNpConnect,
  LauncherNpConnected,
  LauncherNpPump,
  LauncherNpSetPlayerName,
  LauncherNpPlayerName,
  LauncherNpRequestList,
  LauncherNpListCount,
  LauncherNpListGet,
  LauncherNpLocalIp,
  LauncherNpExternalIp,
  LauncherNpCreate,
  LauncherNpJoin,
  LauncherNpLeave,
  LauncherNpInLobby,
  LauncherNpIsHost,
  LauncherNpMemberCount,
  LauncherNpMemberGet,
  LauncherNpMoveMember,
  LauncherNpLocalReady,
  LauncherNpAllReady,
  LauncherNpSetReady,
  LauncherNpRequestStart,
  LauncherNpLaunchPending,
  LauncherNpClearLaunchPending,
  LauncherNpFillLaunch,
  LauncherNpLocalAddressGet,
  LauncherNpKickMember,
  LauncherNpLastError,
  LauncherNpClearLastError,
};
#endif

// --- Scripted input ---
typedef struct {
  uint32 mask;      // button bits to hold
  int hold_frames;  // frames to hold mask (0 = release)
  int wait_frames;  // frames to wait after hold ends before next entry
} ScriptEntry;

static ScriptEntry *g_script_entries;
static int g_script_count;
static int g_script_index;    // current entry
static int g_script_phase;    // 0=holding, 1=waiting
static int g_script_counter;  // frames left in current phase

static uint32 ParseButtonMask(const char *name) {
  if (strcmp(name, "start")  == 0) return 0x0008;
  if (strcmp(name, "select") == 0) return 0x0004;
  if (strcmp(name, "up")     == 0) return 0x0010;
  if (strcmp(name, "down")   == 0) return 0x0020;
  if (strcmp(name, "left")   == 0) return 0x0040;
  if (strcmp(name, "right")  == 0) return 0x0080;
  if (strcmp(name, "a")      == 0) return 0x0100;
  if (strcmp(name, "b")      == 0) return 0x0001;
  if (strcmp(name, "x")      == 0) return 0x0200;
  if (strcmp(name, "y")      == 0) return 0x0002;
  if (strcmp(name, "l")      == 0) return 0x0400;
  if (strcmp(name, "r")      == 0) return 0x0800;
  fprintf(stderr, "script: unknown button '%s'\n", name);
  return 0;
}

static void LoadScript(const char *path) {
  FILE *f = fopen(path, "r");
  if (!f) { fprintf(stderr, "script: cannot open '%s'\n", path); return; }

  // Two-pass: count then fill
  int cap = 64;
  g_script_entries = (ScriptEntry *)malloc(cap * sizeof(ScriptEntry));
  g_script_count = 0;

  char line[256];
  // pending wait accumulates between press commands
  int pending_wait = 0;
  while (fgets(line, sizeof(line), f)) {
    // strip comment and newline
    char *c = strchr(line, '#'); if (c) *c = 0;
    char cmd[64], arg1[64];
    int n = 0;
    if (sscanf(line, "%63s %63s %d", cmd, arg1, &n) < 1) continue;
    if (strcmp(cmd, "wait") == 0) {
      int frames = (sscanf(line, "%*s %d", &n) == 1) ? n : 0;
      pending_wait += frames;
    } else if (strcmp(cmd, "loadstate") == 0) {
      // loadstate N — load savestate slot N (0-indexed, F1=0)
      int slot = 0;
      sscanf(line, "%*s %d", &slot);
      if (g_script_count >= cap) {
        cap *= 2;
        g_script_entries = (ScriptEntry *)realloc(g_script_entries, cap * sizeof(ScriptEntry));
      }
      ScriptEntry *e = &g_script_entries[g_script_count++];
      e->mask = 0x80000000 | (slot & 0xF);  // special flag: high bit = loadstate
      e->hold_frames = 1;
      e->wait_frames = pending_wait;
      pending_wait = 0;
    } else if (strcmp(cmd, "press") == 0) {
      int hold = (sscanf(line, "%*s %*s %d", &n) == 1) ? n : 1;
      if (g_script_count >= cap) {
        cap *= 2;
        g_script_entries = (ScriptEntry *)realloc(g_script_entries, cap * sizeof(ScriptEntry));
      }
      ScriptEntry *e = &g_script_entries[g_script_count++];
      e->mask = ParseButtonMask(arg1);
      e->hold_frames = hold;
      e->wait_frames = pending_wait;
      pending_wait = 0;
    }
  }
  fclose(f);

  if (g_script_count > 0) {
    g_script_index = 0;
    g_script_phase = 1; // start with the wait_frames of first entry
    g_script_counter = g_script_entries[0].wait_frames;
    fprintf(stderr, "script: loaded %d entries from '%s'\n", g_script_count, path);
  }
}

static int g_script_finished;

static uint32 TickScript(void) {
  if (!g_script_entries || g_script_index >= g_script_count) {
    if (g_script_entries && g_script_index >= g_script_count)
      g_script_finished = 1;
    return 0;
  }

  ScriptEntry *e = &g_script_entries[g_script_index];

  if (g_script_phase == 1) {
    // waiting
    if (g_script_counter > 0) { g_script_counter--; return 0; }
    // done waiting — start hold
    g_script_phase = 0;
    g_script_counter = e->hold_frames;
  }

  if (g_script_phase == 0) {
    if (g_script_counter > 0) {
      g_script_counter--;
      if (e->mask & 0x80000000) {
        // loadstate command
        int slot = (int)(e->mask & 0xF);
        if (!snes_netplay_request_load(slot))
          RtlSaveLoad(kSaveLoad_Load, slot);
        return 0;
      }
      return e->mask;
    }
    // hold done — advance
    g_script_index++;
    if (g_script_index < g_script_count) {
      e = &g_script_entries[g_script_index];
      g_script_phase = 1;
      g_script_counter = e->wait_frames;
    } else {
      g_script_finished = 1;
    }
    return 0;
  }
  return 0;
}

void NORETURN Die(const char *error) {
  /* Record the message before exiting: the atexit post-mortem dump
   * includes it and preserves a timestamped crash copy (see
   * host_report_has_fatal in post_mortem.c). */
  host_report_fatal(error);
  SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, kWindowTitle, error, NULL);
  fprintf(stderr, "Error: %s\n", error);
  exit(1);
}

static GamepadInfo *GetGamepadInfo(SDL_JoystickID id) {
  return (g_gamepad[0].joystick_id == id) ? &g_gamepad[0] :
    (g_gamepad[1].joystick_id == id) ? &g_gamepad[1] : NULL;
}

void ChangeWindowScale(int scale_step) {
  if ((SDL_GetWindowFlags(g_window) & (SDL_WINDOW_FULLSCREEN_DESKTOP | SDL_WINDOW_FULLSCREEN | SDL_WINDOW_MINIMIZED | SDL_WINDOW_MAXIMIZED)) != 0)
    return;
  int screen = SDL_GetWindowDisplayIndex(g_window);
  if (screen < 0) screen = 0;
  int max_scale = kMaxWindowScale;
  SDL_Rect bounds;
  int bt = -1, bl, bb, br;
  // note this takes into effect Windows display scaling, i.e., resolution is divided by scale factor
  if (SDL_GetDisplayUsableBounds(screen, &bounds) == 0) {
    // this call may take a while before it is reported by Windows (or not at all in my testing)
    if (SDL_GetWindowBordersSize(g_window, &bt, &bl, &bb, &br) != 0) {
      // guess based on Windows 10/11 defaults
      bl = br = bb = 1;
      bt = 31;
    }
    // Allow a scale level slightly above the max that fits on screen
    int mw = (bounds.w - bl - br + g_snes_width / 4) / g_snes_width;
    int mh = (bounds.h - bt - bb + g_snes_height / 4) / g_snes_height;
    max_scale = IntMin(mw, mh);
  }
  int new_scale = IntMax(IntMin(g_current_window_scale + scale_step, max_scale), 1);
  g_current_window_scale = new_scale;
  int w = new_scale * g_snes_width;
  int h = new_scale * g_snes_height;

  //SDL_RenderSetLogicalSize(g_renderer, w, h);
  SDL_SetWindowSize(g_window, w, h);
  if (bt >= 0) {
    // Center the window on top of the mouse
    int mx, my;
    SDL_GetGlobalMouseState(&mx, &my);
    int wx = IntMax(IntMin(mx - w / 2, bounds.x + bounds.w - bl - br - w), bounds.x + bl);
    int wy = IntMax(IntMin(my - h / 2, bounds.y + bounds.h - bt - bb - h), bounds.y + bt);
    SDL_SetWindowPosition(g_window, wx, wy);
  } else {
    SDL_SetWindowPosition(g_window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
  }
}

#define RESIZE_BORDER 20
static SDL_HitTestResult HitTestCallback(SDL_Window *win, const SDL_Point *pt, void *data) {
  uint32 flags = SDL_GetWindowFlags(win);
  if ((flags & SDL_WINDOW_FULLSCREEN_DESKTOP) != 0 || (flags & SDL_WINDOW_FULLSCREEN) != 0)
    return SDL_HITTEST_NORMAL;

  if ((SDL_GetModState() & KMOD_CTRL) != 0)
    return SDL_HITTEST_DRAGGABLE;

  int w, h;
  SDL_GetWindowSize(win, &w, &h);

  if (pt->y < RESIZE_BORDER) {
    return (pt->x < RESIZE_BORDER) ? SDL_HITTEST_RESIZE_TOPLEFT :
      (pt->x >= w - RESIZE_BORDER) ? SDL_HITTEST_RESIZE_TOPRIGHT : SDL_HITTEST_RESIZE_TOP;
  } else if (pt->y >= h - RESIZE_BORDER) {
    return (pt->x < RESIZE_BORDER) ? SDL_HITTEST_RESIZE_BOTTOMLEFT :
      (pt->x >= w - RESIZE_BORDER) ? SDL_HITTEST_RESIZE_BOTTOMRIGHT : SDL_HITTEST_RESIZE_BOTTOM;
  } else {
    if (pt->x < RESIZE_BORDER) {
      return SDL_HITTEST_RESIZE_LEFT;
    } else if (pt->x >= w - RESIZE_BORDER) {
      return SDL_HITTEST_RESIZE_RIGHT;
    }
  }
  return SDL_HITTEST_NORMAL;
}

/* Legacy half-crop fallback when SNESRECOMP_MW_H2H_FULL_FRAME=0. */
static void mw_present_h2h_local_view(uint8 *pixel_buffer, size_t pitch,
                                      int width, int height, int local_slot) {
  if (!pixel_buffer || width <= 0 || height < 2)
    return;
  if (local_slot != 0 && local_slot != 1)
    return;

  const int half = height / 2;
  enum { kH2hSeamPad = 8 };
  int pad = kH2hSeamPad;
  if (pad > half)
    pad = half;
  const int src_h = half + pad;
  const int src_y0 = (local_slot == 0) ? 0 : (half - pad);
  if (src_y0 < 0 || src_y0 + src_h > height)
    return;

  const size_t row_bytes = (size_t)width * 4u;
  enum { kH2hMaxW = 256 + 2 * kWsExtraMax, kH2hMaxSrcH = 128 };
  if (width > kH2hMaxW || src_h > kH2hMaxSrcH)
    return;

  static uint8_t s_h2h_tmp[kH2hMaxW * kH2hMaxSrcH * 4];
  for (int y = 0; y < src_h; y++) {
    memcpy(s_h2h_tmp + (size_t)y * row_bytes,
           pixel_buffer + (size_t)(src_y0 + y) * pitch, row_bytes);
  }
  for (int y = 0; y < height; y++) {
    const int sy = y * src_h / height;
    memcpy(pixel_buffer + (size_t)y * pitch,
           s_h2h_tmp + (size_t)sy * row_bytes, row_bytes);
  }
}

void RtlDrawPpuFrame(uint8 *pixel_buffer, size_t pitch, uint32 render_flags) {
  // Widescreen presentation (opt-in). With g_ws_active false, g_snes_width is
  // 256 and this reduces to the authentic 256-wide copy — byte-identical to the
  // faithful build. Re-applied every frame because ppu_reset (soft reset /
  // load-state) zeroes the PPU's margin fields.
  const int h2h_local_full =
      snes_netplay_active() && MwIsDualViewport() && MwH2hFullFrameLocalArmed();
  static int h2h_local = -1;
  if (h2h_local < 0) {
    const char *e = getenv("SNESRECOMP_MW_H2H_LOCAL_VIEW");
    h2h_local = (e && e[0] == '0') ? 0 : 1;
  }
  const int use_local_full = h2h_local_full && h2h_local;

  if (g_ws_active) {
    memset(g_my_pixels, 0, (size_t)g_snes_width * 4 * (size_t)g_snes_height);
    MwConfigureWidescreen();
  }
  /* Netplay H2H: full 224-line local camera (real FOV). Fallback: split draw
   * then half-crop when FULL_FRAME=0. */
  if (use_local_full)
    MwDrawPpuFrameLocalFull(snes_netplay_local_slot());
  else
    g_rtl_game_info->draw_ppu_frame();
  RtlWidescreenPresent(pixel_buffer, pitch, g_my_pixels, g_snes_width, g_snes_height);
  /* Full-frame H2H: solid top bar + opponent direction (replaces dual-seam HUD). */
  if (use_local_full)
    MwPresentH2hTopBar(pixel_buffer, pitch, g_snes_width, g_snes_height,
                       snes_netplay_local_slot());
  /* Temporary offline verification: MW_CAP=<dir> writes periodic widescreen
   * PPMs of the present buffer. Zero cost when the env var is unset. */
  {
    static const char *cap_dir;
    static int cap_checked;
    if (!cap_checked) {
      cap_checked = 1;
      cap_dir = getenv("MW_CAP");
    }
    static int cap_every = 120;
    static int cap_from = 2100;
    if (cap_dir && cap_checked == 1) {
      cap_checked = 2;
      const char *ev = getenv("MW_CAP_EVERY");
      const char *fr = getenv("MW_CAP_FROM");
      if (ev)
        cap_every = atoi(ev) > 0 ? atoi(ev) : 120;
      if (fr)
        cap_from = atoi(fr);
    }
    extern int snes_frame_counter;
    if (cap_dir && snes_frame_counter >= cap_from &&
        (snes_frame_counter % cap_every) == 0) {
      char path[1024];
      snprintf(path, sizeof(path), "%s/ws_%06d.ppm", cap_dir,
               snes_frame_counter);
      FILE *f = fopen(path, "wb");
      if (f) {
        fprintf(f, "P6\n%d %d\n255\n", g_snes_width, g_snes_height);
        for (int y = 0; y < g_snes_height; y++) {
          const uint8 *row = pixel_buffer + (size_t)y * pitch;
          for (int x = 0; x < g_snes_width; x++) {
            fputc(row[x * 4 + 2], f);
            fputc(row[x * 4 + 1], f);
            fputc(row[x * 4 + 0], f);
          }
        }
        fclose(f);
      }
    }
  }
  // Present-time color grading (opt-in, SNESRECOMP_SCREEN=crt|trinitron; default
  // raw = no-op). Applied to the present copy only, row by row so the texture
  // pitch is honored. The raw g_my_pixels (frame-hashed oracle) is never touched.
  if (snes_color_lut_active()) {
    for (int y = 0; y < g_snes_height; y++) {
      uint32_t *row = (uint32_t *)(pixel_buffer + (size_t)y * pitch);
      snes_color_lut_map(row, row, (size_t)g_snes_width);
    }
  }

  /* Legacy half-crop when full-frame local present is disabled. */
  if (!use_local_full && snes_netplay_active() && MwIsDualViewport() &&
      h2h_local) {
    mw_present_h2h_local_view(pixel_buffer, pitch, g_snes_width, g_snes_height,
                              snes_netplay_local_slot());
  }
}

static void DrawPpuFrameWithPerf(void) {
  int render_scale = PpuGetCurrentRenderScale(g_ppu, g_ppu_render_flags);
  uint8 *pixel_buffer = 0;
  int pitch = 0;

  g_renderer_funcs.BeginDraw(g_snes_width * render_scale,
                             g_snes_height * render_scale,
                             &pixel_buffer, &pitch);
  if (g_display_perf || g_config.display_perf_title) {
    static float history[64], average;
    static int history_pos;
    uint64 before = SDL_GetPerformanceCounter();
    RtlDrawPpuFrame(pixel_buffer, pitch, g_ppu_render_flags);
    uint64 after = SDL_GetPerformanceCounter();
    float v = (double)SDL_GetPerformanceFrequency() / (after - before);
    average += v - history[history_pos];
    history[history_pos] = v;
    history_pos = (history_pos + 1) & 63;
    g_curr_fps = average * (1.0f / 64);
  } else {
    RtlDrawPpuFrame(pixel_buffer, pitch, g_ppu_render_flags);
  }
  if (g_display_perf)
    RenderNumber(pixel_buffer + pitch * render_scale, pitch, g_curr_fps, render_scale == 4);

  g_renderer_funcs.EndDraw();
}

static SDL_mutex *g_audio_mutex;
static uint8 *g_audiobuffer, *g_audiobuffer_cur, *g_audiobuffer_end;
static int g_frames_per_block;
static uint8 g_audio_channels;
static SDL_AudioDeviceID g_audio_device;

void RtlApuLock(void) {
  SDL_LockMutex(g_audio_mutex);
}

void RtlApuUnlock(void) {
  SDL_UnlockMutex(g_audio_mutex);
}

static void SDLCALL AudioCallback(void *userdata, Uint8 *stream, int len) {
  /* Boot-stage marker: proves the audio thread reached the mixer at
   * least once (the "crashed before the first sound" class of report). */
  static SDL_atomic_t first_cb;
  if (SDL_AtomicCAS(&first_cb, 0, 1))
    host_report_breadcrumb("first audio callback (len=%d)", len);
  if (SDL_LockMutex(g_audio_mutex)) Die("Mutex lock failed!");
  while (len != 0) {
    if (g_audiobuffer_end - g_audiobuffer_cur == 0) {
      RtlRenderAudio((int16 *)g_audiobuffer, g_frames_per_block, g_audio_channels);
      g_audiobuffer_cur = g_audiobuffer;
      g_audiobuffer_end = g_audiobuffer + g_frames_per_block * g_audio_channels * sizeof(int16);
    }
    int n = IntMin(len, g_audiobuffer_end - g_audiobuffer_cur);
    if (g_sdl_audio_mixer_volume == SDL_MIX_MAXVOLUME) {
      memcpy(stream, g_audiobuffer_cur, n);
    } else {
      SDL_memset(stream, 0, n);
      SDL_MixAudioFormat(stream, g_audiobuffer_cur, AUDIO_S16, n, g_sdl_audio_mixer_volume);
    }
    g_audiobuffer_cur += n;
    stream += n;
    len -= n;
  }
  SDL_UnlockMutex(g_audio_mutex);
}


// State for sdl renderer
static SDL_Renderer *g_renderer;
static SDL_Texture *g_texture;
static SDL_Rect g_sdl_renderer_rect;

static bool SdlRenderer_Init(SDL_Window *window) {
  if (g_config.shader)
    fprintf(stderr, "Warning: Shaders are supported only with the OpenGL backend\n");

  SDL_Renderer *renderer = SDL_CreateRenderer(g_window, -1,
                                              g_config.output_method == kOutputMethod_SDLSoftware ? SDL_RENDERER_SOFTWARE :
                                              SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (renderer == NULL) {
    printf("Failed to create renderer: %s\n", SDL_GetError());
    return false;
  }
  SDL_RendererInfo renderer_info;
  SDL_GetRendererInfo(renderer, &renderer_info);
  if (kDebugFlag) {
    printf("Supported texture formats:");
    for (Uint32 i = 0; i < renderer_info.num_texture_formats; i++)
      printf(" %s", SDL_GetPixelFormatName(renderer_info.texture_formats[i]));
    printf("\n");
  }
  g_renderer = renderer;
  if (!g_config.ignore_aspect_ratio)
    SDL_RenderSetLogicalSize(renderer, g_snes_width, g_snes_height);
  if (g_config.linear_filtering)
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "best");

  int tex_mult = 1;
  g_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                g_snes_width * tex_mult, g_snes_height * tex_mult);
  if (g_texture == NULL) {
    printf("Failed to create texture: %s\n", SDL_GetError());
    return false;
  }
  return true;
}

static void SdlRenderer_Destroy(void) {
  SDL_DestroyTexture(g_texture);
  SDL_DestroyRenderer(g_renderer);
}

static void SdlRenderer_GetOutputSize(int *width, int *height) {
  if (SDL_GetRendererOutputSize(g_renderer, width, height) != 0) {
    *width = 0;
    *height = 0;
  }
}

static void SdlRenderer_BeginDraw(int width, int height, uint8 **pixels, int *pitch) {
  g_sdl_renderer_rect.w = width;
  g_sdl_renderer_rect.h = height;
  if (SDL_LockTexture(g_texture, &g_sdl_renderer_rect, (void **)pixels, pitch) != 0) {
    printf("Failed to lock texture: %s\n", SDL_GetError());
    return;
  }
}

static void SdlRenderer_EndDraw(void) {
  //  uint64 before = SDL_GetPerformanceCounter();
  SDL_UnlockTexture(g_texture);
  //  uint64 after = SDL_GetPerformanceCounter();
  //  float v = (double)(after - before) / SDL_GetPerformanceFrequency();
  //  printf("%f ms\n", v * 1000);
  SDL_RenderClear(g_renderer);
  SDL_RenderCopy(g_renderer, g_texture, &g_sdl_renderer_rect, NULL);
  SDL_RenderPresent(g_renderer); // vsyncs to 60 FPS?
}

static const struct RendererFuncs kSdlRendererFuncs = {
  &SdlRenderer_Init,
  &SdlRenderer_Destroy,
  &SdlRenderer_GetOutputSize,
  &SdlRenderer_BeginDraw,
  &SdlRenderer_EndDraw,
};


void MkDir(const char *s) {
#if defined(_WIN32)
  _mkdir(s);
#else
  mkdir(s, 0755);
#endif
}

#include <signal.h>
#include "cpu_state.h"
#include "cpu_trace.h"
#include "post_mortem.h"
extern uint8_t g_ram[0x20000];
static void dump_cpu_state(void) {
  fprintf(stderr, "CPU state at crash:\n");
  fprintf(stderr, "  v2 CpuState: A=%04X X=%04X Y=%04X S=%04X D=%04X DB=%02X PB=%02X "
                  "P=%02X m=%u x=%u e=%u\n",
                  g_cpu.A, g_cpu.X, g_cpu.Y, g_cpu.S, g_cpu.D, g_cpu.DB, g_cpu.PB,
                  g_cpu.P, g_cpu.m_flag, g_cpu.x_flag, g_cpu.emulation);
  fprintf(stderr, "  WRAM $0000..$001F:");
  for (int k = 0; k < 32; k++) fprintf(stderr, " %02x", g_ram[k]);
  fprintf(stderr, "\n");
}
static void crash_handler(int sig) {
  extern const char *g_last_recomp_func;
  extern void RecompStackDump(void);
  fprintf(stderr, "\n*** CRASH (signal %d) in recomp func: %s ***\n",
          sig, g_last_recomp_func ? g_last_recomp_func : "(unknown)");
  dump_cpu_state();
  RecompStackDump();
  cpu_trace_dump_dbpb("CRASH — DB/PB mutations");
  cpu_trace_dump_recent("CRASH — main trace ring", 256);
  fflush(stderr);
  recomp_post_mortem_dump("signal", NULL);
  _exit(128 + sig);
}

#ifdef _WIN32
#include <windows.h>
static LONG WINAPI seh_handler(EXCEPTION_POINTERS* info) {
  extern const char *g_last_recomp_func;
  extern void RecompStackDump(void);
  DWORD code = info->ExceptionRecord->ExceptionCode;
  void* addr = info->ExceptionRecord->ExceptionAddress;
  fprintf(stderr, "\n*** SEH CRASH code=0x%08lX at %p, last recomp func: %s ***\n",
          code, addr, g_last_recomp_func ? g_last_recomp_func : "(unknown)");
  if (code == EXCEPTION_ACCESS_VIOLATION) {
    ULONG_PTR kind = info->ExceptionRecord->ExceptionInformation[0];
    ULONG_PTR fault_addr = info->ExceptionRecord->ExceptionInformation[1];
    fprintf(stderr, "    access violation: %s at 0x%p\n",
            kind == 0 ? "read" : (kind == 1 ? "write" : "execute"),
            (void*)fault_addr);
  }
  dump_cpu_state();
  RecompStackDump();
  cpu_trace_dump_dbpb("SEH CRASH — DB/PB mutations");
  cpu_trace_dump_recent("SEH CRASH — main trace ring", 256);
  fflush(stderr);
  recomp_post_mortem_dump("seh", info);
  return EXCEPTION_EXECUTE_HANDLER;
}
#endif

static void post_mortem_atexit(void) {
  recomp_post_mortem_dump("atexit", NULL);
}

/* Resolve a relative CLI path against the launch cwd before
 * snesrecomp_anchor_to_exe_dir() redefines what relative means.
 * Returns `buf` on success, the original pointer otherwise. */
static const char *AbsolutizePathArg(const char *path, char *buf, size_t size) {
  extern int snesrecomp_abspath(const char *path, char *out, size_t max_len);
  return (path && snesrecomp_abspath(path, buf, size)) ? buf : path;
}

#undef main
/* Issue #4: bring the selected ROM into the exe directory so it sits beside the
 * saves/, config.ini and keybinds.ini already anchored there. Copies (never
 * moves) the ROM under its basename and rewrites `rom_path` to the local copy.
 * No-op when already in the exe dir or the copy can't be made. */
static int RelocateRomToExeDir(char *rom_path, size_t cap) {
  if (!rom_path || !rom_path[0]) return 0;
  const char *base = rom_path;
  for (const char *p = rom_path; *p; p++)
    if (*p == '/' || *p == '\\') base = p + 1;
  if (!*base) return 0;

  char dst[1024];
  if (!snesrecomp_exe_dir_path(base, dst, sizeof(dst))) return 0;
#ifdef _WIN32
  if (_stricmp(dst, rom_path) == 0) return 0;
#else
  if (strcmp(dst, rom_path) == 0) return 0;
#endif

  FILE *in = fopen(rom_path, "rb");
  if (!in) return 0;
  FILE *out = fopen(dst, "wb");
  if (!out) { fclose(in); return 0; }
  char buf[65536];
  size_t n;
  int ok = 1;
  while ((n = fread(buf, 1, sizeof(buf), in)) > 0)
    if (fwrite(buf, 1, n, out) != n) { ok = 0; break; }
  fclose(in);
  fclose(out);
  if (!ok) { remove(dst); return 0; }

  snprintf(rom_path, cap, "%s", dst);
  printf("[Launcher] Copied ROM into the game directory: %s\n", dst);
  return 1;
}

int main(int argc, char** argv) {
  /* On Windows, do NOT install a SIGSEGV handler: the MSVC CRT's signal
   * shim intercepts access violations BEFORE the SetUnhandledExceptionFilter
   * below, so crashes would reach crash_handler with no EXCEPTION_POINTERS —
   * no exception record in the minidump/report. With SIGSEGV uninstalled,
   * AVs reach the SEH filter with full fault context. */
#ifndef _WIN32
  signal(SIGSEGV, crash_handler);
#endif
  signal(SIGABRT, crash_handler);
#ifdef _WIN32
  SetUnhandledExceptionFilter(seh_handler);
  /* Suppress the Windows error dialog so SEH unwinds straight to our
   * filter and we can write the post-mortem report without the user
   * having to dismiss a popup first. */
  SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
#endif
  atexit(post_mortem_atexit);
  host_report_init(kWindowTitle, SNESRECOMP_BUILD_VERSION);
  /* ARM the backwards watcher BEFORE any recompiled code runs. Without
   * this, the trace ring records but no tripwires fire. With this:
   * - DB-watch on every byte SMW shouldn't legitimately use as DB
   * - PB-watch on every non-zero PB
   * - S-watch when stack leaves $0100-$1FFF
   * - Func-watch on the bank03.cfg empty stub
   * - Off-rails dumps (rate-limited) from RomPtr/cart_readLorom soft fails
   * Each tripwire dumps the trace BACKWARDS so we see the chain that
   * birthed the bad state, not just where it died. */
  /* Heap-allocate the cpu trace ring before any tripwire arms. The
   * default 64M entries cover ~64K frames at typical block rates,
   * which means the ring no longer rolls over within any realistic
   * investigation window. Override via SNESRECOMP_CPU_TRACE_RING_ENTRIES. */
  cpu_trace_init();
  cpu_trace_arm_default_watches();
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);
  cpu_set_stage_window_store_hook(MwOnStageWindowStore);
  dma_set_vram_notify_hook(MwNotifyBg2MapDma);
  snes_netplay_set_sync_byte_hooks(CaptureNetplaySyncBytes,
                                   ApplyNetplaySyncBytes);
#ifdef __SWITCH__
  SwitchImpl_Init();
#endif
  argc--, argv++;
  /* Path-carrying args are resolved against the LAUNCH cwd; the anchor
   * below changes what relative paths mean, so absolutize them first. */
  const char *config_file = NULL;
  if (argc >= 2 && strcmp(argv[0], "--config") == 0) {
    static char config_abs[1024];
    config_file = AbsolutizePathArg(argv[1], config_abs, sizeof(config_abs));
    argc -= 2, argv += 2;
  }
  int start_paused = 0;
  if (argc >= 1 && strcmp(argv[0], "--paused") == 0) {
    start_paused = 1;
    argc -= 1, argv += 1;
  }
  const char *script_file = NULL;
  if (argc >= 2 && strcmp(argv[0], "--script") == 0) {
    static char script_abs[1024];
    script_file = AbsolutizePathArg(argv[1], script_abs, sizeof(script_abs));
    argc -= 2, argv += 2;
  }
  const char *framedump_dir = NULL;
  if (argc >= 2 && strcmp(argv[0], "--framedump") == 0) {
    static char framedump_abs[1024];
    framedump_dir = AbsolutizePathArg(argv[1], framedump_abs, sizeof(framedump_abs));
    argc -= 2, argv += 2;
  }
  /* --launcher forces the GUI; --no-launcher skips it (CI / scripts). */
  int force_launcher = 0;
  int force_no_launcher = 0;
  while (argc >= 1) {
    if (strcmp(argv[0], "--launcher") == 0) {
      force_launcher = 1;
      argc -= 1, argv += 1;
      continue;
    }
    if (strcmp(argv[0], "--no-launcher") == 0) {
      force_no_launcher = 1;
      argc -= 1, argv += 1;
      continue;
    }
    break;
  }
  if (argc >= 1 && argv[0] && argv[0][0] != '-' && argv[0][0] != '\0') {
    /* Positional ROM path. */
    static char rom_abs[1024];
    argv[0] = (char *)AbsolutizePathArg(argv[0], rom_abs, sizeof(rom_abs));
  }

  /* The config is config.ini next to the executable — nothing else,
   * no directory walking. Anchoring cwd to the exe dir also pins
   * keybinds.ini, rom.cfg and saves/ there, however the process was
   * launched. (On read-only installs the anchor declines and cwd
   * stays authoritative; see launcher.h.) */
  {
    extern int snesrecomp_anchor_to_exe_dir(void);
    int anchored = snesrecomp_anchor_to_exe_dir();
    host_report_breadcrumb("exe-dir anchor: %s",
                           anchored ? "ok" : "declined (cwd stays authoritative)");
  }

  /* Carry a legacy saves/<title>.srm forward to saves/save.srm if present.
   * Idempotent; RtlReadSram also calls it on boot as a fallback. */
  RtlMigrateLegacySram("metalwarriors");
  if (!config_file)
    EnsureConfigIni();
  /* Pin config.ini to the exe directory for all reads/writes, regardless of the
   * current working directory (defense in depth with snesrecomp_anchor_to_exe_dir
   * above + OFN_NOCHANGEDIR on the file dialogs). EnsureConfigIni already created
   * it next to the exe; this keeps a stray chdir from relocating later writes. */
  static char config_exe_path[1024];
  if (!config_file &&
      snesrecomp_exe_dir_path("config.ini", config_exe_path, sizeof(config_exe_path)))
    config_file = config_exe_path;
  ParseConfigFile(config_file);
  // Apply local overrides if present (gitignored). Lets a developer
  // mute audio etc. without touching the checked-in config.ini. Last
  // parser to set a key wins, so local overrides take precedence.
  {
    FILE *f_local = fopen("config.local.ini", "rb");
    if (f_local) {
      fclose(f_local);
      ParseConfigFile("config.local.ini");
    }
  }
  host_report_breadcrumb(
      "config parsed: output=%d new_renderer=%d scale=%d fullscreen=%d "
      "audio=%d freq=%d samples=%d skip_launcher=%d",
      g_config.output_method, g_config.new_renderer, g_config.window_scale,
      g_config.fullscreen, g_config.enable_audio, g_config.audio_freq,
      g_config.audio_samples, g_config.skip_launcher);
  host_report_breadcrumb("display: widescreen_extra=%d msu1=%d (unsupported)",
                         (int)g_config.widescreen, g_config.msu1_enabled);

  /* Metal Warriors (USA) — SHA-256 is the canonical no-intro digest;
   * 512-byte SMC headers are auto-stripped before hashing. */
  static const uint8_t kMwRomSha256[32] = {
    0x0d,0x7f,0x87,0x58,0x77,0xfe,0x85,0x60,
    0x66,0xcf,0xb3,0x9b,0x4e,0xcd,0xbb,0xe7,
    0xd4,0x83,0x93,0xa7,0x57,0x70,0x72,0x08,
    0x76,0xc9,0x44,0x19,0xf8,0x09,0xbb,0x1c
  };

  static char rom_path_buf[512];
  rom_path_buf[0] = '\0';

  /* Headless/dev bring-up path for two-instance LAN validation. The same
   * facade and runtime loop are used as launcher sessions; only the launch
   * handoff comes from SNES_NET_* environment variables. */
  snes_netplay_config_defaults(&g_netplay_cfg);
  snes_netplay_apply_env(&g_netplay_cfg);
  if (g_netplay_cfg.enabled) {
    g_netplay_pending = 1;
    g_netplay_caps_ws_extra = 71;
    force_no_launcher = 1;
  }

#if defined(RECOMP_LAUNCHER)
  /* Launcher by default. Skip for CI/scripts, SkipLauncher=1, or a
   * positional ROM (unless --launcher forces the GUI). */
  {
    const char *vd = getenv("SDL_VIDEODRIVER");
    int headless = (vd && strcmp(vd, "dummy") == 0);
    int has_positional_rom =
        (argc >= 1 && argv[0] && argv[0][0] != '\0' && argv[0][0] != '-');
    int run_gui = !force_no_launcher && !headless &&
                  !start_paused && !script_file && !framedump_dir &&
                  (force_launcher ||
                   (!g_config.skip_launcher && !has_positional_rom));

    if (run_gui) {
      char assets_dir[1024];
      if (!FindLauncherAssetsDir(assets_dir, sizeof(assets_dir))) {
        fprintf(stderr,
                "[main] launcher assets not found — falling back to ROM resolve\n");
      } else {
        RecompLauncherCSettings ls;
        LauncherSettingsFromConfig(&ls);
        RecompLauncherCGameInfo gi = {0};
        launcher_profile_apply("snes", &gi);
        gi.name = "Metal Warriors";
        gi.region = "(USA)";
        gi.expected_crc = 0xf2ab92d4u;
        gi.has_expected_crc = 1;
        gi.known_sha256 = &kMwRomSha256;
        gi.num_known_sha256 = 1;
        /* Hide WS toggle; netplay match_caps still force expand 71.
         * Offline clears WS and uses native split + local P2. */
        gi.widescreen_supported = 0;
        gi.num_players = 2;
        gi.msu1_supported = 0;
        gi.sram_path = "saves/save.srm";
        gi.config_path = config_file;
        gi.netplay_supported = 1;
        gi.netplay = &g_launcher_netplay_callbacks;

        char initial_rom[512] = "";
        if (has_positional_rom)
          snprintf(initial_rom, sizeof(initial_rom), "%s", argv[0]);
        else {
          FILE *rc = fopen("rom.cfg", "r");
          if (rc) {
            if (fgets(initial_rom, sizeof(initial_rom), rc)) {
              size_t n = strlen(initial_rom);
              while (n && (initial_rom[n - 1] == '\n' ||
                           initial_rom[n - 1] == '\r'))
                initial_rom[--n] = '\0';
            }
            fclose(rc);
          }
        }

        int lr = recomp_launcher_run_window(
            kWindowTitle, &ls, &gi, assets_dir,
            initial_rom[0] ? initial_rom : NULL,
            rom_path_buf, sizeof(rom_path_buf));
        RecompLauncherCNetplayLaunch net = ls.netplay_launch;
        if (lr == 1) {
          if (g_launcher_hosting_lan || g_launcher_joined_lan)
            (void)LauncherNpLeave(NULL);
          host_report_breadcrumb("launcher: quit");
          return 0;
        }
        if (lr == 0 && rom_path_buf[0]) {
          ConfigFromLauncherSettings(&ls);
          /* Netplay: lock WS 71. Offline: hard-off for native split H2H. */
          g_config.widescreen = net.enabled ? 71 : 0;
          WriteConfigFile(config_file);
          FILE *rc = fopen("rom.cfg", "w");
          if (rc) {
            fprintf(rc, "%s\n", rom_path_buf);
            fclose(rc);
          }
          if (net.enabled) {
            snes_netplay_config_defaults(&g_netplay_cfg);
            g_netplay_cfg.enabled = 1;
            g_netplay_cfg.local_slot = net.local_slot;
            g_netplay_cfg.input_player =
                (net.input_player == 0 || net.input_player == 1)
                    ? net.input_player : -1;
            g_netplay_cfg.session_id = net.session_id ? net.session_id : 1u;
            g_netplay_cfg.transport = 0;
            snprintf(g_netplay_cfg.bind_hostport, sizeof(g_netplay_cfg.bind_hostport),
                     "%s", net.bind_hostport);
            snprintf(g_netplay_cfg.peer_hostport, sizeof(g_netplay_cfg.peer_hostport),
                     "%s", net.peer_hostport);
            snes_netplay_apply_env(&g_netplay_cfg);
            /* Host match_caps win over local SNES_NET_INPUT_DELAY. */
            if (net.input_delay >= 0 && net.input_delay <= 16)
              g_netplay_cfg.input_delay = net.input_delay;
            /* MW: always lock peers to 71 — never local 95 / unset fallback. */
            g_netplay_caps_ws_extra = 71;
            g_netplay_pending = 1;
            g_netplay_from_lobby = 1;
            host_report_breadcrumb(
                "launcher: netplay launch rom=%s slot=%d session=%u "
                "bind=%s peer=%s name=%s delay=%d ws_extra=%d",
                rom_path_buf, net.local_slot, (unsigned)net.session_id,
                net.bind_hostport, net.peer_hostport, ls.netplay_player_name,
                net.input_delay, g_netplay_caps_ws_extra);
          } else {
            host_report_breadcrumb("launcher: launch rom=%s", rom_path_buf);
          }
        } else {
          rom_path_buf[0] = '\0';
          host_report_breadcrumb(
              "launcher: unavailable/no-rom — falling back to ROM resolve");
        }
      }
    }
  }
#endif

  /* Resolve the SNES ROM path when the GUI did not supply one:
   * argv[0] -> rom.cfg cache -> file picker. */
  if (!rom_path_buf[0]) {
    char *la_argv[2] = {
      (char *)"metalwarriors",
      (char *)((argc >= 1 && argv[0]) ? argv[0] : "")
    };
    int la_argc = (la_argv[1][0] != '\0') ? 2 : 1;
    extern int snesrecomp_launcher_resolve_rom_sha256(
        int, char **, char *, size_t, const uint8_t *);
    if (!snesrecomp_launcher_resolve_rom_sha256(la_argc, la_argv, rom_path_buf,
                                                sizeof(rom_path_buf), kMwRomSha256)) {
      /* User cancelled the picker or repeatedly chose a non-matching ROM. */
      return 1;
    }
  }
  /* Issue #4: co-locate the ROM with the exe (interactive launches only). */
  if (!start_paused && script_file == NULL && framedump_dir == NULL) {
    if (RelocateRomToExeDir(rom_path_buf, sizeof(rom_path_buf))) {
      FILE *rc = fopen("rom.cfg", "w");
      if (rc) { fprintf(rc, "%s\n", rom_path_buf); fclose(rc); }
    }
  }

  static char *resolved_argv[2];
  resolved_argv[0] = rom_path_buf;
  resolved_argv[1] = NULL;
  argv = resolved_argv;
  argc = 1;
  host_report_breadcrumb("rom resolved: %s", rom_path_buf);

  // Initialize debug server
  {
    extern int debug_server_init(int port);
    extern void debug_server_set_ram(uint8_t *ram, uint32_t ram_size);
    /* Per-game debug server port: 4377 SMW, 4378 Zelda, 4379 MMX, 4380 Metal Warriors.
     * Lets sibling games run concurrently on the same host without TCP-bind collisions. */
    if (debug_server_init(4380) == 0) {
      fprintf(stderr, "[main] Debug server ready on port 4380\n");
    }
    if (start_paused) {
      debug_server_start_paused();
      fprintf(stderr, "[main] Started paused — send 'step N' or 'continue' via TCP\n");
    }
  }

  // Widescreen capacity from config (extra columns per side; 0 = authentic).
  // g_ws_extra/g_ws_active live in the shared runner asset (widescreen.c) so
  // the PPU, the present helper, and the injected game-logic snippets all read
  // one canonical master switch. Default off => byte-identical faithful build.
  g_ws_extra = IntMin((int)g_config.widescreen, kWsExtraMax);
  g_ws_active = (g_ws_extra > 0);
  g_snes_width = 256 + 2 * g_ws_extra;
  g_snes_height = 224;// (g_config.extend_y ? 240 : 224);
  // A wider viewport can expose more sprites on one scanline than the SNES
  // could see at 256px. Keep authentic caps configurable at 4:3, but lift them
  // whenever widescreen is active so sprites do not disappear prematurely.
  g_ppu_render_flags = g_config.new_renderer * kPpuRenderFlags_NewRenderer |
    g_config.extend_y * kPpuRenderFlags_Height240 |
    (g_config.no_sprite_limits || g_ws_active) *
      kPpuRenderFlags_NoSpriteLimits;

  if (g_config.fullscreen == 1)
    g_win_flags ^= SDL_WINDOW_FULLSCREEN_DESKTOP;
  else if (g_config.fullscreen == 2)
    g_win_flags ^= SDL_WINDOW_FULLSCREEN;

  // Window scale (1=100%, 2=200%, 3=300%, etc.)
  g_current_window_scale = (g_config.window_scale == 0) ? 2 : IntMin(g_config.window_scale, kMaxWindowScale);

  // audio_freq: Use common sampling rates (see user config file. values higher than 48000 are not supported.)
  if (g_config.audio_freq < 11025 || g_config.audio_freq > 48000)
    g_config.audio_freq = kDefaultFreq;

  // Currently, the SPC/DSP implementation only supports up to stereo.
  if (g_config.audio_channels < 1 || g_config.audio_channels > 2)
    g_config.audio_channels = kDefaultChannels;

  // audio_samples: power of 2
  if (g_config.audio_samples <= 0 || ((g_config.audio_samples & (g_config.audio_samples - 1)) != 0))
    g_config.audio_samples = kDefaultSamples;

  if (g_config.gamepad_deadzone <= 0 || g_config.gamepad_deadzone > 32767)
    g_config.gamepad_deadzone = 10000;

  SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");

  // set up SDL
  if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMECONTROLLER) != 0) {
    host_report_breadcrumb("SDL_Init FAILED: %s", SDL_GetError());
    printf("Failed to init SDL: %s\n", SDL_GetError());
    return 1;
  }
  host_report_breadcrumb("SDL init ok: video=%s audio=%s",
                         SDL_GetCurrentVideoDriver() ? SDL_GetCurrentVideoDriver() : "(none)",
                         SDL_GetCurrentAudioDriver() ? SDL_GetCurrentAudioDriver() : "(none)");

  /* Load (or generate) keybinds.ini next to the executable (cwd is
   * anchored there; on read-only installs it tracks the config). */
  keybinds_init(NULL);

  bool custom_size = g_config.window_width != 0 && g_config.window_height != 0;
  int window_width = custom_size ? g_config.window_width : g_current_window_scale * g_snes_width;
  int window_height = custom_size ? g_config.window_height : g_current_window_scale * g_snes_height;

  /* Load the SNES ROM once. Rematches reuse the same buffer. */
  uint8 *kRom = NULL;
  uint32 kRom_SIZE = 0;
  if (argv[0]) {
    size_t size;
    kRom = ReadWholeFile(argv[0], &size);
    kRom_SIZE = (uint32)size;
    if (!kRom)
      goto error_reading;
  }
  host_report_breadcrumb("rom loaded: %u bytes", kRom_SIZE);

  extern const RtlGameInfo kSmwGameInfo;
  RtlRegisterGame(&kSmwGameInfo);

  static int s_emu_session = 0;
session_reboot:
  /* Soft-return rematch: recomp-ui launcher_platform_close() calls SDL_Quit().
   * Re-init before recreating the window/audio. First boot already inited
   * above; skip when subsystems are still live. */
  if (!SDL_WasInit(SDL_INIT_VIDEO) || !SDL_WasInit(SDL_INIT_AUDIO) ||
      !SDL_WasInit(SDL_INIT_GAMECONTROLLER)) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMECONTROLLER) !=
        0) {
      host_report_breadcrumb("SDL_Init (session) FAILED: %s", SDL_GetError());
      printf("Failed to init SDL: %s\n", SDL_GetError());
      return 1;
    }
    host_report_breadcrumb(
        "SDL session init ok: video=%s audio=%s",
        SDL_GetCurrentVideoDriver() ? SDL_GetCurrentVideoDriver() : "(none)",
        SDL_GetCurrentAudioDriver() ? SDL_GetCurrentAudioDriver() : "(none)");
  }
  /* Rematch after lobby soft-return re-enters here with updated netplay cfg. */
  {
    const int rematch_session = (++s_emu_session > 1);
    if (rematch_session)
      host_report_breadcrumb("netplay: rematch session reboot");
  }

  /* MotK-class sticky latch: LLE resume PC / host_frames survive snes_free.
   * Clear before SnesInit so rematch cold-boots instead of WAI on a blank chip. */
  MwSessionReset();

  /* Netplay: forced expand 71 (peer lock) + full-frame H2H present (default).
   * Offline: WS hard-off for native dual split + local multiplayer.
   * OAM vert-widen defaults ON for netplay (VERT_WIDEN=0 to opt out).
   * Spawn-Y widen still opt-in (SPAWN_Y_WIDEN=1). Both hard-off offline.
   * Opt out full-frame: SNESRECOMP_MW_H2H_FULL_FRAME=0. */
  if (g_netplay_pending) {
    g_config.widescreen = 71;
    g_ws_extra = 71;
    g_ws_active = 1;
    g_netplay_caps_ws_extra = 71;
    host_report_breadcrumb("netplay: widescreen extra=%d (H2H Phase 2a, forced)",
                           g_ws_extra);
  } else {
    g_config.widescreen = 0;
    g_ws_extra = 0;
    g_ws_active = 0;
    host_report_breadcrumb("offline: widescreen off (native split H2H)");
  }
  g_snes_width = 256 + 2 * g_ws_extra;
  g_ppu_render_flags =
      g_config.new_renderer * kPpuRenderFlags_NewRenderer |
      g_config.extend_y * kPpuRenderFlags_Height240 |
      (g_config.no_sprite_limits || g_ws_active) *
          kPpuRenderFlags_NoSpriteLimits;
  if (!custom_size) {
    window_width = g_current_window_scale * g_snes_width;
    window_height = g_current_window_scale * g_snes_height;
  }

  if (g_config.output_method == kOutputMethod_OpenGL) {
    g_win_flags |= SDL_WINDOW_OPENGL;
    OpenGLRenderer_Create(&g_renderer_funcs);
  } else {
    g_renderer_funcs = kSdlRendererFuncs;
  }

  Snes *snes = SnesInit(kRom, kRom_SIZE);
  host_report_breadcrumb("SnesInit: %s", snes ? "ok" : "FAILED");
  if (snes == NULL) {
error_reading:;
#ifdef __SWITCH__
    ThrowMissingROM();
#else
    char buf[256];
    snprintf(buf, sizeof(buf), "unable to load rom");
    Die(buf);
#endif
    return 1;
  }

  // Connect debug server to SNES RAM
  {
    extern void debug_server_set_ram(uint8_t *ram, uint32_t ram_size);
    debug_server_set_ram(snes->ram, 0x20000);
  }

  g_gamepad[0].joystick_id = g_gamepad[1].joystick_id = -1;
  memset(&g_gamepad[0], 0, sizeof(g_gamepad[0]));
  memset(&g_gamepad[1], 0, sizeof(g_gamepad[1]));
  g_gamepad[0].joystick_id = g_gamepad[1].joystick_id = -1;
  g_input_state = 0;
  g_pad_buttons = 0;
  g_paused = 0;
  g_turbo = 0;

  SDL_Window *window = SDL_CreateWindow(kWindowTitle, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, window_width, window_height, g_win_flags);
  if(window == NULL) {
    host_report_breadcrumb("SDL_CreateWindow FAILED: %s", SDL_GetError());
    printf("Failed to create window: %s\n", SDL_GetError());
    return 1;
  }
  g_window = window;
  SDL_SetWindowHitTest(window, HitTestCallback, NULL);
  host_report_breadcrumb("window created: %dx%d flags=0x%x",
                         window_width, window_height, g_win_flags);

  if (!g_renderer_funcs.Initialize(window)) {
    host_report_breadcrumb("renderer init FAILED (output_method=%d)",
                           g_config.output_method);
    return 1;
  }
  host_report_breadcrumb("renderer initialized: %s",
      g_config.output_method == kOutputMethod_OpenGL ? "opengl" :
      g_config.output_method == kOutputMethod_SDLSoftware ? "sdl-software" : "sdl");

  // Build the present-time color LUT from SNESRECOMP_SCREEN (default raw = off).
  snes_color_lut_setup();

  g_audio_mutex = SDL_CreateMutex();
  if (!g_audio_mutex) Die("No mutex");

  if (!g_spc_player)
    g_spc_player = SmwSpcPlayer_Create();
  g_spc_player->initialize(g_spc_player);
  host_report_breadcrumb("SPC player initialized");

  if (g_config.enable_audio) {
    /* Enumerate output devices into the breadcrumb ring: which device
     * SDL picks (and what else was available) is exactly the per-machine
     * variable a non-reproducible audio/boot crash report needs. */
    {
      int ndev = SDL_GetNumAudioDevices(0);
      host_report_breadcrumb("audio outputs: %d device(s)", ndev);
      for (int i = 0; i < ndev && i < 8; i++)
        host_report_breadcrumb("audio output[%d]: %s", i,
                               SDL_GetAudioDeviceName(i, 0));
    }
    SDL_AudioSpec want = { 0 }, have;
    want.freq = g_config.audio_freq;
    want.format = AUDIO_S16;
    want.channels = 2;
    want.samples = g_config.audio_samples;
    want.callback = &AudioCallback;
    g_audio_device = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (g_audio_device == 0) {
      host_report_breadcrumb("audio device open FAILED: %s", SDL_GetError());
      printf("Failed to open audio device: %s\n", SDL_GetError());
      return 1;
    }
    g_audio_channels = 2;
    /* One native DSP block is 534 samples at the SPC's true output rate
     * of 32040 Hz (1.024 MHz / 32). The old divisor of 32000 understated
     * the rate, playing everything a constant -2.2 cents flat (measured
     * vs the snes9x oracle, MMX issue #4); the truncating division also
     * undersized the block for non-multiple rates. Round to the nearest
     * frame: 32040->534 (1:1, no resample), 48000->800, 44100->735. */
    g_frames_per_block = (534 * have.freq + 32040 / 2) / 32040;
    g_audiobuffer = (uint8 *)calloc(g_frames_per_block * have.channels * sizeof(int16), 1);
    host_report_breadcrumb(
        "audio device opened: freq=%d (want %d) ch=%d samples=%d frames_per_block=%d",
        have.freq, want.freq, have.channels, have.samples, g_frames_per_block);
  } else {
    host_report_breadcrumb("audio disabled in config");
  }

  // Pitch tracks the widescreen capacity (g_snes_width == 256 when off, so this
  // is the authentic 256*4 stride unless widescreen is active).
  PpuBeginDrawing(g_ppu, g_my_pixels, g_snes_width * 4, 0);

  MkDir("saves");
    
  RtlReadSram();

  {
    int njs = SDL_NumJoysticks();
    printf("[Gamepad] SDL reports %d joystick(s) at startup. "
           "enable_gamepad=[%d,%d]\n",
           njs, g_config.enable_gamepad[0], g_config.enable_gamepad[1]);
    for (int i = 0; i < njs; i++) {
      const char *name = SDL_JoystickNameForIndex(i);
      int is_gc = SDL_IsGameController(i);
      printf("[Gamepad]   #%d name=%s is_game_controller=%d\n",
             i, name ? name : "(null)", is_gc);
      OpenOneGamepad(i);
    }
    if (njs == 0) {
      printf("[Gamepad] No joysticks detected. "
             "On Windows, plug controller in BEFORE launching, "
             "or check that XInput drivers are installed.\n");
    }
  }

  /* Netplay (incl. rematch) must cold-boot both peers from the same reset;
   * autosave would restore a mid-match host snapshot and desync / re-latch LLE. */
  if (g_config.autosave && !g_netplay_pending)
    HandleCommand(kKeys_Load + 0, true);

  if (script_file)
    LoadScript(script_file);

  if (framedump_dir)
    FrameDump_Init(framedump_dir);

  bool running = true;
  uint32 lastTick = SDL_GetTicks();
  uint32 curTick = 0;
  uint32 frameCtr = 0;
  uint8 audiopaused = true;
  GamepadInfo *gi;

  if (g_netplay_pending) {
    if (g_netplay_cfg.input_player != 0 && g_netplay_cfg.input_player != 1)
      g_netplay_cfg.input_player =
          ResolveNetplayInputPlayer(g_netplay_cfg.local_slot);
    int nrc = snes_netplay_start(&g_netplay_cfg);
    if (nrc != 0) {
      fprintf(stderr, "snes_netplay_start failed (%d) — continuing offline\n", nrc);
      host_report_breadcrumb("netplay: start failed rc=%d", nrc);
    } else {
      host_report_breadcrumb("netplay: session started slot=%d input_player=%d",
                             snes_netplay_local_slot(), snes_netplay_input_player());
      g_turbo = 0; /* lockstep forbids turbo */
    }
    g_netplay_pending = 0;
  }

  host_report_breadcrumb("entering main loop");

  while (running) {
    SDL_Event event;

    /* Inert unless SNESRECOMP_CRASH_TEST is set — support drill for the
     * whole crash-capture pipeline (minidump + report + crash copy). */
    host_report_crash_test_tick();

    while (SDL_PollEvent(&event)) {
      switch (event.type) {
      case SDL_CONTROLLERDEVICEADDED:
        OpenOneGamepad(event.cdevice.which);
        break;
      case SDL_CONTROLLERDEVICEREMOVED:
        gi = GetGamepadInfo(event.cdevice.which);
        if (gi) {
          memset(gi, 0, sizeof(GamepadInfo));
          gi->joystick_id = -1;
        }
        break;
      case SDL_CONTROLLERAXISMOTION:
        gi = GetGamepadInfo(event.caxis.which);
        if (gi)
          HandleGamepadAxisInput(gi, event.caxis.axis, event.caxis.value);
        break;
      case SDL_CONTROLLERBUTTONDOWN:
      case SDL_CONTROLLERBUTTONUP: {
        gi = GetGamepadInfo(event.cbutton.which);
        if (gi) {
          int b = RemapSdlButton(event.cbutton.button);
          if (b >= 0)
            HandleGamepadInput(gi, b, event.type == SDL_CONTROLLERBUTTONDOWN);
        }
        break;
      }
      case SDL_MOUSEWHEEL:
        if (SDL_GetModState() & KMOD_CTRL && event.wheel.y != 0)
          ChangeWindowScale(event.wheel.y > 0 ? 1 : -1);
        break;
      case SDL_MOUSEBUTTONDOWN:
        if (event.button.button == SDL_BUTTON_LEFT && event.button.state == SDL_PRESSED && event.button.clicks == 2) {
          if ((g_win_flags & SDL_WINDOW_FULLSCREEN_DESKTOP) == 0 && (g_win_flags & SDL_WINDOW_FULLSCREEN) == 0 && SDL_GetModState() & KMOD_SHIFT) {
            g_win_flags ^= SDL_WINDOW_BORDERLESS;
            SDL_SetWindowBordered(g_window, (g_win_flags & SDL_WINDOW_BORDERLESS) == 0 ? SDL_TRUE : SDL_FALSE);
          }
        }
        break;
      case SDL_KEYDOWN:
        if (snes_netplay_active() && event.key.keysym.sym == SDLK_ESCAPE) {
          netplay_soft_exit("escape");
          if (snes_netplay_return_to_lobby_requested())
            running = false;
          break;
        }
        HandleInput(event.key.keysym.sym, event.key.keysym.mod, true);
        break;
      case SDL_KEYUP:
        HandleInput(event.key.keysym.sym, event.key.keysym.mod, false);
        break;
      case SDL_QUIT:
        if (snes_netplay_active())
          netplay_soft_exit("sdl_quit");
        running = false;
        break;
      }
    }

    if (snes_netplay_return_to_lobby_requested()) {
      running = false;
      break;
    }

    if (g_paused != audiopaused) {
      audiopaused = g_paused;
      if (g_audio_device)
        SDL_PauseAudioDevice(g_audio_device, audiopaused);
    }

    if (g_paused) {
      SDL_Delay(16);
      continue;
    }

    // Clear gamepad inputs when joypad directional inputs to avoid wonkiness
    if (g_input_state & 0xf0)
      g_gamepad[0].axis_buttons = 0;
    if (g_input_state & 0xf0000)
      g_gamepad[1].axis_buttons = 0;
    {
      int ls = debug_server_consume_loadstate();
      if (ls >= 0) {
        if (!snes_netplay_request_load(ls))
          RtlSaveLoad(kSaveLoad_Load, ls);
      }
      int ss = debug_server_consume_savestate();
      if (ss >= 0) {
        if (!snes_netplay_request_save(ss))
          RtlSaveLoad(kSaveLoad_Save, ss);
      }
    }
    debug_server_wait_if_paused();

    /* Drive the SNES controller bits in g_input_state from keybinds.ini.
     * config.ini's [KeyMap] still owns system commands (state save/load,
     * fullscreen, pause, etc.); the 12 controller buttons per player
     * come from keybinds.ini. */
    RefreshKeybindControllerBits();

    uint32 inputs;
    if (snes_netplay_active()) {
      g_turbo = 0;
      if (!NetplayBarrierAdmit(&running)) {
        /* Stalled / left session — do not advance sim. */
        if (snes_netplay_return_to_lobby_requested() || !running)
          break;
        continue;
      }
      inputs = snes_netplay_published_inputs() | snes_netplay_active_mask();
      RtlRunFrame(inputs);
      snes_netplay_finish_frame();
      {
        static int test_ticks = -1;
        if (test_ticks < 0) {
          const char *value = getenv("SNES_NET_TEST_TICKS");
          test_ticks = value && value[0] ? atoi(value) : 0;
        }
        if (test_ticks > 0 &&
            snes_netplay_sim_tick() >= (uint32_t)test_ticks) {
          fprintf(stderr,
                  "SNES_NET_TEST_PASS slot=%d tick=%u\n",
                  snes_netplay_local_slot(),
                  (unsigned)snes_netplay_sim_tick());
          snes_netplay_shutdown();
          running = false;
        }
      }
    } else {
      inputs = g_input_state | g_pad_buttons | g_gamepad[0].axis_buttons |
               g_gamepad[1].axis_buttons << 12;
      inputs |= TickScript();
      inputs |= debug_server_get_controller_inputs();
      RtlRunFrame(inputs | GetActiveControllers() |
                  debug_server_get_controller_active_mask());
    }

    /* Headless scripts: exit once the script drains (opt-out via env=0). */
    if (g_script_finished) {
      static int quit_on_script = -1;
      if (quit_on_script < 0) {
        const char *e = getenv("SNESRECOMP_QUIT_ON_SCRIPT");
        quit_on_script = (!e || e[0] != '0') ? 1 : 0;
      }
      if (quit_on_script) {
        fprintf(stderr, "script: finished after %u frames — quitting\n",
                (unsigned)frameCtr);
        running = false;
      }
    }

    // Bank validation removed — 100% oracle mode, no banks enabled.

    frameCtr++;
    if (frameCtr == 1)
      host_report_breadcrumb("first frame simulated");
    else if (frameCtr % 60 == 0)   /* ~1 Hz during LLE bring-up */
      host_report_breadcrumb("heartbeat: frame=%u", frameCtr);
    /* Dev-only headless turbo stress (SNESRECOMP_FORCE_TURBO=1): forces the
     * turbo path every frame so an automated soak can exercise the LLE path's
     * turbo-safety without a human holding Tab. Env-gated; no ship effect. */
    { static int s_ft = -1;
      if (s_ft < 0) { const char *e = getenv("SNESRECOMP_FORCE_TURBO");
                      s_ft = (e && e[0] && e[0] != '0') ? 1 : 0; }
      if (s_ft && !snes_netplay_active()) g_turbo = 1; }
    g_snes->disableRender = g_turbo && (frameCtr & 0xf) != 0;

    if (!g_snes->disableRender) {
      DrawPpuFrameWithPerf();
    } else {
      /* Turbo (render skipped): draw_ppu_frame also simulates HDMA + fires the
       * raster IRQ (I_IRQ). Under the default LLE scheduler
       * the game runs the REAL NMI/IRQ machinery; skipping the raster IRQ would
       * let an IRQ-gated path spin → 5s watchdog longjmp mid-frame → guest stack
       * leak → wedge (the MMX turbo garble class). Run the guest-state sim every
       * frame; skip only the host present. Harmless to HLE. */
      g_rtl_game_info->draw_ppu_frame();
    }

    // if vsync isn't working, delay manually
    curTick = SDL_GetTicks();

    /* Frame-delay pacing locks the loop to ~60 fps so SPC + MSU-1 audio stays
     * in sync with the sound device. On by default. Power users on an
     * exactly-60 Hz / vsync-correct display can set DisableFrameDelay = 1 in
     * config.ini (cfg-only, no UI) to skip it for slightly better perf — at the
     * risk of audio desync on other displays. */
    if (!g_snes->disableRender && !g_config.disable_frame_delay) {
      static const uint8 delays[3] = { 17, 17, 16 }; // 60 fps
      lastTick += delays[frameCtr % 3];

      if (lastTick > curTick) {
        uint32 delta = lastTick - curTick;
        if (delta > 500) {
          lastTick = curTick - 500;
          delta = 500;
        }
        //        printf("Sleeping %d\n", delta);
        SDL_Delay(delta);
      } else if (curTick - lastTick > 500) {
        lastTick = curTick;
      }
    }
  }

  /* Soft-return to lobby: do not overwrite personal autosave with match state. */
  if (g_config.autosave && !snes_netplay_return_to_lobby_requested())
    HandleCommand(kKeys_Save + 0, true);

  snes_netplay_shutdown();
  RtlWriteSram();

  if (g_audio_device) {
    SDL_PauseAudioDevice(g_audio_device, 1);
    SDL_CloseAudioDevice(g_audio_device);
    g_audio_device = 0;
  }
  if (g_audio_mutex) {
    SDL_DestroyMutex(g_audio_mutex);
    g_audio_mutex = NULL;
  }
  free(g_audiobuffer);
  g_audiobuffer = NULL;
  g_audiobuffer_cur = NULL;
  g_audiobuffer_end = NULL;

  g_renderer_funcs.Destroy();
  memset(&g_renderer_funcs, 0, sizeof(g_renderer_funcs));

  if (g_snes) {
    snes_free(g_snes);
    g_snes = NULL;
  }

  SDL_DestroyWindow(window);
  g_window = NULL;
  window = NULL;

#if defined(RECOMP_LAUNCHER)
  if (snes_netplay_return_to_lobby_requested() && g_netplay_from_lobby) {
    /* Soft-return: keep lobby WebSocket; reopen MotK room view. */
    if (g_launcher_hosting_lan || g_launcher_joined_lan)
      (void)rnet_lan_lobby_set_started(LauncherLanLobbyPath(), 0);
    /* Auto-ready: there is no Ready toggle; keep seats launchable on rematch
     * (including older lobby servers that still gate start on all_ready). */
    snes_lobby_set_ready(1);
    snes_lobby_clear_launch_pending();
    snes_netplay_clear_return_to_lobby();
    g_netplay_from_lobby = 0;
    g_netplay_pending = 0;

    char assets_dir[1024];
    if (!FindLauncherAssetsDir(assets_dir, sizeof(assets_dir))) {
      fprintf(stderr, "metalwarriors: resume lobby failed — assets missing\n");
      snes_lobby_disconnect();
      free(kRom);
      SDL_Quit();
      return 1;
    }

    RecompLauncherCSettings ls;
    LauncherSettingsFromConfig(&ls);
    RecompLauncherCGameInfo gi = {0};
    launcher_profile_apply("snes", &gi);
    gi.name = "Metal Warriors";
    gi.region = "(USA)";
    gi.expected_crc = 0xf2ab92d4u;
    gi.has_expected_crc = 1;
    gi.known_sha256 = &kMwRomSha256;
    gi.num_known_sha256 = 1;
    gi.widescreen_supported = 0;
    gi.num_players = 2;
    gi.msu1_supported = 0;
    gi.sram_path = "saves/save.srm";
    gi.config_path = config_file;
    gi.netplay_supported = 1;
    gi.netplay = &g_launcher_netplay_callbacks;

    char resume_rom[512];
    snprintf(resume_rom, sizeof(resume_rom), "%s", rom_path_buf);
    int lr = recomp_launcher_run_window(
        kWindowTitle, &ls, &gi, assets_dir,
        resume_rom[0] ? resume_rom : NULL,
        rom_path_buf, sizeof(rom_path_buf));
    RecompLauncherCNetplayLaunch net = ls.netplay_launch;

    if (lr == 0) {
      ConfigFromLauncherSettings(&ls);
      if (net.enabled) {
        g_config.widescreen = 71;
        WriteConfigFile(config_file);
        snes_netplay_config_defaults(&g_netplay_cfg);
        g_netplay_cfg.enabled = 1;
        g_netplay_cfg.local_slot = net.local_slot;
        g_netplay_cfg.input_player =
            (net.input_player == 0 || net.input_player == 1)
                ? net.input_player : -1;
        g_netplay_cfg.session_id = net.session_id ? net.session_id : 1u;
        g_netplay_cfg.transport = 0;
        snprintf(g_netplay_cfg.bind_hostport, sizeof(g_netplay_cfg.bind_hostport),
                 "%s", net.bind_hostport);
        snprintf(g_netplay_cfg.peer_hostport, sizeof(g_netplay_cfg.peer_hostport),
                 "%s", net.peer_hostport);
        snes_netplay_apply_env(&g_netplay_cfg);
        if (net.input_delay >= 0 && net.input_delay <= 16)
          g_netplay_cfg.input_delay = net.input_delay;
        g_netplay_caps_ws_extra = 71;
        g_netplay_pending = 1;
        g_netplay_from_lobby = 1;
        host_report_breadcrumb(
            "launcher: rematch netplay slot=%d session=%u bind=%s "
            "peer=%s delay=%d ws_extra=%d",
            net.local_slot, (unsigned)net.session_id,
            net.bind_hostport, net.peer_hostport, net.input_delay,
            g_netplay_caps_ws_extra);
        goto session_reboot;
      }

      /* Offline Play after soft-return — leave lobby and cold-boot solo. */
      g_config.widescreen = 0;
      WriteConfigFile(config_file);
      g_netplay_cfg.enabled = 0;
      g_netplay_pending = 0;
      g_netplay_from_lobby = 0;
      if (g_launcher_hosting_lan || g_launcher_joined_lan)
        (void)LauncherNpLeave(NULL);
      snes_lobby_disconnect();
      free(kRom);
      kRom = NULL;
      host_report_breadcrumb("launcher: offline launch after lobby");
      goto session_reboot;
    }

    fprintf(stderr, "metalwarriors: lobby closed after match; exiting.\n");
    if (g_launcher_hosting_lan || g_launcher_joined_lan)
      (void)LauncherNpLeave(NULL);
    snes_lobby_disconnect();
    free(kRom);
#ifdef __SWITCH__
    SwitchImpl_Exit();
#endif
    SDL_Quit();
    return 0;
  }
#endif

#if defined(RECOMP_LAUNCHER)
  if (g_launcher_hosting_lan || g_launcher_joined_lan)
    (void)LauncherNpLeave(NULL);
#endif
  free(kRom);
#ifdef __SWITCH__
  SwitchImpl_Exit();
#endif
  SDL_Quit();
  return 0;
}

static void RenderDigit(uint8 *dst, size_t pitch, int digit, uint32 color, bool big) {
  static const uint8 kFont[] = {
    0x1c, 0x36, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x36, 0x1c,
    0x18, 0x1c, 0x1e, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7e,
    0x3e, 0x63, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x03, 0x63, 0x7f,
    0x3e, 0x63, 0x60, 0x60, 0x3c, 0x60, 0x60, 0x60, 0x63, 0x3e,
    0x30, 0x38, 0x3c, 0x36, 0x33, 0x7f, 0x30, 0x30, 0x30, 0x78,
    0x7f, 0x03, 0x03, 0x03, 0x3f, 0x60, 0x60, 0x60, 0x63, 0x3e,
    0x1c, 0x06, 0x03, 0x03, 0x3f, 0x63, 0x63, 0x63, 0x63, 0x3e,
    0x7f, 0x63, 0x60, 0x60, 0x30, 0x18, 0x0c, 0x0c, 0x0c, 0x0c,
    0x3e, 0x63, 0x63, 0x63, 0x3e, 0x63, 0x63, 0x63, 0x63, 0x3e,
    0x3e, 0x63, 0x63, 0x63, 0x7e, 0x60, 0x60, 0x60, 0x30, 0x1e,
  };
  const uint8 *p = kFont + digit * 10;
  if (!big) {
    for (int y = 0; y < 10; y++, dst += pitch) {
      int v = *p++;
      for (int x = 0; v; x++, v >>= 1) {
        if (v & 1)
          ((uint32 *)dst)[x] = color;
      }
    }
  } else {
    for (int y = 0; y < 10; y++, dst += pitch * 2) {
      int v = *p++;
      for (int x = 0; v; x++, v >>= 1) {
        if (v & 1) {
          ((uint32 *)dst)[x * 2 + 1] = ((uint32 *)dst)[x * 2] = color;
          ((uint32 *)(dst + pitch))[x * 2 + 1] = ((uint32 *)(dst + pitch))[x * 2] = color;
        }
      }
    }
  }
}


static void RenderNumber(uint8 *dst, size_t pitch, int n, uint8 big) {
  char buf[32], *s;
  int i;
  sprintf(buf, "%d", n);
  for (s = buf, i = 2 * 4; *s; s++, i += 8 * 4)
    RenderDigit(dst + ((pitch + i + 4) << big), pitch, *s - '0', 0x404040, big);
  for (s = buf, i = 2 * 4; *s; s++, i += 8 * 4)
    RenderDigit(dst + (i << big), pitch, *s - '0', 0xffffff, big);
}

static void HandleCommand(uint32 j, bool pressed) {
  static const uint8 kKbdRemap[] = { 4, 5, 6, 7, 2, 3, 8, 0, 9, 1, 10, 11 };
  if (j < kKeys_Controls)
    return;

  if (j <= kKeys_Controls_Last) {
    uint32 m = 1 << kKbdRemap[j - kKeys_Controls];
    g_input_state = pressed ? (g_input_state | m) : (g_input_state & ~m);
    return;
  }

  if (j <= kKeys_ControlsP2_Last) {
    uint32 m = 0x1000 << kKbdRemap[j - kKeys_ControlsP2];
    g_input_state = pressed ? (g_input_state | m) : (g_input_state & ~m);
    return;
  }

  if (j == kKeys_Turbo) {
    g_turbo = pressed;
    return;
  }

  if (!pressed)
    return;
  if (j <= kKeys_Load_Last) {
    if (!snes_netplay_request_load(j - kKeys_Load))
      RtlSaveLoad(kSaveLoad_Load, j - kKeys_Load);
  } else if (j <= kKeys_Save_Last) {
    if (!snes_netplay_request_save(j - kKeys_Save))
      RtlSaveLoad(kSaveLoad_Save, j - kKeys_Save);
  } else {
    switch (j) {
    case kKeys_Fullscreen:
      g_win_flags ^= SDL_WINDOW_FULLSCREEN_DESKTOP;
      SDL_SetWindowFullscreen(g_window, g_win_flags & SDL_WINDOW_FULLSCREEN_DESKTOP);
      g_cursor = !g_cursor;
      SDL_ShowCursor(g_cursor);
      break;
    case kKeys_Reset:
      RtlReset(1);
      break;
    case kKeys_Pause: g_paused = !g_paused; break;
    case kKeys_PauseDimmed:
      g_paused = !g_paused;
      // SDL_RenderPresent may not be called more than once per frame.
      // Seems to work on Windows still. Temporary measure until it's fixed.
#ifdef _WIN32
      if (g_paused) {
        SDL_SetRenderDrawBlendMode(g_renderer, SDL_BLENDMODE_BLEND);
        SDL_SetRenderDrawColor(g_renderer, 0, 0, 0, 159);
        SDL_RenderFillRect(g_renderer, NULL);
        SDL_RenderPresent(g_renderer);
      }
#endif
      break;
    case kKeys_WindowBigger: ChangeWindowScale(1); break;
    case kKeys_WindowSmaller: ChangeWindowScale(-1); break;
    case kKeys_DisplayPerf: g_display_perf ^= 1; break;
    case kKeys_ToggleRenderer:
      g_ppu_render_flags ^= kPpuRenderFlags_NewRenderer;
      printf("New renderer = %x\n", g_ppu_render_flags & kPpuRenderFlags_NewRenderer);
      g_new_ppu = (g_ppu_render_flags & kPpuRenderFlags_NewRenderer) != 0;
      break;
    case kKeys_VolumeUp:
    case kKeys_VolumeDown: HandleVolumeAdjustment(j == kKeys_VolumeUp ? 1 : -1); break;
    default: assert(0);
    }
  }
}

static void HandleInput(int keyCode, int keyMod, bool pressed) {
  int j = FindCmdForSdlKey(keyCode, (SDL_Keymod)keyMod);
  if (j != 0)
    HandleCommand(j, pressed);
}

static uint32 GetActiveControllers() {
  uint32 ctrl = g_config.has_keyboard_controls;
  ctrl |= g_gamepad[0].joystick_id != -1 ? 1 : 0;
  ctrl |= g_gamepad[1].joystick_id != -1 ? 2 : 0;
  return ctrl << 30;
}

static void RefreshKeybindControllerBits(void) {
  /* keybinds bit layout -> kKeys_Controls index. Idempotent set/clear. */
  const uint8_t *keys = SDL_GetKeyboardState(NULL);
  uint16_t kb_p1 = keybinds_read_player(keys, 1);
  uint16_t kb_p2 = keybinds_read_player(keys, 2);
  static const uint8 kKb2CtrlsIdx[12] = { 7, 6, 5, 4, 9, 8, 3, 11, 2, 10, 1, 0 };
  for (int i = 0; i < 12; i++) {
    HandleCommand(kKeys_Controls   + i, (kb_p1 >> kKb2CtrlsIdx[i]) & 1);
    HandleCommand(kKeys_ControlsP2 + i, (kb_p2 >> kKb2CtrlsIdx[i]) & 1);
  }
}

/*
 * Auto input_player for netplay. Offline P2 assignment must not steal the
 * sample device from a remote keyboard (or park the host's only pad off-slot).
 *
 * Guest (local_slot 1): always wrap device 0 (P1 keyboard/pad) into sim P2,
 * unless both pads are live (same-PC dual seat → sample seat 1).
 * Host (local_slot 0): device 0 if present; else device 1 when the only pad
 * was saved as offline P2 — still one local human for the host net slot.
 */
static int ResolveNetplayInputPlayer(int local_slot) {
  const int pad0 = (g_gamepad[0].joystick_id != -1);
  const int pad1 = (g_gamepad[1].joystick_id != -1);
  const int kbd0 = (g_config.has_keyboard_controls & 1) != 0;
  const int kbd1 = (g_config.has_keyboard_controls & 2) != 0;

  if (local_slot == 1) {
    /* Guest: ignore a lone P2 pad left over from host/offline config (same-PC
     * or shared ini) so P1 keyboard still wraps into sim P2. */
    if (pad0 && pad1)
      return 1;
    if (!pad0 && !pad1 && !kbd0 && kbd1)
      return 1; /* keyboard-only, binds only on offline P2 */
    return 0;
  }
  if (pad0)
    return 0;
  if (pad1)
    return 1;
  if (!kbd0 && kbd1)
    return 1;
  return 0;
}

/*
 * MotK capture_local_human_pad: sample the host device selected by
 * input_player (not lobby local_slot). recomp-net maps that blob onto
 * local_slot (host→sim P1, guest→sim P2). Exclusive — no all-device merge.
 */
static uint16_t CaptureLocalNetplayButtons(void) {
  int idx = snes_netplay_input_player();
  uint32 raw;
  if (idx < 0 || idx > 1) idx = 0;
  raw = g_input_state | g_pad_buttons |
        g_gamepad[0].axis_buttons | ((uint32)g_gamepad[1].axis_buttons << 12);
  /* Scripted/headless netplay drives the same exclusive local device sample
   * as physical input, so deterministic two-instance integration tests can
   * enter and exercise an actual match. */
  raw |= TickScript();
  if (idx == 1)
    return (uint16_t)((raw >> 12) & 0x0FFFu);
  return (uint16_t)(raw & 0x0FFFu);
}

/* Send BYE and, when the match started from the lobby, request soft-return. */
static void netplay_soft_exit(const char *origin) {
  snes_netplay_shutdown();
#if defined(RECOMP_LAUNCHER)
  if (g_netplay_from_lobby) {
    fprintf(stderr, "metalwarriors: netplay ended (%s) — returning to lobby\n",
            origin ? origin : "?");
    host_report_breadcrumb("netplay: ended (%s) — returning to lobby",
                           origin ? origin : "?");
    snes_netplay_request_return_to_lobby();
  }
#else
  (void)origin;
#endif
}

/*
 * MotK netplay_barrier_admit: stage + poll until INPUT_CONFIRM hash agrees.
 * Returns 1 if admitted (caller owes RtlRunFrame + finish_frame), 0 if the
 * session ended or the caller should skip the sim tick (*running may clear).
 */
static int NetplayBarrierAdmit(bool *running) {
  static int desync_logged = 0;
  if (!snes_netplay_active()) return 0;

  for (;;) {
    uint32_t dt = 0, lh = 0, rh = 0;
    SDL_Event ev;

    if (snes_netplay_peer_disconnected(1500u)) {
      netplay_soft_exit("peer_disconnect");
      desync_logged = 0;
      if (running && snes_netplay_return_to_lobby_requested())
        *running = false;
      return 0;
    }
    if (snes_netplay_input_desync(&dt, &lh, &rh)) {
      if (!desync_logged) {
        fprintf(stderr,
                "snes_netplay: INPUT desync tick=%u local=%08x remote=%08x — stalled\n",
                (unsigned)dt, (unsigned)lh, (unsigned)rh);
        desync_logged = 1;
      }
      SDL_Delay(16);
      while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_QUIT) {
          netplay_soft_exit("sdl_quit");
          if (running) *running = false;
          return 0;
        }
        if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) {
          netplay_soft_exit("escape");
          desync_logged = 0;
          if (running && snes_netplay_return_to_lobby_requested())
            *running = false;
          return 0;
        }
      }
      continue;
    }

    RefreshKeybindControllerBits();
    if (snes_netplay_needs_local_sample())
      snes_netplay_stage_local(CaptureLocalNetplayButtons());
    if (snes_netplay_poll_admit()) {
      desync_logged = 0;
      return 1;
    }

    while (SDL_PollEvent(&ev)) {
      if (ev.type == SDL_QUIT) {
        netplay_soft_exit("sdl_quit");
        if (running) *running = false;
        return 0;
      }
      if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) {
        netplay_soft_exit("escape");
        if (running && snes_netplay_return_to_lobby_requested())
          *running = false;
        return 0;
      }
      if (ev.type == SDL_CONTROLLERDEVICEADDED)
        OpenOneGamepad(ev.cdevice.which);
      else if (ev.type == SDL_CONTROLLERDEVICEREMOVED) {
        GamepadInfo *gi = GetGamepadInfo(ev.cdevice.which);
        if (gi) {
          memset(gi, 0, sizeof(GamepadInfo));
          gi->joystick_id = -1;
        }
      } else if (ev.type == SDL_CONTROLLERAXISMOTION) {
        GamepadInfo *gi = GetGamepadInfo(ev.caxis.which);
        if (gi)
          HandleGamepadAxisInput(gi, ev.caxis.axis, ev.caxis.value);
      } else if (ev.type == SDL_CONTROLLERBUTTONDOWN ||
                 ev.type == SDL_CONTROLLERBUTTONUP) {
        GamepadInfo *gi = GetGamepadInfo(ev.cbutton.which);
        if (gi) {
          int b = RemapSdlButton(ev.cbutton.button);
          if (b >= 0)
            HandleGamepadInput(gi, b, ev.type == SDL_CONTROLLERBUTTONDOWN);
        }
      }
    }
    SDL_Delay(1);
  }
}

static void OpenOneGamepad(int i) {
  if (SDL_IsGameController(i)) {
    SDL_GameController *controller = SDL_GameControllerOpen(i);
    if (!controller) {
      fprintf(stderr, "Could not open gamepad %d: %s\n", i, SDL_GetError());
      return;
    }

    uint32 joystick_id = SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controller));
    if (GetGamepadInfo(joystick_id))
      return;

    char guid[40] = {0};
    SDL_JoystickGUID g = SDL_JoystickGetGUID(SDL_GameControllerGetJoystick(controller));
    SDL_JoystickGetGUIDString(g, guid, sizeof(guid));

    int found_idx = -1;
    /* Prefer the launcher-saved GUID for each player slot. */
    for (int j = 0; j < 2; j++) {
      if (!g_config.enable_gamepad[j] || g_gamepad[j].joystick_id != -1)
        continue;
      if (g_config.input_device[j][0] &&
          !StringEqualsNoCase(g_config.input_device[j], "none") &&
          !StringEqualsNoCase(g_config.input_device[j], "keyboard") &&
          StringEqualsNoCase(g_config.input_device[j], guid)) {
        found_idx = j;
        break;
      }
    }
    if (found_idx < 0) {
      uint8 scan_order[3] = { SDL_GameControllerGetPlayerIndex(controller), 0, 1 };
      for (int k = 0; k < 3; k++) {
        uint8 j = scan_order[k];
        if (j < 2 && g_config.enable_gamepad[j] &&
            (k == 0 || g_gamepad[j].joystick_id == -1)) {
          /* Skip slots that already have a different saved GUID waiting. */
          if (g_config.input_device[j][0] &&
              !StringEqualsNoCase(g_config.input_device[j], "none") &&
              !StringEqualsNoCase(g_config.input_device[j], "keyboard") &&
              !StringEqualsNoCase(g_config.input_device[j], guid))
            continue;
          found_idx = j;
          break;
        }
      }
    }

    printf("Found controller '%s' assigning to player %d\n", SDL_GameControllerName(controller), found_idx + 1);
    if (found_idx >= 0) {
      GamepadInfo *gi = &g_gamepad[found_idx];
      memset(gi, 0, sizeof(GamepadInfo));
      gi->index = found_idx;
      gi->joystick_id = joystick_id;
    }
  }
}

static int RemapSdlButton(int button) {
  switch (button) {
  case SDL_CONTROLLER_BUTTON_A: return kGamepadBtn_A;
  case SDL_CONTROLLER_BUTTON_B: return kGamepadBtn_B;
  case SDL_CONTROLLER_BUTTON_X: return kGamepadBtn_X;
  case SDL_CONTROLLER_BUTTON_Y: return kGamepadBtn_Y;
  case SDL_CONTROLLER_BUTTON_BACK: return kGamepadBtn_Back;
  case SDL_CONTROLLER_BUTTON_GUIDE: return kGamepadBtn_Guide;
  case SDL_CONTROLLER_BUTTON_START: return kGamepadBtn_Start;
  case SDL_CONTROLLER_BUTTON_LEFTSTICK: return kGamepadBtn_L3;
  case SDL_CONTROLLER_BUTTON_RIGHTSTICK: return kGamepadBtn_R3;
  case SDL_CONTROLLER_BUTTON_LEFTSHOULDER: return kGamepadBtn_L1;
  case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER: return kGamepadBtn_R1;
  case SDL_CONTROLLER_BUTTON_DPAD_UP: return kGamepadBtn_DpadUp;
  case SDL_CONTROLLER_BUTTON_DPAD_DOWN: return kGamepadBtn_DpadDown;
  case SDL_CONTROLLER_BUTTON_DPAD_LEFT: return kGamepadBtn_DpadLeft;
  case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: return kGamepadBtn_DpadRight;
  default: return -1;
  }
}

/* Set/clear a SNES controller bit from a gamepad source. Mirrors
 * HandleCommand's kKeys_Controls / kKeys_ControlsP2 logic but writes
 * to g_pad_buttons so the per-frame keyboard polling can't clobber
 * gamepad-set bits. Non-controller commands (system shortcuts bound
 * via config.ini [GamepadMap]) fall through to HandleCommand so things
 * like state save/load on a gamepad button still work. */
static void SetPadButtonOrFallthrough(uint32 j, bool pressed) {
  static const uint8 kKbdRemap[] = { 4, 5, 6, 7, 2, 3, 8, 0, 9, 1, 10, 11 };
  if (j >= kKeys_Controls && j <= kKeys_Controls_Last) {
    uint32 m = 1u << kKbdRemap[j - kKeys_Controls];
    g_pad_buttons = pressed ? (g_pad_buttons | m) : (g_pad_buttons & ~m);
    return;
  }
  if (j >= kKeys_ControlsP2 && j <= kKeys_ControlsP2_Last) {
    uint32 m = 0x1000u << kKbdRemap[j - kKeys_ControlsP2];
    g_pad_buttons = pressed ? (g_pad_buttons | m) : (g_pad_buttons & ~m);
    return;
  }
  HandleCommand(j, pressed);
}

static void HandleGamepadInput(GamepadInfo *gi, int button, bool pressed) {
  if (!!(gi->modifiers & (1 << button)) == pressed)
    return;
  gi->modifiers ^= 1 << button;
  if (pressed)
    gi->last_cmd[button] = FindCmdForGamepadButton(button + gi->index * kGamepadBtn_Count, gi->modifiers);
  if (gi->last_cmd[button] != 0)
    SetPadButtonOrFallthrough(gi->last_cmd[button], pressed);
}

static void HandleVolumeAdjustment(int volume_adjustment) {
#if SYSTEM_VOLUME_MIXER_AVAILABLE
  int current_volume = GetApplicationVolume();
  int new_volume = IntMin(IntMax(0, current_volume + volume_adjustment * 5), 100);
  SetApplicationVolume(new_volume);
  printf("[System Volume]=%i\n", new_volume);
#else
  g_sdl_audio_mixer_volume = IntMin(IntMax(0, g_sdl_audio_mixer_volume + volume_adjustment * (SDL_MIX_MAXVOLUME >> 4)), SDL_MIX_MAXVOLUME);
  printf("[SDL mixer volume]=%i\n", g_sdl_audio_mixer_volume);
#endif
}

// Approximates atan2(y, x) normalized to the [0,4) range
// with a maximum error of 0.1620 degrees
// normalized_atan(x) ~ (b x + x^2) / (1 + 2 b x + x^2)
static float ApproximateAtan2(float y, float x) {
  uint32 sign_mask = 0x80000000;
  float b = 0.596227f;
  // Extract the sign bits
  uint32 ux_s = sign_mask & *(uint32 *)&x;
  uint32 uy_s = sign_mask & *(uint32 *)&y;
  // Determine the quadrant offset
  float q = (float)((~ux_s & uy_s) >> 29 | ux_s >> 30);
  // Calculate the arctangent in the first quadrant
  float bxy_a = b * x * y;
  if (bxy_a < 0.0f) bxy_a = -bxy_a;  // avoid fabs
  float num = bxy_a + y * y;
  float atan_1q = num / (x * x + bxy_a + num + 0.000001f);
  // Translate it to the proper quadrant
  uint32_t uatan_2q = (ux_s ^ uy_s) | *(uint32 *)&atan_1q;
  return q + *(float *)&uatan_2q;
}

static void HandleGamepadAxisInput(GamepadInfo *gi, int axis, Sint16 value) {
  if (axis == SDL_CONTROLLER_AXIS_LEFTX || axis == SDL_CONTROLLER_AXIS_LEFTY) {
    *(axis == SDL_CONTROLLER_AXIS_LEFTX ? &gi->last_axis_x : &gi->last_axis_y) = value;
    int buttons = 0;
    if (gi->last_axis_x * gi->last_axis_x + gi->last_axis_y * gi->last_axis_y >= g_config.gamepad_deadzone * g_config.gamepad_deadzone) {
      // in the non deadzone part, divide the circle into eight 45 degree
      // segments rotated by 22.5 degrees that control which direction to move.
      // todo: do this without floats?
      static const uint8 kSegmentToButtons[8] = {
        1 << 4,           // 0 = up
        1 << 4 | 1 << 7,  // 1 = up, right
        1 << 7,           // 2 = right
        1 << 7 | 1 << 5,  // 3 = right, down
        1 << 5,           // 4 = down
        1 << 5 | 1 << 6,  // 5 = down, left
        1 << 6,           // 6 = left
        1 << 6 | 1 << 4,  // 7 = left, up
      };
      uint8 angle = (uint8)(int)(ApproximateAtan2(gi->last_axis_y, gi->last_axis_x) * 64.0f + 0.5f);
      buttons = kSegmentToButtons[(uint8)(angle + 16 + 64) >> 5];
    }
    gi->axis_buttons = buttons;
  } else if ((axis == SDL_CONTROLLER_AXIS_TRIGGERLEFT || axis == SDL_CONTROLLER_AXIS_TRIGGERRIGHT)) {
    if (value < 12000 || value >= 16000)  // hysteresis
      HandleGamepadInput(gi, axis == SDL_CONTROLLER_AXIS_TRIGGERLEFT ? kGamepadBtn_L2 : kGamepadBtn_R2, value >= 12000);
  }
}

/* Default config.ini content written next to the executable when no
 * config.ini exists there on launch. Mirrors the repo-root config.ini
 * but stripped of dev-only comments; keep them in lock-step when
 * adding new tunables that should be user-discoverable. The
 * [GamepadMap] section gives a plugged-in Xbox controller working
 * defaults out of the box. */
static const char kDefaultConfigIniContent[] =
  "[General]\n"
  "# Automatically save state on quit and reload on start\n"
  "Autosave = 0\n"
  "\n"
  "# Disable the SDL_Delay that happens each frame (slightly better\n"
  "# perf if your display is set to exactly 60hz)\n"
  "DisableFrameDelay = 0\n"
  "\n"
  "# 0 = show the recomp-ui launcher by default. 1 = boot straight to the game.\n"
  "# Force the launcher with --launcher; skip with --no-launcher.\n"
  "SkipLauncher = 0\n"
  "\n"
  "[Graphics]\n"
  "# Window size (Auto or WidthxHeight)\n"
  "WindowSize = Auto\n"
  "\n"
  "# Fullscreen mode (0=windowed, 1=desktop fullscreen, 2=fullscreen w/mode change)\n"
  "Fullscreen = 0\n"
  "\n"
  "# Window scale (1=100%, 2=200%, 3=300%, etc.)\n"
  "WindowScale = 3\n"
  "\n"
  "# Use the optimized SNES PPU implementation\n"
  "NewRenderer = 1\n"
  "\n"
  "# Don't keep the aspect ratio\n"
  "IgnoreAspectRatio = 0\n"
  "\n"
  "# Remove the sprite limits per scan line\n"
  "NoSpriteLimits = 1\n"
  "\n"
  "# Extra columns per side for 16:9 (0 = authentic 4:3, ~71 ≈ 16:9 at 224p).\n"
  "# Title/dialogue stay centered; Mode-1 gameplay expands into the margins.\n"
  "Widescreen = 0\n"
  "\n"
  "[Sound]\n"
  "EnableAudio = 1\n"
  "AudioFreq = 32040\n"
  "AudioChannels = 2\n"
  "AudioSamples = 512\n"
  "\n"
  "[KeyMap]\n"
  "# This section is for system-level shortcuts (save/load state,\n"
  "# fullscreen, pause, etc.). The 12 SNES controller buttons live\n"
  "# in keybinds.ini next to the executable.\n"
  "Fullscreen = Alt+Return\n"
  "Reset = Ctrl+r\n"
  "Pause = Shift+p\n"
  "PauseDimmed = p\n"
  "Turbo = Tab\n"
  "WindowBigger = Ctrl+Up\n"
  "WindowSmaller = Ctrl+Down\n"
  "VolumeUp = Shift+=\n"
  "VolumeDown = Shift+-\n"
  "Load =      F1,     F2,     F3,     F4,     F5,     F6,     F7,     F8,     F9,     F10\n"
  "Save = Shift+F1,Shift+F2,Shift+F3,Shift+F4,Shift+F5,Shift+F6,Shift+F7,Shift+F8,Shift+F9,Shift+F10\n"
  "\n"
  "[GamepadMap]\n"
  "# Enable each player's gamepad slot. SDL_GameController-compatible\n"
  "# controllers (Xbox, PlayStation, Switch Pro, etc.) auto-detect\n"
  "# when plugged in. Set to false to force keyboard-only.\n"
  "EnableGamepad1 = true\n"
  "EnableGamepad2 = true\n"
  "\n"
  "# Launcher device selection (none / keyboard / SDL joystick GUID).\n"
  "# Written by the dashboard CONTROLLERS dropdowns; survives Refresh\n"
  "# and relaunch. Leave empty to derive from EnableGamepad*.\n"
  "InputDevice1 =\n"
  "InputDevice2 =\n"
  "\n"
  "# Analog stick deadzone (raw axis units, range 1-32767).\n"
  "# 10000 ~ 30% deflection. Increase if idle stick drifts trigger movement.\n"
  "GamepadDeadzone = 10000\n"
  "\n"
  "# Default Xbox-layout mapping. Order matches kKeys_Controls:\n"
  "#   Up, Down, Left, Right, Select, Start, A, B, X, Y, L, R\n"
  "# Edit + restart to rebind. Shoulder = L1/Lb (top), trigger = L2.\n"
  "Controls =   DpadUp, DpadDown, DpadLeft, DpadRight, Back, Start, B, A, Y, X, Lb, Rb\n"
  "ControlsP2 = DpadUp, DpadDown, DpadLeft, DpadRight, Back, Start, B, A, Y, X, Lb, Rb\n";

/* Ensure config.ini exists next to the executable (cwd after
 * snesrecomp_anchor_to_exe_dir). First launch from a clean release
 * directory writes the default so the config the user can edit is
 * always sitting right beside the exe. */
static void EnsureConfigIni(void) {
  FILE *f = fopen("config.ini", "rb");
  if (f) {
    fclose(f);
    return;
  }
  f = fopen("config.ini", "w");
  if (!f) {
    fprintf(stderr, "Warning: could not write default config.ini\n");
    return;
  }
  fputs(kDefaultConfigIniContent, f);
  fclose(f);
  printf("[config.ini] Generated default config next to the executable\n");
}

#if defined(RECOMP_LAUNCHER)
static int FindLauncherAssetsDir(char *out, size_t cap) {
  /* recomp-ui resolves its staged assets relative to SDL_GetBasePath(). The
   * C ABI keeps this argument for hosts with custom layouts, so pass the
   * executable directory when available and a harmless cwd fallback. */
  if (snesrecomp_exe_dir_path(".", out, cap)) return 1;
  snprintf(out, cap, ".");
  return 1;
}

static void LauncherSettingsFromConfig(RecompLauncherCSettings *s) {
  memset(s, 0, sizeof(*s));
  s->output_method = g_config.output_method;
  s->window_scale = g_config.window_scale ? g_config.window_scale : 3;
  s->fullscreen = g_config.fullscreen;
  s->ignore_aspect = g_config.ignore_aspect_ratio;
  s->linear_filter = g_config.linear_filtering;
  /* Toggle hidden; netplay forces 71 at session start, offline forces 0. */
  s->widescreen = g_config.widescreen > 0;
  s->widescreen_hud = 1;
  s->enable_audio = g_config.enable_audio;
  s->audio_freq = g_config.audio_freq ? g_config.audio_freq : kDefaultFreq;
  s->volume = 100;
  for (int p = 0; p < 2; p++) {
    const char *dev = g_config.input_device[p];
    if (dev[0]) {
      if (StringEqualsNoCase(dev, "none"))
        s->player_src[p] = 0;
      else if (StringEqualsNoCase(dev, "keyboard"))
        s->player_src[p] = 1;
      else
        s->player_src[p] = 2; /* GUID / gamepad */
    } else {
      int kbd = (g_config.has_keyboard_controls >> p) & 1;
      if (g_config.enable_gamepad[p])
        s->player_src[p] = 2; /* gamepad */
      else if (kbd)
        s->player_src[p] = 1; /* keyboard */
      else
        s->player_src[p] = 0; /* none */
    }
    /* Config stores raw axis units; launcher UI uses 0..100%. */
    int dz = g_config.gamepad_deadzone;
    if (dz <= 0) dz = 10000;
    s->deadzone[p] = (dz * 100 + 16383) / 32767;
    if (s->deadzone[p] < 1) s->deadzone[p] = 1;
    if (s->deadzone[p] > 100) s->deadzone[p] = 100;
  }
  s->skip_launcher = g_config.skip_launcher;
  s->msu1_enabled = g_config.msu1_enabled;
  snprintf(s->msu1_dir, sizeof(s->msu1_dir), "%s", g_config.msu1_dir);
  snprintf(s->netplay_player_name, sizeof(s->netplay_player_name), "%s",
           g_config.netplay_player_name);
}

static void ConfigFromLauncherSettings(const RecompLauncherCSettings *s) {
  g_config.output_method = (uint8)s->output_method;
  g_config.window_scale = (uint8)(s->window_scale > 0 ? s->window_scale : 3);
  g_config.fullscreen = (uint8)s->fullscreen;
  g_config.ignore_aspect_ratio = s->ignore_aspect != 0;
  g_config.linear_filtering = s->linear_filter != 0;
  /* WS applied by caller from netplay vs offline (toggle is hidden). */
  if (s->widescreen)
    g_config.widescreen = g_config.widescreen ? g_config.widescreen : 71;
  else
    g_config.widescreen = 0;
  g_config.enable_audio = s->enable_audio != 0;
  if (s->audio_freq >= 11025 && s->audio_freq <= 48000)
    g_config.audio_freq = (uint16)s->audio_freq;
  g_config.skip_launcher = s->skip_launcher != 0;
  g_config.msu1_enabled = s->msu1_enabled != 0;
  snprintf(g_config.msu1_dir, sizeof(g_config.msu1_dir), "%s", s->msu1_dir);
  snprintf(g_config.netplay_player_name, sizeof(g_config.netplay_player_name),
           "%s", s->netplay_player_name);

  g_config.has_keyboard_controls = 0;
  for (int p = 0; p < 2; p++) {
    if (s->player_src[p] == 0) {
      snprintf(g_config.input_device[p], sizeof(g_config.input_device[p]), "none");
    } else if (s->player_src[p] == 1) {
      snprintf(g_config.input_device[p], sizeof(g_config.input_device[p]), "keyboard");
    } else {
      g_config.input_device[p][0] = '\0';
    }

    if (s->player_src[p] == 1) { /* keyboard */
      g_config.enable_gamepad[p] = false;
      g_config.has_keyboard_controls |= (uint8)(1u << p);
    } else if (s->player_src[p] == 2) { /* gamepad */
      g_config.enable_gamepad[p] = true;
    } else {
      g_config.enable_gamepad[p] = false;
    }
  }
  /* Single shared deadzone field today — use P1's slider. */
  int pct = s->deadzone[0];
  if (pct < 1) pct = 1;
  if (pct > 100) pct = 100;
  g_config.gamepad_deadzone = (pct * 32767 + 50) / 100;
}
#endif /* RECOMP_LAUNCHER */
