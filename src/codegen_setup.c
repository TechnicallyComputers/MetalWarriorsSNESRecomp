/* Spawn snesrecomp_cli.py generate from the recomp-ui setup wizard. */

#include "codegen_setup.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#  include <windows.h>
#else
#  include <sys/stat.h>
#  include <sys/wait.h>
#  include <unistd.h>
#  include <fcntl.h>
#  include <errno.h>
#  include <spawn.h>
extern char** environ;
#endif

/* Keep in sync with README / tools/regen.sh / main.c fingerprints. */
static const char kExpectedCrc32[] = "f2ab92d4";
static const char kExpectedSha256[] =
    "0d7f875877fe856066cfb39b4ecdbbe7d48393a75770720876c94419f809bb1c";

static char g_project_root[1024];
static char g_cli_path[1100];
static char g_python[512];
static int g_ready = 0;

static int path_is_file(const char* path) {
#if defined(_WIN32)
    DWORD attr = GetFileAttributesA(path);
    return attr != INVALID_FILE_ATTRIBUTES &&
           (attr & FILE_ATTRIBUTE_DIRECTORY) == 0;
#else
    struct stat st;
    return stat(path, &st) == 0 && S_ISREG(st.st_mode);
#endif
}

static int path_is_dir(const char* path) {
#if defined(_WIN32)
    DWORD attr = GetFileAttributesA(path);
    return attr != INVALID_FILE_ATTRIBUTES &&
           (attr & FILE_ATTRIBUTE_DIRECTORY) != 0;
#else
    struct stat st;
    return stat(path, &st) == 0 && S_ISDIR(st.st_mode);
#endif
}

static int join_path(char* out, size_t cap, const char* a, const char* b) {
    size_t na = strlen(a);
    int need_slash = na > 0 && a[na - 1] != '/' && a[na - 1] != '\\';
    int n = snprintf(out, cap, "%s%s%s", a, need_slash ? "/" : "", b);
    return n > 0 && (size_t)n < cap;
}

static int looks_like_project_root(const char* root) {
    char cli[1100], cfg[1100];
    if (!join_path(cli, sizeof(cli), root, "snesrecomp/snesrecomp_cli.py"))
        return 0;
    if (!join_path(cfg, sizeof(cfg), root, "recomp/bank00.cfg"))
        return 0;
    return path_is_file(cli) && path_is_file(cfg);
}

static int dirname_copy(char* out, size_t cap, const char* path) {
    size_t n = strlen(path);
    while (n > 0 && (path[n - 1] == '/' || path[n - 1] == '\\'))
        --n;
    while (n > 0 && path[n - 1] != '/' && path[n - 1] != '\\')
        --n;
    while (n > 0 && (path[n - 1] == '/' || path[n - 1] == '\\'))
        --n;
    if (n == 0) {
        if (cap < 2) return 0;
        out[0] = '.';
        out[1] = '\0';
        return 1;
    }
    if (n >= cap) return 0;
    memcpy(out, path, n);
    out[n] = '\0';
    return 1;
}

static int find_python(char* out, size_t cap) {
    const char* env = getenv("PYTHON");
    if (env && env[0] && path_is_file(env)) {
        snprintf(out, cap, "%s", env);
        return 1;
    }
#if defined(_WIN32)
    const char* candidates[] = { "python.exe", "python3.exe", "py.exe" };
#else
    const char* candidates[] = { "python3", "python" };
#endif
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
#if defined(_WIN32)
        char cmd[640];
        snprintf(cmd, sizeof(cmd), "where %s >nul 2>nul", candidates[i]);
        if (system(cmd) == 0) {
            snprintf(out, cap, "%s", candidates[i]);
            return 1;
        }
#else
        char cmd[640];
        snprintf(cmd, sizeof(cmd), "command -v %s >/dev/null 2>&1",
                 candidates[i]);
        if (system(cmd) == 0) {
            snprintf(out, cap, "%s", candidates[i]);
            return 1;
        }
#endif
    }
    return 0;
}

static int discover_project_root(char* out, size_t cap) {
    const char* env = getenv("METALWARRIORS_PROJECT_ROOT");
    if (env && env[0] && looks_like_project_root(env)) {
        snprintf(out, cap, "%s", env);
        return 1;
    }

    char start[1024];
#if defined(_WIN32)
    if (!GetCurrentDirectoryA((DWORD)sizeof(start), start))
        start[0] = '\0';
#else
    if (!getcwd(start, sizeof(start)))
        start[0] = '\0';
#endif

    char cur[1024];
    if (start[0])
        snprintf(cur, sizeof(cur), "%s", start);
    else
        snprintf(cur, sizeof(cur), ".");

    for (int i = 0; i < 8; ++i) {
        if (looks_like_project_root(cur)) {
            snprintf(out, cap, "%s", cur);
            return 1;
        }
        char parent[1024];
        if (!dirname_copy(parent, sizeof(parent), cur))
            break;
        if (strcmp(parent, cur) == 0)
            break;
        snprintf(cur, sizeof(cur), "%s", parent);
    }
    return 0;
}

int mw_codegen_sources_missing(void) {
    if (!g_ready && !discover_project_root(g_project_root, sizeof(g_project_root)))
        return 0;
    char dispatch[1100];
    if (!join_path(dispatch, sizeof(dispatch), g_project_root,
                   "src/gen/dispatch_v2.c"))
        return 1;
    return !path_is_file(dispatch);
}

static int json_get_string(const char* line, const char* key, char* out,
                           size_t out_cap) {
    char pattern[96];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    const char* p = strstr(line, pattern);
    if (!p) return 0;
    p += strlen(pattern);
    size_t i = 0;
    while (*p && *p != '"' && i + 1 < out_cap) {
        if (*p == '\\' && p[1]) {
            ++p;
            out[i++] = *p++;
            continue;
        }
        out[i++] = *p++;
    }
    out[i] = '\0';
    return i > 0;
}

static int json_get_number(const char* line, const char* key, double* out) {
    char pattern[96];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char* p = strstr(line, pattern);
    if (!p) return 0;
    p += strlen(pattern);
    while (*p == ' ') ++p;
    char* end = NULL;
    double v = strtod(p, &end);
    if (end == p) return 0;
    *out = v;
    return 1;
}

static void handle_progress_line(const char* line,
                                 RecompLauncherCPrepareProgressFn on_progress,
                                 void* progress_ctx) {
    if (!line || line[0] != '{' || !on_progress) return;
    char event[64] = "";
    json_get_string(line, "event", event, sizeof(event));
    if (strcmp(event, "phase") == 0) {
        char message[240] = "";
        char phase[64] = "";
        double pct = -1.0;
        json_get_string(line, "message", message, sizeof(message));
        json_get_string(line, "phase", phase, sizeof(phase));
        if (!json_get_number(line, "pct", &pct))
            pct = -1.0;
        if (!message[0] && phase[0])
            snprintf(message, sizeof(message), "%s", phase);
        on_progress(progress_ctx, (float)pct,
                    message[0] ? message : NULL);
    } else if (strcmp(event, "log") == 0) {
        char message[240] = "";
        if (json_get_string(line, "message", message, sizeof(message)))
            on_progress(progress_ctx, -1.0f, message);
    } else if (strcmp(event, "error") == 0) {
        char message[240] = "";
        if (json_get_string(line, "message", message, sizeof(message)))
            on_progress(progress_ctx, -1.0f, message);
    }
}

#if defined(_WIN32)
static int run_generate_win(const char* rom,
                            RecompLauncherCPrepareProgressFn on_progress,
                            void* progress_ctx, char* err_msg, size_t err_cap) {
    char cmdline[4096];
    snprintf(cmdline, sizeof(cmdline),
             "\"%s\" \"%s\" generate --project-root \"%s\" --rom \"%s\" "
             "--cfg-dir recomp --out-dir src/gen --funcs-h recomp/funcs.h "
             "--cfg-roots --expected-crc32 %s --expected-sha256 %s "
             "--json-progress",
             g_python, g_cli_path, g_project_root, rom, kExpectedCrc32,
             kExpectedSha256);

    SECURITY_ATTRIBUTES sa;
    memset(&sa, 0, sizeof(sa));
    sa.nLength = sizeof(sa);
    sa.bInheritHandle = TRUE;
    HANDLE rd = NULL, wr = NULL;
    if (!CreatePipe(&rd, &wr, &sa, 0)) {
        snprintf(err_msg, err_cap, "CreatePipe failed.");
        return 0;
    }
    SetHandleInformation(rd, HANDLE_FLAG_INHERIT, 0);

    STARTUPINFOA si;
    PROCESS_INFORMATION pi;
    memset(&si, 0, sizeof(si));
    memset(&pi, 0, sizeof(pi));
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdOutput = wr;
    si.hStdError = GetStdHandle(STD_ERROR_HANDLE);
    si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);

    char mutable_cmd[4096];
    snprintf(mutable_cmd, sizeof(mutable_cmd), "%s", cmdline);
    if (!CreateProcessA(NULL, mutable_cmd, NULL, NULL, TRUE, 0, NULL,
                        g_project_root, &si, &pi)) {
        CloseHandle(rd);
        CloseHandle(wr);
        snprintf(err_msg, err_cap, "Failed to spawn snesrecomp generate.");
        return 0;
    }
    CloseHandle(wr);

    char buf[512];
    char line[1024];
    size_t line_len = 0;
    DWORD n = 0;
    while (ReadFile(rd, buf, sizeof(buf), &n, NULL) && n > 0) {
        for (DWORD i = 0; i < n; ++i) {
            char c = buf[i];
            if (c == '\r') continue;
            if (c == '\n') {
                line[line_len] = '\0';
                handle_progress_line(line, on_progress, progress_ctx);
                line_len = 0;
                continue;
            }
            if (line_len + 1 < sizeof(line))
                line[line_len++] = c;
        }
    }
    if (line_len) {
        line[line_len] = '\0';
        handle_progress_line(line, on_progress, progress_ctx);
    }
    CloseHandle(rd);
    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD code = 1;
    GetExitCodeProcess(pi.hProcess, &code);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    if (code == 0) return 1;
    if (code == 3)
        snprintf(err_msg, err_cap, "ROM verification failed (wrong dump).");
    else
        snprintf(err_msg, err_cap, "snesrecomp generate failed (exit %lu).",
                 (unsigned long)code);
    return 0;
}
#else
static int run_generate_posix(const char* rom,
                              RecompLauncherCPrepareProgressFn on_progress,
                              void* progress_ctx, char* err_msg,
                              size_t err_cap) {
    int pipefd[2];
    if (pipe(pipefd) != 0) {
        snprintf(err_msg, err_cap, "pipe() failed: %s", strerror(errno));
        return 0;
    }

    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addclose(&actions, pipefd[0]);
    posix_spawn_file_actions_adddup2(&actions, pipefd[1], STDOUT_FILENO);
    posix_spawn_file_actions_addclose(&actions, pipefd[1]);

    char* argv[] = {
        g_python,
        g_cli_path,
        "generate",
        "--project-root", g_project_root,
        "--rom", (char*)rom,
        "--cfg-dir", "recomp",
        "--out-dir", "src/gen",
        "--funcs-h", "recomp/funcs.h",
        "--cfg-roots",
        "--expected-crc32", (char*)kExpectedCrc32,
        "--expected-sha256", (char*)kExpectedSha256,
        "--json-progress",
        NULL
    };

    pid_t pid = 0;
    int rc = posix_spawnp(&pid, g_python, &actions, NULL, argv, environ);
    posix_spawn_file_actions_destroy(&actions);
    close(pipefd[1]);
    if (rc != 0) {
        close(pipefd[0]);
        snprintf(err_msg, err_cap, "Failed to spawn snesrecomp generate: %s",
                 strerror(rc));
        return 0;
    }

    FILE* out = fdopen(pipefd[0], "r");
    if (!out) {
        close(pipefd[0]);
        waitpid(pid, NULL, 0);
        snprintf(err_msg, err_cap, "fdopen failed.");
        return 0;
    }
    char line[1024];
    while (fgets(line, sizeof(line), out)) {
        size_t n = strlen(line);
        while (n && (line[n - 1] == '\n' || line[n - 1] == '\r'))
            line[--n] = '\0';
        handle_progress_line(line, on_progress, progress_ctx);
    }
    fclose(out);

    int status = 0;
    if (waitpid(pid, &status, 0) < 0) {
        snprintf(err_msg, err_cap, "waitpid failed: %s", strerror(errno));
        return 0;
    }
    int code = WIFEXITED(status) ? WEXITSTATUS(status) : 1;
    if (code == 0) return 1;
    if (code == 3)
        snprintf(err_msg, err_cap, "ROM verification failed (wrong dump).");
    else
        snprintf(err_msg, err_cap, "snesrecomp generate failed (exit %d).",
                 code);
    return 0;
}
#endif

static int mw_prepare_generate(const char* source_path, char* out_path,
                               size_t out_cap, char* err_msg, size_t err_cap,
                               RecompLauncherCPrepareProgressFn on_progress,
                               void* progress_ctx) {
    if (!g_ready) {
        snprintf(err_msg, err_cap, "Local codegen tools are not available.");
        return 0;
    }
    if (!source_path || !source_path[0]) {
        snprintf(err_msg, err_cap, "No ROM selected.");
        return 0;
    }
    if (on_progress)
        on_progress(progress_ctx, 0.02f, "Starting snesrecomp generate…");

#if defined(_WIN32)
    if (!run_generate_win(source_path, on_progress, progress_ctx, err_msg,
                          err_cap))
        return 0;
#else
    if (!run_generate_posix(source_path, on_progress, progress_ctx, err_msg,
                            err_cap))
        return 0;
#endif

    snprintf(out_path, out_cap, "%s", source_path);
    if (on_progress)
        on_progress(progress_ctx, 1.0f, "Generate complete");
    return 1;
}

void mw_codegen_setup_apply(RecompLauncherCGameInfo* gi) {
    if (!gi) return;
    g_ready = 0;
    g_project_root[0] = '\0';
    g_cli_path[0] = '\0';
    g_python[0] = '\0';

    if (!discover_project_root(g_project_root, sizeof(g_project_root)))
        return;
    if (!join_path(g_cli_path, sizeof(g_cli_path), g_project_root,
                   "snesrecomp/snesrecomp_cli.py"))
        return;
    if (!path_is_file(g_cli_path))
        return;
    if (!find_python(g_python, sizeof(g_python)))
        return;

    g_ready = 1;
    gi->prepare_with_progress = mw_prepare_generate;
    gi->prepare_use_selected_rom = 1;
    gi->prepare_disc_label = "Generate sources…";
    gi->prepare_section_title = "2. Generate C sources";
    gi->prepare_disc_note =
        "Uses your verified Metal Warriors (USA) ROM with the local snesrecomp "
        "SDK to regenerate src/gen. You must legally own this ROM. After "
        "generate completes, rebuild the game binary to pick up new C before "
        "PLAY uses it.";
    gi->prepare_busy_status = "Generating sources…";
    gi->prepare_success_status =
        "Sources generated. Rebuild the game, then press PLAY.";

    const char* force = getenv("METALWARRIORS_FORCE_SETUP");
    if (mw_codegen_sources_missing() ||
        (force && force[0] && force[0] != '0'))
        gi->needs_setup = 1;
}
