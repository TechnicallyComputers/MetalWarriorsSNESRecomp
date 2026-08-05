#!/usr/bin/env python3
"""Sync recomp/symbols.toml → bank*.cfg managed blocks + mw_symbols.h + catalog.

Metal Warriors progressive naming (partial decomp):

  symbols.toml  ──sync──►  bankXX.cfg  `func` lines (emit=true only)
                        ►  recomp/mw_symbols.h   (all addrs as C macros)
                        ►  docs/SYMBOLS_CATALOG.md

Usage:
  python3 tools/sync_symbols.py           # write outputs
  python3 tools/sync_symbols.py --check   # exit 1 if outputs would change
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib  # type: ignore

ROOT = pathlib.Path(__file__).resolve().parent.parent
SYMBOLS_PATH = ROOT / "recomp" / "symbols.toml"
CFG_DIR = ROOT / "recomp"
HEADER_PATH = ROOT / "recomp" / "mw_symbols.h"
CATALOG_PATH = ROOT / "docs" / "SYMBOLS_CATALOG.md"

BEGIN = "# >>> BEGIN symbols.toml (generated — do not edit)"
END = "# <<< END symbols.toml"
MANAGED_RE = re.compile(
    re.escape(BEGIN) + r".*?" + re.escape(END) + r"\n?",
    re.DOTALL,
)

BANK_BOOTSTRAP = """\
# Metal Warriors (USA) — bank ${bank:02X}
# Hand-authored directives go outside the managed symbols.toml block.
# See docs/SYMBOLS.md for the progressive naming workflow.
bank = {bank:02X}

{BEGIN}
{body}
{END}
"""


def _parse_int(value) -> int:
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        return int(value, 0)
    raise TypeError(f"expected int, got {type(value)!r}")


def load_symbols(path: pathlib.Path) -> dict:
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    funcs = []
    for raw in data.get("func", []):
        bank = _parse_int(raw["bank"])
        pc = _parse_int(raw["pc"])
        funcs.append({
            "bank": bank & 0xFF,
            "pc": pc & 0xFFFF,
            "addr": ((bank & 0xFF) << 16) | (pc & 0xFFFF),
            "name": str(raw["name"]),
            "emit": bool(raw.get("emit", False)),
            "status": str(raw.get("status", "guessed")),
            "note": str(raw.get("note", "")),
            "end": (_parse_int(raw["end"]) & 0xFFFF) if "end" in raw else None,
            "entry_mx": raw.get("entry_mx"),
        })
    sites = []
    for raw in data.get("site", []):
        addr = _parse_int(raw["addr"]) & 0xFFFFFF
        sites.append({
            "addr": addr,
            "name": str(raw["name"]),
            "status": str(raw.get("status", "guessed")),
            "note": str(raw.get("note", "")),
        })
    groups = []
    for raw in data.get("site_group", []):
        addrs = [_parse_int(a) & 0xFFFFFF for a in raw.get("addrs", [])]
        groups.append({
            "name": str(raw["name"]),
            "addrs": addrs,
            "status": str(raw.get("status", "guessed")),
            "note": str(raw.get("note", "")),
        })
    objects = []
    for raw in data.get("object", []):
        objects.append({
            "addr": _parse_int(raw["addr"]) & 0xFFFFFF,
            "name": str(raw["name"]),
            "type": str(raw.get("type", "u8")),
            "status": str(raw.get("status", "guessed")),
            "note": str(raw.get("note", "")),
        })
    structs = {str(s["name"]): {
        "name": str(s["name"]),
        "size": _parse_int(s.get("size", 0)),
        "status": str(s.get("status", "guessed")),
        "note": str(s.get("note", "")),
        "fields": [],
    } for s in data.get("struct", [])}
    for raw in data.get("field", []):
        sname = str(raw["struct"])
        if sname not in structs:
            structs[sname] = {
                "name": sname, "size": 0, "status": "guessed",
                "note": "", "fields": [],
            }
        structs[sname]["fields"].append({
            "offset": _parse_int(raw["offset"]),
            "name": str(raw["name"]),
            "type": str(raw.get("type", "u8")),
            "note": str(raw.get("note", "")),
        })
    for st in structs.values():
        st["fields"].sort(key=lambda f: f["offset"])
    return {
        "funcs": funcs,
        "sites": sites,
        "groups": groups,
        "objects": objects,
        "structs": structs,
    }


def _func_cfg_line(fn: dict) -> str:
    parts = [f"func {fn['name']} {fn['pc']:04X}"]
    if fn["end"] is not None:
        parts.append(f"end:{fn['end']:04X}")
    if fn.get("entry_mx"):
        parts.append(f"entry_mx:{fn['entry_mx']}")
    line = " ".join(parts)
    if fn["note"]:
        return f"{line}  # {fn['note']}"
    return line


def managed_body_for_bank(bank: int, funcs: list[dict]) -> str:
    lines = [
        f"# Auto-synced from recomp/symbols.toml (bank ${bank:02X}).",
        "# Set emit=true on a [[func]] to promote it into AOT codegen.",
    ]
    emitted = [f for f in funcs if f["bank"] == bank and f["emit"]]
    emitted.sort(key=lambda f: f["pc"])
    if not emitted:
        lines.append("# (no emit=true funcs in this bank yet)")
    else:
        for fn in emitted:
            lines.append(_func_cfg_line(fn))
    return "\n".join(lines) + "\n"


def render_bank_cfg(existing: str | None, bank: int, body: str) -> str:
    block = f"{BEGIN}\n{body}{END}\n"
    if existing is None:
        text = BANK_BOOTSTRAP.format(
            bank=bank, BEGIN=BEGIN, END=END, body=body.rstrip("\n"))
        return text if text.endswith("\n") else text + "\n"
    if MANAGED_RE.search(existing):
        return MANAGED_RE.sub(block, existing, count=1)
    # Insert managed block after the bank= line (or at end).
    m = re.search(r"(?m)^bank\s*=\s*[0-9A-Fa-f]+\s*\n", existing)
    if m:
        return existing[: m.end()] + "\n" + block + existing[m.end():]
    if not existing.endswith("\n"):
        existing += "\n"
    return existing + "\n" + block


def banks_needing_cfg(funcs: list[dict]) -> set[int]:
    """Banks that already have a cfg, or have any emit=true func."""
    banks = set()
    for p in CFG_DIR.glob("bank*.cfg"):
        m = re.match(r"bank([0-9A-Fa-f]+)\.cfg$", p.name)
        if m:
            banks.add(int(m.group(1), 16))
    for fn in funcs:
        if fn["emit"]:
            banks.add(fn["bank"])
    # Always keep bank 00.
    banks.add(0)
    return banks


def sync_bank_cfgs(funcs: list[dict]) -> dict[pathlib.Path, str]:
    out: dict[pathlib.Path, str] = {}
    for bank in sorted(banks_needing_cfg(funcs)):
        # Match existing convention: bank00.cfg (zero-padded lowercase hex).
        cfg_path = CFG_DIR / f"bank{bank:02x}.cfg"
        existing = (
            cfg_path.read_text(encoding="utf-8") if cfg_path.exists() else None
        )
        body = managed_body_for_bank(bank, funcs)
        out[cfg_path] = render_bank_cfg(existing, bank, body)
    return out


def _c_ident(name: str) -> str:
    return re.sub(r"[^0-9A-Za-z_]", "_", name)


def emit_header(sym: dict) -> str:
    lines = [
        "/* Auto-generated by tools/sync_symbols.py from recomp/symbols.toml.",
        " * Do NOT hand-edit. Progressive naming / partial-decomp address map.",
        " * See docs/SYMBOLS.md.",
        " */",
        "#pragma once",
        "",
        "#include <stdint.h>",
        "",
        "#ifdef __cplusplus",
        'extern "C" {',
        "#endif",
        "",
        "/* ---- Function entries (24-bit PC) ---- */",
    ]
    for fn in sorted(sym["funcs"], key=lambda f: f["addr"]):
        ident = _c_ident(fn["name"])
        emit = " emit" if fn["emit"] else ""
        meta = f"{fn['status']}{emit}"
        if fn["note"]:
            meta += " — " + fn["note"].replace("*/", "* /")
        lines.append(
            f"#define MW_FN_{ident}  0x{fn['addr']:06X}u  /* {meta} */"
        )
    lines.append("")
    lines.append("/* ---- Opcode hook sites (24-bit PC) ---- */")
    for site in sorted(sym["sites"], key=lambda s: s["addr"]):
        ident = _c_ident(site["name"])
        meta = site["status"]
        if site["note"]:
            meta += " — " + site["note"].replace("*/", "* /")
        lines.append(
            f"#define MW_SITE_{ident}  0x{site['addr']:06X}u  /* {meta} */"
        )
    lines.append("")
    lines.append("/* ---- Site groups (comma-separated 24-bit PCs) ---- */")
    for grp in sym["groups"]:
        ident = _c_ident(grp["name"])
        addrs = ", ".join(f"0x{a:06X}u" for a in grp["addrs"])
        # Keep notes on one line without nesting /* … */ inside /* … */.
        meta = grp["status"]
        if grp["note"]:
            meta += " — " + grp["note"].replace("*/", "* /")
        lines.append(f"/* {meta} */")
        lines.append(f"#define MW_SITE_LIST_{ident}  {addrs}")
        lines.append(
            f"#define MW_SITE_LIST_{ident}_COUNT  "
            f"{len(grp['addrs'])}u"
        )
        lines.append("")
    lines.append("/* ---- WRAM / objects ---- */")
    for obj in sorted(sym["objects"], key=lambda o: o["addr"]):
        ident = _c_ident(obj["name"])
        meta = f"{obj['type']}; {obj['status']}"
        if obj["note"]:
            meta += " — " + obj["note"].replace("*/", "* /")
        lines.append(
            f"#define MW_WRAM_{ident}  0x{obj['addr']:04X}u  /* {meta} */"
        )
    lines.append("")
    lines.append("/* ---- Struct layouts (host-side documentation typedefs) ---- */")
    for st in sorted(sym["structs"].values(), key=lambda s: s["name"]):
        if st["note"]:
            lines.append(
                f"/* {st['note'].replace('*/', '* /')} */"
            )
        # Packed: guest WRAM layouts are byte-exact, not host-ABI aligned.
        lines.append("typedef struct __attribute__((packed)) {")
        # Emit fields; pad gaps as uint8_t arrays when needed.
        cursor = 0
        for field in st["fields"]:
            off = field["offset"]
            if off > cursor:
                lines.append(
                    f"  uint8_t _pad_{cursor:02X}[{off - cursor}];"
                )
                cursor = off
            ctype = {
                "u8": "uint8_t",
                "u16": "uint16_t",
                "u32": "uint32_t",
                "s8": "int8_t",
                "s16": "int16_t",
                "s32": "int32_t",
            }.get(field["type"], "uint8_t")
            width = {"u8": 1, "s8": 1, "u16": 2, "s16": 2,
                     "u32": 4, "s32": 4}.get(field["type"], 1)
            comment = f"+${off:02X}"
            if field["note"]:
                comment += " — " + field["note"].replace("*/", "* /")
            lines.append(f"  {ctype} {field['name']};  /* {comment} */")
            cursor = off + width
        if st["size"] and cursor < st["size"]:
            lines.append(
                f"  uint8_t _pad_tail[{st['size'] - cursor}];"
            )
            cursor = st["size"]
        lines.append(f"}} MwSym_{_c_ident(st['name'])};")
        if st["size"]:
            ident = _c_ident(st["name"])
            lines.append("#ifdef __cplusplus")
            lines.append(
                f'static_assert(sizeof(MwSym_{ident}) == 0x{st["size"]:X}, '
                f'"MwSym_{ident} size");'
            )
            lines.append("#else")
            lines.append(
                f'_Static_assert(sizeof(MwSym_{ident}) == 0x{st["size"]:X}, '
                f'"MwSym_{ident} size");'
            )
            lines.append("#endif")
        lines.append("")
    lines += [
        "#ifdef __cplusplus",
        "}",
        "#endif",
        "",
    ]
    return "\n".join(lines)


def emit_catalog(sym: dict) -> str:
    lines = [
        "<!-- Auto-generated by tools/sync_symbols.py — do not hand-edit.",
        "     Author names in recomp/symbols.toml; see docs/SYMBOLS.md. -->",
        "",
        "# Metal Warriors symbol catalog",
        "",
        "Generated from `recomp/symbols.toml`.",
        "",
        "## Functions",
        "",
        "| Addr | Name | Emit | Status | Note |",
        "|------|------|------|--------|------|",
    ]
    for fn in sorted(sym["funcs"], key=lambda f: f["addr"]):
        emit = "yes" if fn["emit"] else "no"
        note = fn["note"].replace("|", "\\|")
        lines.append(
            f"| `${fn['addr']:06X}` | `{fn['name']}` | {emit} | "
            f"{fn['status']} | {note} |"
        )
    lines += [
        "",
        "## Hook sites",
        "",
        "| Addr | Name | Status | Note |",
        "|------|------|--------|------|",
    ]
    for site in sorted(sym["sites"], key=lambda s: s["addr"]):
        note = site["note"].replace("|", "\\|")
        lines.append(
            f"| `${site['addr']:06X}` | `{site['name']}` | "
            f"{site['status']} | {note} |"
        )
    lines += [
        "",
        "## Site groups",
        "",
    ]
    for grp in sym["groups"]:
        addrs = ", ".join(f"`${a:06X}`" for a in grp["addrs"])
        lines.append(f"### {grp['name']}")
        lines.append("")
        if grp["note"]:
            lines.append(grp["note"])
            lines.append("")
        lines.append(addrs)
        lines.append("")
    lines += [
        "## WRAM / objects",
        "",
        "| Addr | Name | Type | Status | Note |",
        "|------|------|------|--------|------|",
    ]
    for obj in sorted(sym["objects"], key=lambda o: o["addr"]):
        note = obj["note"].replace("|", "\\|")
        lines.append(
            f"| `${obj['addr']:04X}` | `{obj['name']}` | `{obj['type']}` | "
            f"{obj['status']} | {note} |"
        )
    lines += [
        "",
        "## Structs",
        "",
    ]
    for st in sorted(sym["structs"].values(), key=lambda s: s["name"]):
        lines.append(f"### {st['name']} (size `0x{st['size']:X}`)")
        lines.append("")
        if st["note"]:
            lines.append(st["note"])
            lines.append("")
        lines.append("| Off | Field | Type | Note |")
        lines.append("|-----|-------|------|------|")
        for field in st["fields"]:
            note = field["note"].replace("|", "\\|")
            lines.append(
                f"| `+${field['offset']:02X}` | `{field['name']}` | "
                f"`{field['type']}` | {note} |"
            )
        lines.append("")
    return "\n".join(lines)


def write_if_changed(path: pathlib.Path, content: str, check: bool) -> bool:
    """Return True if content differs from disk (or file missing)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    old = path.read_text(encoding="utf-8") if path.exists() else None
    if old == content:
        return False
    if check:
        return True
    path.write_text(content, encoding="utf-8")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--check",
        action="store_true",
        help="exit 1 if generated outputs are stale",
    )
    args = ap.parse_args()

    if not SYMBOLS_PATH.is_file():
        print(f"sync_symbols: missing {SYMBOLS_PATH}", file=sys.stderr)
        return 1

    sym = load_symbols(SYMBOLS_PATH)
    dirty = []

    for path, text in sync_bank_cfgs(sym["funcs"]).items():
        if write_if_changed(path, text, args.check):
            dirty.append(path.relative_to(ROOT).as_posix())

    header = emit_header(sym)
    if write_if_changed(HEADER_PATH, header, args.check):
        dirty.append(HEADER_PATH.relative_to(ROOT).as_posix())

    catalog = emit_catalog(sym)
    if write_if_changed(CATALOG_PATH, catalog, args.check):
        dirty.append(CATALOG_PATH.relative_to(ROOT).as_posix())

    if args.check:
        if dirty:
            print("sync_symbols: stale outputs:", file=sys.stderr)
            for p in dirty:
                print(f"  {p}", file=sys.stderr)
            print("Run: python3 tools/sync_symbols.py", file=sys.stderr)
            return 1
        print("sync_symbols: OK (up to date)")
        return 0

    n_emit = sum(1 for f in sym["funcs"] if f["emit"])
    print(
        f"sync_symbols: {len(sym['funcs'])} funcs "
        f"({n_emit} emit), {len(sym['sites'])} sites, "
        f"{len(sym['groups'])} groups, {len(sym['objects'])} objects, "
        f"{len(sym['structs'])} structs"
    )
    if dirty:
        print("  updated:")
        for p in dirty:
            print(f"    {p}")
    else:
        print("  (no changes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
