#!/usr/bin/env python3
import json, sys, re
from pathlib import Path

# Map C types in the JSON to C++ field types and byte sizes
CTYPE_MAP = {
    "uint8_t":   ("std::uint8_t",   1),
    "int8_t":    ("std::int8_t",    1),
    "uint16_t":  ("std::uint16_t",  2),
    "int16_t":   ("std::int16_t",   2),
    "uint32_t":  ("std::uint32_t",  4),
    "int32_t":   ("std::int32_t",   4),
    "char":      ("char",           1),
    "char[]":    ("std::string",   -1),  # dynamic size, uses full remaining payload
}

HEADER_PREAMBLE = r"""#pragma once
#include <cstdint>
#include <cstring>
#include <array>
#include <string>
#include <vector>
#include <variant>
#include <stdexcept>
#include <type_traits>

// Pull official MSP command macros and constants
#include "msp_protocol.h"

namespace msp {

enum class Direction : std::uint8_t { Request = 0, Reply = 1 };

struct BufferReader {
    const std::uint8_t* p;
    const std::uint8_t* end;
    explicit BufferReader(const std::vector<std::uint8_t>& buf) : p(buf.data()), end(buf.data()+buf.size()) {}
    explicit BufferReader(const std::uint8_t* data, std::size_t len) : p(data), end(data+len) {}

    template <typename T>
    T read_le() {
        static_assert(std::is_integral<T>::value, "read_le requires integral type");
        if (end - p < (ptrdiff_t)sizeof(T)) throw std::runtime_error("underflow");
        T v = 0;
        for (std::size_t i=0;i<sizeof(T);++i) v |= (T)p[i] << (8*i);
        p += sizeof(T);
        return v;
    }
    void read_bytes(std::uint8_t* dst, std::size_t n) {
        if (end - p < (ptrdiff_t)n) throw std::runtime_error("underflow");
        std::memcpy(dst, p, n);
        p += n;
    }
    std::string read_string_rest() {
        std::string s;
        s.resize(end - p);
        std::memcpy(s.data(), p, s.size());
        p = end;
        return s;
    }
    std::size_t remaining() const { return (std::size_t)(end - p); }
};

struct BufferWriter {
    std::vector<std::uint8_t> buf;
    template <typename T>
    void write_le(T v) {
        static_assert(std::is_integral<T>::value, "write_le requires integral type");
        for (std::size_t i=0;i<sizeof(T);++i) buf.push_back((std::uint8_t)((v >> (8*i)) & 0xFF));
    }
    void write_bytes(const void* src, std::size_t n) {
        auto p = static_cast<const std::uint8_t*>(src);
        buf.insert(buf.end(), p, p+n);
    }
    void write_string_bytes(const std::string& s) {
        buf.insert(buf.end(), (const std::uint8_t*)s.data(), (const std::uint8_t*)s.data()+s.size());
    }
};

"""

HEADER_EPILOGUE = r"""
} // namespace msp
"""

IDENT_RE = re.compile(r'[^A-Za-z0-9_]')

def cpp_ident(s):
    return IDENT_RE.sub('_', s)

def field_cpp_type(field):
    ctype = field.get("ctype")
    array = field.get("array", False)
    array_ctype = field.get("array_ctype")
    array_size = field.get("array_size", 0)

    # dynamic arrays expressed as Foo_t[] etc. (non char)
    if ctype and ctype.endswith("[]") and ctype != "char[]":
        base = ctype[:-2]
        cpp_base = base
        return f"std::vector<{cpp_base}>", base, 0

    # char[] dynamic payload
    if ctype == "char[]" or (array and array_ctype == "char" and array_size in (0, "0")):
        return "std::string", None, -1

    # fixed char[N] where N can be a number or a symbolic constant
    if ctype and ctype.startswith("char["):
        m = re.match(r"char\[(.+)\]", ctype)  # accept anything inside [...]
        if not m:
            raise ValueError(f"Bad ctype: {ctype}")
        raw_n = m.group(1).strip()
        if raw_n.isdigit():
            n = int(raw_n)
            return f"std::array<char,{n}>", "char", n
        else:
            sym = cpp_ident(raw_n)
            return f"std::array<char,{sym}>", "char", sym

    # typed arrays like uint16_t[OSD_ITEM_COUNT] or via array=true
    if array:
        elem_ctype = array_ctype or ctype
        if elem_ctype in CTYPE_MAP:
            cpp_t, _ = CTYPE_MAP.get(elem_ctype, (elem_ctype, None))
        else:
            cpp_t = elem_ctype  # custom user type
        if isinstance(array_size, int) and array_size > 0:
            return f"std::array<{cpp_t},{array_size}>", elem_ctype, array_size
        else:
            return f"std::vector<{cpp_t}>", elem_ctype, 0

    # scalar
    if ctype in CTYPE_MAP:
        t, _ = CTYPE_MAP[ctype]
        return t, ctype, 0

    # fallback: custom enum or unknown typedef or nested struct type name
    return ctype, ctype, 0

def normalize_fields(parent_name, payload, out):
    """
    Flatten or transform wrapper nodes.
    Cases:
      - Wrapper with single child: inline it, propagate 'repeating' if present.
      - Wrapper with multiple children and 'repeating': synthesize a nested struct and a vector field of that type.
      - Wrapper with multiple children and no 'repeating': synthesize a nested struct and a single field of that type.
    """
    norm = []
    for f in payload:
        if "name" not in f and "payload" in f and isinstance(f["payload"], list):
            children = f["payload"]
            rep = f.get("repeating")
            if len(children) == 1:
                child = dict(children[0])
                if rep and "repeating" not in child:
                    child["repeating"] = rep
                norm.append(child)
                continue
            # synthesize nested struct
            tag = cpp_ident(rep if rep else "group")
            nested_name = f"{cpp_ident(parent_name)}__{tag}"
            # emit nested struct now
            emit_struct(nested_name, children, out, suffix="")
            # create a field referencing the nested struct
            field_name = tag
            if rep:
                norm.append({"name": field_name, "ctype": f"{cpp_ident(nested_name)}[]"})
            else:
                norm.append({"name": field_name, "ctype": f"{cpp_ident(nested_name)}"})
            continue
        norm.append(f)
    return norm

def emit_struct(name, payload, out, suffix=""):
    # normalize and possibly emit nested structs for wrappers
    payload = normalize_fields(name + suffix, payload, out)
    sname = cpp_ident(name + suffix)
    out.append(f"struct {sname} {{")
    # fields
    for f in payload:
        if "name" not in f:
            raise ValueError(f"Schema field missing 'name': {f}")
        f_name = cpp_ident(f["name"])
        t, elem_ctype, n = field_cpp_type(f)
        out.append(f"    {t} {f_name};")
    out.append("")
    # pack()
    out.append("    static std::vector<std::uint8_t> pack(const " + sname + "& v) {")
    out.append("        BufferWriter w;")
    for f in payload:
        f_name = cpp_ident(f["name"])
        t, elem_ctype, n = field_cpp_type(f)
        is_known_integral = elem_ctype in CTYPE_MAP

        # dynamic string uses all bytes
        if t == "std::string":
            out.append(f"        w.write_string_bytes(v.{f_name});")
            continue

        # custom nested single object (non-std type, non-integral)
        if not is_known_integral and not t.startswith("std::"):
            out.append(f"        {{ auto bytes = {t}::pack(v.{f_name}); w.write_bytes(bytes.data(), bytes.size()); }}")
            continue

        # fixed char[N] including symbolic N
        if t.startswith("std::array<char,"):
            if isinstance(n, int):
                out.append(f"        w.write_bytes(v.{f_name}.data(), {n});")
            else:
                out.append(f"        w.write_bytes(v.{f_name}.data(), sizeof(v.{f_name}));")
            continue

        # std::array<T,N>
        if t.startswith("std::array<") and not t.startswith("std::array<char"):
            el = t[t.find('<')+1:t.rfind('>')]
            if is_known_integral:
                out.append(f"        for (const auto& e : v.{f_name}) w.write_le(e);")
            else:
                out.append(f"        for (const auto& e : v.{f_name}) {{")
                out.append(f"            auto bytes = {el}::pack(e);")
                out.append(f"            w.write_bytes(bytes.data(), bytes.size());")
                out.append(f"        }}")
            continue

        # std::vector<T>
        if t.startswith("std::vector<"):
            el = t[t.find('<')+1:t.rfind('>')]
            if is_known_integral:
                out.append(f"        for (const auto& e : v.{f_name}) w.write_le(e);")
            else:
                out.append(f"        for (const auto& e : v.{f_name}) {{")
                out.append(f"            auto bytes = {el}::pack(e);")
                out.append(f"            w.write_bytes(bytes.data(), bytes.size());")
                out.append(f"        }}")
            continue

        # integral scalar
        if is_known_integral:
            out.append(f"        w.write_le(v.{f_name});")
            continue

        # unknown custom enum-like type: assume integral underlying storage
        out.append(f"        w.write_le(static_cast<std::underlying_type_t<decltype(v.{f_name})>>(v.{f_name}));")
    out.append("        return std::move(w.buf);")
    out.append("    }")
    out.append("")
    # unpack()
    out.append("    static " + sname + " unpack(const std::vector<std::uint8_t>& payload) {")
    out.append("        BufferReader r(payload);")
    out.append("        " + sname + " v{};")
    for i, f in enumerate(payload):
        f_name = cpp_ident(f["name"])
        t, elem_ctype, n = field_cpp_type(f)
        is_known_integral = elem_ctype in CTYPE_MAP

        if t == "std::string":
            out.append(f"        v.{f_name} = r.read_string_rest();")
            continue

        # custom nested single object
        if not is_known_integral and not t.startswith("std::"):
            out.append(f"        static constexpr std::size_t __{f_name}_size = []{{ {t} tmp{{}}; auto b = {t}::pack(tmp); return b.size(); }}();")
            out.append(f"        {{ std::vector<std::uint8_t> chunk(__{f_name}_size); r.read_bytes(chunk.data(), __{f_name}_size); v.{f_name} = {t}::unpack(chunk); }}")
            continue

        if t.startswith("std::array<char,"):
            if isinstance(n, int):
                out.append(f"        r.read_bytes(reinterpret_cast<std::uint8_t*>(v.{f_name}.data()), {n});")
            else:
                out.append(f"        r.read_bytes(reinterpret_cast<std::uint8_t*>(v.{f_name}.data()), sizeof(v.{f_name}));")
            continue

        if t.startswith("std::array<") and not t.startswith("std::array<char"):
            el = t[t.find('<')+1:t.rfind('>')]
            if is_known_integral:
                out.append(f"        for (auto& e : v.{f_name}) e = r.read_le<decltype(e)>();")
            else:
                out.append(f"        static constexpr std::size_t __{f_name}_elem_size = []{{ {el} tmp{{}}; auto b = {el}::pack(tmp); return b.size(); }}();")
                out.append(f"        for (auto& e : v.{f_name}) {{")
                out.append(f"            std::vector<std::uint8_t> chunk(__{f_name}_elem_size);")
                out.append(f"            r.read_bytes(chunk.data(), __{f_name}_elem_size);")
                out.append(f"            e = {el}::unpack(chunk);")
                out.append(f"        }}")
            continue

        if t.startswith("std::vector<"):
            el = t[t.find('<')+1:t.rfind('>')]
            out.append(f"        v.{f_name}.clear();")
            if is_known_integral:
                out.append(f"        while (r.remaining() >= sizeof({el})) v.{f_name}.push_back(r.read_le<{el}>());")
            else:
                out.append(f"        static constexpr std::size_t __{f_name}_elem_size = []{{ {el} tmp{{}}; auto b = {el}::pack(tmp); return b.size(); }}();")
                out.append(f"        while (r.remaining() >= __{f_name}_elem_size) {{")
                out.append(f"            std::vector<std::uint8_t> chunk(__{f_name}_elem_size);")
                out.append(f"            r.read_bytes(chunk.data(), __{f_name}_elem_size);")
                out.append(f"            v.{f_name}.push_back({el}::unpack(chunk));")
                out.append(f"        }}")
            continue

        if is_known_integral:
            cpp_t, size = CTYPE_MAP[elem_ctype]
            out.append(f"        v.{f_name} = r.read_le<{cpp_t}>();")
            continue

        out.append(f"        v.{f_name} = static_cast<decltype(v.{f_name})>(r.read_le<std::underlying_type_t<decltype(v.{f_name})>>());")
    out.append("        return v;")
    out.append("    }")
    out.append("};\n")
    return sname

def emit_variant(parent_name, variants, out):
    # generate child structs, then a variant wrapper with a decode dispatcher
    child_names = []
    for key, vdef in variants.items():
        payload = vdef.get("payload") or (vdef.get("request", {}) or {}).get("payload") or []
        # normalize inside emit_struct call
        child = emit_struct(parent_name, payload, out, suffix="__" + cpp_ident(key))
        child_names.append(child)
    vname = cpp_ident(parent_name) + "_variant"
    out.append(f"using {vname} = std::variant<" + ", ".join(child_names) + ">;")
    # best effort dispatcher by payload size
    out.append(f"inline {vname} unpack_{cpp_ident(parent_name)}(const std::vector<std::uint8_t>& payload) {{")
    out.append("    switch (payload.size()) {")
    # try to key exact sizes for fixed size children
    for key, child in zip(variants.keys(), child_names):
        m = re.findall(r'(\d+)', key)
        if "==" in key and m:
            out.append(f"    case {m[0]}: return {child}::unpack(payload);")
    out.append("    default:")
    # try >= rules
    for key, child in zip(variants.keys(), child_names):
        m = re.findall(r'(\d+)', key)
        if ">=" in key and m:
            out.append(f"        if (payload.size() >= {m[0]}) return {child}::unpack(payload);")
    # fallback
    out.append(f"        return {child_names[0]}::unpack(payload);")
    out.append("    }")
    out.append("}\n")

def main():
    if len(sys.argv) != 3:
        print(f"usage: {Path(sys.argv[0]).name} spec.json output.hpp", file=sys.stderr)
        sys.exit(1)

    spec = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
    out = []
    out.append(HEADER_PREAMBLE)

    # Emit structs and packers
    for msg_name, msg in spec.items():
        variants = msg.get("variants")
        if variants:
            emit_variant(msg_name, variants, out)
            continue

        for direction_key in ("request", "reply"):
            side = msg.get(direction_key)
            if side is None:
                continue
            payload = side.get("payload", [])
            sfx = "__" + direction_key
            emit_struct(msg_name, payload, out, suffix=sfx)

    out.append(HEADER_EPILOGUE)
    Path(sys.argv[2]).write_text("\n".join(out), encoding='utf-8')

if __name__ == "__main__":
    main()
