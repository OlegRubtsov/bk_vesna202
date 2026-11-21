from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
REFERENCE = PROJECT / "reference"
OUTPUT = PROJECT / "include" / "builtin_bins.h"


def cpp_string(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


files = sorted(
    (item for item in REFERENCE.iterdir() if item.is_file() and item.suffix.lower() == ".bin"),
    key=lambda item: item.name.casefold(),
)
lines = [
    "#pragma once",
    "#include <Arduino.h>",
    "#include <pgmspace.h>",
    "",
    "struct BuiltinBinFile { const char *name; const uint8_t *data; size_t size; };",
    "",
]

for index, path in enumerate(files):
    data = path.read_bytes()
    lines.append(f"const uint8_t BIN_FILE_{index:02d}[] PROGMEM = {{")
    for offset in range(0, len(data), 24):
        chunk = data[offset : offset + 24]
        lines.append("  " + ", ".join(f"0x{byte:02X}" for byte in chunk) + ",")
    lines.extend(["};", ""])

lines.append("const BuiltinBinFile BUILTIN_BIN_FILES[] = {")
for index, path in enumerate(files):
    lines.append(
        f"  {{{cpp_string(path.name)}, BIN_FILE_{index:02d}, sizeof(BIN_FILE_{index:02d})}},"
    )
lines.extend(["};", f"constexpr size_t BUILTIN_BIN_FILE_COUNT = {len(files)};", ""])
OUTPUT.write_text("\n".join(lines), encoding="utf-8")
print(f"Embedded {len(files)} BIN files ({sum(item.stat().st_size for item in files)} bytes) into {OUTPUT}")
