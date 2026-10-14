"""Merge the five externally downloaded ASAP7 LVT TT NLDM groups."""

from pathlib import Path
import argparse
import gzip
import re

NAMES = [
    "asap7sc7p5t_SEQ_LVT_TT_nldm_220123.lib",
    "asap7sc7p5t_AO_LVT_TT_nldm_211120.lib.gz",
    "asap7sc7p5t_INVBUF_LVT_TT_nldm_220122.lib.gz",
    "asap7sc7p5t_OA_LVT_TT_nldm_211120.lib.gz",
    "asap7sc7p5t_SIMPLE_LVT_TT_nldm_211120.lib.gz",
]


def groups(text, kind):
    for match in re.finditer(r"\b" + kind + r"\s*\(\s*([^)]*?)\s*\)\s*\{", text):
        pos, level, quoted, escaped = match.end(), 1, False, False
        while level:
            char = text[pos]
            pos += 1
            if escaped:
                escaped = False
                continue
            if char == "\\" and quoted:
                escaped = True
                continue
            if char == '"':
                quoted = not quoted
            if not quoted:
                level += (char == "{") - (char == "}")
        yield match.group(1).strip('"'), text[match.start() : pos]


def prepare(directory, out):
    directory, out = Path(directory), Path(out)
    texts = []
    for name in NAMES:
        path = directory / name
        text = (
            gzip.decompress(path.read_bytes()).decode()
            if name.endswith(".gz")
            else path.read_text()
        )
        texts.append(re.sub(r"/\*.*?\*/", "", text, flags=re.S))
    header = texts[0][texts[0].index("{") + 1 : re.search(r"\bcell\s*\(", texts[0]).start()]
    known = {
        name
        for kind in ("lu_table_template", "power_lut_template")
        for name, _ in groups(header, kind)
    }
    for text in texts[1:]:
        for kind in ("lu_table_template", "power_lut_template"):
            for name, body in groups(text, kind):
                if name not in known:
                    header += "\n" + body
                    known.add(name)
    cells = {}
    for text in texts:
        for name, body in groups(text, "cell"):
            if name in cells:
                raise ValueError("duplicate standard cell " + name)
            cells[name] = body
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        "library (deltaflow_asap7_lvt_tt) {\n" + header + "\n" + "\n".join(cells.values()) + "\n}\n"
    )
    return out


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--lib-dir", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    print(prepare(a.lib_dir, a.out))
