"""Select functional or blackbox SRAM views; preserve the complete system."""

from pathlib import Path
import argparse
import re

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = '// ----- 8< ----- FILE "firrtl_black_box_resource_files.f" ----- 8< -----'


def prepare(source, out, view="functional"):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    text = Path(source).read_text().split(MANIFEST, 1)[0]
    text, count = re.subn(r"(?ms)^module ScratchpadSram\b.*?^endmodule\s*", "", text)
    if count != 1:
        raise ValueError("expected one embedded ScratchpadSram module")
    target = out / "DeltaFlowHarness.sv"
    target.write_text(text)
    sources = [target, ROOT / "rtl/memory_models/ScratchpadSram.sv"]
    suffix = ".sv" if view == "functional" else "_bb.sv"
    for name in ("1024x128", "512x128", "256x32"):
        sources.append(ROOT / f"rtl/memory_models/DF_ASAP7_{name}_1R1W{suffix}")
    (out / "sources.f").write_text("".join(str(p.resolve()) + "\n" for p in sources))
    return sources


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, default=ROOT / "generated/DeltaFlowHarness.sv")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--view", choices=["functional", "blackbox"], default="functional")
    a = p.parse_args()
    prepare(a.source, a.out, a.view)
