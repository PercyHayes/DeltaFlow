"""Fresh hierarchical LVT synthesis; full-system closure is a separate STA step."""

from pathlib import Path
import argparse
import json
import os
import re
import subprocess
from prepare_rtl import prepare
from generate_sram import generate
from prepare_lvt import prepare as prepare_library
from share_definitions import share

ROOT = Path(__file__).resolve().parents[1]
DONT_USE = ["*x1p*_ASAP7*", "*xp*_ASAP7*", "SDF*", "ICG*"]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--yosys", type=Path, required=True)
    p.add_argument("--lib-dir", type=Path, required=True)
    p.add_argument("--out", type=Path, default=ROOT / ".local/synthesis")
    p.add_argument("--source", type=Path, default=ROOT / "generated/DeltaFlowHarness.sv")
    p.add_argument("--top", default="DeltaFlowHarness")
    a = p.parse_args()
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "tmp").mkdir(exist_ok=True)
    lib = prepare_library(a.lib_dir, out / "asap7_lvt_tt.lib")
    macro = generate(out / "sram")
    sources = (
        prepare(a.source, out / "rtl", "blackbox")
        if a.top == "DeltaFlowHarness"
        else [a.source.resolve()]
    )

    def quote(path):
        text = str(path)
        if not re.fullmatch(r"[A-Za-z0-9_./+:-]+", text):
            raise ValueError("Yosys/Slang paths must not contain spaces or command delimiters")
        return text

    blacks = " ".join(
        "--blackboxed-module DF_ASAP7_" + shape + "_1R1W"
        for shape in ("1024x128", "512x128", "256x32")
    )
    frontend = out / "frontend.ys"
    frontend.write_text(
        "\n".join(
            [
                f"read_liberty -lib {quote(macro)}",
                f"read_slang --keep-hierarchy --no-proc --top {a.top} -D SYNTHESIS {blacks} "
                + " ".join(quote(s) for s in sources),
                f"write_rtlil {quote(out / 'frontend.il')}",
            ]
        )
        + "\n"
    )
    env = dict(os.environ, TMPDIR=str(out / "tmp"), YOSYS_MAX_THREADS="1")

    def run(script, name):
        with (out / name).open("w") as log:
            subprocess.run(
                [str(a.yosys.resolve()), "-Q", "-T", "-s", str(script)],
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
                env=env,
            )

    run(frontend, "frontend.log")
    sharing = share(out / "frontend.il", out / "shared.il", out / "definitions.json", a.top)
    # Flatten arithmetic only. Preserve physical panel/tile/SRAM instances.
    arithmetic = [
        name
        for name in sharing["mapping"]
        if sharing["mapping"][name] == name
        and re.match(r"\\(?:RetimedWavePE|RegisteredFp32)(?:_|\$|$)", name)
    ]
    flatten_commands = [
        line
        for name in arithmetic
        for line in (f"select -module {name}", "flatten", "select -clear")
    ]
    dont = " ".join("-dont_use " + name for name in DONT_USE)
    (out / "abc.constr").write_text("set_driving_cell INVx1_ASAP7_75t_L\nset_load 1.0\n")
    script = out / "map.ys"
    script.write_text(
        "\n".join(
            [
                f"read_liberty -lib {quote(macro)}",
                f"read_rtlil {quote(out / 'shared.il')}",
                "chformal -remove",
                f"hierarchy -check -top {a.top}",
                *flatten_commands,
                f"synth -top {a.top} -noabc -hieropt",
                "delete t:$scopeinfo",
                f"dfflibmap -liberty {quote(lib)} {dont}",
                f"abc -liberty {quote(lib)} -constr {quote(out / 'abc.constr')} -D 800 {dont}",
                "clean",
                f"read_liberty -lib {quote(lib)}",
                "check -assert",
                f"tee -o {quote(out / 'stat.json')} stat -json -top {a.top} -liberty {quote(lib)} -liberty {quote(macro)}",
                f"write_verilog -noattr -noexpr {quote(out / 'mapped.v')}",
            ]
        )
        + "\n"
    )
    run(script, "yosys.log")
    stats = json.loads((out / "stat.json").read_text())
    print(json.dumps(stats["design"], indent=2))


if __name__ == "__main__":
    main()
