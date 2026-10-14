"""Full linked ideal-clock OpenSTA; no path exceptions or module extrapolation."""

from pathlib import Path
import argparse
import gzip
import json
import os
import shutil
import subprocess
from prepare_lvt import prepare
from generate_sram import generate

ROOT = Path(__file__).resolve().parents[1]


def unpack_reference(out):
    archive = out / "reference.v.gz"
    with archive.open("wb") as target:
        for part in sorted((ROOT / "reference").glob("DeltaFlowHarness_mapped.v.gz.part*")):
            with part.open("rb") as source:
                shutil.copyfileobj(source, target)
    netlist = out / "mapped.v"
    with gzip.open(archive, "rb") as source, netlist.open("wb") as target:
        shutil.copyfileobj(source, target)
    return netlist


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sta", type=Path, required=True)
    p.add_argument("--lib-dir", type=Path, required=True)
    p.add_argument(
        "--netlist", type=Path, help="omit to use the selected complete mapped reference"
    )
    p.add_argument("--top", default="DeltaFlowHarness")
    p.add_argument("--out", type=Path, default=ROOT / ".local/sta")
    p.add_argument("--threads", type=int, default=4)
    a = p.parse_args()
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "tmp").mkdir(exist_ok=True)
    lib = prepare(a.lib_dir, out / "asap7_lvt_tt.lib")
    macro = generate(out / "sram")
    netlist = a.netlist.resolve() if a.netlist else unpack_reference(out)
    brace = lambda path: "{" + str(path) + "}"
    script = out / "timing.tcl"
    script.write_text(
        "\n".join(
            [
                f"read_liberty {brace(lib)}",
                f"read_liberty {brace(macro)}",
                "set_cmd_units -time ns -capacitance pF",
                f"read_verilog {brace(netlist)}",
                f"link_design {a.top}",
                "create_clock -name core -period 1.0 [get_ports clock]",
                "set_clock_uncertainty -setup 0.1 [get_clocks core]",
                "set_clock_transition 0.025 [get_clocks core]",
                "set data_inputs [all_inputs]",
                "set index [lsearch -exact $data_inputs [lindex [get_ports clock] 0]]",
                'if {$index < 0} {error "clock port missing"}',
                "set data_inputs [lreplace $data_inputs $index $index]",
                "set_input_delay 0.1 -clock core $data_inputs",
                "set_input_transition 0.05 $data_inputs",
                "set_output_delay 0.1 -clock core [all_outputs]",
                "set_load 0.001 [all_outputs]",
                'puts "SETUP [check_setup -verbose]"',
                f"report_worst_slack -max -digits 6 > {brace(out / 'slack.txt')}",
                f"report_tns -digits 6 > {brace(out / 'tns.txt')}",
                "report_checks -path_delay max -group_path_count 10 -sort_by_slack "
                "-format full -fields {capacitance slew fanout} -digits 6 "
                f"> {brace(out / 'paths.txt')}",
                f"report_check_types -max_slew -max_capacitance -max_fanout -violators -max_count 100 > {brace(out / 'electrical.txt')}",
                'puts "STA_COMPLETE"',
            ]
        )
        + "\n"
    )
    env = dict(os.environ, TMPDIR=str(out / "tmp"))
    with (out / "sta.log").open("w") as log:
        subprocess.run(
            [
                str(a.sta.resolve()),
                "-no_init",
                "-no_splash",
                "-threads",
                str(a.threads),
                "-exit",
                str(script),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            check=True,
        )
    text = (out / "sta.log").read_text()
    if "STA_COMPLETE" not in text or any(line.startswith("Error") for line in text.splitlines()):
        raise RuntimeError("OpenSTA reported an error; inspect sta.log")
    print((out / "slack.txt").read_text())


if __name__ == "__main__":
    main()
