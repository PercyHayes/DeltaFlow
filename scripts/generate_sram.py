"""Generate project-owned 1R1W SRAM simulation and illustrative timing views.

No silicon implementation is generated. Areas are the retained architectural
budget, originally estimated using FakeRAM geometry and a 1.5 port multiplier.
"""

from pathlib import Path
import argparse
import json
import re


def ports(rows, bits):
    addr = (rows - 1).bit_length()
    return [
        ("clock", "input", 1),
        ("ren", "input", 1),
        ("wen", "input", 1),
        ("raddr", "input", addr),
        ("waddr", "input", addr),
        ("wdata", "input", bits),
        ("wmask", "input", bits // 8),
        ("rdata", "output", bits),
    ]


def module(name, rows, bits, behavior=True):
    args = []
    for pin, direction, width in ports(rows, bits):
        vector = f"[{width-1}:0] " if width > 1 else ""
        storage = "reg " if direction == "output" and behavior else ""
        args.append(f"{direction} {storage}{vector}{pin}")
    annotation = "" if behavior else "(* blackbox *)\n"
    header = annotation + f"module {name} (\n    " + ",\n    ".join(args) + "\n);\n"
    if behavior:
        header += f"""    reg [{bits-1}:0] mem [0:{rows-1}];
    integer lane;

    always @(posedge clock) begin
        if (ren) begin
            rdata <= mem[raddr];
        end
        if (wen) begin
            for (lane = 0; lane < {bits//8}; lane = lane + 1) begin
                if (wmask[lane]) begin
                    mem[waddr][8*lane +: 8] <= wdata[8*lane +: 8];
                end
            end
        end
    end
"""
    return header + "endmodule\n"


def constraint(value, kind):
    v = f"{value:.6f}, {value:.6f}"
    return f"""timing() {{ related_pin : "clock"; timing_type : {kind};
rise_constraint(constraint_table) {{ values("{v}", "{v}"); }}
fall_constraint(constraint_table) {{ values("{v}", "{v}"); }} }}"""


def liberty_cell(name, rows, bits, area, model):
    types = []
    pin_groups = []
    for pin, direction, width in ports(rows, bits):
        body = f"direction : {direction};"
        if pin == "clock":
            body += " clock : true; capacitance : 0.005;"
        elif direction == "input":
            body += f' capacitance : {model["cap_input_pf"]};'
        if pin == "rdata":
            v = ", ".join([str(model["access_time_ns"])] * 2)
            body += f"""timing() {{ related_pin : "clock"; timing_type : rising_edge;
cell_rise(delay_table) {{ values("{v}", "{v}"); }}
cell_fall(delay_table) {{ values("{v}", "{v}"); }}
rise_transition(delay_table) {{ values("0.02, 0.02", "0.02, 0.02"); }}
fall_transition(delay_table) {{ values("0.02, 0.02", "0.02, 0.02"); }} }}"""
        elif pin != "clock":
            body += constraint(model["t_setup_ns"], "setup_rising") + constraint(
                model["t_hold_ns"], "hold_rising"
            )
        if width == 1:
            pin_groups.append(f"pin({pin}) {{ {body} }}")
        else:
            typ = f"{name}_{pin}_type"
            types.append(
                f"type({typ}) {{ base_type:array; data_type:bit; bit_width:{width}; bit_from:{width-1}; bit_to:0; downto:true; }}"
            )
            # Per-bit records are needed for reliable bus connectivity in Yosys.
            children = "\n".join(f'pin("{pin}[{i}]") {{ {body} }}' for i in range(width))
            pin_groups.append(
                f"bus({pin}) {{ bus_type:{typ}; direction:{direction};\n{children}\n}}"
            )
    return "\n".join(types), f"cell({name}) {{ area : {area:.9f};\n" + "\n".join(pin_groups) + "\n}"


def liberty_header(name):
    return f"""library({name}) {{
technology(cmos); delay_model : table_lookup;
time_unit : "1ns"; voltage_unit : "1V"; current_unit : "1uA";
capacitive_load_unit(1,pf); nom_process:1; nom_voltage:0.7; nom_temperature:25;
lu_table_template(delay_table) {{ variable_1:input_net_transition; variable_2:total_output_net_capacitance;
index_1("0.01, 0.1"); index_2("0.001, 0.01"); }}
lu_table_template(constraint_table) {{ variable_1:constrained_pin_transition; variable_2:related_pin_transition;
index_1("0.01, 0.1"); index_2("0.01, 0.1"); }}
"""


def generate(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    model = {"t_setup_ns": 0.05, "t_hold_ns": 0.05, "access_time_ns": 0.2183, "cap_input_pf": 0.005}
    geometries = [(1024, 128, 8310.904), (512, 128, 4212.376), (256, 32, 540.778)]
    types = []
    cells = []
    records = []
    for rows, bits, area in geometries:
        name = f"DF_ASAP7_{rows}x{bits}_1R1W"
        typ, cell = liberty_cell(name, rows, bits, area, model)
        types.append(typ)
        cells.append(cell)
        (out / (name + ".sv")).write_text(module(name, rows, bits))
        (out / (name + "_bb.sv")).write_text(module(name, rows, bits, False))
        records.append(
            dict(
                name=name,
                rows=rows,
                bits=bits,
                area_um2=area,
                instances_per_tile=16,
                instances_chip=64,
            )
        )
    text = liberty_header("deltaflow_sram_1r1w_budget") + "\n" + "\n".join(types + cells) + "\n}\n"
    # OpenSTA bus type parsing requires whitespace after the attribute colon.
    text = re.sub(r":(?=[^\s])", ": ", text)
    (out / "deltaflow_sram.lib").write_text(text)
    (out / "model.json").write_text(
        json.dumps(
            dict(
                macros=records,
                timing=model,
                limitation="Area and timing budgets, not characterized SRAM or physical design.",
            ),
            indent=2,
        )
        + "\n"
    )
    return out / "deltaflow_sram.lib"


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / ".local/sram")
    print(generate(p.parse_args().out))
