"""Run the full Chipyard harness and summarize its actual execution counters."""

from pathlib import Path
import argparse
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("fixture", type=Path)
    p.add_argument("--binary", type=Path, default=ROOT / ".local/sim/chipyard_hbm")
    a = p.parse_args()
    subprocess.run([str(a.binary.resolve()), str(a.fixture.resolve()), "16"], check=True)
    result = json.loads((a.fixture / "result_16.json").read_text())
    result["main_array_utilization_percent"] = 100 * result["panel_waves"] / (64 * result["cycles"])
    result["latency_ms_at_1ghz"] = result["cycles"] / 1e6
    print(json.dumps(result, indent=2))
