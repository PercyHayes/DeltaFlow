"""Share byte-identical RTLIL module definitions, retaining every instance.

Only module headers and child module type references are normalized for identity.
All attributes, ports, parameters, cells, processes and connections participate.
No gates, registers, memories, instances or connections are removed from a module.
"""

from pathlib import Path
from collections import Counter
import argparse
import json
import re


def share(source, target, report, top=None):
    source, target = Path(source), Path(target)
    assert source.resolve() != target.resolve()
    index = {}
    prefix = b""
    pending = b""
    current = None
    start = 0
    with source.open("rb") as stream:
        offset = 0
        while True:
            line = stream.readline()
            if not line:
                break
            # Binary input makes byte positions exact. Avoid an underlying
            # file-position query on every one of millions of RTLIL lines.
            pos = offset
            offset += len(line)
            if current is None:
                if line.startswith(b"module "):
                    name = line.split()[1].decode()
                    current = name
                    index[name] = dict(start=pos - len(pending), children=Counter())
                    pending = b""
                elif line.startswith(b"attribute "):
                    pending += line
                else:
                    assert not pending, "unexpected global statement after module attributes"
                    prefix += line
            else:
                match = re.match(rb"\s+cell (\S+) ", line)
                if match:
                    index[current]["children"][match[1].decode()] += 1
                if line == b"end\n":
                    index[current]["end"] = offset
                    current = None
    assert current is None and not pending and index
    representatives = {}
    mapping = {}
    with source.open("rb") as stream:

        def body(name):
            item = index[name]
            stream.seek(item["start"])
            return stream.read(item["end"] - item["start"])

        def canonical(name, stack=()):
            if name in mapping:
                return mapping[name]
            assert name not in stack, "recursive module dependency"
            children = {
                c: canonical(c, stack + (name,)) for c in index[name]["children"] if c in index
            }
            raw = body(name)
            raw = re.sub(rb"(?m)^module \S+", lambda _: b"module \\NORMALIZED", raw)
            raw = re.sub(
                rb"(?m)^(\s+cell )(\S+)( )",
                lambda m: m[1]
                + (children[m[2].decode()].encode() if m[2].decode() in children else m[2])
                + m[3],
                raw,
            )
            origin = name.split("$")[0]
            representative = representatives.setdefault((origin, raw), name)
            mapping[name] = representative
            return representative

        for name in index:
            canonical(name)
        with target.open("wb") as dest:
            dest.write(prefix)
            for name in index:
                if mapping[name] != name:
                    continue
                raw = body(name)
                raw = re.sub(
                    rb"(?m)^(\s+cell )(\S+)( )",
                    lambda m: m[1] + mapping.get(m[2].decode(), m[2].decode()).encode() + m[3],
                    raw,
                )
                dest.write(raw)

    # Hierarchical multiplicity before/after must match for all logical origins
    # and every primitive cell; this is independent of the signature grouping.
    def inventory(name, rewritten=False):
        counts = Counter({name.split("$")[0]: 1})
        for child, mult in index[name]["children"].items():
            actual = mapping.get(child, child) if rewritten else child
            if actual in index:
                counts.update({k: mult * v for k, v in inventory(actual, rewritten).items()})
            else:
                counts[actual] += mult
        return counts

    # Memoization is necessary for the 16384 repeated PE definitions.
    from functools import lru_cache

    inventory = lru_cache(None)(inventory)
    if top is None:
        top = "\\DeltaFlowHarness" if "\\DeltaFlowHarness" in index else "\\Probe"
    elif not top.startswith("\\"):
        top = "\\" + top
    assert top in index, ("top not found", top)
    before = inventory(top)
    after = inventory(top, True)
    assert before == after, "sharing changed hierarchical inventory"
    result = dict(
        definitions_before=len(index),
        definitions_after=len(representatives),
        top=top,
        physical_inventory=dict(before),
        mapping=mapping,
        identity=(
            "byte-identical complete module bodies after normalizing only header names "
            "and recursively identical child type names; attributes preserved"
        ),
        physical_instances_preserved=True,
    )
    Path(report).write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("source", type=Path)
    p.add_argument("target", type=Path)
    p.add_argument("report", type=Path)
    p.add_argument("--top")
    a = p.parse_args()
    r = share(a.source, a.target, a.report, a.top)
    print(
        json.dumps(
            {
                k: r[k]
                for k in ("definitions_before", "definitions_after", "physical_instances_preserved")
            },
            indent=2,
        )
    )
