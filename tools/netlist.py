#!/usr/bin/env python3
"""从立创EDA专业版工程里导出 PCB 网表，按位号列出每个焊盘的网络。

用途：把「原理图实际连成什么样」和《硬件接线.md》里的设计意图逐条对照。
DRC 只能查规则违反，查不出「网络接错但仍满足间距规则」这一类错误。

链接方式靠 PAD_NET 记录的自有 id：
    ["PAD_NET", <元件id>, <焊盘号>, <图元id>]  ->  padNet
位号来自 ATTR 记录（key == "Designator"，parentId == 元件id）。
"""

import json
import sys
from collections import OrderedDict, defaultdict
from pathlib import Path

from epro_dump import iter_records, load_epru


def find_pcb_section(lines):
    """扫描所有行，返回 PCB 文档所在的行区间 [start, end)。

    文档以 DOCHEAD 行分段，DOCHEAD 的数据体里带 docType。
    """
    marks = []
    for i, l in enumerate(lines):
        for _, h, b in iter_records(l):
            if h and h.get("type") == "DOCHEAD" and b:
                marks.append((i, b.get("docType")))
    for idx, (i, dt) in enumerate(marks):
        if dt == "PCB":
            end = marks[idx + 1][0] if idx + 1 < len(marks) else len(lines)
            return i, end
    raise SystemExit("工程里没有找到 PCB 文档")


def build(path):
    _, text = load_epru(path)
    lines = text.splitlines()
    start, end = find_pcb_section(lines)

    designators = {}                       # 元件id -> 位号
    device_of = {}                         # 元件id -> 器件名
    comp_geom = {}                         # 元件id -> (x_mm, y_mm, 层)
    nets = defaultdict(OrderedDict)        # 元件id -> {焊盘号: 网络}

    for l in lines[start:end]:
        for _, h, b in iter_records(l):
            if not h or not b:
                continue
            t, rid = h.get("type"), h.get("id")
            if t == "ATTR":
                p = b.get("parentId")
                if b.get("key") == "Designator":
                    designators[p] = b.get("value")
                elif b.get("key") == "Device":
                    device_of[p] = b.get("value")
            elif t == "COMPONENT":
                comp_geom[rid] = (b["x"] * 0.0254, b["y"] * 0.0254, b.get("layerId"))
            elif t == "PAD_NET" and isinstance(rid, str) and rid.startswith("["):
                head = json.loads(rid)
                if len(head) == 4:
                    nets[head[1]][head[2]] = b.get("padNet", "")

    return designators, device_of, comp_geom, nets


def main(path):
    designators, device_of, comp_geom, nets = build(path)
    print(f"工程：{path}")
    print(f"元件 {len(nets)} 个，合计焊盘-网络记录 {sum(len(v) for v in nets.values())} 条")
    print()

    def padnum_key(p):
        return (0, int(p)) if str(p).isdigit() else (1, str(p))

    for cid in nets:
        ref = designators.get(cid, f"<无位号:{cid[:8]}>")
        dev = device_of.get(cid, "?")
        try:
            dev = json.loads(dev).get("name", dev)
        except (json.JSONDecodeError, TypeError):
            pass
        x, y, layer = comp_geom.get(cid, (0, 0, "?"))
        side = {1: "TOP", 2: "BOT"}.get(layer, str(layer))
        print(f"### {ref}  [{dev}]  ({x:.2f}, {y:.2f})mm {side}")
        for p in sorted(nets[cid], key=padnum_key):
            net = nets[cid][p] or "<空>"
            print(f"    pin {p:<4} -> {net}")
        print()


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main(sys.argv[1] if len(sys.argv) > 1 else r"T:/ProPrj_Gewu_2026-10-03.epro2")
