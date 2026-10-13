#!/usr/bin/env python3
"""从立创EDA专业版工程里导出 PCB 网表，按位号列出每个焊盘的网络。

用途：把「原理图实际连成什么样」和《硬件接线.md》里的设计意图逐条对照。
DRC 只能查规则违反，查不出「网络接错但仍满足间距规则」这一类错误。

几处容易踩的地方：

1. 工程里可能有**多个** PCB 文档，本工程就有三个：主板 `PCB1`、顶壳 `TopShell`、
   以及一个已删除的 `Power`。不能只取第一个——第一个是顶壳，一个元件都没有，
   会静默输出「元件 0 个」，看起来像工程是空的。
2. 链接方式靠 PAD_NET 记录的自有 id：
       ["PAD_NET", <元件id>, <焊盘号>, <图元id>]  ->  padNet
3. 位号**不能**取 PCB 段的 `Designator` 属性——那是作者在 layout 时为了手焊方便手工改的
   展示值（`100nF` / `4k7` / `HCT125`），38 个里 21 个与真位号不同。权威位号在原理图段，
   两边靠 `Unique ID` 对上（原理图是 ATTR，PCB 侧在元件 body.attrs 里）。
4. PCB 段 `Device` 属性的值是器件文档的 uuid（`914b327175934b28`），不是名字，
   对它 json.loads 只会抛异常、然后被当成名字原样打印。器件名走 device_index() 查库。
"""

import json
import sys
from collections import Counter, OrderedDict, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from epro_dump import (device_index, doc_sections, find_latest_project,  # noqa: E402
                       iter_section_records, layer_name, load_epru, natural_key)


def build(path):
    """扫一遍工程，返回 {ref_of, device_of, geom, nets, boards}，键都是 PCB 元件 id。"""
    _, text = load_epru(path)
    devs = device_index(text)                       # 器件文档 uuid -> (器件名, 库字段)
    boards = [s for s in doc_sections(text) if s["docType"] == "PCB"]

    sch_ref = {}                        # 原理图元件id -> 位号
    sch_uniq = {}                       # 原理图元件id -> Unique ID
    pcb_label = {}                      # PCB 元件id -> 手改的展示标签
    pcb_uniq = {}                       # PCB 元件id -> Unique ID
    device_of = {}                      # PCB 元件id -> 器件名
    comp_geom = {}                      # PCB 元件id -> (x_mm, y_mm, layerId)
    nets = defaultdict(OrderedDict)     # PCB 元件id -> {焊盘号: 网络}
    comp_count = Counter()              # PCB 文档 uuid -> 存活元件数

    for doc, _, head, body in iter_section_records(text):
        t, rid = head.get("type"), head.get("id")
        if doc["docType"] == "SCH_PAGE":
            if t == "ATTR":
                p = body.get("parentId")
                if body.get("key") == "Designator":
                    sch_ref[p] = body.get("value")
                elif body.get("key") == "Unique ID":
                    sch_uniq[p] = body.get("value")
        elif doc["docType"] == "PCB":
            if t == "ATTR":
                p = body.get("parentId")
                if body.get("key") == "Designator":
                    pcb_label[p] = body.get("value")
                elif body.get("key") == "Device":
                    device_of[p] = devs.get(body.get("value"), ("?", {}))[0]
            elif t == "COMPONENT":
                comp_count[doc["uuid"]] += 1
                pcb_uniq[rid] = (body.get("attrs") or {}).get("Unique ID")
                comp_geom[rid] = (body.get("x", 0) * 0.0254, body.get("y", 0) * 0.0254,
                                  body.get("layerId"))
            elif t == "PAD_NET" and isinstance(rid, str) and rid.startswith("["):
                net_head = json.loads(rid)
                if len(net_head) == 4:
                    nets[net_head[1]][net_head[2]] = body.get("padNet", "")

    uniq_ref = {sch_uniq[cid]: sch_ref[cid] for cid in sch_ref if sch_uniq.get(cid)}
    ref_of = {cid: uniq_ref.get(pcb_uniq.get(cid)) or pcb_label.get(cid)
                   or f"<无位号:{cid[:8]}>"
              for cid in nets}

    for b in boards:
        b["live_components"] = comp_count[b["uuid"]]

    return {"ref_of": ref_of, "device_of": device_of, "geom": comp_geom,
            "nets": nets, "boards": boards}


def main(path):
    data = build(path)
    ref_of, device_of, comp_geom = data["ref_of"], data["device_of"], data["geom"]
    nets = data["nets"]

    board_line = " · ".join(
        f"{b['name']}({b['live_components']} 元件{', 已删除' if b['deleted'] else ''})"
        for b in data["boards"])
    print(f"工程：{path}")
    print(f"板子：{board_line or '（无）'}")
    print(f"元件 {len(nets)} 个，合计焊盘-网络记录 {sum(len(v) for v in nets.values())} 条")
    print()

    for cid in sorted(nets, key=lambda c: natural_key(ref_of[c])):
        ref = ref_of[cid]
        x, y, layer = comp_geom.get(cid, (0, 0, "?"))
        print(f"### {ref}  [{device_of.get(cid, '?')}]  ({x:.2f}, {y:.2f})mm {layer_name(layer)}")
        for p in sorted(nets[cid], key=natural_key):
            print(f"    pin {p:<4} -> {nets[cid][p] or '<空>'}")
        print()


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main(sys.argv[1] if len(sys.argv) > 1 else find_latest_project())
