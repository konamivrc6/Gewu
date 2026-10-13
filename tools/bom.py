#!/usr/bin/env python3
"""从立创EDA专业版工程（.epro2）导出元器件清单。

出三张表，各管一件事：

    明细      逐位号一行，照着板子核对焊接用
    采购      按立创编号聚合，下单用
    手焊清单  PCB 侧按「层面 × 板面标签」分组；标签是作者 layout 时为手焊方便
              手工改的展示值（`100nF` / `4k7`），工具照读，同时给出对应的权威位号

几处容易踩的地方：

1. 元素的 id 在**行头**（`head["id"]`），body 里没有 uuid 字段。更要命的是 DEVICE 段
   META 记录的 `head["id"]` 恒为常量字符串 `"META"`——25 个器件会全塌进同一个键、
   互相覆盖，最后只剩一条。器件身份只能取 DOCHEAD 的 uuid，而原理图 ATTR `Device`
   的值正是它，两边靠这个对上。（曾按「器件名 = partId 去后缀」连过，SW4 就漏了：
   它的 partId 是 `EC11E15244B2`，器件库里那条却叫 `EC11E15244B2_C470754`。）
2. 位号 / 器件 / 封装是同段的 ATTR 记录，靠 `parentId` 指向元件 id。
   COMPONENT 自身的 `attrs.DeviceName` 是器件名，不是位号。
3. .epru 是变更日志，同一元素会有「墓碑」行（`头|||`）和后续的存活行。
   墓碑必须跳过，但**不能因为先见到墓碑就认为元素不存在**——存活行可能在后面。
   还有一条连带的坑：**ATTR 行出现在它的 COMPONENT 行之前**。若边扫边判
   「parentId 是否已在元件表里」，先到的属性会被整批丢掉，38 个位号只剩十来个，
   而且不报错。所以属性一律无条件收集，扫完再连接。
4. 立创编号是 `Supplier Part`（`C5137467` 这种），**不是** `LCSC Part Name`——后者是
   `100nF ±10% 50V` 这类描述。两个键名很像，取反了表里就会印出一串描述冒充编号。
5. 封装名三个来源都不对路：原理图 ATTR 的 `Footprint` 值是 null（38/38），
   DEVICE 的 `Supplier Footprint` 是 `0805` / `插件,P=5.08mm` 这类供应商分类串，
   `Origin Footprint` 只有 1 个器件有。真封装名走 DEVICE META 的 `Footprint`(uuid)
   指向 FOOTPRINT 段，取那一段的 META.title。
6. 位号一律取**原理图段**。PCB 段的 `Designator` 是手改的展示值，不能当位号用；
   两边靠 `Unique ID` 连接。
7. 「主板/顶壳」不能用 `head["docId"]` 区分——PCB COMPONENT 的行头里根本没有这个字段，
   读出来恒为 None，所有元件会塌进同一组。板子名在**段内第一条 META 的 title**。
"""

import argparse
import json
import sys
import unicodedata
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from epro_dump import (device_index, doc_sections, find_latest_project,  # noqa: E402
                       iter_section_records, layer_name, load_epru, natural_key)

MAX_DESC = 30          # 描述列截断宽度，避免一张表宽到没法看


def disp_width(text):
    """终端显示宽度：全角字符占两列，组合记号占零列。"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF"
               else 0 if unicodedata.combining(c) else 1
               for c in text)


def pad(text, width):
    """按显示宽度右补空格。用码点数补会在中文列上错位——一个汉字占两列。"""
    return text + " " * max(0, width - disp_width(text))


def clip(text, width):
    return text if disp_width(text) <= width else text[:width - 1] + "…"


def render_table(headers, rows, gap="  "):
    """按显示宽度对齐输出一张表。"""
    table = [headers] + [[str(c) for c in r] for r in rows]
    widths = [max(disp_width(r[i]) for r in table) for i in range(len(headers))]
    lines = [gap.join(pad(h, w) for h, w in zip(headers, widths)).rstrip(),
             gap.join("-" * w for w in widths)]
    for r in table[1:]:
        lines.append(gap.join(pad(c, w) for c, w in zip(r, widths)).rstrip())
    return "\n".join(lines)


def collect(path):
    """扫一遍工程，返回 {rows, boards, pcb}。"""
    _, text = load_epru(path)
    devs = device_index(text)                                     # 器件文档 uuid -> (名, 库字段)
    sections = doc_sections(text)
    fp_name = {s["uuid"]: s["name"] for s in sections if s["docType"] == "FOOTPRINT"}
    boards = [s for s in sections if s["docType"] == "PCB"]

    sch, sch_attr = {}, defaultdict(dict)
    pcb, pcb_attr = {}, defaultdict(dict)
    comp_count = Counter()

    for doc, _, head, body in iter_section_records(text):
        t, rid = head.get("type"), head.get("id")
        if doc["docType"] == "SCH_PAGE":
            if t == "COMPONENT":
                sch[rid] = body
            elif t == "ATTR" and body.get("parentId"):
                # 无条件收集：ATTR 行可能排在自己的 COMPONENT 行前面（见模块 docstring 第 3 条）
                sch_attr[body["parentId"]][body.get("key")] = body.get("value")
        elif doc["docType"] == "PCB":
            if t == "COMPONENT":
                comp_count[doc["uuid"]] += 1
                pcb[rid] = body
            elif t == "ATTR" and body.get("parentId"):
                pcb_attr[body["parentId"]][body.get("key")] = body.get("value")

    for b in boards:
        b["live_components"] = comp_count[b["uuid"]]

    rows = []
    for cid, comp in sch.items():
        a = sch_attr.get(cid, {})
        ref = a.get("Designator")
        if not ref:                       # 图框、网络标号等没有位号的元素
            continue
        name, lib = devs.get(a.get("Device"), ("", {}))
        rows.append({
            "ref": ref,
            "device": name or (comp.get("partId") or "").rsplit(".", 1)[0],
            "footprint": fp_name.get(lib.get("Footprint") or "", ""),
            "lcsc": lib.get("Supplier Part") or "",
            "desc": lib.get("LCSC Part Name") or lib.get("Description") or "",
            "mfr": lib.get("Manufacturer") or "",
            "mpn": lib.get("Manufacturer Part") or "",
            "unique": a.get("Unique ID") or "",
            # 实例级覆盖优先于器件库默认值（本工程只有 C1 一条覆盖：不贴片）
            "in_bom": (a.get("Add into BOM") or lib.get("Add into BOM") or "yes") != "no",
        })
    rows.sort(key=lambda r: natural_key(r["ref"]))
    return {"rows": rows, "boards": boards, "pcb": pcb, "pcb_attr": pcb_attr}


def aggregate(rows):
    """按立创编号聚合。没有编号的自建器件按器件名单独成组，否则会挤进同一个空键。"""
    groups = defaultdict(list)
    for r in rows:
        if not r["in_bom"]:
            continue
        groups[(r["lcsc"], r["device"] if not r["lcsc"] else "")].append(r)
    out = []
    for (lcsc, dev), rs in groups.items():
        out.append({"lcsc": lcsc, "device": dev or rs[0]["device"], "count": len(rs),
                    "refs": [r["ref"] for r in rs]})
    out.sort(key=lambda g: (-g["count"], natural_key(g["lcsc"] or g["device"])))
    return out


def hand_solder(pcb, pcb_attr, rows):
    """PCB 侧手焊清单：层面 -> 板面标签 -> 位号。标签是手工改的展示值，原样呈现。"""
    uniq_ref = {r["unique"]: r["ref"] for r in rows if r["unique"]}
    layers = defaultdict(lambda: defaultdict(list))
    for cid, body in pcb.items():
        uniq = (body.get("attrs") or {}).get("Unique ID")
        label = pcb_attr.get(cid, {}).get("Designator") or "（无标签）"
        layers[body.get("layerId")][label].append(uniq_ref.get(uniq, "（对不上位号）"))
    out = []
    for lid in sorted(layers, key=lambda x: (x is None, x)):
        entries = sorted(layers[lid].items(),
                         key=lambda kv: (-len(kv[1]), natural_key(kv[0])))
        out.append({"layer": layer_name(lid), "total": sum(len(v) for _, v in entries),
                    "labels": [{"label": lab, "refs": sorted(refs, key=natural_key)}
                               for lab, refs in entries]})
    return out


def text_report(data):
    rows, boards = data["rows"], data["boards"]
    agg = aggregate(rows)
    parts = []

    no_lcsc = [r["ref"] for r in rows if not r["lcsc"]]
    parts.append("=== 位号明细 ===")
    parts.append(render_table(
        ["位号", "器件", "封装", "立创编号", "描述"],
        [[f"{r['ref']}*" if not r["in_bom"] else r["ref"], r["device"],
          clip(r["footprint"], 40), r["lcsc"] or "—", clip(r["desc"], MAX_DESC)]
         for r in rows]))
    note = f"共 {len(rows)} 个位号"
    if no_lcsc:
        note += f"；{len(no_lcsc)} 个无立创编号（自建器件，表中记作「—」）：{' '.join(no_lcsc)}"
    parts.append(note)
    if any(not r["in_bom"] for r in rows):
        excluded = [r["ref"] for r in rows if not r["in_bom"]]
        parts.append(f"* 标记的是 Add into BOM=no 的位号（不贴片），不进采购表：{' '.join(excluded)}")

    parts.append("")
    parts.append("=== 采购聚合 ===")
    parts.append(render_table(
        ["立创编号", "器件", "数量", "位号"],
        [[g["lcsc"] or "—", g["device"], str(g["count"]), " ".join(g["refs"])] for g in agg]))
    parts.append(f"共 {len(agg)} 个料号，合计 {sum(g['count'] for g in agg)} 颗")

    if data.get("hand_solder"):
        parts.append("")
        parts.append("=== 手焊清单（PCB 侧，标签为 layout 时的展示值）===")
        for lay in data["hand_solder"]:
            parts.append(f"{lay['layer']}（{lay['total']} 个）")
            for item in lay["labels"]:
                parts.append(f"    {pad(clip(item['label'], 14), 16)}"
                             f"×{len(item['refs']):<3} {' '.join(item['refs'])}")
    if boards:
        parts.append("")
        parts.append("工程内 PCB 文档：" + " · ".join(
            f"{b['name']}（{b['live_components']} 元件{', 已删除' if b['deleted'] else ''}）"
            for b in boards))
    return "\n".join(parts)


def json_report(data):
    boards = data["boards"]
    return json.dumps({
        "工程": str(data["path"]),
        "板子": [{"名称": b["name"], "元件数": b["live_components"], "已删除": b["deleted"]}
                 for b in boards],
        "明细": [{k: r[k] for k in ("ref", "device", "footprint", "lcsc", "desc",
                                    "mfr", "mpn", "unique", "in_bom")}
                 for r in data["rows"]],
        "采购": aggregate(data["rows"]),
        "手焊清单": data.get("hand_solder") or [],
    }, ensure_ascii=False, indent=2)


def main(argv):
    ap = argparse.ArgumentParser(
        prog="bom.py", description="从立创EDA专业版工程（.epro2）导出元器件清单。",
        epilog="不传工程路径时，自动取 PCB/ 下最新的 .epro2。")
    ap.add_argument("project", nargs="?", metavar="工程",
                    help=".epro2 或 .epru；省略则自动找最新的")
    ap.add_argument("--json", action="store_true", help="输出 JSON 而不是三张表")
    ap.add_argument("-o", "--out", metavar="文件",
                    help="写进这个文件（UTF-8）。需要重定向时请用它——"
                         "PowerShell 的 > 默认产出 UTF-16，别的工具读不了")
    ap.add_argument("--no-pcb", action="store_true", help="跳过手焊清单")
    args = ap.parse_args(argv)

    path = Path(args.project) if args.project else find_latest_project()
    if not path.is_file():
        raise SystemExit(f"找不到工程文件：{path}")

    data = collect(path)
    data["path"] = path
    if not args.no_pcb:
        data["hand_solder"] = hand_solder(data["pcb"], data["pcb_attr"], data["rows"])
    report = json_report(data) if args.json else text_report(data)
    if args.out:
        # newline="\n"：Windows 下文本模式默认把 \n 翻成 \r\n，与本仓库的 LF 约定不符
        Path(args.out).write_text(report + "\n", encoding="utf-8", newline="\n")
        print(f"已写入 {args.out}")
    else:
        print(report)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main(sys.argv[1:])
