#!/usr/bin/env python3
"""解析立创EDA专业版导出的 .epro2 / .epru 工程文件。

.epro2 是 ZIP 容器，内含 <工程名>.epru；.epru 是 JSON Lines，
每行形如 <元素头>||<数据体>，两段各自是合法的 JSON。

这里用 raw_decode 逐段切分，而不是简单地 split('||')——
字符串值里也可能出现 '||'，裸切会切错位置。

本模块同时是 `bom.py` / `netlist.py` 的公共层，凡是被两个以上脚本需要的东西都放这儿，
免得再各写一套：
    load_epru / find_latest_project   取工程文件
    iter_records                      逐条切记录
    doc_sections / iter_section_records  按 DOCHEAD 划分并归属文档段
    device_index                      器件库索引（uuid -> 器件名与库字段）
    natural_key / layer_name          位号排序、层面命名
"""

import json
import re
import sys
import zipfile
from collections import Counter
from pathlib import Path

_dec = json.JSONDecoder()


def iter_records(raw_text):
    """逐行产出 (header, body)。任一段解析失败则整行跳过并记录。"""
    for lineno, line in enumerate(raw_text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            head, end = _dec.raw_decode(line)
            rest = line[end:]
            if not rest.startswith("||"):
                yield lineno, head, None
                continue
            # 删除墓碑写作「头|||」：剥掉两个分隔符后还剩一个孤立的 "|"，
            # 不是一个空串。早先只判空串，这些行就掉进 JSONDecodeError 分支，
            # 被当成解析失败统计，会淹没真正的解析错误。
            rest = rest[2:].strip()
            if rest in ("", "|"):
                yield lineno, head, None
                continue
            body, _ = _dec.raw_decode(rest)
        except json.JSONDecodeError:
            yield lineno, None, None
            continue
        yield lineno, head, body


def load_epru(path):
    """接受 .epro2（ZIP）或裸 .epru，返回 (文件名, 文本)。"""
    path = Path(path)
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as z:
            name = next(n for n in z.namelist() if n.lower().endswith(".epru"))
            return name, z.read(name).decode("utf-8")
    return path.name, path.read_text(encoding="utf-8")


def find_latest_project(root=None):
    """挑出最新的 .epro2 工程，默认在仓库的 PCB/ 下找。

    按文件名里的日期排，取不到日期再回退文件修改时间。三个脚本裸跑时都用它兜底
    ——以前各自硬编码工程名，工程一改名（`2026-10-06` → `2026-10-12`）就集体失效。
    """
    pcb = Path(root) if root else Path(__file__).resolve().parent.parent / "PCB"
    cands = sorted(pcb.glob("*.epro2"))
    if not cands:
        raise SystemExit(f"{pcb} 下没有找到 .epro2 工程文件")

    def stamp(p):
        m = re.search(r"\d{4}-\d{2}-\d{2}", p.name)
        return (m.group(0) if m else "", p.stat().st_mtime)

    return max(cands, key=stamp)


def natural_key(text):
    """自然排序键：数字段按数值比较，让 R2 排在 R10 之前、pin2 排在 pin10 之前。

    以数字开头的排在同组前面——焊盘号 1/2/10 会聚成一簇，看网表时比字母、数字
    混排更易读。位号全部以字母开头，不受这条影响。
    """
    text = str(text)
    runs = tuple((1, int(p)) if p.isdigit() else (0, p)
                 for p in re.split(r"(\d+)", text) if p)
    return (0 if text[:1].isdigit() else 1, runs)


LAYER_NAMES = {1: "顶层", 2: "底层"}


def layer_name(layer_id):
    """PCB 层面：1/2 是顶层/底层，其余原样带出（如 '层49'）。"""
    return LAYER_NAMES.get(layer_id, f"层{layer_id}")


def doc_sections(text):
    """把记录流按 DOCHEAD 切成文档段，返回各段的元信息（按出现顺序）。

    start/end 是 text.splitlines() 的下标（左闭右开），可直接拿去切片。
    name 取段内第一条 META 的 title：器件名、封装名、PCB 板名都在那儿，
    DOCHEAD 的 body 里只有 docType/uuid，没有人类可读的名字。
    count 只数存活记录（不含 DOCHEAD 本身，也不含墓碑行）。
    """
    sections = []
    total = len(text.splitlines())
    for lineno, head, body in iter_records(text):
        if head is None or body is None:
            continue
        if head.get("type") == "DOCHEAD":
            if sections:
                sections[-1]["end"] = lineno - 1
            sections.append({"docType": body.get("docType"), "uuid": body.get("uuid"),
                             "name": None, "meta": None, "start": lineno - 1, "end": total,
                             "count": 0, "deleted": False})
            continue
        if not sections:
            continue
        sec = sections[-1]
        sec["count"] += 1
        if head.get("type") == "META" and sec["meta"] is None:
            # 段内第一条 META 既是段名（title），器件段里还挂着全部库字段
            sec["meta"] = body
            sec["name"] = body.get("title")
        elif head.get("type") == "DELETE_DOC":
            sec["deleted"] = True
    return sections


def device_index(text):
    """器件库索引：DEVICE 段 DOCHEAD 的 uuid -> (器件名, attributes 字典)。

    **不要**拿 META 记录的 head["id"] 当键——它恒为常量字符串 "META"，25 个器件会
    全部塌进同一个键互相覆盖，最后只剩一条。器件身份只能取 DOCHEAD 的 uuid，
    而原理图段 ATTR `Device` 的值正是它，所以两边能对上。
    """
    out = {}
    for sec in doc_sections(text):
        if sec["docType"] == "DEVICE":
            out[sec["uuid"]] = (sec["name"], (sec["meta"] or {}).get("attributes") or {})
    return out


def iter_section_records(text):
    """逐条产出 (所属文档段, 行号, 头, 体)，自动把记录归属到文档段。

    与 iter_records 的区别就是多带了所属段。墓碑行和解析失败行会被跳过。
    归属靠「第 N 个 DOCHEAD 对应第 N 段」定位，因此会先分段再重扫一遍；
    工程文件都在 MB 级以内，这点重复代价可以忽略。段字典此时已定稿，
    调用方可以直接读它的 name / meta / count，不会有「读到一半还在变」的问题。
    """
    sections = doc_sections(text)
    idx = -1
    for lineno, head, body in iter_records(text):
        if head is None or body is None:
            continue
        if head.get("type") == "DOCHEAD":
            idx += 1
            continue
        if 0 <= idx < len(sections):
            yield sections[idx], lineno, head, body


def survey(path):
    """打印文件里各类型元素的计数与文档段概览。"""
    name, text = load_epru(path)
    print(f"文件：{path}")
    print(f"文档：{name}    行数：{len(text.splitlines())}")
    print()

    counts = Counter()
    for _, head, body in iter_records(text):
        if head is None:
            counts["<解析失败>"] += 1
            continue
        if body is None:
            # 删除墓碑：元素已不存在，不能计入类型统计，否则计数虚高
            counts["<墓碑>"] += 1
            continue
        counts[head.get("type", "?")] += 1
    docs = doc_sections(text)

    print("=== 元素类型计数 ===")
    for t, c in counts.most_common():
        print(f"  {t:<20} {c}")
    print()
    print("=== 文档段（按出现顺序）===")
    for d in docs:
        mark = "  [已删除]" if d["deleted"] else ""
        print(f"  {d['docType']:<12} 行{d['start'] + 1:>6}起  元素{d['count']:>5}  "
              f"uuid={d['uuid']}  name={d['name']}{mark}")
    return counts, docs


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    survey(sys.argv[1] if len(sys.argv) > 1 else find_latest_project())
