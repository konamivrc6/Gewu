#!/usr/bin/env python3
"""解析立创EDA专业版导出的 .epro2 / .epru 工程文件。

.epro2 是 ZIP 容器，内含 <工程名>.epru；.epru 是 JSON Lines，
每行形如 <元素头>||<数据体>，两段各自是合法的 JSON。

这里用 raw_decode 逐段切分，而不是简单地 split('||')——
字符串值里也可能出现 '||'，裸切会切错位置。
"""

import json
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


def survey(path):
    """"打印文件里各类型元素的计数与首个样例。"""
    name, text = load_epru(path)
    print(f"文件：{path}")
    print(f"文档：{name}    行数：{len(text.splitlines())}")
    print()

    counts = Counter()
    docs = []          # DOCHEAD 划分出的文档段
    cur = None
    for lineno, head, body in iter_records(text):
        if head is None:
            counts["<解析失败>"] += 1
            continue
        if body is None:
            # 删除墓碑：元素已不存在，不能计入类型统计，否则计数虚高
            counts["<墓碑>"] += 1
            continue
        t = head.get("type", "?")
        counts[t] += 1
        if t == "DOCHEAD":
            # DOCHEAD 行的 body 里带 docType，用它标记后续元素归属
            cur = body.get("docType") if body else "?"
            docs.append({"docType": cur, "uuid": body.get("uuid") if body else None,
                         "start": lineno, "count": 0})
        elif docs:
            docs[-1]["count"] += 1

    print("=== 元素类型计数 ===")
    for t, c in counts.most_common():
        print(f"  {t:<20} {c}")
    print()
    print("=== 文档段（按出现顺序）===")
    for d in docs:
        print(f"  {d['docType']:<12} 行{d['start']:>6}起  元素{d['count']:>5}  uuid={d['uuid']}")
    return counts, docs


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    survey(sys.argv[1] if len(sys.argv) > 1 else r"T:/ProPrj_Gewu_2026-10-03.epro2")
