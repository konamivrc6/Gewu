"""加速度计六面标定：逐姿态采样 + 六面求解。

record：摆好一个姿态，采 N 秒，把均值存进 STORE，并当场报这个姿态摆得正不正
        （实测的「上」方向与构造上应有的方向之间的夹角，以及不该有分量的那两轴读数）。
solve ：把六个姿态两两配对，解出三轴的零点与增益，再检查每个姿态修正后的模长是否为 1 g。
status：看还缺哪几个姿态。

六个姿态的「上」方向在模组坐标系里是**构造给定**的（映射表见《标定.md》第五节），
不是从读数里反推的——这正是六面标定比随手摆几个角度更好的地方：随手摆的姿态取向
只能估到零点几度，而拟合会把这个误差放大；六面用的是 ±1，方向由构造给出。

摆法对不齐不要紧，record 会当场报夹角：< 3° 很好，< 8° 可用。倾角误差对零点免疫
（cos θ 在相加时抵消）、对增益不免疫（倾 8° 就少 1%），所以同一轴的正反两次要摆成
大致相同的角度。

用法：
  python tools/accel_calib.py record --pose flat_up --sec 12
  python tools/accel_calib.py status
  python tools/accel_calib.py solve

依赖 pyserial。不指定 `--port` 时按 VID/PID 自动找格物的 CH343，找不到或有多个候选会列出清单让你指定。姿态数据落在 %TEMP%\\gewu_poses.json，原始数据行落在 %TEMP%\\gewu_imu.log，都不进仓库。换板、换模块、改安装朝向后要重跑六个姿态——求解出的常数是这份硬件独有的。
"""
import argparse
import json
import math
import os
import re
import sys
import tempfile
import time

import serial

from serial_port import PortNotFound, pick_port

TMP = tempfile.gettempdir()
STORE = os.path.join(TMP, "gewu_poses.json")
RAWLOG = os.path.join(TMP, "gewu_imu.log")

ACCEL_LSB = 16384.0   # 标称灵敏度，与固件同名常量一致：AFS_SEL = 0，±2 g

# 与 imu_stat.py 同一份格式契约，改固件输出时两处都要同步。
LINE_RE = re.compile(
    r"^(MPU\d{4})\s+"
    r"ax\s+([-+0-9.]+)\s+ay\s+([-+0-9.]+)\s+az\s+([-+0-9.]+)\s+g\s+\|\s+"
    r"gx\s+([-+0-9.]+)\s+gy\s+([-+0-9.]+)\s+gz\s+([-+0-9.]+)\s+deg/s\s+\|\s+"
    r"([-+0-9.]+)\s+C\s+\(raw\s+(-?\d+)\)"
)

# 姿态名 -> (人话标签, 该姿态下世界「上」在模组坐标系里的方向)
# 方向依据第五节的映射：屏幕 x = 模组 −Y、屏幕 y = 模组 −X、法线 = 模组 −Z。
# 「立左边缘」= x=0 那条边在下，于是屏幕 +x 朝上 = 模组 −Y 朝上。
POSES = {
    "flat_up":     ("点阵面朝上、平放桌面",            (0.0,  0.0, -1.0)),
    "flat_down":   ("点阵面朝下、平放桌面",            (0.0,  0.0,  1.0)),
    "left_edge":   ("立左边缘（x=0 那条边在下）",      (0.0, -1.0,  0.0)),
    "right_edge":  ("立右边缘（x=7 那条边在下）",      (0.0,  1.0,  0.0)),
    "bottom_edge": ("立下边缘（y=0 那条边在下）",      (-1.0, 0.0,  0.0)),
    "top_edge":    ("立上边缘（y=7 那条边在下）",      (1.0,  0.0,  0.0)),
}

AXES = ["X", "Y", "Z"]
FIELDS = ["ax", "ay", "az", "gx", "gy", "gz", "tC"]
MINUS = "−"


def out(text: str) -> None:
    """直写 stdout.buffer：Windows 控制台是 GBK 而串口数据是 UTF-8，用 print 会在中文上报错。"""
    sys.stdout.flush()
    sys.stdout.buffer.write(text.encode("utf-8"))
    sys.stdout.buffer.flush()


def load() -> dict:
    if os.path.exists(STORE):
        with open(STORE, encoding="utf-8") as fh:
            return json.load(fh)
    return {}


def save(data: dict) -> None:
    with open(STORE, "w", encoding="utf-8") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2)


def sample(port: str, sec: float, reset: bool):
    try:
        s = serial.Serial(port=port, baudrate=921600, timeout=0.1)
    except Exception as exc:  # noqa: BLE001
        out(f"打不开 {port}：{exc}\n（串口监视器是不是还开着？）\n")
        return None

    rows, raw_all, tail = [], bytearray(), b""
    with s:
        if reset:
            s.setDTR(False)
            s.setRTS(True)
            time.sleep(0.2)
            s.setRTS(False)
            time.sleep(1.5)
        else:
            time.sleep(0.3)
        while s.read(4096):
            pass

        out(f"采 {sec:g} 秒，期间板子别动……\n")
        deadline = time.time() + sec
        while time.time() < deadline:
            chunk = s.read(4096)
            if not chunk:
                continue
            raw_all.extend(chunk)
            tail = (tail + chunk)[-8192:]
            while b"\n" in tail:
                raw, tail = tail.split(b"\n", 1)
                m = LINE_RE.match(raw.decode("utf-8", errors="replace").strip())
                if m:
                    rows.append([float(m.group(i)) for i in range(2, 9)])

    with open(RAWLOG, "wb") as fh:
        fh.write(raw_all)
    return rows


def stats(rows):
    n = len(rows)
    means, stds = [], []
    for i in range(7):
        col = [r[i] for r in rows]
        mu = sum(col) / n
        var = sum((v - mu) ** 2 for v in col) / n if n > 1 else 0.0
        means.append(mu)
        stds.append(math.sqrt(var))
    return means, stds


def do_record(args) -> None:
    label, expected = POSES[args.pose]
    out(f"姿态 {args.pose}：{label}\n  构造上世界「上」应为模组 "
        f"({expected[0]:+.0f}, {expected[1]:+.0f}, {expected[2]:+.0f})\n\n")

    try:
        port = pick_port(args.port)
    except PortNotFound as exc:
        out(f"{exc}\n")
        return

    rows = sample(port, args.sec, args.reset)
    if not rows:
        out("没收到数据——板子在跑吗？\n")
        return
    n = len(rows)
    means, stds = stats(rows)

    acc = means[:3]
    mag = math.sqrt(sum(v * v for v in acc))
    unit = [v / mag for v in acc]
    dot = sum(unit[i] * expected[i] for i in range(3))
    ang = math.degrees(math.acos(max(-1.0, min(1.0, dot))))

    out(f"\n收到 {n} 个数据点（{n / args.sec:.1f} Hz）\n\n")
    out("  轴      均值 g      原始计数    标准差\n")
    for i in range(3):
        out(f"  {FIELDS[i]:<4} {acc[i]:+9.3f} {acc[i] * ACCEL_LSB:+11.0f} {stds[i]:10.4f}\n")
    for i in (3, 4, 5):
        out(f"  {FIELDS[i]:<4} {means[i]:+9.3f}{'':10}{stds[i]:10.4f}\n")
    out(f"  温度 {means[6]:.1f} C\n")

    out(f"\n  |a| = {mag:.4f} g\n")
    out(f"  实测「上」方向： ({unit[0]:+.4f}, {unit[1]:+.4f}, {unit[2]:+.4f})\n")
    out(f"  与构造方向夹角： {ang:.2f}°\n")

    # 不该有分量的那两根轴，读数直接反映姿态摆得正不正
    zeros = [i for i in range(3) if abs(expected[i]) < 1e-9]
    out("  该为 0 的分量：")
    for i in zeros:
        out(f"  {FIELDS[i]} {acc[i]:+.3f} g")
    out("\n")

    verdict = "摆得很正" if ang < 3 else ("可以接受" if ang < 8 else "偏得有点多，建议重摆")
    out(f"  判：{verdict}（夹角 < 3° 很好，< 8° 可用）\n")

    data = load()
    data[args.pose] = {
        "label": label,
        "expected": list(expected),
        "mean_g": means,
        "std_g": stds,
        "n": n,
        "sec": args.sec,
        "angle_deg": ang,
    }
    save(data)
    out(f"\n已存进 {STORE}\n")


def do_status(_args) -> None:
    data = load()
    out(f"已记录 {len(data)} / {len(POSES)} 个姿态\n\n")
    for key, (label, expected) in POSES.items():
        if key in data:
            d = data[key]
            out(f"  [x] {key:<12} {label:<26} 夹角 {d['angle_deg']:5.2f}°  n={d['n']}\n")
        else:
            out(f"  [ ] {key:<12} {label}\n")
    missing = [k for k in POSES if k not in data]
    if missing:
        out(f"\n还缺：{', '.join(missing)}\n")


def do_solve(_args) -> None:
    data = load()
    missing = [k for k in POSES if k not in data]
    if missing:
        out(f"还缺这些姿态，先补齐：{', '.join(missing)}\n")
        return

    pairs = {}
    for key, (_label, expected) in POSES.items():
        for i in range(3):
            if abs(expected[i]) > 0.5:
                pairs.setdefault(i, {})[int(expected[i])] = key
    for i in range(3):
        if 1 not in pairs[i] or -1 not in pairs[i]:
            out(f"{AXES[i]} 轴缺正反一对姿态，先补齐\n")
            return

    out("六面标定结果\n" + "=" * 62 + "\n\n")
    gain, offset = [0.0] * 3, [0.0] * 3
    for i in range(3):
        up_key, dn_key = pairs[i][1], pairs[i][-1]
        raw_up = data[up_key]["mean_g"][i] * ACCEL_LSB
        raw_dn = data[dn_key]["mean_g"][i] * ACCEL_LSB
        gain[i] = (raw_up - raw_dn) / 2.0
        offset[i] = (raw_up + raw_dn) / 2.0
        out(f"  {AXES[i]} 轴： {data[up_key]['label']} / {data[dn_key]['label']}\n")
        out(f"    灵敏度 S = {gain[i]:+11.1f} 计数/g   （标称 16384，比值 {gain[i] / ACCEL_LSB:.5f}）\n")
        out(f"    零点   b = {offset[i]:+11.1f} 计数   （{offset[i] / ACCEL_LSB:+.5f} g）\n\n")

    out("固件常数表（照抄这几行）：\n\n")
    out("  static const float ACCEL_GAIN[3]   = {%s};\n" %
        ", ".join(f"{gain[i]:.1f}f" for i in range(3)))
    out("  static const float ACCEL_OFFSET[3] = {%s};\n" %
        ", ".join(f"{offset[i]:.1f}f" for i in range(3)))
    out("  // 修正： a_g[i] = (raw[i] - ACCEL_OFFSET[i]) / ACCEL_GAIN[i]\n\n")

    out("六个姿态的自检（修正后每个都应当是 1.0000 g，方向与构造一致）\n")
    out("=" * 62 + "\n\n")
    out("  姿态                    修正后模长   与构造夹角   x        y        z\n")
    worst = 0.0
    for key, (label, expected) in POSES.items():
        d = data[key]
        raw = [d["mean_g"][i] * ACCEL_LSB for i in range(3)]
        corr = [(raw[i] - offset[i]) / gain[i] for i in range(3)]
        mag = math.sqrt(sum(v * v for v in corr))
        unit = [v / mag for v in corr]
        dot = sum(unit[i] * expected[i] for i in range(3))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        worst = max(worst, abs(mag - 1.0))
        out(f"  {label:<22} {mag:8.4f} {ang:10.2f}° {corr[0]:+8.3f} {corr[1]:+8.3f} {corr[2]:+8.3f}\n")

    out(f"\n  最大模长偏差 {worst * 100:.2f}%（判据 1%）→ "
        f"{'通过' if worst < 0.01 else '不通过，看哪个姿态偏了'}\n\n")

    mags = [math.sqrt(sum(v * v for v in d["mean_g"][:3])) for d in data.values()]
    out(f"  修正前模长范围 {min(mags):.4f} ~ {max(mags):.4f} g\n")


def main() -> None:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)

    r = sub.add_parser("record", help="采样一个姿态")
    r.add_argument("--pose", required=True, choices=sorted(POSES))
    r.add_argument("--sec", type=float, default=15.0)
    r.add_argument("--port", default=None, help="串口，例如 COM7；不给就自动找 CH343")
    r.add_argument("--reset", action="store_true")
    r.set_defaults(func=do_record)

    s = sub.add_parser("solve", help="解出六个常数并自检")
    s.set_defaults(func=do_solve)

    t = sub.add_parser("status", help="看还缺哪些姿态")
    t.set_defaults(func=do_status)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
