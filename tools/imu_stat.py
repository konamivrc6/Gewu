"""抓一段 Hardware_Test 的 IMU 数据行，做静止均值统计。

标定用的会话工具：摆好一个姿态，跑这个脚本收 N 秒，它把 MPU6xxx 数据行解析出来，
逐轴给均值、原始计数、标准差、极值，再算加速度模长、世界「上」在模组坐标系里的单位方向、
以及板面偏离水平的角度。

默认**不复位**（复位会冲掉亮度、模式、屏上已画的内容），IMU 数据行本来就在主循环里
无条件输出，不需要任何命令触发。要连贯的静止样本就把板子摆好别碰，--sec 给长一点。

判据与用法见仓库根目录《标定.md》第二、三、五节。

用法：
  python tools/imu_stat.py --sec 12 --label '点阵面朝上平放'
  python tools/imu_stat.py --sec 60 --port COM7

依赖 pyserial。原始数据行写到 %TEMP%\\gewu_imu.log（不进仓库）。
"""
import argparse
import math
import os
import re
import sys
import tempfile
import time

import serial

RAWLOG = os.path.join(tempfile.gettempdir(), "gewu_imu.log")

ACCEL_LSB = 16384.0   # 与固件同名常量一致：AFS_SEL = 0，±2 g
GYRO_LSB = 131.0      # FS_SEL = 0，±250 dps

# MPU6500 ax -0.12 ay +0.00 az -1.04 g  |  gx -2.0 gy +3.5 gz -1.0 deg/s  |  27.8 C (raw 3401)
# 这条格式是固件与本文具之间的契约，改固件时这里要同步。
LINE_RE = re.compile(
    r"^(MPU\d{4})\s+"
    r"ax\s+([-+0-9.]+)\s+ay\s+([-+0-9.]+)\s+az\s+([-+0-9.]+)\s+g\s+\|\s+"
    r"gx\s+([-+0-9.]+)\s+gy\s+([-+0-9.]+)\s+gz\s+([-+0-9.]+)\s+deg/s\s+\|\s+"
    r"([-+0-9.]+)\s+C\s+\(raw\s+(-?\d+)\)"
)

KEYS = ["ax", "ay", "az", "gx", "gy", "gz", "tC"]
UNITS = ["g", "g", "g", "dps", "dps", "dps", "C"]
AXES = ["X", "Y", "Z"]

MINUS = "−"   # 全角减号，跟固件输出里的负号一致


def out(text: str) -> None:
    """直写 stdout.buffer：Windows 控制台是 GBK 而串口数据是 UTF-8，用 print 会在中文上报错。"""
    sys.stdout.flush()
    sys.stdout.buffer.write(text.encode("utf-8"))
    sys.stdout.buffer.flush()


def sgn(v: float, fmt: str = "+.4f") -> str:
    return format(v, fmt).replace("-", MINUS)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sec", type=float, default=8.0, help="采多久（默认 8 秒）")
    ap.add_argument("--port", default="COM11")
    ap.add_argument("--label", default="", help="这个姿态的名字，只用于回显")
    ap.add_argument("--reset", action="store_true", help="先复位并等开机横幅")
    args = ap.parse_args()

    try:
        s = serial.Serial(port=args.port, baudrate=921600, timeout=0.1)
    except Exception as exc:  # noqa: BLE001
        out(f"打不开 {args.port}：{exc}\n（串口监视器是不是还开着？）\n")
        return

    rows = []
    raw_all = bytearray()
    tail = b""

    with s:
        if args.reset:
            s.setDTR(False)
            s.setRTS(True)
            time.sleep(0.2)
            s.setRTS(False)
            time.sleep(1.5)
        else:
            time.sleep(0.3)

        while s.read(4096):
            pass

        head = f"姿态：{args.label}\n" if args.label else ""
        out(f"{head}采 {args.sec:g} 秒，期间板子别动……\n")
        deadline = time.time() + args.sec
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

    n = len(rows)
    if n == 0:
        out(f"\n一个数据点都没解析出来——板子是不是没在跑？原始记录在 {RAWLOG}\n")
        return

    out(f"\n收到 {n} 个数据点（约 {n / args.sec:.1f} Hz，标称 2 Hz / 每 500 ms 一行）\n\n")

    means = []
    out("  轴      均值          原始计数     标准差     最小        最大\n")
    for i, key in enumerate(KEYS):
        col = [r[i] for r in rows]
        mu = sum(col) / n
        var = sum((v - mu) ** 2 for v in col) / n if n > 1 else 0.0
        means.append(mu)
        lsb = "" if i == 6 else f"{mu * (ACCEL_LSB if i < 3 else GYRO_LSB):+10.0f}"
        out(f"  {key:<4} {mu:+9.3f} {UNITS[i]:<5} {lsb:>10}  "
            f"{math.sqrt(var):7.3f} {min(col):+10.3f} {max(col):+10.3f}\n")

    mx, my, mz = means[0], means[1], means[2]
    amag = math.sqrt(mx * mx + my * my + mz * mz)
    out(f"\n  加速度模长 |a| = {amag:.4f} g，偏离 1 g {(amag - 1.0) * 100:+.2f}%\n")
    out("  （未修正的原始模长；标定后的修正见 accel_calib.py）\n")

    # 静止时加速度计测的是支持力，方向朝上，读数矢量就是世界「上」在模组系里的方向。
    up = [mx / amag, my / amag, mz / amag]
    idx = max(range(3), key=lambda k: abs(up[k]))
    # 读数分量沿 +k 为正，说明 +k 轴有朝上的分量；为负则 −k 朝上
    who = f"{'+' if up[idx] > 0 else MINUS}{AXES[idx]}"
    where = "朝上（朝天）" if up[idx] > 0 else "朝下（朝地）"
    out(f"\n  读数最集中的轴：{KEYS[idx]} = {means[idx]:+.3f} g  →  模组 {who} 轴{where}\n")
    out(f"  |{KEYS[idx]}| / |a| = {abs(up[idx]):.4f}，即与竖直线夹角 "
        f"{math.degrees(math.acos(min(1.0, abs(up[idx])))):.2f}°\n")

    tilt = math.degrees(math.atan2(math.hypot(mx, my), abs(mz)))
    out(f"  板面（模组 Z 轴）偏离水平 {tilt:.2f}°\n")

    out("\n  世界「上」在模组坐标系里的单位方向（加速度计读数方向）：\n")
    for i in range(3):
        out(f"    {KEYS[i]}  {sgn(up[i])}\n")
    out("  世界「下」（重力）方向 = 上面取反：\n")
    for i in range(3):
        out(f"    {KEYS[i]}  {sgn(-up[i])}\n")

    out("\n  角速度：\n")
    for i in (3, 4, 5):
        out(f"    {KEYS[i]}  {means[i]:+7.3f} dps\n")
    out(f"  温度 {means[6]:.1f} °C\n")

    out(f"\n原始记录已写到 {RAWLOG}\n")


if __name__ == "__main__":
    main()
