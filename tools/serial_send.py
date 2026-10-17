"""给 Hardware_Test 发一串串口命令，把回显打出来。

调试与标定的通用入口：每轮敲几条命令，看板子怎么回的、屏上灯落在哪、IMU 读到什么。
默认**不复位**——复位会把亮度、显示模式、已经画上去的点、以及 IMU 的初始化状态全冲掉，
而标定过程中往往正需要保留这些。要干净状态就加 --reset，它会拉一次 RTS 并等开机横幅。

命令格式、参数含义见仓库根目录《标定.md》第六节。命令不区分大小写、以回车结束。

用法：
  python tools/serial_send.py '?' 'i 8' 'h 0' 'v 0'
  python tools/serial_send.py --reset '?'
  python tools/serial_send.py 'p 3 5' --port COM7 --wait 0.8

依赖 pyserial。不指定 `--port` 时按 VID/PID 自动找格物的 CH343，找不到或有多个候选会列出清单让你指定。原始回显写到 %TEMP%\\gewu_send.log（不进仓库）。
"""
import argparse
import os
import sys
import tempfile
import time

import serial

from serial_port import PortNotFound, pick_port

LOG_PATH = os.path.join(tempfile.gettempdir(), "gewu_send.log")

buf = bytearray()


def out(text: str) -> None:
    """直写 stdout.buffer。

    Windows 控制台默认是 GBK，而串口数据是 UTF-8，用 print 会在中文上报 UnicodeEncodeError。
    """
    sys.stdout.flush()
    sys.stdout.buffer.write(text.encode("utf-8"))
    sys.stdout.buffer.flush()


def drain(s: serial.Serial, seconds: float) -> None:
    deadline = time.time() + seconds
    while time.time() < deadline:
        chunk = s.read(4096)
        if chunk:
            buf.extend(chunk)


def reset_and_wait_banner(s: serial.Serial) -> bool:
    """拉 RTS 让板子复位，然后等开机横幅。

    注意：单纯打开端口**不会**复位 ESP32（已实测，打开后 millis 接着走），只有这个脉冲会。
    """
    try:
        s.setDTR(False)
        s.setRTS(True)
        time.sleep(0.2)
        s.setRTS(False)
        s.setDTR(False)
    except Exception as exc:  # noqa: BLE001
        out(f"（复位脉冲失败，忽略：{exc}）\n")

    deadline = time.time() + 6.0
    tail = b""
    while time.time() < deadline:
        chunk = s.read(512)
        tail = (tail + chunk)[-4096:]
        buf.extend(chunk)          # 横幅也收进来，--reset 时一并显示，方便核对开机提示
        if "开始：".encode("utf-8") in tail:
            time.sleep(0.3)
            while True:
                extra = s.read(4096)
                if not extra:
                    break
                buf.extend(extra)
            return True
    return False


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("cmds", nargs="+", help="逐条发送，不带行尾（脚本自己补 \\r\\n）")
    ap.add_argument("--port", default=None, help="串口，例如 COM7；不给就自动找 CH343")
    ap.add_argument("--reset", action="store_true", help="先复位并等开机横幅")
    ap.add_argument("--wait", type=float, default=0.4, help="每条命令后收多久")
    args = ap.parse_args()

    try:
        port = pick_port(args.port)
    except PortNotFound as exc:
        out(f"{exc}\n")
        return

    try:
        s = serial.Serial(port=port, baudrate=921600, timeout=0.1)
    except Exception as exc:  # noqa: BLE001
        out(f"打不开 {port}：{exc}\n（串口监视器是不是还开着？）\n")
        return

    with s:
        if args.reset:
            ok = reset_and_wait_banner(s)
            out(f"复位后开机横幅{'已就绪' if ok else '未捕获'}\n")
        else:
            # 端口刚打开时啃掉积压的数据，别把上一次会话的尾巴当成这次的回应
            drain(s, 0.3)

        for cmd in args.cmds:
            out(f">>> {cmd}\n")
            s.write((cmd + "\r\n").encode("utf-8"))
            s.flush()
            drain(s, args.wait)

        drain(s, 0.4)

    text = buf.decode("utf-8", errors="replace")
    with open(LOG_PATH, "w", encoding="utf-8") as fh:
        fh.write(text)

    # MPU 数据行每 500 ms 一行，会把回显冲得找不着，这里滤掉（要看数据用 imu_stat.py）
    keep = [ln.rstrip("\r") for ln in text.splitlines()
            if ln.strip() and not ln.startswith("MPU6")]

    out("\n================ 回显（已滤掉 IMU 数据行）================\n\n")
    if not keep:
        out("  （什么都没收到）\n")
    for ln in keep:
        out(f"  {ln}\n")
    out(f"\n原始记录已写到 {LOG_PATH}\n")


if __name__ == "__main__":
    main()
