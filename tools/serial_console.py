"""交互式串口控制台。

和 `serial_send.py` 的一次性发命令不同，这里是一个常驻的提示符，一行一条命令，边敲边看回显。串口默认 921600、`COM11`，与仓库里其它台面脚本一致。

本地命令以冒号开头，不会发给固件：

    :help            本地命令清单
    :reset           拉 RTS 复位，等开机横幅
    :port COM7       换串口（别名为 :com，也可以写 :com 7）（别名为 :com，也可以写 :com 7；不带参数报当前值）
    :filter          开关 `MPU6` 数据行过滤（默认开，给 Hardware_Test 用）
    :log             打印原始日志路径
    :quit / :q       退出，Ctrl-D 也可以

其余整行原样发给固件，例如 `s`、`g 40`、`f 10`、`v plain 240`、`p 3 4`。

用法：

    python tools/serial_console.py
    python tools/serial_console.py --port COM7
    python tools/serial_console.py --reset
    python tools/serial_console.py --no-color

依赖 pyserial。收到的原始数据边收边写到 `%TEMP%\\gewu_console.log`，不进仓库。

一个已知限制：Windows 的 `input()` 阻塞期间没法同时刷新屏幕，所以编码器事件这类异步回显会在你按下回车之后才出现，不会实时插进提示符上方。命令本身的响应是即时的。
"""
from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time

import serial

LOG_PATH = os.path.join(tempfile.gettempdir(), "gewu_console.log")

ANSI_RESET = "\x1b[0m"
ANSI_PROMPT = "\x1b[1;32mgewu\x1b[0m:\x1b[1;34m{port}\x1b[0m> "
ANSI_HOST = "\x1b[2m[host]\x1b[0m "


def use_utf8() -> None:
    """argparse 的 help 走 stdout 文本层；不重配的话中文在 GBK 控制台上会糊成乱码。"""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass


def enable_ansi() -> bool:
    """老版 conhost 默认不认 ANSI；Windows Terminal 本来就认。失败就退回无颜色。"""
    if os.name != "nt":
        return True
    try:
        import ctypes

        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
        mode = ctypes.c_uint32()
        if not kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            return False
        return bool(kernel32.SetConsoleMode(handle, mode.value | 0x0004))  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
    except Exception:  # noqa: BLE001
        return False


def normalize_port(name: str) -> str:
    """`:com 7`、`:com com7` 都当 COM7；只补数字前缀，别动 /dev/ttyUSB0 这类。"""
    s = name.strip()
    if s.isdigit():
        return "COM" + s
    if len(s) > 3 and s[:3].lower() == "com" and s[3:].isdigit():
        return "COM" + s[3:]
    return s


def out(text: str) -> None:
    """直写 stdout.buffer。串口数据是 UTF-8，Windows 控制台默认 GBK，用 print 会在中文上报错。"""
    sys.stdout.flush()
    sys.stdout.buffer.write(text.encode("utf-8", errors="replace"))
    sys.stdout.buffer.flush()


class Console:
    def __init__(self, port: str, baud: int, color: bool, quiet: float, limit: float,
                 filter_imu: bool):
        self.port = port
        self.baud = baud
        self.color = color
        self.quiet = quiet
        self.limit = limit
        self.filter_imu = filter_imu
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.buf = bytearray()
        # 日志按原始字节写，避免一次 read 正好切在多字节字符中间时把中文写坏
        self.log = open(LOG_PATH, "wb")

    # ---------- 输出 ----------
    def host(self, text: str) -> None:
        prefix = ANSI_HOST if self.color else "[host] "
        out(prefix + text + "\n")

    def print_line(self, line: str) -> None:
        if self.filter_imu and line.startswith("MPU6"):
            return
        out(line + "\n")

    def feed(self, chunk: bytes) -> None:
        """收字节、按行切分、打出来。行可能被拆在两次 read 里，所以缓冲留在 self.buf。"""
        self.buf.extend(chunk)
        self.log.write(chunk)
        self.log.flush()
        while True:
            idx = self.buf.find(b"\n")
            if idx < 0:
                break
            line = bytes(self.buf[:idx]).rstrip(b"\r").decode("utf-8", errors="replace")
            del self.buf[:idx + 1]
            self.print_line(line)

    # ---------- 收发 ----------
    def pump(self) -> None:
        """把已经到达的数据收干净，不等待。用于提示符打印之前。"""
        n = self.ser.in_waiting
        if n:
            self.feed(self.ser.read(n))

    def read_until_quiet(self) -> None:
        """收到连续 quiet 秒没有新数据为止；limit 是硬上限，防止一直有数据时回不来。"""
        start = time.monotonic()
        last = start
        while True:
            chunk = self.ser.read(4096)
            now = time.monotonic()
            if chunk:
                self.feed(chunk)
                last = now
            if now - last >= self.quiet:
                return
            if now - start >= self.limit:
                return

    def send(self, line: str) -> None:
        self.log.write(f">>> {line}\n".encode("utf-8"))
        self.log.flush()
        self.ser.write((line + "\r\n").encode("utf-8"))
        self.ser.flush()
        self.read_until_quiet()

    def reset(self) -> bool:
        self.host("复位，等开机横幅……")
        self.buf.clear()
        try:
            self.ser.setDTR(False)
            self.ser.setRTS(True)
            time.sleep(0.2)
            self.ser.setRTS(False)
            self.ser.setDTR(False)
        except Exception as exc:  # noqa: BLE001
            self.host(f"复位脉冲失败（忽略）：{exc}")

        deadline = time.monotonic() + 6.0
        tail = b""
        while time.monotonic() < deadline:
            chunk = self.ser.read(512)
            if chunk:
                tail = (tail + chunk)[-4096:]
                self.feed(chunk)
            if "开始：".encode("utf-8") in tail:
                self.read_until_quiet()
                self.host("横幅已就绪")
                return True
        self.host("6 秒内没等到开机横幅")
        return False

    def switch_port(self, port: str) -> None:
        if port.upper() == self.port.upper():
            self.host(f"已经在 {self.port}")
            return
        try:
            new = serial.Serial(port=port, baudrate=self.baud, timeout=0.05)
        except Exception as exc:  # noqa: BLE001
            self.host(f"打不开 {port}：{exc}")
            return
        self.ser.close()
        self.ser = new
        self.port = port
        self.host(f"已切到 {port}")

    # ---------- 本地帮助 ----------
    def local_help(self) -> None:
        self.host("本地命令：")
        out("  :help            这份清单\n")
        out("  :reset           拉 RTS 复位并等开机横幅\n")
        out("  :port COM7       换串口（别名为 :com，也可以写 :com 7；不带参数报当前值）\n")
        out("  :filter          开关 MPU6 数据行过滤（默认开）\n")
        out("  :log             打印原始日志路径\n")
        out("  :quit / :q       退出，Ctrl-D 也可以\n")
        out("其余整行原样发给固件，例如 `s`、`g 40`、`f 10`、`v plain 240`。\n")

    # ---------- 主循环 ----------
    def run(self) -> int:
        self.host(f"已连接 {self.port} @ {self.baud}，:help 看本地命令，Ctrl-D 退出")
        prompt = (ANSI_PROMPT.format(port=self.port) if self.color
                  else f"gewu:{self.port}> ")

        while True:
            self.pump()
            try:
                line = input(prompt)
            except KeyboardInterrupt:
                out("^C\n")
                continue
            except EOFError:
                out("\n")
                break

            line = line.strip()
            if not line:
                continue
            if line in (":q", ":quit", ":exit", "quit", "exit"):
                break
            if line in (":help", ":?"):
                self.local_help()
                continue
            if line == ":reset":
                self.reset()
                continue
            if line == ":filter":
                self.filter_imu = not self.filter_imu
                self.host(f"MPU6 数据行过滤：{'开' if self.filter_imu else '关'}")
                continue
            if line == ":log":
                self.host(LOG_PATH)
                continue
            if line.startswith(":port") or line.startswith(":com"):
                parts = line.split()
                if len(parts) == 1:
                    self.host(f"当前串口 {self.port}；用法：:port COM7 或 :com 7")
                elif len(parts) == 2:
                    self.switch_port(normalize_port(parts[1]))
                else:
                    self.host("用法：:port COM7 或 :com 7")
                continue
            if line.startswith(":"):
                self.host(f"未知本地命令 {line}，:help 查看")
                continue
            self.send(line)

        return 0

    def close(self) -> None:
        try:
            self.ser.close()
        finally:
            self.log.close()


def main(argv: list[str] | None = None) -> int:
    use_utf8()
    ap = argparse.ArgumentParser(description="交互式串口控制台")
    ap.add_argument("--port", default="COM11")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--reset", action="store_true", help="连上后先复位并等开机横幅")
    ap.add_argument("--quiet", type=float, default=0.12,
                    help="连续多少秒没有新数据就认为一条命令的回显结束")
    ap.add_argument("--limit", type=float, default=5.0, help="单条命令最多收多少秒")
    ap.add_argument("--no-filter", action="store_true", help="不过滤 MPU6 数据行")
    ap.add_argument("--no-color", action="store_true")
    args = ap.parse_args(argv)

    color = not args.no_color and enable_ansi()
    try:
        con = Console(args.port, args.baud, color, args.quiet, args.limit,
                      not args.no_filter)
    except Exception as exc:  # noqa: BLE001
        out(f"[host] 打不开 {args.port}：{exc}\n（串口监视器是不是还开着？）\n")
        return 1

    try:
        if args.reset:
            con.reset()
        return con.run()
    finally:
        con.close()


if __name__ == "__main__":
    raise SystemExit(main())
