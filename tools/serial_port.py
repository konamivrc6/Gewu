"""串口选择：没给 --port 时自动找格物板上的 CH343。

板子的 USB 转串口是沁恒 CH343（USB\\VID_1A86&PID_55D3）。换一台机器以后 COM 号必然变，
以前四支台面脚本都把默认值写死成 COM11，结果新机器上那个号要么不存在、要么已经是别的设备，
脚本开口就报错。这里不再猜固定编号，而是按 VID/PID 认设备；认不出来再退回「只剩一个串口就用它」，
有多个候选就列出清单让用户用 --port 指定，绝不默默打开一个错设备。

依赖 pyserial；`serial.tools.list_ports` 是它自带的枚举模块，不需要额外的包。
"""
from __future__ import annotations

import serial.tools.list_ports

# 沁恒的厂商号，以及 CH340 / CH341 / CH343 / CH9102 这批常见型号的 PID。
# 本板实测的是 1A86:55D3（CH343），其余几个是同一套驱动下的常见变体，留着以防换线或换板。
CH34X_VID = 0x1A86
CH34X_PIDS = {0x7523, 0x5523, 0x7522, 0x55D3, 0x55D4}


class PortNotFound(RuntimeError):
    """自动选端口失败。消息里已经带好候选清单，调用方直接打印即可。"""


def _is_ch34x(port) -> bool:
    if port.vid != CH34X_VID:
        return False
    return port.pid in CH34X_PIDS or "CH34" in (port.description or "").upper()


def pick_port(explicit: str | None = None) -> str:
    """命令行给了端口就原样用它；否则按「唯一 CH34x → 唯一串口 → 报错」的顺序选。"""
    if explicit:
        return explicit

    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    ch34x = [p for p in ports if _is_ch34x(p)]
    if len(ch34x) == 1:
        return ch34x[0].device

    # 找不出唯一的 CH34x 时，只有在整个系统也只剩一个串口的情况下才敢替用户做主。
    candidates = ch34x if ch34x else ports
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        raise PortNotFound("没找到任何串口：板子插好了吗？也可以用 --port 手动指定。")

    listing = "\n".join(f"  {p.device}  {p.description}" for p in candidates)
    raise PortNotFound("无法自动确定格物的串口，请用 --port 指定：\n" + listing)
