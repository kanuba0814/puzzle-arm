#!/usr/bin/env python3
"""
STS3215 舵机 bring-up 命令行工具 (独立运行, 不依赖项目其他模块)

用途: 树莓派(或任何Linux机器)上对 SO-ARM101 的 5 个 Feetech STS3215 舵机做
总线扫描 / 通信验证 / ID标定 / 中位与方向标定。

设计原则:
  - 默认只读不写; 写操作(setid/center/nudge)有显式子命令并打印 ⚠️ 提示
  - 不 import config/kinematics, 只依赖 pyserial + scservo_sdk
  - scservo_sdk 未安装时自动使用 sdks/offline/ 内的拷贝

用法示例:
  python3 tools/servo_tool.py scan                 # 扫描总线上的舵机
  python3 tools/servo_tool.py ping 1 2 3 4 5       # 通信验证(型号/电压/温度)
  python3 tools/servo_tool.py read                 # 读 5 舵机 位置/负载/电压
  python3 tools/servo_tool.py setid 1 3            # ⚠️ 把 ID=1 的舵机改为 ID=3
  python3 tools/servo_tool.py center               # ⚠️ 全部舵机慢速转到中位 2048
  python3 tools/servo_tool.py nudge 2 +100         # ⚠️ 舵机2 在当前位置+100
"""
import argparse
import glob
import sys
import time
from pathlib import Path

# scservo_sdk 优先用系统已安装的; 否则回退到项目内置离线拷贝
# (注意 scservo_sdk 依赖 pyserial 的 import serial, 一并回退)
_REPO_ROOT = Path(__file__).resolve().parent.parent
_OFFLINE = _REPO_ROOT / 'sdks' / 'offline'
try:
    import serial  # noqa: F401
except ImportError:
    sys.path.insert(0, str(_OFFLINE / 'pyserial_fallback'))
try:
    from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    sys.path.insert(0, str(_OFFLINE))
    try:
        from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    except ImportError:
        print("[错误] 找不到 scservo_sdk/pyserial, 请先运行 tools/setup_pi.sh")
        sys.exit(1)

# ==================== STS3215 寄存器表 (对照 Feetech 官方控制表) ====================
ADDR_MODEL_NUMBER = 3      # 2B, 只读, STS3215 = 777
ADDR_ID = 5                # 1B, EPROM
ADDR_TORQUE_ENABLE = 40    # 1B, SRAM: 0=释放 1=使能
ADDR_GOAL_POSITION = 42    # 2B, 0-4095
ADDR_GOAL_VELOCITY = 46    # 2B, 速度 (单位~0.0146rpm)
ADDR_LOCK = 55             # 1B, EPROM锁: 写0解锁后可改EPROM
ADDR_PRESENT_POSITION = 56  # 2B, 只读
ADDR_PRESENT_LOAD = 60     # 2B, 只读, 低10位幅值+bit10方向
ADDR_PRESENT_VOLTAGE = 62  # 1B, 只读, 单位 0.1V
ADDR_PRESENT_TEMP = 63     # 1B, 只读, 单位 °C
ADDR_MOVING = 66           # 1B, 只读

STS3215_MODEL_NUMBER = 777
SERVO_MIDDLE = 2048        # 0-4095 的中位
SAFE_SPEED = 500           # 标定阶段的慢速 (正常运行可用 2000+)
NUDGE_MAX = 200            # nudge 单次最大步长 (硬限制, 防止误输入甩臂)
DEFAULT_IDS = [1, 2, 3, 4, 5]

PROTOCOL_END = 0           # STS/SMS 系列 = 0


# ==================== 底层辅助 ====================

def find_port(prefer=None):
    """自动探测舵机驱动板串口"""
    if prefer:
        return prefer
    cands = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    if not cands:
        sys.exit("[错误] 未发现 /dev/ttyUSB* 或 /dev/ttyACM*\n"
                 "  请检查: 驱动板USB是否插好? `lsusb` 能否看到设备?")
    if len(cands) > 1:
        sys.exit(f"[错误] 检测到多个串口设备 {cands}\n"
                 "  请用 --port 明确指定舵机驱动板对应的那个")
    return cands[0]


def open_bus(port_name, baudrate):
    """打开串口总线, 返回 (port, packet_handler)"""
    port = PortHandler(port_name)
    if not port.openPort():
        sys.exit(f"[错误] 串口打开失败: {port_name}\n"
                 "  请检查: 1) 设备是否存在 2) 当前用户是否在 dialout 组 "
                 "(`groups` 查看, 不在则 sudo usermod -aG dialout $USER 后重新登录)")
    if not port.setBaudRate(baudrate):
        port.closePort()
        sys.exit(f"[错误] 波特率设置失败: {baudrate}")
    ph = PacketHandler(PROTOCOL_END)
    return port, ph


def check_comm(ph, result, error, what):
    """统一的通信结果检查, 失败时打印可读信息"""
    if result != COMM_SUCCESS:
        print(f"  [通信失败] {what}: {ph.getTxRxResult(result)}")
        return False
    if error != 0:
        print(f"  [舵机报错] {what}: {ph.getRxPacketError(error)}")
        return False
    return True


def decode_load(raw):
    """Present_Load 解码: 低10位为幅值, bit10(0x400)为方向位"""
    magnitude = raw & 0x3FF
    return -magnitude if raw & 0x400 else magnitude


# ==================== 子命令 ====================

def cmd_scan(args):
    """扫描总线上的舵机 (只读)"""
    port_name = find_port(args.port)
    bauds = [int(b) for b in args.baud.split(',')]
    for baud in bauds:
        print(f"[Scan] {port_name} @ {baud}bps, 扫描 ID 1-{args.max_id} ...")
        port, ph = open_bus(port_name, baud)
        port.setPacketTimeout(50)  # 缩短超时加快扫描
        found = []
        for sid in range(1, args.max_id + 1):
            model, result, _ = ph.ping(port, sid)
            if result == COMM_SUCCESS:
                found.append((sid, model))
                print(f"  ✓ 发现舵机: ID={sid}, 型号={model}"
                      f"{'(STS3215)' if model == STS3215_MODEL_NUMBER else ''}")
        port.closePort()
        if found:
            ids = [f[0] for f in found]
            print(f"[Scan] 共 {len(found)} 个: {ids}")
            if len(ids) != len(set(ids)):
                print("  ⚠️ 存在重复ID! (出厂舵机默认全是1, 需逐个插拔用 setid 标定)")
        else:
            print("[Scan] 未发现舵机。检查: 舵机电源是否接通? 波特率是否被改过 "
                  "(试 --baud 1000000,115200)?")
        return


def cmd_ping(args):
    """逐 ID ping + 读型号/电压/温度 (只读)"""
    port, ph = open_bus(find_port(args.port), args.baud_single)
    ok = 0
    for sid in args.ids:
        model, result, error = ph.ping(port, sid)
        if not check_comm(ph, result, error, f"ping ID={sid}"):
            continue
        voltage, r1, e1 = ph.read1ByteTxRx(port, sid, ADDR_PRESENT_VOLTAGE)
        temp, r2, e2 = ph.read1ByteTxRx(port, sid, ADDR_PRESENT_TEMP)
        model_ok = "✓" if model == STS3215_MODEL_NUMBER else \
            f"⚠️ 型号={model} (STS3215应为{STS3215_MODEL_NUMBER}; 若数值像字节交换结果说明协议端序不对)"
        print(f"  ID={sid}: 型号={model} {model_ok}, "
              f"电压={voltage/10:.1f}V, 温度={temp}°C")
        ok += 1
    port.closePort()
    print(f"[Ping] {ok}/{len(args.ids)} 通信正常")
    sys.exit(0 if ok == len(args.ids) else 1)


def cmd_read(args):
    """读舵机 位置/负载/电压/温度/运动标志 (只读)"""
    port, ph = open_bus(find_port(args.port), args.baud_single)
    ids = args.ids or DEFAULT_IDS
    print(f"{'ID':>4} {'位置':>6} {'负载':>7} {'电压':>6} {'温度':>4} {'运动中':>4}")
    for sid in ids:
        pos, r1, _ = ph.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
        load, r2, _ = ph.read2ByteTxRx(port, sid, ADDR_PRESENT_LOAD)
        volt, r3, _ = ph.read1ByteTxRx(port, sid, ADDR_PRESENT_VOLTAGE)
        temp, r4, _ = ph.read1ByteTxRx(port, sid, ADDR_PRESENT_TEMP)
        mov, r5, _ = ph.read1ByteTxRx(port, sid, ADDR_MOVING)
        if COMM_SUCCESS in (r1, r2, r3, r4, r5):
            print(f"{sid:>4} {pos:>6} {decode_load(load):>7} "
                  f"{volt/10:>5.1f}V {temp:>4}° {'是' if mov else '否':>4}")
        else:
            print(f"{sid:>4} 读取失败 (结果码 r1={r1} r2={r2} r3={r3})")
    port.closePort()


def cmd_setid(args):
    """⚠️ 写操作: 修改舵机 ID (需总线上只有这一个舵机!)"""
    old, new = args.old_id, args.new_id
    if not (1 <= new <= 252):
        sys.exit("[错误] 新ID必须在 1-252")
    port, ph = open_bus(find_port(args.port), args.baud_single)

    print("=" * 55)
    print(f"⚠️  即将把舵机 ID {old} -> {new}")
    print("⚠️  请确认总线上【只插了这一个舵机】, 否则可能改错!")
    print("=" * 55)

    model, result, error = ph.ping(port, old)
    if not check_comm(ph, result, error, f"ping 旧ID={old}"):
        port.closePort()
        sys.exit("[中止] 旧ID无响应, 未做任何修改")
    print(f"  旧ID={old} 在线 (型号={model})")

    # 解锁 EPROM (部分固件需要; 失败不致命, 继续尝试)
    ph.write1ByteTxRx(port, old, ADDR_LOCK, 0)

    result, error = ph.write1ByteTxRx(port, old, ADDR_ID, new)
    if not check_comm(ph, result, error, "写ID寄存器"):
        port.closePort()
        sys.exit("[失败] ID写入失败")

    time.sleep(0.1)
    model, result, error = ph.ping(port, new)
    if check_comm(ph, result, error, f"ping 新ID={new}"):
        print(f"[OK] ID 已修改: {old} -> {new} (型号={model})")
        print(f"  ➡️  请立即在此舵机外壳贴上标签: 【ID {new}】")
    port.closePort()


def _write_slow(port, ph, sid, speed=SAFE_SPEED):
    ph.write2ByteTxRx(port, sid, ADDR_GOAL_VELOCITY, speed)


def cmd_center(args):
    """⚠️ 运动: 全部舵机慢速转到中位 2048"""
    ids = args.ids or DEFAULT_IDS
    print("=" * 55)
    print(f"⚠️  舵机 {ids} 将以低速转到中位 {SERVO_MIDDLE}")
    print("⚠️  请确认: 已手扶机械臂, 运动空间无遮挡, 手在电源开关旁")
    print("=" * 55)
    time.sleep(2)  # 留出读警告的时间

    port, ph = open_bus(find_port(args.port), args.baud_single)
    for sid in ids:  # 先限速再发位置, 保证第一下运动也是慢的
        _write_slow(port, ph, sid)
    for sid in ids:
        result, error = ph.write2ByteTxRx(port, sid, ADDR_GOAL_POSITION, SERVO_MIDDLE)
        check_comm(ph, result, error, f"center ID={sid}")
    port.closePort()
    print(f"[OK] 已发送中位指令, 等待 {args.wait}s 后舵机到位")


def cmd_nudge(args):
    """⚠️ 运动: 单舵机在当前位置 ±delta (方向标定用)"""
    sid, delta = args.servo_id, args.delta
    if abs(delta) > NUDGE_MAX:
        sys.exit(f"[错误] 单次步长 |delta| ≤ {NUDGE_MAX} (安全硬限制)")
    port, ph = open_bus(find_port(args.port), args.baud_single)

    pos, result, error = ph.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
    if not check_comm(ph, result, error, f"读位置 ID={sid}"):
        port.closePort()
        sys.exit(1)

    target = max(0, min(4095, pos + delta))
    print(f"⚠️  舵机{sid}: {pos} -> {target} (delta={delta:+d})")
    _write_slow(port, ph, sid)
    result, error = ph.write2ByteTxRx(port, sid, ADDR_GOAL_POSITION, target)
    check_comm(ph, result, error, f"nudge ID={sid}")
    time.sleep(1.0)
    new_pos, _, _ = ph.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
    print(f"[OK] 到位: {new_pos} (请肉眼核对转动方向并记录)")
    port.closePort()


def cmd_torque(args):
    """⚠️ 力矩使能/释放 (off=手掰标定, on=恢复锁定)"""
    ids = args.ids or DEFAULT_IDS
    enable = 1 if args.state == 'on' else 0
    port, ph = open_bus(find_port(args.port), args.baud_single)
    for sid in ids:
        result, error = ph.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, enable)
        check_comm(ph, result, error, f"torque {args.state} ID={sid}")
    port.closePort()
    if enable:
        print(f"[OK] 舵机 {ids} 力矩已使能 (锁定在当前位置)")
    else:
        print(f"[OK] 舵机 {ids} 力矩已释放 (可手掰; 请扶好机械臂)")


# ==================== 入口 ====================

def main():
    parser = argparse.ArgumentParser(
        description='STS3215 舵机 bring-up 工具 (SO-ARM101, 5舵机+电磁铁方案)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument('--port', help='串口路径 (默认自动探测 /dev/ttyUSB*,ttyACM*)')
    sub = parser.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('scan', help='扫描总线舵机 (只读)')
    p.add_argument('--baud', default='1000000', help='逗号分隔, 如 1000000,115200')
    p.add_argument('--max-id', type=int, default=20, help='扫描ID上限(默认20, 全扫用252)')
    p.set_defaults(func=cmd_scan)

    p = sub.add_parser('ping', help='ping+型号/电压/温度 (只读)')
    p.add_argument('ids', type=int, nargs='+', help='舵机ID列表')
    p.set_defaults(func=cmd_ping)

    p = sub.add_parser('read', help='读位置/负载/电压 (只读)')
    p.add_argument('ids', type=int, nargs='*', help='默认 1-5')
    p.set_defaults(func=cmd_read)

    p = sub.add_parser('setid', help='⚠️ 修改舵机ID (总线只插1个舵机时运行)')
    p.add_argument('old_id', type=int)
    p.add_argument('new_id', type=int)
    p.set_defaults(func=cmd_setid)

    p = sub.add_parser('center', help='⚠️ 全部舵机慢速转到中位2048')
    p.add_argument('ids', type=int, nargs='*', help='默认 1-5')
    p.add_argument('--wait', type=float, default=3.0)
    p.set_defaults(func=cmd_center)

    p = sub.add_parser('nudge', help='⚠️ 单舵机微动 (方向标定)')
    p.add_argument('servo_id', type=int)
    p.add_argument('delta', type=int, help='±步长, |delta|<=200')
    p.set_defaults(func=cmd_nudge)

    p = sub.add_parser('torque', help='⚠️ 力矩 on=使能锁定 / off=释放可手掰')
    p.add_argument('state', choices=['on', 'off'])
    p.add_argument('ids', type=int, nargs='*', help='默认 1-5')
    p.set_defaults(func=cmd_torque)

    args = parser.parse_args()
    # 单波特率参数 (除 scan 外的子命令共用)
    args.baud_single = 1000000
    args.func(args)


if __name__ == '__main__':
    main()
