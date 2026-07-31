#!/usr/bin/env python3
"""
SO-ARM101 舵机标定向导 v3 (阶段5, 参考 LeRobot SO 标定流程)

在树莓派上运行:
  cd ~/puzzle && source venv/bin/activate
  python3 tools/calibrate_arm.py

流程 (对照 LeRobot so_follower.calibrate()):
  Step 1  基准零位记录   ~ LeRobot: "Move robot to the middle of its range
                          of motion and press ENTER" + set_half_turn_homings
                          (我们把零偏存软件 config, 不写舵机EPROM, 可重复运行)
  Step 2  方向标定       我们的补充: LeRobot 的方向约定(drive_mode)出厂已知,
                          我们的FK方向需实测: 每关节向"正方向"掰一下即可
  Step 3  全范围记录     ~ LeRobot: record_ranges_of_motion()
                          所有关节依次掰到整个行程极限, 实时 MIN|POS|MAX 表格,
                          时间不限, 回车结束
  Step 4  (可选)映射验证  小幅自转(钳制在已测范围内, 安全)核对"哪个关节动"
  Step 5  输出 calibration_result.json + 汇总, 恢复力矩

全程手掰(除可选Step4), 无时间限制; Ctrl+C 随时中止。
"""
import json
import select
import sys
import time
from pathlib import Path

from servo_tool import (find_port, open_bus, COMM_SUCCESS,
                        ADDR_PRESENT_POSITION, ADDR_TORQUE_ENABLE,
                        ADDR_GOAL_POSITION, ADDR_GOAL_VELOCITY,
                        SAFE_SPEED)

IDS = [1, 2, 3, 4, 5]
RESULT_FILE = Path(__file__).resolve().parent.parent / 'calibration_result.json'
POLL_DT = 0.05

# 关节定义: key, 显示名, 默认ID(按SO-ARM101惯例), 正方向物理描述
JOINTS = [
    ('J1', '底座旋转', 1, '从上往下看【逆时针】转'),
    ('J2', '肩关节',   2, '大臂【向上抬】'),
    ('J3', '肘关节',   3, '小臂【向上弯】'),
    ('J4', '腕俯仰',   4, '腕部末端【向上抬】'),
    ('J5', '腕旋转',   5, '从机械臂正后方看【顺时针】拧'),
]


class Bus:
    def __init__(self):
        self.port, self.ph = open_bus(find_port(None), 1000000)

    def read_pos(self, sid):
        v, r, _ = self.ph.read2ByteTxRx(self.port, sid, ADDR_PRESENT_POSITION)
        return v if r == COMM_SUCCESS else None

    def read_all(self):
        return {sid: v for sid in IDS
                if (v := self.read_pos(sid)) is not None}

    def torque(self, enable):
        for sid in IDS:
            self.ph.write1ByteTxRx(self.port, sid, ADDR_TORQUE_ENABLE,
                                   1 if enable else 0)

    def move_slow(self, sid, raw):
        raw = int(max(0, min(4095, raw)))
        self.ph.write2ByteTxRx(self.port, sid, ADDR_GOAL_VELOCITY, SAFE_SPEED)
        self.ph.write2ByteTxRx(self.port, sid, ADDR_GOAL_POSITION, raw)

    def close(self):
        self.port.closePort()


def ask(prompt, valid):
    while True:
        r = input(prompt).strip().lower()
        if r in valid:
            return r
        print(f"  请输入: {'/'.join(v for v in valid if v)}")


def enter_pressed():
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        sys.stdin.readline()
        return True
    return False


def median_sample(bus, sid, n=12):
    vals = []
    for _ in range(n):
        v = bus.read_pos(sid)
        if v is not None:
            vals.append(v)
        time.sleep(0.04)
    return sorted(vals)[len(vals) // 2] if vals else None


# ==================== Step 1: 基准零位 (LeRobot: middle + half_turn_homings) ====

def step_zero(bus):
    print("\n" + "=" * 60)
    print("【Step 1/5】基准零位记录")
    print("  力矩已释放。请把机械臂摆到基准零位:")
    print("    · 底座: 朝正前方(对准工作区)")
    print("    · 大臂: 竖直向上")
    print("    · 小臂: 水平向前伸直")
    print("    · 腕部: 末端水平(电磁铁面朝正前方)")
    print("=" * 60)
    input("  摆好并保持后按回车 >>>")
    print("  保持不动, 采样中...")
    time.sleep(1.5)

    zero = {}
    for sid in IDS:
        zero[sid] = median_sample(bus, sid)
    offsets = {sid: zero[sid] - 2048 for sid in IDS if zero[sid] is not None}
    for jkey, jname, did, _ in JOINTS:
        print(f"    {jname}(ID{did}): raw={zero[did]:4d} -> OFFSETS={offsets[did]:+5d}")
    return zero, offsets


# ==================== Step 2: 方向标定 (我们的补充) ====================

def step_signs(bus):
    print("\n" + "=" * 60)
    print("【Step 2/5】方向标定")
    print("  按提示把关节朝【指定方向】掰动(幅度大一些), 然后按回车。")
    print("  其他关节动不动无所谓, 只看指定关节。")
    print("=" * 60)
    input("  准备好后按回车开始 >>>")

    signs = {}
    for jkey, jname, did, jdesc in JOINTS:
        while True:
            print(f"\n  >>> 把【{jname}】朝 {jdesc} 方向掰, 掰完按回车 <<<")
            base = bus.read_pos(did)
            time.sleep(0.1)
            peak, peak_dev = base, 0
            while not enter_pressed():
                v = bus.read_pos(did)
                if v is not None and abs(v - base) > peak_dev:
                    peak, peak_dev = v, abs(v - base)
                time.sleep(POLL_DT)
            delta = peak - base
            if abs(delta) < 60:
                r = ask(f"  ⚠️ 变化太小({delta:+d}), 回车重试, 或输s跳过(记+1): ",
                        ['', 's'])
                if r == 's':
                    signs[did] = 1
                    break
                continue
            signs[did] = 1 if delta > 0 else -1
            print(f"  ✓ raw变化 {delta:+d} -> SERVO_SIGNS[{did}]={signs[did]}")
            break
    return signs


# ============ Step 3: 全范围记录 (LeRobot: record_ranges_of_motion) ============

def step_ranges(bus, offsets):
    print("\n" + "=" * 60)
    print("【Step 3/5】关节最大幅度范围记录 (LeRobot方式)")
    print("  请【依次】把每个关节缓慢掰到两个方向的机械极限(不要蛮力),")
    print("  每个关节反复 2-3 次, 五个都掰到。")
    print("  时间不限。屏幕实时显示各关节 [最小|当前|最大],")
    print("  确认都不再变化后, 按回车结束。")
    print("=" * 60)
    input("  准备好后按回车开始记录 >>>")

    start = bus.read_all()
    lo, hi = dict(start), dict(start)
    lines = len(JOINTS) + 3
    first = True
    while True:
        pos = bus.read_all()
        for sid, v in pos.items():
            lo[sid] = min(lo[sid], v)
            hi[sid] = max(hi[sid], v)

        if not first:
            print(f"\x1b[{lines}A", end='')  # 光标上移, 原地重绘(LeRobot同款)
        first = False
        print("  " + "-" * 46)
        print(f"  {'关节':<8} | {'MIN':>6} | {'POS':>6} | {'MAX':>6}")
        for jkey, jname, did, _ in JOINTS:
            p = pos.get(did, '  --')
            print(f"  {jname:<8} | {lo[did]:>6} | {p:>6} | {hi[did]:>6}")
        sys.stdout.flush()

        if enter_pressed():
            break
        time.sleep(POLL_DT)

    print("\n  范围记录结果:")
    bad = []
    for jkey, jname, did, _ in JOINTS:
        span = hi[did] - lo[did]
        if span < 500:
            bad.append(jname)
        print(f"    {jname}(ID{did}): [{lo[did]:4d} ~ {hi[did]:4d}] 跨度={span}"
              + ("  ⚠️很小,可能没掰到" if span < 500 else ""))
    if bad:
        print(f"  ⚠️ {bad} 的范围异常小, 告诉 Claude 或重新运行本向导")
    return lo, hi


# ==================== Step 4: (可选) 映射验证 ====================

def step_verify_mapping(bus, lo, hi, signs):
    print("\n" + "=" * 60)
    print("【Step 4/5】(可选) ID↔关节映射验证")
    print("  按惯例 ID 1-5 = 底座/肩/肘/腕俯仰/腕旋转。")
    print("  每个舵机将【慢速小幅自转】(幅度≤60, 且钳制在你刚测的")
    print("  安全范围内), 你确认动的是不是名义上的关节。")
    print("=" * 60)
    r = ask("  要做映射验证吗? [y=做/n=跳过, 按惯例信任] ", ['y', 'n'])
    if r == 'n':
        return {}

    bus.torque(True)  # 自转需要力矩
    print("  (力矩已临时使能)")
    mismatches = {}
    for jkey, jname, did, _ in JOINTS:
        p0 = bus.read_pos(did)
        if p0 is None:
            continue
        # 在已测范围内选 ±60 中不越界的方向, 留30余量
        margin_lo, margin_hi = lo[did] + 30, hi[did] - 30
        delta = 60 if p0 + 60 <= margin_hi else -60
        if not (margin_lo <= p0 + delta <= margin_hi):
            print(f"  {jname}(ID{did}): 当前位置贴近极限, 跳过自转")
            continue
        print(f"\n  舵机ID{did} (惯例={jname}) 将小幅自转...")
        bus.move_slow(did, p0 + delta)
        time.sleep(1.5)
        bus.move_slow(did, p0)
        time.sleep(1.5)
        ok = ask(f"  刚才动的是【{jname}】吗? [y=是/n=不是] ", ['y', 'n'])
        if ok == 'n':
            mismatches[jkey] = did
            print("  已记录不匹配, 标定后告诉 Claude 调整映射")
    return mismatches


# ==================== 主流程 ====================

def main():
    print("=" * 60)
    print("  SO-ARM101 舵机标定向导 v3 (LeRobot流程, 全程手掰)")
    print("  Ctrl+C 随时中止; 中止后恢复力矩: tools/servo_tool.py torque on")
    print("=" * 60)
    bus = Bus()
    result = {'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
              'method': 'lerobot-style (manual, software offsets)'}

    try:
        snap = bus.read_all()
        print(f"  当前位置: {snap}")
        if len(snap) != len(IDS):
            print("  ⚠️ 有舵机读不到, 先运行 tools/servo_tool.py scan 检查")
            return

        bus.torque(False)
        print("  力矩已释放 (全程手掰, 请扶好机械臂)")

        zero, offsets = step_zero(bus)
        signs = step_signs(bus)
        lo, hi = step_ranges(bus, offsets)
        mismatches = step_verify_mapping(bus, lo, hi, signs)

        result.update({
            'mapping': {jkey: did for jkey, _, did, _ in JOINTS},
            'mapping_mismatches': mismatches,
            'signs': {str(s): signs[s] for s in signs},
            'offsets': {str(s): offsets[s] for s in offsets},
            'zero_raw': {str(s): zero[s] for s in zero},
            'raw_range': {str(s): [lo[s], hi[s]] for s in IDS},
        })
        with open(RESULT_FILE, 'w') as f:
            json.dump(result, f, ensure_ascii=False, indent=2)

        print("\n" + "=" * 60)
        print("【Step 5/5】标定完成! 汇总:")
        print(f"  {'关节':<8} {'ID':>3} {'范围':>14} {'SIGN':>5} {'OFFSET':>7}")
        for jkey, jname, did, _ in JOINTS:
            print(f"  {jname:<8} {did:>3} "
                  f"[{lo[did]:4d}~{hi[did]:4d}]  "
                  f"{signs.get(did, '?'):>4} {offsets.get(did, 0):>+7d}")
        print(f"\n  已保存: {RESULT_FILE}")
        print("  ➡️  告诉 Claude 一声即可, 它会读取此文件回填 config.py")
        print("=" * 60)

        if ask("  恢复力矩? [Y/n] ", ['y', 'n']) != 'n':
            bus.torque(True)
            print("  力矩已恢复, 标定结束 ✅")
    except KeyboardInterrupt:
        print("\n\n  [中止] 力矩可能仍处释放状态, 请扶好机械臂!")
        print("  恢复力矩: python3 tools/servo_tool.py torque on")
    finally:
        bus.close()


if __name__ == '__main__':
    main()
