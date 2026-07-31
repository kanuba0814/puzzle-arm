"""
电磁铁控制模块 (替代夹爪的末端执行器)

硬件: 树莓派 GPIO --(MOSFET低边开关)--> 电磁铁
  - GPIO 高电平 = MOSFET导通 = 电磁铁通电吸附
  - 线圈两端需反向并联续流二极管 (消磁防反峰, 硬件电路)
  - Pi 5 上使用 gpiozero (lgpio后端); 经典 RPi.GPIO 在 Pi 5 不可用

无 gpiozero 的环境 (开发机调试) 自动进入模拟模式, 仅打印动作。
"""
import time

from config import MAGNET_PIN, MAGNET_RELEASE_MS

try:
    from gpiozero import DigitalOutputDevice
    GPIO_OK = True
except ImportError:
    GPIO_OK = False


class MagnetController:
    def __init__(self, pin=MAGNET_PIN):
        self.pin = pin
        self.simulation = not GPIO_OK
        self._dev = None

        if self.simulation:
            print("[Magnet] gpiozero 不可用, 使用模拟模式 (仅打印)")
        else:
            try:
                # initial_value=False: 创建即确保断电状态
                self._dev = DigitalOutputDevice(pin, initial_value=False)
                print(f"[Magnet] GPIO{pin} 就绪 (初始=断电)")
            except Exception as e:
                # 常见于: 非Pi环境/权限不足/引脚被占用
                print(f"[Magnet] GPIO{pin} 初始化失败 ({e}), 使用模拟模式")
                self.simulation = True

    def on(self):
        """电磁铁通电 (吸附)"""
        if self.simulation:
            print("[Sim] 电磁铁 ON (通电吸附)")
            return True
        self._dev.on()
        return True

    def off(self):
        """电磁铁断电 (释放), 内含释放延时等待残磁消退/碎片脱落"""
        if self.simulation:
            print(f"[Sim] 电磁铁 OFF (断电释放, 延时{MAGNET_RELEASE_MS}ms)")
            time.sleep(MAGNET_RELEASE_MS / 1000)
            return True
        self._dev.off()
        time.sleep(MAGNET_RELEASE_MS / 1000)
        return True

    def is_on(self):
        """当前是否通电"""
        if self.simulation:
            return False
        return bool(self._dev.value)

    def close(self):
        """释放GPIO资源 (确保断电)"""
        if not self.simulation and self._dev is not None:
            self._dev.off()
            self._dev.close()
            print("[Magnet] GPIO已释放")


if __name__ == '__main__':
    # 通断自测: 循环3次, 可用万用表测MOSFET漏极或听电磁铁吸合声
    m = MagnetController()
    print("=== 电磁铁通断测试 (3次循环) ===")
    for i in range(3):
        print(f"[{i+1}/3] ON ...")
        m.on()
        time.sleep(1.0)
        print(f"[{i+1}/3] OFF ...")
        m.off()
        time.sleep(1.0)
    m.close()
    print("测试完成")
