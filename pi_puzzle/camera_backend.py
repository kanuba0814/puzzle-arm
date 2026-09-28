"""
相机后端抽象层

为 PuzzleVision 提供统一的取帧接口, 支持:
1. 普通 USB 摄像头 (USBCameraBackend)
2. Orbbec Astra S 深度相机 (AstraBackend)
   - 通过 tools/astra_bridge (C++ OpenNI2 桥接程序) 获取 RGB + Depth
   - PyPI 的 openni Python 包在 Python 3.11+ 下有内存崩溃 bug, 不可用
   - 桥接程序需先用 bash tools/build_bridge.sh 编译
3. MaixCam Pro RTSP 网络相机 (RTSPCameraBackend)
   - MaixCam 只作为相机, 运行 rtsp_stream 应用推流
   - 本机/树莓派通过 RTSP 拉流运算, 零额外依赖 (只需 opencv)
   - 后台线程持续取流, 始终返回最新帧 (避免 RTSP 缓冲延迟)

使用:
    from camera_backend import create_camera_backend
    backend = create_camera_backend()
    frame = backend.read_rgb()      # BGR 图像, 与 cv2.VideoCapture 一致
    depth = backend.read_depth()    # uint16 深度图(mm), 与上次 read_rgb 同步, 可能为 None
    backend.release()
"""
import os
import struct
import subprocess
import select
import threading
import time
import cv2
import numpy as np
from abc import ABC, abstractmethod
from config import *


class CameraBackend(ABC):
    """相机后端抽象基类"""

    @abstractmethod
    def open(self):
        """打开相机"""
        pass

    @abstractmethod
    def read_rgb(self):
        """
        读取一帧 RGB 图像
        返回: BGR 格式的 numpy.ndarray (uint8), 失败返回 None
        """
        pass

    def read_depth(self):
        """
        读取一帧深度图像 (可选)
        返回: uint16 深度图(mm), 不支持则返回 None
        """
        return None

    @abstractmethod
    def release(self):
        """释放相机资源"""
        pass


class USBCameraBackend(CameraBackend):
    """普通 USB 摄像头后端 (兼容原有逻辑; 亦用于 MaixCam UVC gadget)"""

    def __init__(self, index=CAMERA_INDEX, width=CAMERA_WIDTH,
                 height=CAMERA_HEIGHT, mjpg=False):
        self.index = index
        self.width = width
        self.height = height
        self.mjpg = mjpg
        self.cap = None

    def open(self):
        self.cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
        if self.mjpg:
            # 请求MJPG压缩格式 (MaixCam UVC: 1280x720@30 仅MJPG可达)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # 部分摄像头首帧很暗, 预热几帧
        for _ in range(WARMUP_FRAMES):
            self.cap.read()

        print(f"[Camera] USB 摄像头已打开: index={self.index} "
              f"{self.width}x{self.height} mjpg={self.mjpg}")
        return self.cap.isOpened()

    def read_rgb(self):
        if self.cap is None:
            self.open()
        ret, frame = self.cap.read()
        if not ret:
            print("[Camera] USB 摄像头读取失败")
            return None
        return frame

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class RTSPCameraBackend(CameraBackend):
    """
    MaixCam Pro RTSP 后端

    MaixCam 运行 rtsp_stream 应用 (640x480), 本机通过 RTSP 拉流。
    后台线程持续读取并丢弃旧帧, read_rgb() 始终返回最新一帧,
    避免 OpenCV/ffmpeg 内部缓冲造成的数秒延迟。

    断流后自动重连。
    """

    def __init__(self, url=RTSP_URL, transport=RTSP_TRANSPORT):
        self.url = url
        self.transport = transport
        self.cap = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._latest = None      # 最新帧
        self._seq = 0            # 帧序号
        self._fail_count = 0

    def open(self):
        # TCP 传输更稳定 (UDP 在 Wi-Fi 下可能丢包花屏)
        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = (
            f'rtsp_transport;{self.transport}'
            f'|stimeout;{RTSP_OPEN_TIMEOUT_MS * 1000}'
            f'|max_delay;500000|buffer_size;1024000')

        if not self._connect():
            return False

        self._running = True
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()

        # 等待首帧
        deadline = time.time() + RTSP_OPEN_TIMEOUT_MS / 1000
        while time.time() < deadline:
            if self._latest is not None:
                print(f"[Camera] RTSP 已连接: {self.url} "
                      f"{self._latest.shape[1]}x{self._latest.shape[0]}")
                return True
            time.sleep(0.05)

        print("[Camera] RTSP 已打开但首帧超时")
        return False

    def _connect(self):
        self.cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            print(f"[Camera] RTSP 打开失败: {self.url}")
            return False
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return True

    def _grab_loop(self):
        while self._running:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self._fail_count += 1
                if self._fail_count > 30:  # 连续失败, 重连
                    print("[Camera] RTSP 断流, 尝试重连...")
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    time.sleep(1.0)
                    if self._running and self._connect():
                        self._fail_count = 0
                    continue
                time.sleep(0.01)
                continue
            self._fail_count = 0
            with self._lock:
                self._latest = frame
                self._seq += 1

    def read_rgb(self):
        with self._lock:
            if self._latest is None:
                return None
            return self._latest.copy()

    def read_seq(self):
        """当前帧序号, 可用于等待新帧"""
        with self._lock:
            return self._seq

    def release(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class MjpegSnapshotBackend(CameraBackend):
    """
    MaixCam MJPEG 快照后端: 通过 HTTP 拉取单帧 JPEG。
    比 RTSP 简单可靠 (无ffmpeg依赖/无缓冲延迟), 适合按需取帧。
    支持高分辨率端点 (如 /shot1440.jpg 取2560x1440原图)。
    """

    def __init__(self, shot_url=None, timeout=10):
        self.shot_url = shot_url or MJPEG_SHOT_URL
        self.timeout = timeout
        self._ok = False

    def open(self):
        frame = self.read_rgb()
        if frame is None:
            print(f"[Camera] MJPEG 快照打开失败: {self.shot_url}")
            return False
        self._ok = True
        print(f"[Camera] MJPEG 快照已连接: {self.shot_url} "
              f"({frame.shape[1]}x{frame.shape[0]})")
        return True

    def read_rgb(self):
        try:
            import urllib.request
            with urllib.request.urlopen(
                    self.shot_url, timeout=self.timeout) as resp:
                data = resp.read()
            arr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return frame
        except Exception as e:
            print(f"[Camera] MJPEG 取帧失败: {e}")
            return None

    def release(self):
        self._ok = False


class AstraBackend(CameraBackend):
    """
    Orbbec Astra S 后端 (C++ 桥接模式)

    通过 subprocess 运行 tools/astra_bridge 获取帧。
    桥接程序持续采集, Python 发送 'f' 命令取最新快照 (RGB+Depth 同步)。

    注意:
        - read_depth() 返回的是与最近一次 read_rgb() 同一次快照的深度图
        - 需要先编译桥接程序: bash tools/build_bridge.sh
    """

    def __init__(self, bridge_path=None, grab_timeout=3.0):
        if bridge_path is None:
            root = os.path.dirname(os.path.abspath(__file__))
            bridge_path = os.path.join(root, 'tools', 'astra_bridge')
        self.bridge_path = bridge_path
        self.grab_timeout = grab_timeout

        self._proc = None
        self._last_rgb = None
        self._last_depth = None
        self._depth_available = False

    def open(self):
        if not os.path.isfile(self.bridge_path):
            print(f"[Camera] 桥接程序不存在: {self.bridge_path}")
            print("         请先编译: bash tools/build_bridge.sh")
            return False

        try:
            self._proc = subprocess.Popen(
                [self.bridge_path],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=None,  # 桥接日志直接打到终端, 方便诊断
                bufsize=0)
        except OSError as e:
            print(f"[Camera] 桥接程序启动失败: {e}")
            return False

        # 预热: 抓若干帧让流稳定, 同时探测深度流是否可用。
        # 深度流启动比 RGB 慢, 前几次快照可能没有深度, 多抓几帧等待。
        depth_probe_frames = max(WARMUP_FRAMES, 30)
        for _ in range(depth_probe_frames):
            rgb, depth = self._grab()
            if depth is not None:
                self._depth_available = True
                break

        if self._proc.poll() is not None:
            print(f"[Camera] 桥接程序异常退出: {self._proc.returncode}")
            return False

        print(f"[Camera] Astra S 桥接已就绪 (深度流: "
              f"{'可用' if self._depth_available else '不可用'})")
        return True

    # ---------- 桥接协议 ----------

    def _read_exact(self, n):
        fd = self._proc.stdout.fileno()
        buf = b''
        deadline = self.grab_timeout
        while len(buf) < n:
            r, _, _ = select.select([fd], [], [], deadline)
            if not r:
                raise TimeoutError("桥接响应超时")
            chunk = os.read(fd, n - len(buf))
            if not chunk:
                raise IOError("桥接管道已关闭")
            buf += chunk
        return buf

    def _sync_magic(self):
        """逐字节扫描直到出现 'ASTR' 或 'AEND' 同步字"""
        fd = self._proc.stdout.fileno()
        window = b''
        while True:
            r, _, _ = select.select([fd], [], [], self.grab_timeout)
            if not r:
                raise TimeoutError("桥接同步超时")
            c = os.read(fd, 1)
            if not c:
                raise IOError("桥接管道已关闭")
            window = (window + c)[-4:]
            if window == b'ASTR':
                return False  # 还有数据包
            if window == b'AEND':
                return True   # 本次快照结束

    def _grab(self):
        """向桥接请求一次快照, 返回 (bgr_frame, depth_frame)"""
        if self._proc is None or self._proc.poll() is not None:
            return None, None

        try:
            self._proc.stdin.write(b'f')
            self._proc.stdin.flush()

            rgb, depth = None, None
            while True:
                if self._sync_magic():
                    break
                header = self._read_exact(13)
                ftype = header[0:1]
                w, h, size = struct.unpack('<III', header[1:13])
                payload = self._read_exact(size)

                if ftype == b'C':
                    arr = np.frombuffer(payload, np.uint8).reshape(h, w, 3)
                    rgb = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                elif ftype == b'D':
                    depth = np.frombuffer(payload, np.uint16).reshape(h, w)
            return rgb, depth
        except (TimeoutError, IOError) as e:
            print(f"[Camera] 桥接抓取失败: {e}")
            return None, None

    # ---------- CameraBackend 接口 ----------

    def read_rgb(self):
        rgb, depth = self._grab()
        if rgb is not None:
            self._last_rgb = rgb
            if depth is not None:
                self._last_depth = depth
                self._depth_available = True
        else:
            print("[Camera] Astra 读取失败")
        return self._last_rgb

    def read_depth(self):
        """返回与最近一次 read_rgb() 同步的深度图"""
        return self._last_depth if self._depth_available else None

    def release(self):
        if self._proc is not None and self._proc.poll() is None:
            try:
                self._proc.stdin.write(b'q')
                self._proc.stdin.flush()
                self._proc.wait(timeout=3)
            except Exception:
                self._proc.kill()
        self._proc = None


# 全局后端实例缓存, 避免重复创建
_backend_instance = None


def create_camera_backend(camera_type=None, **kwargs):
    """
    工厂函数: 根据配置创建相机后端

    参数:
        camera_type: 'usb' / 'astra' / 'rtsp', 默认读取 config.CAMERA_TYPE
        **kwargs: 传递给后端构造函数的额外参数
    """
    global _backend_instance

    if camera_type is None:
        camera_type = CAMERA_TYPE

    if _backend_instance is not None:
        return _backend_instance

    if camera_type == 'astra':
        backend = AstraBackend(**kwargs)
    elif camera_type == 'rtsp':
        backend = RTSPCameraBackend(**kwargs)
    elif camera_type == 'mjpeg':
        backend = MjpegSnapshotBackend(**kwargs)
    elif camera_type == 'usb':
        # MaixCam UVC gadget: 1280x720 仅 MJPG 可达 (YUYV 最高 640x480)
        # 失败时由下方回退逻辑再尝试默认参数
        backend = USBCameraBackend(
            index=MAIXCAM_UVC_INDEX,
            width=MAIXCAM_UVC_SIZE[0],
            height=MAIXCAM_UVC_SIZE[1],
            mjpg=True,
        )
    else:
        backend = USBCameraBackend(**kwargs)

    if not backend.open():
        print(f"[Camera] {camera_type} 后端打开失败, 尝试回退到 USB 摄像头")
        backend = USBCameraBackend(**kwargs)
        if not backend.open():
            print("[Camera] USB 摄像头也打开失败")
            return None

    _backend_instance = backend
    return backend


def release_camera_backend():
    """释放全局后端实例"""
    global _backend_instance
    if _backend_instance is not None:
        _backend_instance.release()
        _backend_instance = None


if __name__ == '__main__':
    # 简单测试: 打开后端并显示 RGB/深度
    backend = create_camera_backend()
    if backend is None:
        print("相机打开失败")
        exit(1)

    print("按 'q' 退出, 's' 保存截图")
    while True:
        bgr = backend.read_rgb()
        depth = backend.read_depth()

        if bgr is not None:
            cv2.imshow('RGB', bgr)
        if depth is not None:
            depth_vis = cv2.applyColorMap(
                cv2.convertScaleAbs(depth, alpha=255/1000),
                cv2.COLORMAP_JET)
            cv2.imshow('Depth', depth_vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s') and bgr is not None:
            cv2.imwrite('camera_rgb.jpg', bgr)
            if depth is not None:
                cv2.imwrite('camera_depth.png', depth)
            print("已保存 camera_rgb.jpg / camera_depth.png")

    backend.release()
    cv2.destroyAllWindows()
