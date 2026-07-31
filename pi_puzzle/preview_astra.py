"""
Astra S 相机预览脚本

用于快速验证 Astra S 硬件和桥接程序是否正常工作。
通过 tools/astra_bridge (C++ OpenNI2) 同时获取 RGB + Depth。

依赖:
    pip install opencv-python numpy
    bash tools/build_bridge.sh   # 首次使用需编译桥接程序

运行:
    python preview_astra.py

按键:
    q - 退出
    s - 保存截图 (astra_rgb.jpg / astra_depth.png)
"""
import cv2
import numpy as np
from camera_backend import AstraBackend


def main():
    backend = AstraBackend()
    if not backend.open():
        print("[Error] Astra S 打开失败")
        return

    print("[Info] 按 'q' 退出, 按 's' 保存截图")

    while True:
        bgr = backend.read_rgb()
        depth = backend.read_depth()

        if bgr is None:
            print("[Error] 读取 RGB 失败")
            break

        cv2.imshow('Astra S RGB', bgr)

        if depth is not None:
            depth_vis = np.clip(depth, 300, 1000).astype(np.float32)
            depth_vis = cv2.normalize(depth_vis, None, 0, 255,
                                      cv2.NORM_MINMAX).astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.imshow('Astra S Depth', depth_vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('astra_rgb.jpg', bgr)
            if depth is not None:
                cv2.imwrite('astra_depth.png', depth)
            print("[Info] 已保存 astra_rgb.jpg / astra_depth.png")

    backend.release()
    cv2.destroyAllWindows()
    print("[Info] 预览结束")


if __name__ == '__main__':
    main()
