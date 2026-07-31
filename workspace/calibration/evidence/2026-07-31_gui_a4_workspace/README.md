# 2026-07-31 GUI A4 工作区与正式页面

- GUI 标定时间：2026-07-31 14:29:44 +08:00。
- 相机：`/dev/video0`，UVC MJPG，1280×720。
- frame：A4 左下原点，x 右 0–210 mm，y 上 0–297 mm。
- homography：image_px→table_mm，57 个角点，RMS `0.257541 mm`，max `1.029862 mm`。
- 当前黑底红色场景稳定检测到 4 个碎片。
- `formal_pick_place_page.png` 是部署前在 Pi 正式依赖环境生成的 1024×600 页面截图。
- 正式页面只常驻占用 `/dev/fb0`；机械臂动作时才临时连接串口，完成后回 PARK 并断力矩。
- PICK/PLACE 接触高度仍为 `UNKNOWN`，抓取旋转放置按钮保持安全锁定。
