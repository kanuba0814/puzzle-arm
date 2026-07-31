# 项目里程碑总结 - 拼图装置(E题)

## 项目概况
- **赛题**: 2026年全国大学生电子设计竞赛 E题 - 拼图装置
- **硬件**: SO-ARM101机械臂 + 树莓派 + 摄像头
- **技术栈**: Python + OpenCV + 传统逆运动学
- **工作目录**: `E:\点此立刻开始干活\电赛\123`

---

## 已完成的代码模块 (7个文件，全部可运行)

### 1. config.py ✅
- 全局配置: 连杆参数、舵机ID/限位、摄像头参数、任务参数
- SO-ARM101连杆: L1=67mm, L2=100mm, L3=100mm, L4=80mm (需实物校准)
- 舵机: STS3215, 6个舵机(ID 1-6), 波特率1Mbps
- 摄像头: 当前使用 DirectShow 索引1, 分辨率1280x720
- 关键参数: PICK_Z=5mm, APPROACH_Z=80mm, 目标矩形中心(105, 223)mm

### 2. kinematics.py ✅ (已验证, 5点测试误差0.00mm)
- 正运动学: 关节角 -> 末端坐标
- 逆运动学: (x,y,z) -> [J1..J6], 支持指定末端绝对俯仰角
- 舵机值转换: 角度 <-> 0-4095
- 工作空间检查
- 注意: wrist_pitch_deg 是末端绝对角度(非相对), 默认-90度=垂直向下

### 3. vision.py ✅ (框架完成, 需实际调参)
- PuzzleVision类: 摄像头采集 + 透视标定 + 碎片检测
- start_interactive_calibration(): 手动点击A4四角标定(比赛时不可用,需改为自动)
- pixel_to_world(): 像素->毫米坐标转换
- detect_colored_pieces(): HSV颜色检测, 适用于第1项纯色碎片
- detect_pieces_by_edge(): 自适应阈值+边缘检测, 适用于第2项白色/扑克牌碎片
- match_poker_pattern(): ORB特征匹配(需重写, 不能用模板匹配)
- detect_divider_line(): 霍夫直线检测分界线
- **待解决**: 摄像头画面偏暗偏绿, 需软件Gamma校正+白平衡

### 4. puzzle_solver.py ✅ (第1项可用, 第2项需完善)
- PuzzleSolver类
- solve_task1(): 第1项完整可用, 4片竖排拼8cm×8cm矩形
- _generate_task1_targets(): 预计算4个目标位置
- solve_task2(): 第2项框架就绪但搜索算法是空壳
  - _find_edge_matches(): 边长匹配可用(2mm容差)
  - _search_puzzle_layout(): **位置计算写死+30, 需根据共享边几何关系重写**
- _assign_pieces_optimal(): 贪心最近匹配
- optimize_order(): 贪心路径优化

### 5. arm_controller.py ✅ (框架完成, 支持模拟模式)
- ArmController类: STS3215串口通信
- 依赖 scservo_sdk (pip install scservo), 未安装时自动进入模拟模式
- write_positions_sync(): 多舵机同步写
- move_joints() / move_to_xyz(): 高层控制
- open_gripper() / close_gripper()
- go_home() / wait_for_stop() / get_current_position()

### 6. motion_planner.py ✅ (框架完成)
- MotionPlanner类: 接受ArmController
- move_joints_smooth(): 关节空间S曲线插值
- move_line(): 笛卡尔直线插值
- move_arc(): 弧形避障(抬升->平移->下降)
- pick_and_place(): **完整的拾取放置10步序列**
- 世界坐标->机械臂坐标转换: x-=ARM_BASE_X, y-=ARM_BASE_Y

### 7. main.py ✅ (状态机完成)
- PuzzleRobot类: 状态机 INIT->CALIBRATE->DETECT->SOLVE->EXECUTE->DONE
- run(task_mode='task1'|'task2'): 一键启动全流程
- 2分钟超时检查(留10秒余量)
- _signal_done(): 声光指示(LED+蜂鸣器, 需RPi.GPIO)
- 命令行: python main.py task1 / python main.py task2

### 8. debug_vision.py ✅ (调试工具)
- --mock: 模拟数据测试拼图算法(不需要硬件)
- --image: 图片测试视觉检测
- --camera: 摄像头实时测试
- --mode color|edge: 切换检测模式

---

## 当前摄像头状态

### 硬件情况
- 电脑有两个摄像头:
  - 索引0: 笔记本前置摄像头 (640x480, 亮度正常~119)
  - 索引1: 外接目标摄像头 (1280x720, 画面很暗偏绿, B=1 G=1 R=1)
- DirectShow后端可以打开索引1, 但首帧几乎全黑(B=1,G=1,R=1)
- 需要预热多帧后画面才能正常(之前测试30帧预热后亮度~80)
- MSMF后端索引1无法读取帧

### 摄像头问题待解决
1. **画面偏暗**: 亮度仅80/255, 需软件Gamma校正(gamma=0.4~0.5效果待确认)
2. **偏绿色**: G通道略高于B和R, 需灰世界白平衡
3. **硬件参数不可调**: 尝试设置CAP_PROP_EXPOSURE等参数后摄像头读取失败
4. **解决方案**: 纯软件处理(采集后Gamma+白平衡), 不动摄像头硬件参数
5. **config.py需更新**: CAMERA_INDEX应改为 `1 + cv2.CAP_DSHOW`

---

## 赛题补充说明关键约束

### 硬件限制
- **禁止使用电脑/笔记本**: 树莓派OK, 比赛时不能接显示器
- **禁止互联网/手机**: 全离线运行
- **禁止补光/遮光**: 不能加LED灯珠、遮光板
- **禁止AprilTag/标记**: A4纸上不能放任何标签或做标记
- **装置不能放在A4纸上**: 机械臂底座要在A4纸外侧
- **成品机械臂允许**: SO-ARM101合规
- **主控不限**: 树莓派OK, 可多板协同

### 碎片信息
- 第1项: 选手自备, 4片纯色, 2cm×8cm带切口, 拼成8cm×8cm矩形
- 第2项: 现场提供, 镀锌薄铁片(0.3mm)+贴纸, 总厚≤2mm, 重量≤30g
  - 铁片整张铺满纸片, 纸片在铁片上方
  - **磁吸抓取合规** -> 建议改用电磁铁吸盘替代夹爪
  - 数量≤4片, 每片边数≤5, 每边≥2cm
  - 目标矩形: 9cm×5cm ~ 12cm×9cm
  - 扑克牌: 现场随机, 不能提前获取模板, 拼接解唯一, 全部正面朝上

### 评分标准
- 判定标准: 只检查相邻碎片对应顶点距离≤2cm
- 矩形朝向任意, 不需要与A4纸平行
- 2分钟内可自动重试, 重试时间计入总时长
- 一键启动, 完成后声或光提示(其一即可)
- 移动到下半区+完成拼图 = 两项分数同时计算

---

## 待完成任务清单 (按优先级)

### P0 - 必须完成
1. **摄像头画面校正**: 解决索引1画面偏暗偏绿问题(Gamma+白平衡参数确定)
2. **config.py更新**: 摄像头索引改为 `1 + cv2.CAP_DSHOW`, 加入Gamma/白平衡参数
3. **自动标定**: 替换start_interactive_calibration(), 改为自动检测A4纸四角
4. **末端执行器改为电磁铁**: arm_controller.py中GRIPPER控制改为电磁铁通断
5. **连杆参数实测**: 用尺量SO-ARM101实际L1-L4值, 更新config.py

### P1 - 重要
6. **第2项拼图算法重写**: _search_puzzle_layout()根据共享边几何关系计算位置
7. **扑克牌花纹对齐**: 替换match_poker_pattern(), 改为接缝处局部图案连续性检测
8. **HSV颜色阈值标定**: 用实际碎片在摄像头下标定PIECE_COLOR_HSV
9. **抓取失败重试机制**: main.py的_execute()中加入重试逻辑
10. **树莓派部署**: 串口路径改为/dev/ttyUSB0, 安装scservo库, GPIO配置

### P2 - 优化
11. **多帧融合**: 连续采集3-5帧取众数, 过滤检测抖动
12. **ROI裁剪**: 只处理A4纸区域, 提升树莓派性能
13. **路径优化**: 避免机械臂运动中碰到已放置的碎片
14. **设计报告**: 方案论证、理论分析、电路设计、测试方案

---

## 文件结构
```
E:\点此立刻开始干活\电赛\123\
├── config.py              # 全局配置
├── kinematics.py          # 逆运动学 (已验证)
├── vision.py              # 视觉检测 (需调参)
├── puzzle_solver.py       # 拼图算法 (第1项OK, 第2项需重写)
├── arm_controller.py      # 舵机控制 (含模拟模式)
├── motion_planner.py      # 轨迹规划
├── main.py                # 主程序状态机
├── debug_vision.py        # 脱机调试工具
├── camera_test.py         # 摄像头测试脚本(当前版本)
├── multi_detect.py        # 多策略检测脚本
├── analyze_frame.py       # 帧分析脚本
└── homography.npy         # 标定矩阵(运行后生成)
```

## 快速验证命令
```bash
python kinematics.py           # IK验证(5点0误差)
python debug_vision.py --mock  # 模拟数据测试拼图
python camera_test.py          # 摄像头测试
python puzzle_solver.py        # 拼图算法测试
```

---

## Astra S 相机适配 (2026-07-29 完成)

### 结论
- Astra S (USB ID `2bc5:0402`) 是传统 OpenNI 协议设备, **RGB 流不注册为 UVC 设备** (`v4l2-ctl` 看不到), 必须走 OpenNI2
- 官方 **OrbbecSDK v2 / pyorbbecsdk v2 不支持 Astra S**; 正确 SDK 是 [OpenNI_SDK v2.3.0.86-beat6](https://github.com/orbbec/OpenNI_SDK/releases/tag/v2.3.0.86-beat6) (README 设备列表明确含 `Astra(S)`), 有 x64/arm32/arm64 包
- PyPI 的 `openni` Python 包 (2.3.0/2.2.0.post6) 在 Python 3.11+ 下打开设备后 `free(): corrupted unsorted chunks` 崩溃, **不可用**

### 最终方案: C++ 桥接
- `tools/astra_bridge.cpp` — 基于 OpenNI2 C++ SDK 的取帧程序, stdin 收 `f` 命令, stdout 输出二进制帧协议 (已验证稳定)
- `tools/build_bridge.sh` — 编译脚本 (自动定位 SDK, rpath 链接)
- `camera_backend.py` — `CameraBackend` 抽象: `USBCameraBackend` / `AstraBackend`(subprocess 桥接), 带 USB 回退
- `vision.py` `PuzzleVision` 改为注入 backend, 新增 `capture_depth()`; 上层检测代码零改动
- `preview_astra.py` — 预览 RGB+Depth, 按 s 截图
- conda 环境: `puzzle_astra` (Python 3.11), 依赖仅 opencv-python + numpy
- udev 规则已装: `/etc/udev/rules.d/56-orbbec-usb.rules`

### 验证结果
- RGB 640x480@30 ✅ (画面正常, 不偏暗不偏绿)
- Depth 640x480@30 uint16 mm ✅ (深度伪彩图与场景吻合)
- `debug_vision.py --mock` 回归通过 ✅

### 树莓派部署注意
- 下载 arm64 版 OpenNI_SDK 解压到 `sdks/OpenNI_SDK/`
- 重新运行 `bash tools/build_bridge.sh` 编译桥接程序
- 桥接不依赖 conda/pip openni, Pi 上只需 opencv+numpy

### 待办更新
- ~~摄像头画面校正 (偏暗偏绿)~~ — Astra S 替代后不再存在
- 可用深度做增强: TABLE_DEPTH_MM 背景差分替代自适应阈值 (第2项白色碎片), 非必需

---

## MaixCam Pro 纯相机方案 (2026-07-30 完成)

### 结论
- MaixCam Pro 只作相机, 本机/树莓派运算。弃用RTSP (ffmpeg依赖重/缓冲延迟),
  改用设备端 MJPEG HTTP 服务, 按需拉取单帧 JPEG, 简单可靠
- 新增后端 `MjpegSnapshotBackend` (`CAMERA_TYPE='mjpeg'`, 当前默认);
  RTSP/Astra/USB 后端全部保留可切换

### 设备端 (MaixCam, /root/mjpeg_server.py, 开机自启 /etc/init.d/S99mjpeg)
- 相机 2560x1440 采集 + ISP 硬件降采样双通道:
  - `/shot.jpg` + `/stream`: 1280x720 (ISP降噪, **检测主用**)
  - `/shot1440.jpg`: 2560x1440 原图 (备用; 实测软件降采样噪声反而更高, 不用)
- **设备铁律: 每次开机只能起一个相机进程**, 重启相机进程必现
  "No buffer space available" SIGSEGV → 恢复: `echo b > /proc/sysrq-trigger`
  (改服务文件后整体重启设备即可, 勿单独 kill python3)

### 暗光检测管线 (vision.py 重写要点)
- 场景: 夜间亮度~80, 强紫色固定 Pattern 噪声(FPN, 帧平均无效),
  碎片(深绿)与纸面对比度 V差60+, 色度线索全灭
- `auto_detect_paper`: **Otsu V阈值**优先, 分位数规则兜底
  (旧分位数规则会把纸面阴影角切掉)
- `detect_pieces_arbitrary` 双模式:
  - 暗光 (paper_v<140): **平场校正** V/高斯背景 < 0.82 → 碎片0.55-0.72
    vs 纸面/阴影1.0-1.2, 分离干净; 大核闭运算9x9
  - 亮光: V < paper_v-25 为主 + 严格饱和度线索(防阴影误判); 小核闭运算5x5
    (大核会把间距几mm的相邻碎片桥接成一块)
- 多边形拟合: 渐进eps简化(题目保证边数≤5, ≤6顶点即止)
  → `_cleanup_polygon` 共线/短边清理
  → **`_refine_edges` 边缘直线精化** (每条边fitLine, 相邻线求交得顶点,
    比approxPolyDP顶点抗噪一个量级; 注意顶点j=边j-1与边j的交点)
- 主流程入口 `solve_paths.py --image x.jpg`: 帧均值<100时先做
  fastNlMeansDenoisingColored (0.4s, 噪声减半)

### 检测参数扫描 (solve_paths.py)
- 噪声场景单一阈值可能不自洽 → 扫描 flat_th ∈ (0.82,0.78,0.86,0.74,0.90),
  每组检测+求解, 取矩形评分(角度/边长/面积偏差)最优者
- 亮光模式flat_th无效, 检测结果签名相同自动跳过

### 验证结果
- 合成场景端到端 (`tools/make_synthetic_scene.py`): 亮光5/5, 暗光3/3,
  目标100x65mm矩形, 求解误差<2mm, 输出 paths.json + 双可视化 ✅
- 实拍联调 (夜间暗光): MJPEG取帧→检测4片→扫描求解, 全链路无崩溃 ✅
  但桌面现有4片绿色碎片**几何上不能拼成矩形** (任意剪的测试碎片,
  8mm边容差下也无解) — 需用同一矩形真剪出的碎片做最终验证 (待白天)

### 求解器抗噪加固 (2026-07-30 深夜, 实拍验证通过)

用户手动拼好确认碎片可拼后, 逐步定位求解失败根因并修复:

1. **诊断计数** (`solver.stats`): overlap/fit/full_placed/validate_fail/
   dead_end 分阶段拒绝统计 — 发现DFS能找到完整摆放但被后续环节误杀
2. **位姿松弛** `_relax_pose`: 咬合位姿因检测角度误差(长边1-3°=末端
   数mm重叠)被重叠检查误杀 → 邻域采样(旋转±4°/平移±3mm)取最小重叠
3. **fit_margin 属性**: `_fits_in_final_rect` 的面积预算加余量
   (检测轮廓内缩~10%, 真拼法bbox超出"碎片总面积"预算被误剪)
4. **端点吸附自适应**: 矩形校验的端点吸附半径 2mm →
   `min(max(2, tol×0.6), 5)` (松弛位姿的接缝缝隙达数mm)
5. **近似解兜底 (关键)**: 完整摆放的边界走环极脆弱 → 改用免走环的
   **bbox填充率** (minAreaRect面积/碎片总面积) 排序所有完整摆放,
   <1.35接受为近似解, 评分+1000保证严格解优先。
   比赛按相邻顶点距离计分, 近似拼接也能得分!
6. **diag工具**: `tools/diag_assembly.py` (变体组合求解+拒绝统计)

**实拍验证结果**: 夜间暗光下, 4片绿色真碎片 3个独立检测阈值收敛到
~98-102×58mm矩形 (填充率1.039-1.050), 最优 **97.6×57.9mm [近似解]**,
完整输出抓取点/放置点/旋转角/路径 ✅
(paths.json `approximate: true` 标记)

### 已知限制 / 后续
- 暗光实拍检测误差±3-5mm: FPN+2px/mm分辨率所限, 比赛正常光照下余量充足
- 白色/扑克牌碎片检测 (与纸同色) 是下一阶段: 计划边缘法/背景差分
- 机械臂执行未接 (本阶段交付: 计算+可视化)

