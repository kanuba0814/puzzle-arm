# calibration

此目录的文件必须记录来源、版本、时间和 SHA256；不得只按“最新”命名。

当前单臂 accepted 基线：

- `arm_kinematics_single_arm_accepted_20260731.json`
- SHA256：`c207bae180e187b6c639cc848320195dafce1f468920a0f2eba7f00c621eab38`
- 纸面变换：`contact_sheet_to_robot_base_20260731.json`
- 原始证据与拟合报告：`evidence/2026-07-31_single_arm_contacts_v2/`
- 首次真实硬件验证：`evidence/2026-07-31_first_hardware_motion/`
- 正式 PARK：`park_pose_single_arm_20260731.json`
- GUI A4 单应：`table_homography_gui_20260731.npy`
- 当前完整运行标定：`runtime_calibration_manifest_20260731.json`
- 红色碎片重复/巡检：`evidence/2026-07-31_red_piece_center_sweep/`
- 全栈部署、GUI 截图和 USB host failure：`evidence/2026-07-31_full_stack_deployment/`

GUI A4 与触点纸采用相同的左下原点、x 向右、y 向上约定；当前运行总表显式记录该等价假设。继电器 GPIO17/低电平控制链路已确认就绪，但 PICK/PLACE 接触高度仍为 `UNKNOWN`，正式页面因此继续锁定自动磁吸抓放。最新机械臂 USB host failure 后设备节点缺失，恢复前必须先现场支撑并断舵机电源。
