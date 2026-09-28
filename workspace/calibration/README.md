# calibration

此目录的文件必须记录来源、版本、时间和 SHA256；不得只按“最新”命名。

当前单臂 accepted 基线：

- `arm_kinematics_single_arm_accepted_20260731.json`
- SHA256：`c207bae180e187b6c639cc848320195dafce1f468920a0f2eba7f00c621eab38`
- 纸面变换：`contact_sheet_to_robot_base_20260731.json`
- 原始证据与拟合报告：`evidence/2026-07-31_single_arm_contacts_v2/`
- 首次真实硬件验证：`evidence/2026-07-31_first_hardware_motion/`
- 正式 PARK：`park_pose_single_arm_20260731.json`
- 当前 GUI/123 A4 单应：`table_homography_charuco_recalibrated_20260801.npy`（70 角点，RMS 0.185 mm，max 0.411 mm）
- 当前完整运行标定：`runtime_calibration_manifest_20260731.json`
- 红色碎片重复/巡检：`evidence/2026-07-31_red_piece_center_sweep/`
- 全栈部署、GUI 截图和 USB host failure：`evidence/2026-07-31_full_stack_deployment/`

GUI A4 与触点纸采用相同的左下原点、x 向右、y 向上约定；当前运行总表显式记录该等价假设。继电器 GPIO17/低电平控制链路已确认就绪；PICK/PLACE 接触高度验收由 `pick_place_contact_height.json` 单独门控，不能回退到估计值。机械臂 USB host failure 后必须先现场支撑并断舵机电源，再做只读门禁。
