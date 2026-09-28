# integration

`Supervisor` 是唯一可切换全局运行状态的模块，公共面仅由
`integration.__init__` 导出。它复用冻结的 `control.MotionController`，不修改
IK、轨迹、GPIO17 或五轴字段顺序。

- `event_log.py` 使用单调时钟写入符合 `runtime_event.schema.json` 的 JSONL；
- `park_sequence.py` 在 integration 层实现实测读回的 PARK↔HOME 关节空间过渡；
- `sweep_planner.py` 对至多四片穷举最短 table-mm 路线，并在发送前预检整段 IK；
- `supervisor.py` 负责视觉/计划时效、shadow seam、急停锁定与 fail-closed 抓放门禁。

继电器控制链路已由操作者确认为 `/dev/gpiochip0`、BCM GPIO17、低电平触发，
`MagnetController` 仍保持默认关闭和失败断电；吸合后需由调用方显式关闭。PICK/PLACE 接触高度、正式
`table_mm -> robot_base_mm` 变换、磁头 footprint/净空任一未冻结时，Supervisor 必须
拒绝进入磁吸状态；没有调试旁路。shadow 配置不接受任何机器人、PARK mover 或磁铁
对象，防止误动作。未来抓取 FSM 的
`MAGNET_ON/VERIFY_PICK/...` 状态已保留，但 UNKNOWN 条件下绝不会调用
`MagnetController.set_enabled(True)`。

`workspace/tools/supervisor_run.py` 是供 GUI 调用的短命子进程包装，支持新鲜检测、
磁铁关闭的全局最短巡检、单片 PREGRASP/重复巡检、正式 PARK 和 puzzle shadow。
`pick-place` 在接触高度等门禁通过前会在构造硬件对象之前拒绝。当前不启动常驻
Supervisor daemon；legacy `control_red_piece_centers.py` 和现有急停链路保持不变。
