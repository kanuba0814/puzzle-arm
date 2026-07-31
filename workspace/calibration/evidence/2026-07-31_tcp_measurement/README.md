# 2026-07-31 自定义磁吸头 TCP 实测

## 实测事实

- 设备：当前同一台 `so101_follower_modified_single_arm`。
- 测量工具：数显卡尺，具体型号与准确度 `UNKNOWN`。
- 测量基准：J5 旋转输出/安装平面 → 电磁铁实际工作接触面。
- 单次读数：`40.17 mm`。
- 当前只有 1 次读数，重复性仍为 `UNKNOWN`。
- 断力矩下固定 J4、手转 J5，用户观察到磁铁基本绕自身圆心旋转；这是定性观察，不是偏心量定量测量。

| 证据 | SHA256 |
| --- | --- |
| `side_caliper_40.17mm.jpg` | `0a45f83a832a713d22e27c2beedd53370adc9566dd616a32230a4ab12cf8ec40` |
| `front_coaxial_view.jpg` | `19888dd6029346781a0b382b58e63463648ad0d4e12f82f91822337b888bbba3` |

## 工程推导（不是独立实测）

历史 `so101_new_calib.urdf` 记录的 J4→J5 平面分量为 forward `61.1 mm`、normal `18.1 mm`。结合本次单次实测，触点拟合初值为：

- `tool_forward = 61.1 + 40.17 = 101.27 mm`
- `tool_normal = 18.1 mm`

该 URDF 原件及 SHA256 尚未纳入本仓库，故以上只能标记为 `provisional` 工程输入。最终值必须通过新触点拟合和至少 4 个独立 holdout 验收；不得据此解除真实运动门禁。
