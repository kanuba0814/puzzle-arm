# workspace

这里放实际项目代码、标定、模型、数据和日志。当前包只提供目录边界和契约，不预写实现脚本。

建议模块：

- `src/vision`：相机、标定、分割、位姿、验证
- `src/puzzle`：固定模板、几何搜索、纹理排序
- `src/control`：IK、轨迹、安全、磁铁
- `src/robot_plugin`：LeRobot 五轴插件
- `src/policy`：数据处理、ACT 训练/推理适配
- `src/integration`：Supervisor 与状态机
- `tests`：单元、回放、硬件在环和正式流程测试
- `calibration`：版本化标定文件
- `models`：checkpoint 与哈希说明
- `datasets`：只存索引和元数据；大数据按实际策略管理
- `logs`：按测试 ID 保存证据
