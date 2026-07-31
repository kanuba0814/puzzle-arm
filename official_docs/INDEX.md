# 官方资料索引

- 抓取/核对日期：2026-07-29
- 机器可读版本：`sources.lock.json`
- `downloaded` 表示已随包离线保存；`link-locked` 表示精确版本链接已锁定；`dynamic-link` 表示必须自行克隆并记录 commit。

## 已离线保存

### 2026 电赛 E题《拼图装置》
- 版本：2026-07-29
- 本地：`official_docs/competition/E题_拼图装置.pdf`
- SHA256：`1230aa018fa86a98ba3b2e73d55935e4065a990e2e9c230a560da1514d5d3f8e`
- 说明：从用户上传的正式赛题压缩包解出；题面事实最高优先级

### Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware
- 版本：arXiv:2304.13705
- 本地：`official_docs/papers/ACT_paper_2304.13705.pdf`
- SHA256：`2e2fe25860f5f9cee9e655a2714e9f2264a7a5078bcd267c16d3f39af461345a`
- 说明：原始 ACT 研究；论文中的示教数量和性能不能直接外推到本项目

### LeRobot: An Open-Source Library for End-to-End Robot Learning
- 版本：arXiv:2602.22818 / ICLR 2026
- 本地：`official_docs/papers/LeRobot_paper_2602.22818.pdf`
- SHA256：`0f59ee1894d9f78534aa89c54610c022417d1963f0e315bd0c10d0325aca3a99`
- 说明：框架论文

### Raspberry Pi 5 product brief
- 版本：Published April 2026
- 本地：`official_docs/raspberry_pi/Raspberry_Pi_5_product_brief_2026-04.pdf`
- SHA256：`10583c80f867e05af185742bb0f80448458c606a32dd3dbefbbcfca480b59ead`
- 说明：BCM2712 2.4GHz 四核 Cortex-A76、8GB variant、5V/5A 等

### Raspberry Pi 27W USB-C Power Supply product brief
- 版本：Published October 2023
- 本地：`official_docs/raspberry_pi/Raspberry_Pi_27W_USB-C_power_supply_product_brief.pdf`
- SHA256：`d5b4cc0d9724b7e917f29a76155d1f049802ed86e4d806990aee099ab91f66a9`
- 说明：5.1V/5A profile

### Feetech ST-3215-C018 12V detailed specification
- 版本：C018 12V
- 本地：`official_docs/datasheets/Feetech_STS3215_C018_12V_spec.pdf`
- SHA256：`3f1dc84e470ac89cbd7b7244183fcc62b7f7a5345aa41a62120280a9b7b26f5e`
- 说明：仅适用于实物标签确认为 C018 的舵机

## 已锁定的官方网页/源码

- **LeRobot PyPI** — `0.6.0`  
  https://pypi.org/project/lerobot/  
  状态：`link-locked`；0.6.0 发布于 2026-07-06；Python >=3.12
- **LeRobot v0.6.0 GitHub release** — `v0.6.0`  
  https://github.com/huggingface/lerobot/releases/tag/v0.6.0  
  状态：`link-locked`；比赛依赖固定 tag，不使用 main
- **LeRobot SO-101 guide** — `v0.6.0`  
  https://huggingface.co/docs/lerobot/v0.6.0/so101  
  状态：`link-locked`；原版 follower 为 6 个 STS3215；本项目通过自定义 5D 插件改造
- **LeRobot ACT guide** — `v0.6.0`  
  https://huggingface.co/docs/lerobot/v0.6.0/act  
  状态：`link-locked`；模型架构和配置依据
- **LeRobot Bring Your Own Hardware** — `v0.6.0`  
  https://huggingface.co/docs/lerobot/v0.6.0/integrate_hardware  
  状态：`link-locked`；自定义 Robot、features 与插件机制依据
- **LeRobot Imitation Learning on Real-World Robots** — `v0.6.0`  
  https://huggingface.co/docs/lerobot/v0.6.0/il_robots  
  状态：`link-locked`；录制、训练、部署流程依据
- **LeRobotDataset v3** — `v0.6.0`  
  https://huggingface.co/docs/lerobot/v0.6.0/lerobot-dataset-v3  
  状态：`link-locked`；数据组织依据
- **LeRobot asynchronous inference** — `v0.6.0`  
  https://huggingface.co/docs/lerobot/v0.6.0/async  
  状态：`link-locked`；仅在同步基线稳定后考虑
- **SOFollower source** — `v0.6.0`  
  https://github.com/huggingface/lerobot/blob/v0.6.0/src/lerobot/robots/so_follower/so_follower.py  
  状态：`link-locked`；电机表和 feature 自动生成的直接依据
- **SOLeader source** — `v0.6.0`  
  https://github.com/huggingface/lerobot/blob/v0.6.0/src/lerobot/teleoperators/so_leader/so_leader.py  
  状态：`link-locked`；leader 六轴定义依据
- **ACTConfig source** — `v0.6.0`  
  https://github.com/huggingface/lerobot/blob/v0.6.0/src/lerobot/policies/act/configuration_act.py  
  状态：`link-locked`；输入输出特征和 action chunk 配置依据
- **SO-ARM100/SO-101 hardware repository** — `main (dynamic)`  
  https://github.com/TheRobotStudio/SO-ARM100  
  状态：`dynamic-link`；使用前必须本地 clone 并记录 commit；不可把 main 当固定版本
- **SO-101 URDF** — `main (dynamic)`  
  https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf  
  状态：`dynamic-link`；已有公开 issue 指出旧 URDF 可能存在限制/轴向问题；必须与实物和标定核对
- **OpenCV documentation** — `4.13.0`  
  https://docs.opencv.org/4.13.0/  
  状态：`link-locked`；视觉 API 基线
- **OpenCV calib3d** — `4.13.0`  
  https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html  
  状态：`link-locked`；内参、PnP、单应与几何变换相关 API
- **OpenCV geometric image transformations** — `4.13.0`  
  https://docs.opencv.org/4.13.0/da/d54/group__imgproc__transform.html  
  状态：`link-locked`；warpPerspective 等
- **OpenCV distance transform tutorial** — `4.13.0`  
  https://docs.opencv.org/4.13.0/d2/dbd/tutorial_distance_transform.html  
  状态：`link-locked`；最大内接区域抓取点依据
- **Feetech STS3215 7.4V product information** — `STS3215 / C001 family; verify physical label`  
  https://www.feetechrc.com/2020-05-13_56655.html  
  状态：`link-locked`；7.4V、1:345、4096 position units 等；采购件以标签为准

## 未离线镜像的原因

LeRobot 在线文档、GitHub 源码树、OpenCV 文档和 SO-ARM100 CAD 仓库不是稳定的单一 PDF。当前环境也不允许把部分 ZIP/源码 MIME 直接作为附件抓取。为避免提供残缺或伪装成完整镜像的资料，本包保存精确版本、tag/commit、URL 和下载状态。正式开发机应按 `sources.lock.json` 克隆/下载，再生成本地哈希。

## 重要限定

- SO-ARM100 `main` 是动态来源；未记录 commit 前不能用于冻结构建。
- Feetech C001 与 C018 电压和参数不同；必须先看每个舵机实物标签。
- ACT 论文中的任务、示教次数和成功率不能直接外推到 E 题。
- Pi 5 上 ACT 的实际延迟没有官方统一基准，必须在最终 checkpoint、相机和环境上实测。
