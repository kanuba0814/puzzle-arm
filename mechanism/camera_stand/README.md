# camera_stand — MaixCAM Pro 顶置支架参数化模型

对应设计文档 `docs/23_相机支架设计.md`。几何唯一真源：`params.py` + `parts/*.py`（build123d 0.11）。

## 使用

```bash
export PATH="/home/gxxl/miniconda3/bin:$PATH"
cd mechanism/camera_stand
python build_all.py   # 构建全部零件 -> exports/ STL+STEP -> 断言 -> reports/stand_report.json + renders/
```

退出码 0 = 全部硬断言通过。断言覆盖：零件有效性与成型尺寸、接口参数一致性、
装配干涉（对 boardCam 代理体）、镜头高度链（364±20）、倾覆安全系数（分沙/钢砂两方案）、
斜梁挠度估计（soft）。

## 打印顺序

1. **先打 p9a/p9b 配合试片**（约 30 min）：验证 12.0 槽/11.8 插榫、M3 螺母预埋孔、
   M3×8 铜柱预埋孔、Ø2.5 自攻底孔。间隙不合适时改 `params.py` 对应补偿值再正式打印。
2. 正式件：p1a 底座桶（正放）、p1b 顶板（**倒放**，刻字面贴床）、p3 转子（法兰贴床）、
   p4 斜梁（切片软件内旋转至床面对角线平放）、p5 肘节板（平放）、p6 夹箍、p7 角尺×2、p8 线夹×3。
3. 材料 PLA，0.4 喷嘴；壁/筋类零件建议 ≥4 层墙。

## 五金（用户限定：M3×6 短螺丝 + M3 螺母/铜柱）

- M3×6 螺丝约 20 颗；M3 螺母约 4 颗（偏航夹环 1、夹箍 1、备用 2）；
- M3×8 双通铜柱 2 颗（肘节锁紧 barrel）；
- 配重：**钢砂/钢珠 ≥900 g**（干沙方案安全系数 1.84 < 2，不满足断言，见报告 tipping 节）；
- 铰链销 Ø8 为打印件（压配），如磨损可改 M8 螺栓（需确认采购）。

## 关键设计决策（与设计文档的差异/细化）

- 顶部接口：插榫 11.8×11.8 插入 boardCam 叉口（实测 12.0×12），
  **外加 P6 C 形夹箍抱紧叉耳块**；boardCam 底部 Ø9.6/Ø17 孔位 v1 不使用（意图待确认）。
- 底座分两件（p1a 桶 + p1b 顶板），避免 150mm 配重腔顶面无法搭桥。
- 所有夹紧副按 M3×6 行程核算：通孔侧耳板 ≤2.0mm，穿耳+缝后全牙进螺母/铜柱。

## build123d 0.11 踩坑记录（改代码前必读）

- `Cylinder` 默认**中心对齐**：`Pos(0,0,z)*Cylinder(h)` 把圆柱中心放 z，不是底面。
  本仓库多起干涉/半切 bug 都源于此；写完后必跑 `build_all.py` 的碰撞断言。
- `is_valid` 是**属性**不是方法；`intersect` 返回 ShapeList，体积要逐子体求和。
- `Shape.rotate` 需要 `Axis` 对象，不接受元组。
- `tessellate` 返回 Vector 列表，取坐标用 `v.X/v.Y/v.Z`。
- pyright 对 build123d 源码注解过宽：统一用 `common.as_part()` 收窄；
  `Pos/Rot` 用分离参数形式 `Pos(x, y, z)`，不要用元组形式（重载推断会报错）。
- builder 模式定位用 `with Locations((x, y)):`，`Pos(...) * Rectangle(...)` 是代数表达式，
  不会加入 BuildSketch 上下文。
- 圆弧长孔用 `CenterArc + SlotArc + extrude`，比手工布尔楔块可靠。

## UNKNOWN（实测前不得当作事实）

- MaixCAM Pro 质量与镜头 FOV；镜头光心在 boardCam 上的位置（L_LENS=235 为假设）；
- 叉耳槽 Y 向跨度实测（影响插榫深度 11.8 配合）；配重材料密度；打印实心率。
