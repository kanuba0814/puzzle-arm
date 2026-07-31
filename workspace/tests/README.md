# tests

测试层级与冻结条件见 `docs/12_测试与验收计划.md`。每个失败必须保存原始证据和错误码。

`test_accepted_calibration_replay.py` 会校验 accepted 标定引用的四个 SHA256，并用生产 `PlanarArmKinematics` 重放 M/N/O/P，防止拟合脚本与运行时 FK 漂移或只信任 JSON 中自报指标。

## GUI 测试前置

- `test_pick_place_page.py`（赛场 GUI headless 回归）：需要 `pip install pygame-ce`；测试自行加入 `workspace/src`、`workspace/apps`、`workspace/rpi_gui_snapshot` 到 sys.path，并用 SDL dummy 离屏运行。直接执行 `python3 workspace/tests/test_pick_place_page.py -v`。
- `workspace/rpi_gui_snapshot/test_gui_touch.py`（fb0/evdev 原语）：需要 pygame-ce，且 `PYTHONPATH=workspace/src`，在 `workspace/rpi_gui_snapshot` 目录下运行 `python3 -m unittest test_gui_touch -v`。
