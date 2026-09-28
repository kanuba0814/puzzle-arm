# GUI camera-calibration deployment — 2026-08-01

- Pi host/time: `epuzzle-pi`, 2026-08-01 18:12 CST.
- Deployed only the GUI camera-calibration command, its Supervisor adapter, contracts and tests; no position command or magnet-on command was sent.
- Restarting `epuzzle-gui.service` completed. The old process logged `五轴力矩已关闭`; the new GUI process owns `/dev/fb0` and the QDTECH evdev nodes.
- `/sys/class/graphics/fb0/virtual_size` read `1024,600` after restart.
- `post_restart_read_state.json` is a read-only bus result. It records all five `Torque_Enable=0`, relay GPIO17 raw value `1` (external high/off), and `elbow_flex.pos=97.802°`, outside frozen `[-95°, 95°]` motion soft limits after torque-off.
- Therefore the motion lock was created fail-closed. No motion may be commanded until the operator supports/inspects the arm, confirms the state and explicitly clears the lock.
