# Astra S（ROS Noetic / Docker）

适用于 USB ID `2bc5:0402` 的 Orbbec Astra S。该设备的深度接口不是普通 UVC
`/dev/video*`，容器通过 Orbbec 官方 `ros_astra_camera`（固定到 v1.2.2 对应提交
`3759de789bba025e500f9b344104afadfcde83e5`）、其内置 OpenNI2 运行库和
`/dev/bus/usb` 访问。不要改用面向新型号的 OrbbecSDK v2 驱动；Astra S 属于旧款
OpenNI2/Astra 驱动路径。

## 启动

```bash
cd deployment/astra_s_ros
docker compose up --build astra-camera
```

主要话题：

- `/camera/depth/image_raw`：深度图；
- `/camera/depth/camera_info`：深度内参；
- `/camera/ir/image_raw`：红外图（设备支持并启用时）。

## 自动功能测试

保持相机无遮挡并朝向约 0.4–3 m 内、有表面纹理或几何变化的场景，然后运行：

```bash
docker compose up -d astra-camera
docker compose run --rm test
docker compose down
```

测试收到一帧深度图 header、一份相机内参，并成功调用设备信息服务即通过。查看话题与频率：

```bash
docker compose exec astra-camera rostopic list
docker compose exec astra-camera rostopic hz /camera/depth/image_raw
```

> `privileged: true` 用于开发机快速验证 USB。部署时可改为精确映射 USB 设备，
> 但 USB bus/device 编号会在重插后变化，需要配合 udev 规则。
