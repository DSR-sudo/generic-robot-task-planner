# PX4 mission_core adapter

这个 ROS 2 包提供 `mission_core` 的 PX4/MAVROS 执行适配器。旧的 PX4 专用行为树已经删除；
ROS 节点只负责采集平台状态、运行通用 XML 任务、执行动作和发送诊断。

## 运行路径

```text
MAVROS / LCP / UDP
        |
        v
Initialization + GroundStationLink
        |
        v
WorldState + CapabilityState + MissionEvent
        |
        v
mission_core::MissionTree + SafetySupervisor
        |
        v
Px4ActionExecutor -> Offboard -> MAVROS / PX4
```

`ApplicationNode` 以配置的控制频率运行一个单线程周期。动作执行期间，PX4 执行器持续维护设定点、
模式和 ARM/Disarm 请求；监控模式仍然不会发送控制输出。

## PX4 能力

`Px4ActionExecutor` 注册并实现：

- `navigate`: `x`、`y`、`z`，可选 `yaw` 和 `frame=local_enu`；以实际位置容差判断完成。
- `takeoff`: 正的 `height_m`；先完成 OFFBOARD 设定点预热，再请求模式和 ARM。
- `hold`: 可选非负 `seconds`；保持当前实测位置。
- `land`: 维持 `AUTO.LAND`，确认触地后切换 `MANUAL` 并解除解锁。

飞行中动作被安全取消时，执行器会继续执行安全降落，不会直接停止设定点或空中解除解锁。

## 启动

默认任务文件是 `config/generic_flight_mission.xml`，可以通过只读参数替换：

```bash
ros2 launch mavros_xyz_position_offboard flight_stack.launch.py \
  enable_control:=false \
  mission_xml_path:=/absolute/path/to/mission.xml
```

启用真实控制前必须完成 SITL 和无桨台架验证：

```bash
ros2 launch mavros_xyz_position_offboard flight_stack.launch.py \
  enable_control:=true \
  mission_xml_path:=/absolute/path/to/mission.xml
```

任务 XML 只在启动时加载。未知节点、无效动作参数和不支持的能力会导致启动或任务失败；任务终态
不会自动重新开始。

## 构建测试

```bash
source /opt/ros/jazzy/setup.bash
source /home/pi/LSLIDARN10P/install/setup.bash
colcon build --symlink-install \
  --packages-select mission_core mission_sim mavros_xyz_position_offboard \
  --parallel-workers 1
colcon test --packages-select mavros_xyz_position_offboard \
  --event-handlers console_direct+
colcon test-result --all --verbose
```

通用核心和模拟器也可以不加载 ROS 单独构建，具体命令见仓库根目录 README。
