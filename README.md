# Generic Robot Task Planner

这是一个平台无关的 XML 任务编排核心，以及一个 PX4/MAVROS 执行适配器。任务流程由
`mission_core` 解释；平台只负责把标准状态转换为 `WorldState`，并执行 `ActionIntent`。

```text
平台遥测 / UDP
      |
      v
WorldState + CapabilityState + MissionEvent
      |
      v
mission_core::SafetySupervisor + MissionTree(XML)
      |
      v
ActionIntent -> IActionExecutor -> 平台控制输出
```

仓库不再包含旧的 PX4 专用 BehaviorTree.CPP 树。PX4 包现在只包含 ROS 入口、健康检查、通信、
`Px4ActionExecutor` 和 `Offboard` 适配器。

## 模块

| 模块 | 作用 | 依赖 |
| --- | --- | --- |
| `mission_core` | 标准状态、能力、动作生命周期、安全仲裁和 XML 任务运行器 | C++17、tinyxml2 |
| `mission_sim` | 确定性模拟执行器、事件注入和离线示例 | `mission_core` |
| `mavros_xyz_position_offboard` | PX4/MAVROS 状态转换、动作执行和 ROS 入口 | ROS 2、MAVROS、LCP |

核心接口定义在 `mission_core/include/mission_core/`：

- `WorldState`、`CapabilityState`、`ActionIntent`、`ActionFeedback`
- `MissionRuntime`、`MissionEvent`、`SafetyDecision`
- `IActionExecutor`、`CapabilityRegistry`
- `MissionTree`、`SafetySupervisor`

核心没有 ROS、MAVROS、LCP、UDP 或 PX4 头文件，可以独立构建。动作执行器通过能力注册和参数
校验声明平台边界；同一个 XML 可以交给模拟执行器或 PX4 执行器运行。

## XML 任务

运行器支持顺序、回退、等待、能力门禁、动作和事件节点：

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <RequireCapability capability="navigate"/>
      <ExecuteAction id="go_a" capability="navigate"
                     x="2.0" y="1.0" z="0.0" frame="local_enu"/>
      <WaitEvent event="release"/>
      <ExecuteAction id="go_b" capability="navigate"
                     x="-1.0" y="0.5" z="0.0" frame="local_enu"/>
    </Sequence>
  </BehaviorTree>
</root>
```

XML 在启动时解析和校验。未知节点、无效参数、缺少能力和无效坐标系都会产生明确错误。
任务完成、失败或取消后保持终态，不自动重启。

## PX4 运行

PX4 适配器目前提供 `navigate`、`takeoff`、`hold` 和 `land`。它负责 OFFBOARD 预热、模式切换、
ARM/Disarm、连续设定点、实际位置容差和安全降落。通用任务通过现有 ROS 节点运行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/pi/LSLIDARN10P/install/setup.bash
source install/setup.bash

ros2 launch mavros_xyz_position_offboard flight_stack.launch.py \
  enable_control:=false
```

启用控制输出前应先完成 SITL 和无桨台架验证：

```bash
ros2 launch mavros_xyz_position_offboard flight_stack.launch.py \
  enable_control:=true \
  mission_xml_path:=/absolute/path/to/mission.xml
```

监控模式不会发送设定点、模式或 ARM 请求。

## 离线构建和测试

```bash
cmake -S mission_core -B /tmp/mission_core_build
cmake --build /tmp/mission_core_build -j2
ctest --test-dir /tmp/mission_core_build --output-on-failure

cmake -S mission_sim -B /tmp/mission_sim_build
cmake --build /tmp/mission_sim_build -j2
ctest --test-dir /tmp/mission_sim_build --output-on-failure
```

模拟示例：

```bash
/tmp/mission_sim_build/mission_sim_demo mission_sim/config/flight_demo.xml
/tmp/mission_sim_build/mission_sim_demo mission_sim/config/navigation_demo.xml
```

ROS 集成测试需要 ROS 2 Jazzy 和 `lslidar_msgs` overlay：

```bash
colcon build --symlink-install \
  --packages-select mission_core mission_sim mavros_xyz_position_offboard \
  --parallel-workers 1
colcon test --packages-select mavros_xyz_position_offboard \
  --event-handlers console_direct+
colcon test-result --all --verbose
```

SITL 和真实飞行不包含在离线测试中。

更多接口、数据流和兼容边界见 [docs/mission-platform.md](docs/mission-platform.md)。
