# Mission orchestration platform

本仓库的任务运行器由 `mission_core` 提供，平台包只提供状态转换、能力声明和动作执行。

```text
platform telemetry -> WorldState -> SafetySupervisor
                                      |
                                      v
                              MissionTree(XML)
                                      |
                                      v
ActionIntent <- IActionExecutor <- ActionFeedback
      |
      v
platform commands
```

## 核心边界

`mission_core` 是普通 C++17 库，不包含 ROS、MAVROS、LCP、UDP 或 PX4 头文件。核心提供：

- `WorldState`：带时间戳和坐标系的位姿、速度、连接状态及标准化事实。
- `CapabilityState`：`unsupported`、`unavailable`、`available`、`busy`。
- `ActionIntent` 和 `ActionFeedback`：带任务 ID、动作 ID、能力和参数的动作生命周期。
- `MissionRuntime`、`MissionEvent` 和 `SafetyDecision`：任务状态、事件和安全仲裁结果。
- `IActionExecutor`：提交、取消、暂停、恢复、周期更新和反馈查询。
- `MissionTree`：启动时解析 XML，每个控制周期执行一次。
- `SafetySupervisor`：处理状态新鲜度、连接、事实、任务时限和紧急升级。

能力由平台执行器在编译期注册。注册时可以绑定参数校验函数；XML 声明的能力在启动时检查，
暂时不可用的能力由 `RequireCapability` 等待，平台根本不支持的能力直接失败。

## XML 词汇

```xml
<Sequence>...</Sequence>
<ReactiveSequence>...</ReactiveSequence>
<Fallback>...</Fallback>
<ReactiveFallback>...</ReactiveFallback>
<RequireCapability capability="navigate"/>
<ExecuteAction id="go_a" capability="navigate"
               x="2" y="1" z="0" frame="local_enu"/>
<Wait seconds="1.5"/>
<WaitEvent event="release"/>
<EmitEvent event="arrived" payload="{}"/>
```

XML 只组合任务逻辑。它不包含 PX4 模式、MAVROS 消息或 LCP 原始字段。第一版不支持运行中替换
任务、不从目标自动生成规划，也不加载动态插件。

## PX4 适配器

`mavros_xyz_position_offboard` 的 `Px4ActionExecutor` 将 MAVROS/LCP 遥测转换为 `WorldState`，
并实现：

| 能力 | 参数 | 完成条件 |
| --- | --- | --- |
| `navigate` | `x`、`y`、`z`，可选 `yaw`、`frame=local_enu` | 实际位置进入容差 |
| `takeoff` | 正的 `height_m` | 实际位置达到相对高度 |
| `hold` | 可选非负 `seconds` | 保持时间结束 |
| `land` | 无 | 已触地且已解除解锁 |

PX4 执行器负责 OFFBOARD 预热、设定点维护、模式和 ARM 请求。飞行中动作被取消时会继续执行
安全降落，触地后才进入 `MANUAL` 和解除解锁。

## 周期数据流

ROS 入口每个周期执行：

```text
采集 MAVROS/LCP 状态和 UDP 事件
  -> 生成 WorldState / CapabilityState / MissionEvent
  -> MissionTree::tick()
  -> SafetySupervisor 仲裁
  -> XML 节点提交或等待动作
  -> Px4ActionExecutor 更新设定点和反馈
  -> 输出统一任务状态与事件
```

终态 tick 仍然调用执行器更新，以便安全结束动作继续维护必要输出。

## 模拟与验证

`mission_sim` 使用同一个 `IActionExecutor` 接口，不连接 ROS 或硬件。它支持动作完成时间、拒绝、
暂停恢复、事件注入、过期状态和能力不可用测试。

```bash
cmake -S mission_core -B /tmp/mission_core_build
cmake --build /tmp/mission_core_build -j2
ctest --test-dir /tmp/mission_core_build --output-on-failure

cmake -S mission_sim -B /tmp/mission_sim_build
cmake --build /tmp/mission_sim_build -j2
ctest --test-dir /tmp/mission_sim_build --output-on-failure
```

ROS 集成测试和 PX4 SITL 需要额外环境；离线测试通过不代表已经完成真实飞行验证。
