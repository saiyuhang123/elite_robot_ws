# 移植审查报告：elite_polish_app

> 审查对象：`docs/移植方案_ysrob打磨到EliteCS66.md` + 实际移植代码 `src/elite_polish_app/`
> 审查日期：2026-07-26
> 结论：**方案架构判断全部成立，但实际移植存在 2 个致命 bug、3 个会导致流程跑不通的 bug，以及若干文档与代码不一致之处。**

---

## 一、方案文档验证：架构判断正确

| 文档论断 | 验证结果 |
|---|---|
| IK 在状态机内部完成，只换 URDF 链名即可 | ✅ 正确。KDL chain 为 `cs66_base_link → cs66_tool0`（`ysURForceAppControl.cpp:163-165`），与 `eli_cs_robot_description/urdf/cs_macro.xacro:111,335` 一致 |
| 桥接节点屏蔽 Topic/Action 差异 | ✅ 桥接订阅 `/YsUR_driver/joint_trajectory`，转发到 `/scaled_joint_trajectory_controller/follow_joint_trajectory`，名字均正确 |
| 关节名/TF prefix 全局替换 | ✅ joint_names 填 `cs66_` 前缀（`ysURForceAppControl.cpp:63`），关节顺序与 `elite_cs_controllers.yaml:105-111` 完全一致 |
| 力传感器 frame 假设 | ✅ wrench 发布 frame_id 为 `cs66_tool0`；URDF 中 `ft_frame`/`flange`/`tool0` 零偏移重合，旋转等价，无实际问题 |
| 重力/偏置旧值需重标定 | ✅ 已清零并改为启动时自动采集 200 样本标定（`ysURForceAppControl.cpp:87-95`） |
| 三个示教关节角需重新示教 | ✅ 已换占位值 `{8.2, -93.6, -109.4, 61.4, 86.4, -11.2}°` 并标注 TODO（`ysURForceAppControl.cpp:207-227`），**跑流程前必须示教** |
| AGV 状态清理 | ✅ 状态已跳过/直连视觉（567-620 行），抛光结束不再链 AGV（1023-1024 行） |

---

## 二、致命 bug（不修则系统完全不动）

### Bug 1：`tr_camera` link 不存在 → 整个状态机瘫痪

- `ysURForceAppControl.cpp:148`：eye chain 写死为 `cs66_base_link → tr_camera`
- `tr_camera` 在 kybot / Elite 的 URDF 中**不存在**（全 src 搜索仅本包三处引用）
- 后果链：`getChain` 失败 → `ys_eye_fk_solver_` 为 NULL → `timer_callback` 守卫条件（`ysURForceAppControl.cpp:270-274`）永假 → **包括 GO_HOME 在内的所有状态都不执行**，节点启动后只有一行 FATAL 日志然后空转
- `ysCamera3DSolver.cpp:93` 有同样问题：点云回调里 `ys_eye_fk_solver_ != nullptr` 永假 → 视觉永远不解算、不发结果

**修复方向**：把手眼标定（工作区已有 `easy_handeye2` / `biaoding`）得到的相机 link 加进 kybot cell URDF，并同步修改这两处链名。

### Bug 2：多点轨迹所有点 `time_from_start` 相同 → 被控制器 reject 且无声失败

ysrob 自研驱动自己做插补、忽略时间戳；标准 `joint_trajectory_controller` **会校验时间单调递增，不满足直接拒绝 goal**。实际代码：

| 位置 | 问题 |
|---|---|
| `ysURForceAppControl.cpp:652, 781`（`polish_goPolishBase`） | 两个点都是 `(4s, 8ms)` |
| `ysURForceAppControl.cpp:867, 897`（`polish_doForceContact`） | 30~60 个点全是固定 8ms |
| `ysURForceAppControl.cpp:963, 990`（`polish_goBackHome`） | 240 个点全是 20ms |

同时桥接节点（`elite_joint_trajectory_bridge.cpp`）只设了 `result_callback`，**没有 `goal_response_callback`**——goal 被 reject 完全无感知。表现为"机械臂不动、没有任何报错"，极难排查。

**修复方向**：循环内改为 `tmpPt.time_from_start = deltaT * (i + 1)`；桥接节点补 goal_response 回调并打日志。

---

## 三、会导致流程跑不通的 bug

### ~~Bug 3：力传感器 topic 名与仓库配置矛盾~~ 【已排除，2026-07-26 实测】

- 代码订阅 `/force_torque_sensor_broadcaster/wrench`（`ysURForceAppControl.cpp:197`）
- 仓库配置 `elite_cs_controllers.yaml:44` 写的是 `topic_name: ft_data`，疑似矛盾
- **2026-07-26 真机实测**：`ros2 topic echo /force_torque_sensor_broadcaster/wrench --once` 正常输出（frame_id: `cs66_tool0`），即实际运行配置并未采用仓库 yaml 中的 `ft_data`。**代码订阅正确，此项非 bug**。
- 实测空载残差约 force ~1 N / torque ~0.1 N·m，属于未标定偏置；代码启动时自动采集 200 样本做偏置标定，行为正常。

### Bug 4：launch 文件没启动 `ysCamera3DSolver`

- `launch/elite_polish.launch.py` 只起了 bridge、`ysURForceAppControl`、`ysAppCommand` 三个节点
- 发命令 3（视觉）后状态机在 sub_step 302/303 死等：`/elite_vision_job_cmd` 无人接收，`ys_vision_job_done_` 永不置位
- 文档第六步的 launch 模板里有相机节点，实际 launch 漏了
- 次要：`prefix='x-terminal-emulator -e'` 在无桌面终端的环境会让 `ysAppCommand` 起不来

### Bug 5：`ys_gravityRepairWrench` 力矩分量计算错误（继承自 ysrob 的原生 bug）

`ysURForceAppControl.cpp:477-479`：

```cpp
value.torque.data[1] = data.torque.data[0] - (...);  // 应为 data.torque.data[1]
value.torque.data[2] = data.torque.data[0] - (...);  // 应为 data.torque.data[2]
```

三个分量都减自 `data.torque.data[0]`。当前重力参数清零时恰好无影响，**一旦标定重力补偿必现错误**。ysrob 原版同样错（移植时原样带入）。

---

## 四、设计层面的风险

**力控阶段 25Hz 连续发单点 action goal**

`polish_doCurvePolishing` 每 40ms 发一条单点轨迹（`time_from_start = 20ms`），即 25Hz 连续向 JTC 发 action goal。每个新 goal 抢占旧 goal，旧 goal 以 ABORTED 结束 → result_callback 以 25Hz 刷 "Trajectory failed" 错误日志。功能上也许能动，但这是把 ysrob 的 Topic 流式语义硬套在 Action 上，脆弱。

**长期建议**：改走流式接口，如 `forward_position_controller`，或使用 JTC 的 topic 输入方式（若控制器版本支持）。

---

## 五、文档与代码不一致

| 项 | 文档说法 | 实际情况 |
|---|---|---|
| launch 可执行名 | `eliteURForceAppControl` / `eliteCamera3DSolver` / `eliteAppCommandNode` | CMake target 实际为 `ysURForceAppControl` / `ysCamera3DSolver` / `ysAppCommand`（文档滞后，以 CMake 为准） |
| AGV 清理 | "已清理 AGV 依赖" | 状态逻辑已跳过，但 AGV 的 pub/sub 仍残留（`ysURForceAppControl.cpp:189-191`，topic `ysrob_agv_job_cmd` / `/ysrob_agv_job_result`），命令 1/2 仍在菜单（`ysAppCommandNode.cpp:44-45`）。无害但不干净 |
| 力传感器 topic | 文档前置验证第 2 条写 `/wrench` | 仓库 yaml 写 `ft_data`（见 Bug 3） |

---

## 六、次要问题（不阻塞，建议择机处理）

- `ysFTSensorData.cpp/.hpp`、`pclTemplateAlign.cpp` 在包内但未编译——死代码
- 接触力发布 topic 仍为 `ys_contact_fts_broadcaster/wrench`——ys_ 命名残留，仅自用
- 视觉裁剪盒坐标（`pclCalcTransform.hpp:49-50, 59-83, 167-168`）、`getCurveFrame` 的 0.9306/1.89539、`frame_polishcloud_base_` 等工艺参数仍是 ysrob 旧值——文档已声明需重标定，属待办
- 视觉结果 pose 的 `header.frame_id` 写 `"tr_camera"` 但数值实际是 base 系变换——语义错误，消费方未使用 frame_id，仅误导
- `CMakeLists.txt` 未显式 `find_package(kdl_parser)` / `find_package(urdf)`，目前靠 trac_ik_lib 传递依赖编过，换环境可能挂
- `subCommandStateCB` 收到未知命令会把 `app_cmd_` 打成 NOTHING 并清 `sub_step_`（260-263 行）——误发命令会静默中断正在跑的任务，原生行为

---

## 七、修复优先级清单

| 优先级 | 项 | 说明 |
|---|---|---|
| P0 | Bug 1 `tr_camera` | 相机 link 进 URDF + 改两处链名，否则整个系统不动 |
| P0 | Bug 2 `time_from_start` | 所有多点轨迹改为时间递增；桥接补 goal_response 日志 |
| P1 | ~~Bug 3 力传感器 topic~~ | 已实测排除：`/wrench` 正确，代码无需改 |
| P1 | Bug 4 launch 补相机节点 | 否则视觉流程必卡死 |
| P1 | 示教三个关节角 | 占位值不可用于实际运行 |
| P2 | Bug 5 重力补偿力矩 | 标定重力前必须修 |
| P2 | 文档更新 | launch 可执行名、AGV 残留、力 topic、风险清单补充 Bug 1/2 两条 |
| P3 | 流式接口改造 | 替代 25Hz action goal 抢占模式 |
