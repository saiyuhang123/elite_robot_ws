# qwen_vision（Elite CS66 + 图漾 Percipio 版）

把 `/home/nvidia/kybot_ws/src/qwen_vision` 的“大模型检测 + 深度测距”逻辑移植到当前
机器人项目，坐标计算不再用 `map` TF，而是和 `YOLO/yolo_grasp_perception.py` 完全一致：

```text
大模型返回 bbox
  -> 图漾深度图取 bbox 中心 7x7 中值（0.25mm/LSB）
  -> 相机系 3D 点
  -> hand_eye_result.json：相机 -> tool0
  -> TF：tool0 -> cs66_base_link
  -> /target_object_pose（cs66_base_link 系）
```

## 一键启动（含 YOLO 切换）

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash

ros2 launch qwen_vision qwen_vision.launch.py backend:=qwen run_yolo:=true
```

启动前需要：
- 机械臂驱动已启动（提供 `cs66_base_link -> cs66_tool0` TF）；
- 图漾相机已启动（`/camera/color/image_raw`、`/camera/depth/image_raw`、
  `/camera/color/camera_info`）；
- `biaoding/hand_eye_result.json` 是最新标定。

## 运行时切换 YOLO / Qwen

```bash
# 切到大模型
ros2 topic pub --once /vision_backend std_msgs/msg/String "data: 'qwen'"

# 切回 YOLO
ros2 topic pub --once /vision_backend std_msgs/msg/String "data: 'yolo'"

# 查询当前后端
ros2 service call /vision_perception/backend std_srvs/srv/Trigger
```

切换器会先关掉另一个后端，再开启目标后端，保证 `/target_object_pose` 同一时刻只有
一个发布者。抓取主程序 `yolo_grasp.py` 已改为优先调用通用服务
`/vision_perception/set_enabled`，切换器不在时仍会回退到原
`/yolo_perception/set_enabled`。

## 只跑 Qwen（不启动 YOLO）

```bash
ros2 launch qwen_vision qwen_vision.launch.py backend:=qwen run_yolo:=false
```

或手动启动：

```bash
ros2 run qwen_vision qwen_perception_node \
  --ros-args --params-file src/qwen_vision/config/qwen_vision.yaml
ros2 run qwen_vision vision_backend_switch \
  --ros-args -p default_backend:=qwen
```

## 接口

| 类型 | 名称 | 说明 |
|---|---|---|
| 话题 | `/target_object_pose` | `PoseStamped`，基座系目标点（与 YOLO 相同） |
| 话题 | `/qwen/annotated_image` | 画框后的调试图 |
| 文件 | `/tmp/qwen_vision/annotated_*.png` | debug 模式自动保存带绿框的原图 |
| 话题 | `/qwen/object_name` | 识别到的目标名 |
| 话题 | `/qwen/description` | 中文距离描述 |
| 话题 | `/qwen/perception_done` | 一次识别结束信号（success/failed） |
| 服务 | `/qwen_perception/set_enabled` | 持续识别开关 |
| 服务 | `/qwen_perception/locate_object` | 异步触发一次检测 |
| 服务 | `/qwen_perception/locate_object_sync` | 同步触发，返回 JSON |
| 服务 | `/qwen_perception/status` | 查询状态 |
| 话题 | `/vision_backend` | 发送 `yolo` / `qwen` 切换后端 |
| 服务 | `/vision_perception/set_enabled` | 开关当前后端 |

## 关键参数（config/qwen_vision.yaml）

- `target`：默认检测目标，可用逗号分隔多个，发 `all` 检测全部；
- `depth_scale`：图漾默认 `0.00025`（0.25mm/LSB），换 RealSense 时改 `0.001`；
- `use_depth_centroid`：用 bbox 内深度点群质心代替几何中心，修正大模型框偏；
- `depth_tol_m`：质心选取的深度容差，默认 0.05m；
- `base_frame` / `tool_frame`：默认 `cs66_base_link` / `cs66_tool0`；
- `poll_interval`：持续识别时两次大模型调用的间隔；
- `api_key` / `base_url` / `model`：通义千问 VL 接口配置。

## 大模型识别兜底

`yolo_grasp.py` 在 Qwen 模式下不会像 YOLO 那样 8 秒没目标就移动/回位，而是：

1. 等待 `/target_object_pose` 发布目标；
2. 或者等待 `/qwen/perception_done` 表示“模型已经返回结果”（没找到目标也算返回）；
3. 只有超过 `LLM_RESULT_TIMEOUT`（默认 90s）才判定超时。

这样机械臂不会在 Qwen API 还没返回时就提前回 Home2。

## 抓取

Qwen 后端开启后，现有 `yolo_grasp.py` 不需要改用法：

```bash
cd ~/Documents/elite_robot_ws/biaoding
python3 yolo_grasp.py
```

按 `g` 时它会通过 `/vision_perception/set_enabled` 开启当前后端，等到
`/target_object_pose` 后锁存目标并抓取。
