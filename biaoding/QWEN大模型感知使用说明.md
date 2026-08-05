# Qwen 大模型感知使用说明（与 YOLO 切换）

## 0. 重要：Qwen 返回的是 0~1000 归一化坐标

`qwen3.7-plus` 返回的 `bbox` 不是像素坐标，而是相对图像宽高的千分比坐标：

- 图像中心 = `[500, 500]`
- 换算公式：`像素 = 千分值 / 1000 * 图像宽或高`

`qwen_perception_node.py` 和 `biaoding/qwen_box_test.py` 都已经自动做这个换算，
不需要手动处理。以后如果自己写脚本调用 Qwen，必须按这个公式换算，否则绿框和
3D 坐标都会偏。

## 1. 与 YOLO 的关系

新功能放在 `src/qwen_vision`：

- `qwen_perception_node.py`：Qwen-VL 检测 + 图漾深度测距 + 手眼标定 + TF 转基座；
- `vision_backend_switch.py`：运行时在 Qwen / YOLO 之间切换；
- `yolo_grasp.py` 已兼容：优先调用 `/vision_perception/set_enabled`，没有切换器时
  自动回退到 `/yolo_perception/set_enabled`。

两者输出同一个 `/target_object_pose`（`cs66_base_link` 系），因此抓取主程序不用
区分后端。

## 2. 启动

前提：机械臂驱动、图漾相机、手眼标定 `hand_eye_result.json` 都已就绪。

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/elite_robot_ws/install/setup.bash

# 默认 Qwen，同时把 YOLO 也拉起来备用
ros2 launch qwen_vision qwen_vision.launch.py backend:=qwen run_yolo:=true
```

若只想跑 Qwen：

```bash
ros2 launch qwen_vision qwen_vision.launch.py backend:=qwen run_yolo:=false
```

## 3. 运行时切换 YOLO / Qwen

```bash
# 切到大模型
ros2 topic pub --once /vision_backend std_msgs/msg/String "data: 'qwen'"

# 切回 YOLO
ros2 topic pub --once /vision_backend std_msgs/msg/String "data: 'yolo'"

# 查询当前后端
ros2 service call /vision_perception/backend std_srvs/srv/Trigger
```

切换器会先关闭另一个后端，再开启目标后端，保证 `/target_object_pose` 同一时刻
只有一个发布者。

## 4. 单独绿框测试（不经过机械臂）

独立脚本只订阅图漾彩色图并调用 Qwen，不碰 TF/手眼，用来确认 Qwen 的框到底准不准。

```bash
cd ~/Documents/elite_robot_ws/biaoding
python3 qwen_box_test.py --target apple
```

按键：

- `s`：手动识别一次；
- `t`：切换目标；
- `q`：退出；
- 默认每 5 秒自动识别一次，`--auto 0` 可关闭自动识别。

绿框图保存在：

```bash
/tmp/qwen_box_test/annotated_*.png
```

如果这里绿框已经偏，说明是 Qwen 模型本身的定位问题，和机械臂/标定无关。

## 5. 正式节点手动测试

```bash
# 持续识别（等价于按 yolo_grasp.py 里 e 键的效果）
ros2 service call /qwen_perception/set_enabled std_srvs/srv/SetBool "data: true"

# 单次同步检测，返回 JSON 坐标
ros2 service call /qwen_perception/locate_object_sync std_srvs/srv/Trigger

# 查看实时调试图
cd ~/Documents/elite_robot_ws/YOLO
python3 view_annotated.py /qwen/annotated_image
```

正式节点每次检测的原图 + 绿框/红叉也会自动保存到：

```bash
/tmp/qwen_vision/annotated_*.png
```

红叉表示实际用于计算坐标的像素点。

## 6. 大模型识别兜底

Qwen 模式下抓取流程会等待两种信号之一：

- `/target_object_pose`：识别到目标；
- `/qwen/perception_done`：模型已返回结果（即使没找到目标）。

只有超过 90 秒仍未等到结果才会判定超时并回 Home2，避免机械臂在 API 还没返回时
就提前回位。YOLO 模式保持原来的 8 秒短等待，不受影响。

## 7. 坐标链路

```text
Qwen 返回 0~1000 归一化 bbox
  -> 换算成图像像素 bbox
  -> 深度图取目标点（图漾 0.25mm/LSB）
  -> 相机系 (Xc, Yc, Zc)
  -> hand_eye_result.json：T_cam2tool
  -> TF：T_tool2base（cs66_tool0 -> cs66_base_link）
  -> /target_object_pose
```

关键点：

- 彩色帧到达时立即锁存当时的 TF，避免 API 返回后拿到移动后的机械臂位姿；
- 彩色/深度时间戳差超过 0.5 秒的帧会被丢弃；
- 日志里的 `[时间戳]` 可以看到实际时间差。

## 8. 关键参数（config/qwen_vision.yaml）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `target` | `apple` | 检测目标，多个用逗号分隔，`all` 检测全部 |
| `depth_scale` | `0.00025` | 图漾 0.25mm/LSB；换 RealSense 改 `0.001` |
| `use_depth_centroid` | `false` | 是否用深度簇/圆拟合代替 bbox 几何中心 |
| `depth_tol_m` | `0.03` | 深度簇容差（米） |
| `depth_color_tol_s` | `0.5` | 彩色/深度最大时间差（秒） |
| `api_timeout` | `60.0` | Qwen API 超时（秒） |
| `poll_interval` | `1.0` | 持续识别时两次调用的间隔 |
| `use_model_center` | `false` | 是否优先用模型返回的 center 点（不建议开） |
| `base_frame` / `tool_frame` | `cs66_base_link` / `cs66_tool0` | TF 坐标系 |

## 9. 常见问题

- 绿框偏：先跑 `qwen_box_test.py` 单独确认；如果偏，检查 Qwen 原始 bbox 是否是
  `0~1000` 归一化值，是否已经换算；
- 大模型没反应：检查 `api_key`、`base_url`、`model`，以及网络/DNS；
- `Connection timed out` / `name resolution`：机器到外网或 DNS 不通，先修网络；
- 切到 YOLO 失败：确认 `yolo_grasp_perception.py` 已启动，且
  `/yolo_perception/set_enabled` 可用；
- 坐标明显偏：重新做手眼标定并运行 `update_hand_eye_json.py`；
- 日志出现 `[TF诊断]`：说明拍照时刻 TF 没取到、退回最新 TF，机械臂如果动过坐标会偏；
- 深度全为 0：确认图漾深度话题有数据，且当前相机不是 RealSense（RealSense 要把
  `depth_scale` 改成 `0.001`）。
