# 大模型初检 ROI 改造说明（roi_mode: yolo）

> 已由 `roi_mode: depth` 的深度地面差分方案替代，当前 launch 不再启动 YOLO 节点。
> 本文仅保留为历史方案记录；现行说明见 `深度平面差分改造说明.md`。

目标：在"动态 ROI / cluster 模式"之上再加一级视觉初检——用 YOLO-World 大模型
对彩色图做板面开放词汇检测，按检测框直接在眼系点云上圈 ROI，进一步降低对
停车位置/粗框标定的依赖。三层回退链：**yolo → cluster → fixed**。

## 改动文件

- `scripts/yolo_board_detect_node.py`：新增，YOLO-World 板面初检节点（Python）
- `src/ysCamera3DSolver.cpp` / `include/ysCamera3DSolver.hpp`：订阅检测框、
  眼系像素矩形裁剪、参数注入
- `include/pclCalcTransform.hpp`：`getTargetCloud()` 新增 yolo 分支（优先级最高）
- `config/polish_params.yaml`：`roi_mode: yolo` + 3 个 yolo 参数
- `launch/elite_polish.launch.py`：新增 yolo_board_detect_node 启动
- `CMakeLists.txt`：`install(PROGRAMS scripts/yolo_board_detect_node.py ...)`

## 工作方式

1. `yolo_board_detect_node` 订阅彩色图（默认 `/camera/color/image_raw`），
   YOLO-World 按 `prompts`（默认 `metal plate,board`）检测板面；每帧取置信度
   最高的匹配框发布到 `/elite_polish/board_bbox`
   （`std_msgs/Float32MultiArray`，data = [x1, y1, x2, y2, conf, unix_timestamp_sec]）。
   **无检测时不发布**（简单约定，C++ 侧靠超时判断失效）。
2. 状态机发 `/elite_vision_job_cmd`(=1) 后，`ysCamera3DSolver` 点云回调里：
   - `roi_mode=="yolo"` 时先检查缓存 bbox：未收到过、或距当前点云时刻超过
     `yolo_bbox_max_age`（默认 3.0s）→ 视为无效，直接走回退；
   - 点云必须 `is_organized`（organized 才能按像素矩形裁剪），否则走回退；
   - bbox 四边按 `yolo_roi_shrink_ratio`（默认 0.1）内缩（防板边深度噪声），
     在眼系原始点云上按行列区间裁剪，丢弃 NaN 点；
   - 裁剪云用**拍摄时刻同一 FK×手眼矩阵**（与 `base_cloud_` 同一来源）变换到
     base 系，注入 `YsPCLCalcTransform::yolo_target_cloud`。
3. `getTargetCloud()` 里 yolo 分支优先级最高：注入云 >100 点即直接作为
   targetCloud（**跳过 box 裁剪**），后续精 RANSAC + refit + 覆盖范围质量门
   原样复用；yolo 分支还保存 `template_file` 前缀下 `yolo_target.pcd` 供留底。
4. `planeFrame` 原点：yolo 实际生效时（`roi_actual=="yolo"`）跟随拟合平面质心
   （同 cluster 模式，curve_box 跟随工件）；回退到 cluster/fixed 时沿用原有判定，
   cluster/fixed 既有行为不变。

## 回退链行为

`roi_mode: yolo` 时按以下顺序尝试，全部失败则本次视觉任务失败（`try_count_`
累计，超 5 次发 failed 位姿）：

| 级 | 条件 | 失败原因举例 |
|---|---|---|
| yolo | 缓存 bbox 新鲜 + 点云 organized + 内缩矩形 ≥8px + 裁剪有效点 ≥100 | 检测节点没跑/没检出、bbox 超 3s、点云非 organized |
| cluster | `dynamic_coarse_box_min/max` 已配置（非零） | 粗框未配置、无合格候选平面 |
| fixed | `dynamic_roi_fallback_to_fixed: true` | 该参数为 false 时不再回退 |

回退过程的 `[roi]` 日志依次打印：`yolo mode failed ...; try cluster fallback`
→ `cluster fallback succeeded/failed/skipped` → `using fixed target box`。

注意：`roi_mode: cluster` / `fixed` 的旧行为完全不变（yolo 分支不介入）。

## 关键参数

ysCamera3DSolver 段（config/polish_params.yaml）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `roi_mode` | `yolo` | yolo/cluster/fixed，三层链式回退 |
| `yolo_bbox_topic` | `/elite_polish/board_bbox` | 检测框话题 |
| `yolo_bbox_max_age` | 3.0 | bbox 超过该时长未更新视为失效(s) |
| `yolo_roi_shrink_ratio` | 0.1 | 检测框四边内缩比例，防边缘深度噪声 |

yolo_board_detect_node（launch 传入）：

| 参数 | 默认 | 作用 |
|---|---|---|
| `model_path` | `~/Documents/elite_robot_ws/YOLO/yolov8x-worldv2.pt` | YOLO-World 权重 |
| `prompts` | `metal plate,board` | 开放词汇检测目标（逗号分隔） |
| `conf` | 0.2 | 置信度阈值 |
| `color_topic` | `/camera/color/image_raw` | 彩色图话题 |
| `bbox_topic` | `/elite_polish/board_bbox` | 发布话题（与 C++ 侧一致即可） |
| `show_debug_image` | launch 中 True | 是否发布标注调试图 |

已知约定：Float32MultiArray 元素为 float32，unix 时间戳(~1.7e9)被量化到
约百秒级精度，仅作日志参考；C++ 侧新鲜度判断以 **bbox 消息到达时刻**为准。

## 现场验证方法

1. 编译：`cd ~/Documents/elite_robot_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select elite_polish_app`
2. 启动顺序：机器人驱动 → 相机 → `ros2 launch elite_polish_app elite_polish.launch.py`
   （launch 已含 yolo_board_detect_node，无需单独启动）。
3. 确认话题：`ros2 topic hz /elite_polish/board_bbox`（有板时应持续发布）；
   `ros2 topic echo /elite_polish/board_bbox` 看框坐标和 conf。
4. 看调试图：`rqt_image_view /elite_polish/board_bbox_image`
   （或 Rviz 加 Image 面板），确认绿框套住板面、无框时无发布。
5. 触发一次视觉任务（命令 3），看 ysCamera3DSolver 终端日志：
   - 成功：`[roi] yolo: bbox crop [...] points N` + `[roi] yolo mode succeeded, bbox target cloud size N`
     + `[plane fit] OK ...`；
   - 回退：`[roi] yolo: ... fallback` → `[roi] cluster fallback ...` → `[roi] using fixed target box`。
6. 留底点云：`template_file` 前缀目录下 `yolo_target.pcd`（yolo 裁剪结果）、
   `base.pcd`、`plane_fit_inliers.pcd`，可用 `pcd_box_tool.py` / pcl_viewer 复查。
7. 若检测不到板：先调 `prompts`/`conf`（launch 里改），确认 `/camera/color/image_raw`
   有图且画面里看得到板；实在不用 yolo 时把 `roi_mode` 改回 `cluster` 即可恢复原行为。
