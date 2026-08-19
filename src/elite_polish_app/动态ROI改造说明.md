# 动态 ROI / 聚类识别改造说明

目标：移动平台带着机械臂和深度相机时，允许停车位置误差增大，不再依赖固定 target_box。

## 改动文件

- `include/pclCalcTransform.hpp`：新增 cluster ROI 模式
- `src/ysCamera3DSolver.cpp`：加载并注入动态 ROI 参数
- `config/polish_params.yaml`：新增动态 ROI 参数

## 工作方式

`roi_mode: cluster` 时：

1. 在 `dynamic_coarse_box` 内对点云降采样；
2. 用带法向约束的平面分割连续提取候选平面；
3. 按法向角度、xy/z 质心先验过滤候选，并用先验中心软权重选择最优板面；
4. 用候选板面 bbox 在原始全分辨率点云中回捞局部 ROI；
5. 后续仍走原有 RANSAC 精拟合 + refit + 覆盖范围质量门；
6. `planeFrame` 原点在 cluster 模式下取候选平面质心，使 `curve_box` 跟随工件；
7. cluster 失败且 `dynamic_roi_fallback_to_fixed: true` 时，自动回退固定 `target_box`。

## 关键参数

| 参数 | 作用 |
|---|---|
| `dynamic_coarse_box_min/max` | 粗搜索范围，必须覆盖最大停车误差和相机视野 |
| `dynamic_expected_center` | 工件板面质心先验（base 系） |
| `dynamic_center_xy_tol` | 停车 xy 允许范围 |
| `dynamic_center_z_tol` | 用于排除地面等 z 不同的平面 |
| `dynamic_normal_tolerance_deg` | 候选平面法向容差 |
| `dynamic_min_inliers` | 候选平面最少点数 |
| `dynamic_roi_xy/z_padding` | 候选 bbox 回捞余量 |
| `dynamic_roi_fallback_to_fixed` | 失败是否回退固定盒 |
| `dynamic_origin_from_plane_centroid` | 轨迹原点是否跟随候选平面质心 |

## 标定方法

1. 把车故意停到最大正向/负向误差位置，拍照保存点云；
2. 用 `pcd_box_tool.py` / Rviz 确认这些位置下板面完整出现在 `dynamic_coarse_box` 内；
3. 确认地面、墙面、夹具不会和板面同时满足 xy/z 先验过滤；
4. 逐步加大 `dynamic_center_xy_tol`，直到所有停车样本都能 `[roi] candidate ... ACCEPTED`；
5. 实际跑视觉命令，检查 `[plane fit] OK`、`dynamic target cloud size` 和 `curve box cloud size`；
6. 失败时先看 `[roi]` 日志：粗框太小、法向不对、还是先验中心不对。
