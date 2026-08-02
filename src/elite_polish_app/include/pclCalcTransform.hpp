#ifndef ysPCLCalcTransform_HPP
#define ysPCLCalcTransform_HPP

#include <Eigen/Core>
#include <algorithm>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <cmath>

class YsPCLCalcTransform
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  public:

    YsPCLCalcTransform () 
    {
      baseCloud_ = std::make_shared< PointCloud >();
      targetCloud_ = std::make_shared< PointCloud >();
      planeCloud_ = std::make_shared< PointCloud >();
      curveCloud_ = std::make_shared< PointCloud >();
    }
    ~YsPCLCalcTransform() {
    }

    // 逐层检查点云是否为空，任何一层不达标返回 false（调用方中止本次视觉任务）。
    // 原实现在空云时会产出垃圾坐标系甚至崩溃（savePCDFileBinary 空云抛异常）。
    bool
    calc (PointCloud::Ptr &base_cloud, Eigen::Matrix4f &result)
    {
      failed_ = false;
      pcl::copyPointCloud(*base_cloud, *baseCloud_);
      getTargetCloud();
      if (targetCloud_->points.size() < 100) {
        std::cout<<"[calc] FAIL: target cloud too small: "<<targetCloud_->points.size()<<std::endl;
        return false;
      }
      getPlaneFrame();
      if (failed_) {
        std::cout<<"[calc] FAIL: plane measure box empty"<<std::endl;
        return false;
      }
      getPlaneCloud();
      getCurveCloud();
      if (failed_ || curveCloud_->points.size() < 50) {
        std::cout<<"[calc] FAIL: curve cloud too small"<<std::endl;
        return false;
      }
      getCurveFrame();
      resultFrame_.setIdentity();
      resultFrame_ = (planeFrame_ * curveFrame_);
      result = resultFrame_;
      return true;
    }

    // ---- 可调参数（由 ysCamera3DSolver 从 ROS 参数注入；默认值 = 2026-07-27 标定值）----
    // 大盒 (base_link 系)
    Eigen::Vector4f target_box_min{0.520, -0.209, 0.498, 1.0};
    Eigen::Vector4f target_box_max{0.720,  0.178, 0.740, 1.0};
    // 三测点盒: x/y ±5mm, z 贯通
    double plane_box_zmin = 0.45, plane_box_zmax = 0.75;
    double plane_ox = 0.590, plane_oy = -0.120;  // 原点测点
    double plane_xx = 0.670, plane_xy = -0.010;  // X 轴测点(离机器人更远!)
    double plane_yx = 0.590, plane_yy =  0.100;  // Y 参考测点
    // RANSAC 平面拟合。约束法向接近世界 X+，避免把桌面等其它大平面误识别为工件面。
    bool plane_fit_enabled = true;
    double plane_fit_distance_threshold = 0.003;
    int plane_fit_max_iterations = 1000;
    int plane_fit_min_inliers = 1000;
    double plane_fit_min_inlier_ratio = 0.15;
    double plane_fit_max_rms = 0.0025;
    double plane_fit_max_normal_angle_deg = 10.0;
    // 2026-08-02 鲁棒化: 第二遍在首遍内点上重拟合(剔除被带入的板边/杂物)，
    // 以及内点覆盖面校验(内点只是一窄条时角度/位置都会歪，判失败宁可重拍)。
    bool plane_fit_refit_enabled = true;
    double plane_fit_min_extent_x = 0.12;  // 内点平面局部x(扫掠向)最小覆盖(m)
    double plane_fit_min_extent_y = 0.10;  // 内点平面局部y(竖直向)最小覆盖(m)
    // 2026-08-02: 板子平放在地、法兰向下打磨的布置。期望法向/平面参考默认按旧逻辑
    // (竖直板: 期望法向=世界X+, 平面参考=world_up_in_base); 模长>0.5 时覆盖。
    // 地面板配置: expected=(0.7431,-0.0120,-0.6691)(世界向下), up_ref=(0,1,0)(世界Y)。
    Eigen::Vector3f plane_expected_normal_override{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f plane_up_reference_override{0.0f, 0.0f, 0.0f};
    // 圆弧盒 (平面系)
    Eigen::Vector4f curve_box_min{-0.08, -0.14, -0.02, 1.0};
    Eigen::Vector4f curve_box_max{ 0.28,  0.24,  0.02, 1.0};
    // 圆弧坐标系常数
    double curve_radius = 0.9306;
    double curve_center_dz = 1.89539;
    double curve_tool_offset = 0.18;   // 打磨头伸出法兰长度(m)
    double curve_y_offset = 0.1;
    // 2026-08-01: 工具姿态世界水平化。板面物理垂直但拟合平面因相机/手眼畸变倾斜时，
    // 强制工具轴 = 世界X+(水平)、扫掠 = 世界Y(水平)、换道 = 世界Z(竖直)。
    bool tool_align_world = false;
    Eigen::Vector3f world_up_in_base{-0.7431, 0.0120, 0.6691};

    void getTargetCloud() {
      std::cout<<"base cloud size "<<baseCloud_->points.size()<<std::endl;
      targetCloud_->clear();
      ///---------------直通滤波
      pcl::CropBox<pcl::PointXYZ> boxFilter;
      boxFilter.setMin(target_box_min);
      boxFilter.setMax(target_box_max);
      boxFilter.setInputCloud(baseCloud_);
      boxFilter.filter(*targetCloud_);
      std::cout<<"target cloud size "<<targetCloud_->points.size()<<std::endl;
    }

    Eigen::Vector3f expectedWorldX() const {
      Eigen::Vector3f wup = world_up_in_base;
      if (wup.norm() < 1e-6f) {
        return Eigen::Vector3f::Zero();
      }
      wup.normalize();
      Eigen::Vector3f world_y = Eigen::Vector3f(0, 1, 0)
        - Eigen::Vector3f(0, 1, 0).dot(wup) * wup;
      if (world_y.norm() < 1e-6f) {
        return Eigen::Vector3f::Zero();
      }
      world_y.normalize();
      Eigen::Vector3f world_x = world_y.cross(wup);
      world_x.normalize();
      return world_x;
    }

    // 单趟 RANSAC 拟合 + 全部质量门(内点数/比例/法向角/RMS)。
    // tag 用于日志区分 pass1/pass2；成功返回 true 并填好输出参数。
    bool fitPlaneOnce(PointCloud::Ptr input, const Eigen::Vector3f &expected_normal,
        double max_angle_deg,
        Eigen::Vector3f &normal_out, float &d_out, PointCloud::Ptr &inlier_cloud_out,
        double &rms_out, double &ratio_out, double &angle_out, const char *tag) {
      pcl::SACSegmentation<pcl::PointXYZ> segmentation;
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      segmentation.setOptimizeCoefficients(true);
      segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
      segmentation.setMethodType(pcl::SAC_RANSAC);
      segmentation.setAxis(expected_normal);
      segmentation.setEpsAngle(
        max_angle_deg * 3.14159265358979323846 / 180.0);
      segmentation.setDistanceThreshold(plane_fit_distance_threshold);
      segmentation.setMaxIterations(plane_fit_max_iterations);
      segmentation.setInputCloud(input);
      segmentation.segment(*inliers, *coefficients);

      if (coefficients->values.size() < 4 || inliers->indices.empty()) {
        std::cout<<"[plane fit] "<<tag<<" FAIL: RANSAC found no constrained plane"<<std::endl;
        return false;
      }

      const double ratio = static_cast<double>(inliers->indices.size())
        / static_cast<double>(input->points.size());
      if (static_cast<int>(inliers->indices.size()) < plane_fit_min_inliers
          || ratio < plane_fit_min_inlier_ratio) {
        std::cout<<"[plane fit] "<<tag<<" FAIL: insufficient inliers="<<inliers->indices.size()
                 <<" ratio="<<ratio
                 <<" required="<<plane_fit_min_inliers<<"/"<<plane_fit_min_inlier_ratio
                 <<std::endl;
        return false;
      }

      Eigen::Vector3f normal(
        coefficients->values[0], coefficients->values[1], coefficients->values[2]);
      const float normal_length = normal.norm();
      if (normal_length < 1e-6f) {
        std::cout<<"[plane fit] "<<tag<<" FAIL: zero normal"<<std::endl;
        return false;
      }
      normal /= normal_length;
      float plane_d = coefficients->values[3] / normal_length;
      if (normal.dot(expected_normal) < 0.0f) {
        normal = -normal;
        plane_d = -plane_d;
      }

      const float normal_dot = std::max(-1.0f,
        std::min(1.0f, normal.dot(expected_normal)));
      const double angle_deg =
        std::acos(normal_dot) * 180.0 / 3.14159265358979323846;
      if (!std::isfinite(angle_deg) || angle_deg > max_angle_deg) {
        std::cout<<"[plane fit] "<<tag<<" FAIL: normal angle="<<angle_deg
                 <<"deg exceeds "<<max_angle_deg<<"deg"<<std::endl;
        return false;
      }

      double squared_distance_sum = 0.0;
      PointCloud::Ptr fitted_plane(new PointCloud);
      fitted_plane->reserve(inliers->indices.size());
      for (const int index : inliers->indices) {
        if (index < 0 || static_cast<size_t>(index) >= input->points.size()) {
          continue;
        }
        const auto &point = input->points[static_cast<size_t>(index)];
        const double distance = normal.x() * point.x + normal.y() * point.y
          + normal.z() * point.z + plane_d;
        squared_distance_sum += distance * distance;
        fitted_plane->push_back(point);
      }
      if (fitted_plane->empty()) {
        std::cout<<"[plane fit] "<<tag<<" FAIL: all inlier indices invalid"<<std::endl;
        return false;
      }
      const double rms = std::sqrt(
        squared_distance_sum / static_cast<double>(fitted_plane->size()));
      if (!std::isfinite(rms) || rms > plane_fit_max_rms) {
        std::cout<<"[plane fit] "<<tag<<" FAIL: RMS="<<rms
                 <<"m exceeds "<<plane_fit_max_rms<<"m"<<std::endl;
        return false;
      }

      normal_out = normal;
      d_out = plane_d;
      inlier_cloud_out = fitted_plane;
      rms_out = rms;
      ratio_out = ratio;
      angle_out = angle_deg;
      return true;
    }

    void getPlaneFrame() {
      if (!plane_fit_enabled) {
        std::cout<<"[plane fit] disabled, using legacy three-box frame"<<std::endl;
        getPlaneFrameThreePoint();
        return;
      }

      // 期望法向: 默认旧逻辑(竖直板=世界X+); 地面板等布置用 override 注入(如世界向下)。
      const Eigen::Vector3f expected_normal =
        (plane_expected_normal_override.norm() > 0.5f)
          ? plane_expected_normal_override.normalized()
          : expectedWorldX();
      if (expected_normal.norm() < 1e-6f) {
        std::cout<<"[plane fit] FAIL: invalid expected normal / world_up_in_base"<<std::endl;
        failed_ = true;
        return;
      }
      // 平面参考向(决定扫掠/换道方向): 默认 world_up_in_base; 法向竖直时它会退化,
      // 地面板必须用 override 注入水平参考(如世界Y)。
      const Eigen::Vector3f up_reference =
        (plane_up_reference_override.norm() > 0.5f)
          ? plane_up_reference_override
          : world_up_in_base;

      // 第一遍: 在固定裁剪框点云上拟合
      Eigen::Vector3f normal;
      float plane_d = 0.0f;
      PointCloud::Ptr fitted_plane(new PointCloud);
      double rms = 0.0, inlier_ratio = 0.0, normal_angle_deg = 0.0;
      if (!fitPlaneOnce(targetCloud_, expected_normal, plane_fit_max_normal_angle_deg,
                        normal, plane_d, fitted_plane,
                        rms, inlier_ratio, normal_angle_deg, "pass1")) {
        failed_ = true;
        return;
      }

      // 2026-08-02 鲁棒化: 第二遍在首遍内点上重拟合。首遍若被板边窄条/杂物带歪,
      // 离面点在重拟合时被剔掉; 重拟合不过质量门则回退首遍结果, 不会更差。
      if (plane_fit_refit_enabled && fitted_plane->size() >= 100) {
        Eigen::Vector3f normal2;
        float d2 = 0.0f;
        PointCloud::Ptr fitted2(new PointCloud);
        double rms2 = 0.0, ratio2 = 0.0, angle2 = 0.0;
        if (fitPlaneOnce(fitted_plane, expected_normal, plane_fit_max_normal_angle_deg,
                         normal2, d2, fitted2,
                         rms2, ratio2, angle2, "pass2")) {
          normal = normal2; plane_d = d2; fitted_plane = fitted2;
          rms = rms2; inlier_ratio = ratio2; normal_angle_deg = angle2;
          std::cout<<"[plane fit] pass2 refit accepted: inliers="<<fitted2->size()
                   <<" RMS="<<rms2<<"m"<<std::endl;
        } else {
          std::cout<<"[plane fit] pass2 refit rejected, keep pass1"<<std::endl;
        }
      }

      // 平面局部坐标轴(与下方建系同一套): x=扫掠向, y=换道/竖直向, z=法向
      Eigen::Vector3f plane_y = up_reference;
      plane_y.normalize();
      plane_y -= plane_y.dot(normal) * normal;
      if (plane_y.norm() < 1e-6f) {
        std::cout<<"[plane fit] FAIL: world-up projection degenerates"<<std::endl;
        failed_ = true;
        return;
      }
      plane_y.normalize();
      Eigen::Vector3f plane_x = plane_y.cross(normal);
      plane_x.normalize();
      plane_y = normal.cross(plane_x);
      plane_y.normalize();

      // 2026-08-02 内点覆盖面校验: 内点数量够但只是一窄条(板边/棱)时角度位置都会歪,
      // 此前只查数量不查形状直接放行。用 1%/99% 分位边界抗飞点, 不足判失败宁可重拍。
      {
        Eigen::Vector4f extent_centroid;
        pcl::compute3DCentroid(*fitted_plane, extent_centroid);
        std::vector<float> us, vs;
        us.reserve(fitted_plane->size());
        vs.reserve(fitted_plane->size());
        for (const auto &p : fitted_plane->points) {
          const Eigen::Vector3f d = p.getVector3fMap() - extent_centroid.head<3>();
          us.push_back(d.dot(plane_x));
          vs.push_back(d.dot(plane_y));
        }
        auto pct = [](std::vector<float>& v, float q) {
          size_t k = std::min(v.size() - 1, (size_t)(q * v.size()));
          std::nth_element(v.begin(), v.begin() + k, v.end());
          return v[k];
        };
        const float extent_x = pct(us, 0.99f) - pct(us, 0.01f);
        const float extent_y = pct(vs, 0.99f) - pct(vs, 0.01f);
        if (extent_x < plane_fit_min_extent_x || extent_y < plane_fit_min_extent_y) {
          std::cout<<"[plane fit] FAIL: inlier extent too small: x="<<extent_x
                   <<"m y="<<extent_y<<"m required="
                   <<plane_fit_min_extent_x<<"/"<<plane_fit_min_extent_y
                   <<"m (narrow strip, likely board edge/clutter)"<<std::endl;
          failed_ = true;
          return;
        }
        std::cout<<"[plane fit] inlier extent: x="<<extent_x<<"m y="<<extent_y<<"m"<<std::endl;
      }

      Eigen::Vector4f centroid4;
      pcl::compute3DCentroid(*fitted_plane, centroid4);
      // 沿用原 plane_point_o 的 x/y 作为轨迹局部原点锚点，再正交投影到拟合平面，
      // 这样现有 curve_box 的局部坐标范围不会因为改用内点质心而整体漂移。
      Eigen::Vector3f origin_guess(
        static_cast<float>(plane_ox), static_cast<float>(plane_oy), centroid4.z());
      Eigen::Vector3f origin = origin_guess
        - (normal.dot(origin_guess) + plane_d) * normal;

      // plane_x/plane_y/normal 已在覆盖面校验前按同一套约定算好
      Eigen::Matrix3f rotation;
      rotation.col(0) = plane_x;  // 单行扫掠方向，接近世界 Y
      rotation.col(1) = plane_y;  // 换道方向，接近世界竖直向上
      rotation.col(2) = normal;   // 工具 Z+/压入方向，指向工件内部
      planeFrame_.setIdentity();
      planeFrame_.topLeftCorner<3,3>() = rotation;
      planeFrame_.block<3,1>(0,3) = origin;

      pcl::io::savePCDFileBinary("plane_fit_inliers.pcd", *fitted_plane);
      Eigen::Vector3f rpy = rotation.eulerAngles(2, 1, 0);
      std::cout<<"[plane fit] OK: inliers="<<fitted_plane->size()
               <<"/"<<targetCloud_->size()<<" ratio="<<inlier_ratio
               <<" RMS="<<rms<<"m angle_to_world_x="<<normal_angle_deg<<"deg"<<std::endl;
      std::cout<<"[plane fit] normal_base=("<<normal.x()<<","<<normal.y()<<","<<normal.z()
               <<") origin_base=("<<origin.x()<<","<<origin.y()<<","<<origin.z()<<")"<<std::endl;
      std::cout<<"[plane fit] frame RPY=("<<rpy[2]<<","<<rpy[1]<<","<<rpy[0]<<")"<<std::endl;
    }

    void getPlaneFrameThreePoint() {
      // 三测点在工件平面上拉开大三角，盒 x/y ±5mm、z 贯通。
      // 点序约定: X测点必须在"离机器人更远"的位置，使 法向=(X-O)×(Y-O) 指向工件内部！
      //origin
      pcl::PointXYZ minO, maxO, midO;
      minO.x = plane_ox-0.005;
      minO.y = plane_oy-0.005;
      minO.z = plane_box_zmin;
      maxO.x = plane_ox+0.005;
      maxO.y = plane_oy+0.005;
      maxO.z = plane_box_zmax;
      midO = getBoxCloudCenter(targetCloud_, minO, maxO);
      std::cout<<"plane origin x "<<midO.x<<" y "<<midO.y<<" z "<<midO.z<<std::endl;
      pcl::PointXYZ minX, maxX, midX;
      minX.x = plane_xx-0.005;
      minX.y = plane_xy-0.005;
      minX.z = plane_box_zmin;
      maxX.x = plane_xx+0.005;
      maxX.y = plane_xy+0.005;
      maxX.z = plane_box_zmax;
      midX = getBoxCloudCenter(targetCloud_, minX, maxX);
      std::cout<<"plane x axis x "<<midX.x<<" y "<<midX.y<<" z "<<midX.z<<std::endl;
      pcl::PointXYZ minY, maxY, midY;
      minY.x = plane_yx-0.005;
      minY.y = plane_yy-0.005;
      minY.z = plane_box_zmin;
      maxY.x = plane_yx+0.005;
      maxY.y = plane_yy+0.005;
      maxY.z = plane_box_zmax;
      midY = getBoxCloudCenter(targetCloud_, minY, maxY);
      std::cout<<"plane y axis x "<<midY.x<<" y "<<midY.y<<" z "<<midY.z<<std::endl;

      //cal frame
      Eigen::Vector3d xVect;
      xVect[0] = midX.x-midO.x;
      xVect[1] = midX.y-midO.y;
      xVect[2] = midX.z-midO.z;
      Eigen::Vector3d yVect;
      yVect[0] = midY.x-midO.x;
      yVect[1] = midY.y-midO.y;
      yVect[2] = midY.z-midO.z;

      Eigen::Vector3d xAxis = xVect;
      Eigen::Vector3d refAxis = yVect;
      xAxis.normalize();
      refAxis.normalize();

      Eigen::Vector3d zAxis = xAxis.cross(refAxis);
      zAxis.normalize();
      Eigen::Vector3d yAxis = zAxis.cross(xAxis);
      yAxis.normalize();

      Eigen::Matrix3d rotation;

      rotation.col(0) = xAxis;
      rotation.col(1) = yAxis;
      rotation.col(2) = zAxis;

      Eigen::Vector3d rpy;
      rpy = rotation.eulerAngles(2, 1, 0);
      std::cout<<"plane RPY x "<<rpy[2]<<" y "<<rpy[1]<<" z "<<rpy[0]<<std::endl;
      double rx = rpy[2];
      double ry = rpy[1];
      double rz = rpy[0];

      planeFrame_.setIdentity();
      planeFrame_ = (Eigen::Translation3f(midO.x, midO.y, midO.z)
          * Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ())
          * Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX())
          ).matrix();
    }

    pcl::PointXYZ getBoxCloudCenter(PointCloud::Ptr input, pcl::PointXYZ min, pcl::PointXYZ max) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud = std::make_shared< pcl::PointCloud<pcl::PointXYZ> >();
      tmpCloud->clear();
      ///---------------直通滤波
      pcl::CropBox<pcl::PointXYZ> boxFilter;
      boxFilter.setMin(Eigen::Vector4f(min.x, min.y, min.z, 1.0));
      boxFilter.setMax(Eigen::Vector4f(max.x, max.y, max.z, 1.0));
      boxFilter.setInputCloud(input);
      boxFilter.filter(*tmpCloud);
      std::cout<<"tmp cloud size "<<tmpCloud->points.size()<<std::endl;

      pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud = std::make_shared< pcl::PointCloud<pcl::PointXYZ> >();
      resultCloud->clear();
      if (tmpCloud->points.empty()) {
        std::cout<<"[getBoxCloudCenter] FAIL: box empty"<<std::endl;
        failed_ = true;
        return pcl::PointXYZ(0, 0, 0);
      }
      ///---------------统计滤波
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> radFilter;
      radFilter.setInputCloud(tmpCloud);
      radFilter.setRadiusSearch(0.005);
      radFilter.setMinNeighborsInRadius(2);//
      radFilter.filter(*resultCloud);
      std::cout<<"reslut cloud size "<<resultCloud->points.size()<<std::endl;

      pcl::PointXYZ minD, maxD, midD(0, 0, 0);
      if (resultCloud->points.empty()) {
        std::cout<<"[getBoxCloudCenter] FAIL: cloud empty after outlier removal"<<std::endl;
        failed_ = true;
        return midD;
      }
      pcl::getMinMax3D(*resultCloud, minD, maxD);
      midD.x = (minD.x+maxD.x)/2;
      midD.y = (minD.y+maxD.y)/2;
      midD.z = (minD.z+maxD.z)/2;
      return midD;
    }

    void getPlaneCloud() {
      planeCloud_->clear();
      pcl::transformPointCloud (*targetCloud_, *planeCloud_, planeFrame_.inverse());
      if (planeCloud_->points.size() > 0)
        pcl::io::savePCDFileBinary ("plane_trans.pcd", *planeCloud_);
    }

    void getCurveCloud() {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud = std::make_shared< pcl::PointCloud<pcl::PointXYZ> >();
      tmpCloud->clear();
      ///---------------直通滤波
      // 圆弧盒(平面系下圈打磨特征区域)
      pcl::CropBox<pcl::PointXYZ> boxFilter;
      boxFilter.setMin(curve_box_min);
      boxFilter.setMax(curve_box_max);
      boxFilter.setInputCloud(planeCloud_);
      boxFilter.filter(*tmpCloud);
      std::cout<<"curve box cloud size "<<tmpCloud->points.size()<<std::endl;
      if (tmpCloud->points.empty()) {
        std::cout<<"[getCurveCloud] FAIL: curve box empty"<<std::endl;
        failed_ = true;
        return;
      }

      curveCloud_->clear();
      ///---------------统计滤波
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> radFilter;
      radFilter.setInputCloud(tmpCloud);
      radFilter.setRadiusSearch(0.002);
      radFilter.setMinNeighborsInRadius(2);//
      radFilter.filter(*curveCloud_);
      std::cout<<"curve raduis cloud size "<<curveCloud_->points.size()<<std::endl;
      if (curveCloud_->points.size() > 0)
        pcl::io::savePCDFileBinary ("curve_plane.pcd", *curveCloud_);
    }

    void getCurveFrame() {
      // 用 2%~98% 分位数边界代替裸 min/max：盒面边缘的飞点会拉扯包围盒
      // （实测挪 5cm 测出 4~7cm 抖动），分位数边界对少量离群点不敏感。
      std::vector<float> xs, ys, zs;
      xs.reserve(curveCloud_->points.size());
      ys.reserve(curveCloud_->points.size());
      zs.reserve(curveCloud_->points.size());
      for (const auto& p : curveCloud_->points) {
        xs.push_back(p.x); ys.push_back(p.y); zs.push_back(p.z);
      }
      auto pct = [](std::vector<float>& v, float q) {
        size_t k = std::min(v.size() - 1, (size_t)(q * v.size()));
        std::nth_element(v.begin(), v.begin() + k, v.end());
        return v[k];
      };
      pcl::PointXYZ min, max, mid;
      min.x = pct(xs, 0.02f);  max.x = pct(xs, 0.98f);
      min.y = pct(ys, 0.02f);  max.y = pct(ys, 0.98f);
      min.z = pct(zs, 0.02f);  max.z = pct(zs, 0.98f);
      mid.x = (min.x+max.x)/2;
      mid.y = (min.y+max.y)/2;
      mid.z = (min.z+max.z)/2;
      std::cout<<"curve frame bbox x ["<<min.x<<","<<max.x<<"] y ["<<min.y<<","<<max.y<<"] z ["<<min.z<<","<<max.z<<"]"<<std::endl;

      if (tool_align_world) {
        // 世界水平化参考系: 轨迹点 = p_new + x*WY + y*WZ + z_line*WX。
        // 工具尖端在 z_line 处贴 c_base(弧面中心), 法兰 = 表面 - tool_offset*WX。
        Eigen::Vector3f wup = world_up_in_base; wup.normalize();
        Eigen::Vector3f wy = Eigen::Vector3f(0,1,0) - (Eigen::Vector3f(0,1,0).dot(wup))*wup; wy.normalize();
        Eigen::Vector3f wx = wy.cross(wup);              // 世界 X+ 在 base 系 = 工具压入方向
        Eigen::Vector3f midO_b = planeFrame_.block<3,1>(0,3);
        Eigen::Vector3f c_base = planeFrame_.topLeftCorner<3,3>() * Eigen::Vector3f(mid.x, mid.y, mid.z) + midO_b;
        float z_line = curve_center_dz - curve_radius;  // 轨迹 z 线高度(曲线系)
        Eigen::Vector3f p_new = c_base - (z_line + curve_tool_offset) * wx;
        Eigen::Matrix3f R_plane = planeFrame_.topLeftCorner<3,3>();
        Eigen::Matrix3f R_world; R_world.col(0) = wy; R_world.col(1) = wup; R_world.col(2) = wx;
        curveFrame_.setIdentity();
        curveFrame_.topLeftCorner<3,3>() = R_plane.transpose() * R_world;
        curveFrame_.block<3,1>(0,3) = R_plane.transpose() * (p_new - midO_b);
        return;
      }

      Eigen::Vector3f trans;
      trans(0) = mid.x;
      trans(1) = max.y - curve_y_offset;
      // 打磨头伸出补偿: 平面系 z 轴 = 法向 = 指向工件内部 = 打磨头伸出方向（法兰 z 指向工件）。
      // 要让尖端贴表面，法兰目标必须 = 表面 - curve_tool_offset×法向，故为减。
      // 符号写反(+)会把法兰送进工件内部，切勿改错。
      trans(2) = min.z + curve_radius - curve_center_dz - curve_tool_offset;

      curveFrame_.setIdentity();
      curveFrame_.block<3,1>(0, 3) = trans;
    }

  private:
    PointCloud::Ptr baseCloud_;
    PointCloud::Ptr targetCloud_;
    PointCloud::Ptr planeCloud_;
    PointCloud::Ptr curveCloud_;
    Eigen::Matrix4f planeFrame_;
    Eigen::Matrix4f curveFrame_;
    Eigen::Matrix4f resultFrame_;
    bool failed_ = false;  // 某一层点云为空时置位，calc() 据此返回 false
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

#endif // ysPCLCalcTransform_HPP
