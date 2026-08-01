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

    void getPlaneFrame() {
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
