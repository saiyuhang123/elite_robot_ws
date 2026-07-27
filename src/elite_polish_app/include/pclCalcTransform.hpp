#ifndef ysPCLCalcTransform_HPP
#define ysPCLCalcTransform_HPP

#include <Eigen/Core>
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

    void getTargetCloud() {
      std::cout<<"base cloud size "<<baseCloud_->points.size()<<std::endl;
      targetCloud_->clear();
      ///---------------直通滤波
      // 2026-07-27 标定 v2（差分法，模拟工件=竖直纸箱）: 工件范围 + 2cm 余量
      pcl::CropBox<pcl::PointXYZ> boxFilter;
      boxFilter.setMin(Eigen::Vector4f(0.520, -0.209, 0.498, 1.0));
      boxFilter.setMax(Eigen::Vector4f(0.720,  0.178, 0.740, 1.0));
      boxFilter.setInputCloud(baseCloud_);
      boxFilter.filter(*targetCloud_);
      std::cout<<"target cloud size "<<targetCloud_->points.size()<<std::endl;
    }

    void getPlaneFrame() {
      // 2026-07-27 标定: 工件(竖直纸箱)工作面为竖直面，三测点在面上拉开大三角，
      // 盒 x/y 方向 ±5mm、z 方向贯通(0.45~0.78)。
      // 注意点序约定: X测点必须选在"离机器人更远"的位置(0.690,-0.010)，
      // 使 法向=(X-O)×(Y-O) 指向盒内(远离机器人)——反了则下压方向指向机器人自身，IK 无解。
      //origin
      pcl::PointXYZ minO, maxO, midO;
      minO.x = 0.590-0.005;
      minO.y = -0.120-0.005;
      minO.z = 0.45;
      maxO.x = 0.590+0.005;
      maxO.y = -0.120+0.005;
      maxO.z = 0.75;
      midO = getBoxCloudCenter(targetCloud_, minO, maxO);
      std::cout<<"plane origin x "<<midO.x<<" y "<<midO.y<<" z "<<midO.z<<std::endl;
      pcl::PointXYZ minX, maxX, midX;
      minX.x = 0.670-0.005;
      minX.y = -0.010-0.005;
      minX.z = 0.45;
      maxX.x = 0.670+0.005;
      maxX.y = -0.010+0.005;
      maxX.z = 0.75;
      midX = getBoxCloudCenter(targetCloud_, minX, maxX);
      std::cout<<"plane x axis x "<<midX.x<<" y "<<midX.y<<" z "<<midX.z<<std::endl;
      pcl::PointXYZ minY, maxY, midY;
      minY.x = 0.590-0.005;
      minY.y = 0.100-0.005;
      minY.z = 0.45;
      maxY.x = 0.590+0.005;
      maxY.y = 0.100+0.005;
      maxY.z = 0.75;
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
      // 2026-07-27 标定 v2(模拟工件=竖直纸箱): 平面系下圈工件竖直面（模拟占位，真工件需重标）
      pcl::CropBox<pcl::PointXYZ> boxFilter;
      boxFilter.setMin(Eigen::Vector4f(-0.08, -0.14, -0.02, 1.0));
      boxFilter.setMax(Eigen::Vector4f( 0.28,  0.24,  0.02, 1.0));
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
      pcl::PointXYZ min, max, mid;
      pcl::getMinMax3D(*curveCloud_, min, max);
      mid.x = (min.x+max.x)/2;
      mid.y = (min.y+max.y)/2;
      mid.z = (min.z+max.z)/2;

      Eigen::Vector3f trans;
      trans(0) = mid.x;
      trans(1) = max.y-0.1;
      // 2026-07-27: -0.18 为 Elite 打磨头伸出补偿（方向推导）:
      // 平面系 z 轴 = 法向 = 指向工件内部 = 打磨头伸出方向（法兰 z 指向工件）。
      // ysrob 的 KDL 链末端在打磨头尖端，路径目标即"贴表面"；Elite 链末端是法兰 tool0，
      // 要让尖端贴表面，法兰目标必须 = 表面 - 0.18×法向（悬在表面外侧 18cm），故为 -0.18。
      // 符号写反(+0.18)会把法兰送进工件内部，切勿改错。
      trans(2) = min.z + 0.9306 - 1.89539 - 0.18;

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
