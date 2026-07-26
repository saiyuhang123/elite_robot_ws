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

    void
    calc (PointCloud::Ptr &base_cloud, Eigen::Matrix4f &result)
    {
      pcl::copyPointCloud(*base_cloud, *baseCloud_);
      getTargetCloud();
      getPlaneFrame();
      getPlaneCloud();
      getCurveCloud();
      getCurveFrame();
      resultFrame_.setIdentity();
      resultFrame_ = (planeFrame_ * curveFrame_);
      result = resultFrame_;
    }

    void getTargetCloud() {
      std::cout<<"base cloud size "<<baseCloud_->points.size()<<std::endl;
      targetCloud_->clear();
      ///---------------直通滤波
      pcl::CropBox<pcl::PointXYZ> boxFilter;
      boxFilter.setMin(Eigen::Vector4f(-0.6, -1.4, -0.3, 1.0));
      boxFilter.setMax(Eigen::Vector4f(0.4, -0.6, 0.4, 1.0));
      boxFilter.setInputCloud(baseCloud_);
      boxFilter.filter(*targetCloud_);
      std::cout<<"target cloud size "<<targetCloud_->points.size()<<std::endl;
    }

    void getPlaneFrame() {
      //origin
      pcl::PointXYZ minO, maxO, midO;
      minO.x = -0.115-0.005;
      minO.y = -1.4;
      minO.z = -0.18-0.005;
      maxO.x = -0.115+0.005;
      maxO.y = -0.6;
      maxO.z = -0.18+0.005;
      midO = getBoxCloudCenter(targetCloud_, minO, maxO);
      std::cout<<"plane origin x "<<midO.x<<" y "<<midO.y<<" z "<<midO.z<<std::endl;
      pcl::PointXYZ minX, maxX, midX;
      minX.x = -0.015-0.005;
      minX.y = -1.4;
      minX.z = -0.18-0.005;
      maxX.x = -0.015+0.005;
      maxX.y = -0.6;
      maxX.z = -0.18+0.005;
      midX = getBoxCloudCenter(targetCloud_, minX, maxX);
      std::cout<<"plane x axis x "<<midX.x<<" y "<<midX.y<<" z "<<midX.z<<std::endl;
      pcl::PointXYZ minY, maxY, midY;
      minY.x = -0.115-0.005;
      minY.y = -1.4;
      minY.z = -0.08-0.005;
      maxY.x = -0.115+0.005;
      maxY.y = -0.6;
      maxY.z = -0.08+0.005;
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
      ///---------------统计滤波
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> radFilter;
      radFilter.setInputCloud(tmpCloud);
      radFilter.setRadiusSearch(0.005);
      radFilter.setMinNeighborsInRadius(2);//
      radFilter.filter(*resultCloud);
      std::cout<<"reslut cloud size "<<resultCloud->points.size()<<std::endl;

      pcl::PointXYZ minD, maxD, midD;
      pcl::getMinMax3D(*resultCloud, minD, maxD);
      midD.x = (minD.x+maxD.x)/2;
      midD.y = (minD.y+maxD.y)/2;
      midD.z = (minD.z+maxD.z)/2;
      return midD;
    }

    void getPlaneCloud() {
      planeCloud_->clear();
      pcl::transformPointCloud (*targetCloud_, *planeCloud_, planeFrame_.inverse());
      pcl::io::savePCDFileBinary ("plane_trans.pcd", *planeCloud_);
    }

    void getCurveCloud() {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud = std::make_shared< pcl::PointCloud<pcl::PointXYZ> >();
      tmpCloud->clear();
      ///---------------直通滤波
      pcl::CropBox<pcl::PointXYZ> boxFilter;
      boxFilter.setMin(Eigen::Vector4f(-0.25, 0.24, -0.095, 1.0));
      boxFilter.setMax(Eigen::Vector4f(0.25, 0.54, -0.045, 1.0));
      boxFilter.setInputCloud(planeCloud_);
      boxFilter.filter(*tmpCloud);
      std::cout<<"curve box cloud size "<<tmpCloud->points.size()<<std::endl;

      curveCloud_->clear();
      ///---------------统计滤波
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> radFilter;
      radFilter.setInputCloud(tmpCloud);
      radFilter.setRadiusSearch(0.002);
      radFilter.setMinNeighborsInRadius(2);//
      radFilter.filter(*curveCloud_);
      std::cout<<"curve raduis cloud size "<<curveCloud_->points.size()<<std::endl;
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
      trans(2) = min.z + 0.9306 - 1.89539;

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
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

#endif // ysPCLCalcTransform_HPP
