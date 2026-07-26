#include "pclTemplateAlign.hpp"
#include "pclCalcTransform.hpp"


// int main (int argc, char **argv)
// {
// {  // Load the target cloud PCD file
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::io::loadPCDFile ("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/cameracapture0.pcd", *cloud);
//   printf ("load polish_x size %d\n", cloud->size());

//   // ... and downsampling the point cloud
//   const float voxel_grid_size = 0.002f;
//   pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
//   vox_grid.setInputCloud (cloud);
//   vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
//   //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
//   pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
//   vox_grid.filter (*tempCloud);
//   cloud = tempCloud; 
//   printf ("VoxelGrid 0.002 polish_x size %d\n", cloud->size());

//   pcl::io::savePCDFileBinary ("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/vox_cameracapture0.2.pcd", *cloud);
// }
// {  // Load the target cloud PCD file
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::io::loadPCDFile ("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/cameracapture0.pcd", *cloud);
//   printf ("load polish_x size %d\n", cloud->size());

//   // ... and downsampling the point cloud
//   const float voxel_grid_size = 0.005f;
//   pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
//   vox_grid.setInputCloud (cloud);
//   vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
//   //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
//   pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
//   vox_grid.filter (*tempCloud);
//   cloud = tempCloud; 
//   printf ("VoxelGrid 0.005 polish_x size %d\n", cloud->size());

//   pcl::io::savePCDFileBinary ("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/vox_cameracapture0.5.pcd", *cloud);
// }
// {  // Load the target cloud PCD file
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::io::loadPCDFile ("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/polish_feature0.pcd", *cloud);
//   printf ("load polish_x size %d\n", cloud->size());

//   // ... and downsampling the point cloud
//   const float voxel_grid_size = 0.002f;
//   pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
//   vox_grid.setInputCloud (cloud);
//   vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
//   //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
//   pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
//   vox_grid.filter (*tempCloud);
//   cloud = tempCloud; 
//   printf ("VoxelGrid 0.002 polish_x size %d\n", cloud->size());

//   pcl::io::savePCDFileBinary ("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/polish_feature_template0.2.pcd", *cloud);
// }
// {  // Load the target cloud PCD file
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::io::loadPCDFile ("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/polish_feature0.pcd", *cloud);
//   printf ("load polish_x size %d\n", cloud->size());

//   // ... and downsampling the point cloud
//   const float voxel_grid_size = 0.005f;
//   pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
//   vox_grid.setInputCloud (cloud);
//   vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
//   //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
//   pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
//   vox_grid.filter (*tempCloud);
//   cloud = tempCloud; 
//   printf ("VoxelGrid 0.005 polish_x size %d\n", cloud->size());

//   pcl::io::savePCDFileBinary ("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/polish_feature_template0.5.pcd", *cloud);
// }
//   return (0);
// }

// Align a collection of object templates to a sample point cloud
int main (int argc, char **argv)
{
  {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr temcloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile ("template.pcd", *temcloud);
    // printf ("load template size %d\n", temcloud->size());
    // //downsampling the point cloud
    // const float voxel_grid_size = 0.005f;
    // pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    // vox_grid.setInputCloud (temcloud);
    // vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>); 
    // vox_grid.filter (*tmpCloud);
    // printf ("VoxelGrid 0.005 template size %d\n", tmpCloud->size());
    // // Assign to the template FeatureCloud
    // FeatureCloud template_cloud;
    // template_cloud.setInputCloud (tmpCloud);
  
    // Load the target cloud PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr tarcloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("target.pcd", *tarcloud);
    printf ("load cameracapture size %d\n", tarcloud->size());
    // // downsampling the point cloud
    // vox_grid.setInputCloud (tarcloud);
    // vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr voxcloud (new pcl::PointCloud<pcl::PointXYZ>); 
    // vox_grid.filter (*voxcloud);
    // printf ("VoxelGrid 0.005 cameracapture size %d\n", voxcloud->size());
    // // Assign to the target FeatureCloud
    // FeatureCloud target_cloud;
    // target_cloud.setInputCloud (voxcloud);
  
    // // Set the TemplateAlignment inputs
    // TemplateAlignment template_align;
    // // template_align.addTemplateCloud (template_cloud);
    // template_align.setTargetCloud (target_cloud);
  
    // // Find the best template alignment
    // TemplateAlignment::Result best_alignment;
    // template_align.align(template_cloud, best_alignment);
  
    // // Print the alignment fitness score (values less than 0.00002 are good)
    // printf ("Best fitness score: %f\n", best_alignment.fitness_score);
  
    // // Print the rotation matrix and translation vector
    // Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
    // Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);
  
    // printf ("\n");
    // printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    // printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    // printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    // printf ("\n");
    // printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  
    // // Save the aligned template for visualization
    // pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    // pcl::transformPointCloud (*template_cloud.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
    // pcl::io::savePCDFileBinary ("result.pcd", transformed_cloud);
    // printf ("save  result.pcd\n");
    
    YsPCLCalcTransform calcFrame;
    Eigen::Matrix4f result;
    calcFrame.calc(tarcloud, result);
    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotationC = result.block<3,3>(0, 0);
    Eigen::Vector3f translationC = result.block<3,1>(0, 3);
  
    printf ("\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", rotationC (0,0), rotationC (0,1), rotationC (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", rotationC (1,0), rotationC (1,1), rotationC (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", rotationC (2,0), rotationC (2,1), rotationC (2,2));
    printf ("\n");
    printf ("t = < %0.3f, %0.3f, %0.3f >\n", translationC (0), translationC (1), translationC (2));
  }
// {
//   // Load the object templates specified in the object_templates.txt file
//   std::vector<FeatureCloud> object_templates;
//   // std::ifstream input_stream (argv[1]);
//   object_templates.resize (0);
//   std::string pcd_filename="/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/polish_feature_template0.2.pcd";
//   // while (input_stream.good ())
//   // {
//   //   std::getline (input_stream, pcd_filename);
//   //   if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
//   //     continue;

//     FeatureCloud template_cloud;
//     printf ("define template_cloud\n");
//     template_cloud.loadInputCloud (pcd_filename);
//     printf ("load polishcurve_template size %d\n", template_cloud.getPointCloud()->size());
//     object_templates.push_back (template_cloud);
//     // }
//   // input_stream.close ();

//   std::vector<std::string> patharr, savearr;
//   patharr.push_back("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/cameracapture0.pcd");
//   savearr.push_back("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/polish_output0.2.pcd");
//   patharr.push_back("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/cameracapture.pcd");
//   savearr.push_back("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/polish_output1.2.pcd");
//   patharr.push_back("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/cameracapture2.pcd");
//   savearr.push_back("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/polish_output2.2.pcd");
//   patharr.push_back("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/cameracapture3.pcd");
//   savearr.push_back("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/polish_output3.2.pcd");
//   patharr.push_back("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/cameracapture4.pcd");
//   savearr.push_back("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/polish_output4.2.pcd");

//   for (size_t i = 0; i < patharr.size(); i++)
//   {
//     printf ("2mm load cameracapture  %d\n", i);
//     printf ("load cameracapture  %s\n", patharr.at(i).c_str());

//   // Load the target cloud PCD file
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::io::loadPCDFile (patharr.at(i), *cloud);
//   printf ("load cameracapture size %d\n", cloud->size());

//   // for (size_t i = 0; i < cloud->size(); i++)
//   // {
//   //   cloud->points[i].x *=1000;
//   //   cloud->points[i].y *=1000;
//   //   cloud->points[i].z *=1000;
//   // }
//   // pcl::io::savePCDFileBinary ("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/cameracapture3.pcd", *cloud);

//   // Preprocess the cloud by...
//   // ...removing distant points
//   const float depth_limit = 1.0;
//   pcl::PassThrough<pcl::PointXYZ> zpass;
//   zpass.setInputCloud (cloud);
//   zpass.setFilterFieldName ("z");
//   zpass.setFilterLimits (0, depth_limit);
//   zpass.filter (*cloud);
//   printf ("pass z 1.0 cameracapture size %d\n", cloud->size());
//   pcl::PassThrough<pcl::PointXYZ> xpass;
//   xpass.setInputCloud (cloud);
//   xpass.setFilterFieldName ("x");
//   xpass.setFilterLimits (-0.7, 0.2);
//   xpass.filter (*cloud);
//   printf ("pass x 0.2 cameracapture size %d\n", cloud->size());
//   // ... and downsampling the point cloud
//   const float voxel_grid_size = 0.002f;
//   pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
//   vox_grid.setInputCloud (cloud);
//   vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
//   //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
//   pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
//   vox_grid.filter (*tempCloud);
//   cloud = tempCloud; 
//   printf ("VoxelGrid 0.002 cameracapture size %d\n", cloud->size());
//   // pcl::io::savePCDFileBinary ("/home/long/my_ws/src/ys_app_polish/ys_ur5_polish_robot/etc/cameracapture_vox.pcd", *cloud);

//   // Assign to the target FeatureCloud
//   FeatureCloud target_cloud;
//   target_cloud.setInputCloud (cloud);

//   // Set the TemplateAlignment inputs
//   TemplateAlignment template_align;
//   for (std::size_t i = 0; i < object_templates.size (); ++i)
//   {
//     template_align.addTemplateCloud (object_templates[i]);
//   }
//   template_align.setTargetCloud (target_cloud);

//   // Find the best template alignment
//   TemplateAlignment::Result best_alignment;
//   int best_index = template_align.findBestAlignment (best_alignment);
//   const FeatureCloud &best_template = object_templates[best_index];

//   // Print the alignment fitness score (values less than 0.00002 are good)
//   printf ("Best fitness score: %f\n", best_alignment.fitness_score);

//   // Print the rotation matrix and translation vector
//   Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
//   Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

//   printf ("\n");
//   printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
//   printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
//   printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
//   printf ("\n");
//   printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

//   // Save the aligned template for visualization
//   pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
//   pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
//   pcl::io::savePCDFileBinary (savearr.at(i), transformed_cloud);
//   printf ("save  %s\n", savearr.at(i).c_str());
// }
// }
  return (0);
}
