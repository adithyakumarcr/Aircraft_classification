#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>



int main (int argc, char** argv)
{
 
  int file_no=0;

 while (file_no<=40)
 {

  int j=0;
  std::stringstream directory, filename;
  directory<<"/home/adithya/Desktop/Flight_PCD/a321/";
  filename << directory.str()<<"a321_"<<file_no<<".pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>),  cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename.str (), *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file a321_0.pcd \n");
    return (-1);
  }

  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from a321_"<<file_no<<".pcd with the following fields"
            << std::endl;
	

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);


 if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;


 pcl::ExtractIndices<pcl::PointXYZ> extract;


  pcl::PCDWriter writer;

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      
    }

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud.swap (cloud_f);
	ss <<"segmented_a321_"<<file_no<<".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud, false);
    file_no++;
}

  return 0;

}

