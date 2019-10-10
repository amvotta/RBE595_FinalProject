#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
// #include <pcl/common/colors.h>
#include <pcl/visualization/pcl_visualizer.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("tableTop.pcd", *original_cloud) == -1)
    {
      std::cout << "Cloud reading failed." << std::endl;
      return (-1);
    }


// ************************** REMOVE TABLE *********************************
  pcl::PointCloud<pcl::PointXYZ>::Ptr noNaN_cloud(new pcl::PointCloud<pcl::PointXYZ>); // define new point cloud to remove nan from original
  std::vector< int > index; //no clue
  pcl::removeNaNFromPointCloud(*original_cloud, *noNaN_cloud, index); //remove nan from original point cloud


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); //coefficeints for equation defining table plane
  pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices); //define new PointIndices object
  pcl::ExtractIndices<pcl::PointXYZ> extract_table; //define new ExtractIndices object (to remove table from point cloud)

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Finds the table plane and the coefficients defining it
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (.01);
  seg.setInputCloud (noNaN_cloud);
  seg.segment (*table_inliers, *coefficients); //indices of original point cloud that contain table, coefficients of equation defining point cloud

  if (table_inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

pcl::PointCloud<pcl::PointXYZ>::Ptr noTable_cloud(new pcl::PointCloud<pcl::PointXYZ>(*noNaN_cloud)); //copy full PC, called noTable_cloud

//extract indices of table_inliers from point cloud, to remove table
extract_table.setInputCloud(noTable_cloud);
extract_table.setIndices(table_inliers);
extract_table.setNegative(true);
extract_table.filter(*noTable_cloud);



// *************************** Make each object a different point cloud ********************************
// from PCL Euclidean Cluster Extraction tutorial


std::cout << "PointCloud before filtering has: " << noTable_cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of .5cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (noTable_cloud);
  vg.setLeafSize (0.005f, 0.005f, 0.005f); // change leaf size here
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  pcl::PCDWriter writer;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

// this gets the indices of clusters (objects) in cloud_filtered
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  // creates a vector of point cloud objects
  std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds;
  int j = 0;

  // assigns each object to one of the point clouds in sourceClouds
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {sourceCloud->points.push_back (cloud_filtered->points[*pit]);} //*

    sourceCloud->width = sourceCloud->points.size ();
    sourceCloud->height = 1;
    sourceCloud->is_dense = true;

    sourceClouds.push_back(sourceCloud);

    std::cout << "PointCloud representing the Cluster: " << sourceCloud->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *sourceCloud, false); //*
    j++;
  }

// std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds_rbg;
// for (int i=0; sourceClouds.size(); i++) {
//   std::cout<<"test"<<i<<std::endl;
// }






// // ************************** SEGMENT OBJECTS *********************************
// // from PCL region growing segmentation tutorial
// // Commented for now while I figure out how to separate objects first
//
//   pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//   pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
//   normal_estimator.setSearchMethod (tree);
//   normal_estimator.setInputCloud (noTable_cloud);
//   normal_estimator.setKSearch (50);
//   normal_estimator.compute (*normals);
//
//   pcl::IndicesPtr indices (new std::vector <int>);
//   pcl::PassThrough<pcl::PointXYZ> pass;
//   pass.setInputCloud (noTable_cloud);
//   pass.setFilterFieldName ("z");
//   pass.setFilterLimits (0.0, 1.0);
//   pass.filter (*indices);
//
//   pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
//   reg.setMinClusterSize (5); //50
//   reg.setMaxClusterSize (1000000);
//   reg.setSearchMethod (tree);
//   reg.setNumberOfNeighbours (30);
//   reg.setInputCloud (noTable_cloud);
//   //reg.setIndices (indices);
//   reg.setInputNormals (normals);
//   reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI); // 3.0
//   reg.setCurvatureThreshold (1.0); // 1.0
//
//   std::vector <pcl::PointIndices> clusters;
//   reg.extract (clusters);
//
//   std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
//   std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
//   std::cout << "These are the indices of the points of the initial" <<
//     std::endl << "cloud that belong to the first cluster:" << std::endl;
//   int counter = 0;
  // while (counter < clusters[0].indices.size ())
  // {
  //   std::cout << clusters[0].indices[counter] << ", ";
  //   counter++;
  //   if (counter % 10 == 0)
  //     std::cout << std::endl;
  // }
  // std::cout << std::endl;




  // ******************* PCL Visualizer *******************

  // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

  // PCL visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer.reset(new pcl::visualization::PCLVisualizer);

  // for loop to display all objects (ideally in a different color)
  // issue is that each PC needs a diffierent name ("cloud1, cloud2, etc") and I'm not sure how to do that in a for loop...
  for (int i=0; i<=sourceClouds.size(); i++) {
    // int r_i=
    // std::string
  pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> blue(sourceClouds[0],0,0,255); //set color of point cloud
  viewer->addPointCloud<pcl::PointXYZ>(sourceClouds[0],blue,"cloud1");//display point cloud in viewer
}
  // viewer.addPointCloud(sourceClouds[5],"cloud2");
  viewer->spin();


return (0);

}
