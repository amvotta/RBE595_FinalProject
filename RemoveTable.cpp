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
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/lexical_cast.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <Eigen/StdVector>
#include <pcl/features/boundary.h>
#include <pcl/impl/point_types.hpp>
#include <math.h>

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
  vg.setLeafSize (0.002f, 0.002f, 0.002f); // change leaf size here
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




  // ******************* PCL Visualizer *******************

  // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  std::vector < pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGB>::Ptr > > coloredClouds;
    // std::vector < pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGB>::Ptr > > cloudSegmentClusters;

  // // PCL visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer.reset(new pcl::visualization::PCLVisualizer);
  viewer->addCoordinateSystem (1.0);
  // for loop to display all objects (ideally in a different color)
  // issue is that each PC needs a diffierent name ("cloud1, cloud2, etc") and I'm not sure how to do that in a for loop...

  // pcl::visualization::CloudViewer viewer ("Cluster viewer");



  std::cout << "number of PCs " << sourceClouds.size() << std::endl;

  std::vector < pcl::PointCloud<pcl::PointXYZ> > object_top_points;
  std::vector < Eigen::Matrix< float, 4, 1 > > object_centroids;
  // vector<Eigen::Matrix<float,4,1> > object_centroids;

  for (int i=0; i<sourceClouds.size(); i++) {

    std::string cloud0("cloud");
    std::string PCnumber = boost::lexical_cast<std::string>(i);
    std::string cloud;
    cloud=cloud0+PCnumber;




    // ************************** SEGMENT OBJECTS *********************************
    // from PCL region growing segmentation tutorial
    // Commented for now while I figure out how to separate objects first

    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (sourceClouds[i]);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (sourceClouds[i]);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50); //50
    reg.setMaxClusterSize (10000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (sourceClouds[i]);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI); // 3.0
    reg.setCurvatureThreshold (1.0); // 1.0

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    // int counter = 0;

    double current_high_z=0.0;
    bool initialized=false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_top_surf (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix< float, 4, 1 > top_surf_centroid;

    for (j=0;j<clusters.size();j++) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>(*sourceClouds[i]));
      pcl::ExtractIndices<pcl::PointXYZ> extractCluster;

      std::string cloud1("cloud");
      std::string objectNum = boost::lexical_cast<std::string>(i);
      std::string clusterNum = boost::lexical_cast<std::string>(j);
      std::string cloud2;
      cloud2=objectNum+cloud1+clusterNum;

      pcl::PointIndices::Ptr current_indices(new  pcl::PointIndices(clusters[j]));

      // Eigen::Matrix< float, 4, 1 > centroid;
      // pcl::compute3DCentroid(cluster_cloud,current_indices,centroid);
      // std::double current_z=centroid[2];


      extractCluster.setInputCloud(cluster_cloud);
      extractCluster.setIndices(current_indices);
      extractCluster.setNegative(false);
      extractCluster.filter(*cluster_cloud);

      Eigen::Matrix< float, 4, 1 > centroid;
      pcl::compute3DCentroid(*cluster_cloud,centroid);
      double current_z=centroid[1];
      // std::cout  << "object" << i << " , " << centroid << " , " << current_high_z << std::endl;

      if (!initialized||current_z<current_high_z) {
        current_high_z=current_z;
        current_top_surf=cluster_cloud;
        top_surf_centroid=centroid;
        initialized=true;

      }


      int rand1 = rand() % 255;
      int rand2 = rand() % 255;
      int rand3 = rand() % 255;


      pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> randColor(cluster_cloud,0,255,0);//rand1,rand2,rand3); //set color of point cloud
      viewer->addPointCloud<pcl::PointXYZ>(cluster_cloud, randColor, cloud2);


    }

    object_top_points.push_back(*current_top_surf);
    object_centroids.push_back(top_surf_centroid);
    pcl::PointCloud<pcl::PointXYZ>::Ptr top_point(new pcl::PointCloud<pcl::PointXYZ>(object_top_points[i]));
    pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> randColor(top_point,255,0,0);
    viewer->addPointCloud<pcl::PointXYZ>(top_point,randColor,cloud);
    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr coloredCloud = reg.getColoredCloud ();
      // pcl::visualization::CloudViewer viewer ("Cluster viewer");
      // viewer.showCloud(colored_cloud);

    // coloredClouds.push_back(coloredCloud);


    // viewer->addPointCloud<pcl::PointXYZRGB>(coloredCloud,cloud);
  // pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> blue(sourceClouds[i],0,0,255); //set color of point cloud
  // viewer->addPointCloud<pcl::PointXYZ>(sourceClouds[i],blue,cloud);//display point cloud in viewer
std::cout << object_centroids[i] << std::endl;

}


pcl::PointCloud<pcl::PointXYZ>::Ptr centroids_PC (new pcl::PointCloud<pcl::PointXYZ>);
centroids_PC->width  = object_top_points.size();
centroids_PC->height = 1;
centroids_PC->points.resize (centroids_PC->width * centroids_PC->height);

pcl::PointCloud<pcl::PointXYZ>::Ptr closest_point (new pcl::PointCloud<pcl::PointXYZ>);
closest_point->width  = object_top_points.size();
closest_point->height = 1;
closest_point->points.resize (closest_point->width * closest_point->height);



for (int n=0;n<object_top_points.size();n++) {


  // ********************************** Find Boundaries **********************************

  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
  pcl::PointCloud<pcl::Normal>::Ptr boundary_normals(new pcl::PointCloud<pcl::Normal>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>(object_top_points[n]));

  normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(boundary_cloud));
  normEst.setRadiusSearch(20);
  normEst.compute(*boundary_normals);

  boundEst.setInputCloud(boundary_cloud);
  boundEst.setInputNormals(boundary_normals);
  boundEst.setRadiusSearch(10);
  boundEst.setAngleThreshold(M_PI/1.5);
  boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
  boundEst.compute(boundaries);

  for(int i = 0; i < boundary_cloud->points.size(); i++)
  {
         if(boundaries[i].boundary_point < 1)
         {
                 boundary_cloud->at(i).z = 0;
         }
  }

  std::string cloud0("boundary_cloud");
  std::string objNumber = boost::lexical_cast<std::string>(n);
  std::string cloud;
  cloud=cloud0+objNumber;

  pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> randColor(boundary_cloud,255,255,255);
  viewer->addPointCloud<pcl::PointXYZ>(boundary_cloud,cloud);

  // *************** Create PC with object centroids ********************
  centroids_PC->points[n].x =object_centroids[n][0];
  centroids_PC->points[n].y =object_centroids[n][1];
  centroids_PC->points[n].z =object_centroids[n][2];

// ********************************** Find closest point to centroid **********************************

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (boundary_cloud);

  pcl::PointXYZ searchPoint;
  searchPoint.x =object_centroids[n][0];
  searchPoint.y =object_centroids[n][1];
  searchPoint.z =object_centroids[n][2];

  int K = 1;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  // pcl::PointIndices::Ptr closest_point_index (new pcl::PointIndices);

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      for (size_t p = 0; p < pointIdxNKNSearch.size (); ++p) {

        closest_point->points[n].x =  boundary_cloud->points[pointIdxNKNSearch[p]].x;
        closest_point->points[n].y = boundary_cloud->points[pointIdxNKNSearch[p]].y;
        closest_point->points[n].z = boundary_cloud->points[pointIdxNKNSearch[p]].z;
        // closest_point_index=pointIdxNKNSearch[p];

        // std::cout  << "    "  <<   boundary_cloud->points[pointIdxNKNSearch[p]].x
        //         << " " << boundary_cloud->points[pointIdxNKNSearch[p]].y
        //         << " " << boundary_cloud->points[pointIdxNKNSearch[p]].z
        //         << " (squared distance: " << pointNKNSquaredDistance[p] << ")" << std::endl;

      }
    }

  // std::vector<int> pointIdxRadiusSearch;
  // std::vector<float> pointRadiusSquaredDistance;
  //
  // float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

  std::string cloud1("line");
  std::string lineNum = boost::lexical_cast<std::string>(n);
  std::string line;
  line=cloud1+lineNum;

  pcl::ModelCoefficients line_coeff;
  line_coeff.values.resize (6);    // We need 6 values
  line_coeff.values[0] = closest_point->points[n].x;
  line_coeff.values[1] = closest_point->points[n].y;
  line_coeff.values[2] = closest_point->points[n].z;
  line_coeff.values[3] = centroids_PC->points[n].x-closest_point->points[n].x;
  line_coeff.values[4] = centroids_PC->points[n].y-closest_point->points[n].y;
  line_coeff.values[5] = centroids_PC->points[n].z-closest_point->points[n].z;

  viewer->addLine (line_coeff,line);



}

pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> white(centroids_PC,255,255,255);
viewer->addPointCloud<pcl::PointXYZ>(centroids_PC,white,"centroids");
pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> green(closest_point,0,255,255);
viewer->addPointCloud<pcl::PointXYZ>(closest_point,green,"closest_point");

viewer->spin();


return (0);

}
