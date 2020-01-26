#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <fstream>
ros::Publisher pub;

tf2_ros::Buffer tf_buffer_;
geometry_msgs::TransformStamped transform;

ros::Publisher point_cloud_non_ground_plane_publisher;
std::ofstream outfile;
int _counter = 0;
// void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
// {
  
//   try
//   {
//     // tf::StampedTransform sensor_to_world_tf;
//     // transform = tf_buffer_.lookupTransform("world", msg->header.frame_id, msg->header.stamp, ros::Time(0));
//     transform = tf_buffer_.lookupTransform("world", msg->header.frame_id, msg->header.stamp, ros::Duration(20.0));
//     sensor_msgs::PointCloud2 cloud_out;
//     tf2::doTransform(*msg, cloud_out, transform);
//     pub.publish(cloud_out);
//   }
//   catch (tf2::TransformException& ex)
//   {
//     ROS_WARN("%s", ex.what());
//   }
// }

//http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
void onPointCloudOnPCL(const sensor_msgs::PointCloud2& cloud) {
  
  pcl::PointCloud<pcl::PointXYZ> pc;
  if(cloud.width<1){
    return;
  }
  pcl::fromROSMsg(cloud, pc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>)
      , cloud_f(new pcl::PointCloud<pcl::PointXYZ>), cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
  *in_cloud = pc;

  ros::Time lasttime=ros::Time::now();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(in_cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>());
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }
    // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);


  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  sensor_msgs::PointCloud2 point_cloud_non_ground_plane_cloud;

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    for(int i=0; i< cloud_cluster->points.size(); i++){
      cloud_out->push_back(cloud_cluster->points[i]);
    }
    j++;
  }

  ros::Time currtime = ros::Time::now();
  ros::Duration diff = currtime-lasttime;
  std::ofstream output;
  output.exceptions(std::ios_base::badbit | std::ios_base::failbit);
  std::string filename = "/home/geesara/catkin_ws/src/test/pcl_filter.csv";

  output.open(filename.c_str(), std::ios_base::app);
  output<< diff << ","<< _counter <<"\n" ; 
  output.close();

  pcl::toROSMsg(*(cloud_out).get(), point_cloud_non_ground_plane_cloud);
  point_cloud_non_ground_plane_cloud.header.frame_id="world";
  point_cloud_non_ground_plane_publisher.publish(point_cloud_non_ground_plane_cloud);
  return;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_convertor");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");
  tf2_ros::TransformListener tfListener(tf_buffer_);
  std::string cloud_in = "/cloud_in";
  nh.param("point_cloud_topic_name", cloud_in, cloud_in);
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (cloud_in, 1, onPointCloudOnPCL);
  // Create a ROS publisher for the output point cloud
  // pub = nh.advertise<sensor_msgs::PointCloud2> ("/apollo/sensor/velodyne16/point_cloud", 1);
  point_cloud_non_ground_plane_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/pcl/point_cloud_non_ground_plane", 1);
  // Spin
  ros::spin();
}