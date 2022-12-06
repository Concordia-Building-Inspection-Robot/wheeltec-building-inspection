//
// Created by william on 05/12/22.
//

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

ros::Publisher pub

laser_geometry::LaserProjection projector;
tf::TransformListener listener;

void fusePointClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
                     sensor_msgs::PointCloud2& fused_cloud_msg) {
  // Downsample the pointclouds using a voxel grid filter
  pcl::VoxelGrid <pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(0.01, 0.01, 0.01);
  voxel_grid.setInputCloud(cloud1);
  voxel_grid.filter(*cloud1);
  voxel_grid.setInputCloud(cloud2);
  voxel_grid.filter(*cloud2);

  // Concatenate the two pointclouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr fused_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  *fused_cloud = *cloud1 + *cloud2;

  // Convert the fused PointCloud to a PointCloud2 message
  pcl::toROSMsg(*fused_cloud, fused_cloud_msg);
}

// Callback function that is called whenever new messages are received on the two topics
void callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
              const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
  // Create PointCloud messages type
  pcl::PointCloud<pcl::PointXYZ>::Ptr depthCameraCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2 lidarCloudRosMsg;

  // Convert the PointCloud2 message to a PointCloud message
  pcl::fromROSMsg(*cloud_msg, *depthCameraCloud);

  if(!listener.waitForTransform(scan_msg->header.frame_id,
                                 "/base_link",
                                 scan_msg->header.stamp + ros::Duration().fromSec(scan_msg->ranges.size()*scan_msg->time_increment),
                                 ros::Duration(1.0))) {
    return;
  }

  projector.transformLaserScanToPointCloud("/base_link", *scan_msg, lidarCloudRosMsg, listener);

  pcl::fromROSMsg(lidarCloudRosMsg, *lidarCloud);

  sensor_msgs::PointCloud2 fusedPointClouds;

  fusePointClouds(depthCameraCloud, lidarCloud, fusedPointClouds);

  pub.publish(fusedPointClouds);
}

int main(int argc, char** argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "data_collection");
  ros::NodeHandle nh;

  // Create a publisher for the PointCloud2 message
  pub = &nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

  // Create subscribers for the two topics
  message_filters::Subscriber<sensor_msgs::LaserScan> depthCameraSub(nh, "topic1", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidarSub(nh, "topic2", 1);

  // Create a synchronizer to combine the messages from the two topics
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::PointCloud2> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), depthCameraSub, lidarSub);

  // Register the callback function to be called whenever new messages are received on the two topics
  sync.registerCallback(callback);

  // Spin to process incoming messages
  ros::spin();

  return 0;
}
