#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include <sound_play/sound_play.h>
#include "exercise6/Message.h"
#include <stdio.h>
#include <algorithm>
using namespace std;
#include <vector>
#include "math.h"
#include <string>


typedef unsigned TAlphaColor;

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pubm;
ros::Publisher chatter_pub;

tf2_ros::Buffer tf2_buffer;

typedef pcl::PointXYZRGB PointT;

float markers[4][2];
int m_id = 0;
// sound_play::SoundClient sc;


double returnHSVvalues(vector<double>&rgb){
                                           
  double hue, sat;
  double maxval, minval;
  
  maxval=*max_element(rgb.begin(), rgb.end());
  
  minval=*min_element(rgb.begin(), rgb.end());
  
  double difference=maxval-minval;
  
  double red, green, blue;
  red=rgb.at(0);
  green=rgb.at(1);
  blue=rgb.at(2);
      
  if(difference==0)
  hue=0;
  else if(red==maxval)
  hue= fmod(((60*((green-blue)/difference))+360), 360.0);
  else if(green=maxval)
  hue= fmod(((60*((blue-red)/difference))+120), 360.0);
  else if(blue=maxval)
  hue= fmod(((60*((red-green)/difference))+240),360.0);
  
  return hue;
}


void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  // All the objects needed

  sound_play::SoundClient sc;
  ros::Time time_rec, time_test;
  time_rec = ros::Time::now();
  
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  Eigen::Vector4f centroid;

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
  
  // Read in the cloud data
  pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  
  /*pcl::PCLPointCloud2 outcloud_plane;
  pcl::toPCLPointCloud2 (*cloud_plane, outcloud_plane);
  pubx.publish (outcloud_plane);*/

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.025);
  seg.setRadiusLimits (0.08, 0.15);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);

  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
          
    pcl::compute3DCentroid (*cloud_cylinder, centroid);
    std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << std::endl;

	  //Create a point in the "camera_rgb_optical_frame"
    geometry_msgs::PointStamped point_camera;
    geometry_msgs::PointStamped point_map;
    visualization_msgs::Marker marker;
    geometry_msgs::TransformStamped tss;
          
    point_camera.header.frame_id = "camera_rgb_optical_frame";
    point_camera.header.stamp = ros::Time::now();

    point_map.header.frame_id = "map";
    point_map.header.stamp = ros::Time::now();

    point_camera.point.x = centroid[0];
    point_camera.point.y = centroid[1];
    point_camera.point.z = centroid[2];

	  try{
		  time_test = ros::Time::now();

		  std::cerr << time_rec << std::endl;
		  std::cerr << time_test << std::endl;
      tss = tf2_buffer.lookupTransform("map","camera_rgb_optical_frame", time_rec);
      //tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
	  }
      catch (tf2::TransformException &ex)
	  {
      ROS_WARN("Transform warning: %s\n", ex.what());
	  }
    
    /*pcl::PCLPointCloud2 outcloud_cylinder;
    pcl::toPCLPointCloud2 (*cloud_cylinder, outcloud_cylinder);
    puby.publish (outcloud_cylinder);
    ROS_WARN("Width: %d\n", outcloud_cylinder.width);
    ROS_WARN("Height: %d\n", outcloud_cylinder.height);*/
    
    pcl::PointCloud<pcl::PointXYZ> output;
    pcl::copyPointCloud(*cloud_cylinder, output);
    geometry_msgs::PointStamped min_map;
    geometry_msgs::PointStamped min_camera;
    geometry_msgs::PointStamped max_map;
    geometry_msgs::PointStamped max_camera;
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(output, min, max);
    min_camera.point.x = min.x;
    min_camera.point.y = min.y;
    min_camera.point.z = min.z;
    max_camera.point.x = max.x;
    max_camera.point.y = max.y;
    max_camera.point.z = max.z;
    tf2::doTransform(min_camera, min_map, tss);
    tf2::doTransform(max_camera, max_map, tss);
    std::cerr << "Min: " << min_map.point.x << " " <<  min_map.point.y << " " <<  min_map.point.z << std::endl;
    std::cerr << "Max: " << max_map.point.x << " " <<  max_map.point.y << " " <<  max_map.point.z << std::endl;

    float h = min_map.point.z - max_map.point.z;
    float w = sqrt(pow(min_map.point.x - max_map.point.x, 2) + pow(min_map.point.y - max_map.point.y, 2));
    ROS_WARN("Visina: %f\n", h);
    ROS_WARN("Sirina: %f\n", w);

    //std::cerr << tss ;

    tf2::doTransform(point_camera, point_map, tss);

    /*std::cerr << "point_camera: " << point_camera.point.x << " " <<  point_camera.point.y << " " <<  point_camera.point.z << std::endl; 

    std::cerr << "point_map: " << point_map.point.x << " " <<  point_map.point.y << " " <<  point_map.point.z << std::endl;*/

    float len = cloud_cylinder->points.size() / 2;
    int n = static_cast<int>(len);
    float_t avg_r = 0;
    float_t avg_g = 0;
    float_t avg_b = 0;
    for (size_t i = 0; i < len; i++) {
      avg_r += cloud_cylinder->points[i].r;
      avg_g += cloud_cylinder->points[i].g;
      avg_b += cloud_cylinder->points[i].b;
    }
    avg_r /= len;
    avg_g /= len;
    avg_b /= len;

    //pcl::compute3DCentroid (cloud_xyzrgb, color);

    vector<double> rgb;
    rgb.push_back(avg_r);
    rgb.push_back(avg_g);
    rgb.push_back(avg_b);
    double hsv;
    hsv = returnHSVvalues(rgb);
    std::string color = ""; 

    std::cerr << " R: " << avg_r << " G: " <<  avg_g << " B: " <<  avg_b << std::endl;
    if (hsv < 15.0 || hsv > 330.0) {
      color = "red";
    }
    else if (40.0 < hsv && hsv < 70.0) {
      color = "yellow";
    }
    else if (90.0 < hsv && hsv < 150.0) {
      color = "green";
    }
    else if (175.0 < hsv && hsv < 240.0) {
      color = "blue";
    }
    std::cerr << " Color: " << color << "(" << hsv << ")" << std::endl;

    bool check = true;

    if (h > 0.44 && h < 0.48 && (w > 0.30 && w < 0.34 || w > 0.25 && w < 0.28 || w > 0.15 && w < 0.18)) {
      for (int i = 0; i < m_id; i++) {
        if ((abs(point_map.point.x - markers[i][0]) < 0.5) && (abs(point_map.point.y - markers[i][1]) < 0.5)) {
          check = false;
        }
      }

      if (check) {

        /*ROS_WARN("Visina: %f\n", h);
        ROS_WARN("Sirina: %f\n", w);*/

        markers[m_id][0] = point_map.point.x;
        markers[m_id][1] = point_map.point.y;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "cylinder";
        marker.id = m_id;

        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = point_map.point.x;
        marker.pose.position.y = point_map.point.y;
        marker.pose.position.z = point_map.point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = avg_r / 255;
        marker.color.g = avg_g / 255;
        marker.color.b = avg_b / 255;
        marker.color.a = 1.0f;

        marker.lifetime = ros::Duration();

        pubm.publish (marker);
        m_id += 1;

        string ss = "Detected " + color + " cylinder.";
        sc.say(ss, "voice_kal_diphone", 1.0);
        if (m_id == 4) {
          exercise6::Message msg;
          chatter_pub.publish(msg);
        } 
      }
    }
  }  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cylinder_segment");
  ros::NodeHandle nh;

  // For transforming between coordinate frames
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubx = nh.advertise<pcl::PCLPointCloud2> ("planes", 1);
  puby = nh.advertise<pcl::PCLPointCloud2> ("cylinder", 1);

  pubm = nh.advertise<visualization_msgs::Marker>("detected_cylinder",1);
  chatter_pub = nh.advertise<exercise6::Message>("chatter", 1);

  // Spin
  ros::spin ();
}
