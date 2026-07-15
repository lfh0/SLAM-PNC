#include <iostream>
#include <string>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
 
int main (int argc, char **argv)
{
  ros::init (argc, argv, "UandBdetect");
  ros::NodeHandle nh;
  std::string pcd_file_path;
  std::string pointcloudFrame;
  std::string readpcd_topic;
  int meanK;
  double Thresh;
  pcl::PointCloud<pcl::PointXYZ> cloud_output;


  nh.getParam("navi_map/output/readpcd_topic", readpcd_topic);
  nh.getParam("navi_map/map_dir/pcd_file_path", pcd_file_path);
  std::cout << "pcd_file_path"<<pcd_file_path << std::endl;
  nh.getParam("navi_map/frame/pointcloudFrame", pointcloudFrame);
  nh.getParam("navi_map/OutlierRemoval/meanK", meanK);
  nh.getParam("navi_map/OutlierRemoval/Thresh", Thresh);


  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> (readpcd_topic, 1);

  // 加载点云文件
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (pcd_file_path.c_str(), *cloud_input); //修改自己pcd文件所在路径

  // 滤波操作，将点云中的离群雷达噪声点过滤一遍
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_input);
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(Thresh);
  sor.filter(cloud_output);
  
  //Convert the cloud to ROS message
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud_output, output);
  output.header.frame_id = pointcloudFrame;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
