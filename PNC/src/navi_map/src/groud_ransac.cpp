#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_ransac_tilt");
    ros::NodeHandle nh;
    double z0;
    double z_window;
    double dist_thresh;
    double max_tilt_deg;
    int max_iters;
    std::string input_pcd;
    std::string ground_pcd;
    std::string nonground_pcd;
    std::string ground_topic;
    int MeanK;
    double Thresh;
    nh.getParam("navi_map/map_dir/pcd_file_path", input_pcd);
    nh.getParam("navi_map/map_dir/pcd_ground_out_path", ground_pcd);
    nh.getParam("navi_map/map_dir/pcd_non_ground_out_path", nonground_pcd);
    nh.getParam("navi_map/sensor/height", z0);
    z0=-z0;
    nh.getParam("navi_map/OutlierRemoval/z_window", z_window);
    nh.getParam("navi_map/OutlierRemoval/dist_thresh", dist_thresh);
    nh.getParam("navi_map/OutlierRemoval/max_tilt_deg", max_tilt_deg);
    nh.getParam("navi_map/OutlierRemoval/max_iters", max_iters);
    nh.getParam("navi_map/OutlierRemoval/topic", ground_topic);
    nh.getParam("navi_map/OutlierRemoval/meanK", MeanK);
    nh.getParam("navi_map/OutlierRemoval/Thresh", Thresh);

    ros::Publisher ground_pub = nh.advertise<sensor_msgs::PointCloud2> (ground_topic, 1);

    // 读取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd, *cloud) != 0) {
        ROS_ERROR("Failed to load PCD: %s", input_pcd.c_str());
        return -1;
    }
    ROS_INFO("Loaded %zu points", cloud->points.size());

    // 1) 高度裁剪：只保留 z0 ± z_window
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z0 - z_window, z0 + z_window);
    pass.filter(*cloud_z);
    
    if (cloud_z->empty()) {
        ROS_WARN("No points around z0=%.2f in +/-%.2f m window", z0, z_window);
        return -1;
    }

    // 滤波：移除离群点（可选）
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_z);
    sor.setMeanK(MeanK);
    sor.setStddevMulThresh(Thresh);
    sor.filter(*cloud_z);
    sensor_msgs::PointCloud2 cloud_output_msg;
    cloud_output_msg.header.frame_id = "map";
    pcl::toROSMsg(*cloud_z, cloud_output_msg);

    // 2) 约束法向量接近 Z 轴，允许一定倾斜角
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iters);
    seg.setDistanceThreshold(dist_thresh);

    seg.setAxis(Eigen::Vector3f(0.f, 0.f, 1.f));                    // 地面法向大致对齐 Z
    seg.setEpsAngle(static_cast<float>(max_tilt_deg * M_PI / 180)); // 允许的倾角

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.setInputCloud(cloud_z);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.empty()) {
        ROS_WARN("No near-horizontal plane found (tilt<=%.1f deg).", max_tilt_deg);
        return -1;
    }

    // coeff: ax + by + cz + d = 0
    const double a = coeff->values[0];
    const double b = coeff->values[1];
    const double c = coeff->values[2];
    const double d = coeff->values[3];
    ROS_INFO("RANSAC plane: a=%.4f b=%.4f c=%.4f d=%.4f", a, b, c, d);
    double pinfang = std::sqrt(a*a+b*b+c*c);
    double dist= std::fabs(a*0 + b*0 + c*0 + d)/pinfang;
    ROS_INFO("lidar height: %.4f", dist);


    
    // 3)（可选）用内点做一次最小二乘重拟合，得到更稳的法向
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_z, *inliers, centroid);
    // 近似法向（归一化）
    Eigen::Vector3f n(0, 0, 1.f);
    n.normalize();
    // 根据重拟合法向重估 d：用质心点代入 n·p + d = 0
    float d_refit = -(n.x()*centroid.x() + n.y()*centroid.y() + n.z()*centroid.z());
    ROS_INFO("Refit normal≈[%.3f %.3f %.3f], d=%.3f", n.x(), n.y(), n.z(), d_refit);


    // 4) 基于内点，把“全量”点云分为地面/非地面
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonground(new pcl::PointCloud<pcl::PointXYZ>);
    ground->clear();
    for (const auto& p : cloud->points) {
        double dist = std::fabs(a*p.x + b*p.y + c*p.z + d) / std::sqrt(a*a+b*b+c*c);
        if (dist <= dist_thresh) ground->push_back(p);
        else nonground->push_back(p);
    }

    ROS_INFO("Ground: %zu, Non-ground: %zu", ground->size(), nonground->size());
    pcl::io::savePCDFileBinary(ground_pcd, *ground);
    pcl::io::savePCDFileBinary(nonground_pcd, *nonground);
    ROS_INFO("Saved:\n  %s\n  %s", ground_pcd.c_str(), nonground_pcd.c_str());

    ros::Rate rate(10);
    while (ros::ok()) {
        cloud_output_msg.header.frame_id = "map";
        cloud_output_msg.header.stamp = ros::Time::now();
        ground_pub.publish(cloud_output_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
