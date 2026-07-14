
#include "Utility.h"
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>  

class Missionpointmaker
{
private:
    ros::NodeHandle nh_;
    
    ros::Subscriber getPathpointSub_;
    ros::Publisher showPathpointPub_;
    ros::Subscriber PathpointPub_;

    std::vector<geometry_msgs::Pose> missionPointstest_; //测试用
    geometry_msgs::PoseArray missionPointsVis_; //发布给rviz用
    std::vector<MissionPoint> missionPoints_;//写入ymal文件用




private:

    void getPathpointCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);



public:
    Missionpointmaker(ros::NodeHandle nh);
    void process();
    std::string dirPackage = ros::package::getPath("missionmaker");
    int point_id = 0; 

};


Missionpointmaker::Missionpointmaker(ros::NodeHandle nh):
nh_(nh)
{
    ROS_INFO("This is a node that mark the path point...");
 
    getPathpointSub_ = nh_.subscribe("/move_base_simple/goal", 1, &Missionpointmaker::getPathpointCallBack, this);
    showPathpointPub_ = nh_.advertise<geometry_msgs::PoseArray>("/mission_points",1);
}


void Missionpointmaker::getPathpointCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg){
    MissionPoint thismissionpoint;
    geometry_msgs::Pose pointPose;

    pointPose = msg->pose;
    missionPointsVis_.poses.push_back(pointPose);
    std::cout << "点击的点" << pointPose << std::endl;
    for(int i = 0 ; i < missionPoints_.size(); i++){
        std::cout << "列表" << missionPoints_[i] << std::endl << std::endl;
    }
    thismissionpoint.id = point_id;
    thismissionpoint.x = msg->pose.position.x;
    thismissionpoint.y = msg->pose.position.y;
    thismissionpoint.yaw = tf::getYaw(msg->pose.orientation);
    thismissionpoint.conPts = {0};
    thismissionpoint.isStop = true;
    missionPoints_.push_back(thismissionpoint);


    point_id ++;

}

void Missionpointmaker::process(){
    missionPointsVis_.header.frame_id = "map";
    missionPointsVis_.header.seq = 0;
    showPathpointPub_.publish(missionPointsVis_);
    ros::Duration(0.1).sleep();
    YamlProcess::writeMpsTOYAML(missionPoints_, dirPackage + "/config/pointsmission.yaml");

}




int main(int argc, char** argv){
    ros::init(argc,argv,"missionmaker");
    ros::NodeHandle nh;

    Missionpointmaker missionpointmaker(nh );

    ros::Rate rate(20);

    while(ros::ok())
    {   
        missionpointmaker.process();
        ros::spinOnce();
        rate.sleep();
    }


}