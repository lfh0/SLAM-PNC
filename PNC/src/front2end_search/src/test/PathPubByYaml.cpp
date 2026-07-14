#include "Utility.h"
#include "smoother.hpp"
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>  
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


/////////////////////////////////////////////////////////////////////////////////////////////////
class GlobalPathMaker
{
private:
    ros::NodeHandle nh_;
    std::vector<MissionPoint> missionPoints_;
    nav_msgs::Path gobalPath_;
    nav_msgs::Path midpath_;



    double densepointnums_ = 100;

public:
    GlobalPathMaker(ros::NodeHandle nh);
    nav_msgs::Path StraightPath();
    nav_msgs::Path StraightPathbyMidline();
    void GlobalPath();
    void ReadTxt();
    void ShowPoints(ros::Publisher,std::vector<Eigen::Vector3d> pointslist,double r ,double g ,double b);
    void GetStraightLine();
    void RuninPath(nav_msgs::Path gobalpath);


    ros::Publisher gobalPathPub_,showPointsPub_,showPointsPub2_;
    std::string dirPackage = ros::package::getPath("missionmaker");
    std::vector<Eigen::Vector3d> midline_;
    std::vector<Eigen::Vector3d> midline_straight_;
    std::vector<std::vector<Eigen::Vector3d>> midline_list_;
};






GlobalPathMaker::GlobalPathMaker(ros::NodeHandle nh):
nh_(nh)
{
    ROS_INFO("This is a ndoe will pub global after optimze");
    gobalPathPub_ = nh_.advertise<nav_msgs::Path>("/planner/global_traj_path", 10);
    showPointsPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/showpoints", 10);
    showPointsPub2_ = nh_.advertise<visualization_msgs::MarkerArray>("/showpoints2", 10);
    YamlProcess::readMpsFromYAML(missionPoints_, dirPackage + "/config/pointsmission.yaml");

    
}



//获得基于中心线规划的全局路径
nav_msgs::Path GlobalPathMaker::StraightPathbyMidline(){
    nav_msgs::Path straightpath;
    geometry_msgs::PoseStamped somepoint;
    geometry_msgs::PoseStamped markerpoint;;


    for(int i =0; i < missionPoints_.size() -1 ; i++){
        // std::cout << "point num " << i << std::endl;
        double A,B,C;
        Eigen::Vector2d start,end;
        start[0] = missionPoints_[i].x;
        start[1] = missionPoints_[i].y;
        end[0] = missionPoints_[i + 1].x;
        end[1] = missionPoints_[i + 1].y;

        markerpoint.pose.position.x = missionPoints_[i].x;
        markerpoint.pose.position.y = missionPoints_[i].y;
        markerpoint.pose.position.z = 0;
        markerpoint.pose.orientation = tf::createQuaternionMsgFromYaw(missionPoints_[i].yaw);
        straightpath.poses.push_back(markerpoint);

        getLineFromTwoPoints(start,end,A,B,C);
        // std::cout << "A " << A << "B "<< B << "C "<< C <<std::endl;
        for(int j = 0; j < midline_straight_.size(); j++){

            geometry_msgs::PoseStamped somepoint;


            Eigen::Vector2d thispoint;
            double distance = 0.0;
            thispoint[0] = midline_straight_[j][0];
            thispoint[1] = midline_straight_[j][1];
            
            distance = distanceFromPointToLine(thispoint,A,B,C);
            // std::cout << "distance" << distance <<std::endl;

            if(distance < 1){
                if(
                ((start[0] -2 < thispoint[0] && thispoint[0] < end[0] +2)||(start[0] + 2 > thispoint[0] && thispoint[0] > end[0] -2)) 
                &&
                ((start[1] -2< thispoint[1] && thispoint[1] < end[1] +2)||(start[1] +2> thispoint[1] && thispoint[1] > end[1]-2))
                ){
                somepoint.header.frame_id = "map";
                somepoint.pose.position.x = midline_straight_[j][0];
                somepoint.pose.position.y = midline_straight_[j][1];
                somepoint.pose.position.z = 0;
                somepoint.pose.orientation = tf::createQuaternionMsgFromYaw(missionPoints_[i].yaw);
                straightpath.poses.push_back(somepoint);
                }

            }
        }
    }

    markerpoint.pose.position.x = missionPoints_[missionPoints_.size()-1].x;
    markerpoint.pose.position.y = missionPoints_[missionPoints_.size()-1].y;
    markerpoint.pose.position.z = 0;
    markerpoint.pose.orientation = tf::createQuaternionMsgFromYaw(missionPoints_[missionPoints_.size()-1].yaw);
    straightpath.poses.push_back(markerpoint);
    straightpath.header.frame_id= "map";

    return straightpath;

}

void GlobalPathMaker::RuninPath(nav_msgs::Path gobalpath){
    double dis,nums,runindis;
    runindis = 0.25;
    geometry_msgs::PoseStamped somepoint;
    gobalPath_.header.frame_id = "map";
    for(int i = 0;  i < gobalpath.poses.size() -1 ; i++ ){
        dis = GetDistanceFromPathpoint(gobalpath.poses[i+1],gobalpath.poses[i]);
        // std::cout << "i  " << i << "dis  " << dis <<std::endl;
            somepoint.header.frame_id = "map";
            somepoint.pose.position.x = gobalpath.poses[i].pose.position.x;
            somepoint.pose.position.y = gobalpath.poses[i].pose.position.y;
            somepoint.pose.position.z = 0 ;
            somepoint.pose.orientation = gobalpath.poses[i].pose.orientation;
            gobalPath_.poses.push_back(somepoint);
        if(dis> runindis){
            double start_x = gobalpath.poses[i].pose.position.x;
            double start_y = gobalpath.poses[i].pose.position.y;
            double end_x = gobalpath.poses[i+1].pose.position.x;
            double end_y = gobalpath.poses[i+1].pose.position.y;
            nums = dis / runindis;
            
            for(double j = 0; j < nums; j++){

                somepoint.header.frame_id = "map";
                somepoint.pose.position.x = start_x + j/ nums *(end_x -start_x); 
                somepoint.pose.position.y = start_y + j/ nums *(end_y -start_y);
                somepoint.pose.position.z = 0 ;
                somepoint.pose.orientation =  gobalpath.poses[i].pose.orientation;
                gobalPath_.poses.push_back(somepoint);
            }
        }
    }
}




nav_msgs::Path GlobalPathMaker::StraightPath(){
    nav_msgs::Path straightPath;

    int points_nums = missionPoints_.size();
    // std::vector<geometry_msgs::PoseStamped> thisstraightpath;
    for(int i = 0; i < points_nums-1; i++){
        double start_x = missionPoints_[i].x;
        double start_y = missionPoints_[i].y;
        double end_x = missionPoints_[i + 1].x;
        double end_y = missionPoints_[i + 1].y;
        // std::cout<< "start " << start_x << "  "<< start_y <<std::endl;
        // std::cout<< "end " << end_x << "  "<< end_y <<std::endl;
        for(double j = 0; j < densepointnums_; j++){
            geometry_msgs::PoseStamped somepoint;
            somepoint.header.frame_id = "map";
            somepoint.pose.position.x = start_x + j/ densepointnums_ *(end_x -start_x); 
            somepoint.pose.position.y = start_y + j/ densepointnums_ *(end_y -start_y);
            somepoint.pose.position.z = 0 ;
            somepoint.pose.orientation = tf::createQuaternionMsgFromYaw(missionPoints_[i].yaw);
            straightPath.poses.push_back(somepoint);
        }
    }
    straightPath.header.frame_id= "map";
    // gobalPathPub_.publish(straightPath_);

    // ros::Duration(1).sleep();
    return straightPath;
}

void GlobalPathMaker::GlobalPath(){

    /* 以前的版本 */
    // nav_msgs::Path gobalpath;
    // Smoother smoother(-251.250003, -212.399997,300,300);
    // gobalPath_ = StraightPath();
    // smoother.smoothPath(gobalPath_);
    // gobalPathPub_.publish(gobalPath_);

    gobalPath_.poses.clear();
    nav_msgs::Path gobalpath;
    Smoother smoother(-251.250003, -212.399997,300,300);
    gobalpath = StraightPathbyMidline();
    // std::cout<< "path run in" << std::endl;
    RuninPath(gobalpath);
    std::cout << "smooth path" << std::endl;
    smoother.smoothPath(gobalPath_);
    std::cout<<"path pub" <<std::endl;
    gobalPathPub_.publish(gobalPath_);
}

void GlobalPathMaker::ReadTxt(){
    std::ifstream file(dirPackage + "/roadpoint/0719_road.txt");
    std::string line;
    while(std::getline(file,line)){
        std::stringstream midpointxyz(line);
        Eigen::Vector3d midpoint;
        double num;
        int i = 0;
        while(midpointxyz >> num){
            midpoint[i] = num;
            i++;
        }
        midpoint[2] = 0.0;
        midline_.push_back(midpoint);
    }
    file.close();
}


void GlobalPathMaker::ShowPoints(ros::Publisher showpub,std::vector<Eigen::Vector3d> pointslist, double r ,double g ,double b){
    visualization_msgs::Marker show_marker;
    visualization_msgs::MarkerArray show_marker_array;
    for(int i = 0 ; i < pointslist.size(); i ++){
        show_marker.id = i;
        show_marker.header.stamp = ros::Time::now();
        show_marker.header.frame_id = "map";
        show_marker.color.r = r;
        show_marker.color.g = g;
        show_marker.color.b = b;
        show_marker.color.a = 1.0;
        show_marker.scale.x = 1;
        show_marker.scale.y = 1;
        show_marker.scale.z = 1;
        show_marker.pose.orientation.w = 1;
        show_marker.pose.orientation.x = 0;
        show_marker.pose.orientation.y = 0;
        show_marker.pose.orientation.z = 0;
        show_marker.pose.position.x = pointslist[i][0];
        show_marker.pose.position.y = pointslist[i][1];
        show_marker.pose.position.z = 0;
        //somepoimtPub.publish(occupancy_marker);
        show_marker_array.markers.push_back(show_marker);
        // std::cout << "nums " << i << "x" << pointslist[i][0] << "y" << pointslist[i][0]<<std::endl; 
    }
    showpub.publish(show_marker_array);
}


//获得比较直的中心线存入midline_straight_中
void GlobalPathMaker::GetStraightLine(){
    int leftptr,rightptr,steplenght;
    double k;
    double variance;
    std::vector<double> variance_list;
    steplenght = 10;
    for(leftptr = 0 ;leftptr < midline_.size()-steplenght; leftptr += steplenght){
        rightptr = leftptr + steplenght;
        std::vector<double> k_list;
        for(int i = 0; i< steplenght/2;i++){
            double y = midline_[leftptr + i + steplenght / 2][1] - midline_[leftptr + i][1];
            double x = (midline_[leftptr + i + steplenght / 2][0] - midline_[leftptr + i][0]);
            k = CalAtan(y,x);
            k_list.push_back(k);
        }
        variance = CalculateVariance(k_list);
        variance_list.push_back(variance);
        // std::cout<<"leftptr " <<leftptr <<"variance" << variance << std::endl;
        // std::cout << "rightptr" << rightptr <<std::endl;
        if(variance < 5){
            for(int j = leftptr ; j < rightptr; j++){
                midline_straight_.push_back(midline_[j]);
            }
        }
    }

    // std::cout<<"size of midline_" << midline_.size() << std::endl;

}



int main(int argc, char** argv){
    ros::init(argc,argv,"missionmaker");
    ros::NodeHandle nh;
    GlobalPathMaker globalpathmaker(nh );
    nav_msgs::Path tempPath;
    ros::Rate rate(1);
    globalpathmaker.ReadTxt();
    globalpathmaker.GetStraightLine();



    while(ros::ok())
    {   
        ros::spinOnce();
        // std::cout<<"while" << std::endl;
        globalpathmaker.GlobalPath();
        globalpathmaker.ShowPoints(globalpathmaker.showPointsPub_,globalpathmaker.midline_,0.0,0.0,255.0);
        globalpathmaker.ShowPoints(globalpathmaker.showPointsPub2_, globalpathmaker.midline_straight_,255,0.0,0.0);

        rate.sleep();
    }
}





