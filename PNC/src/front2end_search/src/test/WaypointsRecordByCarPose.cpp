/**************************************************************************
 * NaviProcess.cpp
 * 
 * Author： Born Chow
 * Date: 2022.07.28
 * 
 * 
 * 说明: 
 * 记录任务点的程序
 ***************************************************************************/
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include "Utility.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <termio.h>
#include <stdio.h>
#include <thread>
#include <mutex>
#include <tf/transform_datatypes.h>             //转换函数头文件

std::vector<MissionPoint> mps_;
nav_msgs::Odometry thisOdom_;
int flag_;
std::mutex mut_;

void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg){
    thisOdom_ = *msg;
}


int scanKeyboard()
{
    int input;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
      
    input = getchar();
      
    tcsetattr(0,TCSANOW,&stored_settings);
    return input;
}

void keyboardThread(){

    while (1)
    {
        mut_.lock();
        flag_ = scanKeyboard();
        std::cout << " input 3 to make mission point 4 to end recode: " << flag_ << std::endl;
        mut_.unlock();
    }
    
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "mission_maker");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    ros::Subscriber trajSub = nh.subscribe<nav_msgs::Odometry>("/odometry/imu", 10, odomCallBack);
    ros::Publisher missionPointPub = nh.advertise<nav_msgs::Odometry>("mission_pose", 10);
    
    std::thread keyboard_thread(keyboardThread);

    ros::Rate rate(20);


    std::vector<MissionPoint> mps;

    std::string dirPackage = ros::package::getPath("missionmaker");
    std::string missionYamlFile = dirPackage + "/config/mission.yaml";

    int id = 0;

    while (ros::ok())
    {
     
        switch (flag_)
        {
            case 49:{
                std::cout << "---- recored " << flag_ << std::endl;
                MissionPoint p;
                p.id = id;
                p.x = thisOdom_.pose.pose.position.x;
                p.y = thisOdom_.pose.pose.position.y;
                p.yaw = tf::getYaw(thisOdom_.pose.pose.orientation);
                p.conPts = {0};
                p.isStop = true;
                mps.push_back(p);
                id++;
                flag_ = 51;
                break;
            }


            case 50:{
                std::cout << "---- recored " << flag_ << std::endl;
                MissionPoint p;
                p.id = id;
                p.x = thisOdom_.pose.pose.position.x;
                p.y = thisOdom_.pose.pose.position.y;
                p.yaw = tf::getYaw(thisOdom_.pose.pose.orientation);
                p.conPts = {0};

                p.isStop = false;
                mps.push_back(p);
                id++;
                flag_ = 51;
                break;
            }

            case 52:{
                
                std::cout << " ========== "<< std::endl;
                std::cout << mps.size() << std::endl;
                YamlProcess::writeMpsTOYAML(mps, missionYamlFile);

                return 1;
            }

            default:{
                std::cout << "---- un-recored " << flag_ << "input 1 to recored stop mission, 2 to recored normal mission,  4 to end "<< std::endl;
                break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    keyboard_thread.join();
}