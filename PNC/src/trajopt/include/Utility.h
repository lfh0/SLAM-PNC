/**************************************************************************
 * utility.h
 * 
 * @Author： 
 * @Date: 2022.03.29
 * 
 * @Description:
 *  需要全局定义的某些变量或者申明
 *  
 *  ****************************************************/
#ifndef _UTILITY_H
#define _UTILITY_H

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include "yaml-cpp/yaml.h"


using namespace std;

// state 枚举声明
enum robotStateEnum
{
    Indoor, 
    In2Outdoor,
    LaneTracking,
    LaneChanging,
    Crossing,
    EmergencyStop,
};


typedef struct MissionPoint{
    int id;
    float x;
    float y;
    float yaw;
    bool isStop; //是否启用视觉伺服进行精准停靠
    std::vector<int> conPts;

    //输出重载
    friend ostream &operator << (ostream&, MissionPoint &p ){
        cout << "== id: " << p.id << " x: "<< p.x << " y:" << p.y << " yaw:" << p.yaw <<" is stop: " << p.isStop << " conpts: ";
        for(auto conPt : p.conPts){
            cout << conPt << " ";
        }
        cout << endl;
        return cout;
    }

}MissionPoint;


namespace YamlProcess{

    void writeMpsTOYAML(std::vector<MissionPoint> mps, std::string yamlFile){
        
        YAML::Node missionNode;
        YAML::Node missionList;
        assert(missionNode.IsNull());
        assert(missionList.IsNull());
        missionNode["mission_points"] = missionList;

        for(auto mp: mps){
            YAML::Node newNode;
            newNode["id"] = mp.id;
            newNode["point"].push_back(mp.x);
            newNode["point"].push_back(mp.y);
            newNode["point"].push_back(mp.yaw);
            newNode["isStop"] = mp.isStop;
            newNode["conPts"] = mp.conPts;
            missionList.push_back(newNode);
        }

        std::cout << "the follow msg will be writted in mission yaml" << std::endl;
        std::cout << "==============================================" << std::endl;
        std::cout << missionNode << std::endl;
        std::cout << "==============================================" << std::endl;

        ofstream fout(yamlFile);
        fout << missionNode;
        fout.close();
        cout << yamlFile << endl;
    }

    void readMpsFromYAML(std::vector<MissionPoint>& missionPoints, std::string yamlFile){
        YAML::Node missionNode;
        try
        {
            missionNode = YAML::LoadFile(yamlFile);
        }
        catch(const YAML::Exception& e)
        {
            std::cerr << "NO Such File" << yamlFile << std::endl;
        }

        YAML::Node missionList = missionNode["mission_points"];
     
        if(missionList.IsSequence()){
            for(auto && item : missionList){
                MissionPoint mp;
                mp.id = item["id"].as<int>();
                mp.x = item["point"][0].as<float>();
                mp.y = item["point"][1].as<float>();
                mp.yaw = item["point"][2].as<float>();
                mp.isStop = item["isStop"].as<bool>();
                mp.conPts = item["conPts"].as<std::vector<int>>();
                missionPoints.push_back(mp);
            }

        }else{
            std::cerr << "the file type is error" << std::endl;
        }
        
    }

}

double CalculateVariance(std::vector<double> list){
    double sum, nums , err_sum, mean , variance;
    nums = list.size();
    sum = 0.0;
    err_sum = 0.0;
    for(int i =0;i < nums; i++){
        sum = sum + list[i];
    }
    mean = sum / nums;
    for(int i =0;i < nums; i++){
        err_sum = err_sum + std::pow(list[i] - mean, 2);
    }
    variance = err_sum / nums;
    return variance;

}

double CalAtan(double y,double x ){
    double angle = std::atan2(y,x);
    angle = angle *180 /M_PI;
    if(angle > 90 ){
        angle = angle -180;
    }
    else if(angle < -90){
        angle = angle + 180;
    }
    return angle;
}


void getLineFromTwoPoints(Eigen::Vector2d p1, Eigen::Vector2d p2, double& A, double& B, double& C) 
{
    A = p2.y() - p1.y();
    B = p1.x() - p2.x();
    C = p2.x() * p1.y() - p1.x() * p2.y();
}

double distanceFromPointToLine(Eigen::Vector2d p, double A, double B, double C) {
    return abs(A * p.x() + B * p.y() + C) / sqrt(A * A + B * B);
}

double GetDistanceFromPathpoint(geometry_msgs::PoseStamped p1 , geometry_msgs::PoseStamped p2 ){
    double dis_x = p2.pose.position.x - p1.pose.position.x;
    double dis_y = p2.pose.position.y - p1.pose.position.y;
    return sqrt(dis_x*dis_x+dis_y*dis_y);
}


#endif