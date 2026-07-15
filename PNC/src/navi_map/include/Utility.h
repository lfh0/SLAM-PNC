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

#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

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

    inline void writeMpsTOYAML(const std::vector<MissionPoint>& mps, const std::string& yamlFile){
        
        YAML::Node missionNode;
        YAML::Node missionList;
        assert(missionNode.IsNull());
        assert(missionList.IsNull());
        missionNode["mission_points"] = missionList;

        for(const auto& mp: mps){
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

    inline void readMpsFromYAML(std::vector<MissionPoint>& missionPoints, const std::string& yamlFile){
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




#endif
