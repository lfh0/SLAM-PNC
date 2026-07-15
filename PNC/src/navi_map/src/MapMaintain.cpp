/**************************************************************************
 * PalnningServer.cpp
 * 
 * @Author： RobotTeam
 * @Date: 2022.07.07
 * 
 * @Description:
 * 本程序是导航地图维护模块
 *  ****************************************************/
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

class MapMaintain
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber staticMapSub_, odomSub_;
    ros::Publisher globalMapPub_, localMapPub_;

    nav_msgs::OccupancyGrid staticOriginMap_;  //这是全局静态地图
    nav_msgs::OccupancyGrid cSpaceGlobalMap_; // 膨胀后的全局地图
    nav_msgs::Odometry odom_; //小车里程计, 需要弄清这个是表示base_link 到map的转换还是 base_link到odom的转换

    int cnt_;

    float inflationRadius_;
    float localMapSize_;
    std::string mapFrameId_;
    std::string odomTopic_;

private:
    void staticMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void odomCallBack(const nav_msgs::OdometryConstPtr &msg);
    void inflationGlobalMap(nav_msgs::OccupancyGrid map, float radius, nav_msgs::OccupancyGrid &map_infla);
public:
    MapMaintain(ros::NodeHandle nh, ros::NodeHandle privateHh);
    void process();
    ~MapMaintain();
};

MapMaintain::MapMaintain(ros::NodeHandle nh, ros::NodeHandle):
nh_(nh)
{
    ROS_INFO("this is map_maintain process node!...");


    nh_.param<float>("navi_map/mapmaintain/inflation_radius", inflationRadius_, 0.7);
    nh_.param<float>("navi_map/mapmaintain/local_map_size", localMapSize_, 5.0);
    nh_.param<std::string>("navi_map/frame/mapFrame", mapFrameId_, "map");
    nh_.param<std::string>("navi_map/mapmaintain/odom_topic", odomTopic_, "/lio/odom");

    odomSub_ = nh_.subscribe<nav_msgs::Odometry>(odomTopic_, 10, &MapMaintain::odomCallBack, this);
    staticMapSub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/merged_map", 10, &MapMaintain::staticMapCallBack, this);

    globalMapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/global_map", 10);
    localMapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map", 10);

    cnt_ = -2;

    ros::Rate r(20);
    while (staticOriginMap_.data.size()<1){
        ros::spinOnce();
        r.sleep();
        if(cnt_ == -2){
            ROS_INFO("waiting for map....");
            cnt_ = -1;
        }
    } 

    std::cout << "-------------param-----------------------" <<std::endl;
    std::cout << "inflation_radius | " << inflationRadius_ << std::endl;
    std::cout << "local_map_size   | " << localMapSize_ << std::endl;
    std::cout << "map_frame_id     | " << mapFrameId_ << std::endl;
    std::cout << "odom_topic       | " << odomTopic_ << std::endl;
    ROS_INFO("map_maintain process node start!!!!!");
}

MapMaintain::~MapMaintain()
{
}

/** 
 * @brief 膨胀原始全局地图
 * 注意原始地图中 map中 值100表示单元被占用；值0表示空闲，即未占用；-1（无符号类型下的255）表示未知
 * @param map    原始全局地图
 * @param radius  膨胀半径
 * @return map_infla 膨胀后的原始全局地图
 */


void MapMaintain::inflationGlobalMap(nav_msgs::OccupancyGrid map, float radius, nav_msgs::OccupancyGrid &map_infla)
{
    // code for inflation map

    // 向Rviz发送的数据中一定要包含frame_id
    for(size_t i = 0; i < map.data.size(); i++)
    {
        if(map.data[i] >= 50 && map.data[i] <= 100)
        {
            map.data[i] = 100;
        }
        if(map.data[i] >= 0 && map.data[i] < 50)
        {
            map.data[i] = 0;
        }
        
    }
    map_infla.header.frame_id= mapFrameId_;
    map_infla.header.stamp = ros::Time::now();  
    map_infla.info = map.info;
    map_infla.data.clear();
    map_infla.data.resize(map.info.width * map.info.height);

    //第一版膨胀
    std::vector<std::vector<int>> idx;
    std::vector<int> vec_tmp;
    for(int x = 0; x < static_cast<int>(map.info.height); x++)
    {
        for(int y = 0; y < static_cast<int>(map.info.width); y++)
        {
            vec_tmp.push_back(map.data[x * map.info.width + y]);
        }
        idx.push_back(vec_tmp);
        vec_tmp.clear();
    }

    for(int x = 0; x < static_cast<int>(map_infla.info.height); x++)
    {
        for(int y = 0; y < static_cast<int>(map_infla.info.width); y++)
        {
            if(idx[x][y] == 100)
            {
                for(int u = x - radius/map_infla.info.resolution; u < x + radius/map_infla.info.resolution; u++)
                {
                    for (int v = y - radius/map_infla.info.resolution; v < y + radius/map_infla.info.resolution; v++)
                    {
                        if (u < 0 || u >= static_cast<int>(map_infla.info.height))
                        continue;
                        if (v < 0 || v >= static_cast<int>(map_infla.info.width))
                        continue;
                        
                        map_infla.data[u * map_infla.info.width + v] = 100;
                    }
                }
            }
        }
    }
    
    idx.clear();
    
}

void MapMaintain::staticMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    staticOriginMap_ = *msg;
}

void MapMaintain::odomCallBack(const nav_msgs::OdometryConstPtr &msg){
    odom_ = *msg;
}

void MapMaintain::process(){
    cnt_++;

    if(cnt_ % 100 == 0){
        inflationGlobalMap(staticOriginMap_, inflationRadius_, cSpaceGlobalMap_);
    }

    globalMapPub_.publish(cSpaceGlobalMap_);     // 测试用


    //第三步: 根据 local_map_size 把局部地图从全局地图分出来
    nav_msgs::OccupancyGrid local_map;
    local_map.header.frame_id = mapFrameId_;
    local_map.header.stamp = ros::Time::now();
    local_map.info.resolution = cSpaceGlobalMap_.info.resolution;

    float x_origin = odom_.pose.pose.position.x - localMapSize_/2 - 1;
    float y_origin = odom_.pose.pose.position.y - localMapSize_/2;

    local_map.info.origin.position.x = x_origin;
    local_map.info.origin.position.y = y_origin;
    local_map.info.origin.position.z = 0;
    local_map.info.origin.orientation.x = 0;
    local_map.info.origin.orientation.y = 0;
    local_map.info.origin.orientation.z = 0;
    local_map.info.origin.orientation.w = 1;

    int col = (x_origin - cSpaceGlobalMap_.info.origin.position.x)/local_map.info.resolution;
    int row = (y_origin - cSpaceGlobalMap_.info.origin.position.y)/local_map.info.resolution;

    local_map.info.width = localMapSize_/local_map.info.resolution;
    local_map.info.height = localMapSize_/local_map.info.resolution;

    local_map.data.clear();
    local_map.data.resize(local_map.info.width * local_map.info.height);

    std::vector<int> l_idx;
    for(int i = 0; i < static_cast<int>(local_map.info.height); i++)
    {
        for(int j = 0; j < static_cast<int>(local_map.info.width); j++)
        {
            if(cSpaceGlobalMap_.data[(i + row) * cSpaceGlobalMap_.info.width + (j + col)] == 100)
            {
                l_idx.push_back(i * local_map.info.width + j);
            }
        }
    }

    for (auto iter = l_idx.begin(); iter != l_idx.end(); iter++)
    {
        if (*iter < 0 || static_cast<size_t>(*iter) >= local_map.data.size())
        continue;
        local_map.data[*iter] = 100;
        
    }
    

    localMapPub_.publish(local_map);
}

///////


int main(int argc, char** argv){

    ros::init(argc, argv, "map_maintain");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    MapMaintain mapMaintain(nh, nhPrivate);

    ros::Rate rate(20);

    while (ros::ok())
    {
        mapMaintain.process();

        ros::spinOnce();

        rate.sleep();

    }
}
