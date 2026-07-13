#ifndef RRT_H
#define RRT_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

using namespace std;

namespace path_searching {

class RRT {
public:
    // 树节点结构
    struct Node {
        Eigen::Vector2d position;  // 节点位置
        Node* parent;              // 父节点指针
        double cost;               // 从根节点到当前节点的代价
        std::vector<Node*> children; // 子节点列表
        
        Node() : parent(nullptr), cost(0.0) {}
    };

    RRT();
    ~RRT();

	void init(ros::NodeHandle& nh);
    void setMap(const nav_msgs::OccupancyGrid& map);
    int search(const Eigen::Vector2d& start_pos, const Eigen::Vector2d& goal_pos);
    std::vector<Eigen::Vector2d> getKinoPath(){return final_path_;}
    void reset();
    enum { REACH_HORIZON = 1, REACH_END = 2,  NO_PATH = 3, REACH_END_BUT_SHOT_FAILS = 4};

private:
    // 核心RRT函数
    Node* getRandomNode();
    Node* findNearestNode(const Eigen::Vector2d& point);
    Eigen::Vector2d steer(const Eigen::Vector2d& from, const Eigen::Vector2d& to);
    bool isPathCollisionFree(const Eigen::Vector2d& start, const Eigen::Vector2d& end);
    void rewire(Node* new_node, double radius);
    void retrievePath(Node* end_node);

    // 工具函数
    bool isInMap(const Eigen::Vector2d& pos);
    bool isOccupied(const Eigen::Vector2d& pos);
    void posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& index);
    void indexToPos(const Eigen::Vector2i& index, Eigen::Vector2d& pos);
    double getDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);

    // 地图数据
    nav_msgs::OccupancyGrid global_map_;
    std::string map_topic_;
    int occupied_threshold_;
    bool unknown_as_occupied_;
    std::vector<signed char> occupancy_buffer_;
    Eigen::Vector2d map_origin_;
    Eigen::Vector2i map_size_;
    double resolution_, inv_resolution_;

    // 算法参数
    double step_size_;          // 单步扩展长度
    double goal_bias_;          // 目标偏向概率
    double search_radius_;      // 重布线半径
    double max_search_time_;    // 最大搜索时间(ms)
    int max_iterations_;        // 最大迭代次数

    // 树结构
    Node* root_;
    std::vector<Node*> nodes_;
    std::vector<Node*> node_pool_;
    std::vector<Eigen::Vector2d> final_path_;
    

    // 随机数生成器
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> x_dist_;
    std::uniform_real_distribution<> y_dist_;
    std::uniform_real_distribution<> bias_dist_;

    // ROS相关
    ros::NodeHandle nh_;
    typedef shared_ptr<RRT> Ptr;
};
} // namespace path_searching


#endif // RRT_H
