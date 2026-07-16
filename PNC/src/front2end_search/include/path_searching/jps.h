#ifndef JPS_H
#define JPS_H

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

using namespace std;

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

namespace path_searching{

class JPSNode {
    public:
        /* -------------------- */
        Eigen::Vector2i index;
        Eigen::Vector2d state;
        double g_score, f_score;
        JPSNode* parent;
        char node_state;
        int number;
        /* -------------------- */
        JPSNode() {
            parent = NULL;
            node_state = NOT_EXPAND;
        }
        ~JPSNode(){};
};
typedef JPSNode* JPSNodePtr;

class JPSNodeComparator {
public:
    template <class NodePtr>
    bool operator()(NodePtr node1, NodePtr node2) {
        return node1->f_score > node2->f_score;
    }
};

template <typename T>
struct JPSmatrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (long int i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

template <class NodePtr>
class JPSNodeHashTable {
private:
  /* data */

  std::unordered_map<Eigen::Vector2i, NodePtr, JPSmatrix_hash<Eigen::Vector2i>> data_2d_;
  std::unordered_map<Eigen::Vector3i, NodePtr, JPSmatrix_hash<Eigen::Vector3i>> data_3d_;

public:
    JPSNodeHashTable(/* args */) {
    }
    ~JPSNodeHashTable() {
    }
    // : for 2d vehicle planning
    void insert(Eigen::Vector2i idx, NodePtr node) {
        data_2d_.insert(std::make_pair(idx, node));
    }
    NodePtr find(Eigen::Vector2i idx) {
        auto iter = data_2d_.find(idx);
        return iter == data_2d_.end() ? NULL : iter->second;
    }
    void clear() {
        data_2d_.clear();
        data_3d_.clear();
    }
};

class JPS{
public:
    JPS();
    ~JPS();
    ros::NodeHandle nh_;
    typedef shared_ptr<JPS> Ptr;

    ros::Publisher open_set_pub_;
    ros::Publisher close_set_pub_;
    ros::Publisher path_pub_;
    ros::Publisher jump_point_pub_;

    void visualizeOpenSet();
    void visualizeCloseSet();
    void visualizePath(const std::vector<Eigen::Vector2d>& path);
    void visualizeJumpPoints(const std::vector<Eigen::Vector2i>& jump_points);

    const std::vector<float> OPEN_SET_COLOR = {0.0, 1.0, 0.0, 0.5}; // 绿色
    const std::vector<float> CLOSE_SET_COLOR = {1.0, 0.0, 0.0, 0.5}; // 红色
    const std::vector<float> PATH_COLOR = {0.0, 0.0, 1.0, 1.0}; // 蓝色
    const std::vector<float> JUMP_POINT_COLOR = {1.0, 1.0, 0.0, 1.0}; // 黄色
    
private:
    vector<JPSNodePtr> path_node_pool_;
    int use_node_num_, iter_num_;
    JPSNodeHashTable<JPSNodePtr> expanded_nodes_;
    std::priority_queue<JPSNodePtr, std::vector<JPSNodePtr>, JPSNodeComparator> open_set_;

    std::vector<JPSNodePtr> path_nodes_;
    vector<Eigen::Vector2d> final_path_;
    nav_msgs::OccupancyGrid globalMap_;
    std::string map_topic_;
    int occupied_threshold_;
    double obstacle_cost_weight_;
    bool unknown_as_occupied_;
    Eigen::Vector2d goal_pos_;
    
    double max_seach_time = 0.1;
    int allocate_num_;

    bool has_path_ = false;
    double lambda_heu_;
    double resolution_, inv_resolution_;
    Eigen::Vector2d origin_, map_size_3d_;
    Eigen::Vector2i global_map_size_;
    Eigen::Vector2d map_origin_;
    vector<double> occupancy_buffer_2d_;
    double tie_breaker_ = 1.0 + 1.0 / 10000; 

public:
    void init(ros::NodeHandle& nh);
    void reset();
    void setMap(const nav_msgs::OccupancyGrid& map);
    enum { REACH_HORIZON = 1, REACH_END = 2,  NO_PATH = 3, REACH_END_BUT_SHOT_FAILS = 4};

    bool isInMap2d(const Eigen::Vector2d &pos);
    bool isInMap2d(const Eigen::Vector2i &id);
    void posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
    void indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos);
    int getVoxelState2d(const Eigen::Vector2d &pos);
    bool isOccupiedindex(const Eigen::Vector2i& id);
    bool isOccupied(const Eigen::Vector2d& pos);
    double getTraversalCost(const Eigen::Vector2i& start, const Eigen::Vector2i& end) const;

    inline double getHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
    {
      double dx = abs(x1(0) - x2(0));
      double dy = abs(x1(1) - x2(1));
      return tie_breaker_* sqrt(dx * dx + dy * dy);
    }

    void retrievePath(JPSNodePtr end_node);
    void ConvertNodePathToPointPath(vector<JPSNodePtr> path_nodes_);
    vector<Eigen::Vector2d> getKinoPath(){return final_path_;}

    // 跳跃函数
    int search(Eigen::Vector2d& start_pos, Eigen::Vector2d& goal_pos);
    Eigen::Vector2i jump(const Eigen::Vector2i& current, const Eigen::Vector2i& direction);
    
    // 获取强制邻居
    void getForcedNeighbors(const Eigen::Vector2i& current, 
                            const Eigen::Vector2i& direction,
                            std::vector<Eigen::Vector2i>& neighbors);
    // 检查直线路径是否可行
    bool hasLineOfSight(const Eigen::Vector2i& start, const Eigen::Vector2i& end);
    
    // 方向枚举
    enum Direction {
        NONE = 0,
        NORTH = 1,
        SOUTH = 2,
        EAST = 4,
        WEST = 8,
        NE = NORTH | EAST,
        NW = NORTH | WEST,
        SE = SOUTH | EAST,
        SW = SOUTH | WEST
    };
    
    // 方向向量
    const std::vector<Eigen::Vector2i> directions_ = {
        Eigen::Vector2i(0, 1),    // NORTH
        Eigen::Vector2i(1, 0),    // EAST
        Eigen::Vector2i(0, -1),   // SOUTH
        Eigen::Vector2i(-1, 0),   // WEST
        Eigen::Vector2i(1, 1),    // NE
        Eigen::Vector2i(-1, 1),   // NW
        Eigen::Vector2i(1, -1),   // SE
        Eigen::Vector2i(-1, -1)   // SW
    };
};
}



#endif // JPS_H
