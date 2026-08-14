#ifndef _ASTAR_H
#define _ASTAR_H

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

class AstarNode {
public:
    /* -------------------- */
    Eigen::Vector2i index;
    Eigen::Vector2d state;
    double g_score, f_score;
    AstarNode* parent;
    char node_state;
    int number;
    /* -------------------- */
    AstarNode() {
      parent = NULL;
      node_state = NOT_EXPAND;
    }
    ~AstarNode(){};
};
typedef AstarNode* AstarNodePtr;

class AstarNodeComparator {
public:
    template <class NodePtr>
    bool operator()(NodePtr node1, NodePtr node2) {
        return node1->f_score > node2->f_score;
    }
};

template <typename T>
struct Astarmatrix_hash : std::unary_function<T, size_t> {
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
class AstarNodeHashTable {
private:
  /* data */

  std::unordered_map<Eigen::Vector2i, NodePtr, Astarmatrix_hash<Eigen::Vector2i>> data_2d_;
  std::unordered_map<Eigen::Vector3i, NodePtr, Astarmatrix_hash<Eigen::Vector3i>> data_3d_;

public:
    AstarNodeHashTable(/* args */) {
    }
    ~AstarNodeHashTable() {
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

class Astar {
private:
    /* ---------- main data structure ---------- */
    vector<AstarNodePtr> path_node_pool_;
    int use_node_num_, iter_num_;
    AstarNodeHashTable<AstarNodePtr> expanded_nodes_;
    std::priority_queue<AstarNodePtr, std::vector<AstarNodePtr>, AstarNodeComparator> open_set_;
    std::vector<AstarNodePtr> path_nodes_;

    vector<Eigen::Vector2d> final_path_;
    nav_msgs::OccupancyGrid globalMap_;
    std::string map_topic_;
    int occupied_threshold_;
    double obstacle_cost_weight_;
    bool unknown_as_occupied_;

    bool has_path_ = false;
    double lambda_heu_;
    double resolution_, inv_resolution_;
    Eigen::Vector2d origin_, map_size_3d_;
    Eigen::Vector2i global_map_size_;
    Eigen::Vector2d map_origin_;
    vector<double> occupancy_buffer_2d_;

    double max_seach_time = 0.1;
    int allocate_num_;
    bool use_search_window_;
    double search_window_margin_;
    Eigen::Vector2i search_min_idx_, search_max_idx_;
    double tie_breaker_ = 1.0 + 1.0 / 10000; 
    bool yaw_prefix_enabled_;
    bool yaw_prefix_strict_;
    int yaw_prefix_points_;

    bool isInMap2d(const Eigen::Vector2d &pos);
    bool isInMap2d(const Eigen::Vector2i &id);
    bool isInSearchWindow(const Eigen::Vector2i &id);
    void posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
    void indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos);
    int getVoxelState2d(const Eigen::Vector2d &pos);
    bool isOccupied(const Eigen::Vector2i& id);
    bool isOccupied(const Eigen::Vector2d& pos);

    void retrievePath(AstarNodePtr end_node);
    void ConvertNodePathToPointPath(vector<AstarNodePtr> path_nodes_);
    bool buildYawPrefix(const Eigen::Vector2d& start_pos,
                        double start_yaw,
                        std::vector<Eigen::Vector2d>& prefix,
                        Eigen::Vector2d& search_start);

    inline double getHeu(Eigen::Vector2d x1, Eigen::Vector2d x2)
    {
      double dx = abs(x1(0) - x2(0));
      double dy = abs(x1(1) - x2(1));
      return tie_breaker_* sqrt(dx * dx + dy * dy);
    }

public:
    Astar();
    ~Astar();
    ros::NodeHandle nh_;
    enum { REACH_HORIZON = 1, REACH_END = 2,  NO_PATH = 3, REACH_END_BUT_SHOT_FAILS = 4};
    
	    void init(ros::NodeHandle& nh);

	    void reset();
	    void setMap(const nav_msgs::OccupancyGrid& map);
	    void setSearchWindowMargin(double margin) { search_window_margin_ = margin; }

    int search(Eigen::Vector2d& start_pos, Eigen::Vector2d& goal_pos);
    int search(Eigen::Vector2d& start_pos, Eigen::Vector2d& goal_pos, double start_yaw);

	    vector<Eigen::Vector2d> getPath(){return final_path_;}
	    typedef shared_ptr<Astar> Ptr;
};
}
#endif
