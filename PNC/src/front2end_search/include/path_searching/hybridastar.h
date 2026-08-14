#ifndef HYBRIDASTAR_H
#define HYBRIDASTAR_H

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <array>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace path_searching {

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
    HybridStateVector;
typedef std::array<Eigen::Vector2d, 4> VehicleCorners;

enum class NodeStatus {
    UNVISITED = 0,
    IN_OPEN_SET,
    IN_CLOSE_SET
};

enum class Direction {
    NONE = 0,
    FORWARD = 1,
    BACKWARD = -1
};

class HybridNode {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 连续车辆状态：x、y、yaw。
    Eigen::Vector3d state;
    // 离散搜索索引：x、y、yaw。
    Eigen::Vector3i grid_index;

    Direction direction;
    int steering_index;
    double steering;

    double g_score;
    double f_score;

    NodeStatus status;
    HybridNode* parent;

    // 从父节点扩展到当前节点的运动原语中间状态。
    HybridStateVector intermediate_states;

    HybridNode()
        : state(Eigen::Vector3d::Zero()),
          grid_index(Eigen::Vector3i::Zero()),
          direction(Direction::NONE),
          steering_index(0),
          steering(0.0),
          g_score(std::numeric_limits<double>::infinity()),
          f_score(std::numeric_limits<double>::infinity()),
          status(NodeStatus::UNVISITED),
          parent(nullptr) {
    }

    ~HybridNode() = default;
};

typedef HybridNode* HybridNodePtr;

// Hybrid A* 的节点唯一键由位姿栅格和行驶方向共同组成。
struct HybridNodeKey {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3i grid_index;
    Direction direction;

    HybridNodeKey()
        : grid_index(Eigen::Vector3i::Zero()),
          direction(Direction::NONE) {
    }

    HybridNodeKey(const Eigen::Vector3i& index, const Direction node_direction)
        : grid_index(index),
          direction(node_direction) {
    }

    bool operator==(const HybridNodeKey& other) const {
        return direction == other.direction
            && (grid_index.array() == other.grid_index.array()).all();
    }
};

struct HybridNodeKeyHash {
    std::size_t operator()(const HybridNodeKey& key) const {
        std::size_t seed = 0;
        const auto hash_combine = [&seed](const int value) {
            seed ^= std::hash<int>()(value) + 0x9e3779b9
                + (seed << 6) + (seed >> 2);
        };

        hash_combine(key.grid_index.x());
        hash_combine(key.grid_index.y());
        hash_combine(key.grid_index.z());
        hash_combine(static_cast<int>(key.direction));
        return seed;
    }
};

// Open Set 保存入队时的代价快照，避免修改节点代价破坏堆顺序。
struct OpenSetEntry {
    double f_score;
    double g_score;
    HybridNodePtr node;

    OpenSetEntry(const double node_f_score,
                 const double node_g_score,
                 HybridNodePtr node_ptr)
        : f_score(node_f_score),
          g_score(node_g_score),
          node(node_ptr) {
    }
};

struct OpenSetComparator {
    bool operator()(const OpenSetEntry& lhs, const OpenSetEntry& rhs) const {
        if (lhs.f_score != rhs.f_score) {
            return lhs.f_score > rhs.f_score;
        }
        // f 相同时优先扩展 g 更大、更接近目标的节点。
        return lhs.g_score < rhs.g_score;
    }
};

class HybridAstar {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum SearchStatus {
        REACH_END = 1,
        NO_PATH = 2
    };

    HybridAstar();
    ~HybridAstar();

    void init(ros::NodeHandle& nh);
    void setMap(const nav_msgs::OccupancyGrid& map);
    void reset();

    int search(const Eigen::Vector3d& start_state,
               const Eigen::Vector3d& goal_state);

    const HybridStateVector& getPath() const;
    const HybridStateVector& getVisitedStates() const;

private:
    // 独立 ROS 节点接口：订阅原始地图、里程计和目标，固定发布规划路径。
    ros::Subscriber map_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber goal_subscriber_;
    ros::Publisher path_publisher_;
    ros::Publisher search_tree_publisher_;
    std::string map_topic_;
    std::string odom_topic_;
    std::string goal_topic_;
    nav_msgs::Odometry latest_odom_;
    bool odom_initialized_;
    int search_tree_max_edges_;

    // 地图参数，由原始栅格地图的信息初始化。
    Eigen::Vector2d map_origin_;
    Eigen::Vector2i map_size_;
    double map_resolution_;
    double inverse_map_resolution_;
    bool map_initialized_;

    // 状态离散参数。
    int yaw_grid_size_;
    double yaw_resolution_;
    double inverse_yaw_resolution_;

    // 车辆参数，连续状态中的位置表示后轴中心。
    double vehicle_width_;
    double vehicle_length_;
    double wheel_base_;
    double rear_overhang_;
    double max_steering_angle_;
    double minimum_turning_radius_;
    // 单侧转角离散等级，不包含零转角。
    int steering_discrete_num_;

    // 运动原语参数。
    double primitive_length_;
    double primitive_sample_step_;

    // 搜索代价参数。
    double forward_penalty_;
    double reverse_penalty_;
    double gear_switch_penalty_;
    double steering_penalty_;
    double steering_change_penalty_;
    double heuristic_weight_;
    double reeds_shepp_switch_distance_;
    double analytic_expansion_distance_;

    // 终点判定与搜索限制参数。
    double goal_position_tolerance_;
    double goal_yaw_tolerance_;
    int max_iterations_;
    double max_search_time_;

    // 将角度归一化到 [-pi, pi)。
    double normalizeAngle(double angle) const;
    // 将航向角离散到 [0, yaw_grid_size_ - 1]。
    int yawToIndex(double yaw) const;
    // 判断二维栅格索引是否位于地图范围内。
    bool isInsideMap(const Eigen::Vector2i& grid_index) const;
    // 将世界坐标系中的二维位置转换为地图栅格索引。
    bool positionToIndex(const Eigen::Vector2d& position,
                         Eigen::Vector2i& grid_index) const;
    // 将世界坐标系中的连续状态转换为离散搜索索引。
    bool stateToIndex(const Eigen::Vector3d& state,
                      Eigen::Vector3i& grid_index) const;

    // 将离散转角索引转换为实际前轮转角。
    double steeringIndexToAngle(int steering_index) const;
    // 使用车辆运动学模型生成一段运动原语及其内部采样状态。
    bool generatePrimitive(const HybridNode& current_node,
                           Direction direction,
                           int steering_index,
                           HybridStateVector& intermediate_states) const;

    // 根据转角和最外侧车角运动距离计算运动原语采样段数。
    int calculatePrimitiveSampleCount(double steering) const;

    // 地图外、未知栅格及非零占用栅格均视为障碍物。
    bool isOccupied(const Eigen::Vector2i& grid_index) const;
    // 检查栅格直线经过的全部单元是否无碰撞。
    bool isGridLineCollisionFree(const Eigen::Vector2i& start,
                                 const Eigen::Vector2i& end) const;

    // 根据后轴中心状态计算车辆四个角点。
    VehicleCorners getVehicleCorners(const Eigen::Vector3d& state) const;
    // 只检查车辆四条边是否无碰撞。
    bool isVehicleBoundaryCollisionFree(
        const Eigen::Vector3d& state) const;
    // 检查车辆矩形覆盖的全部栅格是否无碰撞。
    bool isVehicleFootprintCollisionFree(
        const Eigen::Vector3d& state) const;
    // 中间状态检查四边，原语末端检查完整车身。
    bool isPrimitiveCollisionFree(
        const HybridStateVector& intermediate_states) const;

    // 计算一段运动原语产生的累计代价。
    double calculateTransitionCost(const HybridNode& current_node,
                                   Direction next_direction,
                                   double next_steering) const;

    // 计算忽略障碍物但满足车辆最小转弯半径的 Reeds-Shepp 距离。
    double calculateReedsSheppHeuristic(
        const Eigen::Vector3d& state,
        const Eigen::Vector3d& goal_state) const;
    // 远距离使用欧氏距离，近距离使用 Reeds-Shepp 距离。
    double calculateHeuristic(const Eigen::Vector3d& state,
                              const Eigen::Vector3d& goal_state) const;
    // 从目标栅格反向运行一次八邻域 Dijkstra。
    bool buildObstacleHeuristic(const Eigen::Vector2i& goal_index);
    // 查询当前状态对应的二维绕障距离缓存。
    double calculateObstacleHeuristic(
        const Eigen::Vector3d& state) const;

    // 生成当前状态到精确目标的 Reeds-Shepp 曲线并检查碰撞。
    bool tryReedsSheppConnection(
        const Eigen::Vector3d& start_state,
        const Eigen::Vector3d& goal_state,
        HybridStateVector& connection_states) const;
    // 将无碰撞的解析连接拼接到最终路径。
    void appendAnalyticPath(
        const HybridStateVector& connection_states);

    bool isGoalReached(const Eigen::Vector3d& state,
                       const Eigen::Vector3d& goal_state) const;
    HybridNodePtr createNode();
    void retrievePath(HybridNodePtr goal_node);

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& message);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& message);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& message);
    void publishPath(const HybridStateVector& path);
    void publishSearchTree() const;

    std::unordered_map<HybridNodeKey, HybridNodePtr, HybridNodeKeyHash>
        expanded_nodes_;
    std::priority_queue<OpenSetEntry,
                        std::vector<OpenSetEntry>,
                        OpenSetComparator>
        open_set_;
    std::vector<std::unique_ptr<HybridNode>> node_pool_;

    HybridStateVector final_path_;
    HybridStateVector visited_states_;
    nav_msgs::OccupancyGrid map_;

    // Reeds-Shepp 状态空间。
    std::shared_ptr<ompl::base::ReedsSheppStateSpace>
        reeds_shepp_state_space_;

    // 目标点反向 Dijkstra 生成的二维障碍距离缓存。
    std::vector<double> obstacle_heuristic_cost_;
    Eigen::Vector2i obstacle_heuristic_goal_index_;
    bool obstacle_heuristic_ready_;
};

}  // namespace path_searching

#endif  // HYBRIDASTAR_H
