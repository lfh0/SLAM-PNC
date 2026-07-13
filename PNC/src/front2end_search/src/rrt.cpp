#include <path_searching/rrt.h>
#include <algorithm>
#include <cmath>

#include <sstream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

namespace path_searching {


RRT::RRT(){

}

RRT::~RRT() {
    reset();
}

void RRT::init(ros::NodeHandle& nh) {
    nh_ = nh;
    // 从参数服务器获取参数
    nh_.param("rrt/step_size", step_size_, 1.0);
    nh_.param("rrt/goal_bias", goal_bias_, 0.7);
    nh_.param("rrt/search_radius", search_radius_, 3.0);
    nh_.param("rrt/max_search_time", max_search_time_, 50000.0);
    nh_.param("rrt/max_iterations", max_iterations_, 50000);
    nh_.param("search/occupied_threshold", occupied_threshold_, 50);
    nh_.param("search/unknown_as_occupied", unknown_as_occupied_, true);
    
    // 初始化随机数生成器范围（会在setMap中更新）
    x_dist_ = std::uniform_real_distribution<>(0.0, 10.0);
    y_dist_ = std::uniform_real_distribution<>(0.0, 10.0);
    bias_dist_ = std::uniform_real_distribution<>(0.0, 1.0);
    
    nh_.param<std::string>("search/map_topic", map_topic_, "/projected_map");
}

void RRT::setMap(const nav_msgs::OccupancyGrid& map) {
    global_map_ = map;
    resolution_ = global_map_.info.resolution;
    inv_resolution_ = 1.0 / resolution_;
    map_size_ = Eigen::Vector2i(global_map_.info.width, global_map_.info.height);
    map_origin_ = Eigen::Vector2d(
        global_map_.info.origin.position.x,
        global_map_.info.origin.position.y);

    x_dist_ = std::uniform_real_distribution<>(
        map_origin_.x(), map_origin_.x() + map_size_.x() * resolution_);
    y_dist_ = std::uniform_real_distribution<>(
        map_origin_.y(), map_origin_.y() + map_size_.y() * resolution_);

    occupancy_buffer_.resize(global_map_.data.size());
    for (size_t i = 0; i < global_map_.data.size(); i++) {
        occupancy_buffer_[i] = global_map_.data[i];
    }
    ROS_INFO("RRT map set from PlanningServer: size=(%d,%d), resolution=%.3f",
             map_size_.x(), map_size_.y(), resolution_);
}

int RRT::search(const Eigen::Vector2d& start_pos, const Eigen::Vector2d& goal_pos) {
    if (!isInMap(start_pos) || !isInMap(goal_pos)) {
        Eigen::Vector2i start_idx, goal_idx;
        posToIndex(start_pos, start_idx);
        posToIndex(goal_pos, goal_idx);
        ROS_WARN("RRT Start or goal position out of map boundary! map_topic=%s, "
                 "start=(%.3f, %.3f) idx=(%d, %d), goal=(%.3f, %.3f) idx=(%d, %d), "
                 "map_origin=(%.3f, %.3f), map_max=(%.3f, %.3f), size=(%d, %d), resolution=%.3f",
                 map_topic_.c_str(),
                 start_pos.x(), start_pos.y(), start_idx.x(), start_idx.y(),
                 goal_pos.x(), goal_pos.y(), goal_idx.x(), goal_idx.y(),
                 map_origin_.x(), map_origin_.y(),
                 map_origin_.x() + map_size_.x() * resolution_,
                 map_origin_.y() + map_size_.y() * resolution_,
                 map_size_.x(), map_size_.y(), resolution_);
        return NO_PATH;
    }

    if (isOccupied(start_pos) || isOccupied(goal_pos)) {
        ROS_WARN("Start or goal position is occupied!");
        return NO_PATH;
    }

    reset();
    ros::Time t1 = ros::Time::now();

    // 初始化根节点
    root_ = new Node;
    root_->position = start_pos;
    root_->cost = 0.0;
    nodes_.push_back(root_);

    // 预分配节点内存
    node_pool_.reserve(max_iterations_);

    for (int iter = 0; iter < max_iterations_; ++iter) {
        // 检查超时
        if ((ros::Time::now() - t1).toSec() * 1000 > max_search_time_) {
            ROS_WARN("RRT: Reach max search time");
            return NO_PATH; // 超时但可能有路径
        }

        // 随机采样 (带目标偏向)
        Eigen::Vector2d random_point;
        if (bias_dist_(gen_) < goal_bias_) {
            random_point = goal_pos; // 偏向目标点
        } else {
            random_point = Eigen::Vector2d(x_dist_(gen_), y_dist_(gen_));
        }

        // 找到最近的节点
        Node* nearest_node = findNearestNode(random_point);
        if (!nearest_node) continue;

        // 向随机点方向扩展
        Eigen::Vector2d new_point = steer(nearest_node->position, random_point);
        if (!isInMap(new_point)) continue;

        // 检查路径是否无碰撞
        if (!isPathCollisionFree(nearest_node->position, new_point)) {
            continue;
        }

        // 创建新节点
        Node* new_node = new Node;
        new_node->position = new_point;
        new_node->parent = nearest_node;
        new_node->cost = nearest_node->cost + getDistance(nearest_node->position, new_point);
        
        // 添加到树中
        nearest_node->children.push_back(new_node);
        nodes_.push_back(new_node);
        node_pool_.push_back(new_node);

        // 重布线优化
        rewire(new_node, search_radius_);

        // 检查是否到达目标
        if (getDistance(new_point, goal_pos) < step_size_) {
            if (isPathCollisionFree(new_point, goal_pos)) {
                // 创建目标节点
                ros::Time t2 = ros::Time::now();
                std::cout<<"RRT search time: "<<(t2-t1).toSec() * 1000 <<" ms"<<std::endl;
                cout << "RRT iter: " << iter << endl;

                Node* goal_node = new Node;
                goal_node->position = goal_pos;
                goal_node->parent = new_node;
                goal_node->cost = new_node->cost + getDistance(new_node->position, goal_pos);
                
                new_node->children.push_back(goal_node);
                nodes_.push_back(goal_node);
                node_pool_.push_back(goal_node);

                // 提取路径
                retrievePath(goal_node);

                double original_length = 0.0;
				std::vector<double> curvatures;
				std::vector<double> distance;
				for(int i = 1; i < final_path_.size(); i++) {
					original_length += (final_path_[i] - final_path_[i-1]).norm();
				}
				for(size_t i = 1; i < final_path_.size()-1; i++) {
				Eigen::Vector2d p0 = final_path_[i-1];
				Eigen::Vector2d p1 = final_path_[i];
				Eigen::Vector2d p2 = final_path_[i+1];
				
				double dx1 = p1.x() - p0.x();
				double dy1 = p1.y() - p0.y();
				double dx2 = p2.x() - p1.x();
				double dy2 = p2.y() - p1.y();
				
				double denom = pow(dx1 * dx1 + dy1 * dy1, 1.5);
				if(denom < 1e-6) {
					curvatures.push_back(0.0);
					continue;
				}
				
				double curvature = (dx1 * dy2 - dx2 * dy1) / denom;
				curvatures.push_back(abs(curvature));
				}
				cout << "rrt original_length: " << original_length << endl;
				double avg_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0) / curvatures.size();
				double max_curvature = *std::max_element(curvatures.begin(), curvatures.end());
				cout << "rrt original_avg_curvature: " << avg_curvature << " rrt original_max_curvature: " << max_curvature << endl;
                return REACH_END; // 成功找到路径
            }
        }
    }

    ROS_WARN("RRT: Reach max iterations without finding path");
    return NO_PATH; // 达到最大迭代次数但可能有路径
}

RRT::Node* RRT::getRandomNode() {
    Eigen::Vector2d random_point(x_dist_(gen_), y_dist_(gen_));
    Node* node = new Node;
    node->position = random_point;
    return node;
}

RRT::Node* RRT::findNearestNode(const Eigen::Vector2d& point) {
    if (nodes_.empty()) return nullptr;

    Node* nearest = nodes_[0];
    double min_dist = getDistance(point, nearest->position);

    for (auto node : nodes_) {
        double dist = getDistance(point, node->position);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }

    return nearest;
}

Eigen::Vector2d RRT::steer(const Eigen::Vector2d& from, const Eigen::Vector2d& to) {
    Eigen::Vector2d direction = to - from;
    double dist = direction.norm();
    direction.normalize();
    
    if (dist <= step_size_) {
        return to;
    } else {
        return from + direction * step_size_;
    }
}

bool RRT::isPathCollisionFree(const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
    Eigen::Vector2d diff = end - start;
    double length = diff.norm();
    diff.normalize();
    
    int steps = static_cast<int>(length / resolution_);
    for (int i = 0; i <= steps; ++i) {
        Eigen::Vector2d pt = start + diff * i * resolution_;
        if (isOccupied(pt)) {
            return false;
        }
    }
    
    // 检查终点
    return !isOccupied(end);
}

void RRT::rewire(Node* new_node, double radius) {
    for (auto node : nodes_) {
        if (node == new_node || node == new_node->parent) continue;
        
        double dist = getDistance(new_node->position, node->position);
        if (dist > radius) continue;
        
        if (new_node->cost + dist < node->cost) {
            if (isPathCollisionFree(new_node->position, node->position)) {
                // 从原父节点中移除
                if (node->parent) {
                    auto& siblings = node->parent->children;
                    siblings.erase(std::remove(siblings.begin(), siblings.end(), node), siblings.end());
                }
                
                // 更新父节点和代价
                node->parent = new_node;
                node->cost = new_node->cost + dist;
                new_node->children.push_back(node);
                
                // 递归更新子节点代价
                std::vector<Node*> queue = {node};
                while (!queue.empty()) {
                    Node* current = queue.back();
                    queue.pop_back();
                    
                    for (auto child : current->children) {
                        double new_cost = current->cost + getDistance(current->position, child->position);
                        if (new_cost < child->cost) {
                            child->cost = new_cost;
                            queue.push_back(child);
                        }
                    }
                }
            }
        }
    }
}

void RRT::retrievePath(Node* end_node) {
    final_path_.clear();
    Node* current = end_node;
    
    while (current != nullptr) {
        final_path_.push_back(current->position);
        current = current->parent;
    }
    
    std::reverse(final_path_.begin(), final_path_.end());
}

void RRT::reset() {
    // 释放所有节点内存
    for (auto node : node_pool_) {
        delete node;
    }
    
    nodes_.clear();
    node_pool_.clear();
    final_path_.clear();
    root_ = nullptr;
}

bool RRT::isInMap(const Eigen::Vector2d& pos) {
    Eigen::Vector2i index;
    posToIndex(pos, index);
    return index.x() >= 0 && index.x() < map_size_.x() && 
           index.y() >= 0 && index.y() < map_size_.y();
}

bool RRT::isOccupied(const Eigen::Vector2d& pos) {
    Eigen::Vector2i index;
    posToIndex(pos, index);
    if (!isInMap(pos)) return true;
    const int state = occupancy_buffer_[index.y() * map_size_.x() + index.x()];
    return (unknown_as_occupied_ && state < 0) || state >= occupied_threshold_;
}

void RRT::posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& index) {
    index = Eigen::Vector2i(
        floor((pos.x() - map_origin_.x()) * inv_resolution_),
        floor((pos.y() - map_origin_.y()) * inv_resolution_)
    );
}

void RRT::indexToPos(const Eigen::Vector2i& index, Eigen::Vector2d& pos) {
    pos = Eigen::Vector2d(
        index.x() * resolution_ + map_origin_.x(),
        index.y() * resolution_ + map_origin_.y()
    );
}

double RRT::getDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    return (p2 - p1).norm();
}

} // namespace path_searching
