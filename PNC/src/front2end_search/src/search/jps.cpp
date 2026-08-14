#include <path_searching/jps.h>

#include <algorithm>
#include <iostream>
#include <numeric>

#include <visualization_msgs/Marker.h>

using namespace std;
using namespace Eigen;

namespace path_searching
{
    JPS::JPS()
	{
		
	}

	JPS::~JPS()
	{
		for (int i = 0; i < allocate_num_; i++)
		{
			delete path_node_pool_[i];
		}
	}

	    void JPS::setMap(const nav_msgs::OccupancyGrid& map)
		{
			globalMap_ = map;
			resolution_ = globalMap_.info.resolution;
			inv_resolution_ = 1 / resolution_;
			global_map_size_[0] = globalMap_.info.width;
			global_map_size_[1] = globalMap_.info.height;
			map_origin_[0] = globalMap_.info.origin.position.x;
			map_origin_[1] = globalMap_.info.origin.position.y;

			int buffer_size_2d_ = global_map_size_[0] * global_map_size_[1];
			occupancy_buffer_2d_.resize(buffer_size_2d_);
   for(size_t i = 0; i < globalMap_.data.size(); i++)
			{
				occupancy_buffer_2d_[i] = globalMap_.data[i];
			}
			ROS_INFO("JPS map set from PlanningServer: size=(%d,%d), resolution=%.3f",
			         global_map_size_[0], global_map_size_[1], resolution_);
		}

	    void JPS::init(ros::NodeHandle& nh)
	{
		nh_ = nh;
		nh_.param("jps/lambda_heu", lambda_heu_, 1.0);
		nh_.param("jps/allocate_num", allocate_num_, 500000);
		nh_.param("jps/max_search_time", max_seach_time, 5000.1);
		nh_.param("jps/occupied_threshold", occupied_threshold_, 50);
		nh_.param("jps/obstacle_cost_weight", obstacle_cost_weight_, 5.0);
		nh_.param("jps/unknown_as_occupied", unknown_as_occupied_, true);

		ros::NodeHandle private_nh("~");
		private_nh.param<std::string>("search/map_topic", map_topic_, "/projected_map");
		open_set_pub_ = nh.advertise<visualization_msgs::Marker>("/jps/open_set", 10);
		close_set_pub_ = nh.advertise<visualization_msgs::Marker>("/jps/close_set", 10);
		path_pub_ = nh.advertise<visualization_msgs::Marker>("/jps/path", 10);
		jump_point_pub_ = nh.advertise<visualization_msgs::Marker>("/jps/jump_points", 10);

		/* ---------- pre-allocated node ---------- */
		path_node_pool_.resize(allocate_num_);
		for (int i = 0; i < allocate_num_; i++)
		{
			path_node_pool_[i] = new JPSNode;
		}
		path_nodes_.clear();
		use_node_num_ = 0;
		iter_num_ = 0;

		inv_resolution_ = 1.0 / resolution_;

	}

	void JPS::visualizeOpenSet() {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "jps";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = marker.scale.y = resolution_ * 0.5;
		
		marker.color.r = OPEN_SET_COLOR[0];
		marker.color.g = OPEN_SET_COLOR[1];
		marker.color.b = OPEN_SET_COLOR[2];
		marker.color.a = OPEN_SET_COLOR[3];
		
		// 复制open_set内容到临时队列
		auto temp_queue = open_set_;
		while (!temp_queue.empty()) {
			auto node = temp_queue.top();
			temp_queue.pop();
			
			geometry_msgs::Point p;
			p.x = node->state.x();
			p.y = node->state.y();
			p.z = 0.1; // 稍微抬高避免与地图重叠
			marker.points.push_back(p);
		}
		
		open_set_pub_.publish(marker);
	}
	
	void JPS::visualizeCloseSet() {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "jps";
		marker.id = 1;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = marker.scale.y = resolution_ * 0.5;
		
		marker.color.r = CLOSE_SET_COLOR[0];
		marker.color.g = CLOSE_SET_COLOR[1];
		marker.color.b = CLOSE_SET_COLOR[2];
		marker.color.a = CLOSE_SET_COLOR[3];
		
		for (int i = 0; i < use_node_num_; ++i) {
			if (path_node_pool_[i]->node_state == IN_CLOSE_SET) {
				geometry_msgs::Point p;
				p.x = path_node_pool_[i]->state.x();
				p.y = path_node_pool_[i]->state.y();
				p.z = 0.1;
				marker.points.push_back(p);
			}
		}
		
		close_set_pub_.publish(marker);
	}
	
	void JPS::visualizePath(const std::vector<Eigen::Vector2d>& path) {
		if (path.empty()) return;
		
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "jps";
		marker.id = 2;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = resolution_ * 0.3;
		
		marker.color.r = PATH_COLOR[0];
		marker.color.g = PATH_COLOR[1];
		marker.color.b = PATH_COLOR[2];
		marker.color.a = PATH_COLOR[3];
		
		for (const auto& point : path) {
			geometry_msgs::Point p;
			p.x = point.x();
			p.y = point.y();
			p.z = 0.2; // 抬高避免与其他可视化重叠
			marker.points.push_back(p);
		}
		
		path_pub_.publish(marker);
	}
	
	void JPS::visualizeJumpPoints(const std::vector<Eigen::Vector2i>& jump_points) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "jps";
		marker.id = 3;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = marker.scale.y = resolution_ * 0.8;
		
		marker.color.r = JUMP_POINT_COLOR[0];
		marker.color.g = JUMP_POINT_COLOR[1];
		marker.color.b = JUMP_POINT_COLOR[2];
		marker.color.a = JUMP_POINT_COLOR[3];
		
		for (const auto& jp : jump_points) {
			Eigen::Vector2d pos;
			indexToPos2d(jp, pos);
			
			geometry_msgs::Point p;
			p.x = pos.x();
			p.y = pos.y();
			p.z = 0.15;
			marker.points.push_back(p);
		}
		
		jump_point_pub_.publish(marker);
	}

	bool JPS::isInMap2d(const Eigen::Vector2d &pos)
	{
		Eigen::Vector2i idx;
		posToIndex2d(pos, idx);
		return isInMap2d(idx);
	}

	bool JPS::isInMap2d(const Eigen::Vector2i &id)
	{
		if(id(0) < 0 || id(0) >= global_map_size_(0) || id(1) < 0 || id(1) >= global_map_size_(1))
		{
			return false;
		}
		else
			return true;
	};

	void JPS::posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id)
	{
		for(int i = 0; i < 2; i++)
		{
			id(i) = floor((pos(i) - map_origin_(i)) * inv_resolution_);
		}
	}

	void JPS::indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
	{
		for(int i = 0; i < 2; i++)
		{
			pos(i) = id(i) * resolution_ + map_origin_(i);
		}
	}

	int JPS::getVoxelState2d(const Eigen::Vector2d &pos)
	{
		Eigen::Vector2i id;
		posToIndex2d(pos, id);
		if(!isInMap2d(id))
			return -1;
		// todo: add local map range

		return occupancy_buffer_2d_[id(1) * global_map_size_(0) + id(0)];
	}

	bool JPS::isOccupied(const Eigen::Vector2d& pos)
	{
		const int state = getVoxelState2d(pos);
		return (unknown_as_occupied_ && state < 0) || state >= occupied_threshold_;
	}

	bool JPS::isOccupiedindex(const Eigen::Vector2i& id)
	{
		Eigen::Vector2d pos;
		indexToPos2d(id, pos);
		const int state = getVoxelState2d(pos);
		return (unknown_as_occupied_ && state < 0) || state >= occupied_threshold_;
	}

	double JPS::getTraversalCost(const Eigen::Vector2i& start,
	                             const Eigen::Vector2i& end) const
	{
		const Eigen::Vector2i delta = end - start;
		const int steps = std::max(std::abs(delta.x()), std::abs(delta.y()));
		if (steps == 0) {
			return 0.0;
		}

		const Eigen::Vector2i direction(
			(delta.x() > 0) - (delta.x() < 0),
			(delta.y() > 0) - (delta.y() < 0));
		const double step_length =
			(direction.x() == 0 || direction.y() == 0)
			? resolution_ : resolution_ * std::sqrt(2.0);
		const double max_soft_cost = std::max(1, occupied_threshold_ - 1);

		double cost = 0.0;
		Eigen::Vector2i index = start;
		for (int i = 0; i < steps; ++i) {
			index += direction;
			const int cell_cost = occupancy_buffer_2d_[
				index.y() * global_map_size_.x() + index.x()];
			const double normalized_cost =
				std::max(0, cell_cost) / max_soft_cost;
			cost += step_length * (1.0 + obstacle_cost_weight_
				* normalized_cost * normalized_cost);
		}
		return cost;
	}

    int JPS::search(Eigen::Vector2d& start_pos, Eigen::Vector2d& goal_pos) {
		// ros::Rate vis_rate(10); // 10Hz
    	// int vis_counter = 0;

        if (!isInMap2d(start_pos) || !isInMap2d(goal_pos)) {
			Eigen::Vector2i start_idx, goal_idx;
			posToIndex2d(start_pos, start_idx);
			posToIndex2d(goal_pos, goal_idx);
			ROS_WARN("JPS Start or goal position out of map boundary! map_topic=%s, "
			         "start=(%.3f, %.3f) idx=(%d, %d), goal=(%.3f, %.3f) idx=(%d, %d), "
			         "map_origin=(%.3f, %.3f), map_max=(%.3f, %.3f), size=(%d, %d), resolution=%.3f",
			         map_topic_.c_str(),
			         start_pos.x(), start_pos.y(), start_idx.x(), start_idx.y(),
			         goal_pos.x(), goal_pos.y(), goal_idx.x(), goal_idx.y(),
			         map_origin_.x(), map_origin_.y(),
			         map_origin_.x() + global_map_size_.x() * resolution_,
			         map_origin_.y() + global_map_size_.y() * resolution_,
			         global_map_size_.x(), global_map_size_.y(), resolution_);
			return NO_PATH;
		}

		if (isOccupied(goal_pos)) {
			ROS_WARN("Goal position is occupied!");
			return NO_PATH;
		}

		goal_pos_ =  goal_pos;
		// 计时，若超时则不予继续搜索
        ros::Time t1 = ros::Time::now();
		JPSNodePtr cur_node = path_node_pool_[0];
        cur_node->parent = NULL;
    	cur_node->state = start_pos;
    	posToIndex2d(start_pos, cur_node->index);
		cur_node->g_score = 0.0;
		cur_node->f_score = lambda_heu_ * getHeu(cur_node->state, goal_pos);
		cur_node->node_state = IN_OPEN_SET;
		cur_node->number = 0;
		open_set_.push(cur_node);

		use_node_num_ += 1;

        expanded_nodes_.insert(cur_node->index, cur_node);

        JPSNodePtr terminate_node = nullptr;
        while (!open_set_.empty())
		{
			
            cur_node = open_set_.top();
			// if(vis_counter++ % 10 == 0)
			// {
			// 	visualizeOpenSet();
            // 	visualizeCloseSet();

			// 	// 可视化当前路径
			// 	std::vector<Eigen::Vector2d> current_path;
			// 	JPSNodePtr node = cur_node;
			// 	while (node != nullptr) {
			// 		current_path.push_back(node->state);
			// 		node = node->parent;
			// 	}
			// 	std::reverse(current_path.begin(), current_path.end());
			// 	visualizePath(current_path);
				
			// 	// 可视化跳点
			// 	std::vector<Eigen::Vector2i> jump_points;
			// 	for (int i = 0; i < use_node_num_; ++i) {
			// 		if (path_node_pool_[i]->node_state != NOT_EXPAND) {
			// 			jump_points.push_back(path_node_pool_[i]->index);
			// 		}
			// 	}
			// 	visualizeJumpPoints(jump_points);
				
			// 	vis_rate.sleep();
			// }
			if ((cur_node->state - goal_pos).norm() < resolution_) {
				final_path_.clear();
				path_nodes_.clear();
				ros::Time t3 = ros::Time::now();
				double searchTime = (t3 - t1).toSec() * 1000;
				cout << "JPS searchTime: " << searchTime << "   iter_num_: " << iter_num_ << endl;
				terminate_node = cur_node;
				retrievePath(terminate_node);
				ConvertNodePathToPointPath(path_nodes_);

				double original_length = 0.0;
				std::vector<double> curvatures;
				std::vector<double> distance;
    for(size_t i = 1; i < final_path_.size(); i++) {
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
				cout << "jps original_length: " << original_length << endl;
				double avg_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0) / curvatures.size();
				double max_curvature = *std::max_element(curvatures.begin(), curvatures.end());
				cout << "jps original_avg_curvature: " << avg_curvature << " jps original_max_curvature: " << max_curvature << endl;
				return REACH_END;
			}

            ros::Time t2 = ros::Time::now();
      		double runTime = (t2 - t1).toSec() * 1000;

			if(runTime > max_seach_time){
				ROS_WARN("JPS: Reach the max search time %.2f ms", max_seach_time);
          		return NO_PATH;
			}
			cur_node->node_state = IN_CLOSE_SET;
			open_set_.pop();
			iter_num_ += 1;

            std::vector<Eigen::Vector2i> directions;
        
            // 确定搜索方向
            if (cur_node->parent) {
                Eigen::Vector2i dir = cur_node->index - cur_node->parent->index;
                dir = dir.array().sign().cast<int>(); // 标准化方向
                
                if (dir.x() != 0 && dir.y() != 0) {
                    // 对角线移动
                    directions.push_back(Eigen::Vector2i(dir.x(), 0));
                    directions.push_back(Eigen::Vector2i(0, dir.y()));
                    directions.push_back(dir);
                } else {
                    // 直线移动
                    if (dir.x() == 0) {
                        // 垂直移动
                        directions.push_back(Eigen::Vector2i(1, dir.y()));
                        directions.push_back(Eigen::Vector2i(-1, dir.y()));
                        directions.push_back(dir);
                    } else {
                        // 水平移动
                        directions.push_back(Eigen::Vector2i(dir.x(), 1));
                        directions.push_back(Eigen::Vector2i(dir.x(), -1));
                        directions.push_back(dir);
                    }
                }
            } else {
                // 起点情况，尝试所有方向
                directions = directions_;
            }
        
            // 在每个方向上跳跃
            for (const auto& dir : directions) {
				// if(abs(dir.x()) == abs(dir.y())) continue;
                Eigen::Vector2i jump_point = jump(cur_node->index, dir);
                
                if (jump_point.x() == -1 && jump_point.y() == -1) continue;
                
                if (!isInMap2d(jump_point) || isOccupiedindex(jump_point)) continue;
                
                // 计算代价
                Eigen::Vector2d jump_pos;
                indexToPos2d(jump_point, jump_pos);
                double move_cost = getTraversalCost(cur_node->index, jump_point);
                double tmp_g_score = cur_node->g_score + move_cost;
                double tmp_f_score = tmp_g_score + lambda_heu_ * getHeu(jump_pos, goal_pos);
                
                // 检查节点是否已存在
                JPSNodePtr neighbor;
                neighbor = expanded_nodes_.find(jump_point);

                if (neighbor != NULL && neighbor->node_state == IN_CLOSE_SET)
                {
                    continue;
                }
                
                if (neighbor == nullptr) {
                    // 创建新节点时才消耗节点池；已有节点更新不需要新内存。
                    if (use_node_num_ >= allocate_num_) {
                        ROS_WARN("JPS run out of nodes: used=%d, allocate_num=%d, map_cells=%d, iter=%d",
                                 use_node_num_, allocate_num_,
                                 global_map_size_[0] * global_map_size_[1], iter_num_);
                        return NO_PATH;
                    }
                    neighbor = path_node_pool_[use_node_num_++];
                    neighbor->parent = cur_node;
                    neighbor->index = jump_point;
                    neighbor->state = jump_pos;
                    neighbor->g_score = tmp_g_score;
                    neighbor->f_score = tmp_f_score;
                    neighbor->node_state = IN_OPEN_SET;
                    neighbor->number = cur_node->number + 1;
                    
                    open_set_.push(neighbor);
                    expanded_nodes_.insert(jump_point, neighbor);
                } 
                else if(neighbor->node_state == IN_OPEN_SET){
                    if (tmp_g_score < neighbor->g_score) {
                        neighbor->index = jump_point;
                        neighbor->state = jump_pos;
                        neighbor->parent = cur_node;
                        neighbor->g_score = tmp_g_score;
                        neighbor->f_score = tmp_f_score;
                        neighbor->number = cur_node->number + 1;
                    }
                    // if (neighbor->node_state == 2) { // IN_CLOSE_SET
                    //     neighbor->node_state = 1; // IN_OPEN_SET
                    //     open_set_.push(neighbor);
                    // }
                }
                else
                {
                    cout << "error type in searching: " << neighbor->node_state << endl;
                }
            }
        }
        ROS_WARN("Open set empty, no path found!");
        return NO_PATH;
    }

    Eigen::Vector2i JPS::jump(const Eigen::Vector2i& current, const Eigen::Vector2i& direction) {
        Eigen::Vector2i next = current + direction;
        const int max_steps = global_map_size_.x() + global_map_size_.y();
        int steps = 0;

        while (steps++ < max_steps) {
            // 检查是否超出地图或遇到障碍
            if (!isInMap2d(next) || isOccupiedindex(next)) {
                return Eigen::Vector2i(-1, -1);
            }

            // 检查是否是目标点
            Eigen::Vector2d next_pos;
            indexToPos2d(next, next_pos);
            if ((next_pos - goal_pos_).norm() < resolution_) {
                return next;
            }

            // 检查强制邻居
            std::vector<Eigen::Vector2i> forced_neighbors;
            getForcedNeighbors(next, direction, forced_neighbors);
            if (!forced_neighbors.empty()) {
                return next;
            }

            // 对角移动时，水平或垂直方向存在跳点，则当前点也是跳点。
            if (direction.x() != 0 && direction.y() != 0) {
                if (jump(next, Eigen::Vector2i(direction.x(), 0)).x() != -1 ||
                    jump(next, Eigen::Vector2i(0, direction.y())).x() != -1) {
                    return next;
                }
            }

            next += direction;
        }

        return Eigen::Vector2i(-1, -1);
    }
    
    void JPS::getForcedNeighbors(const Eigen::Vector2i& current, 
                               const Eigen::Vector2i& direction,
                               std::vector<Eigen::Vector2i>& neighbors) {
        neighbors.clear();
        
        if (direction.x() != 0 && direction.y() != 0) {
            // 对角线移动
            bool natural1 = !isOccupiedindex(current + Eigen::Vector2i(0, direction.y()));
            bool natural2 = !isOccupiedindex(current + Eigen::Vector2i(direction.x(), 0));
            
            if (natural1 && isOccupiedindex(current - Eigen::Vector2i(direction.x(), 0))) {
                neighbors.push_back(current + Eigen::Vector2i(-direction.x(), direction.y()));
            }
            
            if (natural2 && isOccupiedindex(current - Eigen::Vector2i(0, direction.y()))) {
                neighbors.push_back(current + Eigen::Vector2i(direction.x(), -direction.y()));
            }
        } else {
            // 直线移动
            if (direction.x() == 0) {
                // 垂直移动
                if (isOccupiedindex(current + Eigen::Vector2i(1, 0)) && 
                    !isOccupiedindex(current + Eigen::Vector2i(1, direction.y()))) {
                    neighbors.push_back(current + Eigen::Vector2i(1, direction.y()));
                }
                
                if (isOccupiedindex(current + Eigen::Vector2i(-1, 0)) && 
                    !isOccupiedindex(current + Eigen::Vector2i(-1, direction.y()))) {
                    neighbors.push_back(current + Eigen::Vector2i(-1, direction.y()));
                }
            } else {
                // 水平移动
                if (isOccupiedindex(current + Eigen::Vector2i(0, 1)) && 
                    !isOccupiedindex(current + Eigen::Vector2i(direction.x(), 1))) {
                    neighbors.push_back(current + Eigen::Vector2i(direction.x(), 1));
                }
                
                if (isOccupiedindex(current + Eigen::Vector2i(0, -1)) && 
                    !isOccupiedindex(current + Eigen::Vector2i(direction.x(), -1))) {
                    neighbors.push_back(current + Eigen::Vector2i(direction.x(), -1));
                }
            }
        }
    }
    
    bool JPS::hasLineOfSight(const Eigen::Vector2i& start, const Eigen::Vector2i& end) {
        Eigen::Vector2i diff = end - start;
        int steps = std::max(abs(diff.x()), abs(diff.y()));
        
        for (int i = 1; i <= steps; ++i) {
            Eigen::Vector2i pt = start + diff * i / steps;
            if (isOccupiedindex(pt)) return false;
        }
        
        return true;
    }
    
    void JPS::ConvertNodePathToPointPath(vector<JPSNodePtr> path_nodes_)
	{
  for(size_t i = 0; i < path_nodes_.size(); i++)
		{
			Eigen::Vector2d pos = path_nodes_[i]->state.head(2);
			final_path_.push_back(pos);
		}
	}

	void JPS::reset()
	{
		expanded_nodes_.clear();
		path_nodes_.clear();
		final_path_.clear();

		std::priority_queue<JPSNodePtr, std::vector<JPSNodePtr>, JPSNodeComparator> empty_queue;
		open_set_.swap(empty_queue);

		for (int i = 0; i < use_node_num_; i++)
		{
			JPSNodePtr node = path_node_pool_[i];
			node->parent = NULL;
			node->node_state = NOT_EXPAND;
		}
	
		use_node_num_ = 0;
		iter_num_ = 0;
	}

	void JPS::retrievePath(JPSNodePtr end_node)
	{
		JPSNodePtr cur_node = end_node;
		path_nodes_.push_back(cur_node);
	
		while (cur_node->parent != NULL)
		{
			cur_node = cur_node->parent;
			path_nodes_.push_back(cur_node);
		}
	
		reverse(path_nodes_.begin(), path_nodes_.end());
	}

}
