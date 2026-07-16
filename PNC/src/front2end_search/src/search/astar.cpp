#include <path_searching/astar.h>

#include <iostream>
#include <numeric>

using namespace std;
using namespace Eigen;

namespace path_searching
{
	Astar::Astar()
	{
		
	}

	Astar::~Astar()
	{
		for (int i = 0; i < allocate_num_; i++)
		{
			delete path_node_pool_[i];
		}
	}

		void Astar::setMap(const nav_msgs::OccupancyGrid& map)
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
			ROS_INFO("Astar map set from PlanningServer: size=(%d,%d), resolution=%.3f",
			         global_map_size_[0], global_map_size_[1], resolution_);
		}

	void Astar::init(ros::NodeHandle& nh)
	{
		nh_ = nh;
		nh_.param("search/lambda_heu", lambda_heu_, 1.0001);
		nh_.param("search/allocate_num", allocate_num_, 500000);
		nh_.param("search/max_search_time", max_seach_time, 5000.1);
		nh_.param("search/use_search_window", use_search_window_, true);
		nh_.param("search/search_window_margin", search_window_margin_, 20.0);
		nh_.param("search/occupied_threshold", occupied_threshold_, 99);
		nh_.param("search/obstacle_cost_weight", obstacle_cost_weight_, 5.0);
		nh_.param("search/unknown_as_occupied", unknown_as_occupied_, true);

			nh_.param<std::string>("search/map_topic", map_topic_, "/projected_map");
			ROS_INFO("Astar map_topic=%s, occupied_threshold=%d, obstacle_cost_weight=%.3f, unknown_as_occupied=%d",
			         map_topic_.c_str(), occupied_threshold_, obstacle_cost_weight_,
			         static_cast<int>(unknown_as_occupied_));

		/* ---------- pre-allocated node ---------- */
		path_node_pool_.resize(allocate_num_);
		for (int i = 0; i < allocate_num_; i++)
		{
			path_node_pool_[i] = new AstarNode;
		}
		path_nodes_.clear();
		use_node_num_ = 0;
		iter_num_ = 0;

		inv_resolution_ = 1.0 / resolution_;

		// expandNodesVis = nh.advertise<sensor_msgs::PointCloud2>("/vis/expanded_nodes", 1);
	}

	bool Astar::isInMap2d(const Eigen::Vector2d &pos)
	{
		Eigen::Vector2i idx;
		posToIndex2d(pos, idx);
		return isInMap2d(idx);
	}

	bool Astar::isInMap2d(const Eigen::Vector2i &id)
	{
		if(id(0) < 0 || id(0) >= global_map_size_(0) || id(1) < 0 || id(1) >= global_map_size_(1))
		{
			return false;
		}
		else
			return true;
	};

	bool Astar::isInSearchWindow(const Eigen::Vector2i &id)
	{
		if (!use_search_window_) {
			return true;
		}
		return id(0) >= search_min_idx_(0) && id(0) <= search_max_idx_(0) &&
		       id(1) >= search_min_idx_(1) && id(1) <= search_max_idx_(1);
	}

	void Astar::posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id)
	{
		for(int i = 0; i < 2; i++)
		{
			id(i) = floor((pos(i) - map_origin_(i)) * inv_resolution_);
		}
	}

	void Astar::indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
	{
		for(int i = 0; i < 2; i++)
		{
			pos(i) = id(i) * resolution_ + map_origin_(i);
		}
	}

	int Astar::getVoxelState2d(const Eigen::Vector2d &pos)
	{
		Eigen::Vector2i id;
		posToIndex2d(pos, id);
		if(!isInMap2d(id))
			return -1;
		// todo: add local map range

		return occupancy_buffer_2d_[id(1) * global_map_size_(0) + id(0)];
	}

	bool Astar::isOccupied(const Eigen::Vector2d& pos)
	{
		const int state = getVoxelState2d(pos);
		return (unknown_as_occupied_ && state < 0) || state >= occupied_threshold_;
	}

	bool Astar::isOccupied(const Eigen::Vector2i& id)
	{
		Eigen::Vector2d pos;
		indexToPos2d(id, pos);
		const int state = getVoxelState2d(pos);
		return (unknown_as_occupied_ && state < 0) || state >= occupied_threshold_;
	}

	int Astar::search(Eigen::Vector2d& start_pos, Eigen::Vector2d& goal_pos)
	{
		if (!isInMap2d(start_pos) || !isInMap2d(goal_pos)) {
			Eigen::Vector2i start_idx, goal_idx;
			posToIndex2d(start_pos, start_idx);
			posToIndex2d(goal_pos, goal_idx);
			ROS_WARN("A star Start or goal position out of map boundary! map_topic=%s, "
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

		Eigen::Vector2i start_idx, goal_idx;
		posToIndex2d(start_pos, start_idx);
		posToIndex2d(goal_pos, goal_idx);
		const int margin_cells = std::max(1, static_cast<int>(std::ceil(search_window_margin_ * inv_resolution_)));
		search_min_idx_ = start_idx.cwiseMin(goal_idx) - Eigen::Vector2i(margin_cells, margin_cells);
		search_max_idx_ = start_idx.cwiseMax(goal_idx) + Eigen::Vector2i(margin_cells, margin_cells);
		search_min_idx_ = search_min_idx_.cwiseMax(Eigen::Vector2i(0, 0));
		search_max_idx_ = search_max_idx_.cwiseMin(global_map_size_ - Eigen::Vector2i(1, 1));
		const int window_cells = (search_max_idx_(0) - search_min_idx_(0) + 1) *
		                         (search_max_idx_(1) - search_min_idx_(1) + 1);
		if (use_search_window_) {
			ROS_INFO("Astar search window: min=(%d,%d), max=(%d,%d), cells=%d, margin=%.2fm",
			         search_min_idx_(0), search_min_idx_(1),
			         search_max_idx_(0), search_max_idx_(1),
			         window_cells, search_window_margin_);
		}
		
		ros::Time t1 = ros::Time::now();
		AstarNodePtr cur_node = path_node_pool_[0];
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

		AstarNodePtr terminate_node = nullptr;
		while (!open_set_.empty())
		{
			cur_node = open_set_.top();
			if ((cur_node->state - goal_pos).norm() < resolution_) {
				final_path_.clear();
				path_nodes_.clear();
				ros::Time t3 = ros::Time::now();
				double searchTime = (t3 - t1).toSec() * 1000;
				cout << "astar searchTime: " << searchTime << "   iter_num_: " << iter_num_ << endl;
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

				cout << "astar original_length: " << original_length << endl;
				double avg_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0) / curvatures.size();
				double max_curvature = *std::max_element(curvatures.begin(), curvatures.end());
				cout << "astar original_avg_curvature: " << avg_curvature << " astar original_max_curvature: " << max_curvature << endl;
				return REACH_END;
			}

			ros::Time t2 = ros::Time::now();
      		double runTime = (t2 - t1).toSec() * 1000;

			if(runTime > max_seach_time){
				ROS_WARN("AstarSearch: Reach the max seach time");
          		return REACH_END;
			}

			cur_node->node_state = IN_CLOSE_SET;
			open_set_.pop();
			iter_num_ += 1;

			for (int dx = -1; dx <= 1; dx++) {
				for (int dy = -1; dy <= 1; dy++) {
					if (dx == 0 && dy == 0) continue;  // 跳过当前节点
	
					Eigen::Vector2i pro_id = cur_node->index + Eigen::Vector2i(dx, dy);
					Eigen::Vector2d pro_state;
					indexToPos2d(pro_id, pro_state);
					// 检查邻居是否有效
					if (!isInMap2d(pro_id)) continue;
					if (!isInSearchWindow(pro_id)) continue;
					if (isOccupied(pro_id)) continue;
						// 计算移动代价，并将膨胀层栅格值作为软障碍代价。
						double move_cost = (dx == 0 || dy == 0) ? resolution_ : resolution_ * sqrt(2);
						const int cell_cost = getVoxelState2d(pro_state);
						const double max_soft_cost = std::max(1, occupied_threshold_ - 1);
						const double normalized_cost =
							std::max(0, cell_cost) / max_soft_cost;
						const double obstacle_cost = obstacle_cost_weight_
							* normalized_cost * normalized_cost * move_cost;
					// 检查是否已经探索过该邻居
					AstarNodePtr pro_node;
					pro_node = expanded_nodes_.find(pro_id);

					if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
					{
						continue;
					}

					double tmp_g_score = 0.0;
        			double tmp_f_score = 0.0;

						tmp_g_score = cur_node->g_score + move_cost + obstacle_cost;
					tmp_f_score = tmp_g_score + lambda_heu_ * getHeu(pro_state, goal_pos);

					if (pro_node == NULL)
        			{
						// 创建新节点时才消耗节点池；已有节点更新不需要新内存。
						if (use_node_num_ >= allocate_num_) {
							ROS_WARN("Astar run out of nodes: used=%d, allocate_num=%d, map_cells=%d, iter=%d, "
							         "use_window=%d, window_min=(%d,%d), window_max=(%d,%d)",
							         use_node_num_, allocate_num_,
							         global_map_size_[0] * global_map_size_[1], iter_num_,
							         static_cast<int>(use_search_window_),
							         search_min_idx_(0), search_min_idx_(1),
							         search_max_idx_(0), search_max_idx_(1));
							return NO_PATH;
						}
						pro_node = path_node_pool_[use_node_num_++];
						pro_node->parent = cur_node;
						pro_node->index = pro_id;
						pro_node->state = pro_state;
						pro_node->g_score = tmp_g_score;
						pro_node->f_score = tmp_f_score;
						pro_node->node_state = IN_OPEN_SET;
						pro_node->number = cur_node->number + 1;

						open_set_.push(pro_node);
						expanded_nodes_.insert(pro_id,pro_node);
					}
					else if (pro_node->node_state == IN_OPEN_SET)
					{
						if (tmp_g_score < pro_node->g_score)
						{
							pro_node->index = pro_id;
							pro_node->state = pro_state;
							pro_node->f_score = tmp_f_score;
							pro_node->g_score = tmp_g_score;
							pro_node->parent = cur_node;
							pro_node->number = cur_node->number + 1;
						}
					}
					else
					{
						cout << "error type in searching: " << pro_node->node_state << endl;
					}
				}
			}
		}
		ROS_WARN("Open set empty, no path found!");
    	return NO_PATH;
	}

	void Astar::ConvertNodePathToPointPath(vector<AstarNodePtr> path_nodes_)
	{
  for(size_t i = 0; i < path_nodes_.size(); i++)
		{
			Eigen::Vector2d pos = path_nodes_[i]->state.head(2);
			final_path_.push_back(pos);
		}
	}

	void Astar::reset()
	{
		expanded_nodes_.clear();
		path_nodes_.clear();
		final_path_.clear();

		std::priority_queue<AstarNodePtr, std::vector<AstarNodePtr>, AstarNodeComparator> empty_queue;
		open_set_.swap(empty_queue);

		for (int i = 0; i < use_node_num_; i++)
		{
			AstarNodePtr node = path_node_pool_[i];
			node->parent = NULL;
			node->node_state = NOT_EXPAND;
		}
	
		use_node_num_ = 0;
		iter_num_ = 0;
	}

	void Astar::retrievePath(AstarNodePtr end_node)
	{
		AstarNodePtr cur_node = end_node;
		path_nodes_.push_back(cur_node);
	
		while (cur_node->parent != NULL)
		{
			cur_node = cur_node->parent;
			path_nodes_.push_back(cur_node);
		}
	
		reverse(path_nodes_.begin(), path_nodes_.end());
	}
}
