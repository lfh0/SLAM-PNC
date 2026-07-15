#include <path_searching/kino_astar.h>

#include <iostream>
#include <numeric>

#include <ompl/geometric/SimpleSetup.h>

using namespace std;
using namespace Eigen;

namespace path_searching
{
  KinoAstar::KinoAstar()
  {
    
  }

  KinoAstar::~KinoAstar()
  {
    for (int i = 0; i < allocate_num_; i++)
    {
      delete path_node_pool_[i];
    }
  }

  void KinoAstar::setMap(const nav_msgs::OccupancyGrid& map)
  {
    globalMap_ = map;
    resolution_ = globalMap_.info.resolution;
    inv_resolution_ = 1.0 / resolution_;
    global_map_size_ = Eigen::Vector2i(globalMap_.info.width, globalMap_.info.height);
    map_origin_ = Eigen::Vector2d(
      globalMap_.info.origin.position.x,
      globalMap_.info.origin.position.y);

    occupancy_buffer_2d_.resize(globalMap_.data.size());
    for(size_t i = 0; i < globalMap_.data.size(); i++)
    {
      occupancy_buffer_2d_[i] = globalMap_.data[i];
    }
    ROS_INFO("KinoAstar map set from PlanningServer: size=(%d,%d), resolution=%.3f",
             global_map_size_.x(), global_map_size_.y(), resolution_);
  }

  void KinoAstar::init(ros::NodeHandle& nh)
  {
      nh_ = nh;
    
      nh_.param("search/horizon", horizon_, 50.0);
      nh_.param("search/yaw_resolution", yaw_resolution_, 0.3);
      nh_.param("search/lambda_heu", lambda_heu_, 1.5);
      nh_.param("search/allocate_num", allocate_num_, 500000);
      nh_.param("search/check_num", check_num_, 5);
      nh_.param("search/max_search_time", max_seach_time, 3000.1);
      nh_.param("search/occupied_threshold", occupied_threshold_, 50);
      nh_.param("search/obstacle_cost_weight", obstacle_cost_weight_, 5.0);
      nh_.param("search/unknown_as_occupied", unknown_as_occupied_, true);
      nh_.param("search/traj_forward_penalty", traj_forward_penalty, 1.0);
      nh_.param("search/traj_back_penalty", traj_back_penalty, 5.0);
      nh_.param("search/traj_gear_switch_penalty", traj_gear_switch_penalty, 0.0);
      nh_.param("search/traj_steer_penalty", traj_steer_penalty, 0.2);
      nh_.param("search/traj_steer_change_penalty", traj_steer_change_penalty, 0.0);
      nh_.param("search/step_arc", step_arc, 1.0);//2.0
      nh_.param("search/checkl", checkl, 0.2);

      nh_.param("vehicle/car_width", car_width_, 0.6);
      nh_.param("vehicle/car_length", car_length_, 1.0);
      nh_.param("vehicle/car_wheelbase", car_wheelbase_, 0.8);
      nh_.param("vehicle/car_front_suspension", car_front_suspension_, 0.93);
      nh_.param("vehicle/car_rear_suspension", car_rear_suspension_, 1.1);
      nh_.param("vehicle/car_max_steering_angle", car_max_steering_angle_, 45.0);
      nh_.param("vehicle/car_d_cr", car_d_cr_, 0.0);

      nh_.param<std::string>("search/map_topic", map_topic_, "/projected_map");

      /* ---------- pre-allocated node ---------- */
      path_node_pool_.resize(allocate_num_);
      middle_node_pool_.resize(allocate_num_);
      for (int i = 0; i < allocate_num_; i++)
      {
        path_node_pool_[i] = new PathNode;
        middle_node_pool_[i] = new MiddleNode;
      }
      path_nodes_.clear();
      use_node_num_ = 0;
      use_time_node_num_ = 0;
      iter_num_ = 0;

      
      nh_.param("search/max_vel", max_vel_, 0.5);
      nh_.param("search/max_acc", max_acc_, 0.3);
      nh_.param("search/max_cur", max_cur_, 0.3);
      nh_.param("vehicle/car_max_steering_angle", max_steer_, 45.0);

      nh_.param("search/time_resolution", time_resolution_, 0.1);
      nh_.param("search/distance_resolution", distance_resolution_, 0.5);
      nh_.param("search/velocity_resolution", velocity_resolution_, 0.5);
      max_steer_ = max_steer_ * M_PI / 180.0;

      min_vel_ = -max_vel_;
      min_acc_ = -max_acc_;

      inv_resolution_ = 1.0 / resolution_;
      inv_yaw_resolution_ = 1.0 / yaw_resolution_;

      shotptr =std::make_shared<ompl::base::ReedsSheppStateSpace>(1.0 / max_cur_);
      
      // non_siguav = 1.0e-2;
      non_siguav = 0.05;

      car_vertex_small_.clear();
      Eigen::Vector2d vertex_small;
      vertex_small << car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
      car_vertex_small_.push_back(vertex_small);
      vertex_small << car_length_ / 2.0 + car_d_cr_, -car_width_ / 2.0;
      car_vertex_small_.push_back(vertex_small);
      vertex_small << -car_length_ / 2.0 + car_d_cr_, -car_width_ / 2.0;
      car_vertex_small_.push_back(vertex_small);
      vertex_small << -car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
      car_vertex_small_.push_back(vertex_small);
      vertex_small << car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
      car_vertex_small_.push_back(vertex_small);

      // stores the vertexs of the car
      car_vertex_.clear();
      Eigen::Vector2d vertex;
      vertex << car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
      car_vertex_.push_back(vertex);
      vertex << car_length_ / 2.0 + car_d_cr_, -car_width_ / 2.0;
      car_vertex_.push_back(vertex);
      vertex << -car_length_ / 2.0 + car_d_cr_, -car_width_ / 2.0;
      car_vertex_.push_back(vertex);
      vertex << -car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
      car_vertex_.push_back(vertex);
      vertex << car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
      car_vertex_.push_back(vertex);

      car_length_ += 0.4;
      car_width_ += 0.8;

      car_vertex_big_.clear();
      Eigen::Vector2d vertex_big;
      vertex_big << car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
      car_vertex_big_.push_back(vertex_big);
      vertex_big << car_length_ / 2.0 + car_d_cr_, -car_width_ / 2.0;
      car_vertex_big_.push_back(vertex_big);
      vertex_big << -car_length_ / 2.0 + car_d_cr_, -car_width_ / 2.0;
      car_vertex_big_.push_back(vertex_big);
      vertex_big << -car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
      car_vertex_big_.push_back(vertex_big);
      vertex_big << car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
      car_vertex_big_.push_back(vertex_big);   

      car_length_ -= 0.4;
      car_width_ -= 0.8;   
  }


  bool KinoAstar::isInMap2d(const Eigen::Vector2d &pos)
  {
      Eigen::Vector2i idx;
      posToIndex2d(pos, idx);
      return isInMap2d(idx);
  }

  bool KinoAstar::isInMap2d(const Eigen::Vector2i &id)
  {
      if(id(0) < 0 || id(0) >= global_map_size_(0) || id(1) < 0 || id(1) >= global_map_size_(1))
      {
          return false;
      }
      else
          return true;
  };

  void KinoAstar::posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id)
  {
      for(int i = 0; i < 2; i++)
      {
          id(i) = floor((pos(i) - map_origin_(i)) * inv_resolution_);
      }
  }

  void KinoAstar::indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
  {
      for(int i = 0; i < 2; i++)
      {
          pos(i) = id(i) * resolution_ + map_origin_(i);
      }
  }

  int KinoAstar::getVoxelState2d(const Eigen::Vector2d &pos)
  {
      Eigen::Vector2i id;
      posToIndex2d(pos, id);
      if(!isInMap2d(id))
          return -1;

      return occupancy_buffer_2d_[id(1) * global_map_size_(0) + id(0)];
  }

  bool KinoAstar::isOccupied(const Eigen::Vector2d& pos)
	{
		const int state = getVoxelState2d(pos);
    return (unknown_as_occupied_ && state < 0) || state >= occupied_threshold_;
	}

  bool KinoAstar::isOccupied(const Eigen::Vector2i& id)
	{
		Eigen::Vector2d pos;
		indexToPos2d(id, pos);
		const int state = getVoxelState2d(pos);
    return (unknown_as_occupied_ && state < 0) || state >= occupied_threshold_;
	}

  int KinoAstar::yawToIndex(double& yaw)
  {
    double normalized_yaw = normalize_angle(yaw);
    int idx = floor((normalized_yaw - yaw_origin_) * inv_yaw_resolution_);
    int yaw_num = ceil(2.0 * M_PI * inv_yaw_resolution_);
    if (idx < 0) {
      idx = 0;
    }
    if (idx >= yaw_num) {
      idx = yaw_num - 1;
    }
    return idx;
  }

  void KinoAstar::stateTransit(Eigen::Vector3d &state0,  Eigen::Vector3d &state1,
              Eigen::Vector2d &ctrl_input)
  {
      //helpful var
      double psi = ctrl_input[0]; 
      double s = ctrl_input[1]; 
      if(psi!=0){
        double k = car_wheelbase_ / tan(psi);
        state1[0] = state0[0] + k*(sin(state0[2]+s/k)-sin(state0[2]));
        state1[1] = state0[1] - k*(cos(state0[2]+s/k)-cos(state0[2]));
        state1[2] = state0[2] + s/k;
      }
      else{
        state1[0] = state0[0] + s * cos(state0[2]);
        state1[1] = state0[1] + s * sin(state0[2]);
        state1[2] = state0[2]; 
      }
  }

  void KinoAstar::checkCollisionUsingPosAndYaw(const Eigen::Vector3d &state, bool &res)
  {
      res = false;
      Eigen::Vector2d pos = state.head(2);
      double yaw = state[2];
      Eigen::Matrix2d Rotation_matrix;
      Rotation_matrix << cos(yaw),  -sin(yaw),
                         sin(yaw),  cos(yaw);
      for(int i = 0; i < 4; i++)
      {
          Eigen::Vector2d start_point = pos + Rotation_matrix * car_vertex_[i];
          Eigen::Vector2d end_point = pos + Rotation_matrix * car_vertex_[i+1];

          RayCaster raycaster;
          raycaster.setInput((start_point - map_origin_) / resolution_,
                             (end_point - map_origin_) / resolution_);
          Eigen::Vector2d half(0.5, 0.5);
          Eigen::Vector2d ray_pt;
          while(raycaster.step(ray_pt))
          {
              Eigen::Vector2d tmp = (ray_pt + half) * resolution_ + map_origin_;
              if(isOccupied(tmp))
              {
                  res = true;
                  return;
              }
          }
      }
  }

  void KinoAstar::checkCollisionUsingLine(const Eigen::Vector2d &start_pt, const Eigen::Vector2d &end_pt, bool &res)
  {
      res = false;
      RayCaster raycaster;
      raycaster.setInput((start_pt - map_origin_) / resolution_,
                         (end_pt - map_origin_) / resolution_);
      Eigen::Vector2d half = Eigen::Vector2d(0.5, 0.5);
      Eigen::Vector2d ray_pt;

      while(raycaster.step(ray_pt))
      {
          Eigen::Vector2d tmp = (ray_pt + half) * resolution_ + map_origin_;
          if(isOccupied(tmp))
          {
              res = true;
              return;
          }
      }
  }

  void KinoAstar::ConvertNodePathToPointPath(vector<PathNodePtr> path_nodes_)
  {
    for(size_t i = 0; i < path_nodes_.size(); i++)
    {
        Eigen::Vector2d pos = path_nodes_[i]->state.head(2);
        final_path_.push_back(pos);
    }
  }

  int KinoAstar::search(Eigen::Vector4d start_state, Eigen::Vector2d init_ctrl,
                               Eigen::Vector4d end_state)
  {
    ros::Time t1 = ros::Time::now();
    final_path_.clear();

    Eigen::Vector2d start_pos = start_state.head(2);
    Eigen::Vector2d goal_pos = end_state.head(2);

    if (!isInMap2d(start_pos) || !isInMap2d(goal_pos)) {
			ROS_WARN("kino astar Start or goal position out of map boundary!");
			return NO_PATH;
		}

		if (isOccupied(goal_pos)) {
			ROS_WARN("Goal position is occupied!");
			return NO_PATH;
		}

    // checkCollisionUsingPosAndYaw(end_state.head(3), isocc);
    // if(isocc)
    // {
    //   ROS_WARN("Goal position is occupied!");
    //   return NO_PATH;
    // }

    start_state_ = start_state;
    start_ctrl_ = init_ctrl;
    end_state_ = end_state;
    Eigen::Vector2i end_index;
    posToIndex2d(end_state.head(2), end_index);
    /* ---------- initialize ---------- */
    
    PathNodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->state = start_state.head(3);
    posToIndex2d(start_state.head(2), cur_node->index);
    cur_node->g_score = 0.0;
    cur_node->input = Eigen::Vector2d(0.0,0.0);
    cur_node->singul = getSingularity(start_state(3));
    cur_node->number = 0;
    cur_node->f_score = lambda_heu_ * getHeu(cur_node->state, end_state);
    cur_node->node_state = IN_OPEN_SET;
    open_set_.push(cur_node);
    use_node_num_ += 1;

    expanded_nodes_.insert(cur_node->index, cur_node);

    PathNodePtr terminate_node = NULL;
    /* ---------- search loop ---------- */
    while (!open_set_.empty())
    {
      /* ---------- get lowest f_score node ---------- */
      cur_node = open_set_.top();

      /* ---------- determine termination ---------- */
      // double t1 = ros::Time::now().toSec();
      if((cur_node->state.head(2) - end_state_.head(2)).norm() < 3.0){
        is_shot_sucess(cur_node->state,end_state_.head(3));//直接检查两点之间有没有障碍物， 没有障碍物则将 is_shot_succ_ 置为true
      }

      if (is_shot_succ_)
      {
        final_path_.clear();
        path_nodes_.clear();
        terminate_node = cur_node;
        
        retrievePath(terminate_node);
        ConvertNodePathToPointPath(path_nodes_);
        ros::Time t3 = ros::Time::now();
        std::cout<<" kinoastar time: "<<(t3-t1).toSec() * 1000 <<" ms" << " iter_num_: " << iter_num_ <<std::endl;

        ompl::base::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
        Eigen::Vector3d state1, state2;
        state1 = path_nodes_.back()->state.head(3);
        state2 = end_state_.head(3);
        from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
        to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
        double shotLength = shotptr->distance(from(), to());
        std::vector<double> reals;
        for(double l = checkl; l < shotLength; l += checkl)
        {
          shotptr->interpolate(from(), to(), l/shotLength, s());
          reals = s.reals();
          final_path_.push_back(Eigen::Vector2d(reals[0], reals[1]));
        }
        if((final_path_.back() - end_state_.head(2)).norm() < checkl)
        {
          final_path_.pop_back();
        }
        final_path_.push_back(end_state_.head(2));
        has_path_ = true;


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
        cout << "kino original_length: " << original_length << endl;
        double avg_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0) / curvatures.size();
        double max_curvature = *std::max_element(curvatures.begin(), curvatures.end());
        cout << "kino original_avg_curvature: " << avg_curvature << " kino original_max_curvature: " << max_curvature << endl;

        if (is_shot_succ_)
        {
          return REACH_END;
        }
      }
      ros::Time t2 = ros::Time::now();

      double runTime = (t2 - t1).toSec() * 1000;
      // TODO
      if(runTime > max_seach_time){
          ROS_WARN("KinoSearch: Reach the max seach time");
          return REACH_END;
      }
      /* ---------- pop node and add to close set ---------- */
      open_set_.pop();
      cur_node->node_state = IN_CLOSE_SET;
      iter_num_ += 1;
      /* ---------- init state propagation ---------- */
      Eigen::Vector3d cur_state = cur_node->state;
      Eigen::Vector3d pro_state;
      Eigen::Vector2d ctrl_input;
      vector<Eigen::Vector2d> inputs;
      for (double arc = -step_arc; arc <= step_arc + 1e-3; arc += 0.5 * step_arc){
        if(fabs(arc)<1.0e-2) continue;
        for (double steer = -max_steer_; steer <= max_steer_ + 1e-3; steer += 0.5 * max_steer_ * 1.0)
        {
          ctrl_input << steer, arc;
          inputs.push_back(ctrl_input);
        }
      }
      /* ---------- state propagation loop ---------- */
      for (auto& input:inputs){
        int singul = input[1]>0?1:-1;
        stateTransit(cur_state, pro_state, input);
        // NodeVis(pro_state);
        Eigen::Vector2d pro_pos = pro_state.head(2);
        if(!isInMap2d(pro_pos))
        {
          continue;
        }
        if(isOccupied(pro_pos))
        {
          continue;
        }

        /* not in close set */
        Eigen::Vector2i pro_id;
        posToIndex2d(pro_state.head(2), pro_id);
        PathNodePtr pro_node;
        pro_node = expanded_nodes_.find(pro_id);

        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          continue;
        }

        Eigen::Vector3d xt;
        bool is_occ = false;
        double obstacle_cost = 0.0;
        const double sample_length = std::fabs(input[1]) / check_num_;
        const double max_soft_cost = std::max(1, occupied_threshold_ - 1);
        for (int k = 1; k <= check_num_; ++k)
        {
          double tmparc = input[1] * double(k) / double(check_num_);
          Eigen::Vector2d tmpctrl; tmpctrl << input[0],tmparc;
          stateTransit(cur_state, xt, tmpctrl);
          Eigen::Vector2d xt_pos =  xt.head(2);
          if (isOccupied(xt_pos)) {
            is_occ = true;
            break;
          }
          const double normalized_cost =
              std::max(0, getVoxelState2d(xt_pos)) / max_soft_cost;
          obstacle_cost += obstacle_cost_weight_ * normalized_cost
              * normalized_cost * sample_length;
        }
        if (is_occ)  continue;
        /* ---------- compute cost ---------- */
        double tmp_g_score = 0.0;
        double tmp_f_score = 0.0;
        int lastDir = cur_node->singul;
        if(singul>0){
          tmp_g_score +=  std::fabs(input[1]) * traj_forward_penalty;
        }
        else{
          tmp_g_score += std::fabs(input[1]) * traj_back_penalty;
        }
        if(singul * lastDir < 0){
          tmp_g_score += traj_gear_switch_penalty;
        }
        tmp_g_score += traj_steer_penalty * std::fabs(input[0]) * std::fabs(input[1]);
        tmp_g_score += traj_steer_change_penalty * std::fabs(input[0]-cur_node->input[0]);
        tmp_g_score += obstacle_cost;
        tmp_g_score += cur_node->g_score;

        tmp_f_score = tmp_g_score + lambda_heu_ * getHeu(pro_state, end_state);
        /* ---------- compare expanded node in this loop ---------- */
        if (pro_node == NULL)
        {
          pro_node = path_node_pool_[use_node_num_];
          pro_node->index = pro_id;
          pro_node->state = pro_state;
          pro_node->f_score = tmp_f_score;
          pro_node->g_score = tmp_g_score;
          pro_node->input = input;
          pro_node->parent = cur_node;
          pro_node->node_state = IN_OPEN_SET;
          pro_node->singul = singul;
          pro_node->number = cur_node->number + 1;
          open_set_.push(pro_node);

          expanded_nodes_.insert(pro_id, pro_node);
          use_node_num_ += 1;
          if (use_node_num_ == allocate_num_)
          {
            cout << "run out of memory." << endl;
            return NO_PATH;
          }
        }
        else if (pro_node->node_state == IN_OPEN_SET)
        {

          if (tmp_g_score < pro_node->g_score)
          {
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = input;
            pro_node->parent = cur_node;
            pro_node->singul = singul;
            pro_node->number = cur_node->number + 1;
          }
        }
        else
        {
          cout << "error type in searching: " << pro_node->node_state << endl;
        }
      }
    }

    ROS_ERROR("Open set empty, no path!");
    return NO_PATH;
  }


  bool KinoAstar::is_shot_sucess(Eigen::Vector3d state1,Eigen::Vector3d state2){
    
    std::vector<Eigen::Vector3d> path_list;
    double len;
    computeShotTraj(state1,state2,path_list,len);
    // double t1 = ros::Time::now().toSec();
    for(unsigned int i = 0; i < path_list.size(); ++i){
        Eigen::Vector2d path_pos = path_list[i].head(2);
        if(isOccupied(path_pos))
        {
          return false;
        }
    }
    // double t2 = ros::Time::now().toSec();

    is_shot_succ_ = true;
    return true;
  }

  double KinoAstar::computeShotTraj(Eigen::Vector3d &state1, Eigen::Vector3d &state2,
                                    std::vector<Eigen::Vector3d> &path_list,
                                    double& len){
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    ob::ScopedState<> from(shotptr), to(shotptr), s(shotptr);
    from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
    to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
    std::vector<double> reals;
    len = shotptr->distance(from(), to());
    double sum_T = len/max_vel_;    

    for (double l = 0.0; l <=len; l += checkl)
    {
      shotptr->interpolate(from(), to(), l/len, s());
      reals = s.reals();
      path_list.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
    }
  
    return sum_T;
  }

  // to retrieve the path to the correct order
  void KinoAstar::retrievePath(PathNodePtr end_node)
  {
    PathNodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL)
    {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node);
    }

    reverse(path_nodes_.begin(), path_nodes_.end());
  }

  void KinoAstar::reset()
  {
    expanded_nodes_.clear();
    path_nodes_.clear();
    final_path_.clear();

    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
      PathNodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    is_shot_succ_ = false;
  }

  const double kPi = acos(-1.0);// pai
  double KinoAstar::normalize_angle(const double& theta)
  {
    double theta_tmp = theta;
    theta_tmp -= (theta >= kPi) * 2 * kPi;
    theta_tmp += (theta < -kPi) * 2 * kPi;
    return theta_tmp;
  }

  void KinoAstar::getFlatState(Eigen::Vector4d state, Eigen::Vector2d control_input,
                                  Eigen::MatrixXd &flat_state, int singul)
  {

    flat_state.resize(2, 3);

    double angle = state(2);
    double vel   = state(3); // vel > 0 

    Eigen::Matrix2d init_R;
    init_R << cos(angle),  -sin(angle),
              sin(angle),   cos(angle);
    
    flat_state << state.head(2), init_R*Eigen::Vector2d(vel, 0.0), 
                  init_R*Eigen::Vector2d(control_input(1), std::tan(control_input(0)) / car_wheelbase_ * std::pow(vel, 2));
  }
}
