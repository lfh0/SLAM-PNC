#include <memory>
#include <string>

#include <nav_msgs/OccupancyGrid.h>
#include <robot_trajectory_msgs/LocalOptimizationRequest.h>
#include <robot_trajectory_msgs/LocalOptimizationStatus.h>
#include <robot_trajectory_msgs/LocalPathCandidate.h>
#include <robot_trajectory_msgs/ReferenceBlockage.h>
#include <robot_trajectory_msgs/ReferenceWindow.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "local_optimization_manager/request_manager.h"

namespace local_optimization_manager {

class RequestManagerNode {
 public:
  RequestManagerNode() : nh_(), pnh_("~") {
    Config config;
    pnh_.param("path_replan_collision_threshold",
               config.path_replan_collision_threshold, 80);
    pnh_.param("unknown_is_obstacle", config.unknown_is_obstacle, true);
    pnh_.param("endpoint_change_distance_m",
                config.endpoint_change_distance_m, 0.5);
    pnh_.param("path_change_distance_m", config.path_change_distance_m, 0.3);
    pnh_.param("relative_length_change", config.relative_length_change, 0.10);
    pnh_.param("topology_deadband_m", config.topology_deadband_m, 0.20);
    pnh_.param("predicted_execution_delay_s",
                config.predicted_execution_delay_s, 0.15);

    pnh_.param("candidate_topic", candidate_topic_,
                std::string("/local_replanner/candidate"));
    pnh_.param("blockage_topic", blockage_topic_,
                std::string("/local_reference/blockage"));
    pnh_.param("reference_window_topic", reference_window_topic_,
                std::string("/local_reference/window"));
    pnh_.param("costmap_topic", costmap_topic_,
                std::string("/local_costmap_node/costmap/costmap"));
    pnh_.param("optimizer_status_topic", optimizer_status_topic_,
                std::string("/local_optimizer/status"));
    pnh_.param("request_topic", request_topic_,
                std::string("/local_optimizer/request"));

    manager_.reset(new RequestManager(config));
    request_pub_ = nh_.advertise<robot_trajectory_msgs::LocalOptimizationRequest>(
        request_topic_, 4);
    state_pub_ = nh_.advertise<std_msgs::String>(
        "/local_optimizer/request_manager_status", 1, true);
    candidate_sub_ = nh_.subscribe(candidate_topic_, 2,
        &RequestManagerNode::candidateCallback, this);
    blockage_sub_ = nh_.subscribe(blockage_topic_, 2,
        &RequestManagerNode::blockageCallback, this);
    window_sub_ = nh_.subscribe(reference_window_topic_, 2,
        &RequestManagerNode::windowCallback, this);
    map_sub_ = nh_.subscribe(costmap_topic_, 1,
        &RequestManagerNode::mapCallback, this);
    optimizer_status_sub_ = nh_.subscribe(optimizer_status_topic_, 4,
        &RequestManagerNode::optimizerStatusCallback, this);

    ROS_INFO("Local optimization request manager ready: candidate=%s request=%s path_replan_threshold=%d",
             candidate_topic_.c_str(), request_topic_.c_str(),
             config.path_replan_collision_threshold);
  }

 private:
  void windowCallback(const robot_trajectory_msgs::ReferenceWindowConstPtr& message) {
    const Update update = manager_->updateGlobalTrajectory(
        message->global_trajectory_id, message->header.stamp, message->header.frame_id);
    if (update.reason == "GLOBAL_TRAJECTORY_CHANGED") {
      // 缓存候选属于旧全局轨迹，禁止在新阻断状态到达时被重放。
      has_latest_candidate_ = false;
    }
    publishUpdate(update);
  }

  void publishUpdate(const Update& update) {
    for (const auto& request : update.requests) {
      request_pub_.publish(request);
      ROS_INFO("Local optimizer request: action=%s episode=%llu request=%llu reason=%s",
               request.action == robot_trajectory_msgs::LocalOptimizationRequest::OPTIMIZE
                   ? "OPTIMIZE" : "CANCEL",
               static_cast<unsigned long long>(request.episode_id),
               static_cast<unsigned long long>(request.request_id),
               request.reason.c_str());
    }
    std_msgs::String state;
    state.data = std::string(RequestManager::stateName(manager_->state())) +
        " reason=" + update.reason +
        " episode=" + std::to_string(manager_->episodeId()) +
        " request=" + std::to_string(manager_->requestId());
    state_pub_.publish(state);
  }

  void candidateCallback(
      const robot_trajectory_msgs::LocalPathCandidateConstPtr& message) {
    latest_candidate_ = *message;
    has_latest_candidate_ = true;
    publishUpdate(manager_->consider(*message));
  }

  void blockageCallback(
      const robot_trajectory_msgs::ReferenceBlockageConstPtr& message) {
    latest_blockage_state_ = message->state;
    publishUpdate(manager_->updateBlockage(message->state, message->header.stamp,
                                           message->header.frame_id));
    if (message->state == robot_trajectory_msgs::ReferenceBlockage::REPLAN_REQUIRED &&
        has_latest_candidate_) {
      // ROS跨话题不保证回调顺序：候选可能比对应阻断状态先到达，状态就绪后重放一次。
      publishUpdate(manager_->consider(latest_candidate_));
    } else if (message->state !=
               robot_trajectory_msgs::ReferenceBlockage::REPLAN_REQUIRED) {
      has_latest_candidate_ = false;
    }
  }

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& message) {
    publishUpdate(manager_->updateMap(*message));
    if (latest_blockage_state_ ==
            robot_trajectory_msgs::ReferenceBlockage::REPLAN_REQUIRED &&
        has_latest_candidate_) {
      // 地图也可能晚于候选到达；始终在最新地图上重新校验缓存候选。
      publishUpdate(manager_->consider(latest_candidate_));
    }
  }

  void optimizerStatusCallback(
      const robot_trajectory_msgs::LocalOptimizationStatusConstPtr& message) {
    publishUpdate(manager_->updateOptimizerStatus(*message));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::unique_ptr<RequestManager> manager_;
  ros::Subscriber candidate_sub_;
  ros::Subscriber blockage_sub_;
  ros::Subscriber window_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber optimizer_status_sub_;
  ros::Publisher request_pub_;
  ros::Publisher state_pub_;
  std::string candidate_topic_;
  std::string blockage_topic_;
  std::string reference_window_topic_;
  std::string costmap_topic_;
  std::string optimizer_status_topic_;
  std::string request_topic_;
  robot_trajectory_msgs::LocalPathCandidate latest_candidate_;
  bool has_latest_candidate_ = false;
  uint8_t latest_blockage_state_ =
      robot_trajectory_msgs::ReferenceBlockage::NORMAL;
};

}  // namespace local_optimization_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_optimization_manager");
  local_optimization_manager::RequestManagerNode node;
  ros::spin();
  return 0;
}
