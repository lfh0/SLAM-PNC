#include <gtest/gtest.h>

#include "plan_manage/traj_optimizer.h"

namespace
{

std::vector<Eigen::MatrixXd> makeSampledCorridors(const int count)
{
  // 矩形安全走廊：每列为 [外法向; 法向上一点]。
  Eigen::MatrixXd corridor(4, 4);
  corridor.col(0) << 1.0, 0.0, 3.0, 0.0;
  corridor.col(1) << -1.0, 0.0, -1.0, 0.0;
  corridor.col(2) << 0.0, 1.0, 0.0, 2.0;
  corridor.col(3) << 0.0, -1.0, 0.0, -2.0;
  return std::vector<Eigen::MatrixXd>(count, corridor);
}

plan_manage::MincoRequest makeRequest()
{
  plan_manage::MincoRequest request;
  request.initial_state.resize(2, 3);
  request.final_state.resize(2, 3);
  request.initial_state << 0.0, 0.20, 0.0,
                           0.0, 0.00, 0.0;
  request.final_state << 2.0, 0.20, 0.0,
                         0.0, 0.00, 0.0;
  request.initial_inner_points.resize(2, 1);
  request.initial_inner_points << 1.0, 0.0;
  request.total_time = 10.0;
  request.piece_time_ratios = Eigen::Vector2d(0.5, 0.5);
  // 两段轨迹的首尾段各采样 destraj_resolution + 1 次。
  request.sampled_corridors = makeSampledCorridors(42);
  request.singular_direction = 1;
  request.start_time = 0.0;
  return request;
}

plan_manage::PolyTrajOptimizer makeOptimizer()
{
  plan_manage::MincoConfig config;
  config.destraj_resolution = 20;
  config.traj_resolution = 8;
  config.wei_obs = 0.0;
  config.wei_surround = 0.0;
  config.wei_feas = 10.0;
  config.wei_time = 1.0;
  config.max_vel = 1.0;
  config.max_acc = 1.0;
  plan_manage::PolyTrajOptimizer optimizer;
  optimizer.configure(config);
  optimizer.setVerbose(false);
  return optimizer;
}

TEST(MincoCore, RejectsInvalidBoundaryShape)
{
  auto optimizer = makeOptimizer();
  auto request = makeRequest();
  request.initial_state.resize(3, 3);
  const auto result = optimizer.optimize(request);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.cancelled);
  EXPECT_EQ("boundary_state_must_be_2x3", result.failure_reason);
}

TEST(MincoCore, PreservesNonzeroBoundaryVelocity)
{
  auto optimizer = makeOptimizer();
  const auto result = optimizer.optimize(makeRequest());
  ASSERT_TRUE(result.success) << result.failure_reason;
  EXPECT_NEAR(0.20, result.trajectory.getdSigma(0.0).x(), 1.0e-6);
  EXPECT_NEAR(0.20, result.trajectory.getdSigma(result.trajectory.getTotalDuration()).x(), 1.0e-6);
  EXPECT_GT(result.trajectory.getTotalDuration(), 0.0);
}

TEST(MincoCore, StopsWhenRequestIsCancelled)
{
  auto optimizer = makeOptimizer();
  auto request = makeRequest();
  request.cancel_checker = [] { return true; };
  const auto result = optimizer.optimize(request);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.cancelled);
  EXPECT_EQ("cancelled", result.failure_reason);
}

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_minco_core", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
