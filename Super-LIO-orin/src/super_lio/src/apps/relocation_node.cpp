
#include <csignal>
#include <cstdlib>
#include <ros/ros.h>
#include "lio/super_lio_reloc.h"
#include "ros/ROSWrapper.h"


using namespace LI2Sup;

volatile std::sig_atomic_t g_sigint_received = 0;

void SigHandle(int sig) {
  g_sigint_received = 1;
  g_flag_run = false;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "lio", ros::init_options::NoSigintHandler);
  signal(SIGINT, SigHandle);
  ros::NodeHandle nh;
  LoadParamFromRos(nh);

  ROSWrapper::Ptr data_wrapper = std::make_shared<ROSWrapper>();
  auto lio = std::make_shared<SuperLIOReLoc>();
  lio->setROSWrapper(data_wrapper);
  lio->init();

  ros::Rate rate(500);  // 500 Hz
  while (ros::ok() && g_flag_run) {
    data_wrapper->spinOnce();
    lio->process();
    rate.sleep();
  }

  lio->saveMap();
  lio->printTimeRecord();

  // 先释放持有 ROS 订阅器/发布器的对象，ROS 通信由 NodeHandle 生命周期收尾。
  lio.reset();
  data_wrapper.reset();
  if (g_sigint_received) {
    // SIGINT 退出时绕过 ROS/boost 静态对象清理，避免 roslaunch 停止阶段抛 lock_error。
    std::_Exit(0);
  }
  return 0;
}
