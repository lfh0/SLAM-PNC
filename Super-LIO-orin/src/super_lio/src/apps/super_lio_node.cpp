
#include <csignal>
#include <ros/ros.h>
#include "lio/super_lio.h"
#include "ros/ROSWrapper.h"


using namespace LI2Sup;

void SigHandle(int sig) {
  g_flag_run = false;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "lio");
  signal(SIGINT, SigHandle);
  ros::NodeHandle nh;
  LoadParamFromRos(nh);

  ROSWrapper::Ptr data_wrapper = std::make_shared<ROSWrapper>();
  auto lio = std::make_shared<SuperLIO>();
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
  return 0;
}