#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <thread>

#include "read_configs.h"
#include "ros_subscriber.h"
#include "map_builder.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "air_slam");

  std::string config_path, model_dir;
  ros::param::get("~config_path", config_path);
  ros::param::get("~model_dir", model_dir);
  VisualOdometryConfigs configs(config_path, model_dir);
  std::cout << "config done" << std::endl;

  ros::param::get("~dataroot", configs.dataroot);
  ros::param::get("~camera_config_path", configs.camera_config_path);
  ros::param::get("~saving_dir", configs.saving_dir);

  ros::NodeHandle nh;
  MapBuilder map_builder(configs, nh);
  std::cout << "map_builder done" << std::endl;

  RosSubscriber ros_subscriber(configs.ros_subscriber_config, nh, map_builder);
  std::cout << "ros_subscriber done" << std::endl;

  // ros::Rate rate(500);  // 500 Hz
  // while (ros::ok()) {
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  ros::spin();

  std::cout << "Waiting to stop..." << std::endl; 
  map_builder.Stop();
  while(!map_builder.IsStopped()){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Map building has been stopped" << std::endl; 

  std::string trajectory_path = ConcatenateFolderAndFileName(configs.saving_dir, "trajectory_v0.txt");
  map_builder.SaveTrajectory(trajectory_path);
  map_builder.SaveMap(configs.saving_dir);
  ros::shutdown();

  return 0;
}
