#ifndef ROS_SUBSCRIBER_H_
#define ROS_SUBSCRIBER_H_

#include <mutex>
#include <map>
#include <vector>
#include <unordered_map>
#include <thread>
#include <queue>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "utils.h"
#include "imu.h"
#include "read_configs.h"
#include "map_builder.h"

class RosSubscriber{
public:
  RosSubscriber(const RosSubscriberConfig& ros_subscriber_config, ros::NodeHandle nh, MapBuilder &map_builder);

  void leftImgCallBack(const sensor_msgs::ImageConstPtr &img_msg);
  void rightImgCallBack(const sensor_msgs::ImageConstPtr &img_msg);
  void imuCallBack(const sensor_msgs::ImuConstPtr &imu_msg);
  void sync_thread();
  
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
  void getImuFrontFromMsg(ImuData &imu_data, ImuDataList &batch_imu_data);
  bool batchImuData(double t0, double t1, ImuDataList &batch_imu_data);

private:
  RosSubscriberConfig _config;	//TODO in read_configs.h

  MapBuilder &_map_builder;

  std::thread _sync_thread;

  ros::Subscriber _ros_left_img_sub;
  ros::Subscriber _ros_right_img_sub;
  ros::Subscriber _ros_imu_sub;

  std::queue<sensor_msgs::ImuConstPtr> _imu_buf;
  std::queue<sensor_msgs::ImageConstPtr> _left_img_buf;
  std::queue<sensor_msgs::ImageConstPtr> _right_img_buf;
 
  std::mutex _m_buf;
  
  double _last_image_time, _image_time;
  int _frame_index;

  ImuDataList _batch_imu_data;
};


#endif // ROS_SUBSCRIBER_H_

