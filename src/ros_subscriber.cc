#include "ros_subscriber.h"
#include <Eigen/Geometry>
#include "utils.h"

RosSubscriber::RosSubscriber(const RosSubscriberConfig& ros_subscriber_config, ros::NodeHandle nh, MapBuilder &map_builder): _config(ros_subscriber_config), _last_image_time(-1.0), _frame_index(0), _map_builder(map_builder), _use_imu(map_builder.UseIMU())
{

  if (_config.sub_topic) {
    _ros_left_img_sub = nh.subscribe(_config.left_img_topic, 100, &RosSubscriber::leftImgCallBack, this);
    _ros_right_img_sub = nh.subscribe(_config.right_img_topic, 100, &RosSubscriber::rightImgCallBack, this);
    if (_use_imu) {
      _ros_imu_sub = nh.subscribe(_config.imu_topic, 2000, &RosSubscriber::imuCallBack, this);
    }

    _sync_thread = std::thread(boost::bind(&RosSubscriber::sync_thread, this));
  }
}


void RosSubscriber::getImuFrontFromMsg(ImuData &imu_data, ImuDataList &batch_imu_data)
{
  double t = _imu_buf.front()->header.stamp.toSec();
  double dx = _imu_buf.front()->linear_acceleration.x;
  double dy = _imu_buf.front()->linear_acceleration.y;
  double dz = _imu_buf.front()->linear_acceleration.z;
  double rx = _imu_buf.front()->angular_velocity.x;
  double ry = _imu_buf.front()->angular_velocity.y;
  double rz = _imu_buf.front()->angular_velocity.z;

  Eigen::Vector3d acc(dx, dy, dz);
  Eigen::Vector3d gyr(rx, ry, rz);

  imu_data.timestamp = t;
  imu_data.acc = acc;
  imu_data.gyr = gyr;

  batch_imu_data.emplace_back(imu_data);
}


cv::Mat RosSubscriber::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {   
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }   
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}


int RosSubscriber::batchImuData(double t0, double t1, ImuDataList &batch_imu_data)
{
  ImuData imu_data, back_imu_data, second_back_imu_data;
  
  if (_imu_buf.empty()) {
    ROS_WARN("Not receive imu");
    return 0;
  }

  if (t1 >= _imu_buf.back()->header.stamp.toSec()) {
    ROS_WARN("Wait for imu");
    return -1;
  }
 
  if (batch_imu_data.size() >= 2) {
    back_imu_data = *(--batch_imu_data.end());
    second_back_imu_data = *(batch_imu_data.end() - 2);
    batch_imu_data.clear();
    batch_imu_data.emplace_back(second_back_imu_data);
    batch_imu_data.emplace_back(back_imu_data);
  }
  else {
    ROS_WARN("Size of last imu data batch is less than 2");
    batch_imu_data.clear();
  }

  _m_buf.lock();

  while (_imu_buf.front()->header.stamp.toSec() <= t0) {
    _imu_buf.pop();
    ROS_WARN("Clean outdated imu data");
  }
  while (_imu_buf.front()->header.stamp.toSec() <= t1) {
    getImuFrontFromMsg(imu_data, batch_imu_data);
    _imu_buf.pop();
  }
  getImuFrontFromMsg(imu_data, batch_imu_data);
  _imu_buf.pop();

  _m_buf.unlock();

  return 1;
}


void RosSubscriber::leftImgCallBack(const sensor_msgs::ImageConstPtr &img_msg) {
  _m_buf.lock();
  _left_img_buf.push(img_msg);
  _m_buf.unlock();
}


void RosSubscriber::rightImgCallBack(const sensor_msgs::ImageConstPtr &img_msg) {
  _m_buf.lock();
  _right_img_buf.push(img_msg);
  _m_buf.unlock();
}


void RosSubscriber::imuCallBack(const sensor_msgs::ImuConstPtr &imu_msg) {
  _m_buf.lock();
  _imu_buf.push(imu_msg);
  _m_buf.unlock();
}


void RosSubscriber::sync_thread() {
  while(true) {
    cv::Mat img_left, img_right;
    std_msgs::Header header;
    _m_buf.lock();
    if (!_left_img_buf.empty() && !_right_img_buf.empty()) {
      double time0 = _left_img_buf.front()->header.stamp.toSec();
      double time1 = _right_img_buf.front()->header.stamp.toSec();
      // ROS_WARN("time0 = %f, time1 = %f", time0, time1);
      if (time0 < time1) {
        _left_img_buf.pop();
        ROS_WARN("throw left img");
      }
      else if (time0 > time1) {
        _right_img_buf.pop();
        ROS_WARN("throw right img");
      }
      else {
        std::cout << "i ====== " << _frame_index << std::endl;
#ifdef VERBOSE
        printf("_left_img_buf size: %d\n", _left_img_buf.size());
        printf("_right_img_buf size: %d\n", _right_img_buf.size());
#endif
        _image_time = _left_img_buf.front()->header.stamp.toSec();
        header = _left_img_buf.front()->header;
        img_left = getImageFromMsg(_left_img_buf.front());
        _left_img_buf.pop();
        img_right = getImageFromMsg(_right_img_buf.front());
        _right_img_buf.pop();
      }
    }
    _m_buf.unlock();

    if (!img_left.empty()) {
      int has_batch;
      double td = _map_builder.GetTd();

      if (_use_imu) {
        while (true) {
          has_batch = batchImuData(_last_image_time + td, _image_time + td, _batch_imu_data);
          // wait for imu
          if (has_batch == -1) 
            usleep(2000);
          // no imu data or normal return
          else if (has_batch == 0 || has_batch == 1)
            break;
          else {
            std::cout << "batchImuData return wrong." << std::endl;
            break;
          }
        }
      }

#ifdef VERBOSE
      // printf("_batch_imu_data size = %d\n", _batch_imu_data.size());
      printf("t0 = %f, t1 = %f\n", _last_image_time, _image_time);
      // for (auto &imu_data: _batch_imu_data) {
      //   printf("%f  ", imu_data.timestamp);
      // }
      // printf("\n");
#endif
      printf("td = %f\n", td);
      
      // using imu and no imu data before imgs
      if (_use_imu && has_batch == 0) { 
        ROS_WARN("throw left and right imgs");
        continue;
      }
      
      InputDataPtr data = std::shared_ptr<InputData>(new InputData());
      data->index = _frame_index;
      data->time = _image_time;
      data->td_used = td;
      data->image_left = img_left;
      data->image_right = img_right;
      data->batch_imu_data = _batch_imu_data;
      
      auto before_infer = std::chrono::high_resolution_clock::now();   
      _map_builder.AddInput(data);
      auto after_infer = std::chrono::high_resolution_clock::now();
      auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
      std::cout << "One Frame Processinh Time: " << cost_time << " ms." << std::endl;
      _frame_index++;
      _last_image_time = _image_time;
    }
  }
}

