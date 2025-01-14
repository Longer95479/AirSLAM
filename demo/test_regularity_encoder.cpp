#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <thread>

#include "read_configs.h"
#include "dataset.h"
#include "map_builder.h"
#include "debug.h"

#include "plnet.h"
#include "feature_detector.h"
#include "regularity_encoder.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "air_slam");

  // std::string camera_config_path = "/media/code/ubuntu_files/airvio/catkin_ws/src/AirVIO/configs/camera/euroc.yaml";
  // std::string dataroot = "/media/data/datasets/euroc/seq/MH_01_easy/";
  // std::string model_dir = "/media/code/ubuntu_files/airvio/catkin_ws/src/AirVIO/output";
  // std::string save_root = "/media/code/ubuntu_files/airvio/catkin_ws/src/AirVIO/debug/point_detection";

  // std::string camera_config_path = "/media/code/ubuntu_files/airvio/catkin_ws/src/AirVIO/configs/camera/tartanair.yaml";
  // std::string dataroot = "/media/bssd/datasets/tartanair/mapping_relocalization/relocalization/abandonedfactory/sequences/P000";
  // std::string model_dir = "/media/code/ubuntu_files/airvio/catkin_ws/src/AirVIO/output";
  // std::string save_root = "/media/code/ubuntu_files/airvio/catkin_ws/src/AirVIO/debug/line_detection";
  
  std::string camera_config_path = "/root/catkin_ws/src/AirSLAM/configs/camera/euroc.yaml";
  std::string dataroot = "/workspace/mav0";
  std::string model_dir = "/root/catkin_ws/src/AirSLAM/output";
  // std::string save_root = "/root/catkin_ws/src/AirSLAM/debug/line_detection";
  std::string save_root = "/workspace/debug_regularity_encoder";
  std::string left_image_dir = ConcatenateFolderAndFileName(dataroot, "cam0/data");
  std::string right_image_dir = ConcatenateFolderAndFileName(dataroot, "cam1/data");

  MakeDir(save_root);

  PLNetConfig plnet_config;
  plnet_config.use_superpoint = 1;
  plnet_config.max_keypoints = 400;
  plnet_config.keypoint_threshold = 0.04;
  plnet_config.remove_borders = 4;
  plnet_config.line_threshold = 0.5;
  plnet_config.line_length_threshold = 50;
  plnet_config.SetModelPath(model_dir);

  CameraPtr _camera = std::shared_ptr<Camera>(new Camera(camera_config_path));

  FeatureDetectorPtr feature_detector = std::shared_ptr<FeatureDetector>(new FeatureDetector(plnet_config));

  RegularityEncoderPtr regularity_encoder = std::shared_ptr<RegularityEncoder>(new RegularityEncoder(_camera, 0.3, 0.99));

  std::vector<std::string> image_names;
  GetFileNames(left_image_dir, image_names);
  // size_t dataset_length = image_names.size();
  size_t dataset_length = 14;
  for(size_t i = 0; i < dataset_length && ros::ok(); ++i){
    std::cout << "i ==================================== " << i << std::endl;

    std::string image_path = ConcatenateFolderAndFileName(left_image_dir, image_names[i]);
    cv::Mat image = cv::imread(image_path, 0);

    cv::Mat image_left_rect;
    _camera->UndistortImage(image, image_left_rect);

    Eigen::Matrix<float, 259, Eigen::Dynamic> features;
    std::vector<float> line_scores;
    std::vector<Eigen::Vector4d> lines;

    auto before_infer = std::chrono::high_resolution_clock::now();

    cv::Mat resized_image;
    cv::resize(image_left_rect, resized_image, cv::Size(512, 512));

    // feature_detector->Detect(image_left_rect, features);
    feature_detector->Detect(image_left_rect, features, lines);

    std::map<int, std::vector<Eigen::Vector3d>> DDs;
    std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>> normal_inliers;
    std::map<int, std::map<int, std::map<int, Eigen::Vector4d>>> line_inliers;
    std::map<int, std::vector<Eigen::Vector4d>> DDs_on_image;

    regularity_encoder->encode(lines, DDs, normal_inliers);
    regularity_encoder->printNormalInliers(normal_inliers);
    regularity_encoder->projectDDsOnImage(DDs, DDs_on_image);
    regularity_encoder->normalInliers2LineInliers(lines, normal_inliers, line_inliers);
    regularity_encoder->printLineInliers(line_inliers);

    SaveLineDetectionResult(image_left_rect, lines, save_root, std::to_string(i));
    SaveLineRegularityEncodeResult(image_left_rect, line_inliers, DDs_on_image, save_root, std::to_string(i));
    SaveDetectorResult(image_left_rect, features, save_root, std::to_string(i));

    std::cout << "lines ==================================== " << lines.size() << std::endl;

    auto after_infer = std::chrono::high_resolution_clock::now();
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
    std::cout << "One Frame Processinh Time: " << cost_time << " ms." << std::endl;
  }


  ros::shutdown();

  return 0;
}
