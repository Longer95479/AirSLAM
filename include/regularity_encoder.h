#ifndef REGULARITY_ENCODER_H_
#define REGULARITY_ENCODER_H_

#include <istream>
#include <map>
#include <string>
#include <vector>
#include <map>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include "camera.h"

#include "utils.h"


class RegularityEncoder {
public:
  RegularityEncoder(const CameraPtr camera, double P_of_vertival_inlier, double P_of_all_inlier);

  bool encode(const std::vector<Eigen::Vector4d>& lines, 
        std::map<int, std::vector<Eigen::Vector3d>>& DDs, 
        std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>>& inliers);

  bool encode(const std::vector<Eigen::Vector4d>& lines, 
        const Eigen::Vector3d& g_dir,
        std::map<int, std::vector<Eigen::Vector3d>>& DDs, 
        std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>>& inliers); 

  void projectDDsOnImage(const std::map<int, std::vector<Eigen::Vector3d>>& DDs, 
        std::map<int, std::vector<Eigen::Vector4d>>& DDs_on_image);

  void normalInliers2LineInliers(const std::vector<Eigen::Vector4d>& lines, 
        const std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>>& normal_inliers, 
        std::map<int, std::map<int, std::map<int, Eigen::Vector4d>>>& line_inliers);

  void printNormalInliers(const std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>>& inliers);

  void printLineInliers(const std::map<int, std::map<int, std::map<int, Eigen::Vector4d>>>& inliers);

private:
  void  getValidInterval(const Eigen::Vector3d& last_type_DD, const Eigen::Vector3d& ref_inlier, 
        const Eigen::Vector3d& normal, const int& normal_id,
        std::map<double, std::pair<int, int>>& intervals);

  void getOverlapRegion(const std::map<double, std::pair<int, int>>& intervals, 
        std::vector<std::pair<double, double>>& overlaps);

  void getDDsAndInliers(const Eigen::Vector3d& last_type_DD, 
        const Eigen::Vector3d& ref_inlier, 
        const std::map<double, std::pair<int, int>>& intervals,
        const std::vector<std::pair<double, double>>& overlaps, 
        std::map<int, Eigen::Vector3d>& normals_of_projection_plane,
        std::vector<Eigen::Vector3d>& DDs, 
        std::map<int, std::map<int, Eigen::Vector3d>>& inliers);


  bool computeTheNormalsOfProjectionPlane(const std::vector<Eigen::Vector4d>& lines, 
        std::map<int, Eigen::Vector3d>& normals_of_projection_plane);
  
  void sampleTwoVerticalInlier(const std::map<int, Eigen::Vector3d>& normals_of_projection_plane, 
        Eigen::Vector3d& vertical_sample_0, Eigen::Vector3d& vertical_sample_1);

  int estimateVerticalDDsAndInliers(std::map<int, Eigen::Vector3d>& normals_of_projection_plane,
        const Eigen::Vector3d& vertical_sample_0, const Eigen::Vector3d& vertical_sample_1, 
        Eigen::Vector3d& vertical_DD, std::map<int, Eigen::Vector3d>& vertical_inliers);

  int estimateVerticalDDsAndInliers(std::map<int, Eigen::Vector3d>& normals_of_projection_plane,
        const Eigen::Vector3d& vertical_DD, std::map<int, Eigen::Vector3d>& vertical_inliers);

  int estimateHorizontalDDsAndInliers(std::map<int, Eigen::Vector3d>& normals_of_projection_plane, 
        const std::map<int, Eigen::Vector3d>& vertical_inliers, 
        const Eigen::Vector3d& vertical_DD,
        std::vector<Eigen::Vector3d>& horizontal_DDs, 
        std::map<int, std::map<int, Eigen::Vector3d>>& horizontal_inliers);

  int estimateSloppingDDsAndInliers(std::map<int, Eigen::Vector3d>& normals_of_projection_plane, 
        const std::map<int, std::map<int, Eigen::Vector3d>>& horizontal_inliers, 
        const std::vector<Eigen::Vector3d>& horizontal_DDs,
        std::vector<Eigen::Vector3d>& slopping_DDs, 
        std::map<int, std::map<int, Eigen::Vector3d>>& slopping_inliers);


  Eigen::Vector2d cam_project(const Eigen::Vector3d& point) const; 

  void reset();

  void printInterval(double start, double end, char *str); // for debug

private:
 
  // param
  int _M; // sample a pair of normals M times for computing the vertical DD vm
  int _tau = 4; // cardinality threshold
  double _epsilon = std::sin(M_PI/180); // inner product threshold
  double _cardin_peak_thr = 2 * 2 * M_PI/180; // merge two sets of valid intervals if their corresponding in under-stabbing probes are close

  double _fx, _fy, _cx, _cy;

  // lines id, normal
  std::map<int, Eigen::Vector3d> _normals_of_projection_plane;

  // tmp
  Eigen::Vector3d _vertical_DD; 
  std::vector<Eigen::Vector3d> _horizontal_DDs;
  std::vector<Eigen::Vector3d> _slopping_DDs;

  // int: lines id in a local frame
  std::map<int, Eigen::Vector3d> _vertical_inliers;
  // first int: different horizontal DDs, second int: lines id in a local frame
  std::map<int, std::map<int, Eigen::Vector3d>> _horizontal_inliers; 
  // first int: different slopping DDs, second int: lines id in a local frame
  std::map<int, std::map<int, Eigen::Vector3d>> _slopping_inliers;

private:
  const CameraPtr _camera;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::shared_ptr<RegularityEncoder> RegularityEncoderPtr;

#endif // REGULARITY_ENCODER_H_
