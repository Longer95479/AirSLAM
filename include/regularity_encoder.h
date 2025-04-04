#ifndef REGULARITY_ENCODER_H_
#define REGULARITY_ENCODER_H_

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <map>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include "camera.h"

#include "utils.h"
#include "read_configs.h"


struct Polar {
  Polar() {}
  Polar(double x, double y) {
    _range = std::sqrt(x*x + y*y);
    _angle = std::atan(y/x);
  }
  friend std::ostream& operator<<(std::ostream& os, const Polar& p); 

  double _range; 
  double _angle;
};


class GlobalDD {
public:
  GlobalDD(){}
  GlobalDD(const Eigen::Vector3d &vec, const Eigen::Matrix3d &cov): _gDD(vec), _cov(cov) {
    _is_init = true;
  }

  bool isInit() {return _is_init;}
  long long getUpdateCnt() const {return _update_cnt;}
  void initDD(const Eigen::Vector3d &prior, const Eigen::Matrix3d &cov);
  void updateDD(const Eigen::Vector3d &obs, const Eigen::Matrix3d &cov_obs);

  Eigen::Vector3d _gDD;
  Eigen::Matrix3d _cov;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  bool _is_init = false;
  long long _update_cnt = 0;
};


class RegularityEncoder {
public:
  RegularityEncoder(const CameraPtr camera, double P_of_vertival_inlier, double P_of_all_inlier, const RegularityEncoderConfig& cfg);

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

  void printNormalInliers(const std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>>& inliers, 
                          const std::map<int, std::map<int, int>>& lDDtype_lDDid_gDDid);

  void printLineInliers(const std::map<int, std::map<int, std::map<int, Eigen::Vector4d>>>& inliers);

  void getLineIdMapToGDDId(const std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>>& inliers, 
                           const std::map<int, std::map<int, int>>& lDDtype_lDDid_gDDid, 
                           std::vector<std::pair<int, int>> *lines_gDD_ptr);

  void printLineIdGDDId(const std::vector<std::pair<int, int>> &lines_gDD);

  // global DDs
  void refineGlobalDDs(const std::map<int, std::vector<Eigen::Vector3d>> &local_DDs, 
                       const Eigen::Matrix3d &Rwc, 
                       std::shared_ptr<std::map<int, std::map<int, int>>> lDDtype_lDDid_gDDid);

  void printGlobalDDs();

  inline const GlobalDD& getVerticalGDD() const { return _vertical_gDD; }
  inline const std::vector<GlobalDD>& getHorizontalGDDs() const { return _horizontal_gDDs; }
  inline const std::vector<GlobalDD>& getSloppingGDDs() const { return _slopping_gDDs; }

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
        std::vector<Eigen::Vector3d>& horizontal_DDs, std::map<int, std::map<int, Eigen::Vector3d>>& horizontal_inliers);

  int estimateSloppingDDsAndInliers(std::map<int, Eigen::Vector3d>& normals_of_projection_plane, 
        const std::map<int, std::map<int, Eigen::Vector3d>>& horizontal_inliers, 
        const std::vector<Eigen::Vector3d>& horizontal_DDs,
        std::vector<Eigen::Vector3d>& slopping_DDs, 
        std::map<int, std::map<int, Eigen::Vector3d>>& slopping_inliers);


  Eigen::Vector2d cam_project(const Eigen::Vector3d& point) const; 

  void reset();

  void printInterval(double start, double end, char *str, int id); // for debug

  // global DDs
  void initGlobalDDs(const std::map<int, std::vector<Eigen::Vector3d>> &local_DDs,
                     const Eigen::Matrix3d &Rwc,
                     std::shared_ptr<std::map<int, std::map<int, int>>> lDDtype_lDDid_gDDid);


private:
 
  // param
  int _M; // sample a pair of normals M times for computing the vertical DD vm
  int _tau = 5; // cardinality threshold
  double _epsilon = std::sin(M_PI/180); // inner product threshold
  double _cardin_peak_thr = 9 * M_PI/180; // merge two sets of valid intervals if their corresponding in under-stabbing probes are close

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


  // global DDs, will be maintained in a longlife way
  GlobalDD _vertical_gDD;
  std::vector<GlobalDD> _horizontal_gDDs;
  std::vector<GlobalDD> _slopping_gDDs;

  bool _gDDs_init = false;


private:
  const CameraPtr _camera;
  RegularityEncoderConfig _config;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::shared_ptr<RegularityEncoder> RegularityEncoderPtr;

#endif // REGULARITY_ENCODER_H_
