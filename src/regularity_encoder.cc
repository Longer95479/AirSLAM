#include "regularity_encoder.h"

#include "time.h"


std::ostream& operator<<(std::ostream& os, const Polar& p) {
  os << "range: " << p._range << ", angle: " << p._angle;
  return os;
}


RegularityEncoder::RegularityEncoder(const CameraPtr camera, double P_of_vertival_inlier, double P_of_all_inlier): _camera(camera){
  double P = P_of_all_inlier;
  double k = P_of_vertival_inlier;
  _M = std::ceil( std::log(1 - P) / std::log(1 - k*k) );

  _fx = _camera->Fx();
  _fy = _camera->Fy();
  _cx = _camera->Cx();
  _cy = _camera->Cy();

  //DEBUG
  std::cout << "-----[RegularityEncoder Init]-----" << std::endl;
  std::cout << "P_of_vertival_inlier: " << P_of_vertival_inlier << std::endl;
  std::cout << "P_of_all_inlier" << P_of_all_inlier << std::endl;
  std::cout << "_M: " << _M << std::endl;
}

/**
 *  map of DDs: 0 for vertical, 1 for horizontal, 2 for slopping
 *
 *  map of inliers: 
 *    - 1st int: 0 for vertical, 1 for horizontal, 2 for slopping. 
 *    - 2nd int: id of a DD in a specific type. 
 *               e.g. only one vertical DD, so id is only 0
 *    - 3rd int: line id
 *
 */
bool RegularityEncoder::encode(const std::vector<Eigen::Vector4d>& lines, 
      std::map<int, std::vector<Eigen::Vector3d>>& DDs, 
      std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>>& inliers) 
{
  if (!computeTheNormalsOfProjectionPlane(lines, _normals_of_projection_plane))
    return false;

  int max_inlier_num = 0;
  int inlier_num = 0, vertical_inlier_num = 0, horizontal_inlier_num = 0, slopping_inlier_num = 0;

  Eigen::Vector3d vertical_sample_0, vertical_sample_1;

  std::srand((unsigned)time(NULL));

  for (int i = 0; i < _M; i++) {
    // DEBUG
    // std::cout << "iters: " << i << std::endl;
    std::map<int, Eigen::Vector3d> normals_of_projection_plane = _normals_of_projection_plane;
    sampleTwoVerticalInlier(normals_of_projection_plane, vertical_sample_0, vertical_sample_1);

    vertical_inlier_num = estimateVerticalDDsAndInliers(normals_of_projection_plane, 
                                                        vertical_sample_0, vertical_sample_1, 
                                                        _vertical_DD, _vertical_inliers);
    
    horizontal_inlier_num = estimateHorizontalDDsAndInliers(normals_of_projection_plane, 
                                                            _vertical_inliers, _vertical_DD,
                                                            _horizontal_DDs, _horizontal_inliers);

    // slopping_inlier_num = estimateSloppingDDsAndInliers(normals_of_projection_plane, _horizontal_inliers[0], 
    //                                                     _slopping_DDs, _slopping_inliers);

    inlier_num = vertical_inlier_num + horizontal_inlier_num + slopping_inlier_num; 


    if (inlier_num > max_inlier_num) {
      
      DDs.clear();
      inliers.clear();

      DDs[0].push_back(_vertical_DD);
      DDs[1] = _horizontal_DDs; 
      DDs[2] = _slopping_DDs;

      inliers[0].insert(std::make_pair(0, _vertical_inliers));
      inliers[1] = _horizontal_inliers;
      inliers[2] = _slopping_inliers;

      max_inlier_num = inlier_num;

      // DEBUG
      std::cout << "-------------------------------" << std::endl;
      std::cout << "[Regularity Encoder Debug Info]" << std::endl;
      std::cout << "iters: " << i << std::endl;
      std::cout << std::endl;
      std::cout << "line_num: " << lines.size() << std::endl;
      std::cout << "max_inlier_num: " << max_inlier_num << std::endl;
      std::cout << "inlier_num: " << inlier_num << std::endl;
      std::cout << "- vertical_inlier_num: " << vertical_inlier_num << std::endl;
      std::cout << "- horizontal_inlier_num: " << horizontal_inlier_num << std::endl;
      std::cout << "- slopping_inlier_num: " << slopping_inlier_num << std::endl;
      std::cout << std::endl;
      std::cout << "hDD_num: " << _horizontal_DDs.size() << std::endl;
      std::cout << "sDD_num: " << _slopping_DDs.size() << std::endl;
      std::cout << "-------------------------------" << std::endl;
      // DEBUG END


    }
    
    _vertical_DD = Eigen::Vector3d::Zero();
    _horizontal_DDs.clear();
    _slopping_DDs.clear();

    _vertical_inliers.clear();
    _horizontal_inliers.clear();
    _slopping_inliers.clear();

  }

  reset();

  return true;
}


/**
 *  Gravity direction version
 *
 *  map of DDs: 0 for vertical, 1 for horizontal, 2 for slopping
 *
 *  map of inliers: 
 *    - 1st int: 0 for vertical, 1 for horizontal, 2 for slopping. 
 *    - 2nd int: id of a DD in a specific type. 
 *               e.g. only one vertical DD, so id is only 0
 *    - 3rd int: line id
 *
 */
bool RegularityEncoder::encode(const std::vector<Eigen::Vector4d>& lines, 
      const Eigen::Vector3d& g_dir,
      std::map<int, std::vector<Eigen::Vector3d>>& DDs, 
      std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>>& inliers) 
{
  if (!computeTheNormalsOfProjectionPlane(lines, _normals_of_projection_plane))
    return false;

  int max_inlier_num = 0;
  int inlier_num = 0, vertical_inlier_num = 0, horizontal_inlier_num = 0, slopping_inlier_num = 0;

  Eigen::Vector3d g_dir_normed = g_dir / g_dir.norm();
  _vertical_DD = g_dir_normed;

  // std::srand((unsigned)time(NULL));

  // DEBUG
  // std::cout << "iters: " << i << std::endl;
  std::map<int, Eigen::Vector3d> normals_of_projection_plane = _normals_of_projection_plane;

  vertical_inlier_num = estimateVerticalDDsAndInliers(normals_of_projection_plane, 
                                                      _vertical_DD, _vertical_inliers);

  horizontal_inlier_num = estimateHorizontalDDsAndInliers(normals_of_projection_plane, 
                                                          _vertical_inliers, _vertical_DD,
                                                          _horizontal_DDs, _horizontal_inliers);

  // slopping_inlier_num = estimateSloppingDDsAndInliers(normals_of_projection_plane, _horizontal_inliers[0], 
  //                                                     _slopping_DDs, _slopping_inliers);

  inlier_num = vertical_inlier_num + horizontal_inlier_num + slopping_inlier_num; 
    
  DDs.clear();
  inliers.clear();

  DDs[0].push_back(_vertical_DD);
  DDs[1] = _horizontal_DDs; 
  DDs[2] = _slopping_DDs;

  inliers[0].insert(std::make_pair(0, _vertical_inliers));
  inliers[1] = _horizontal_inliers;
  inliers[2] = _slopping_inliers;

  // DEBUG
  std::cout << "-------------------------------" << std::endl;
  std::cout << "[Regularity Encoder Debug Info]" << std::endl;
  std::cout << "line_num: " << lines.size() << std::endl;
  std::cout << "inlier_num: " << inlier_num << std::endl;
  std::cout << "- vertical_inlier_num: " << vertical_inlier_num << std::endl;
  std::cout << "- horizontal_inlier_num: " << horizontal_inlier_num << std::endl;
  std::cout << "- slopping_inlier_num: " << slopping_inlier_num << std::endl;
  std::cout << std::endl;
  std::cout << "hDD_num: " << _horizontal_DDs.size() << std::endl;
  std::cout << "sDD_num: " << _slopping_DDs.size() << std::endl;
  std::cout << "-------------------------------" << std::endl;
  // DEBUG END
  
  _vertical_DD = Eigen::Vector3d::Zero();
  _horizontal_DDs.clear();
  _slopping_DDs.clear();

  _vertical_inliers.clear();
  _horizontal_inliers.clear();
  _slopping_inliers.clear();

  reset();

  return true;
}


bool RegularityEncoder::computeTheNormalsOfProjectionPlane(const std::vector<Eigen::Vector4d>& lines, 
      std::map<int, Eigen::Vector3d>& normals_of_projection_plane)
{
  Eigen::Vector2d ep0_2d, ep1_2d; //endpoints of 2D line
  Eigen::Vector3d ep0_3d, ep1_3d; //endpoints of 3D line
  Eigen::Vector3d normal;

  for (int i = 0; i < lines.size(); i++) {
    ep0_2d = lines[i].block<2, 1>(0, 0);
    ep1_2d = lines[i].block<2, 1>(2, 0);
    _camera->BackProjectMono(ep0_2d, ep0_3d);    
    _camera->BackProjectMono(ep1_2d, ep1_3d);    
    normal = ep0_3d.cross(ep1_3d);
    normal = normal / normal.norm();
    normals_of_projection_plane.insert(std::make_pair(i, normal));
  }

  if (normals_of_projection_plane.size() != lines.size())
    return false;
  else
    return true;
}


void RegularityEncoder::sampleTwoVerticalInlier(const std::map<int, Eigen::Vector3d>& normals_of_projection_plane, 
    Eigen::Vector3d& vertical_sample_0, Eigen::Vector3d& vertical_sample_1)
{
  size_t size = normals_of_projection_plane.size();

  int index0 = std::rand() % size;
  int index1 = std::rand() % size;

  while (index0 == index1) {
    index0 = std::rand() % size;
    index1 = std::rand() % size;
  }

  // DEBUG
  // std::cout << "index0: " << index0 << std::endl;
  // std::cout << "index1: " << index1 << std::endl;
  // std::cout << "------------"  << std::endl;

  vertical_sample_0 = normals_of_projection_plane.at(index0);
  vertical_sample_1 = normals_of_projection_plane.at(index1);
  Eigen::Vector3d DD = vertical_sample_0.cross(vertical_sample_1);

  while (DD.norm() < std::sin(M_PI / 4)) {
    index0 = std::rand() % size;
    index1 = std::rand() % size;
    vertical_sample_0 = normals_of_projection_plane.at(index0);
    vertical_sample_1 = normals_of_projection_plane.at(index1);
    DD = vertical_sample_0.cross(vertical_sample_1);
  }

  // still have bug
  // vertical_sample_0 = (normals_of_projection_plane.begin() + index0)->second;
  // vertical_sample_1 = (normals_of_projection_plane.begin() + index1)->second;
}


int RegularityEncoder::estimateVerticalDDsAndInliers(std::map<int, Eigen::Vector3d>& normals_of_projection_plane,
      const Eigen::Vector3d& vertical_sample_0, const Eigen::Vector3d& vertical_sample_1, 
      Eigen::Vector3d& vertical_DD, std::map<int, Eigen::Vector3d>& vertical_inliers)
{
  vertical_DD = vertical_sample_0.cross(vertical_sample_1);
  vertical_DD = vertical_DD / vertical_DD.norm();

  std::map<int, Eigen::Vector3d>::iterator it;
  for (it = normals_of_projection_plane.begin(); it != normals_of_projection_plane.end();) {
    double abs_inner_product = std::abs(vertical_DD.dot(it->second));
    if (abs_inner_product < _epsilon) {
      // DEBUG
      // std::cout << "vertical inlier lines id:" << it->first << std::endl;

      vertical_inliers.insert(std::make_pair(it->first, it->second));
      normals_of_projection_plane.erase(it++);
    }
    else 
      it++;
  }

  return vertical_inliers.size();
}


/**
 *  gravity direction version
 *  vertical_DD is the gravity direction in camera coordination
 */
int RegularityEncoder::estimateVerticalDDsAndInliers(std::map<int, Eigen::Vector3d>& normals_of_projection_plane,
      const Eigen::Vector3d& vertical_DD, std::map<int, Eigen::Vector3d>& vertical_inliers)
{
  std::map<int, Eigen::Vector3d>::iterator it;
  for (it = normals_of_projection_plane.begin(); it != normals_of_projection_plane.end();) {
    double abs_inner_product = std::abs(vertical_DD.dot(it->second));
    if (abs_inner_product < _epsilon) {
      // DEBUG
      // std::cout << "vertical inlier lines id:" << it->first << std::endl;

      vertical_inliers.insert(std::make_pair(it->first, it->second));
      normals_of_projection_plane.erase(it++);
    }
    else 
      it++;
  }

  return vertical_inliers.size();
}



int RegularityEncoder::estimateHorizontalDDsAndInliers(std::map<int, Eigen::Vector3d>& normals_of_projection_plane, 
      const std::map<int, Eigen::Vector3d>& vertical_inliers, 
      const Eigen::Vector3d& vertical_DD,
      std::vector<Eigen::Vector3d>& horizontal_DDs, 
      std::map<int, std::map<int, Eigen::Vector3d>>& horizontal_inliers)
{
  assert(vertical_inliers.size() > 0);
  
  // intervals
  // <endpoint, <0/1, normal_id>>, 0 for interval start, 1 for interval end
  std::map<double, std::pair<int, int>> intervals;
  // <left endpoint, right endpoint>
  std::vector<std::pair<double, double>> overlaps;

  Eigen::Vector3d ref_inlier = vertical_inliers.begin()->second;
  
  // compute valid intervals for all normals
  std::map<int, Eigen::Vector3d>::iterator it_normals;
  for (it_normals = normals_of_projection_plane.begin(); it_normals != normals_of_projection_plane.end();) {
    double normal_id = it_normals->first;
    Eigen::Vector3d normal = it_normals->second;

    getValidInterval(vertical_DD, ref_inlier, normal, normal_id, intervals);

    it_normals++;
  }

  getOverlapRegion(intervals, overlaps);

  getDDsAndInliers(vertical_DD, ref_inlier, intervals, overlaps, normals_of_projection_plane,
      horizontal_DDs, horizontal_inliers);


  double inliers_num = 0;
  for (auto& DDid_inlier: horizontal_inliers) {
    inliers_num += DDid_inlier.second.size();
  }

  return inliers_num;

}

int estimateSloppingDDsAndInliers(std::map<int, Eigen::Vector3d>& normals_of_projection_plane, 
      const std::map<int, std::map<int, Eigen::Vector3d>>& horizontal_inliers, 
      const std::vector<Eigen::Vector3d>& horizontal_DDs,
      std::vector<Eigen::Vector3d>& slopping_DDs, 
      std::map<int, std::map<int, Eigen::Vector3d>>& slopping_inliers) 
{
  ;
}

void RegularityEncoder::getValidInterval(const Eigen::Vector3d& last_type_DD, const Eigen::Vector3d& ref_inlier, 
        const Eigen::Vector3d& normal, const int& normal_id, 
        std::map<double, std::pair<int, int>>& intervals)
{
  Eigen::Matrix3d last_type_DD_skew;
  last_type_DD_skew << 0.0, -last_type_DD(2), last_type_DD(1),
                       last_type_DD(2), 0.0, -last_type_DD(0),
                       -last_type_DD(1), last_type_DD(0), 0.0;

  Eigen::Vector3d tmp_vec = last_type_DD_skew * normal;
  double a = ref_inlier.transpose() * last_type_DD_skew  * tmp_vec;
  double b = ref_inlier.transpose() * tmp_vec;
  double sqrt_a2_puls_b2 = std::sqrt(a*a + b*b);

  double epsilon_tilde = _epsilon / sqrt_a2_puls_b2; 

  // assert(std::abs(epsilon_tilde) <= 1.0); 
  // if (std::abs(epsilon_tilde) >= 1.0) {
  //   std::cout << "[warn]: epsilon_tilde >= 1.0" << std::endl;
  //   return;
  // }

  if (std::abs(epsilon_tilde) >= 10*_epsilon) {
    std::cout << "[warn]: valid interval is too wide." << std::endl;
    return;
  }

  double asin_epsilon_tilde = std::asin(epsilon_tilde);

  if (std::abs(b/sqrt_a2_puls_b2) < asin_epsilon_tilde) {
    std::cout << "[warn]: valid interval is not continue." << std::endl;
    return;
  }

  double atan_a_over_b = std::atan(a/b);

  double start = -asin_epsilon_tilde - atan_a_over_b;
  double end = asin_epsilon_tilde - atan_a_over_b;

  intervals.insert(std::make_pair(start, std::make_pair(0, normal_id)));
  intervals.insert(std::make_pair(end, std::make_pair(1, normal_id)));

  // DEBUG
  printInterval(start, end, "+", normal_id);

}

// interval: <endpoint_position, <0/1, normal_id>>, 
//           0 for interval start, 1 for interval end
//
// overlaps: <left_endpoint_position, right_endpoint_position> 
void RegularityEncoder::getOverlapRegion(const std::map<double, std::pair<int, int>>& intervals, 
      std::vector<std::pair<double, double>>& overlaps)
{
  int votes_num_tmp = 0;
  // <votes_num, endpoint_position>
  std::vector<std::pair<int, double>> votes_num_and_eps;

  // voting
  for (std::map<double, std::pair<int, int>>::const_iterator it = intervals.begin(); 
       it != intervals.end();) {
    // reach a left endpoint
    if (it->second.first == 0) {
      votes_num_tmp++;
      votes_num_and_eps.push_back(std::make_pair(votes_num_tmp, it->first));
    }
    // reach a right endpoint
    else {
      votes_num_tmp--;
      votes_num_and_eps.push_back(std::make_pair(votes_num_tmp, it->first));
    }

    it++;
  }

  // DEBUG
  // std::cout << "(votes_num, eps): " << std::endl;
  // for (auto& v_n_ep: votes_num_and_eps) {
  //   std::cout << "(" << v_n_ep.first << ", " << v_n_ep.second << ")  ";
  // }
  // std::cout << std::endl;
  // DEBUG


  // get overlapping intervals
  if (votes_num_and_eps.size() > 2) {
    for (std::vector<std::pair<int, double>>::iterator it = votes_num_and_eps.begin() + 1; 
         it != votes_num_and_eps.end() - 1;) {
      // votes need to be higher than cardinality threshold
      if (it->first > _tau) {
        // and also achives a local maximum
        if (it->first > (it-1)->first && it->first > (it+1)->first) {
          overlaps.push_back(std::make_pair(it->second, (it+1)->second));
        }
      }
      it++;
    }

    // DEBUG
    std::cout << "overlaps: " << std::endl;
    // for (auto& overlap: overlaps) {
    //   std::cout << "[" << overlap.first << "," << overlap.second << "]  ";
    // }
    // std::cout << std::endl;
    //
    // DEBUG
    for (auto& overlap: overlaps) {
      printInterval(overlap.first, overlap.second, "0", -1);
    }

  }
  else {
    std::cout << "[Regularity Encoder]: No enough lines." << std::endl;
  }
}


void RegularityEncoder::getDDsAndInliers(const Eigen::Vector3d& last_type_DD,
      const Eigen::Vector3d& ref_inlier, 
      const std::map<double, std::pair<int, int>>& intervals,
      const std::vector<std::pair<double, double>>& overlaps, 
      std::map<int, Eigen::Vector3d>& normals_of_projection_plane,
      std::vector<Eigen::Vector3d>& DDs, 
      std::map<int, std::map<int, Eigen::Vector3d>>& inliers)
{
  // lines id, left endpoint, right endpoint
  std::map<int, std::pair<double, double>> intervals_rerange;

  // transform intervals format:
  // <endpoint, <0/1, normal_id>>, 0 for interval start, 1 for interval end
  //  ->
  // lines id, left endpoint, right endpoint
  for (std::map<double, std::pair<int, int>>::const_iterator interval = intervals.begin();
      interval != intervals.end();) {
    int ep_type = interval->second.first;
    int line_id = interval->second.second;
    double ep_position = interval->first;

    if (ep_type == 0) {
      intervals_rerange[line_id].first = ep_position;
    }
    else {
      intervals_rerange[line_id].second = ep_position;
    }

    interval++;
  }

  // DEBUG
  // double max_width = 0;
  // std::cout << "intervals_rerange: " <<std::endl;
  // for (auto& itval_re: intervals_rerange) {
  //   double width = itval_re.second.second - itval_re.second.first; 
  //   if (width > max_width)
  //     max_width = width;

  //   // std::cout << "[" << itval_re.second.first << ", " << itval_re.second.second << "]|";
  //   // std::cout << width << "|  ";
  // }
  // std::cout << "max_interval_width: " << max_width;
  // std::cout << std::endl;
  // DEBUG

  std::list<double> thetas;
  for (std::vector<std::pair<double, double>>::const_iterator overlap = overlaps.begin(); 
      overlap != overlaps.end();) {
    double theta = (overlap->first + overlap->second) / 2;
    thetas.push_back(theta);

    overlap++;
  }

  thetas.sort();

  // merge two theta if they are close
  double cnt = 1.0;
  for (std::list<double>::iterator theta = thetas.begin();
      theta != thetas.end();) {

    if (++theta == thetas.end())
      break;
    theta--;

    double t1 = *theta;
    theta++;
    double t2 = *theta;
    if (std::abs(t1 - t2) < _cardin_peak_thr) {
      double tmp = (t1 * cnt + t2) / (cnt + 1);
      cnt += 1.0;

      theta++;
      thetas.insert(theta, tmp);
      theta--;
      theta--;
      theta--;
      theta = thetas.erase(theta);
      theta = thetas.erase(theta);
    }
    else {
      cnt = 1;
    }
  }
  
  //
  //
  //

  int DDs_id = 0;
  for (std::list<double>::iterator it_theta = thetas.begin(); 
      it_theta != thetas.end();) {

    double theta = *it_theta;
    // DEBUG 
    std::cout << "merged thetas: " << std::endl;
    printInterval(theta, theta, "o", -1);
    
    // reduce inormals_of_projection_plane 
    // get normal inliers
    std::cout << "picked interval: " << std::endl;
    for (std::map<int, std::pair<double, double>>::iterator it = intervals_rerange.begin();
        it != intervals_rerange.end();) {
      int line_id = it->first;
      double left_ep = it->second.first;
      double right_ep = it->second.second;

      if (theta >= left_ep && theta < right_ep) {
        // DEBUG
        printInterval(left_ep, right_ep, "p", line_id);

        inliers[DDs_id].insert(std::make_pair(line_id, normals_of_projection_plane[line_id])); 
        normals_of_projection_plane.erase(line_id);
        it = intervals_rerange.erase(it);
      }
      else {
        it++;
      }

    }

    // get DDs
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(theta, last_type_DD);
    DDs.push_back(R*ref_inlier);
    DDs_id++;

    it_theta++;
  }

  //
  //
  //
}


void RegularityEncoder::reset()
{
  _normals_of_projection_plane.clear();

  _vertical_DD = Eigen::Vector3d::Zero();
  _horizontal_DDs.clear();
  _slopping_DDs.clear();

  _vertical_inliers.clear();
  _horizontal_inliers.clear();
  _slopping_inliers.clear();
}


void RegularityEncoder::projectDDsOnImage(const std::map<int, std::vector<Eigen::Vector3d>>& DDs, 
      std::map<int, std::vector<Eigen::Vector4d>>& DDs_on_image)
{
  // traverse vertical, horizontal, slopping DDs
  for (std::map<int, std::vector<Eigen::Vector3d>>::const_iterator DDs_one_type = DDs.begin();
      DDs_one_type != DDs.end();) {
    if (DDs_one_type->second.size() > 0) {
      int type_of_DD = DDs_one_type->first;
      std::vector<Eigen::Vector4d> DDs_one_type_on_image;

      int size = DDs_one_type->second.size();
      // traverse vertical/horizontal/slopping DD
      for (int i = 0; i < size; i++) {
        Eigen::Vector4d DD_on_image;
        DD_on_image.block<2, 1>(0, 0) = cam_project(Eigen::Vector3d(0, 0, 1)); 
        Eigen::Vector3d DD = DDs_one_type->second[i];
        DD_on_image.block<2, 1>(2, 0) = cam_project(DD); 

        DDs_one_type_on_image.push_back(DD_on_image);
      }

      DDs_on_image.insert(std::make_pair(type_of_DD, DDs_one_type_on_image));
    }
    
    DDs_one_type++;
  }

}


void RegularityEncoder::normalInliers2LineInliers(const std::vector<Eigen::Vector4d>& lines, 
      const std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>>& normal_inliers, 
      std::map<int, std::map<int, std::map<int, Eigen::Vector4d>>>& line_inliers)
{
  for (std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>>::const_iterator n_inliers = normal_inliers.begin();
      n_inliers != normal_inliers.end();) {

    int type_of_DD = n_inliers->first;
    std::map<int, std::map<int, Eigen::Vector3d>> dir_lineid_normals = n_inliers->second;
    std::map<int, std::map<int, Eigen::Vector4d>> dir_lineid_lines;

    for (auto& dir_lineid_normal: dir_lineid_normals) {
      int id_of_DD = dir_lineid_normal.first;
      std::map<int, Eigen::Vector3d> lineid_normals = dir_lineid_normal.second;
      std::map<int, Eigen::Vector4d> lineid_lines;

      for (auto& lineid_normal: lineid_normals) {
        int line_id = lineid_normal.first;
        Eigen::Vector4d line_inlier = lines[line_id];
        lineid_lines.insert(std::make_pair(line_id, line_inlier));
      }

      dir_lineid_lines.insert(std::make_pair(id_of_DD, lineid_lines));
    }

    line_inliers.insert(std::make_pair(type_of_DD, dir_lineid_lines));

    n_inliers++; 
  }

}


Eigen::Vector2d RegularityEncoder::cam_project(const Eigen::Vector3d& point) const {
  double z_inv = 1.0 / point(2);
  Eigen::Vector2d point_2d;
  point_2d(0) = point(0) * z_inv * _fx + _cx;
  point_2d(1) = point(1) * z_inv * _fy + _cy;
  return point_2d;
}



      

void RegularityEncoder::printNormalInliers(const std::map<int, std::map<int, std::map<int, Eigen::Vector3d>>>& inliers)
{
  std::cout << "print normal inliers: " << std::endl;
  for (auto& inlier: inliers) {
    std::cout << "- type of DD: " << inlier.first << std::endl;
    auto& DDid_lineid_normals = inlier.second;
    for (auto& DDid_lineid_normal: DDid_lineid_normals) {
      std::cout << "  - DD id: " << DDid_lineid_normal.first << std::endl;
      std::cout << "    - line id: ";
      auto& lineid_normals = DDid_lineid_normal.second;
      for (auto& lineid_normal: lineid_normals) {
        std::cout << lineid_normal.first << " ";
      }
      std::cout << std::endl;
    }
  }

}


void RegularityEncoder::printLineInliers(const std::map<int, std::map<int, std::map<int, Eigen::Vector4d>>>& inliers)
{
  std::cout << "print line inliers: " << std::endl;
  for (auto& inlier: inliers) {
    std::cout << "- type of DD: " << inlier.first << std::endl;
    auto& DDid_lineid_normals = inlier.second;
    for (auto& DDid_lineid_normal: DDid_lineid_normals) {
      std::cout << "  - DD id: " << DDid_lineid_normal.first << std::endl;
      std::cout << "    - line id: ";
      auto& lineid_normals = DDid_lineid_normal.second;
      for (auto& lineid_normal: lineid_normals) {
        std::cout << lineid_normal.first << " ";
      }
      std::cout << std::endl;
    }
  }

}


void RegularityEncoder::printInterval(double start, double end, char *str, int id)
{
  int st = (int)((start + M_PI/2.0) * 50.0);
  int ed = (int)((end + M_PI/2.0) * 50.0);

  for (int i = 0; i < 157; i++) {
    if (i < st) {
      printf("-");
    }
    else if (i >= st && i <= ed) {
      printf("%s", str);
    }
    else if (i > ed) {
      printf("-");
    }
  }
  printf("\t(%f, %f)\tid:%d", start, end, id);
  printf("\n");
}




void RegularityEncoder::refineGlobalDDs(const std::map<int, std::vector<Eigen::Vector3d>> &local_DDs, 
                                        const Eigen::Matrix3d &Rwc)
{
  if (_gDDs_init == false) {
    // use first bunch of lDDs to init
    initGlobalDDs(local_DDs, Rwc);
    _gDDs_init = true;
    return;
  }

  // travel all local DDs
  //   if: lDD align with existed gDD, update gDD
  //   else if: lDD doesn't align any existed gDDs, creat gDD
  std::map<int, std::vector<Eigen::Vector3d>>::const_iterator it = local_DDs.begin();
  for (; it != local_DDs.end(); it++) {
    int DD_type = it->first;
    std::vector<Eigen::Vector3d> lDDs = it->second;

    if (DD_type == 0) { // vertical DDs
      // travel all lDD
      for (auto &lDD: lDDs) {
        Eigen::Vector3d gDD_measure = Rwc * lDD;
        Eigen::Vector3d &vgDD = _vertical_gDD._gDD;

        // compare with the exist vertical DD
        double inner_product = vgDD.dot(gDD_measure);
        double abs_inner_product = std::abs(inner_product);
        if (abs_inner_product > 0.96) { // this measurement is belong to this gDD
          if (inner_product < 0) {
            gDD_measure = -gDD_measure;
          }
          _vertical_gDD.updateDD(gDD_measure, Eigen::Matrix3d::Identity());
        }
      }
    }
    else if (DD_type == 1) { // horizental DDs
      // travel all lDD
      for (auto &lDD: lDDs) {
        Eigen::Vector3d gDD_measure = Rwc * lDD;

        // Not observe any horiznetal global DD yet, 
        // just create a new horizental gloabal DD
        if (_horizontal_gDDs.size() == 0) {
          GlobalDD new_hori_gDD(gDD_measure, Eigen::Matrix3d::Identity());
          _horizontal_gDDs.push_back(new_hori_gDD);
          continue;
        }
 
        // compare with all horizental global DD
        bool is_align = false;
        for (auto &hori_gDD: _horizontal_gDDs) {
          Eigen::Vector3d &hgDD = hori_gDD._gDD;

          // compare
          double inner_product = hgDD.dot(gDD_measure);
          double abs_inner_product = std::abs(inner_product);
          if (abs_inner_product > 0.96) { // align exist horizental global DD
            if (inner_product < 0) {
              gDD_measure = -gDD_measure;
            }
            hori_gDD.updateDD(gDD_measure, Eigen::Matrix3d::Identity());
            is_align = true;
            break;
          }
        }

        if (!is_align) { // create a new horizental global DD
          GlobalDD new_hori_gDD(gDD_measure, Eigen::Matrix3d::Identity());
          _horizontal_gDDs.push_back(new_hori_gDD);
        }

      }
    }
    else if (DD_type == 2) { // slop DDs

    }
  }

}


void RegularityEncoder::initGlobalDDs(const std::map<int, std::vector<Eigen::Vector3d>> &local_DDs,
                                      const Eigen::Matrix3d &Rwc)
{
  std::map<int, std::vector<Eigen::Vector3d>>::const_iterator it = local_DDs.begin();

  for (;it != local_DDs.end(); it++) {
    int lDD_type = it->first;
    std::vector<Eigen::Vector3d> DDs = it->second;

    if (lDD_type == 0) {
      Eigen::Vector3d gDD_tmp = Rwc * DDs[0];
      _vertical_gDD.initDD(gDD_tmp, Eigen::Matrix3d::Identity());
    }
    else if (lDD_type == 1) {
      for (auto &DD: DDs) {
        Eigen::Vector3d gDD_tmp = Rwc * DD;
        GlobalDD tmp(gDD_tmp, Eigen::Matrix3d::Identity());
        _horizontal_gDDs.push_back(tmp);
      }
    }
    else if (lDD_type == 2) {

    }
  }
}


void RegularityEncoder::printGlobalDDs()
{
  std::cout << "[print global DDs]" << std::endl;

  std::cout << "- vertical global DD:" << std::endl;
  std::cout << "val: " << _vertical_gDD._gDD.transpose() << std::endl;
  std::cout << "cov: " << std::endl;
  std::cout << _vertical_gDD._cov << std::endl;

  std::cout << "- horizental global DD:" << std::endl;
  std::cout << "size: " << _horizontal_gDDs.size() << std::endl;
  int i = 0;
  for (auto &hgDD: _horizontal_gDDs) {
    std::cout << "update cnt: " << hgDD.getUpdateCnt() << std::endl;

    std::cout << "val" << i << ": " << hgDD._gDD.transpose() << std::endl;
    Polar gDD_polar(hgDD._gDD(0), hgDD._gDD(1));
    std::cout << gDD_polar << std::endl;

    std::cout << "cov" << i << ": " << std::endl;
    std::cout << hgDD._cov << std::endl;

    std::cout << std::endl;
    i++;
  }
}


void GlobalDD::initDD(const Eigen::Vector3d &prior, const Eigen::Matrix3d &cov)
{
  _gDD = prior;
  _cov = cov;
  _is_init = true;
}


void GlobalDD::updateDD(const Eigen::Vector3d &obs, const Eigen::Matrix3d &cov_obs)
{
  Eigen::Matrix3d gain;
  gain = _cov * (cov_obs + _cov).inverse();
  _gDD = _gDD + gain * (obs - _gDD);
  _gDD = _gDD / _gDD.norm();
  _cov = (Eigen::Matrix3d::Identity() - gain) * _cov;
  _update_cnt++;
}



