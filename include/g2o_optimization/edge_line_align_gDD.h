#ifndef EDGE_LINE_ALIGN_GDD_H_
#define EDGE_LINE_ALIGN_GDD_H_ 

#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/g2o_types_sba_api.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d_addons/line3d.h>

#include "utils.h"
#include "g2o_optimization/vertex_line3d.h"

class EdgeLineAlignGDD:
    public g2o::BaseUnaryEdge<1, Eigen::Vector3d, VertexLine3D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeLineAlignGDD() {};

  virtual bool read(std::istream& is){return false;}
  virtual bool write(std::ostream& os) const{return false;}
  void computeError();
};

#endif  // EDGE_LINE_ALIGN_GDD_H_ 
