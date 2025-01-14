#ifndef EDGE_EXTRINSIC_PRIOR_H_
#define EDGE_EXTRINSIC_PRIOR_H_

#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/g2o_types_sba_api.h>
#include <g2o/types/sba/vertex_se3_expmap.h>

#include "utils.h"
#include "imu.h"
#include "g2o_optimization/vertex_extrinsic.h"


class EdgeExtrinsicPrior : public g2o::BaseUnaryEdge<6, Vector6d, VertexExtrinsic>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeExtrinsicPrior(const Eigen::Matrix3d& Rcb, const Eigen::Vector3d& tcb): _Rcb(Rcb), _tcb(tcb){}


  virtual bool read(std::istream& is){return false;}
  virtual bool write(std::ostream& os) const{return false;}

  void computeError();

  Eigen::Matrix3d _Rcb;
  Eigen::Vector3d _tcb;
};

#endif  // EDGE_EXTRINSIC_PRIOR_H_
