#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/types/slam3d/isometry3d_mappings.h>

#include "utils.h"
#include "g2o_optimization/edge_extrinsic_prior.h"

void EdgeExtrinsicPrior::computeError(){
  const VertexExtrinsic* ve1= static_cast<const VertexExtrinsic*>(_vertices[0]);
  Eigen::Vector3d er, et;
  
  SO3Log(_Rcb.transpose() * ve1->estimate().Rcb, er);
  et = ve1->estimate().tcb - _tcb;

  _error << er, et;

  // std::cout << "---[debug extrinsic computeError]---"  << std::endl;
  // std::cout << "ve1->estimate().Rcb = " << std::endl;
  // std::cout << ve1->estimate().Rcb << std::endl;
  // std::cout << "ve1->estimate().tcb" << ve1->estimate().tcb.transpose() << std::endl;
  // std::cout << "---" << std::endl;
  // std::cout << "_Rcb = " << std::endl;
  // std::cout << _Rcb << std::endl;
  // std::cout << "_tcb = " << _tcb.transpose() << std::endl;
  // std::cout << "_error = " << _error.transpose() << std::endl;
  // std::cout << "------------------------------------"  << std::endl;
}

