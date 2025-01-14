#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/g2o_types_sba_api.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d_addons/line3d.h>

#include "utils.h"
#include "imu.h"
#include "g2o_optimization/vertex_extrinsic.h"

Extrinsic::Extrinsic() : n_it(0){
  Rcb.setIdentity();
  tcb.setZero();
}

Extrinsic::Extrinsic(const Eigen::Matrix3d& Rcb_, const Eigen::Vector3d& tcb_) : n_it(0){
  SetParam(Rcb_, tcb_);
}


Extrinsic& Extrinsic::operator =(const Extrinsic& other){
  Rcb = other.Rcb;
  tcb = other.tcb;
  n_it = other.n_it;
  return *this;
}

void Extrinsic::SetParam(const Eigen::Matrix3d& Rcb_, const Eigen::Vector3d& tcb_){
  Rcb = Rcb_;
  tcb = tcb_;
}

void Extrinsic::Update(const double* v){
  Eigen::Vector3d dr, dt;
  for(size_t i = 0; i < 3; ++i){
    dr(i) = v[i];
    dt(i) = v[i+3];
  } 

  Eigen::Matrix3d dR;
  SO3Exp(dr, dR);

  // TODO test
  // Eigen::Matrix3d Rbc = Rcb.transpose();
  // Eigen::Vector3d tbc = -Rbc * tcb;
  // tbc += Rbc * dt;
  // Rbc = Rbc * dR;
  // Rcb = Rbc.transpose();
  // tcb = -Rcb * tbc;

  tcb += Rcb * dt;
  Rcb = Rcb * dR;

  n_it++;
  if(n_it >= 3){
    Rcb = NormalizeRotation(Rcb);
    n_it = 0;
  }
  
}

VertexExtrinsic::VertexExtrinsic() : g2o::BaseVertex<6, Extrinsic>(){
}

bool VertexExtrinsic::read(std::istream& is) {
  return false; 
}

bool VertexExtrinsic::write(std::ostream& os) const {
  return false; 
}
