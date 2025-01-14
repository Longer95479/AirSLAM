#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/types/slam3d/isometry3d_mappings.h>

#include "utils.h"
#include "g2o_optimization/edge_project_point_td.h"

#define OPEN_EXTRINSIC_ESTIMATE

// monocular point
EdgeSE3ProjectPointTd::EdgeSE3ProjectPointTd()
    : g2o::BaseMultiEdge<2, Eigen::Vector2d>() {
#ifdef OPEN_EXTRINSIC_ESTIMATE
  resize(4);
#else
  resize(3);
#endif
}

bool EdgeSE3ProjectPointTd::read(std::istream &is) {
  g2o::internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

bool EdgeSE3ProjectPointTd::write(std::ostream &os) const {
  g2o::internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeSE3ProjectPointTd::computeError() {
  const g2o::VertexPointXYZ *v1 =
      static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
  const VertexVIPose *v2 = static_cast<const VertexVIPose *>(_vertices[1]);
  const VertexTd *v3 = static_cast<const VertexTd *>(_vertices[2]);
  Eigen::Vector2d obs(_measurement);
#ifdef OPEN_EXTRINSIC_ESTIMATE
  const VertexExtrinsic *v4 = static_cast<const VertexExtrinsic *>(_vertices[3]);
  Eigen::Matrix3d Rbw = v2->estimate().Rwb.transpose();
  Eigen::Vector3d tbw = -Rbw * v2->estimate().twb;

  _error = (obs - (v3->estimate() -  td_used) * velocity) +
    - cam_project(v4->estimate().Rcb * (Rbw * v1->estimate() + tbw) + v4->estimate().tcb);
#else
  _error = (obs - (v3->estimate() -  td_used) * velocity) +
    - cam_project(v2->estimate().Rcw * v1->estimate() + v2->estimate().tcw);
#endif
}

bool EdgeSE3ProjectPointTd::isDepthPositive(){
  const g2o::VertexPointXYZ *v1 =
      static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
  const VertexVIPose *v2 = static_cast<const VertexVIPose *>(_vertices[1]);
  return (v2->estimate().Rcw * v1->estimate() + v2->estimate().tcw)(2) > 0;
}

Eigen::Vector2d EdgeSE3ProjectPointTd::cam_project(const Eigen::Vector3d& point) const {
  double z_inv = 1.0 / point(2);
  Eigen::Vector2d point_2d;
  point_2d(0) = point(0) * z_inv * fx + cx;
  point_2d(1) = point(1) * z_inv * fy + cy;
  return point_2d;
}

// Eigen::Matrix<double, 2, 3> EdgeSE3ProjectPointTd::cam_projection_jacobian(const Eigen::Vector3d &point) const {
//   Eigen::Matrix<double, 2, 3> jac;
//   jac(0, 0) = fx / point[2];
//   jac(0, 1) = 0.;
//   jac(0, 2) = -fx * point[0] / (point[2] * point[2]);
//   jac(1, 0) = 0.;
//   jac(1, 1) = fy / point[2];
//   jac(1, 2) = -fy * point[1] / (point[2] * point[2]);
//   return jac;
// }

// void EdgeSE3ProjectPointTd::linearizeOplus(){

//   const g2o::VertexPointXYZ *v1 =
//       static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
//   const VertexVIPose *v2 = static_cast<const VertexVIPose *>(_vertices[1]);

//   const Eigen::Matrix3d& Rcw = v2->estimate().Rcw;
//   const Eigen::Matrix3d& Rcb = v2->estimate().Rcb;
//   const Eigen::Vector3d& tcw = v2->estimate().tcw;

//   const Eigen::Vector3d Xc = Rcw * v1->estimate() + tcw;
//   const Eigen::Vector3d Xb = v2->estimate().Rbc * Xc + v2->estimate().tbc;

//   const Eigen::Matrix<double,2,3> projection_jacobian = cam_projection_jacobian(Xc);
//   _jacobianOplusXi = -projection_jacobian * Rcw;

//   Eigen::Matrix<double,3,6> SE3deriv;
//   double x = Xb(0);
//   double y = Xb(1);
//   double z = Xb(2);

//   SE3deriv <<  0.0,   z,  -y, 1.0, 0.0, 0.0,
//                 -z , 0.0,   x, 0.0, 1.0, 0.0,
//                 y ,  -x, 0.0, 0.0, 0.0, 1.0;

//   _jacobianOplusXj = projection_jacobian * Rcb * SE3deriv;
// }


// stereo point
EdgeSE3ProjectStereoPointTd::EdgeSE3ProjectStereoPointTd()
    : g2o::BaseMultiEdge<3, Eigen::Vector3d>() {
#ifdef OPEN_EXTRINSIC_ESTIMATE
  resize(4);
#else
  resize(3);
#endif
}

bool EdgeSE3ProjectStereoPointTd::read(std::istream &is) {
  g2o::internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

bool EdgeSE3ProjectStereoPointTd::write(std::ostream &os) const {
  g2o::internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeSE3ProjectStereoPointTd::computeError() {
  const g2o::VertexPointXYZ *v1 =
      static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
  const VertexVIPose *v2 = static_cast<const VertexVIPose *>(_vertices[1]);
  const VertexTd *v3 = static_cast<const VertexTd *>(_vertices[2]);
  Eigen::Vector3d obs(_measurement);
  Eigen::Vector3d delta_uv;
  delta_uv.block<2, 1>(0, 0) = (v3->estimate() -  td_used) * velocity;
  delta_uv(2) = delta_uv(0);
#ifdef OPEN_EXTRINSIC_ESTIMATE
  const VertexExtrinsic *v4 = static_cast<const VertexExtrinsic *>(_vertices[3]);
  Eigen::Matrix3d Rbw = v2->estimate().Rwb.transpose();
  Eigen::Vector3d tbw = -Rbw * v2->estimate().twb;

  _error = (obs - delta_uv) - 
            cam_project(v4->estimate().Rcb * (Rbw * v1->estimate() + tbw) + v4->estimate().tcb);
#else
  _error = (obs - delta_uv) - cam_project(v2->estimate().Rcw * v1->estimate() + v2->estimate().tcw);
#endif
}

bool EdgeSE3ProjectStereoPointTd::isDepthPositive(){
  const g2o::VertexPointXYZ *v1 =
      static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
  const VertexVIPose *v2 = static_cast<const VertexVIPose *>(_vertices[1]);
  return (v2->estimate().Rcw * v1->estimate() + v2->estimate().tcw)(2) > 0;
}

Eigen::Vector3d EdgeSE3ProjectStereoPointTd::cam_project(const Eigen::Vector3d& point) const {
  double z_inv = 1.0 / point(2);
  Eigen::Vector3d point_2d;
  point_2d(0) = point(0) * z_inv * fx + cx;
  point_2d(1) = point(1) * z_inv * fy + cy;
  point_2d(2) = point_2d(0) - bf * z_inv;
  return point_2d;
}

// Eigen::Matrix<double, 2, 3> EdgeSE3ProjectStereoPointTd::cam_projection_jacobian(const Eigen::Vector3d &point) const {
//   Eigen::Matrix<double, 2, 3> jac;
//   jac(0, 0) = fx / point[2];
//   jac(0, 1) = 0.;
//   jac(0, 2) = -fx * point[0] / (point[2] * point[2]);
//   jac(1, 0) = 0.;
//   jac(1, 1) = fy / point[2];
//   jac(1, 2) = -fy * point[1] / (point[2] * point[2]);
//   return jac;
// }

// void EdgeSE3ProjectStereoPointTd::linearizeOplus(){

//   const g2o::VertexPointXYZ *v1 =
//       static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
//   const VertexVIPose *v2 = static_cast<const VertexVIPose *>(_vertices[1]);

//   const Eigen::Matrix3d& Rcw = v2->estimate().Rcw;
//   const Eigen::Matrix3d& Rcb = v2->estimate().Rcb;
//   const Eigen::Vector3d& tcw = v2->estimate().tcw;

//   const Eigen::Vector3d Xc = Rcw * v1->estimate() + tcw;
//   const Eigen::Vector3d Xb = v2->estimate().Rbc * Xc + v2->estimate().tbc;
//   const double inv_z2 = 1.0 / (Xc(2) * Xc(2));


//   Eigen::Matrix<double,3,3> projection_jacobian;
//   projection_jacobian.block<2,3>(0,0) = cam_projection_jacobian(Xc);
//   projection_jacobian.block<1,3>(2,0) = projection_jacobian.block<1,3>(0,0);
//   projection_jacobian(2,2) += bf * inv_z2;
//   _jacobianOplusXi = -projection_jacobian * Rcw;

//   Eigen::Matrix<double,3,6> SE3deriv;
//   double x = Xb(0);
//   double y = Xb(1);
//   double z = Xb(2);

//   SE3deriv <<  0.0,   z,  -y, 1.0, 0.0, 0.0,
//                 -z , 0.0,   x, 0.0, 1.0, 0.0,
//                 y ,  -x, 0.0, 0.0, 0.0, 1.0;

//   _jacobianOplusXj = projection_jacobian * Rcb * SE3deriv;
// }
