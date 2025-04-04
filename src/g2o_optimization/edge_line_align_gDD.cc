#include "g2o_optimization/edge_line_align_gDD.h"

void EdgeLineAlignGDD::computeError()
{
  const VertexLine3D* v1 = static_cast<const VertexLine3D*>(_vertices[0]);
  Eigen::Vector3d obs(_measurement);
  Eigen::Vector3d d = v1->estimate().d();
  Eigen::Vector3d crs = d.cross(obs);
  double error = crs.norm();
  _error(0) = error;
  
}
