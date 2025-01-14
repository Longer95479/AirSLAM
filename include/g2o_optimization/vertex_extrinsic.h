#ifndef VERTEX_VI_ENTRINSIC_H_
#define VERTEX_VI_ENTRINSIC_H_

#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/sba/g2o_types_sba_api.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d_addons/line3d.h>

class Extrinsic{
public:
  Extrinsic();
  Extrinsic(const Eigen::Matrix3d& Rcb_, const Eigen::Vector3d& tcb_);
  Extrinsic& operator =(const Extrinsic& other);
  void SetParam(const Eigen::Matrix3d& Rcb_, const Eigen::Vector3d& tcb_);
  void Update(const double* v);

public:
  Eigen::Matrix3d Rcb;
  Eigen::Vector3d tcb;
  int n_it;
};

class VertexExtrinsic : public g2o::BaseVertex<6, Extrinsic> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexExtrinsic();
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual void setToOriginImpl() {}

  virtual void oplusImpl(const double* update_) {
    _estimate.Update(update_);
    updateCache();
  }

  virtual int estimateDimension() const { return 6; }
};

#endif  // VERTEX_VI_ENTRINSIC_H_ 
