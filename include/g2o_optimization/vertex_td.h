#ifndef VERTEX_TD_H_
#define VERTEX_TD_H_

#include <g2o/core/base_vertex.h>
#include <Eigen/Core>
#include <iostream>

class VertexTd: public g2o::BaseVertex<1, double> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexTd();
    VertexTd(double td);
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    virtual void setToOriginImpl();
    virtual void oplusImpl(const double* update); 
    void getValue(double& value) const;
};

#endif
