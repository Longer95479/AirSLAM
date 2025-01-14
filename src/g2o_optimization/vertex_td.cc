#include "g2o_optimization/vertex_td.h"

VertexTd::VertexTd() {}

VertexTd::VertexTd(double td) {
  setEstimate(td);
}

bool VertexTd::read(std::istream& is){
  return false;
}

bool VertexTd::write(std::ostream& os) const {
  return false;
}

void VertexTd::setToOriginImpl() {
    _estimate = 0.0;  
}

void VertexTd::oplusImpl(const double* update) {
    _estimate += update[0];  
}

void VertexTd::getValue(double& value) const {
    value = _estimate;
}

