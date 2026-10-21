#pragma once

// Ceres 2.1 replaced ceres::LocalParameterization with ceres::Manifold (2.2 removed
// LocalParameterization). The backend's parameterisations derive from
// CeresParameterization, which is whichever of the two this Ceres has; localSize()/
// globalSize() hide the renamed size accessors. The Manifold path implements
// Minus(y, x) (Ceres' argument order) with the backend's own minus(x, y) and
// MinusJacobian with its liftJacobian.

#include <ceres/ceres.h>
#include <ceres/version.h>

#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
#define SVO_CERES_USE_MANIFOLD 1
#endif

namespace svo {
namespace ceres_backend {

#ifdef SVO_CERES_USE_MANIFOLD
using CeresParameterization = ceres::Manifold;
inline int localSize(const CeresParameterization* p) { return p->TangentSize(); }
inline int globalSize(const CeresParameterization* p) { return p->AmbientSize(); }
#else
using CeresParameterization = ceres::LocalParameterization;
inline int localSize(const CeresParameterization* p) { return p->LocalSize(); }
inline int globalSize(const CeresParameterization* p) { return p->GlobalSize(); }
#endif

}  // namespace ceres_backend
}  // namespace svo
