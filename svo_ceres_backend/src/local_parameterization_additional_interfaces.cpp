/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2016, ETH Zurich, Wyss Zurich, Zurich Eye
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jul 27, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Zurich Eye
 *********************************************************************************/

#include "svo/ceres_backend/local_parameterization_additional_interfaces.hpp"

#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since c++11
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <ceres/ceres.h>
#include <Eigen/Core>
#pragma diagnostic pop

namespace svo {
namespace ceres_backend {

// Verifies the correctness of a inplementation.
bool LocalParamizationAdditionalInterfaces::verify(
    const double* x_raw, double purturbation_magnitude) const
{
  const CeresParameterization* casted =
      dynamic_cast<const CeresParameterization*>(this);
  if (!casted)
  {
    return false;
  }
  // verify plus/minus
  Eigen::VectorXd x(globalSize(casted));
  memcpy(x.data(), x_raw, sizeof(double) * globalSize(casted));
  Eigen::VectorXd delta_x(localSize(casted));
  Eigen::VectorXd x_plus_delta(globalSize(casted));
  Eigen::VectorXd delta_x2(localSize(casted));
  delta_x.setRandom();
  delta_x *= purturbation_magnitude;
  casted->Plus(x.data(), delta_x.data(), x_plus_delta.data());
#ifdef SVO_CERES_USE_MANIFOLD
  casted->Minus(x_plus_delta.data(), x.data(), delta_x2.data());
#else
  this->Minus(x.data(), x_plus_delta.data(), delta_x2.data());
#endif
  if ((delta_x2 - delta_x).norm() > 1.0e-12)
  {
    return false;
  }

  // plusJacobian numDiff
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_plus_num_diff(
      globalSize(casted), localSize(casted));
  const double dx = 1.0e-9;
  for (int i = 0; i < localSize(casted); ++i)
  {
    Eigen::VectorXd delta_p(localSize(casted));
    delta_p.setZero();
    delta_p[i] = dx;
    Eigen::VectorXd delta_m(localSize(casted));
    delta_m.setZero();
    delta_m[i] = -dx;

    // reset
    Eigen::VectorXd x_p(globalSize(casted));
    Eigen::VectorXd x_m(globalSize(casted));
    memcpy(x_p.data(), x_raw, sizeof(double) * globalSize(casted));
    memcpy(x_m.data(), x_raw, sizeof(double) * globalSize(casted));
    casted->Plus(x.data(), delta_p.data(), x_p.data());
    casted->Plus(x.data(), delta_m.data(), x_m.data());
    J_plus_num_diff.col(i) = (x_p - x_m) / (2 * dx);
  }

  // verify lift
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_plus(globalSize(casted),
                                                        localSize(casted));
  Eigen::Matrix<double, -1, -1, Eigen::RowMajor> J_lift(localSize(casted),
                                                        globalSize(casted));
#ifdef SVO_CERES_USE_MANIFOLD
  casted->PlusJacobian(x_raw, J_plus.data());
#else
  casted->ComputeJacobian(x_raw, J_plus.data());
#endif
  ComputeLiftJacobian(x_raw, J_lift.data());
  Eigen::MatrixXd identity(localSize(casted), localSize(casted));
  identity.setIdentity();
  if (((J_lift * J_plus) - identity).norm() > 1.0e-6)
  {
    return false;
  }

  // verify numDiff jacobian
  if ((J_plus - J_plus_num_diff).norm() > 1.0e-6)
  {
    return false;
  }

  // everything fine...
  return true;
}

} // namespace ceres_backend
} // namespace svo

