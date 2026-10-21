// Numerical self-check of the backend's Ceres parameterisations (built by the
// repository-root CMakeLists.txt): on random states, for PoseLocalParameterization
// and HomogeneousPointLocalParameterization,
//   - Minus(Plus(x, d), x) == d                    (Minus inverts Plus),
//   - PlusJacobian == central finite differences   (of Plus at d = 0),
//   - MinusJacobian == central finite differences  (of Minus(., x) at x),
//   - MinusJacobian * PlusJacobian == I            (the lift inverts the plus Jacobian),
// Exits non-zero on any failure. The backend's own
// LocalParamizationAdditionalInterfaces::verify() is reported, not gated: it takes
// 1e-9 central differences against a 1e-6 tolerance on the Jacobian's norm, which
// rounding error alone exceeds for coordinates of order 10 (the reason upstream's
// call to it in map.cpp is commented out); it is run at unit scale here.
#include <cstdio>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "svo/ceres_backend/homogeneous_point_local_parameterization.hpp"
#include "svo/ceres_backend/pose_local_parameterization.hpp"

using svo::ceres_backend::CeresParameterization;
using svo::ceres_backend::globalSize;
using svo::ceres_backend::localSize;
using RowMajor = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;

namespace {

std::mt19937 rng(7);
std::normal_distribution<double> normal;

double PlusMinus(const CeresParameterization& p, const Eigen::VectorXd& x, const Eigen::VectorXd& d) {
  Eigen::VectorXd y(globalSize(&p)), d2(localSize(&p));
  p.Plus(x.data(), d.data(), y.data());
#ifdef SVO_CERES_USE_MANIFOLD
  p.Minus(y.data(), x.data(), d2.data());
#else
  dynamic_cast<const svo::ceres_backend::LocalParamizationAdditionalInterfaces&>(p).Minus(x.data(), y.data(), d2.data());
#endif
  return (d2 - d).norm();
}

// max |J - J_numeric| for PlusJacobian and MinusJacobian, and |lift * plus - I|.
void Jacobians(const CeresParameterization& p, const Eigen::VectorXd& x, double* plus_err, double* minus_err,
               double* lift_err) {
  const int n = globalSize(&p), m = localSize(&p);
  const double h = 1e-6;
  RowMajor j_plus(n, m), j_lift(m, n), num_plus(n, m);
#ifdef SVO_CERES_USE_MANIFOLD
  p.PlusJacobian(x.data(), j_plus.data());
  p.MinusJacobian(x.data(), j_lift.data());
#else
  p.ComputeJacobian(x.data(), j_plus.data());
  dynamic_cast<const svo::ceres_backend::LocalParamizationAdditionalInterfaces&>(p).ComputeLiftJacobian(
      x.data(), j_lift.data());
#endif
  for (int i = 0; i < m; ++i) {
    Eigen::VectorXd dp = Eigen::VectorXd::Zero(m), dm = Eigen::VectorXd::Zero(m), xp(n), xm(n);
    dp[i] = h;
    dm[i] = -h;
    p.Plus(x.data(), dp.data(), xp.data());
    p.Plus(x.data(), dm.data(), xm.data());
    num_plus.col(i) = (xp - xm) / (2 * h);
  }
#ifdef SVO_CERES_USE_MANIFOLD
  // Minus is only defined near the manifold (the homogeneous point's w must not
  // change): differentiate it along the ambient images of the tangent directions,
  // v = PlusJacobian * e_i, i.e. compare MinusJacobian * v with the numeric one.
  RowMajor num_minus_v(m, m);
  for (int i = 0; i < m; ++i) {
    const Eigen::VectorXd v = j_plus.col(i);
    Eigen::VectorXd yp = x + h * v, ym = x - h * v, dp(m), dm(m);
    p.Minus(yp.data(), x.data(), dp.data());
    p.Minus(ym.data(), x.data(), dm.data());
    num_minus_v.col(i) = (dp - dm) / (2 * h);
  }
  *minus_err = (j_lift * j_plus - num_minus_v).cwiseAbs().maxCoeff();
#else
  *minus_err = 0.0;
#endif
  *plus_err = (j_plus - num_plus).cwiseAbs().maxCoeff();
  *lift_err = (j_lift * j_plus - RowMajor::Identity(m, m)).cwiseAbs().maxCoeff();
}

Eigen::VectorXd RandomPose() {
  Eigen::VectorXd x(7);
  Eigen::Quaterniond q(normal(rng), normal(rng), normal(rng), normal(rng));
  q.normalize();
  x << 10 * normal(rng), 10 * normal(rng), 10 * normal(rng), q.x(), q.y(), q.z(), q.w();
  return x;
}

Eigen::VectorXd RandomPoint() {
  Eigen::VectorXd x(4);
  x << 10 * normal(rng), 10 * normal(rng), 10 * normal(rng), 1.0;
  return x;
}

}  // namespace

int main() {
  svo::ceres_backend::PoseLocalParameterization pose;
  svo::ceres_backend::HomogeneousPointLocalParameterization point;
  struct Case {
    const char* name;
    const CeresParameterization* p;
    Eigen::VectorXd (*sample)();
  } cases[] = {{"pose", &pose, RandomPose}, {"homogeneous point", &point, RandomPoint}};

  bool ok = true;
  for (const Case& c : cases) {
    double roundtrip = 0, plus_err = 0, minus_err = 0, lift_err = 0;
    int verified = 0;
    for (int k = 0; k < 1000; ++k) {
      const Eigen::VectorXd x = c.sample();
      Eigen::VectorXd d(localSize(c.p));
      for (int i = 0; i < d.size(); ++i) d[i] = 0.3 * normal(rng);
      roundtrip = std::max(roundtrip, PlusMinus(*c.p, x, d));
      double pe, me, le;
      Jacobians(*c.p, x, &pe, &me, &le);
      plus_err = std::max(plus_err, pe);
      minus_err = std::max(minus_err, me);
      lift_err = std::max(lift_err, le);
      const Eigen::VectorXd x_unit = c.sample() / 10.0;  // translation/point ~ 1
      Eigen::VectorXd x_verify = x_unit;
      if (globalSize(c.p) == 7) x_verify.tail<4>() = x.tail<4>();  // keep a unit quaternion
      else x_verify[3] = 1.0;
      verified += dynamic_cast<const svo::ceres_backend::LocalParamizationAdditionalInterfaces*>(c.p)->verify(
          x_verify.data());
    }
    const bool pass = roundtrip < 1e-9 && plus_err < 1e-6 && minus_err < 1e-6 && lift_err < 1e-9;
    std::printf("%-18s Minus(Plus(x,d),x)-d %.1e  PlusJacobian %.1e  MinusJacobian %.1e  lift*plus-I %.1e  "
                "verify() %d/1000  -> %s\n",
                c.name, roundtrip, plus_err, minus_err, lift_err, verified, pass ? "PASS" : "FAIL");
    ok &= pass;
  }
  return ok ? 0 : 1;
}
