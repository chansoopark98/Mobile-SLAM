
#include "reality/engine/geometry/analytical-residual/pose-estimation.h"

namespace c8 {
namespace {
constexpr double SMALL_FLOAT = 1e-12;
double computeObservedPointWeight(const ObservedPoint &pt) {
  // Larger scales weighted less
  const auto scaleWeight = 1.0f / (1.0f + pt.scale / 2.0f);

  // larger descriptor distances weighted less
  const auto distWeight =
  pt.descriptorDist > 1000.0f ? 1.0f : 1.0f / (1.0f + pt.descriptorDist / 100.0f);

  return scaleWeight * distWeight * (pt.weight > SMALL_FLOAT ? pt.weight : 1.0f);
}
}  // namespace

namespace analytic {

PoseEstimation::PoseEstimation(
  double x, double y, double z, const ObservedPoint &pt, double scale)
  : x_(x), y_(y), z_(z), u_(pt.position.x()), v_(pt.position.y()), scale_(scale) {
  w_ = computeObservedPointWeight(pt);
}

bool PoseEstimation::Evaluate(
  double const *const *c, double *residuals, double **jacobians) const {
  static bool first = true;
  static double rx, ry, rz, tx, ty, tz, theta2;
  const int needsParamUpdate = first + (c[0][0] != rx) + (c[0][1] != ry) + (c[0][2] != rz)
  + (c[0][3] != tx) + (c[0][4] != ty) + (c[0][5] != tz);

  first = false;
  rx = c[0][0];
  ry = c[0][1];
  rz = c[0][2];
  tx = c[0][3];
  ty = c[0][4];
  tz = c[0][5];
  theta2 = rx * rx + ry * ry + rz * rz;

  if (theta2 < 1e-6) {
const double x0 = rx*y_ - ry*x_ + tz + z_;
const double x1 = ((x0 < 0) ? (
   100*w_
)
: (
   w_
));
const double x2 = ry*z_ - rz*y_ + tx + x_;
const double x3 = 1.0/x0;
const double x4 = -rx*z_ + rz*x_ + ty + y_;
const double x5 = scale_/std::pow(x0, 2);
const double x6 = x2*x5;
const double x7 = x1*x6;
const double x8 = scale_*x3;
const double x9 = x8*z_;
const double x10 = x1*x8;
const double x11 = x4*x5;
const double x12 = x1*x11;
residuals[0] = x1*(-scale_*u_ + scale_*x2*x3);
residuals[1] = x1*(-scale_*v_ + scale_*x3*x4);

  if (jacobians == NULL) {
  return true;
  }
  double *jacobian = jacobians[0];
  if (jacobian == NULL) {
  return true;
  }
  
jacobian[0] = -x7*y_;
jacobian[1] = x1*(x6*x_ + x9);
jacobian[2] = -x10*y_;
jacobian[3] = x10;
jacobian[4] = 0;
jacobian[5] = -x7;
jacobian[6] = x1*(-x11*y_ - x9);
jacobian[7] = x12*x_;
jacobian[8] = x10*x_;
jacobian[9] = 0;
jacobian[10] = x10;
jacobian[11] = -x12;
return true;
} else {
// Memoizable expressions
static double y0,y1,y2,y3,y4,y5,y6,y7,y17,y18,y28,y29,y34,y35,y37,y51,y52,y54,y57,y62,y64;
if (needsParamUpdate) {
y0 = std::pow(rx, 2);
y1 = std::pow(ry, 2);
y2 = std::pow(rz, 2);
y3 = y0 + y1 + y2;
y4 = std::sqrt(y3);
y5 = std::cos(y4);
y6 = std::sin(y4);
y7 = 1.0/y4;
y17 = 1.0 - y5;
y18 = y17*y7;
y28 = std::pow(y3, -3.0/2.0);
y29 = rx*y28;
y34 = y5*y7;
y35 = rx*y34;
y37 = y0*y28;
y51 = ry*rz;
y52 = y28*y51;
y54 = y1*y28;
y57 = ry*y34;
y62 = y2*y28;
y64 = rz*y34;
}
const double y8 = y7*y_;
const double y9 = rx*y8;
const double y10 = x_*y7;
const double y11 = ry*y10;
const double y12 = -y11 + y9;
const double y13 = rx*y10;
const double y14 = ry*y8;
const double y15 = y7*z_;
const double y16 = rz*y15 + y13 + y14;
const double y19 = y16*y18;
const double y20 = rz*y19 + tz + y12*y6 + y5*z_;
const double y21 = ((y20 < 0) ? (
   100*w_
)
: (
   w_
));
const double y22 = rz*y8;
const double y23 = ry*y15 - y22;
const double y24 = rx*y19 + tx + x_*y5 + y23*y6;
const double y25 = 1.0/y20;
const double y26 = -rx*y15 + rz*x_*y7;
const double y27 = ry*y19 + ty + y26*y6 + y5*y_;
const double y30 = y29*z_;
const double y31 = ry*y30;
const double y32 = y29*y_;
const double y33 = rz*y32;
const double y36 = y16*y6/y3;
const double y38 = y16*y17;
const double y39 = ry*y32;
const double y40 = rz*y30;
const double y41 = y18*(-x_*y37 + y10 - y39 - y40);
const double y42 = scale_*y25;
const double y43 = x_*y29;
const double y44 = ry*y43;
const double y45 = rx*y36;
const double y46 = y29*y38;
const double y47 = rz*y45 - rz*y46;
const double y48 = rx*y6*y7*z_ - rz*y41 - y12*y35 - y47 - y6*(-y37*y_ + y44 + y8);
const double y49 = scale_/std::pow(y20, 2);
const double y50 = y24*y49;
const double y53 = y52*y_;
const double y55 = y52*z_;
const double y56 = y18*(-y44 - y54*y_ - y55 + y8);
const double y58 = ry*y45 - ry*y46;
const double y59 = y36*y51 - y38*y52;
const double y60 = ry*y6*y7*z_ - rz*y56 - y12*y57 - y59 - y6*(x_*y1*y28 - y10 - y39);
const double y61 = rz*y43;
const double y63 = y18*(y15 - y53 - y61 - y62*z_);
const double y65 = -x_*y52;
const double y66 = rz*y6*y7*z_ - rz*y63 - y12*y64 + y16*y17*y2*y28 - y19 - y2*y36 - y6*(-y33 - y65);
const double y67 = y21*y42;
const double y68 = y27*y49;
residuals[0] = y21*(-scale_*u_ + scale_*y24*y25);
residuals[1] = y21*(-scale_*v_ + scale_*y25*y27);

  if (jacobians == NULL) {
  return true;
  }
  double *jacobian = jacobians[0];
  if (jacobian == NULL) {
  return true;
  }
  
jacobian[0] = y21*(y42*(rx*y41 + y0*y36 - y13*y6 + y19 + y23*y35 - y37*y38 + y6*(-y31 + y33)) + y48*y50);
jacobian[1] = y21*(y42*(rx*y56 - y11*y6 + y23*y57 + y58 + y6*(y15 + y53 - y54*z_)) + y50*y60);
jacobian[2] = y21*(y42*(rx*y63 - rz*y10*y6 + y23*y64 + y47 + y6*(y2*y28*y_ - y55 - y8)) + y50*y66);
jacobian[3] = y67;
jacobian[4] = 0;
jacobian[5] = -y21*y50;
jacobian[6] = y21*(y42*(ry*y41 + y26*y35 + y58 - y6*y9 + y6*(y0*y28*z_ - y15 - y61)) + y48*y68);
jacobian[7] = y21*(y42*(ry*y56 + y1*y36 - y14*y6 + y19 + y26*y57 - y38*y54 + y6*(y31 + y65)) + y60*y68);
jacobian[8] = y21*(y42*(ry*y63 - y22*y6 + y26*y64 + y59 + y6*(-x_*y62 + y10 + y40)) + y66*y68);
jacobian[9] = 0;
jacobian[10] = y67;
jacobian[11] = -y21*y68;
return true;
}
}

} // namespace analytic
} // namespace c8
