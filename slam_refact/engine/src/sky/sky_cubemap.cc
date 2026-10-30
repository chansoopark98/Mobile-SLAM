// CPU port of SemanticsCubemapRenderer. See sky_cubemap.h and docs/spec/09-sky.md §3.
//
// Source map (PUB = public reference runtime commit 4edfa484):
//   PUB:reality/engine/semantics/semantics-cubemap-renderer.cc  constructor :114-340,
//     setProjectionIntrinsics :342-358, updateSrcTexture :360-373, projectImageToFace :375-447,
//     renderToCubemap :492-537, initOutputSemanticsScene :539-583, renderCubemap :637-686,
//     gaussianBlurLinear :688-751, setSkyPostProcessParameters :753-760, renderSkyTexture :762-816,
//     drawOutputSemantics (texId overload) :818-868
//   PUB:reality/engine/semantics/shaders/semantics-project.frag, semantics-gaussian-linear.frag,
//     semantics-sky.frag; PUB:c8/pixels/render/shaders/{image,cubemap}.frag
//   PUB:c8/geometry/raycaster.cc rayPlaneIntersection / intersectFrustumWithPlane,
//     PUB:c8/geometry/line3.cc Line3::intersects, PUB:c8/geometry/mesh.cc isPointInTriangle
#include "sky/sky_cubemap.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "c8/geometry/intrinsics-clip.h"
#include "c8/pixels/render/object8.h"
#include "sky/sky_preprocess.h"

namespace slam_refact::sky {
namespace {

constexpr int kIntmdDownsizeScale = 5;  // INTMD_DOWNSIZE_SCALE
constexpr float kBlurRadius = 11;       // BLUR_RADIUS (kernel size)
constexpr float kBlurSigma = 10.0f;     // BLUR_SIGMA
constexpr float kEpsilon = 0.0000001f;  // raycaster.cc, line3.cc EPSILON
constexpr float kMeshEpsilon = 1.0e-6f;  // mesh.cc EPSILON
constexpr double kPi = 3.14159265358979323846;

using Vec3 = std::array<float, 3>;

Vec3 Sub(const Vec3& a, const Vec3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
float Dot(const Vec3& a, const Vec3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

// raycaster.cc rayPlaneIntersection with rayOrigin = (0, 0, 0).
bool RayPlaneIntersection(const Vec3& ray_vector, const Vec3& plane_normal, float plane_d,
                          Vec3* out) {
  const float denom = Dot(ray_vector, plane_normal);
  if (std::fabs(denom) < kEpsilon) {
    return false;
  }
  const float t = -(0.0f * plane_normal[0] + 0.0f * plane_normal[1] + 0.0f * plane_normal[2] +
                    plane_d) /
                  denom;
  if (t > kEpsilon) {
    *out = {0.0f + ray_vector[0] * t, 0.0f + ray_vector[1] * t, 0.0f + ray_vector[2] * t};
    return true;
  }
  return false;
}

// HVector3::unit() of c8/hvector.h (x / norm).
Vec3 Unit(const Vec3& v) {
  const float n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  return {v[0] / n, v[1] / n, v[2] / n};
}

// mesh.cc toBarycentricWeights / isPointInTriangle.
std::array<float, 3> ToBarycentricWeights(const Vec3& pt, const std::array<Vec3, 3>& tri) {
  const Vec3 normal = Cross(Sub(tri[1], tri[0]), Sub(tri[2], tri[0]));
  float u1, u2, u3, u4, v1, v2, v3, v4;
  if (std::fabs(normal[0]) >= std::fabs(normal[1]) && std::fabs(normal[0]) >= std::fabs(normal[2])) {
    u1 = tri[0][1] - tri[2][1];
    u2 = tri[1][1] - tri[2][1];
    u3 = pt[1] - tri[0][1];
    u4 = pt[1] - tri[2][1];
    v1 = tri[0][2] - tri[2][2];
    v2 = tri[1][2] - tri[2][2];
    v3 = pt[2] - tri[0][2];
    v4 = pt[2] - tri[2][2];
  } else if (std::fabs(normal[1]) >= std::fabs(normal[2])) {
    u1 = tri[0][2] - tri[2][2];
    u2 = tri[1][2] - tri[2][2];
    u3 = pt[2] - tri[0][2];
    u4 = pt[2] - tri[2][2];
    v1 = tri[0][0] - tri[2][0];
    v2 = tri[1][0] - tri[2][0];
    v3 = pt[0] - tri[0][0];
    v4 = pt[0] - tri[2][0];
  } else {
    u1 = tri[0][0] - tri[2][0];
    u2 = tri[1][0] - tri[2][0];
    u3 = pt[0] - tri[0][0];
    u4 = pt[0] - tri[2][0];
    v1 = tri[0][1] - tri[2][1];
    v2 = tri[1][1] - tri[2][1];
    v3 = pt[1] - tri[0][1];
    v4 = pt[1] - tri[2][1];
  }
  const float denominator = v1 * u2 - v2 * u1;
  const float inv_denominator = 1.0f / denominator;
  const float x = (v4 * u2 - v2 * u4) * inv_denominator;
  const float y = (v1 * u3 - v3 * u1) * inv_denominator;
  return {x, y, 1.0f - x - y};
}

bool IsPointInTriangle(const Vec3& pt, const std::array<Vec3, 3>& tri) {
  const Vec3 n = Cross(Sub(tri[1], tri[0]), Sub(tri[2], tri[0]));
  const float d = -(tri[0][0] * n[0] + tri[0][1] * n[1] + tri[0][2] * n[2]);
  const float plane = pt[0] * n[0] + pt[1] * n[1] + pt[2] * n[2] + d;
  if (std::fabs(plane) > kMeshEpsilon) {
    return false;
  }
  const auto w = ToBarycentricWeights(pt, tri);
  return w[0] >= 0.0f && w[0] <= 1.0f && w[1] >= 0.0f && w[1] <= 1.0f && (w[0] + w[1]) <= 1.0f;
}

// line3.cc Line3::intersects(other, &muThis, &muOther, ...): `this` = (p1, p2), `other` = (p3, p4).
bool LineIntersects(const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& p4, float* mu_this,
                    float* mu_other) {
  const Vec3 p13 = {p1[0] - p3[0], p1[1] - p3[1], p1[2] - p3[2]};
  const Vec3 p43 = {p4[0] - p3[0], p4[1] - p3[1], p4[2] - p3[2]};
  if (std::fabs(p43[0]) < kEpsilon && std::fabs(p43[1]) < kEpsilon && std::fabs(p43[2]) < kEpsilon) {
    return false;
  }
  const Vec3 p21 = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
  if (std::fabs(p21[0]) < kEpsilon && std::fabs(p21[1]) < kEpsilon && std::fabs(p21[2]) < kEpsilon) {
    return false;
  }
  const float d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2];
  const float d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2];
  const float d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2];
  const float d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2];
  const float d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];
  const float denom = d2121 * d4343 - d4321 * d4321;
  if (std::fabs(denom) < kEpsilon) {
    return false;
  }
  const float numer = d1343 * d4321 - d1321 * d4343;
  *mu_this = numer / denom;
  *mu_other = (d1343 + d4321 * (*mu_this)) / d4343;
  return true;
}

// raycaster.cc intersectFrustumWithPlane. `plane` = {A, B, C, D}; `triangles` = 6 points (two
// triangles); `boundary` = 4 lines (each a start/end point pair).
bool IntersectFrustumWithPlane(const std::array<float, 4>& plane,
                               const std::array<Vec3, 4>& frustum_ray_dir,
                               const std::array<Vec3, 6>& triangles,
                               const std::array<std::array<Vec3, 2>, 4>& boundary) {
  std::vector<Vec3> inter_pts;
  int intersection_count = 0;
  Vec3 inter_pt{};
  const Vec3 plane_normal = {plane[0], plane[1], plane[2]};
  for (size_t i = 0; i < 4; ++i) {
    if (RayPlaneIntersection(Unit(frustum_ray_dir[i]), plane_normal, plane[3], &inter_pt)) {
      ++intersection_count;
      inter_pts.push_back(inter_pt);
    }
  }
  if (intersection_count == 4) {
    return true;
  }
  if (intersection_count == 0) {
    return false;
  }
  for (size_t tri = 0; tri < triangles.size(); tri += 3) {
    const std::array<Vec3, 3> triangle = {triangles[tri], triangles[tri + 1], triangles[tri + 2]};
    for (const auto& p : inter_pts) {
      if (IsPointInTriangle(p, triangle)) {
        return true;
      }
    }
  }
  if (intersection_count == 1) {
    return false;
  }
  for (size_t i = 0; i + 1 < inter_pts.size(); ++i) {
    for (size_t j = i + 1; j < inter_pts.size(); ++j) {
      for (size_t li = 0; li < boundary.size(); ++li) {
        float mua = 0, mub = 0;
        // boundaryLines[li].intersects(lineIj, &mua, &mub, ...): this = boundary line, other = line i-j.
        const bool intersect =
            LineIntersects(boundary[li][0], boundary[li][1], inter_pts[i], inter_pts[j], &mua, &mub);
        if (intersect && mua >= 0.0f && mua <= 1.0f && mub >= 0.0f && mub <= 1.0f) {
          return true;
        }
      }
    }
  }
  return false;
}

// modelViewMatrix(face) of the constructor file.
c8::HMatrix ModelViewMatrix(int face) {
  c8::HMatrix flip_y = c8::HMatrixGen::rotationR(0.0f, 0.0f, static_cast<float>(kPi));
  c8::HMatrix plane_mat = c8::HMatrixGen::i();
  switch (face) {
    case 0:  // POSITIVE_X
      plane_mat = c8::HMatrixGen::rotationR(0.0f, 0.5f * static_cast<float>(kPi), 0.0f);
      break;
    case 1:  // NEGATIVE_X
      plane_mat = c8::HMatrixGen::rotationR(0.0f, -0.5f * static_cast<float>(kPi), 0.0f);
      break;
    case 2:  // POSITIVE_Y
      flip_y = c8::HMatrixGen::i();
      plane_mat = c8::HMatrixGen::rotationR(-0.5f * static_cast<float>(kPi), 0.0f, 0.0f);
      break;
    case 3:  // NEGATIVE_Y
      flip_y = c8::HMatrixGen::i();
      plane_mat = c8::HMatrixGen::rotationR(0.5f * static_cast<float>(kPi), 0.0f, 0.0f);
      break;
    case 4:  // POSITIVE_Z
      plane_mat = c8::HMatrixGen::rotationR(0.0f, static_cast<float>(kPi), 0.0f);
      break;
    default:  // NEGATIVE_Z
      break;
  }
  return flip_y * plane_mat;
}

// Intrinsics::perspectiveProjectionRightHanded(M_PI * 0.5f, 1.0f, near, far), PUB:c8/geometry/
// intrinsics.cc:223-243: returns the matrix entries that the per-pixel inverse needs (f, fova).
struct Persp90 {
  float f;
  float fova;
};
Persp90 Perspective90() {
  const float f = 1.0f / std::tan(0.5f * static_cast<float>(kPi * 0.5));
  return {f, f / 1.0f};
}

// Homogeneous product M * (x, y, z, 1) as doubles.
std::array<double, 4> MulPoint(const c8::HMatrix& m, double x, double y, double z) {
  std::array<double, 4> r{};
  for (int i = 0; i < 4; ++i) {
    r[i] = static_cast<double>(m(i, 0)) * x + static_cast<double>(m(i, 1)) * y +
           static_cast<double>(m(i, 2)) * z + static_cast<double>(m(i, 3));
  }
  return r;
}

// Face selection and (s, t) of a direction (GL spec cube map face table), face order PX NX PY NY PZ NZ.
struct CubeCoord {
  int face;
  double s, t;  // in [0, 1]
};
CubeCoord ResolveDirection(double rx, double ry, double rz) {
  const double ax = std::fabs(rx), ay = std::fabs(ry), az = std::fabs(rz);
  int face;
  double sc, tc, ma;
  if (ax >= ay && ax >= az) {
    ma = ax;
    if (rx >= 0) { face = 0; sc = -rz; tc = -ry; } else { face = 1; sc = rz; tc = -ry; }
  } else if (ay >= az) {
    ma = ay;
    if (ry >= 0) { face = 2; sc = rx; tc = rz; } else { face = 3; sc = rx; tc = -rz; }
  } else {
    ma = az;
    if (rz >= 0) { face = 4; sc = rx; tc = -ry; } else { face = 5; sc = -rx; tc = -ry; }
  }
  return {face, 0.5 * (sc / ma + 1.0), 0.5 * (tc / ma + 1.0)};
}

// Inverse of the table above: a direction for face-local (s, t) (possibly outside [0, 1]).
void FaceDirection(int face, double s, double t, double* rx, double* ry, double* rz) {
  const double sc = 2 * s - 1, tc = 2 * t - 1;
  switch (face) {
    case 0: *rx = 1; *rz = -sc; *ry = -tc; break;
    case 1: *rx = -1; *rz = sc; *ry = -tc; break;
    case 2: *ry = 1; *rx = sc; *rz = tc; break;
    case 3: *ry = -1; *rx = sc; *rz = -tc; break;
    case 4: *rz = 1; *rx = sc; *ry = -tc; break;
    default: *rz = -1; *rx = -sc; *ry = -tc; break;
  }
}

}  // namespace

SkyCubemapRenderer::SkyCubemapRenderer(int tex_size)
    : tex_size_(tex_size),
      project_camera_projection_(c8::HMatrixGen::i()),
      output_camera_projection_(c8::HMatrixGen::i()) {
  // Constructor :129-137: cubePlanes_.
  cube_planes_ = {{{1.0f, 0.0f, 0.0f, -1.0f}, {1.0f, 0.0f, 0.00f, 1.0f}, {0.0f, 1.0f, 0.0f, -1.0f},
                   {0.0f, 1.0f, 0.00f, 1.0f}, {0.0f, 0.0f, 1.0f, -1.0f}, {0.0f, 0.0f, 1.00f, 1.0f}}};
  const std::array<Vec3, 8> v = {{{-1.0f, 1.0f, 1.0f},
                                  {-1.0f, -1.0f, 1.0f},
                                  {1.0f, 1.0f, 1.0f},
                                  {1.0f, -1.0f, 1.0f},
                                  {-1.0f, 1.0f, -1.0f},
                                  {-1.0f, -1.0f, -1.0f},
                                  {1.0f, 1.0f, -1.0f},
                                  {1.0f, -1.0f, -1.0f}}};
  auto set_face = [&](int i, int a, int b, int c, int d, std::array<std::array<float, 2>, 4> uvs) {
    faces_[i].points = {v[a], v[b], v[c], v[d]};
    faces_[i].uvs = uvs;
  };
  set_face(0, 3, 7, 2, 6, {{{0.f, 1.f}, {1.f, 1.f}, {0.f, 0.f}, {1.f, 0.f}}});  // +X
  set_face(1, 1, 5, 0, 4, {{{1.f, 1.f}, {0.f, 1.f}, {1.f, 0.f}, {0.f, 0.f}}});  // -X
  set_face(2, 0, 4, 2, 6, {{{0.f, 1.f}, {0.f, 0.f}, {1.f, 1.f}, {1.f, 0.f}}});  // +Y
  set_face(3, 1, 5, 3, 7, {{{0.f, 1.f}, {0.f, 0.f}, {1.f, 1.f}, {1.f, 0.f}}});  // -Y
  set_face(4, 0, 1, 2, 3, {{{0.f, 0.f}, {0.f, 1.f}, {1.f, 0.f}, {1.f, 1.f}}});  // +Z
  set_face(5, 4, 5, 6, 7, {{{1.f, 0.f}, {1.f, 1.f}, {0.f, 0.f}, {0.f, 1.f}}});  // -Z

  // GL_RGBA texImage2D(null): zero. The cubemap texture and the two working sets.
  for (int i = 0; i < 6; ++i) {
    cube_tex_[i] = Rgba8Image(tex_size_, tex_size_);
    for (int k = 0; k < 2; ++k) {
      cube_faces_[k][i] = Rgba8Image(tex_size_, tex_size_);
    }
  }
  Rgba8Image black(tex_size_, tex_size_);  // solid black pixels for negative-Y ground face
  for (size_t p = 3; p < black.px.size(); p += 4) {
    black.px[p] = 255;
  }
  cube_tex_[3] = black;
  Rgba8Image yellow(tex_size_, tex_size_);  // solid yellow pixels for positive-Y top face
  for (size_t p = 0; p < yellow.px.size(); p += 4) {
    yellow.px[p] = 255;
    yellow.px[p + 1] = 255;
    yellow.px[p + 2] = 0;
    yellow.px[p + 3] = 255;
  }
  cube_tex_[2] = yellow;
  for (int k = 0; k < 2; ++k) {
    cube_faces_[k][2] = yellow;
  }
  // Pixels above the horizon line: transparent... (255, 255, 0, 255) for the first half of the
  // rows on the four side faces of both working sets.
  Rgba8Image horizon(tex_size_, tex_size_);
  const int half_rows = tex_size_ / 2;
  for (int r = 0; r < half_rows; ++r) {
    for (int c = 0; c < tex_size_; ++c) {
      uint8_t* p = horizon.at(c, r);
      p[0] = 255;
      p[1] = 255;
      p[2] = 0;
      p[3] = 255;
    }
  }
  for (int k = 0; k < 2; ++k) {
    for (int face : {0, 1, 4, 5}) {
      cube_faces_[k][face] = horizon;
    }
  }
  // Gaussian kernel: SemanticsGaussianLinearShader::setGaussianLinearKernels(BLUR_RADIUS, BLUR_SIGMA).
  kernel_size_ = static_cast<int>(kBlurRadius);
  const int mean = kernel_size_ / 2;
  float sum = 0;
  for (int x = 0; x < kernel_size_; ++x) {
    kernel_[x] = static_cast<float>(std::exp(-0.5 * std::pow((x - mean) / kBlurSigma, 2.0)));
    sum += kernel_[x];
  }
  for (int x = 0; x < kernel_size_; ++x) {
    kernel_[x] /= sum;
  }
}

void SkyCubemapRenderer::SetProjectionIntrinsics(const c8_PixelPinholeCameraModel& k) {
  // setProjectionIntrinsics :342-358: corners of the frustum at distance 1.
  const float w = static_cast<float>(k.pixelsWidth - 1);
  const float h = static_cast<float>(k.pixelsHeight - 1);
  const c8::HMatrix inv_k = c8::HMatrixGen::intrinsic(k).inv();
  const std::array<c8::HPoint3, 4> pix = {c8::HPoint3{0.0f, 0.0f, 1.0f}, c8::HPoint3{0.0f, h, 1.0f},
                                          c8::HPoint3{w, 0.0f, 1.0f}, c8::HPoint3{w, h, 1.0f}};
  for (size_t i = 0; i < 4; ++i) {
    const c8::HPoint3 p = inv_k * pix[i];
    frustum_local_corners_[i] = {p.x(), p.y(), p.z()};
  }
  // ObGen::perspectiveCamera(k, k.pixelsWidth, k.pixelsHeight) (object8.cc:793-801).
  const auto k_viewport =
      c8::Intrinsics::rotateCropAndScaleIntrinsics(k, k.pixelsWidth, k.pixelsHeight);
  project_camera_projection_ = c8::Intrinsics::toClipSpaceMatLeftHanded(
      k_viewport, c8::CAM_PROJECTION_NEAR_CLIP, c8::CAM_PROJECTION_FAR_CLIP);
  has_projection_camera_ = true;
}

void SkyCubemapRenderer::UpdateSrcTexture(const Rgba8Image& src_pixels) { src_tex_ = src_pixels; }

void SkyCubemapRenderer::RenderToCubemap(const c8::HMatrix& camera_extrinsics) {
  last_rendered_faces_ = 0;
  if (!has_projection_camera_) {
    return;
  }
  // cameraWorldPos_ = extrinsics * (0, 0, 0); frustumWorldCorners_ = extrinsics * localCorners.
  const c8::HPoint3 pos_p = camera_extrinsics * c8::HPoint3{0.0f, 0.0f, 0.0f};
  const Vec3 camera_world_pos = {pos_p.x(), pos_p.y(), pos_p.z()};
  std::array<Vec3, 4> ray_dir{};
  for (size_t i = 0; i < 4; ++i) {
    const c8::HPoint3 c = camera_extrinsics * c8::HPoint3{frustum_local_corners_[i][0],
                                                          frustum_local_corners_[i][1],
                                                          frustum_local_corners_[i][2]};
    ray_dir[i] = {c.x() - camera_world_pos[0], c.y() - camera_world_pos[1],
                  c.z() - camera_world_pos[2]};
  }
  // camMvp = projection * (translation(-camPos) * world).inv()  (projectImageToFace :383-388).
  const c8::HMatrix cam_world =
      c8::HMatrixGen::translation(-camera_world_pos[0], -camera_world_pos[1],
                                  -camera_world_pos[2]) *
      camera_extrinsics;
  const c8::HMatrix cam_mvp = project_camera_projection_ * cam_world.inv();

  const int target_set = (cube_tex_sampler_index_ + 1) % 2;
  const Persp90 p90 = Perspective90();
  const int n = tex_size_;
  for (int f = 0; f < 6; ++f) {
    if (f == 3) {
      continue;  // skip rendering to negative-Y
    }
    const auto& g = faces_[f];
    const std::array<Vec3, 6> tri_pts = {g.points[0], g.points[2], g.points[1],
                                         g.points[1], g.points[2], g.points[3]};
    const std::array<std::array<Vec3, 2>, 4> boundary = {{{g.points[0], g.points[1]},
                                                          {g.points[1], g.points[3]},
                                                          {g.points[3], g.points[2]},
                                                          {g.points[2], g.points[0]}}};
    if (!IntersectFrustumWithPlane(cube_planes_[f], ray_dir, tri_pts, boundary)) {
      continue;
    }
    last_rendered_faces_ |= 1 << f;
    const Rgba8Image& prev = cube_faces_[cube_tex_sampler_index_][f];
    Rgba8Image& out = cube_faces_[target_set][f];
    out = Rgba8Image(n, n);
    // Quad -> pixel mapping: viewport n x n, mvp = persp90 * modelView(f). The face quad lies in the
    // view plane z = -1, so the NDC of a pixel centre is the view-space (x, y) times (1 / fova,
    // 1 / f); the 3D point is modelView^-1 applied to it.
    const c8::HMatrix mv_inv = ModelViewMatrix(f).inv();
    const Vec3& p0 = g.points[0];
    const Vec3 axis_a = Sub(g.points[2], p0);  // quad edge 0 -> 2
    const Vec3 axis_b = Sub(g.points[1], p0);  // quad edge 0 -> 1
    const double aa = Dot(axis_a, axis_a), bb = Dot(axis_b, axis_b);
    for (int py = 0; py < n; ++py) {
      for (int px = 0; px < n; ++px) {
        const double ndc_x = (px + 0.5) / n * 2.0 - 1.0;
        const double ndc_y = (py + 0.5) / n * 2.0 - 1.0;
        const auto wp = MulPoint(mv_inv, ndc_x / p90.fova, ndc_y / p90.f, -1.0);
        const Vec3 p = {static_cast<float>(wp[0]), static_cast<float>(wp[1]),
                        static_cast<float>(wp[2])};
        // uv of the quad at p (affine in the plane).
        const Vec3 d = Sub(p, p0);
        const double s = Dot(d, axis_a) / aa;
        const double t = Dot(d, axis_b) / bb;
        const double u = g.uvs[0][0] + s * (g.uvs[2][0] - g.uvs[0][0]) + t * (g.uvs[1][0] - g.uvs[0][0]);
        const double vv = g.uvs[0][1] + s * (g.uvs[2][1] - g.uvs[0][1]) + t * (g.uvs[1][1] - g.uvs[0][1]);
        float prev_col[4];
        SampleBilinearClamp(prev, u, vv, prev_col);
        // sourceClipPos = camMvp * (p, 1); the shader divides without testing w.
        const auto clip = MulPoint(cam_mvp, p[0], p[1], p[2]);
        const float cx = static_cast<float>(clip[0] / clip[3]);
        const float cy = static_cast<float>(clip[1] / clip[3]);
        float frag[4];
        if (cx < -1.0f || cx > 1.0f || cy < -1.0f || cy > 1.0f) {
          for (int k = 0; k < 4; ++k) frag[k] = prev_col[k];
        } else {
          const float su = cx * 0.5f + 0.5f;
          const float sv = cy * -0.5f + 0.5f;
          float sem[4];
          SampleBilinearClamp(src_tex_, su, sv, sem);
          if (prev_col[2] == 0.0f) {
            // first time the pixel is seen
            frag[0] = sem[0];
            frag[1] = sem[1];
            frag[2] = std::max(0.1f, sem[2]);
            frag[3] = sem[3];
          } else {
            const float val = std::max(sem[0], 0.9f * prev_col[0]);
            frag[0] = val;
            frag[1] = val;
            frag[2] = 1.0f;
            frag[3] = 1.0f;
          }
        }
        uint8_t* o = out.at(px, py);
        for (int k = 0; k < 4; ++k) {
          o[k] = ToUnorm8(frag[k]);
        }
      }
    }
    // copyTexture(cubeFaces_[target][f] -> cubeTex_ face f): identity image shader pass.
    cube_tex_[f] = out;
  }
  cube_tex_sampler_index_ = target_set;
}

void SkyCubemapRenderer::InitOutputSemanticsScene(const c8_PixelPinholeCameraModel& k,
                                                  int output_width, int output_height) {
  if (output_width <= 0 || output_height <= 0) {
    return;
  }
  const auto k_viewport =
      c8::Intrinsics::rotateCropAndScaleIntrinsics(k, k.pixelsWidth, k.pixelsHeight);
  output_camera_projection_ = c8::Intrinsics::toClipSpaceMatLeftHanded(
      k_viewport, c8::CAM_PROJECTION_NEAR_CLIP, c8::CAM_PROJECTION_FAR_CLIP);
  has_output_camera_ = true;
  portrait_width_ = (output_width <= output_height) ? output_width : output_height;
  portrait_height_ = (output_width >= output_height) ? output_width : output_height;
  output_width_ = output_width;
  output_height_ = output_height;
}

void SkyCubemapRenderer::SetSkyPostProcessParameters(float edge_smoothness, bool have_seen_sky,
                                                     bool flip_alpha_mask) {
  have_seen_sky_ = have_seen_sky ? 1 : 0;
  flip_alpha_mask_ = flip_alpha_mask ? 1 : 0;
  edge_smoothness_ = edge_smoothness;
}

std::array<float, 4> SkyCubemapRenderer::SampleCubemap(float dx, float dy, float dz) const {
  const int n = tex_size_;
  const CubeCoord c = ResolveDirection(dx, dy, dz);
  const double x = c.s * n - 0.5;
  const double y = c.t * n - 0.5;
  const double x0 = std::floor(x), y0 = std::floor(y);
  const float fx = QuantizeFilterWeight(static_cast<float>(x - x0));
  const float fy = QuantizeFilterWeight(static_cast<float>(y - y0));
  auto texel = [&](int xi, int yi) -> const uint8_t* {
    if (xi >= 0 && xi < n && yi >= 0 && yi < n) {
      return cube_tex_[c.face].at(xi, yi);
    }
    if (!seamless_) {  // clamp to the face
      return cube_tex_[c.face].at(std::min(std::max(xi, 0), n - 1), std::min(std::max(yi, 0), n - 1));
    }
    // Seamless filtering: fetch the texel across the edge from the neighbouring face.
    double rx, ry, rz;
    FaceDirection(c.face, (xi + 0.5) / n, (yi + 0.5) / n, &rx, &ry, &rz);
    const CubeCoord c2 = ResolveDirection(rx, ry, rz);
    const int x2 = std::min(std::max(static_cast<int>(std::floor(c2.s * n)), 0), n - 1);
    const int y2 = std::min(std::max(static_cast<int>(std::floor(c2.t * n)), 0), n - 1);
    return cube_tex_[c2.face].at(x2, y2);
  };
  const uint8_t* p00 = texel(static_cast<int>(x0), static_cast<int>(y0));
  const uint8_t* p10 = texel(static_cast<int>(x0) + 1, static_cast<int>(y0));
  const uint8_t* p01 = texel(static_cast<int>(x0), static_cast<int>(y0) + 1);
  const uint8_t* p11 = texel(static_cast<int>(x0) + 1, static_cast<int>(y0) + 1);
  std::array<float, 4> out{};
  for (int k = 0; k < 4; ++k) {
    const float top = static_cast<float>(p00[k]) * (1.0f - fx) + static_cast<float>(p10[k]) * fx;
    const float bot = static_cast<float>(p01[k]) * (1.0f - fx) + static_cast<float>(p11[k]) * fx;
    out[k] = (top * (1.0f - fy) + bot * fy) / 255.0f;
  }
  return out;
}

Rgba8Image SkyCubemapRenderer::RenderCubemap(const c8::HMatrix& camera_extrinsics, int width,
                                             int height) const {
  Rgba8Image out(width, height);
  if (!has_output_camera_) {
    return out;
  }
  // renderCubemap :637-686: mvp = projection * (translation(-camPos) * world).inv().
  const c8::HPoint3 pos_p = camera_extrinsics * c8::HPoint3{0.0f, 0.0f, 0.0f};
  const c8::HMatrix cam_world =
      c8::HMatrixGen::translation(-pos_p.x(), -pos_p.y(), -pos_p.z()) * camera_extrinsics;
  const c8::HMatrix mvp = output_camera_projection_ * cam_world.inv();
  const c8::HMatrix mvp_inv = mvp.inv();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double ndc_x = (x + 0.5) / width * 2.0 - 1.0;
      const double ndc_y = (y + 0.5) / height * 2.0 - 1.0;
      // A point on the pixel's view ray (z_ndc = 0, between near and far); the cube is centred on
      // the camera, so the ray direction is the point itself.
      const auto h = MulPoint(mvp_inv, ndc_x, ndc_y, 0.0);
      const auto c = SampleCubemap(static_cast<float>(h[0] / h[3]), static_cast<float>(h[1] / h[3]),
                                   static_cast<float>(h[2] / h[3]));
      uint8_t* o = out.at(x, y);
      for (int k = 0; k < 4; ++k) {
        o[k] = ToUnorm8(c[k]);
      }
    }
  }
  return out;
}

namespace {

// Quad uv of the original's unit / flip-Y / flip-Y-CW / flip-Y-CCW quads at an NDC position
// (semantics-cubemap-renderer.cc constructor :188-221; vertices (-1,1) (-1,-1) (1,1) (1,-1)).
void QuadUv(int kind, double ndc_x, double ndc_y, double* u, double* v) {
  switch (kind) {
    case 0:  // unit quad
      *u = (ndc_x + 1) / 2;
      *v = (ndc_y + 1) / 2;
      break;
    case 1:  // flipYQuad_
      *u = (ndc_x + 1) / 2;
      *v = (1 - ndc_y) / 2;
      break;
    case 2:  // flipYCWQuad_
      *u = (ndc_y + 1) / 2;
      *v = (ndc_x + 1) / 2;
      break;
    default:  // flipYCCWQuad_
      *u = (1 - ndc_y) / 2;
      *v = (1 - ndc_x) / 2;
      break;
  }
}

}  // namespace

Rgba8Image SkyCubemapRenderer::DrawOutputSemantics(const c8::HMatrix& camera_extrinsics,
                                                   int tex_width, int tex_height) {
  if (output_width_ <= 0 || output_height_ <= 0 || tex_width <= 0 || tex_height <= 0) {
    return Rgba8Image();
  }
  // Portrait render of the accumulated cubemap (useDirectSemanticsTex_ is false in the deployed
  // controller).
  const Rgba8Image portrait = RenderCubemap(camera_extrinsics, portrait_width_, portrait_height_);

  // Downscale to the small size (renderTextureDirect through the unit quad, GL_LINEAR minification).
  const int intmd_w = portrait_width_ / kIntmdDownsizeScale;
  const int intmd_h = portrait_height_ / kIntmdDownsizeScale;
  Rgba8Image small1(intmd_w, intmd_h);
  for (int y = 0; y < intmd_h; ++y) {
    for (int x = 0; x < intmd_w; ++x) {
      double u, v;
      QuadUv(0, (x + 0.5) / intmd_w * 2 - 1, (y + 0.5) / intmd_h * 2 - 1, &u, &v);
      float c[4];
      SampleBilinearClamp(portrait, u, v, c);
      for (int k = 0; k < 4; ++k) small1.at(x, y)[k] = ToUnorm8(c[k]);
    }
  }
  // Blur (vertical, then horizontal), semantics-gaussian-linear.frag.
  auto blur = [&](const Rgba8Image& src, float dir_u, float dir_v) {
    Rgba8Image dst(intmd_w, intmd_h);
    const float u_step = 1.0f / static_cast<float>(intmd_w);
    const float v_step = 1.0f / static_cast<float>(intmd_h);
    const float dir_norm = std::sqrt(dir_u * dir_u + dir_v * dir_v);
    float dir_u_norm = dir_u, dir_v_norm = dir_v;
    if (dir_norm > 0) {
      dir_u_norm = dir_u / dir_norm;
      dir_v_norm = dir_v / dir_norm;
    }
    const float step_u = dir_u_norm * u_step, step_v = dir_v_norm * v_step;
    const int half_kernel = kernel_size_ / 2;
    for (int y = 0; y < intmd_h; ++y) {
      for (int x = 0; x < intmd_w; ++x) {
        const float tex_u = (static_cast<float>(x) + 0.5f) / static_cast<float>(intmd_w);
        const float tex_v = (static_cast<float>(y) + 0.5f) / static_cast<float>(intmd_h);
        float acc[4] = {0, 0, 0, 0};
        for (int step = 0; step < kernel_size_; ++step) {
          const float offset = static_cast<float>(step - half_kernel);
          float c[4];
          SampleBilinearClamp(src, tex_u + offset * step_u, tex_v + offset * step_v, c);
          for (int k = 0; k < 4; ++k) acc[k] += kernel_[step] * c[k];
        }
        for (int k = 0; k < 4; ++k) dst.at(x, y)[k] = ToUnorm8(acc[k]);
      }
    }
    return dst;
  };
  const Rgba8Image small2 = blur(small1, 0.0f, 1.0f);
  const Rgba8Image blurred = blur(small2, 1.0f, 0.0f);

  // Sky shader (renderSkyTexture): quad by device orientation.
  int quad_kind = 1;
  if (device_orientation_ == 90) {
    quad_kind = 3;  // flipYCCWQuad_
  } else if (device_orientation_ == -90) {
    quad_kind = 2;  // flipYCWQuad_
  }
  const float smooth = 1.0f - (1.0f - edge_smoothness_) * (1.0f - edge_smoothness_);
  Rgba8Image result(tex_width, tex_height);
  for (int y = 0; y < tex_height; ++y) {
    for (int x = 0; x < tex_width; ++x) {
      double u, v;
      QuadUv(quad_kind, (x + 0.5) / tex_width * 2 - 1, (y + 0.5) / tex_height * 2 - 1, &u, &v);
      float color[4], raw[4];
      SampleBilinearClamp(blurred, u, v, color);
      SampleBilinearClamp(portrait, u, v, raw);
      const float blur_edge = std::min(std::max(color[0] - 0.5f, 0.0f), 1.0f) * 2.0f;
      const float is_marked = raw[2];
      const float sky = raw[0] * (1.0f - smooth) + blur_edge * smooth;
      float frag[4] = {sky, sky, is_marked, 1.0f};
      if (sky > 0.0f) {
        if (have_seen_sky_ > 0 && is_marked == 0.0f) {
          frag[0] = 1.0f;
          frag[1] = 1.0f;
          frag[2] = is_marked;
          frag[3] = 1.0f;
        }
        if (have_seen_sky_ == 0 && is_marked < 1.0f) {
          frag[0] = 0.0f;
          frag[1] = 0.0f;
          frag[2] = is_marked;
          frag[3] = 1.0f;
        }
      }
      if (flip_alpha_mask_ > 0) {
        frag[0] = 1.0f - frag[0];
        frag[1] = 1.0f - frag[1];
        frag[2] = is_marked;
        frag[3] = 1.0f;
      }
      for (int k = 0; k < 4; ++k) result.at(x, y)[k] = ToUnorm8(frag[k]);
    }
  }
  return result;
}

}  // namespace slam_refact::sky
