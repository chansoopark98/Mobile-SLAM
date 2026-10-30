#include "engine/floor_detector.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace slam_refact {
namespace {

struct P2 {
  float x;
  float z;
};

float Cross(const P2& o, const P2& a, const P2& b) {
  return (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
}

// Andrew's monotone chain; returns the hull counter-clockwise without repeating the first point.
std::vector<P2> ConvexHull(std::vector<P2> pts) {
  std::sort(pts.begin(), pts.end(), [](const P2& a, const P2& b) {
    return a.x < b.x || (a.x == b.x && a.z < b.z);
  });
  pts.erase(std::unique(pts.begin(), pts.end(),
                        [](const P2& a, const P2& b) { return a.x == b.x && a.z == b.z; }),
            pts.end());
  if (pts.size() < 3) return pts;
  std::vector<P2> h(2 * pts.size());
  size_t k = 0;
  for (const P2& p : pts) {
    while (k >= 2 && Cross(h[k - 2], h[k - 1], p) <= 0.f) --k;
    h[k++] = p;
  }
  for (size_t i = pts.size() - 1, t = k + 1; i-- > 0;) {
    while (k >= t && Cross(h[k - 2], h[k - 1], pts[i]) <= 0.f) --k;
    h[k++] = pts[i];
  }
  h.resize(k - 1);
  return h;
}

float HullArea(const std::vector<Vec3f>& hull) {
  float a = 0.f;
  for (size_t i = 0; i < hull.size(); ++i) {
    const Vec3f& p = hull[i];
    const Vec3f& q = hull[(i + 1) % hull.size()];
    a += p.x * q.z - q.x * p.z;
  }
  return 0.5f * std::fabs(a);
}

bool InsideHull(const std::vector<Vec3f>& hull, float x, float z) {
  if (hull.size() < 3) return false;
  for (size_t i = 0; i < hull.size(); ++i) {
    const Vec3f& a = hull[i];
    const Vec3f& b = hull[(i + 1) % hull.size()];
    if ((b.x - a.x) * (z - a.z) - (b.z - a.z) * (x - a.x) < 0.f) return false;
  }
  return true;
}

float Median(std::vector<float> v) {
  const size_t m = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(m), v.end());
  return v[m];
}

}  // namespace

FloorDetector::FloorDetector(const FloorDetectorParams& params) : params_(params) {}

void FloorDetector::Reset() {
  cloud_.clear();
  frame_ = 0;
  state_ = SurfaceState{};
  have_height_ = false;
  smoothed_height_ = 0.f;
}

void FloorDetector::Update(const Vec3f& camera, const std::vector<WorldPointOut>& points) {
  ++frame_;
  for (const WorldPointOut& w : points) {
    if (w.id == 0) continue;  // 0-padding (Q10)
    if (!std::isfinite(w.position.x) || !std::isfinite(w.position.y) || !std::isfinite(w.position.z)) {
      continue;
    }
    cloud_[w.id] = Stored{w.position, frame_};
  }
  for (auto it = cloud_.begin(); it != cloud_.end();) {
    if (frame_ - it->second.frame > params_.max_age_frames) {
      it = cloud_.erase(it);
    } else {
      ++it;
    }
  }
  if (static_cast<int>(cloud_.size()) > params_.max_points) {
    std::vector<std::pair<int64_t, uint64_t>> by_age;
    by_age.reserve(cloud_.size());
    for (const auto& [id, s] : cloud_) by_age.emplace_back(s.frame, id);
    std::sort(by_age.begin(), by_age.end());
    const size_t drop = cloud_.size() - static_cast<size_t>(params_.max_points);
    for (size_t k = 0; k < drop; ++k) cloud_.erase(by_age[k].second);
  }
  Detect(camera);
}

void FloorDetector::Detect(const Vec3f& camera) {
  state_.cloud_points = static_cast<int>(cloud_.size());
  std::vector<Vec3f> below;
  below.reserve(cloud_.size());
  for (const auto& [id, s] : cloud_) {
    if (s.p.y < camera.y - params_.min_below && s.p.y > camera.y - params_.max_below) {
      below.push_back(s.p);
    }
  }
  state_.others.clear();
  if (static_cast<int>(below.size()) < params_.min_inliers) {
    state_.floor_detected = false;
    return;
  }
  float lo = below[0].y, hi = below[0].y;
  for (const Vec3f& p : below) {
    lo = std::min(lo, p.y);
    hi = std::max(hi, p.y);
  }
  const float bin = params_.bin;
  const int nbins = std::min(4000, static_cast<int>((hi - lo) / bin) + 3);
  std::vector<int> hist(static_cast<size_t>(nbins), 0);
  for (const Vec3f& p : below) {
    const int b = std::clamp(static_cast<int>((p.y - lo) / bin) + 1, 0, nbins - 1);
    ++hist[static_cast<size_t>(b)];
  }
  std::vector<int> smooth(hist.size(), 0);
  for (int b = 0; b < nbins; ++b) {
    for (int d = -1; d <= 1; ++d) {
      const int k = b + d;
      if (k >= 0 && k < nbins) smooth[static_cast<size_t>(b)] += hist[static_cast<size_t>(k)];
    }
  }
  std::vector<HorizontalSurface> candidates;
  for (int b = 0; b < nbins; ++b) {
    const int c = smooth[static_cast<size_t>(b)];
    if (c < params_.min_inliers) continue;
    const int left = b > 0 ? smooth[static_cast<size_t>(b - 1)] : 0;
    const int right = b + 1 < nbins ? smooth[static_cast<size_t>(b + 1)] : 0;
    if (c < left || c <= right) continue;
    const float center = lo + (static_cast<float>(b) - 0.5f) * bin;
    std::vector<float> ys;
    std::vector<P2> xz;
    for (const Vec3f& p : below) {
      if (std::fabs(p.y - center) <= 2.f * bin) ys.push_back(p.y);
    }
    if (static_cast<int>(ys.size()) < params_.min_inliers) continue;
    const float h = Median(ys);
    for (const Vec3f& p : below) {
      if (std::fabs(p.y - h) <= 2.f * bin) xz.push_back({p.x, p.z});
    }
    HorizontalSurface s;
    s.height = h;
    s.inliers = static_cast<int>(xz.size());
    for (const P2& q : ConvexHull(xz)) s.hull.push_back(Vec3f{q.x, h, q.z});
    s.area = HullArea(s.hull);
    s.confidence = std::min(1.f, static_cast<float>(s.inliers) / 60.f);
    candidates.push_back(std::move(s));
  }
  // Merge candidates closer than 3 bins (keep the better supported).
  std::sort(candidates.begin(), candidates.end(),
            [](const HorizontalSurface& a, const HorizontalSurface& b) { return a.height < b.height; });
  std::vector<HorizontalSurface> merged;
  for (HorizontalSurface& c : candidates) {
    if (!merged.empty() && std::fabs(c.height - merged.back().height) < 3.f * bin) {
      if (c.inliers > merged.back().inliers) merged.back() = std::move(c);
      continue;
    }
    merged.push_back(std::move(c));
  }
  int floor_idx = -1;
  for (size_t k = 0; k < merged.size(); ++k) {
    if (static_cast<float>(merged[k].inliers) >=
        params_.min_floor_fraction * static_cast<float>(below.size())) {
      floor_idx = static_cast<int>(k);
      break;
    }
  }
  if (floor_idx < 0) {
    state_.floor_detected = false;
    return;
  }
  HorizontalSurface floor = merged[static_cast<size_t>(floor_idx)];
  if (have_height_ && std::fabs(floor.height - smoothed_height_) < 6.f * bin) {
    smoothed_height_ += params_.height_alpha * (floor.height - smoothed_height_);
  } else {
    smoothed_height_ = floor.height;  // first detection or a different surface became the floor
    have_height_ = true;
  }
  floor.height = smoothed_height_;
  for (Vec3f& v : floor.hull) v.y = smoothed_height_;
  state_.floor = std::move(floor);
  state_.floor_detected = true;
  for (size_t k = static_cast<size_t>(floor_idx) + 1; k < merged.size(); ++k) {
    state_.others.push_back(std::move(merged[k]));
  }
  // Total order: inliers descending, then height ascending (merged heights are distinct, so no ties remain).
  std::sort(state_.others.begin(), state_.others.end(),
            [](const HorizontalSurface& a, const HorizontalSurface& b) {
              if (a.inliers != b.inliers) return a.inliers > b.inliers;
              return a.height < b.height;
            });
}

std::vector<HitResult> FloorDetector::HitTest(const Vec3f& o, const Vec3f& dir, uint32_t mask) const {
  std::vector<HitResult> hits;
  const float n = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
  if (!(n > 0.f)) return hits;
  const Vec3f d{dir.x / n, dir.y / n, dir.z / n};
  const auto plane_hit = [&](const HorizontalSurface& s, int index, bool estimated_allowed) {
    if (std::fabs(d.y) < 1e-4f) return;
    const float t = (s.height - o.y) / d.y;
    if (t <= 0.f) return;
    const Vec3f p{o.x + t * d.x, s.height, o.z + t * d.z};
    const bool inside = InsideHull(s.hull, p.x, p.z);
    if (inside && (mask & static_cast<uint32_t>(HitType::kDetectedSurface)) != 0) {
      hits.push_back({HitType::kDetectedSurface, p, t, index});
    } else if (!inside && estimated_allowed &&
               (mask & static_cast<uint32_t>(HitType::kEstimatedSurface)) != 0) {
      hits.push_back({HitType::kEstimatedSurface, p, t, index});
    }
  };
  if (state_.floor_detected) {
    plane_hit(state_.floor, 0, true);
    for (size_t k = 0; k < state_.others.size(); ++k) {
      plane_hit(state_.others[k], static_cast<int>(k) + 1, false);
    }
  }
  if ((mask & static_cast<uint32_t>(HitType::kFeaturePoint)) != 0) {
    constexpr float kSigma = 0.02f;  // radians
    double wsum = 0.0, dsum = 0.0;
    for (const auto& [id, s] : cloud_) {
      const Vec3f v{s.p.x - o.x, s.p.y - o.y, s.p.z - o.z};
      const float along = v.x * d.x + v.y * d.y + v.z * d.z;
      if (along <= 0.f) continue;
      const float vn = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
      const float cosang = std::clamp(along / vn, -1.f, 1.f);
      const float ang = std::acos(cosang);
      if (ang > 3.f * kSigma) continue;
      const double w = std::exp(-0.5 * (ang / kSigma) * (ang / kSigma));
      wsum += w;
      dsum += w * along;
    }
    if (wsum > 1e-3) {
      const float t = static_cast<float>(dsum / wsum);
      hits.push_back({HitType::kFeaturePoint, Vec3f{o.x + t * d.x, o.y + t * d.y, o.z + t * d.z}, t, -1});
    }
  }
  // Total order: distance, then hit type, then surface (each (type, surface) occurs once).
  std::sort(hits.begin(), hits.end(), [](const HitResult& a, const HitResult& b) {
    if (a.distance != b.distance) return a.distance < b.distance;
    if (a.type != b.type) return static_cast<uint32_t>(a.type) < static_cast<uint32_t>(b.type);
    return a.surface < b.surface;
  });
  return hits;
}

}  // namespace slam_refact
