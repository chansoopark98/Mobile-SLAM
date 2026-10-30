#include "metric_scale/alignment_math.h"

#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>

#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

#include "c8/c8-log.h"
#include "common/check.h"

namespace slam_refact::metric_scale {
namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

// 1231: float seconds -> int64 ns, saturating like the wasm i64.trunc_f32_s guard.
int64_t SecondsToNs(float s) {
  const float c = s * 1000000000.0f;
  if (std::fabs(c) < 9223372036854775808.0f) {
    return static_cast<int64_t>(c);
  }
  return std::numeric_limits<int64_t>::min();
}

// 1223: int64 ns -> float seconds.
float NsToSeconds(int64_t ns) { return static_cast<float>(ns) * 1e-9f; }

// Grid period (ns) of 1181: int64(double(1.0f / fs) * 1e9), saturating.
int64_t GridPeriodNs(float fs) {
  const double p = static_cast<double>(1.0f / fs) * 1000000000.0;
  if (std::fabs(p) < 9223372036854775808.0) {
    return static_cast<int64_t>(p);
  }
  return std::numeric_limits<int64_t>::min();
}

// |v|^2 summed as z^2 + (y^2 + (x^2 + 0)) (objdump 1191/1192).
float NormSq(const Vec3& v) { return v[2] * v[2] + (v[1] * v[1] + (v[0] * v[0] + 0.0f)); }

// a.b summed as z z' + (y y' + (x x' + 0)) (objdump 1191).
float Dot(const Vec3& a, const Vec3& b) { return a[2] * b[2] + (a[1] * b[1] + (a[0] * b[0] + 0.0f)); }

Eigen::Matrix3f Skew(const Vec3& w) {
  Eigen::Matrix3f m;
  m << 0.f, -w[2], w[1], w[2], 0.f, -w[0], -w[1], w[0], 0.f;
  return m;
}

}  // namespace

float MeanDt(const std::vector<float>& t) {
  float acc = 0.0f;
  for (size_t i = 1; i < t.size(); ++i) {
    acc = (acc + t[i]) - t[i - 1];
  }
  return acc / static_cast<float>(static_cast<uint32_t>(t.size() - 1));
}

std::vector<float> PlaceOnGrid(const std::vector<float>& t, float fs, bool start_at_end) {
  std::vector<float> out;
  out.reserve(t.size());
  if (!start_at_end) {
    // Front-first variant (S5 §5.4; decoded, not executed: UNVERIFIED).
    const int64_t period = GridPeriodNs(fs);
    SR_CHECK(period != 0, "PlaceOnGrid: zero grid period");
    int64_t m = 0;
    for (float s : t) {
      const int64_t o = SecondsToNs(s) / period;
      m = (m > (o & 0xffffffffLL)) ? m : o;
      out.push_back(static_cast<float>(static_cast<double>((m & 0xffffffffLL) * period) *
                                       static_cast<double>(1e-9f)));
      m = (m + 1) & 0xffffffffLL;
    }
    return out;
  }
  std::vector<int64_t> ns;
  ns.reserve(t.size());
  for (float s : t) {
    ns.push_back(SecondsToNs(s));
  }
  std::vector<int64_t> slots;
  if (fs <= 0.0f) {
    slots = ns;
  } else if (!ns.empty()) {
    const int64_t period = GridPeriodNs(fs);
    SR_CHECK(period != 0, "PlaceOnGrid: zero grid period");
    slots.assign(ns.size(), 0);
    uint32_t f = 0xffffffffu;
    for (size_t b = ns.size(); b >= 1;) {
      --b;
      const uint32_t q = static_cast<uint32_t>(ns[b] / period);
      f = (f < q) ? f : q;
      slots[b] = static_cast<int64_t>(static_cast<uint64_t>(f) * static_cast<uint64_t>(period));
      f -= 1;
      if (f == 0) {
        break;  // earlier samples stay at slot 0
      }
    }
  }
  for (int64_t v : slots) {
    out.push_back(NsToSeconds(v));
  }
  return out;
}

std::vector<float> Interpolate(const std::vector<float>& ts, const std::vector<float>& xs,
                               const std::vector<float>& td) {
  const size_t n = ts.size();
  if (n == 0 || n != xs.size() || td.empty()) {
    return {};
  }
  if (n == 1) {
    return std::vector<float>(td.size(), xs[0]);
  }
  std::vector<float> out;
  out.reserve(td.size());
  for (float s : td) {
    size_t lo = 0;
    size_t count = n;
    while (count > 0) {  // lower_bound
      const size_t step = count / 2;
      if (ts[lo + step] < s) {
        lo += step + 1;
        count -= step + 1;
      } else {
        count = step;
      }
    }
    size_t f;
    if (lo == 0) {
      f = 0;
    } else if (lo == n) {
      f = n - 1;
    } else {
      f = (std::fabs(s - ts[lo - 1]) < std::fabs(s - ts[lo])) ? lo - 1 : lo;
    }
    const float t = ts[f];
    size_t i0;
    size_t i1;
    if (t > s) {
      i0 = f >= 1 ? f - 1 : f;
      i1 = f >= 1 ? f : f + 1;
    } else {
      i0 = f < n - 1 ? f : f - 1;
      i1 = f < n - 1 ? f + 1 : f;
    }
    const float dt = ts[i1] - ts[i0];
    const float w = dt == 0.0f ? 0.0f : (xs[i1] - xs[i0]) / dt;
    out.push_back(s * w + (xs[f] - t * w));  // intercept first (S5 §5.6)
  }
  return out;
}

std::vector<Vec3> Interpolate3(const std::vector<float>& ts, const std::vector<Vec3>& xs,
                               const std::vector<float>& td) {
  if (xs.empty()) {
    return {};
  }
  std::array<std::vector<float>, 3> cols;
  for (int j = 0; j < 3; ++j) {
    std::vector<float> axis;
    axis.reserve(xs.size());
    for (const Vec3& v : xs) {
      axis.push_back(v[j]);
    }
    cols[j] = Interpolate(ts, axis, td);
  }
  std::vector<Vec3> out(cols[0].size());
  for (size_t i = 0; i < out.size(); ++i) {
    out[i] = {cols[0][i], cols[1][i], cols[2][i]};
  }
  return out;
}

std::vector<float> CrossCorrelate(const std::vector<float>& x, const std::vector<float>& y) {
  if (x.empty() || y.empty()) {
    return {};
  }
  const size_t n = x.size();
  const size_t m = y.size();
  const size_t k = n > m ? n : m;
  std::vector<float> out(n + m - 1);
  for (size_t h = 0; h < out.size(); ++h) {
    const size_t o = k - 1 - h;  // wraps (unsigned), like the wasm i32 index
    float t = 0.0f;
    for (size_t i = 0; i < n; ++i) {
      const size_t r = o + i;
      const float v = r < m ? y[r] : 0.0f;
      t = x[i] * v + t;
    }
    out[h] = t;
  }
  return out;
}

void ShiftTimes(float dt, std::vector<float>* t) {
  for (float& v : *t) {
    v += dt;
  }
}

void CropToRange(float lo, float hi, std::vector<float>* t, std::vector<Vec3>* data) {
  std::vector<float> kept_t;
  std::vector<Vec3> kept_d;
  for (size_t i = 0; i < t->size(); ++i) {
    const float v = (*t)[i];
    if (lo <= v && v <= hi) {
      kept_t.push_back(v);
      if (data != nullptr && i < data->size()) {
        kept_d.push_back((*data)[i]);
      }
    }
  }
  *t = std::move(kept_t);
  if (data != nullptr) {
    *data = std::move(kept_d);
  }
}

std::vector<float> MergeTimes(const std::vector<float>& a, const std::vector<float>& b) {
  std::vector<float> out;
  out.reserve(a.size() + b.size());
  size_t i = 0;
  size_t j = 0;
  while (i < a.size() && j < b.size()) {
    if (a[i] < b[j]) {
      out.push_back(a[i++]);
    } else {
      out.push_back(b[j++]);
    }
  }
  out.insert(out.end(), a.begin() + static_cast<std::ptrdiff_t>(i), a.end());
  out.insert(out.end(), b.begin() + static_cast<std::ptrdiff_t>(j), b.end());
  return out;
}

std::vector<float> Dedupe(const std::vector<float>& t, float eps) {
  std::vector<float> out;
  for (float v : t) {
    if (out.empty() || v - out.back() > eps) {
      out.push_back(v);
    }
  }
  return out;
}

std::vector<float> GaussianKernel(float sigma, int size) {
  const int half = size / 2;
  const int len = (half << 1) | 1;
  std::vector<float> k;
  if (len <= 0) {
    return k;
  }
  const float denom = sigma * -2.0f * sigma;
  const double norm = 1.0 / static_cast<double>(sigma) * 2.5066282746310002;
  float total = 0.0f;
  for (int d = 0; d < len; ++d) {
    const int i = d - half;
    const double o = norm * static_cast<double>(std::exp(static_cast<float>(i * i) / denom));
    k.push_back(static_cast<float>(o));
    total = static_cast<float>(o + static_cast<double>(total));
  }
  for (float& v : k) {
    v = v / total;
  }
  return k;
}

std::vector<float> GaussianSmooth(const std::vector<float>& x, float sigma, int size) {
  const std::vector<float> k = GaussianKernel(sigma, size);
  if (k.empty()) {
    return x;
  }
  const int half = static_cast<int>(k.size() - 1) / 2;
  const int n = static_cast<int>(x.size());
  std::vector<float> out(x.size());
  for (int i = 0; i < n; ++i) {
    float c = 0.0f;
    float s = 0.0f;
    for (int j = 0; j < static_cast<int>(k.size()); ++j) {
      const int d = i - half + j;
      if (d >= 0 && d < n) {
        c = k[j] * x[d] + c;
        s = s + k[j];
      }
    }
    out[i] = c / s;
  }
  return out;
}

std::vector<Vec3> GaussianSmooth3(const std::vector<Vec3>& x, float sigma, int size) {
  std::array<std::vector<float>, 3> cols;
  for (int j = 0; j < 3; ++j) {
    std::vector<float> axis;
    axis.reserve(x.size());
    for (const Vec3& v : x) {
      axis.push_back(v[j]);
    }
    cols[j] = GaussianSmooth(axis, sigma, size);
  }
  std::vector<Vec3> out(x.size());
  for (size_t i = 0; i < out.size(); ++i) {
    out[i] = {cols[0][i], cols[1][i], cols[2][i]};
  }
  return out;
}

ScaleFit ScaleCameraSpace(const std::vector<Vec3>& v, const std::vector<Vec3>& imu) {
  ScaleFit fit;
  const size_t n = v.size();
  if (n == 0) {  // 0x7fc00000 scale; residual 0 / float(0u) = NaN
    fit.scale = kNan;
    fit.residual = kNan;
    return fit;
  }
  float num = Dot(v[0], imu[0]) + 0.0f;
  float den = Dot(v[0], v[0]) + 0.0f;
  for (size_t i = 1; i < n; ++i) {
    num = num + Dot(v[i], imu[i]);
    den = den + Dot(v[i], v[i]);
  }
  const float s = std::fabs(num / den);
  float res = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    const Vec3 d = {imu[i][0] - s * v[i][0], imu[i][1] - s * v[i][1], imu[i][2] - s * v[i][2]};
    res = res + NormSq(d);
  }
  fit.scale = s;
  fit.residual = res / static_cast<float>(static_cast<uint32_t>(n));
  return fit;
}

ScaleFit ScaleL2(const std::vector<Vec3>& v, const std::vector<Vec3>& imu) {
  ScaleFit fit;
  const size_t n = v.size();
  if (n == 0) {  // 0x7fc00000 scale; residual 0 / float(0u) = NaN
    fit.scale = kNan;
    fit.residual = kNan;
    return fit;
  }
  float num = 0.0f;
  float den = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    const float ni = std::sqrt(NormSq(imu[i]));
    const float nv = std::sqrt(NormSq(v[i]));
    num = nv * ni + num;
    den = nv * nv + den;
  }
  const float s = num / den;
  const float neg = -s;
  float res = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    const double d = static_cast<double>(neg * std::sqrt(NormSq(v[i])) + std::sqrt(NormSq(imu[i])));
    res = static_cast<float>(d * d + static_cast<double>(res));
  }
  fit.scale = s;
  fit.residual = res / static_cast<float>(static_cast<uint32_t>(n));
  return fit;
}

FftCompute::FftCompute(int n) : n_(n) {}

void FftCompute::Compute(const std::vector<float>& data, float sampling_rate) {
  std::vector<std::complex<float>> in(static_cast<size_t>(n_));
  std::vector<std::complex<float>> out(static_cast<size_t>(n_));
  for (int i = 0; i < n_; ++i) {
    in[i] = {data[i], 0.0f};
  }
  // Complex-input forward transform (the real-input overload takes a half-length path, S5 §7.3).
  Eigen::FFT<float> fft;
  fft.fwd(out, in);
  frequencies_.assign(static_cast<size_t>(n_ / 2), 0.0f);
  amplitudes_.assign(static_cast<size_t>(n_ / 2), 0.0f);
  for (int i = 0; i < n_ / 2; ++i) {
    const float re = out[i].real();
    const float im = out[i].imag();
    amplitudes_[i] = std::sqrt(re * re + im * im) * 1.f / static_cast<float>(n_);
    frequencies_[i] = static_cast<float>(i) * sampling_rate / static_cast<float>(n_);
  }
}

ScaleFit ScaleFrequency(const std::vector<float>& v_t, const std::vector<Vec3>& v_a,
                        const std::vector<Vec3>& imu, const std::vector<float>& t_c,
                        float max_freq) {
  // 1239 pushBackCheckNan: logs and keeps going.
  for (const Vec3& s : imu) {
    for (float c : s) {
      if (std::isnan(c)) {
        c8::C8Log("[frequency-scale-estimator] Found nan value");
      }
    }
  }
  float dt_mean = 0.0f;
  if (t_c.size() > 1) {
    float acc = 0.0f;
    for (size_t h = 1; h < t_c.size(); ++h) {
      acc = acc + (t_c[h] - t_c[h - 1]);
    }
    dt_mean = acc / static_cast<float>(t_c.size() - 1);
  }
  const float fs = 1.0f / dt_mean;
  const std::vector<Vec3> vio = Interpolate3(v_t, v_a, t_c);
  const int n = static_cast<int>(t_c.size());
  std::array<float, 3> num{0.f, 0.f, 0.f};
  std::array<float, 3> den{0.f, 0.f, 0.f};
  for (int j = 0; j < 3; ++j) {
    std::vector<float> ai(static_cast<size_t>(n), 0.f);
    std::vector<float> av(static_cast<size_t>(n), 0.f);
    for (int i = 0; i < n; ++i) {
      ai[i] = i < static_cast<int>(imu.size()) ? imu[i][j] : 0.f;
      av[i] = i < static_cast<int>(vio.size()) ? vio[i][j] : 0.f;
    }
    FftCompute fi(n);
    FftCompute fv(n);
    fi.Compute(ai, fs);
    fv.Compute(av, fs);
    SR_CHECK(fi.frequencies() == fv.frequencies(), "Can only compute diff for same frequencies");
    for (size_t b = 0; b < fi.frequencies().size(); ++b) {
      if (fi.frequencies()[b] > max_freq) {
        continue;
      }
      num[j] = fi.amplitudes()[b] * fv.amplitudes()[b] + num[j];
      den[j] = fv.amplitudes()[b] * fv.amplitudes()[b] + den[j];
    }
  }
  ScaleFit fit;
  fit.scale = ((num[0] + num[1]) + num[2]) / ((den[0] + den[1]) + den[2]);
  fit.residual = -1.f;
  return fit;
}

ScaleFit ScaleSpatial(const std::vector<Vec3>& v, const std::vector<Vec3>& imu,
                      const std::vector<Vec3>& omega, const std::vector<Vec3>& omega_dot) {
  Eigen::Matrix4f ata = Eigen::Matrix4f::Zero();
  Eigen::Vector4f atb = Eigen::Vector4f::Zero();
  const size_t n = v.size();
  for (size_t k = 0; k < n; ++k) {
    const Vec3 w = k < omega.size() ? omega[k] : Vec3{0.f, 0.f, 0.f};
    const Vec3 al = k < omega_dot.size() ? omega_dot[k] : Vec3{0.f, 0.f, 0.f};
    const Eigen::Matrix3f wx = Skew(w);
    const Eigen::Matrix3f m = wx * wx + Skew(al);
    Eigen::Matrix<float, 3, 4> row;
    row.leftCols<3>() = m;
    row.col(3) << v[k][0], v[k][1], v[k][2];
    const Eigen::Vector3f b(imu[k][0], imu[k][1], imu[k][2]);
    ata.noalias() += row.transpose() * row;
    atb.noalias() += row.transpose() * b;
  }
  // Eigen LDLT: solve() zeroes components whose |D| <= FLT_MIN (S5 §7.2).
  const Eigen::Vector4f x = Eigen::LDLT<Eigen::Matrix4f>(ata).solve(atb);
  ScaleFit fit;
  fit.scale = std::fabs(x(3));
  fit.residual = -1.f;
  return fit;
}

LeverArm FixedLeverArm(int32_t device_model) {
  // 1196: br_table on deviceModel - 52 (device names INFERRED from the public enum order).
  struct Entry {
    int32_t index;
    Vec3 t;
    std::array<float, 4> q;
  };
  static constexpr Entry kTable[] = {
      {0, {-0.05524266138672829f, -0.08398990333080292f, 0.00020957000378984958f},
       {0.0003271400055382401f, -0.016569100320339203f, 0.009478099644184113f, -0.9998176097869873f}},
      {1, {-0.04877173900604248f, -0.037656109780073166f, -0.00824609026312828f},
       {0.009567679837346077f, -0.0011512900236994028f, 0.0026723500341176987f, -0.9999499320983887f}},
      {2, {-0.0487731508910656f, -0.09466294944286346f, -0.008556979708373547f},
       {0.008143840357661247f, -0.002145460108295083f, -0.006375249940901995f, -0.9999441504478455f}},
      {5, {-0.05398846045136452f, -0.07146242260932922f, -0.004572100006043911f},
       {-0.006610630080103874f, -0.010129470378160477f, -0.0018680599750950933f, 0.999924898147583f}},
      {6, {-0.020575640723109245f, 0.002507690107449889f, -0.0038582501001656055f},
       {0.005582420155405998f, -0.0012088599614799023f, 0.00019870999676641077f, 0.9999836683273315f}},
      {8, {-0.026119699701666832f, -0.013809259980916977f, -0.007925550453364849f},
       {0.0016072100261226296f, -0.0010944399982690811f, -0.00028418999863788486f, 0.9999979734420776f}},
      {9, {-0.052289459854364395f, -0.017155729234218597f, -0.007813540287315845f},
       {-0.0016219000099226832f, -0.0037101299967616796f, -0.0018988300580531359f, 0.9999899864196777f}},
      {21, {-0.018319450318813324f, -0.0016115499893203378f, -0.006659059785306454f},
       {0.002013830002397299f, -0.0006014999817125499f, -0.0016365599585697055f, -0.9999963641166687f}},
      {27, {-0.006726529914885759f, -0.04026602953672409f, -0.009898499585688114f},
       {0.0037452001124620438f, -0.003220590064302087f, -0.001021270058117807f, -0.9999873042106628f}},
      {33, {-0.00452703982591629f, -0.012137290090322495f, -0.0029716899152845144f},
       {0.0006766300066374242f, 0.0005609199870377779f, -0.002454020082950592f, 0.9999964833259583f}},
      {34, {-0.04694743826985359f, 0.0031610499136149883f, -0.004694050177931786f},
       {0.0030963700264692307f, 0.002758079906925559f, 0.0023811599239706993f, 0.9999884366989136f}},
      {35, {-0.004793659783899784f, -0.02464609034359455f, 0.001762610045261681f},
       {-0.00027660001069307327f, -0.003420789958909154f, 0.001196729950606823f, -0.9999933242797852f}},
      {47, {-0.02377321943640709f, -0.005135590210556984f, -0.005503740161657333f},
       {0.01577216014266014f, -0.005471639800816774f, 0.0031340098939836025f, -0.999855637550354f}},
      {49, {-0.024320760741829872f, -0.015374859794974327f, -0.009302579797804356f},
       {0.007921960204839706f, -0.0021419799886643887f, 0.0014542200369760394f, -0.9999651908874512f}},
      {50, {-0.013132399879395962f, 0.0021084900945425034f, -0.007177799940109253f},
       {0.007033499889075756f, 0.004118780139833689f, 0.002795160049572587f, 0.9999628067016602f}},
      {55, {0.02604248933494091f, -0.011477289721369743f, -0.008395910263061523f},
       {0.0023205899633467197f, 0.001248529995791614f, 0.002399310003966093f, 0.9999935030937195f}},
      {56, {0.017606519162654877f, -0.009105259552598f, -0.0028818000573664904f},
       {-0.002461459953337908f, -0.004263539798557758f, 0.0020741100888699293f, 0.9999855756759644f}},
  };
  LeverArm arm;
  const uint32_t index = static_cast<uint32_t>(device_model - 52);
  for (const Entry& e : kTable) {
    if (static_cast<uint32_t>(e.index) == index) {
      arm.translation = e.t;
      arm.rotation_xyzw = e.q;
    }
  }
  return arm;
}

ScaleFit ScaleFixedSpatial(const std::vector<Vec3>& v, const std::vector<Vec3>& imu,
                           const std::vector<Vec3>& omega, const std::vector<Vec3>& omega_dot,
                           const Vec3& lever_arm) {
  const Vec3& r = lever_arm;
  std::vector<Vec3> corrected;
  corrected.reserve(v.size());
  for (size_t k = 0; k < v.size(); ++k) {
    const Vec3 md = k < omega_dot.size() ? omega_dot[k] : Vec3{0.f, 0.f, 0.f};
    const Vec3 om = k < omega.size() ? omega[k] : Vec3{0.f, 0.f, 0.f};
    // w = W x r
    const float wx = om[1] * r[2] - r[1] * om[2];
    const float wy = om[2] * r[0] - r[2] * om[0];
    const float wz = om[0] * r[1] - r[0] * om[1];
    const float cz = md[0] * r[1] - r[0] * md[1] + v[k][2] + om[0] * wy - wx * om[1];
    const float cy = v[k][1] + md[2] * r[0] - r[2] * md[0] + om[2] * wx - wz * om[0];
    const float cx = v[k][0] + md[1] * r[2] - r[1] * md[2] + om[1] * wz - wy * om[2];
    corrected.push_back({cx, cy, cz});
  }
  return ScaleCameraSpace(corrected, imu);
}

}  // namespace slam_refact::metric_scale
