// Checks that Eigen's unsupported/Eigen/FFT (kissfft backend) reproduces c8::FftCompute
// (public reality/engine/tracking/fft-compute.cc, linked in xr-slam as func1240/1241) on the
// golden vectors of mse-probe.mjs P6. Golden amplitudes are read from stdin as
// "n amp0 amp1 ..." lines (one per FFT size; see spec 05 section 7.3 / commands.txt).
//
// Build: g++ -std=c++17 -O2 -ffp-contract=off -I/usr/include/eigen3 eigen_fft_check.cc
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <unsupported/Eigen/FFT>

namespace {

// Same LCG as mse-probe.mjs (Math.imul(s, 1664525) + 1013904223) >>> 0, / 2^32.
struct Lcg {
  uint32_t s;
  double next() {
    s = s * 1664525u + 1013904223u;
    return s / 4294967296.0;
  }
};

// FftCompute::compute semantics: complex FFT of real input (imag 0), amplitude |X_k| / N for
// k in [0, N/2) (DC included), frequency k * fs / N computed in float.
void FftComputeEigen(const std::vector<float>& data, float fs, std::vector<float>* freqs,
                     std::vector<float>* amps) {
  const int n = static_cast<int>(data.size());
  std::vector<std::complex<float>> in(n), out(n);
  for (int i = 0; i < n; ++i) in[i] = {data[i], 0.0f};
  Eigen::FFT<float> fft;
  fft.fwd(out, in);
  freqs->resize(n / 2);
  amps->resize(n / 2);
  for (int i = 0; i < n / 2; ++i) {
    (*amps)[i] = std::sqrt(out[i].real() * out[i].real() + out[i].imag() * out[i].imag()) * 1.f / n;
    (*freqs)[i] = i * fs / n;
  }
}

}  // namespace

int main() {
  Lcg rnd{6};
  int failures = 0;
  double worst = 0.0;
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    int n;
    is >> n;
    std::vector<float> golden;
    float v;
    while (is >> v) golden.push_back(v);
    std::vector<float> data(n);
    for (int i = 0; i < n; ++i) {
      double x = std::sin(2 * M_PI * 3.0 * i / 60) + 0.5 * std::cos(2 * M_PI * 7.5 * i / 60) +
                 0.1 * (rnd.next() - 0.5);
      data[i] = static_cast<float>(x);
    }
    std::vector<float> freqs, amps;
    FftComputeEigen(data, 60.0f, &freqs, &amps);
    float max_amp = 0;
    for (float a : golden) max_amp = std::max(max_amp, a);
    const double tol = 1e-5 * max_amp + 1e-7;  // fixed in spec section 8
    double max_abs = 0;
    int exact = 0;
    for (int i = 0; i < n / 2; ++i) {
      max_abs = std::max(max_abs, static_cast<double>(std::fabs(amps[i] - golden[i])));
      exact += amps[i] == golden[i];
    }
    worst = std::max(worst, max_abs / max_amp);
    const bool ok = static_cast<int>(golden.size()) == n / 2 && max_abs <= tol;
    failures += !ok;
    std::printf("n=%d bins=%d bitwise_equal=%d max_abs_diff=%.3g tol=%.3g %s\n", n, n / 2, exact,
                max_abs, tol, ok ? "PASS" : "FAIL");
  }
  std::printf("worst_rel=%.3g %s\n", worst, failures ? "FAIL" : "PASS");
  return failures ? 1 : 0;
}
