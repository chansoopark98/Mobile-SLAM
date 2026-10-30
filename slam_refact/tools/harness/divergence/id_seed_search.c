// Search the 32-bit seed s of a libc++ std::mt19937 (seeded with one word, G641) such that draw j1 == u1 and draw
// j2 == u2. libc++ regenerates x[i] in place per draw: draw j (< 624) uses the seeding values x[j], x[j+1] and
// x[j+397] (j < 227) or the regenerated x[j-227] (227 <= j <= 453, itself from seeding values x[j-227], x[j-226],
// x[j+170]). usage: seedsearch <j1> <u1> <j2> <u2> <thread index> <threads>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define L 16
static inline uint32_t temper(uint32_t y) {
  y ^= y >> 11;
  y ^= (y << 7) & 0x9d2c5680u;
  y ^= (y << 15) & 0xefc60000u;
  y ^= y >> 18;
  return y;
}
static inline uint32_t regen(uint32_t xi, uint32_t xi1, uint32_t xm) {
  const uint32_t y = (xi & 0x80000000u) | (xi1 & 0x7fffffffu);
  return xm ^ (y >> 1) ^ ((y & 1u) ? 0x9908b0dfu : 0u);
}
static uint32_t draw(uint32_t seed, int j) {
  uint32_t x[624];
  x[0] = seed;
  for (int k = 1; k < 624; ++k) x[k] = 1812433253u * (x[k - 1] ^ (x[k - 1] >> 30)) + (uint32_t)k;
  uint32_t out = 0;
  for (int i = 0; i <= j; ++i) {
    x[i] = regen(x[i], x[(i + 1) % 624], x[(i + 397) % 624]);
    out = temper(x[i]);
  }
  return out;
}
int main(int argc, char** argv) {
  if (argc != 7) { fprintf(stderr, "usage: id_seed_search j1 u1 j2 u2 t T\n"); return 2; }
  const int j1 = atoi(argv[1]);
  const uint32_t u1 = (uint32_t)strtoul(argv[2], 0, 10);
  const int j2 = atoi(argv[3]);
  const uint32_t u2 = (uint32_t)strtoul(argv[4], 0, 10);
  const uint64_t ti = strtoull(argv[5], 0, 10), tn = strtoull(argv[6], 0, 10);
  if (j1 < 1 || j1 > 453) { fprintf(stderr, "j1 must be in [1, 453]\n"); return 2; }
  const int hiidx = j1 < 227 ? j1 + 397 : j1 + 170;
  const uint64_t chunk = (1ull << 32) / tn;
  const uint64_t lo = ti * chunk, hi = (ti + 1 == tn) ? (1ull << 32) : lo + chunk;
  for (uint64_t base = lo; base < hi; base += L) {
    uint32_t x[L], a[L], b[L], c[L], d[L], e[L];
    for (int l = 0; l < L; ++l) x[l] = (uint32_t)(base + l);
    for (int k = 1; k <= hiidx; ++k) {
      for (int l = 0; l < L; ++l) x[l] = 1812433253u * (x[l] ^ (x[l] >> 30)) + (uint32_t)k;
      if (k == j1) for (int l = 0; l < L; ++l) a[l] = x[l];
      if (k == j1 + 1) for (int l = 0; l < L; ++l) b[l] = x[l];
      if (j1 >= 227 && k == j1 - 227) for (int l = 0; l < L; ++l) d[l] = x[l];
      if (j1 >= 227 && k == j1 - 226) for (int l = 0; l < L; ++l) e[l] = x[l];
    }
    for (int l = 0; l < L; ++l) c[l] = x[l];
    for (int l = 0; l < L; ++l) {
      uint32_t xm = c[l];
      if (j1 >= 227) xm = regen(d[l], e[l], c[l]);
      if (temper(regen(a[l], b[l], xm)) == u1) {
        const uint32_t s = (uint32_t)(base + l);
        if (draw(s, j2) == u2) { printf("seed %u\n", s); fflush(stdout); }
      }
    }
  }
  return 0;
}
