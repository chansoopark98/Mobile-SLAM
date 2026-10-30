// Cross-check of mt_tools.py against libstdc++: prints for n in a few values the text state conversion result ("to_libstdcxx_text" input comes from
// python) and the libstdc++ next-8 signature after n draws.  Usage: mt_check <n> <text-state-file>  -> prints "ok" when libstdc++ loaded from the
// text state yields the same next 8 outputs as a fresh std::mt19937(5489) advanced by n draws.
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
int main(int argc, char** argv) {
  const unsigned long long n = std::strtoull(argv[1], nullptr, 10);
  std::ifstream f(argv[2]);
  std::stringstream ss;
  ss << f.rdbuf();
  std::mt19937 a(5489), b;
  a.discard(n);
  ss >> b;
  bool ok = !ss.fail();
  for (int k = 0; k < 8 && ok; ++k) ok = a() == b();
  std::printf("%s n=%llu\n", ok ? "ok" : "MISMATCH", n);
  return ok ? 0 : 1;
}
