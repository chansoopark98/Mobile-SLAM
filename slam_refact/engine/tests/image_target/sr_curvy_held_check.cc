// Command line front of the curved target HELD device (curvy_sr_held.h over support/sr_held.h) for tests that are not C++ (the
// WASM node test):
//   sr_curvy_held_check --kind <kind> --data-dir <dir> [--pin <file>]...
// The gate (id, version, preregistration directory), the designation and the pinned files come from the environment
// (SLAM_REFACT_CURVY_GATE_ID, _GATE_VERSION, _PREREG_DIR, _DESIGNATION, _PINS; no default, see curvy_sr_held.h); --pin adds
// files to the presented set. data-dir with variant dev (gt.tsv "variant dev"): fine in REPORT mode (no gate selected), and
// with a gate selected unless it IS sealed gate data -> exit 3. data-dir with variant registered: needs the full release of
// the selected gate (designation, review record, the whole pin set incl. this executable, tree sha256, one-run marker);
// exit 0 = released, else 4 (HELD). Anything else (no gt.tsv, unknown variant): exit 5.
// The release log is printed on success too (the node test keeps it; audit F-6).

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "curvy_sr_held.h"

int main(int argc, char** argv) {
  namespace ch = slam_refact::curvy_gate_held;
  std::string kind, dir;
  std::vector<std::string> pins;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    if (k == "--kind") kind = argv[i + 1];
    if (k == "--data-dir") dir = argv[i + 1];
    if (k == "--pin") pins.push_back(argv[i + 1]);
  }
  std::string gt;
  if (kind.empty() || dir.empty() || !slam_refact::sr_held::ReadFile(dir + "/gt.tsv", &gt)) {
    std::printf("ERROR: --kind, --data-dir and a gt.tsv are needed\n");
    return 5;
  }
  const size_t v = gt.find("variant\t");
  const std::string variant = v == std::string::npos ? "" : gt.substr(v + 8, gt.find('\n', v) - v - 8);
  const ch::Selection sel = ch::SelectFromEnv();
  std::vector<std::string> log;
  if (variant == "dev") {
    const std::string why = ch::DevGuard(sel, dir, &log);
    ch::Print(log);
    if (!why.empty()) {
      std::printf("%s\n", why.c_str());
      return 3;
    }
    std::printf("DEV\n");
    return 0;
  }
  if (variant != "registered") {
    std::printf("ERROR: unknown variant '%s'\n", variant.c_str());
    return 5;
  }
  const std::string why = ch::GateGuard(sel, kind, dir, &log, pins);
  ch::Print(log);
  if (!why.empty()) {
    std::printf("%s\n", why.c_str());
    return 4;
  }
  std::printf("RELEASED\n");
  return 0;
}
