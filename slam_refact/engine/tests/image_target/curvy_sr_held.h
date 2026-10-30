// Curved target gate front of the shared HELD device (tests/support/sr_held.h, docs/design/sr-held-design.md). It replaces the
// per-gate device curvy_held.h for the curvy tests (the old header stays only because its_held.h still builds on it).
//
// NOTHING about the gate is compiled in: the gate id, the version and the preregistration directory come from the environment
// and have NO default, so a binary can never be tied to a gate by accident (the v3 constants "curvy-it-gate" / "3" /
// build/.../preregistration used to be hard-coded in four places):
//   SLAM_REFACT_CURVY_GATE_ID       e.g. curvy-it-gate
//   SLAM_REFACT_CURVY_GATE_VERSION  decimal N of <gate_id>.v<N>
//   SLAM_REFACT_CURVY_PREREG_DIR    directory of the manifest, review, designation file and markers
//   SLAM_REFACT_CURVY_DESIGNATION   sha256 of the coordinator's designation file (registered data only)
//   SLAM_REFACT_CURVY_PINS          ':'-separated paths of ALL pinned files (the manifest held.pins is one map for every kind
//                                   and sr_held requires the presented set to equal it, so every process presents the whole
//                                   set; this process's own executable is always added from /proc/self/exe)
// Three states (Select):
//   none given          REPORT mode: dev data runs unguarded (nothing to compare with), registered data is HELD
//   all three named     the gate is named: dev data is refused if it is sealed gate data (sr_held::RefuseSealed), registered
//                       data needs the full release (sr_held::HeldGate: designation, review, pins, tree sha256, O_EXCL marker)
//   anything else       HELD (a partial or malformed selection never falls back to REPORT; a designation without a gate
//                       is refused, so the old "designation only" invocation of the v3 runner cannot half-work)
// It is an accident guard, not an authentication (protocol 12.6). The manifest must register held.seed=false (the coordinator
// seed is read by the rendering tools, not by this device).
// Pure functions of their arguments plus the files they name, so that curvy_sr_held_test.cc can feed synthetic files.

#ifndef SLAM_REFACT_TESTS_IMAGE_TARGET_CURVY_SR_HELD_H_
#define SLAM_REFACT_TESTS_IMAGE_TARGET_CURVY_SR_HELD_H_

#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include "../support/sr_held.h"

namespace slam_refact::curvy_gate_held {

constexpr char kEnvGateId[] = "SLAM_REFACT_CURVY_GATE_ID";
constexpr char kEnvGateVersion[] = "SLAM_REFACT_CURVY_GATE_VERSION";
constexpr char kEnvPreregDir[] = "SLAM_REFACT_CURVY_PREREG_DIR";
constexpr char kEnvDesignation[] = "SLAM_REFACT_CURVY_DESIGNATION";
constexpr char kEnvPins[] = "SLAM_REFACT_CURVY_PINS";

struct Selection {
  enum class State { kReport, kNamed, kRefused };
  State state = State::kReport;
  std::string gate_id, prereg_dir, designation, why;  // why: kRefused only
  int version = 0;
  std::vector<std::string> pins;  // SLAM_REFACT_CURVY_PINS, split
};

inline std::string Env(const char* name) {
  const char* v = std::getenv(name);
  return v == nullptr ? std::string() : std::string(v);
}

// Pure: empty strings mean "not given".
inline Selection Select(const std::string& gate_id, const std::string& version, const std::string& prereg_dir,
                        const std::string& designation, const std::string& pins) {
  Selection s;
  const bool any = !gate_id.empty() || !version.empty() || !prereg_dir.empty() || !designation.empty() || !pins.empty();
  if (!any) return s;  // REPORT mode
  s.state = Selection::State::kRefused;
  if (gate_id.empty() || version.empty() || prereg_dir.empty()) {
    s.why = std::string("HELD: the gate selection is incomplete: ") + kEnvGateId + ", " + kEnvGateVersion + " and " +
            kEnvPreregDir + " must all be set (or none of them, for REPORT mode)";
    return s;
  }
  if (!sr_held::IsId(gate_id)) {
    s.why = std::string("HELD: ") + kEnvGateId + " is not [A-Za-z0-9][A-Za-z0-9_-]{0,63}";
    return s;
  }
  if (!sr_held::IsDecimal(version) || version.size() > 6 || std::atoi(version.c_str()) < 1) {
    s.why = std::string("HELD: ") + kEnvGateVersion + " is not a decimal number in 1..999999";
    return s;
  }
  s.gate_id = gate_id;
  s.version = std::atoi(version.c_str());
  s.prereg_dir = prereg_dir;
  s.designation = designation;
  for (size_t at = 0; at <= pins.size() && !pins.empty();) {
    const size_t colon = pins.find(':', at);
    const std::string one = pins.substr(at, colon == std::string::npos ? std::string::npos : colon - at);
    if (one.empty()) {
      s.why = std::string("HELD: ") + kEnvPins + " has an empty entry";
      return s;
    }
    s.pins.push_back(one);
    if (colon == std::string::npos) break;
    at = colon + 1;
  }
  s.state = Selection::State::kNamed;
  return s;
}

inline Selection SelectFromEnv() {
  return Select(Env(kEnvGateId), Env(kEnvGateVersion), Env(kEnvPreregDir), Env(kEnvDesignation), Env(kEnvPins));
}

inline std::string Canonical(const std::string& path) {
  char* r = ::realpath(path.c_str(), nullptr);
  const std::string out = r == nullptr ? path : std::string(r);
  std::free(r);
  return out;
}

// The pins this process presents: its own executable, the selected ones, and `extra` (command line), each file once.
inline std::vector<std::string> PinsOf(const Selection& s, const std::vector<std::string>& extra = {}) {
  std::vector<std::string> out;
  std::vector<std::string> seen;
  auto add = [&](const std::string& p) {
    const std::string c = Canonical(p);
    for (const std::string& q : seen) {
      if (q == c) return;
    }
    seen.push_back(c);
    out.push_back(p);
  };
  add(sr_held::SelfExe());
  for (const std::string& p : s.pins) add(p);
  for (const std::string& p : extra) add(p);
  return out;
}

inline std::string Refusal(const sr_held::Outcome& o) { return "HELD [" + o.code + "]: " + o.why; }

// "" when the engine may run on `data_dir`, which must hold DEV data: REPORT mode runs it unguarded; a named gate refuses it
// if it is sealed gate data (decided by the content of the directory, not by its name).
inline std::string DevGuard(const Selection& s, const std::string& data_dir, std::vector<std::string>* log = nullptr) {
  if (s.state == Selection::State::kRefused) return s.why;
  if (s.state == Selection::State::kReport) {
    if (log != nullptr) log->push_back("[CURVY-HELD] REPORT mode: no gate is selected, so the dev data " + data_dir + " is not compared with sealed gate data");
    return "";
  }
  sr_held::Request rq;
  rq.prereg_dir = s.prereg_dir;
  rq.gate_id = s.gate_id;
  rq.version = s.version;
  rq.data_dir = data_dir;
  const sr_held::Outcome o = sr_held::RefuseSealed(rq);
  return o.ok ? "" : "HELD: " + data_dir + " is used as dev data: " + Refusal(o);
}

// "" when the first real run of (`kind`, `data_dir`) is released. One process = one run: the marker of a kind is created by the
// first call, and a repeated call for the same kind and directory in the same process reuses that claim (CurvyCrop then
// CurvySynth in one process); the same kind with another directory is refused.
inline std::string GateGuard(const Selection& s, const std::string& kind, const std::string& data_dir,
                             std::vector<std::string>* log = nullptr, const std::vector<std::string>& extra_pins = {}) {
  struct Claim {
    std::string data_dir, why;
  };
  static std::map<std::string, Claim> claims;
  if (s.state == Selection::State::kRefused) return s.why;
  if (s.state == Selection::State::kReport) {
    return std::string("HELD: registered gate data needs a named gate (") + kEnvGateId + ", " + kEnvGateVersion + ", " +
           kEnvPreregDir + ") and the coordinator designation (" + kEnvDesignation + ")";
  }
  const auto it = claims.find(kind);
  if (it != claims.end()) {
    if (!it->second.why.empty()) return it->second.why;
    return it->second.data_dir == data_dir ? "" : "HELD: kind " + kind + " was already released for " + it->second.data_dir + ", not " + data_dir;
  }
  sr_held::Request rq;
  rq.prereg_dir = s.prereg_dir;
  rq.gate_id = s.gate_id;
  rq.version = s.version;
  rq.kind = kind;
  rq.env_value = s.designation;
  rq.pins = PinsOf(s, extra_pins);
  rq.data_dir = data_dir;
  rq.want_seed = false;
  const sr_held::Outcome o = sr_held::HeldGate(rq);
  if (log != nullptr) log->insert(log->end(), o.log.begin(), o.log.end());
  Claim c;
  c.data_dir = data_dir;
  c.why = o.ok ? "" : Refusal(o);
  claims[kind] = c;
  return c.why;
}

inline void Print(const std::vector<std::string>& log) {
  for (const std::string& l : log) std::printf("%s\n", l.c_str());
}

}  // namespace slam_refact::curvy_gate_held

#endif  // SLAM_REFACT_TESTS_IMAGE_TARGET_CURVY_SR_HELD_H_
