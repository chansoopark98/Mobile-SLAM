// HELD glue of the sky gate (US-020; sky-us020.v2 and later). The device itself is the shared sr_held
// (engine/tests/support/sr_held.h, docs/design/sr-held-design.md); this header only
//   - reads the registration that the run is for from explicit inputs (environment): the preregistration directory, the gate
//     id, the version, the kind and the designation sha256. Nothing here names a gate: there is no default id, version,
//     manifest, sealed-list or marker file name in this file or in tests/sky/sky_gate_test.cc (c2-execution-plan-r4b B8);
//   - calls sr_held::HeldGate (every check, then the one-run marker) and writes the run record, which carries the sha256 of
//     the gate binary (sky-us020.v1 review F-10);
//   - decides "is this recorded set a sealed gate set" for report mode and for the replay tool (bench/sky_replay_main.cc,
//     IsSealedGateSet below) by scanning EVERY registration in the preregistration directory (sr_held manifests:
//     held.inputs / held.sealed_file_sha256; and the older <id>.v<N>.sealed.json lists of v1, which stay refused), never by
//     a gate name.
// A safety device against mistakes, not an authentication (protocol 12.6): anyone who can write the registration
// directory and the environment can satisfy every check. The manifest is never written by this code and no check needs it
// to be edited after the review (sky-us020.v1 review F-1): records made after a run go to a new file.
#ifndef SLAM_REFACT_SKY_BENCH_SKY_HELD_H_
#define SLAM_REFACT_SKY_BENCH_SKY_HELD_H_

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <functional>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "../../../tests/support/sr_held.h"
#include "sky/sky_model.h"

namespace slam_refact::sky {

inline std::string HeldReadAll(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

inline std::string HeldSha(const std::string& bytes) {
  return Sha256Hex(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
}

// The registration a gate-mode run is for. Every field is required; there is no default for any of them.
struct GateRequest {
  std::string prereg_dir;  // directory with <gate_id>.v<N>.manifest.json, the review, the designation file and the markers
  std::string gate_id;
  int version = 0;
  std::string kind;        // the one-run kind (manifest held.kinds)
  std::string reviewer;    // sha256 of the designation file (the value the coordinator hands over)
};

// Returns "" or the reason (HELD). `env` returns "" for an unset variable.
inline std::string ReadGateRequest(const std::function<std::string(const char*)>& env, GateRequest* g) {
  *g = GateRequest();
  std::string version, missing;
  const std::pair<const char*, std::string*> fields[] = {{"SR_SKY_PREREG_DIR", &g->prereg_dir}, {"SR_SKY_GATE_ID", &g->gate_id},
                                                         {"SR_SKY_GATE_VERSION", &version},     {"SR_SKY_GATE_KIND", &g->kind},
                                                         {"SR_SKY_REVIEWER", &g->reviewer}};
  for (const auto& f : fields) {
    *f.second = env(f.first);
    if (f.second->empty()) missing += std::string(missing.empty() ? "" : ", ") + f.first;
  }
  if (!missing.empty()) return "HELD: gate mode needs " + missing + " (no default: the registration is named by the caller)";
  if (version.size() > 6 || !std::all_of(version.begin(), version.end(), [](char c) { return c >= '0' && c <= '9'; }) || version[0] == '0') {
    return "HELD: SR_SKY_GATE_VERSION is not a positive decimal integer";
  }
  g->version = std::stoi(version);
  return "";
}

// The recorded sets of the gate data root: its immediate subdirectories, sorted by name; each must hold the replay.txt of a
// recorded set. The registered tree sha256 of the root (manifest held.inputs[kind]) binds their content; this only reads
// the structure, before the marker is taken (a root that is not a root of recorded sets must not consume the run).
inline std::string ListGateSets(const std::string& ref_root, std::vector<std::string>* sets) {
  namespace fs = std::filesystem;
  sets->clear();
  std::error_code ec;
  for (const auto& e : fs::directory_iterator(ref_root, ec)) {
    if (!e.is_directory()) continue;
    const std::string name = e.path().filename().string();
    if (!fs::is_regular_file(e.path() / "replay.txt", ec)) return "HELD: " + e.path().string() + " has no replay.txt: it is not a recorded set";
    sets->push_back(name);
  }
  if (ec) return "HELD: gate data root " + ref_root + " cannot be listed";
  if (sets->empty()) return "HELD: gate data root " + ref_root + " holds no recorded set";
  std::sort(sets->begin(), sets->end());
  return "";
}

// Every check of the registration (sr_held), then the one-run marker. Returns "" when released, else the HELD reason.
// `log` receives the lines of the device, the gate binary (path and sha256) and the sets. Nothing is measured and no report
// directory exists before this returns "".
inline std::string Release(const GateRequest& g, const std::string& ref_root, const std::vector<std::string>& pins,
                           std::vector<std::string>* log, std::vector<std::string>* sets) {
  const std::string self = sr_held::SelfExe();
  std::string self_sha;
  log->push_back("[SKY-HELD] gate binary " + (self.empty() ? std::string("<unreadable>") : self) + " sha256=" +
                 (sr_held::Sha256File(self, &self_sha) ? self_sha : std::string("<unreadable>")));
  const std::string listed = ListGateSets(ref_root, sets);
  if (!listed.empty()) return listed;
  sr_held::Request rq;
  rq.prereg_dir = g.prereg_dir;
  rq.gate_id = g.gate_id;
  rq.version = g.version;
  rq.kind = g.kind;
  rq.env_value = g.reviewer;
  rq.pins = pins;
  rq.data_dir = ref_root;
  rq.want_seed = false;
  const sr_held::Outcome o = sr_held::HeldGate(rq);
  for (const std::string& l : o.log) log->push_back(l);
  if (!o.ok) return "HELD [" + o.code + "]: " + o.why;
  for (const std::string& name : *sets) {
    std::string replay_sha;
    log->push_back("[SKY-HELD] set " + name + " replay.txt sha256=" +
                   (sr_held::Sha256File(ref_root + "/" + name + "/replay.txt", &replay_sha) ? replay_sha : std::string("<unreadable>")));
  }
  log->push_back("[SKY-HELD] sets=" + std::to_string(sets->size()) + " (the tree sha256 of the root is the registered one)");
  return "";
}

// <report dir>/held-run-record.txt: what the one-run marker (a fixed-form file of sr_held) does not carry, notably the
// sha256 of the gate binary. Written after the marker exists, from values read back from disk.
inline void WriteRunRecord(const std::string& report_dir, const GateRequest& g, const std::vector<std::string>& log) {
  sr_held::Request rq;
  rq.gate_id = g.gate_id;
  rq.version = g.version;
  rq.kind = g.kind;
  const std::string marker = g.prereg_dir + "/" + sr_held::MarkerName(rq);
  std::string marker_sha, self_sha;
  const std::string self = sr_held::SelfExe();
  std::ofstream out(report_dir + "/held-run-record.txt");
  out << "gate " << g.gate_id << ".v" << g.version << " kind " << g.kind << "\n";
  out << "designation_sha256 " << g.reviewer << "\n";
  out << "marker " << marker << " sha256 " << (sr_held::Sha256File(marker, &marker_sha) ? marker_sha : std::string("<unreadable>")) << "\n";
  out << "gate_binary " << self << " sha256 " << (sr_held::Sha256File(self, &self_sha) ? self_sha : std::string("<unreadable>")) << "\n";
  out << "pid " << ::getpid() << "\n";
  for (const std::string& l : log) out << l << "\n";
}

namespace internal {

inline std::vector<std::string> FilesWithSuffix(const std::string& dir, const std::string& suffix) {
  namespace fs = std::filesystem;
  std::vector<std::string> out;
  std::error_code ec;
  for (const auto& e : fs::directory_iterator(dir, ec)) {
    const std::string name = e.path().filename().string();
    if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) out.push_back(name);
  }
  std::sort(out.begin(), out.end());
  return out;
}

// One preregistration directory. Sets *found when it holds at least one registration; returns true when the set is sealed
// (or when a registration cannot be read: fail closed).
inline bool ScanSealed(const std::string& prereg_dir, const std::string& repo_root, const std::string& set_name,
                       const std::string& set_dir, bool* found, std::string* why) {
  // (a) the lists of the older registrations: "<id>.v<N>.sealed.json" with names, replay_sha256, sums_sha256 and a root
  for (const std::string& file : FilesWithSuffix(prereg_dir, ".sealed.json")) {
    *found = true;
    const std::string text = HeldReadAll(prereg_dir + "/" + file);
    if (text.empty()) {
      *why = file + " unreadable: cannot tell whether this is a sealed set (fail closed)";
      return true;
    }
    std::set<std::string> names, replays, sums;
    auto collect = [&](const char* key, std::set<std::string>* dst, const char* value_re) {
      const std::regex re(std::string("\"") + key + "\"\\s*:\\s*\"(" + value_re + ")\"");
      for (std::sregex_iterator it(text.begin(), text.end(), re), end; it != end; ++it) dst->insert((*it)[1]);
    };
    collect("name", &names, "[^\"]+");
    collect("replay_sha256", &replays, "[0-9a-f]{64}");
    collect("sums_sha256", &sums, "[0-9a-f]{64}");
    if (names.count(set_name)) {
      *why = "the set name " + set_name + " is a sealed gate set (" + file + ")";
      return true;
    }
    const std::string replay = HeldReadAll(set_dir + "/replay.txt");
    if (!replay.empty() && replays.count(HeldSha(replay))) {
      *why = "replay.txt of " + set_dir + " has the sha256 of a sealed gate set (" + file + ")";
      return true;
    }
    const std::string sum = HeldReadAll(set_dir + "/SHA256SUMS");
    if (!sum.empty() && sums.count(HeldSha(sum))) {
      *why = "SHA256SUMS of " + set_dir + " has the sha256 of a sealed gate set (" + file + ")";
      return true;
    }
    const std::regex root_re("\"root\"\\s*:\\s*\"([^\"]+)\"");
    std::smatch m;
    if (std::regex_search(text, m, root_re)) {
      namespace fs = std::filesystem;
      std::error_code ec;
      fs::path root(m[1].str());
      if (root.is_relative()) root = fs::path(repo_root) / root;
      const fs::path r = fs::weakly_canonical(root, ec);
      const fs::path d = fs::weakly_canonical(fs::path(set_dir), ec);
      const auto rel = d.lexically_relative(r);
      if (!r.empty() && !rel.empty() && *rel.begin() != "..") {
        *why = set_dir + " is below the sealed root " + r.string() + " (" + file + ")";
        return true;
      }
    }
  }
  // (b) the sr_held manifests: held.inputs (tree sha256) and held.sealed_file_sha256 (replay.txt, SHA256SUMS), by content
  for (const std::string& file : FilesWithSuffix(prereg_dir, ".manifest.json")) {
    std::string text, err;
    sr_held::Json m;
    if (!sr_held::ReadFile(prereg_dir + "/" + file, &text) || !sr_held::ParseJson(text, &m, &err) || m.type != sr_held::Json::Type::kObject) {
      *found = true;
      *why = file + " cannot be read, so the set cannot be told from registered data (fail closed)";
      return true;
    }
    const sr_held::Json* held = m.Get("held");
    const sr_held::Json* id = m.Get("gate_id");
    const sr_held::Json* ver = m.Get("version");
    if (held == nullptr || held->type != sr_held::Json::Type::kObject) continue;  // not an sr_held manifest
    *found = true;
    sr_held::Request rq;
    rq.prereg_dir = prereg_dir;
    rq.gate_id = id != nullptr && id->type == sr_held::Json::Type::kString ? id->text : std::string();
    rq.version = ver != nullptr && ver->type == sr_held::Json::Type::kNumber && sr_held::IsDecimal(ver->text) && ver->text.size() < 7
                     ? std::stoi(ver->text)
                     : 0;
    rq.data_dir = set_dir;
    const sr_held::Outcome s = sr_held::RefuseSealed(rq);
    if (!s.ok) {
      *why = s.why + " (" + file + ")";
      return true;
    }
  }
  return false;
}

}  // namespace internal

// Report mode and the replay tool must never measure a sealed gate set. Decided by what the set is (the tree sha256 or a file
// sha256 a registration holds, and for the older lists its name, the sha256 of its replay.txt / SHA256SUMS, a path below the
// sealed root), not by the root path the caller names. Consulted: `prereg_dir` and, when the environment names one,
// SR_SKY_PREREG_DIR. When no registration can be found at all, or one cannot be read, the answer is "sealed" (fail closed).
inline bool IsSealedGateSet(const std::string& prereg_dir, const std::string& repo_root, const std::string& set_name,
                            const std::string& set_dir, std::string* why) {
  std::vector<std::string> dirs = {prereg_dir};
  if (const char* e = std::getenv("SR_SKY_PREREG_DIR"); e != nullptr && *e != '\0' && std::string(e) != prereg_dir) dirs.push_back(e);
  bool found = false;
  for (const std::string& d : dirs) {
    if (internal::ScanSealed(d, repo_root, set_name, set_dir, &found, why)) return true;
  }
  if (!found) {
    *why = "no registration found in " + prereg_dir + ": cannot tell whether " + set_dir + " is a sealed gate set (fail closed)";
    return true;
  }
  return false;
}

}  // namespace slam_refact::sky

#endif  // SLAM_REFACT_SKY_BENCH_SKY_HELD_H_
