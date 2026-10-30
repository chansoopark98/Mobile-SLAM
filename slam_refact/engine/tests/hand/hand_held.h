// HELD glue of the hand gate (US-021; hand-us021.v2 and later). The device itself is the shared sr_held
// (tests/support/sr_held.h, docs/design/sr-held-design.md); this header only
//   - reads the registration that the run is for from explicit inputs (environment): the preregistration directory, the gate
//     id, the version, the kind and the designation sha256. Nothing here names a gate: there is no default id, version,
//     manifest, sealed-list or marker file name in this file or in hand_gate_test.cc (c2-execution-plan-r4b B8);
//   - calls sr_held::HeldGate (every check, then the one-run marker) and writes the run record;
//   - decides "is this data sealed gate data" for report mode by scanning EVERY registration in the preregistration
//     directory (sr_held manifests: held.inputs / held.sealed_file_sha256; and the older <id>.v<N>.sealed.json lists of
//     v1, which stay refused), never by a gate name.
// A safety device against mistakes, not an authentication (protocol 12.6): anyone who can write the registration
// directory and the environment can satisfy every check.
#ifndef SLAM_REFACT_TESTS_HAND_HAND_HELD_H_
#define SLAM_REFACT_TESTS_HAND_HAND_HELD_H_

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <functional>
#include <regex>
#include <set>
#include <string>
#include <vector>

#include "../support/sr_held.h"

namespace slam_refact::hand {

using Log = std::vector<std::string>;

inline std::string HeldReadAll(const std::string& path) {
  std::string out;
  return sr_held::ReadFile(path, &out) ? out : std::string();
}

inline std::string HeldSha(const std::string& bytes) { return testutil::Sha256Hex(bytes); }

// The data sets below the gate data root (structure of the data, not a gate name).
inline const std::vector<std::string>& HandDataSets() {
  static const std::vector<std::string> kSets = {"g1", "g2", "g3", "g3-tensors-1"};
  return kSets;
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
  const std::pair<const char*, std::string*> fields[] = {{"SR_HAND_PREREG_DIR", &g->prereg_dir}, {"SR_HAND_GATE_ID", &g->gate_id},
                                                         {"SR_HAND_GATE_VERSION", &version},     {"SR_HAND_GATE_KIND", &g->kind},
                                                         {"SR_HAND_REVIEWER", &g->reviewer}};
  for (const auto& f : fields) {
    *f.second = env(f.first);
    if (f.second->empty()) missing += std::string(missing.empty() ? "" : ", ") + f.first;
  }
  if (!missing.empty()) return "HELD: gate mode needs " + missing + " (no default: the registration is named by the caller)";
  if (version.size() > 6 || !std::all_of(version.begin(), version.end(), [](char c) { return c >= '0' && c <= '9'; }) || version[0] == '0') {
    return "HELD: SR_HAND_GATE_VERSION is not a positive decimal integer";
  }
  g->version = std::stoi(version);
  return "";
}

// The data root must hold the four data sets as directories. Checked before the marker is taken: a wrong argument must
// not consume the run.
inline std::string CheckDataLayout(const std::string& root) {
  namespace fs = std::filesystem;
  for (const std::string& name : HandDataSets()) {
    std::error_code ec;
    if (!fs::is_directory(fs::path(root) / name, ec)) return "HELD: data set " + name + " is not a directory below " + root;
  }
  return "";
}

// Every check of the registration (sr_held), then the one-run marker. Returns "" when released, else the HELD reason.
// `log` receives the lines of the device and the gate binary (path and sha256). Nothing is measured and no report
// directory exists before this returns "".
inline std::string Release(const GateRequest& g, const std::string& data_root, const std::vector<std::string>& pins, Log* log) {
  const std::string self = sr_held::SelfExe();
  std::string self_sha;
  log->push_back("[HAND-HELD] gate binary " + (self.empty() ? std::string("<unreadable>") : self) + " sha256=" +
                 (sr_held::Sha256File(self, &self_sha) ? self_sha : std::string("<unreadable>")));
  sr_held::Request rq;
  rq.prereg_dir = g.prereg_dir;
  rq.gate_id = g.gate_id;
  rq.version = g.version;
  rq.kind = g.kind;
  rq.env_value = g.reviewer;
  rq.pins = pins;
  rq.data_dir = data_root;
  rq.want_seed = false;
  const sr_held::Outcome o = sr_held::HeldGate(rq);
  for (const std::string& l : o.log) log->push_back(l);
  if (!o.ok) return "HELD [" + o.code + "]: " + o.why;
  return "";
}

// <report dir>/held-run-record.txt: what the one-run marker (a fixed-form file of sr_held) does not carry. Written after the
// marker exists, from values read back from disk.
inline void WriteRunRecord(const std::string& report_dir, const GateRequest& g, const Log& log) {
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

// sha256 of the checksum list that tools/hand/seal_gate_set.sh writes for a directory: "<sha256>  <name>" for every
// regular file of the directory (not recursive, SHA256SUMS excluded), names sorted bytewise.
inline std::string DirSumsSha(const std::string& dir) {
  namespace fs = std::filesystem;
  std::vector<std::string> names;
  std::error_code ec;
  for (const auto& e : fs::directory_iterator(dir, ec)) {
    if (e.is_regular_file() && e.path().filename() != "SHA256SUMS") names.push_back(e.path().filename().string());
  }
  if (names.empty()) return "";
  std::sort(names.begin(), names.end());
  std::string list;
  for (const auto& n : names) list += HeldSha(HeldReadAll(dir + "/" + n)) + "  " + n + "\n";
  return HeldSha(list);
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

// One preregistration directory. Sets *found when it holds at least one registration; returns true when `dir` is sealed
// data of one of them (or when a registration cannot be read: fail closed).
inline bool ScanSealed(const std::string& prereg_dir, const std::string& repo_root, const std::string& dir, bool* found, std::string* why) {
  // (a) the lists of the older registrations: "<id>.v<N>.sealed.json" with sums_sha256 / root
  for (const std::string& name : FilesWithSuffix(prereg_dir, ".sealed.json")) {
    *found = true;
    const std::string text = HeldReadAll(prereg_dir + "/" + name);
    if (text.empty()) {
      *why = name + " unreadable: cannot tell whether this is sealed data (fail closed)";
      return true;
    }
    std::set<std::string> sums;
    const std::regex re("\"sums_sha256\"\\s*:\\s*\"([0-9a-f]{64})\"");
    for (std::sregex_iterator it(text.begin(), text.end(), re), end; it != end; ++it) sums.insert((*it)[1]);
    const std::string listed = DirSumsSha(dir);
    if (!listed.empty() && sums.count(listed)) {
      *why = "the files of " + dir + " have the checksum list of a sealed gate set (" + name + ")";
      return true;
    }
    const std::string sum_file = HeldReadAll(dir + "/SHA256SUMS");
    if (!sum_file.empty() && sums.count(HeldSha(sum_file))) {
      *why = "SHA256SUMS of " + dir + " has the sha256 of a sealed gate set (" + name + ")";
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
      const fs::path d = fs::weakly_canonical(fs::path(dir), ec);
      const auto rel = d.lexically_relative(r);
      if (!r.empty() && !rel.empty() && *rel.begin() != "..") {
        *why = dir + " is below the sealed root " + r.string() + " (" + name + ")";
        return true;
      }
    }
  }
  // (b) the sr_held manifests: held.inputs (tree sha256) and held.sealed_file_sha256, by content
  for (const std::string& name : FilesWithSuffix(prereg_dir, ".manifest.json")) {
    std::string text, err;
    sr_held::Json m;
    if (!sr_held::ReadFile(prereg_dir + "/" + name, &text) || !sr_held::ParseJson(text, &m, &err) || m.type != sr_held::Json::Type::kObject) {
      *found = true;
      *why = name + " cannot be read, so the data cannot be told from registered data (fail closed)";
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
    rq.data_dir = dir;
    const sr_held::Outcome s = sr_held::RefuseSealed(rq);
    if (!s.ok) {
      *why = s.why + " (" + name + ")";
      return true;
    }
  }
  return false;
}

}  // namespace internal

// Report mode must never measure sealed gate data: decided by what the directory contains (registered tree sha256, a file
// with a registered sha256, the checksum list of an older registration) or by a location below an older sealed root, not by
// the path or a name the caller gives. Consulted: `prereg_dir` and, when the environment names one, SR_HAND_PREREG_DIR. When
// no registration can be found at all, or one cannot be read, the answer is "sealed" (fail closed).
inline bool IsSealedHandData(const std::string& prereg_dir, const std::string& repo_root, const std::string& dir, std::string* why) {
  std::vector<std::string> dirs = {prereg_dir};
  if (const char* e = std::getenv("SR_HAND_PREREG_DIR"); e != nullptr && *e != '\0' && std::string(e) != prereg_dir) dirs.push_back(e);
  bool found = false;
  for (const std::string& d : dirs) {
    if (internal::ScanSealed(d, repo_root, dir, &found, why)) return true;
  }
  if (!found) {
    *why = "no registration found in " + prereg_dir + ": cannot tell whether " + dir + " is sealed data (fail closed)";
    return true;
  }
  return false;
}

}  // namespace slam_refact::hand

#endif  // SLAM_REFACT_TESTS_HAND_HAND_HELD_H_
