// HELD device of the curved target gate (build/slam-refact-full-20261019/preregistration/curvy-it-gate.v3.md section 6),
// after preregistration-protocol.md sections 12.5 and 12.6 (the OR-4 / orient devices). The first real run of the gate is
// refused by CODE unless every condition holds, and it can happen once per data kind:
//   (a) env SLAM_REFACT_CURVY_DESIGNATION = sha256 of the coordinator's designation file, and that is the value of manifest
//       review.designation_file_sha256 and the actual sha256 of the file;
//   (b) manifest review.at is a recorded string later than created_at;
//   (c) the designation file names the gate and version, first_real_run_permitted is true, and its gate_file_sha256,
//       manifest_core_sha256 and review_file_sha256 equal the actual files (the manifest "core" = the manifest text with the
//       review object and the first_real_run_at value blanked) and the manifest's review.sha256;
//   (d) the gate file sha256 equals manifest held_run.gate_file_sha256 and manifest sha256;
//   (e) every pinned executable / WASM file handed over (and /proc/self/exe) has the sha256 of manifest held_pins[<basename>];
//   (f) the tree sha256 of the data directory equals manifest inputs_tree_sha256[<kind>];
//   (g) manifest first_real_run_at is still null, and the one-run marker <gate id>.first-run.<kind>.marker is created with
//       O_EXCL (a start that created it is a run; the code never removes it).
// It is an accident guard, not an authentication: anyone who can write the manifest, the designation file and the marker
// directory can satisfy it (protocol 12.6). Development data (variant dev) never needs it, and registered data is refused on
// the dev path (IsRegisteredTree).
// Pure functions of their arguments, so that curvy_held_test.cc can feed synthetic files and check that each bypass is
// rejected for the expected reason.

#ifndef SLAM_REFACT_TESTS_IMAGE_TARGET_CURVY_HELD_H_
#define SLAM_REFACT_TESTS_IMAGE_TARGET_CURVY_HELD_H_

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "../common/sha256_hex.h"

namespace slam_refact::curvy_held {

inline bool ReadFile(const std::string& path, std::string* out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  std::stringstream ss;
  ss << in.rdbuf();
  *out = ss.str();
  return true;
}

inline std::string Sha256File(const std::string& path, bool* ok) {
  std::string data;
  *ok = ReadFile(path, &data);
  return *ok ? testutil::Sha256Hex(data) : std::string();
}

// sha256 over the sorted lines "<relative path>\t<file sha256>\n" of every regular file below `dir`.
inline void ListFiles(const std::string& dir, const std::string& rel, std::vector<std::string>* out) {
  DIR* d = ::opendir((dir + (rel.empty() ? "" : "/" + rel)).c_str());
  if (d == nullptr) return;
  while (const dirent* e = ::readdir(d)) {
    const std::string name = e->d_name;
    if (name == "." || name == "..") continue;
    const std::string r = rel.empty() ? name : rel + "/" + name;
    struct stat st;
    if (::stat((dir + "/" + r).c_str(), &st) != 0) continue;
    if (S_ISDIR(st.st_mode)) {
      ListFiles(dir, r, out);
    } else if (S_ISREG(st.st_mode)) {
      out->push_back(r);
    }
  }
  ::closedir(d);
}

inline std::string TreeSha(const std::string& dir) {
  std::vector<std::string> files;
  ListFiles(dir, "", &files);
  std::sort(files.begin(), files.end());
  std::string lines;
  for (const std::string& f : files) {
    bool ok = false;
    lines += f + "\t" + Sha256File(dir + "/" + f, &ok) + "\n";
  }
  return files.empty() ? std::string() : testutil::Sha256Hex(lines);
}

// ---- minimal JSON access (the manifest and the designation file are written by tools with indent 1) ----
inline size_t FindKey(const std::string& s, const std::string& key, size_t from = 0) {
  const std::string pat = "\"" + key + "\"";
  for (size_t at = s.find(pat, from); at != std::string::npos; at = s.find(pat, at + 1)) {
    size_t e = at + pat.size();
    while (e < s.size() && std::isspace(static_cast<unsigned char>(s[e]))) ++e;
    if (e < s.size() && s[e] == ':') return at;
  }
  return std::string::npos;
}

inline std::string JsonObject(const std::string& s, const std::string& key) {
  const size_t at = FindKey(s, key);
  if (at == std::string::npos) return "";
  size_t e = s.find(':', at) + 1;
  while (e < s.size() && std::isspace(static_cast<unsigned char>(s[e]))) ++e;
  if (e >= s.size() || s[e] != '{') return "";
  int depth = 0;
  for (size_t i = e; i < s.size(); ++i) {
    if (s[i] == '{') ++depth;
    if (s[i] == '}' && --depth == 0) return s.substr(e, i - e + 1);
  }
  return "";
}

// String value (without quotes) or the raw token (true / false / number); "" for null and for a missing key.
inline std::string FindValue(const std::string& s, const std::string& key) {
  const size_t at = FindKey(s, key);
  if (at == std::string::npos) return "";
  size_t e = s.find(':', at) + 1;
  while (e < s.size() && std::isspace(static_cast<unsigned char>(s[e]))) ++e;
  if (e >= s.size()) return "";
  if (s[e] == '"') {
    const size_t q = s.find('"', e + 1);
    return q == std::string::npos ? "" : s.substr(e + 1, q - e - 1);
  }
  size_t q = e;
  while (q < s.size() && s[q] != ',' && s[q] != '}' && s[q] != '\n' && !std::isspace(static_cast<unsigned char>(s[q]))) ++q;
  const std::string tok = s.substr(e, q - e);
  return tok == "null" ? "" : tok;
}

// The manifest with the review object and the first_real_run_at value blanked (the part that must not change between the
// registration and the designation).
inline std::string ManifestCoreText(std::string m) {
  const size_t r = FindKey(m, "review");
  if (r != std::string::npos) {
    size_t e = m.find(':', r) + 1;
    while (e < m.size() && std::isspace(static_cast<unsigned char>(m[e]))) ++e;
    if (e < m.size() && m[e] == '{') {
      int depth = 0;
      for (size_t i = e; i < m.size(); ++i) {
        if (m[i] == '{') ++depth;
        if (m[i] == '}' && --depth == 0) {
          m.replace(e, i - e + 1, "{}");
          break;
        }
      }
    }
  }
  const size_t f = FindKey(m, "first_real_run_at");
  if (f != std::string::npos) {
    size_t e = m.find(':', f) + 1;
    while (e < m.size() && std::isspace(static_cast<unsigned char>(m[e]))) ++e;
    size_t q = e;
    if (q < m.size() && m[q] == '"') {
      q = m.find('"', q + 1);
      q = q == std::string::npos ? m.size() : q + 1;
    } else {
      while (q < m.size() && m[q] != ',' && m[q] != '}' && m[q] != '\n') ++q;
    }
    m.replace(e, q - e, "null");
  }
  return m;
}

struct Release {
  bool ok = false;
  std::string why;
  std::vector<std::string> log;
};

struct Request {
  std::string prereg_dir;        // directory of the manifest, designation file, review file, gate file and markers
  std::string manifest_name;     // curvy-it-gate.v3.manifest.json
  std::string gate_id;           // curvy-it-gate
  std::string gate_version;      // "3"
  std::string env_value;         // SLAM_REFACT_CURVY_DESIGNATION
  std::vector<std::string> pins; // paths of executables / WASM files that must match held_pins[<basename>] (incl. /proc/self/exe)
  std::string kind;              // data kind: roi | cylinder | cone | wasm-cylinder | wasm-cone
  std::string data_dir;          // directory whose tree sha256 must equal inputs_tree_sha256[kind] ("" = none)
};

inline Release CheckRelease(const Request& rq) {
  Release rel;
  auto fail = [&](const std::string& why) {
    rel.ok = false;
    rel.why = why;
    return rel;
  };
  if (rq.env_value.empty()) {
    return fail("HELD: SLAM_REFACT_CURVY_DESIGNATION is not set (the first real run of " + rq.gate_id +
                " needs the coordinator-designated review)");
  }
  std::string manifest;
  if (!ReadFile(rq.prereg_dir + "/" + rq.manifest_name, &manifest)) return fail("HELD: manifest unreadable");
  const std::string review = JsonObject(manifest, "review"), held = JsonObject(manifest, "held_run");
  const std::string at = FindValue(review, "at"), created = FindValue(manifest, "created_at");
  if (at.empty()) return fail("HELD: manifest review.at is null: the review is not recorded");
  if (created.empty() || at.size() != created.size() || !(created < at)) {
    return fail("HELD: manifest review.at is not later than created_at");
  }
  const std::string dname = FindValue(review, "designation_file"), dsha = FindValue(review, "designation_file_sha256");
  if (dname.empty() || dsha.empty() || dname.find('/') != std::string::npos) {
    return fail("HELD: manifest review.designation_file / designation_file_sha256 missing");
  }
  std::string designation;
  if (!ReadFile(rq.prereg_dir + "/" + dname, &designation)) return fail("HELD: designation file unreadable");
  const std::string dactual = testutil::Sha256Hex(designation);
  rel.log.push_back("[CURVY-HELD] designation_file " + dname + " sha256 actual=" + dactual + " manifest=" + dsha);
  if (dactual != dsha) return fail("designation file sha256 differs from the manifest");
  if (rq.env_value != dsha) return fail("HELD: SLAM_REFACT_CURVY_DESIGNATION is not the sha256 of the registered designation file");
  if (FindValue(designation, "gate") != rq.gate_id) return fail("designation file does not name " + rq.gate_id);
  if (FindValue(designation, "version") != rq.gate_version) return fail("designation file names another gate version");
  if (FindValue(designation, "first_real_run_permitted") != "true") return fail("designation file: first_real_run_permitted is not true");
  // the gate file
  const std::string gname = FindValue(held, "gate_file"), gsha = FindValue(held, "gate_file_sha256");
  if (gname.empty() || gsha.empty() || gname.find('/') != std::string::npos) return fail("HELD: manifest held_run.gate_file / gate_file_sha256 missing");
  bool ok = false;
  const std::string gactual = Sha256File(rq.prereg_dir + "/" + gname, &ok);
  rel.log.push_back("[CURVY-HELD] gate_file " + gname + " sha256 actual=" + gactual + " registered=" + gsha);
  if (!ok || gactual != gsha || gsha != FindValue(manifest, "sha256")) return fail("gate file sha256 differs from the manifest");
  if (FindValue(designation, "gate_file_sha256") != gactual) return fail("designation gate_file_sha256 differs from the gate file");
  // the manifest core, the review file
  const std::string core = testutil::Sha256Hex(ManifestCoreText(manifest));
  rel.log.push_back("[CURVY-HELD] manifest_core_sha256 actual=" + core + " designation=" + FindValue(designation, "manifest_core_sha256"));
  if (FindValue(designation, "manifest_core_sha256") != core) return fail("manifest core sha256 differs from the designation (the manifest was edited beyond review / first_real_run_at)");
  const std::string rname = FindValue(review, "file"), rsha = FindValue(review, "sha256");
  if (rname.empty() || rsha.empty() || rname.find('/') != std::string::npos) return fail("HELD: manifest review.file / review.sha256 missing");
  const std::string ractual = Sha256File(rq.prereg_dir + "/" + rname, &ok);
  if (!ok || ractual != rsha || FindValue(designation, "review_file_sha256") != ractual) return fail("review file sha256 differs from the manifest or the designation");
  // executables
  const std::string pins = JsonObject(manifest, "held_pins");
  for (const std::string& path : rq.pins) {
    const size_t slash = path.find_last_of('/');
    const std::string base = slash == std::string::npos ? path : path.substr(slash + 1);
    const std::string want = FindValue(pins, base);
    const std::string have = Sha256File(path, &ok);
    rel.log.push_back("[CURVY-HELD] pin " + base + " sha256 actual=" + have + " registered=" + want);
    if (want.empty()) return fail("HELD: manifest held_pins has no entry for " + base);
    if (!ok || have != want) return fail("held file " + base + " sha256 differs from the registered one");
  }
  // input data
  if (!rq.data_dir.empty()) {
    const std::string want = FindValue(JsonObject(manifest, "inputs_tree_sha256"), rq.kind);
    const std::string have = TreeSha(rq.data_dir);
    rel.log.push_back("[CURVY-HELD] inputs " + rq.kind + " tree sha256 actual=" + have + " registered=" + want);
    if (want.empty()) return fail("HELD: manifest inputs_tree_sha256 has no entry for " + rq.kind);
    if (have != want) return fail("input data " + rq.data_dir + " differs from the registered data (tree sha256)");
  }
  if (!FindValue(manifest, "first_real_run_at").empty()) {
    return fail("manifest first_real_run_at is already set: this gate was run before (one run per registration)");
  }
  rel.ok = true;
  return rel;
}

// One-run marker (protocol 12.6): O_EXCL, never removed. A refusal before this point is not a run.
inline bool ClaimFirstRun(const std::string& prereg_dir, const std::string& gate_id, const std::string& version,
                          const std::string& kind, std::string* why) {
  const std::string path = prereg_dir + "/" + gate_id + ".v" + version + ".first-run." + kind + ".marker";
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0444);
  if (fd < 0) {
    *why = "HELD: the first-run marker " + path + " exists (or cannot be created): this data kind was run before";
    return false;
  }
  const std::string body = "started\n";
  const ssize_t n = ::write(fd, body.data(), body.size());
  ::close(fd);
  (void)n;
  return true;
}

// The registered data must never be used on the dev path (a directory that has the registered tree sha256).
inline bool IsRegisteredTree(const std::string& prereg_dir, const std::string& manifest_name, const std::string& kind,
                             const std::string& data_dir) {
  std::string manifest;
  if (!ReadFile(prereg_dir + "/" + manifest_name, &manifest)) return false;
  const std::string want = FindValue(JsonObject(manifest, "inputs_tree_sha256"), kind);
  return !want.empty() && TreeSha(data_dir) == want;
}

inline std::string SelfExe() {
  char buf[4096];
  const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  return n > 0 ? std::string(buf, static_cast<size_t>(n)) : std::string();
}

// Process-local memo: a process that claimed a kind has the right to use it for all its tests (one real run = one process).
inline std::set<std::string>& ClaimedKinds() {
  static std::set<std::string> s;
  return s;
}

// The complete gate: CheckRelease, then the marker. Returns "" when released.
inline std::string HeldGate(const Request& rq, std::vector<std::string>* log = nullptr) {
  if (ClaimedKinds().count(rq.kind) != 0) return "";
  const Release rel = CheckRelease(rq);
  if (log != nullptr) *log = rel.log;
  if (!rel.ok) return rel.why;
  std::string why;
  if (!ClaimFirstRun(rq.prereg_dir, rq.gate_id, rq.gate_version, rq.kind, &why)) return why;
  ClaimedKinds().insert(rq.kind);
  return "";
}

}  // namespace slam_refact::curvy_held

#endif  // SLAM_REFACT_TESTS_IMAGE_TARGET_CURVY_HELD_H_
