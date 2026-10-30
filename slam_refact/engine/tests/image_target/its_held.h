// HELD device of the image-target scale gate (build/slam-refact-full-20261019/preregistration/it-scale-gate.v1.md section 6),
// after preregistration-protocol.md sections 12.5 and 12.6. It follows curvy_held.h (the US-014 device) and reuses its pure
// helpers (file and tree sha256, the minimal JSON access, the manifest "core", the one-run marker); only the release check
// has its own messages, tag and environment variable. The first real run of the gate is refused by CODE unless every
// condition holds, and it can happen once:
//   (a) env SLAM_REFACT_ITS_DESIGNATION = sha256 of the coordinator's designation file = manifest review.designation_file_sha256
//       = the actual sha256 of that file;
//   (b) manifest review.at is recorded and later than created_at;
//   (c) the designation file names the gate and version, first_real_run_permitted is true, and its gate_file_sha256,
//       manifest_core_sha256 and review_file_sha256 equal the actual files (manifest core = the manifest text with the review
//       object and the first_real_run_at value blanked) and the manifest's review.sha256;
//   (d) the gate file sha256 equals manifest held_run.gate_file_sha256 and manifest sha256;
//   (e) every pinned executable (/proc/self/exe) has the sha256 of manifest held_pins[<basename>];
//   (f) the tree sha256 of the registered dataset directory equals manifest inputs_tree_sha256[registered];
//   (g) manifest first_real_run_at is still null, and the one-run marker <gate id>.v<version>.first-run.<kind>.marker is
//       created with O_EXCL (a start that created it is a run; the code never removes it).
// It is an accident guard, not an authentication (protocol 12.6): anyone who can write the manifest, the designation file
// and the marker directory can satisfy it. Development data never needs it, and registered data is refused on the dev path
// (IsRegisteredTree).

#ifndef SLAM_REFACT_TESTS_IMAGE_TARGET_ITS_HELD_H_
#define SLAM_REFACT_TESTS_IMAGE_TARGET_ITS_HELD_H_

#include <set>
#include <string>
#include <vector>

#include "curvy_held.h"

namespace slam_refact::its_held {

using curvy_held::ClaimFirstRun;
using curvy_held::FindValue;
using curvy_held::IsRegisteredTree;
using curvy_held::JsonObject;
using curvy_held::ManifestCoreText;
using curvy_held::ReadFile;
using curvy_held::Release;
using curvy_held::SelfExe;
using curvy_held::Sha256File;
using curvy_held::TreeSha;

constexpr const char* kEnvName = "SLAM_REFACT_ITS_DESIGNATION";

struct Request {
  std::string prereg_dir;         // directory of the manifest, designation file, review file, gate file and markers
  std::string manifest_name;      // it-scale-gate.v1.manifest.json
  std::string gate_id;            // it-scale-gate
  std::string gate_version;       // "1"
  std::string env_value;          // value of SLAM_REFACT_ITS_DESIGNATION
  std::vector<std::string> pins;  // executables that must match held_pins[<basename>] (incl. /proc/self/exe)
  std::string kind;               // data kind: registered
  std::string data_dir;           // directory whose tree sha256 must equal inputs_tree_sha256[kind] ("" = none)
};

inline Release CheckRelease(const Request& rq) {
  Release rel;
  auto fail = [&](const std::string& why) {
    rel.ok = false;
    rel.why = why;
    return rel;
  };
  if (rq.env_value.empty()) {
    return fail(std::string("HELD: ") + kEnvName + " is not set (the first real run of " + rq.gate_id +
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
  rel.log.push_back("[ITS-HELD] designation_file " + dname + " sha256 actual=" + dactual + " manifest=" + dsha);
  if (dactual != dsha) return fail("designation file sha256 differs from the manifest");
  if (rq.env_value != dsha) {
    return fail(std::string("HELD: ") + kEnvName + " is not the sha256 of the registered designation file");
  }
  if (FindValue(designation, "gate") != rq.gate_id) return fail("designation file does not name " + rq.gate_id);
  if (FindValue(designation, "version") != rq.gate_version) return fail("designation file names another gate version");
  if (FindValue(designation, "first_real_run_permitted") != "true") {
    return fail("designation file: first_real_run_permitted is not true");
  }
  // the gate file
  const std::string gname = FindValue(held, "gate_file"), gsha = FindValue(held, "gate_file_sha256");
  if (gname.empty() || gsha.empty() || gname.find('/') != std::string::npos) {
    return fail("HELD: manifest held_run.gate_file / gate_file_sha256 missing");
  }
  bool ok = false;
  const std::string gactual = Sha256File(rq.prereg_dir + "/" + gname, &ok);
  rel.log.push_back("[ITS-HELD] gate_file " + gname + " sha256 actual=" + gactual + " registered=" + gsha);
  if (!ok || gactual != gsha || gsha != FindValue(manifest, "sha256")) return fail("gate file sha256 differs from the manifest");
  if (FindValue(designation, "gate_file_sha256") != gactual) return fail("designation gate_file_sha256 differs from the gate file");
  // the manifest core, the review file
  const std::string core = testutil::Sha256Hex(ManifestCoreText(manifest));
  rel.log.push_back("[ITS-HELD] manifest_core_sha256 actual=" + core + " designation=" + FindValue(designation, "manifest_core_sha256"));
  if (FindValue(designation, "manifest_core_sha256") != core) {
    return fail("manifest core sha256 differs from the designation (the manifest was edited beyond review / first_real_run_at)");
  }
  const std::string rname = FindValue(review, "file"), rsha = FindValue(review, "sha256");
  if (rname.empty() || rsha.empty() || rname.find('/') != std::string::npos) return fail("HELD: manifest review.file / review.sha256 missing");
  const std::string ractual = Sha256File(rq.prereg_dir + "/" + rname, &ok);
  if (!ok || ractual != rsha || FindValue(designation, "review_file_sha256") != ractual) {
    return fail("review file sha256 differs from the manifest or the designation");
  }
  // executables
  const std::string pins = JsonObject(manifest, "held_pins");
  for (const std::string& path : rq.pins) {
    const size_t slash = path.find_last_of('/');
    const std::string base = slash == std::string::npos ? path : path.substr(slash + 1);
    const std::string want = FindValue(pins, base);
    const std::string have = Sha256File(path, &ok);
    rel.log.push_back("[ITS-HELD] pin " + base + " sha256 actual=" + have + " registered=" + want);
    if (want.empty()) return fail("HELD: manifest held_pins has no entry for " + base);
    if (!ok || have != want) return fail("held file " + base + " sha256 differs from the registered one");
  }
  // input data
  if (!rq.data_dir.empty()) {
    const std::string want = FindValue(JsonObject(manifest, "inputs_tree_sha256"), rq.kind);
    const std::string have = TreeSha(rq.data_dir);
    rel.log.push_back("[ITS-HELD] inputs " + rq.kind + " tree sha256 actual=" + have + " registered=" + want);
    if (want.empty()) return fail("HELD: manifest inputs_tree_sha256 has no entry for " + rq.kind);
    if (have != want) return fail("input data " + rq.data_dir + " differs from the registered data (tree sha256)");
  }
  if (!FindValue(manifest, "first_real_run_at").empty()) {
    return fail("manifest first_real_run_at is already set: this gate was run before (one run per registration)");
  }
  rel.ok = true;
  return rel;
}

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

}  // namespace slam_refact::its_held

#endif  // SLAM_REFACT_TESTS_IMAGE_TARGET_ITS_HELD_H_
