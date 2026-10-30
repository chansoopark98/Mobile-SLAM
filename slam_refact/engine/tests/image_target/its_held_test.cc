// Unit tests of the HELD device of the image-target scale gate (its_held.h) on synthetic files: every bypass the curvy v2
// review found (and the ones of protocol 12.5 / 12.6) must be rejected for the expected reason, and the valid case must pass
// exactly once. No engine, no real manifest, no registered data. Same cases as curvy_held_test.cc, on the ITS release check.

#include <gtest/gtest.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <fstream>
#include <string>
#include <vector>

#include "its_held.h"

#ifndef SR_ITS_PYTHON
#define SR_ITS_PYTHON ""
#define SR_ITS_HELD_TOOL ""
#endif

namespace slam_refact {
namespace {

using its_held::CheckRelease;
using its_held::ClaimFirstRun;
using its_held::ReadFile;
using its_held::Release;
using its_held::Request;
using testutil::Sha256Hex;

constexpr char kAt[] = "2026-10-20T11:00:00+09:00";
constexpr char kCreated[] = "2026-10-20T10:30:00+09:00";

std::string MakeDir() {
  char tmpl[] = "/tmp/its-held-test-XXXXXX";
  const char* d = ::mkdtemp(tmpl);
  EXPECT_NE(d, nullptr);
  return d == nullptr ? std::string() : std::string(d);
}

void Write(const std::string& path, const std::string& body) {
  std::ofstream f(path, std::ios::binary);
  f << body;
}

struct Scenario {
  std::string dir, data, pin;
  std::string gate = "gate text", review = "review text", pin_body = "binary bytes";
  std::string review_at = kAt, created = kCreated, first_run = "null", env_override;
  std::string designation_gate_id = "it-scale-gate", designation_version = "1", permitted = "true";
  // knobs applied after the valid files were written
  bool edit_manifest_after_designation = false, edit_gate_after = false, edit_review_after = false,
       edit_pin_after = false, edit_data_after = false, drop_pin_entry = false, drop_input_entry = false;
  std::string designation_sha_in_manifest_override;
};

struct Built {
  Request rq;
  std::string manifest_path;
};

Built Build(const Scenario& sc) {
  Built b;
  const std::string dir = sc.dir;
  Write(dir + "/gate.md", sc.gate);
  Write(dir + "/review.md", sc.review);
  Write(dir + "/pinned.bin", sc.pin_body);
  ::mkdir((dir + "/data").c_str(), 0755);
  Write(dir + "/data/a.txt", "alpha");
  Write(dir + "/data/b.txt", "beta");
  const std::string tree = its_held::TreeSha(dir + "/data");
  const std::string gate_sha = Sha256Hex(sc.gate), review_sha = Sha256Hex(sc.review), pin_sha = Sha256Hex(sc.pin_body);
  auto manifest = [&](const std::string& designation_sha) {
    return std::string("{\n \"gate_id\": \"it-scale-gate\",\n \"version\": 1,\n \"file\": \"x/gate.md\",\n \"sha256\": \"") + gate_sha +
           "\",\n \"created_at\": \"" + sc.created + "\",\n \"note\": \"the word review appears in text: review.at\",\n \"held_run\": {\n  \"gate_file\": \"gate.md\",\n"
           "  \"gate_file_sha256\": \"" + gate_sha + "\"\n },\n \"held_pins\": {\n" +
           (sc.drop_pin_entry ? "" : "  \"pinned.bin\": \"" + pin_sha + "\"\n") + " },\n \"inputs_tree_sha256\": {\n" +
           (sc.drop_input_entry ? "" : "  \"k\": \"" + tree + "\"\n") + " },\n \"review\": {\n  \"reviewer\": \"r\",\n  \"at\": " +
           (sc.review_at.empty() ? std::string("null") : "\"" + sc.review_at + "\"") + ",\n  \"file\": \"review.md\",\n  \"sha256\": \"" + review_sha +
           "\",\n  \"designation_file\": \"designation.json\",\n  \"designation_file_sha256\": \"" + designation_sha + "\"\n },\n \"first_real_run_at\": " +
           sc.first_run + "\n}\n";
  };
  const std::string core = Sha256Hex(its_held::ManifestCoreText(manifest("")));
  const std::string designation = "{\n \"gate\": \"" + sc.designation_gate_id + "\",\n \"version\": " + sc.designation_version +
                                  ",\n \"first_real_run_permitted\": " + sc.permitted + ",\n \"gate_file_sha256\": \"" + gate_sha +
                                  "\",\n \"manifest_core_sha256\": \"" + core + "\",\n \"review_file_sha256\": \"" + review_sha + "\"\n}\n";
  Write(dir + "/designation.json", designation);
  const std::string dsha = sc.designation_sha_in_manifest_override.empty() ? Sha256Hex(designation) : sc.designation_sha_in_manifest_override;
  std::string m = manifest(dsha);
  if (sc.edit_manifest_after_designation) {
    const size_t at = m.find("\"version\": 1");
    m.replace(at, 12, "\"version\": 4");   // an edit of the core after the designation
  }
  Write(dir + "/manifest.json", m);
  if (sc.edit_gate_after) Write(dir + "/gate.md", sc.gate + " edited");
  if (sc.edit_review_after) Write(dir + "/review.md", sc.review + " edited");
  if (sc.edit_pin_after) Write(dir + "/pinned.bin", sc.pin_body + " rebuilt");
  if (sc.edit_data_after) Write(dir + "/data/a.txt", "alpha changed");
  b.rq.prereg_dir = dir;
  b.rq.manifest_name = "manifest.json";
  b.rq.gate_id = "it-scale-gate";
  b.rq.gate_version = "1";
  b.rq.env_value = sc.env_override.empty() ? Sha256Hex(designation) : (sc.env_override == "-" ? "" : sc.env_override);
  b.rq.pins = {dir + "/pinned.bin"};
  b.rq.kind = "k";
  b.rq.data_dir = dir + "/data";
  b.manifest_path = dir + "/manifest.json";
  return b;
}

Scenario Fresh() {
  Scenario s;
  s.dir = MakeDir();
  return s;
}

TEST(ItsHeld, ValidReleaseThenTheMarkerAllowsOneProcessOnly) {
  const Scenario sc = Fresh();
  const Built b = Build(sc);
  const Release r = CheckRelease(b.rq);
  EXPECT_TRUE(r.ok) << r.why;
  std::string why;
  EXPECT_TRUE(ClaimFirstRun(sc.dir, "it-scale-gate", "1", "k", &why)) << why;
  EXPECT_FALSE(ClaimFirstRun(sc.dir, "it-scale-gate", "1", "k", &why));       // O_EXCL: a second claim is refused
  EXPECT_NE(why.find("marker"), std::string::npos);
  EXPECT_TRUE(ClaimFirstRun(sc.dir, "it-scale-gate", "1", "other-kind", &why));  // per data kind
  // HeldGate: first call claims, the same process keeps the right, another kind needs its own marker
  const Scenario sc2 = Fresh();
  Built b2 = Build(sc2);
  EXPECT_EQ(its_held::HeldGate(b2.rq), "");
  EXPECT_EQ(its_held::HeldGate(b2.rq), "");  // memo of this process
  its_held::ClaimedKinds().clear();           // a new process: the marker is there
  EXPECT_NE(its_held::HeldGate(b2.rq).find("marker"), std::string::npos);
}

struct Bypass {
  const char* name;
  void (*edit)(Scenario*);
  const char* expect;
};

TEST(ItsHeld, EveryBypassIsRejectedForTheExpectedReason) {
  const Bypass cases[] = {
      {"env not set", [](Scenario* s) { s->env_override = "-"; }, "is not set"},
      {"env is an arbitrary non-empty string", [](Scenario* s) { s->env_override = "1"; }, "not the sha256"},
      {"review.at null", [](Scenario* s) { s->review_at = ""; }, "review.at is null"},
      {"review.at earlier than created_at", [](Scenario* s) { s->review_at = "2026-10-20T10:00:00+09:00"; }, "not later than created_at"},
      {"review.at of another shape", [](Scenario* s) { s->review_at = "later"; }, "not later than created_at"},
      {"designation edited after registration", [](Scenario* s) { s->designation_sha_in_manifest_override = Sha256Hex("{}"); },
       "designation file sha256 differs"},
      {"designation names another gate", [](Scenario* s) { s->designation_gate_id = "other"; }, "does not name"},
      {"designation names another version", [](Scenario* s) { s->designation_version = "2"; }, "another gate version"},
      {"designation does not permit", [](Scenario* s) { s->permitted = "false"; }, "first_real_run_permitted is not true"},
      {"gate file changed", [](Scenario* s) { s->edit_gate_after = true; }, "gate file sha256 differs"},
      {"manifest edited beyond review / first_real_run_at", [](Scenario* s) { s->edit_manifest_after_designation = true; }, "manifest core sha256 differs"},
      {"review file changed", [](Scenario* s) { s->edit_review_after = true; }, "review file sha256 differs"},
      {"held binary rebuilt", [](Scenario* s) { s->edit_pin_after = true; }, "held file pinned.bin sha256 differs"},
      {"held binary not pinned at all", [](Scenario* s) { s->drop_pin_entry = true; }, "no entry for pinned.bin"},
      {"input data changed", [](Scenario* s) { s->edit_data_after = true; }, "differs from the registered data"},
      {"input kind not registered", [](Scenario* s) { s->drop_input_entry = true; }, "no entry for k"},
      {"already run once", [](Scenario* s) { s->first_run = "\"2026-10-20T12:00:00+09:00\""; }, "first_real_run_at is already set"},
  };
  for (const Bypass& c : cases) {
    Scenario sc = Fresh();
    c.edit(&sc);
    const Built b = Build(sc);
    const Release r = CheckRelease(b.rq);
    EXPECT_FALSE(r.ok) << c.name;
    EXPECT_NE(r.why.find(c.expect), std::string::npos) << c.name << ": got '" << r.why << "'";
  }
}

TEST(ItsHeld, MessagesNameTheItsEnvironmentVariable) {
  Scenario sc = Fresh();
  sc.env_override = "-";
  const Built b = Build(sc);
  const Release r = CheckRelease(b.rq);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.why.find("SLAM_REFACT_ITS_DESIGNATION"), std::string::npos) << r.why;
  EXPECT_EQ(std::string(its_held::kEnvName), "SLAM_REFACT_ITS_DESIGNATION");
  Scenario other = Fresh();
  other.env_override = "1";
  const Release r2 = CheckRelease(Build(other).rq);
  EXPECT_NE(r2.why.find("SLAM_REFACT_ITS_DESIGNATION"), std::string::npos) << r2.why;
  EXPECT_EQ(r2.why.find("CURVY"), std::string::npos) << r2.why;
}

TEST(ItsHeld, FirstRunAtOnlyChangesNothingInTheCore) {
  // The coordinator sets review.* at the designation and the executor / coordinator sets first_real_run_at after the run:
  // neither may change the core sha256; any other manifest edit must.
  const Scenario sc = Fresh();
  const Built b = Build(sc);
  std::string m;
  ASSERT_TRUE(ReadFile(b.manifest_path, &m));
  const std::string core = Sha256Hex(its_held::ManifestCoreText(m));
  std::string m2 = m;
  m2.replace(m2.find("\"first_real_run_at\": null"), 25, "\"first_real_run_at\": \"2026-10-20T12:00:00+09:00\"");
  EXPECT_EQ(Sha256Hex(its_held::ManifestCoreText(m2)), core);
  std::string m3 = m;
  m3.replace(m3.find("\"reviewer\": \"r\""), 15, "\"reviewer\": \"someone else, longer\"");
  EXPECT_EQ(Sha256Hex(its_held::ManifestCoreText(m3)), core);
  std::string m4 = m;
  m4.replace(m4.find("\"created_at\""), 12, "\"created_AT\"");
  EXPECT_NE(Sha256Hex(its_held::ManifestCoreText(m4)), core);
}

TEST(ItsHeld, CoreShaAndTreeShaAgreeWithThePythonTool) {
  ASSERT_FALSE(std::string(SR_ITS_PYTHON).empty()) << "BLOCKED: SR_ITS_PYTHON not set";
  const Scenario sc = Fresh();
  const Built b = Build(sc);
  const std::string out = sc.dir + "/py.txt";
  const std::string base = std::string("\"") + SR_ITS_PYTHON + "\" -I \"" + SR_ITS_HELD_TOOL + "\" ";
  ASSERT_EQ(std::system((base + "core-sha \"" + b.manifest_path + "\" > \"" + out + "\"").c_str()), 0);
  std::string text;
  ASSERT_TRUE(ReadFile(out, &text));
  std::string m;
  ASSERT_TRUE(ReadFile(b.manifest_path, &m));
  EXPECT_EQ(text.substr(0, 64), Sha256Hex(its_held::ManifestCoreText(m)));
  ASSERT_EQ(std::system((base + "tree-sha \"" + sc.dir + "/data\" > \"" + out + "\"").c_str()), 0);
  ASSERT_TRUE(ReadFile(out, &text));
  EXPECT_EQ(text.substr(0, 64), its_held::TreeSha(sc.dir + "/data"));
}

TEST(ItsHeld, RegisteredDataIsRefusedOnTheDevPath) {
  const Scenario sc = Fresh();
  const Built b = Build(sc);
  EXPECT_TRUE(its_held::IsRegisteredTree(sc.dir, "manifest.json", "k", sc.dir + "/data"));
  Write(sc.dir + "/data/c.txt", "one more file");
  EXPECT_FALSE(its_held::IsRegisteredTree(sc.dir, "manifest.json", "k", sc.dir + "/data"));
  EXPECT_FALSE(its_held::IsRegisteredTree(sc.dir, "manifest.json", "unknown-kind", sc.dir + "/data"));
  EXPECT_FALSE(its_held::IsRegisteredTree(sc.dir, "missing.json", "k", sc.dir + "/data"));
}

TEST(ItsHeld, TreeShaIsContentSensitiveAndOrderIndependent) {
  const Scenario sc = Fresh();
  Build(sc);
  const std::string a = its_held::TreeSha(sc.dir + "/data");
  EXPECT_EQ(a.size(), 64u);
  Write(sc.dir + "/data/a.txt", "alpha!");
  EXPECT_NE(its_held::TreeSha(sc.dir + "/data"), a);
  Write(sc.dir + "/data/a.txt", "alpha");
  EXPECT_EQ(its_held::TreeSha(sc.dir + "/data"), a);
  EXPECT_EQ(its_held::TreeSha(sc.dir + "/does-not-exist"), "");
}

}  // namespace
}  // namespace slam_refact
