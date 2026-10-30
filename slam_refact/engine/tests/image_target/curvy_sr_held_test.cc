// Unit tests of the curvy gate front of the shared HELD device (curvy_sr_held.h) on synthetic files. They check what the front
// adds to sr_held.h (whose own decisions are covered by tools/prereg/sr_held_cases.json, tests/support/sr_held_conformance.cc):
//   - without any selection the mode is REPORT (dev data runs, registered data is HELD), as before;
//   - nothing about the gate is compiled in: an incomplete or malformed selection is HELD, never REPORT, and a designation
//     without a gate is refused;
//   - a selected gate: sealed data is refused on the dev path, every bypass of the registered path is refused for the expected
//     code and creates no marker, a valid release claims one marker per kind and a process reuses its own claim.
// No engine, no real manifest, no registered data. The gate is a made-up "demo-curvy" version 7.

#include <gtest/gtest.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <fstream>
#include <string>
#include <vector>

#include "curvy_sr_held.h"

namespace slam_refact {
namespace {

namespace ch = curvy_gate_held;
using Selection = ch::Selection;
using testutil::Sha256Hex;

constexpr char kGate[] = "demo-curvy";
constexpr int kVersion = 7;

std::string MakeDir() {
  char tmpl[] = "/tmp/curvy-sr-held-test-XXXXXX";
  const char* d = ::mkdtemp(tmpl);
  EXPECT_NE(d, nullptr);
  return d == nullptr ? std::string() : std::string(d);
}

void Write(const std::string& path, const std::string& body) {
  std::ofstream f(path, std::ios::binary);
  f << body;
}

bool Exists(const std::string& path) {
  struct stat st;
  return ::lstat(path.c_str(), &st) == 0;
}

std::string Sha(const std::string& path) {
  std::string sha;
  EXPECT_TRUE(sr_held::Sha256File(path, &sha)) << path;
  return sha;
}

std::string TreeOf(const std::string& dir) {
  std::string sha, code;
  EXPECT_TRUE(sr_held::TreeSha(dir, &sha, &code)) << dir << " " << code;
  return sha;
}

struct Knobs {
  std::string review_at = "\"2026-10-20T11:00:00+09:00\"";
  std::string first_run = "null";
  bool tamper_manifest_after_designation = false;
  bool wrong_gate_in_designation = false;
};

// A registration: prereg dir, two data dirs (kinds a and b) and the pins {this executable, pinned.bin}.
struct Scenario {
  std::string root, prereg, data_a, data_b, pin;
  std::string designation_sha;
  Selection selection;
};

Scenario Build(const std::vector<std::string>& kinds, const Knobs& k = Knobs()) {
  Scenario sc;
  sc.root = MakeDir();
  sc.prereg = sc.root + "/prereg";
  sc.data_a = sc.root + "/data-a";
  sc.data_b = sc.root + "/data-b";
  sc.pin = sc.root + "/pinned.bin";
  ::mkdir(sc.prereg.c_str(), 0755);
  ::mkdir(sc.data_a.c_str(), 0755);
  ::mkdir(sc.data_b.c_str(), 0755);
  Write(sc.data_a + "/gt.tsv", "variant\tregistered\n");
  Write(sc.data_b + "/gt.tsv", "variant\tregistered\nb\n");
  Write(sc.pin, "pinned bytes");
  const std::string name = std::string(kGate) + ".v" + std::to_string(kVersion);
  Write(sc.prereg + "/" + name + ".md", "gate text");
  Write(sc.prereg + "/" + name + ".review.md", "review text");
  std::string kind_list;
  for (const std::string& kind : kinds) kind_list += (kind_list.empty() ? "" : ", ") + std::string("\"") + kind + "\"";
  std::string inputs;
  for (const std::string& kind : kinds) inputs += (inputs.empty() ? "" : ", ") + std::string("\"") + kind + "\": \"" + TreeOf(kind.back() == 'b' ? sc.data_b : sc.data_a) + "\"";
  const std::string self = sr_held::SelfExe();
  const std::string self_base = self.substr(self.find_last_of('/') + 1);
  auto manifest = [&](const std::string& designation_sha) {
    return std::string("{\n \"gate_id\": \"") + kGate + "\",\n \"version\": " + std::to_string(kVersion) + ",\n \"file\": \"" + name +
           ".md\",\n \"sha256\": \"" + Sha256Hex("gate text") + "\",\n \"created_at\": \"2026-10-20T10:00:00+09:00\",\n \"held\": {\n  \"kinds\": [" +
           kind_list + "],\n  \"pins\": {\"" + self_base + "\": \"" + Sha(self) + "\", \"pinned.bin\": \"" + Sha(sc.pin) +
           "\"},\n  \"inputs\": {" + inputs + "},\n  \"seed\": false\n },\n \"review\": {\n  \"reviewer\": \"r\",\n  \"at\": " + k.review_at +
           ",\n  \"file\": \"" + name + ".review.md\",\n  \"sha256\": \"" + Sha256Hex("review text") + "\",\n  \"designation_file\": \"" + name +
           ".designation.json\",\n  \"designation_file_sha256\": \"" + designation_sha + "\"\n },\n \"first_real_run_at\": " + k.first_run + "\n}\n";
  };
  const std::string manifest_path = sc.prereg + "/" + name + ".manifest.json";
  Write(manifest_path, manifest(std::string(64, '0')));
  std::string text, err, canon;
  sr_held::Json m;
  EXPECT_TRUE(sr_held::ReadFile(manifest_path, &text));
  EXPECT_TRUE(sr_held::ParseJson(text, &m, &err)) << err;
  EXPECT_TRUE(sr_held::CanonManifestSha(m, &canon));
  const std::string designation =
      std::string("{\"gate\": \"") + (k.wrong_gate_in_designation ? "other-gate.v7" : name) + "\", \"first_real_run_permitted\": true, \"pinned\": {\"gate_md_sha256\": \"" +
      Sha256Hex("gate text") + "\", \"review_file_sha256\": \"" + Sha256Hex("review text") + "\", \"manifest_before_review_sha256\": \"" + canon + "\"}}\n";
  Write(sc.prereg + "/" + name + ".designation.json", designation);
  sc.designation_sha = Sha256Hex(designation);
  Write(manifest_path, manifest(sc.designation_sha));
  if (k.tamper_manifest_after_designation) {
    std::string t;
    sr_held::ReadFile(manifest_path, &t);
    t.replace(t.find("\"seed\": false"), 13, "\"seed\": true ");
    Write(manifest_path, t);
  }
  sc.selection = ch::Select(kGate, std::to_string(kVersion), sc.prereg, sc.designation_sha, sc.pin);
  return sc;
}

std::string MarkerOf(const Scenario& sc, const std::string& kind) {
  return sc.prereg + "/" + kGate + ".v" + std::to_string(kVersion) + ".first-run." + kind + ".marker";
}

bool Has(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

// ---------------------------------------------------------------------------------------------------------------------

TEST(CurvySrHeld, NothingSelectedIsReportModeNotAGate) {
  const Selection s = ch::Select("", "", "", "", "");
  EXPECT_EQ(s.state, Selection::State::kReport);
  const std::string dev = MakeDir();
  std::vector<std::string> log;
  EXPECT_EQ(ch::DevGuard(s, dev, &log), "") << "REPORT mode: dev data runs";
  ASSERT_EQ(log.size(), 1u);
  EXPECT_TRUE(Has(log[0], "REPORT mode"));
  const std::string held = ch::GateGuard(s, "report-kind", dev);
  EXPECT_TRUE(Has(held, "HELD:") && Has(held, ch::kEnvGateId)) << held;
}

TEST(CurvySrHeld, IncompleteOrMalformedSelectionIsHeldNeverReport) {
  const std::string d = "/some/prereg";
  const std::vector<Selection> refused = {
      ch::Select("demo-curvy", "", "", "", ""),                    // only the id
      ch::Select("", "7", "", "", ""),                             // only the version
      ch::Select("", "", d, "", ""),                               // only the directory
      ch::Select("demo-curvy", "7", "", "", ""),                   // no directory
      ch::Select("demo-curvy", "", d, "", ""),                     // no version
      ch::Select("", "7", d, "", ""),                              // no id
      ch::Select("", "", "", std::string(64, 'a'), ""),            // a designation without a gate (the old v3 invocation)
      ch::Select("", "", "", "", "/bin/true"),                     // pins without a gate
      ch::Select("../demo", "7", d, "", ""),                       // id with a path
      ch::Select("demo curvy", "7", d, "", ""),                    // id with a space
      ch::Select("demo-curvy", "0", d, "", ""),                    // version 0
      ch::Select("demo-curvy", "07", d, "", ""),                   // leading zero
      ch::Select("demo-curvy", "x", d, "", ""),                    // not a number
      ch::Select("demo-curvy", "1000000", d, "", ""),              // too large
      ch::Select("demo-curvy", "7 ", d, "", ""),                   // trailing space
      ch::Select("demo-curvy", "7", d, "", "/a::/b"),              // empty pin entry
      ch::Select("demo-curvy", "7", d, "", "/a:"),                 // trailing separator
      ch::Select("demo-curvy", "7", d, "", ":/a"),                 // leading separator
  };
  for (size_t i = 0; i < refused.size(); ++i) {
    EXPECT_EQ(refused[i].state, Selection::State::kRefused) << "case " << i;
    EXPECT_TRUE(Has(refused[i].why, "HELD:")) << "case " << i << ": " << refused[i].why;
    // every guard refuses a refused selection, on both paths
    EXPECT_EQ(ch::DevGuard(refused[i], "/x"), refused[i].why);
    EXPECT_EQ(ch::GateGuard(refused[i], "k-refused", "/x"), refused[i].why);
  }
  const Selection ok = ch::Select("demo-curvy", "7", d, "", "/a:/b");
  EXPECT_EQ(ok.state, Selection::State::kNamed);
  EXPECT_EQ(ok.gate_id, "demo-curvy");
  EXPECT_EQ(ok.version, 7);
  EXPECT_EQ(ok.prereg_dir, d);
  EXPECT_EQ(ok.pins, (std::vector<std::string>{"/a", "/b"}));
}

TEST(CurvySrHeld, SelectionIsReadFromTheEnvironmentOnly) {
  for (const char* n : {ch::kEnvGateId, ch::kEnvGateVersion, ch::kEnvPreregDir, ch::kEnvDesignation, ch::kEnvPins}) ::unsetenv(n);
  EXPECT_EQ(ch::SelectFromEnv().state, Selection::State::kReport);
  ::setenv(ch::kEnvGateId, "demo-curvy", 1);
  EXPECT_EQ(ch::SelectFromEnv().state, Selection::State::kRefused);
  ::setenv(ch::kEnvGateVersion, "7", 1);
  ::setenv(ch::kEnvPreregDir, "/p", 1);
  const Selection s = ch::SelectFromEnv();
  EXPECT_EQ(s.state, Selection::State::kNamed);
  EXPECT_EQ(s.version, 7);
  for (const char* n : {ch::kEnvGateId, ch::kEnvGateVersion, ch::kEnvPreregDir}) ::unsetenv(n);
  EXPECT_EQ(ch::SelectFromEnv().state, Selection::State::kReport);
}

TEST(CurvySrHeld, PinsIncludeTheRunningExecutableOnceAndEachFileOnce) {
  const Scenario sc = Build({"p-a"});
  const std::string self = sr_held::SelfExe();
  const std::vector<std::string> pins = ch::PinsOf(sc.selection, {self, sc.pin, sc.root + "/./pinned.bin"});
  ASSERT_EQ(pins.size(), 2u);
  EXPECT_EQ(pins[0], self);
  EXPECT_EQ(pins[1], sc.pin);
}

TEST(CurvySrHeld, ValidReleaseClaimsOneMarkerPerKindAndTheProcessReusesIt) {
  const Scenario sc = Build({"rel-a", "rel-b"});
  std::vector<std::string> log;
  EXPECT_FALSE(Exists(MarkerOf(sc, "rel-a")));
  EXPECT_EQ(ch::GateGuard(sc.selection, "rel-a", sc.data_a, &log), "") << ch::GateGuard(sc.selection, "rel-a", sc.data_a);
  EXPECT_TRUE(Exists(MarkerOf(sc, "rel-a")));
  EXPECT_FALSE(Exists(MarkerOf(sc, "rel-b"))) << "a kind that was not claimed has no marker";
  bool logged_designation = false;
  for (const std::string& l : log) logged_designation = logged_designation || Has(l, "designation_file");
  EXPECT_TRUE(logged_designation) << "the release log reaches the caller (audit F-6)";
  // the same process, same kind and directory: the claim is reused (CurvyCrop then CurvySynth)
  EXPECT_EQ(ch::GateGuard(sc.selection, "rel-a", sc.data_a), "");
  // the same kind on other data is not released
  const std::string other = ch::GateGuard(sc.selection, "rel-a", sc.data_b);
  EXPECT_TRUE(Has(other, "already released")) << other;
  // another kind needs and gets its own marker
  EXPECT_EQ(ch::GateGuard(sc.selection, "rel-b", sc.data_b), "");
  EXPECT_TRUE(Exists(MarkerOf(sc, "rel-b")));
}

TEST(CurvySrHeld, ASecondProcessIsRefusedByTheMarker) {
  const Scenario sc = Build({"mk-a"});
  Write(MarkerOf(sc, "mk-a"), "started\n");  // what an earlier run left
  const std::string why = ch::GateGuard(sc.selection, "mk-a", sc.data_a);
  EXPECT_TRUE(Has(why, "MARKER_EXISTS")) << why;
}

TEST(CurvySrHeld, EveryBypassIsRefusedForTheExpectedCodeAndLeavesNoMarker) {
  struct Case {
    std::string kind;
    std::string expect;
    Knobs knobs;
    int mutate;  // 0 none, 1 wrong designation value, 2 pin file edited, 3 pin withheld, 4 data edited, 5 unregistered kind,
                 // 6 other version selected, 7 no designation value
  };
  Knobs future_run;
  future_run.first_run = "\"2026-10-20T12:00:00+09:00\"";
  Knobs no_review;
  no_review.review_at = "null";
  Knobs tampered;
  tampered.tamper_manifest_after_designation = true;
  Knobs wrong_gate;
  wrong_gate.wrong_gate_in_designation = true;
  Knobs review_in_the_future;
  review_in_the_future.review_at = "\"2999-01-01T00:00:00+09:00\"";
  const std::vector<Case> cases = {
      {"by-env", "ENV_MISMATCH", Knobs(), 1},          {"by-no-env", "ENV_MISSING", Knobs(), 7},
      {"by-pin-edit", "PIN_MISMATCH", Knobs(), 2},     {"by-pin-withheld", "PIN_NOT_PRESENTED", Knobs(), 3},
      {"by-data", "INPUT_MISMATCH", Knobs(), 4},       {"by-kind", "KIND_UNREGISTERED", Knobs(), 5},
      {"by-version", "MANIFEST_UNREADABLE", Knobs(), 6}, {"by-run-at", "ALREADY_RUN", future_run, 0},
      {"by-review", "REVIEW_MISSING", no_review, 0},   {"by-tamper", "DESIGNATION_PIN", tampered, 0},
      {"by-gate", "DESIGNATION_GATE", wrong_gate, 0},  {"by-future", "ORDER", review_in_the_future, 0},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.kind);
    Scenario sc = Build({c.kind}, c.knobs);
    std::string kind = c.kind;
    std::string data = sc.data_a;
    switch (c.mutate) {
      case 1: sc.selection.designation = std::string(64, 'f'); break;
      case 2: Write(sc.pin, "pinned bytes, edited"); break;
      case 3: sc.selection.pins.clear(); break;
      case 4: Write(sc.data_a + "/extra.bin", "x"); break;
      case 5: kind = "not-registered"; break;
      case 6: sc.selection.version = kVersion + 1; break;
      case 7: sc.selection.designation.clear(); break;
      default: break;
    }
    const std::string why = ch::GateGuard(sc.selection, kind, data);
    EXPECT_TRUE(Has(why, "HELD [" + c.expect + "]")) << why;
    EXPECT_FALSE(Exists(MarkerOf(sc, kind))) << "a refusal is not a run";
    // a refusal is remembered by the process: asking again (CurvyCrop after CurvyRoi, say) never releases
    EXPECT_EQ(ch::GateGuard(sc.selection, kind, data), why);
  }
}

TEST(CurvySrHeld, DevPathRefusesSealedDataOnlyWhenAGateIsSelected) {
  const Scenario sc = Build({"dev-a"});
  const std::string fresh = MakeDir();
  Write(fresh + "/gt.tsv", "variant\tdev\n");
  // a named gate: the registered tree is refused even though the caller calls it dev; other data is fine
  const std::string sealed = ch::DevGuard(sc.selection, sc.data_a);
  EXPECT_TRUE(Has(sealed, "SEALED_DATA")) << sealed;
  EXPECT_EQ(ch::DevGuard(sc.selection, fresh), "");
  // a copy elsewhere is the same content: refused (by content, not by name)
  const std::string copy = MakeDir();
  Write(copy + "/gt.tsv", "variant\tregistered\n");
  EXPECT_TRUE(Has(ch::DevGuard(sc.selection, copy), "SEALED_DATA"));
  // the gate is named but its manifest is not there: fail closed
  Selection missing = sc.selection;
  missing.version = kVersion + 1;
  EXPECT_TRUE(Has(ch::DevGuard(missing, fresh), "SEALED_DATA"));
  // REPORT mode (no gate): nothing to compare with, the same sealed tree runs (the runner always selects the gate)
  EXPECT_EQ(ch::DevGuard(ch::Select("", "", "", "", ""), sc.data_a), "");
}

}  // namespace
}  // namespace slam_refact
