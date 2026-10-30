// Conformance test of sr_held.h against the shared case table sr_held_cases.json (made by
// slam_refact/tools/prereg/gen_sr_held_cases.py from independent references). The node twin is
// slam_refact/tools/prereg/sr_held_conformance.test.mjs; both read the same file and must make the same decision for every case.
//
//   sr_held_conformance <sr_held_cases.json>
//
// Environment (all optional):
//   SR_HELD_TMP         directory for the per-case work roots (default: the system temp dir); roots are kept
//   SR_HELD_DECISIONS   file that receives one "DECISION <id>#<n> <op> <OK|CODE> seed=<yes|no>" line per step, sorted
//                       (the node test writes the same lines; tools/prereg/run_sr_held_conformance.sh diffs the two)
//   SR_HELD_ONLY        run only the case with this id
// Output: one "CASE <id> PASS|FAIL <detail>" line per case, then "SUMMARY pass=N fail=M". Exit code 1 when any case fails.
// The case table is read with the parser of the header under test; the table is plain JSON, so a broken parser fails every case.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "sr_held.h"

namespace sh = slam_refact::sr_held;
namespace fs = std::filesystem;
using sh::Json;

namespace {

std::string Str(const Json* j, const char* key, const std::string& fallback = "") {
  const Json* v = j == nullptr ? nullptr : j->Get(key);
  return v != nullptr && v->type == Json::Type::kString ? v->text : fallback;
}

std::string Base64Decode(const std::string& in) {
  std::string out;
  uint32_t acc = 0;
  int bits = 0;
  for (const char c : in) {
    int v = -1;
    if (c >= 'A' && c <= 'Z') v = c - 'A';
    else if (c >= 'a' && c <= 'z') v = c - 'a' + 26;
    else if (c >= '0' && c <= '9') v = c - '0' + 52;
    else if (c == '+') v = 62;
    else if (c == '/') v = 63;
    if (v < 0) continue;
    acc = (acc << 6) | static_cast<uint32_t>(v);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<char>((acc >> bits) & 0xFF));
    }
  }
  return out;
}

std::string Body(const Json& spec) {
  const Json* text = spec.Get("text");
  if (text != nullptr) return text->text;
  return Base64Decode(Str(&spec, "b64"));
}

void WriteFile(const std::string& path, const std::string& body) {
  fs::create_directories(fs::path(path).parent_path());
  std::ofstream f(path, std::ios::binary);
  f << body;
}

std::vector<std::string> Markers(const std::string& root) {
  std::vector<std::string> names;
  std::error_code ec;
  for (const auto& e : fs::directory_iterator(root + "/prereg", ec)) {
    const std::string n = e.path().filename().string();
    if (n.size() > 7 && n.compare(n.size() - 7, 7, ".marker") == 0) names.push_back(n);
  }
  std::sort(names.begin(), names.end());
  return names;
}

struct Run {
  std::vector<std::string> decisions;
  int pass = 0;
  int fail = 0;
  void Report(const std::string& id, const std::string& problem) {
    if (problem.empty()) {
      ++pass;
      std::printf("CASE %s PASS\n", id.c_str());
    } else {
      ++fail;
      std::printf("CASE %s FAIL %s\n", id.c_str(), problem.c_str());
    }
    std::fflush(stdout);
  }
};

bool Contains(const std::string& hay, const std::string& needle) { return !needle.empty() && hay.find(needle) != std::string::npos; }

// Returns "" when the case passes, else the first deviation.
std::string RunCase(const Json& table, const Json& c, const std::string& work, Run* run) {
  const std::string id = Str(&c, "id");
  std::printf("RUN %s\n", id.c_str());  // a crash inside a case leaves the last RUN line without a result
  std::fflush(stdout);
  const Json* base = table.Get("bases")->Get(Str(&c, "base"));
  if (base == nullptr) return "unknown base";
  std::map<std::string, std::string> files;
  const Json* base_files = base->Get("files");
  for (size_t i = 0; i < base_files->keys.size(); ++i) files[base_files->keys[i]] = Body(base_files->values[i]);
  if (const Json* over = c.Get("files")) {
    for (size_t i = 0; i < over->keys.size(); ++i) {
      if (over->values[i].type == Json::Type::kNull) files.erase(over->keys[i]);
      else files[over->keys[i]] = Body(over->values[i]);
    }
  }
  const std::string root = work + "/" + id;
  fs::create_directories(root);
  for (const auto& kv : files) WriteFile(root + "/" + kv.first, kv.second);
  if (const Json* links = c.Get("symlinks")) {
    for (size_t i = 0; i < links->keys.size(); ++i) {
      fs::create_directories(fs::path(root + "/" + links->keys[i]).parent_path());
      fs::create_symlink(links->values[i].text, root + "/" + links->keys[i]);
    }
  }
  const Json* steps = c.Get("steps");
  for (size_t n = 0; n < steps->items.size(); ++n) {
    const Json& s = steps->items[n];
    const Json* over = s.Get("request");
    const Json* breq = base->Get("request");
    auto field = [&](const char* key) -> const Json* {
      const Json* v = over->Get(key);
      return v != nullptr ? v : breq->Get(key);
    };
    auto abs = [&](const std::string& p) { return p.empty() ? std::string() : root + "/" + p; };
    sh::Request rq;
    rq.prereg_dir = abs(field("prereg_dir")->text);
    rq.gate_id = field("gate_id")->text;
    rq.version = std::atoi(field("version")->text.c_str());
    rq.kind = field("kind")->text;
    rq.env_value = field("env")->text;
    for (const Json& p : field("pins")->items) rq.pins.push_back(abs(p.text));
    rq.data_dir = abs(field("data_dir")->text);
    rq.want_seed = field("want_seed")->boolean;
    rq.now = field("now")->text;

    const std::string op = Str(&s, "op");
    const std::vector<std::string> before = Markers(root);
    sh::Outcome o;
    if (op == "release") o = sh::HeldGate(rq);
    else if (op == "check") o = sh::CheckRelease(rq);
    else if (op == "audit") o = sh::Audit(rq);
    else if (op == "sealed") o = sh::RefuseSealed(rq);
    else return "unknown op " + op;
    const std::string label = id + "#" + std::to_string(n) + " " + op;
    run->decisions.push_back("DECISION " + id + "#" + std::to_string(n) + " " + op + " " + (o.ok ? "OK" : o.code) + " seed=" + (o.seed.empty() ? "no" : "yes"));

    const Json* expect = s.Get("expect");
    const bool want_ok = expect->Get("ok")->boolean;
    if (o.ok != want_ok) return label + ": ok=" + (o.ok ? "true" : "false") + " code=" + o.code + " (" + o.why + ")";
    if (o.code != Str(expect, "code")) return label + ": code " + o.code + " (" + o.why + "), expected " + Str(expect, "code");
    if (o.seed != Str(expect, "seed")) return label + ": seed differs";
    const std::vector<std::string> after = Markers(root);
    if (op == "release" && o.ok) {
      std::vector<std::string> created;
      for (const std::string& m : after) {
        if (std::find(before.begin(), before.end(), m) == before.end()) created.push_back(m);
      }
      if (created.size() != 1 || created[0] != sh::MarkerName(rq)) return label + ": exactly the marker of this kind must be created";
      const std::string marker = rq.prereg_dir + "/" + created[0];
      std::string text, err;
      Json doc;
      if (!sh::ReadFile(marker, &text) || !sh::ParseJson(text, &doc, &err)) return label + ": marker body is not JSON";
      if (Str(&doc, "gate") != rq.gate_id + ".v" + std::to_string(rq.version) || Str(&doc, "kind") != rq.kind ||
          Str(&doc, "designation_sha256") != o.designation_sha256) {
        return label + ": marker body fields";
      }
      struct stat st;
      if (::stat(marker.c_str(), &st) != 0 || (st.st_mode & 0777) != 0444) return label + ": marker mode";
      if (o.marker != marker) return label + ": marker path";
    } else {
      if (after != before) return label + ": a step that does not release must leave the markers unchanged";
      if (!o.marker.empty()) return label + ": no marker path";
    }
    std::string text = o.why + "\n" + o.code;
    for (const std::string& l : o.log) text += "\n" + l;
    if (const Json* leak = expect->Get("no_leak")) {
      for (const Json& secret : leak->items) {
        if (Contains(text, secret.text)) return label + ": the seed leaked into the reason or the log";
      }
    }
    if (op != "release" && !o.seed.empty()) return label + ": only a release hands out the seed";
  }
  return "";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <sr_held_cases.json>\n", argv[0]);
    return 2;
  }
  std::string text, err;
  Json table;
  if (!sh::ReadFile(argv[1], &text) || !sh::ParseJson(text, &table, &err)) {
    std::printf("FATAL the case table cannot be read: %s\n", err.c_str());
    return 2;
  }
  const char* tmp_env = std::getenv("SR_HELD_TMP");
  const char* only = std::getenv("SR_HELD_ONLY");
  const std::string tmp = tmp_env != nullptr ? tmp_env : fs::temp_directory_path().string();
  fs::create_directories(tmp);
  const std::string work = tmp + "/sr-held-cpp-" + std::to_string(::getpid());
  fs::create_directories(work);
  Run run;

  for (const Json& c : table.Get("cases")->items) {
    if (only != nullptr && Str(&c, "id") != only) continue;
    run.Report(Str(&c, "id"), RunCase(table, c, work, &run));
  }

  const Json& canon = *table.Get("canon_cases");
  for (const Json& c : canon.items) {
    const std::string id = "canon-" + Str(&c, "name");
    if (only != nullptr && id != only) continue;
    Json v;
    std::string perr, got = "ERR";
    if (sh::ParseJson(Body(*c.Get("input")), &v, &perr)) {
      std::string t;
      if (sh::CanonManifestText(v, &t)) got = t;
    }
    run.decisions.push_back("DECISION " + id + " canon " + (got == "ERR" ? "ERR" : "OK") + " seed=no");
    std::string problem;
    if (got != Str(&c, "expect_text")) problem = "canonical text differs: " + got.substr(0, 200);
    if (problem.empty() && c.Get("expect_sha256") != nullptr && slam_refact::testutil::Sha256Hex(got) != Str(&c, "expect_sha256")) problem = "canonical sha256 differs";
    run.Report(id, problem);
  }

  for (const Json& m : table.Get("canon_manifests")->items) {
    const std::string id = "canon-manifest-" + Str(&m, "base");
    if (only != nullptr && id != only) continue;
    Json v;
    std::string perr, sha, problem;
    const Json* spec = table.Get("bases")->Get(Str(&m, "base"))->Get("files")->Get(Str(&m, "file"));
    if (!sh::ParseJson(Body(*spec), &v, &perr) || !sh::CanonManifestSha(v, &sha)) problem = "manifest cannot be canonicalised";
    else if (sha != Str(&m, "expect_sha256")) problem = "canonical sha256 differs: " + sha;
    run.Report(id, problem);
  }

  const Json& instants = *table.Get("instant_cases");
  for (size_t n = 0; n < instants.items.size(); ++n) {
    const Json& c = instants.items[n];
    const std::string id = "instant-" + std::to_string(n);
    if (only != nullptr && id != only) continue;
    sh::Instant t;
    const bool ok = sh::ParseInstant(Str(&c, "text"), &t);
    run.decisions.push_back("DECISION " + id + " instant " + (ok ? "OK" : "INVALID") + " seed=no");
    std::string problem;
    if (c.Get("valid")->boolean != ok) {
      problem = std::string("\"") + Str(&c, "text") + "\" must be " + (c.Get("valid")->boolean ? "accepted" : "refused");
    } else if (ok && (std::to_string(t.sec) != c.Get("sec")->text || std::to_string(t.ns) != c.Get("ns")->text)) {
      problem = "instant value differs: " + std::to_string(t.sec) + "." + std::to_string(t.ns);
    }
    run.Report(id, problem);
  }

  if (only == nullptr || std::string(only) == "tree-probe") {
    const Json& probe = *table.Get("tree_probe");
    const std::string dir = work + "/tree-probe";
    const Json* pf = probe.Get("files");
    for (size_t i = 0; i < pf->keys.size(); ++i) WriteFile(dir + "/" + pf->keys[i], Body(pf->values[i]));
    std::string sha, code;
    const bool ok = sh::TreeSha(dir, &sha, &code);
    run.Report("tree-probe", ok && sha == Str(&probe, "expect_sha256") ? "" : "tree sha256 differs: " + sha + " " + code);
  }

  if (const char* path = std::getenv("SR_HELD_DECISIONS")) {
    std::sort(run.decisions.begin(), run.decisions.end());
    std::ofstream f(path);
    for (const std::string& d : run.decisions) f << d << "\n";
  }
  std::printf("SUMMARY pass=%d fail=%d\n", run.pass, run.fail);
  return run.fail == 0 ? 0 : 1;
}
