// slam_refact_orient_replay: exact-input replay of a run-orient.mjs recording of the ORIGINAL SDK through GlueRunner (US-023
// REPORT driver, not a gate; the replay itself is orient_replay.h). Output: <out>/trace.jsonl in the schema of the original trace
// (i, t, fresh, status, p, q_wxyz, worldPoints; positions and quaternions as shortest round-trip float digits).
//
//   slam_refact_orient_replay --rec <recording dir with tsv/> --out <new dir> [--rotation R]
// --rotation R replays with another engineInit rotation than the recording (a sensitivity run).

#include <sys/stat.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "orient_replay.h"

int main(int argc, char** argv) {
  std::string rec, out;
  slam_refact::orient_replay::Options opt;
  for (int k = 1; k + 1 < argc; k += 2) {
    const std::string a = argv[k];
    if (a == "--rec") rec = argv[k + 1];
    else if (a == "--out") out = argv[k + 1];
    else if (a == "--rotation") opt.rotation_override = std::atoi(argv[k + 1]);
    else { std::fprintf(stderr, "unknown argument %s\n", a.c_str()); return 2; }
  }
  if (rec.empty() || out.empty()) { std::fprintf(stderr, "usage: --rec <dir> --out <new dir> [--rotation R]\n"); return 2; }
  const slam_refact::orient_replay::Outcome o = slam_refact::orient_replay::Replay(rec, opt);
  if (!o.ok) { std::fprintf(stderr, "replay failed: %s\n", o.error.c_str()); return 2; }
  if (::mkdir(out.c_str(), 0775) != 0) { std::fprintf(stderr, "cannot create %s (a new run directory is required)\n", out.c_str()); return 2; }
  std::ofstream trace(out + "/trace.jsonl");
  int fresh = 0;
  for (const auto& t : o.rows) {
    fresh += t.fresh;
    char buf[400];
    std::snprintf(buf, sizeof(buf), "{\"i\":%d,\"fresh\":%s,\"status\":\"%s\",\"p\":[%.9g,%.9g,%.9g],\"q_wxyz\":[%.9g,%.9g,%.9g,%.9g],\"worldPoints\":%d}\n",
                  t.frame, t.fresh ? "true" : "false", t.status.c_str(), t.p[0], t.p[1], t.p[2], t.q[0], t.q[1], t.q[2], t.q[3], t.world_points);
    trace << buf;
  }
  std::ofstream meta(out + "/meta.json");
  meta << "{\"rec\":\"" << rec << "\",\"rotation\":" << o.rotation << ",\"capture\":[" << o.width << "," << o.height << "],\"rows\":" << o.rows.size()
       << ",\"fresh\":" << fresh << ",\"skippedSamples\":" << o.skipped_samples << ",\"manufacturer\":\"" << o.manufacturer << "\",\"model\":\"" << o.model << "\"}\n";
  std::printf("rows=%zu fresh=%d skipped_samples=%d out=%s\n", o.rows.size(), fresh, o.skipped_samples, out.c_str());
  return 0;
}
