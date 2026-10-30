// Unit tests of the replay bundle reader and trace writer (WP5). Synthetic files only.

#include <gtest/gtest.h>
#include <unistd.h>
#include <zlib.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "bundle_reader.h"
#include "trace_writer.h"

namespace slam_refact::harness {
namespace {

std::string MakeTempDir() {
  char tmpl[] = "/tmp/sr_replay_io_test_XXXXXX";
  const char* dir = mkdtemp(tmpl);
  EXPECT_NE(dir, nullptr);
  return dir == nullptr ? std::string() : std::string(dir);
}

void WriteText(const std::string& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary);
  out << text;
}

void RemoveDir(const std::string& dir, std::initializer_list<const char*> files) {
  for (const char* f : files) std::remove((dir + "/" + f).c_str());
  rmdir(dir.c_str());
}

std::string ReadText(const std::string& path) {
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void WritePgm(const std::string& path, int w, int h, uint8_t base) {
  std::string data = "P5\n# comment\n" + std::to_string(w) + " " + std::to_string(h) + "\n255\n";
  for (int k = 0; k < w * h; ++k) data.push_back(static_cast<char>(base + k));
  WriteText(path, data);
}

void WriteGzipAtlas(const std::string& path, int rows) {
  gzFile f = gzopen(path.c_str(), "wb");
  ASSERT_NE(f, nullptr);
  std::vector<uint8_t> rgba(static_cast<size_t>(1024 * 4 * rows));
  for (size_t k = 0; k < rgba.size(); ++k) rgba[k] = static_cast<uint8_t>(k * 7);
  ASSERT_EQ(gzwrite(f, rgba.data(), static_cast<unsigned>(rgba.size())), static_cast<int>(rgba.size()));
  gzclose(f);
}

class BundleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = MakeTempDir();
    WritePgm(dir_ + "/f0.pgm", 4, 6, 10);
    WritePgm(dir_ + "/f1.pgm", 4, 6, 20);
    WriteGzipAtlas(dir_ + "/cap-000.rgba.gz", 3);
    WriteText(dir_ + "/engine-config.tsv",
              "scenario\tsynthetic\ncapture_width\t4\ncapture_height\t6\nrotation_deg\t0\n"
              "manufacturer\t\nmodel\tK\nestimate_scale\t1\ndisable_vio\t0\n"
              "pyramid_model\tswiftshader-approx\norigin\t0 2 0\nfacing_wxyz\t1 0 0 0\n"
              "left_handed_axes\t0\nmirrored_display\t0\nworld_point_sample_every\t50\n");
    WriteText(dir_ + "/sensors.tsv",
              "bootstrap\t986955000\t-0.1\t8.700000000000001\t4.4\t-\t-\t-\t4.1\t6.2\t4.0\t24.03\t359.5\t63.25\t1.25\n"
              "run\t1000000000\t-0.1\t8.700000000000001\t4.4\t0.25\t-0.5\t0.125\t4.1\t6.2\t4.0\t24.03\t359.5\t63.25\t1.25\n");
    WriteText(dir_ + "/frames.tsv",
              "bootstrap\t-1\t986955000\t986955000\t-0.013045\t1\t" + dir_ + "/f0.pgm\t" + dir_ +
                  "/cap-000.rgba.gz\n"
                  "frame\t0\t1002955000\t1002955000\t0.002955\t1\t" + dir_ + "/f0.pgm\t-\n"
                  "frame\t1\t1043745000\t1043745000\t0.043745\t0\t" + dir_ + "/f1.pgm\t-\n");
  }
  void TearDown() override {
    RemoveDir(dir_, {"f0.pgm", "f1.pgm", "cap-000.rgba.gz", "engine-config.tsv", "sensors.tsv", "frames.tsv"});
  }
  std::string dir_;
};

TEST_F(BundleTest, LoadsSensorsFramesAndConfig) {
  ReplayBundle b;
  std::string error;
  ASSERT_TRUE(LoadBundle(dir_, &b, &error)) << error;
  ASSERT_EQ(b.sensors.size(), 2u);
  EXPECT_TRUE(b.sensors[0].bootstrap);
  EXPECT_FALSE(b.sensors[0].motion.acceleration.has_value());
  ASSERT_TRUE(b.sensors[1].motion.acceleration.has_value());
  EXPECT_EQ(b.sensors[1].motion.acceleration->y, -0.5f);
  EXPECT_EQ(b.sensors[1].motion.receive_time_ns, 1000000000);
  EXPECT_EQ(b.sensors[1].orientation.receive_time_ns, 1000000000);
  EXPECT_EQ(b.sensors[1].motion.acceleration_including_gravity->y, static_cast<float>(8.700000000000001));
  EXPECT_EQ(b.sensors[1].motion.rotation_rate_abg->z, 4.0f);
  EXPECT_EQ(b.sensors[1].motion.interval_ms, 24.03);
  EXPECT_EQ(b.sensors[1].orientation.beta_deg, 63.25);
  ASSERT_EQ(b.frames.size(), 3u);
  EXPECT_TRUE(b.frames[0].bootstrap);
  EXPECT_EQ(b.frames[2].index, 1);
  EXPECT_FALSE(b.frames[2].process);
  EXPECT_EQ(b.frames[1].t, 0.002955);

  const EngineConfig ec = b.MakeEngineConfig();
  EXPECT_TRUE(ec.estimate_scale);
  EXPECT_FALSE(ec.disable_vio);
  EXPECT_EQ(ec.capture_width, 4);
  EXPECT_EQ(ec.capture_height, 6);
  EXPECT_EQ(ec.model, "K");
  EXPECT_EQ(ec.manufacturer, "");
  EXPECT_EQ(ec.pyramid_model, ShaderNumericModel::kSwiftShaderApprox);
  const GlueConfig gc = b.MakeGlueConfig();
  EXPECT_EQ(gc.origin.y, 2.f);
  EXPECT_EQ(gc.facing.w, 1.f);
}

TEST_F(BundleTest, LoadsImagesAndBuildsFrameInput) {
  ReplayBundle b;
  std::string error;
  ASSERT_TRUE(LoadBundle(dir_, &b, &error)) << error;
  LoadedImages images;
  ASSERT_TRUE(LoadImages(b.frames[0], &images, &error)) << error;
  EXPECT_EQ(images.width, 4);
  EXPECT_EQ(images.height, 6);
  EXPECT_EQ(images.gray[5], 15);
  EXPECT_EQ(images.atlas_rows, 3);
  EXPECT_EQ(images.atlas[9], static_cast<uint8_t>(63));
  const FrameInput in = MakeFrameInput(b.frames[0], images);
  EXPECT_EQ(in.time_ns, 986955000);
  EXPECT_EQ(in.gray.stride, 4);
  EXPECT_EQ(in.pyramid.size, 1024);
  EXPECT_EQ(in.pyramid.rows, 3);
  EXPECT_TRUE(in.imu_events.empty());
  ASSERT_TRUE(LoadImages(b.frames[1], &images, &error)) << error;
  EXPECT_TRUE(images.atlas.empty());
  EXPECT_EQ(MakeFrameInput(b.frames[1], images).pyramid.rgba, nullptr);
}

// Optional column 9 (frame_time_ns, 647 frameTimestampNanos): tracking time when non-zero; the
// binding time column 3 is kept.
TEST_F(BundleTest, FrameTimeColumnDrivesFrameInputTime) {
  WriteText(dir_ + "/frames.tsv",
            "bootstrap\t-1\t986955000\t986955000\t-0.013045\t1\t" + dir_ + "/f0.pgm\t-\t0\n"
            "frame\t0\t1002955000\t1002955000\t0.002955\t1\t" + dir_ + "/f0.pgm\t-\t1000100000\n");
  ReplayBundle b;
  std::string error;
  ASSERT_TRUE(LoadBundle(dir_, &b, &error)) << error;
  ASSERT_EQ(b.frames.size(), 2u);
  EXPECT_EQ(b.frames[1].time_ns, 1002955000);
  EXPECT_EQ(b.frames[1].frame_time_ns, 1000100000);
  LoadedImages images;
  ASSERT_TRUE(LoadImages(b.frames[1], &images, &error)) << error;
  EXPECT_EQ(MakeFrameInput(b.frames[1], images).time_ns, 1000100000);
  EXPECT_EQ(MakeFrameInput(b.frames[0], images).time_ns, 986955000);  // 0 -> time_ns
}

TEST_F(BundleTest, RejectsMalformedRows) {
  WriteText(dir_ + "/sensors.tsv", "run\t1000\t1\t2\n");
  ReplayBundle b;
  std::string error;
  EXPECT_FALSE(LoadBundle(dir_, &b, &error));
  EXPECT_NE(error.find("sensors.tsv"), std::string::npos);
}

TEST(JsNumberTest, MatchesJavaScriptNumberToString) {
  EXPECT_EQ(JsNumber(0.1), "0.1");
  EXPECT_EQ(JsNumber(2.0), "2");
  EXPECT_EQ(JsNumber(-0.0), "0");
  EXPECT_EQ(JsNumber(static_cast<double>(2.006481647491455f)), "2.006481647491455");
  EXPECT_EQ(JsNumber(-0.00027345443959347904), "-0.00027345443959347904");
  EXPECT_EQ(JsNumber(1e-7), "1e-7");
  EXPECT_EQ(JsNumber(1.5e-7), "1.5e-7");
  EXPECT_EQ(JsNumber(0.000001), "0.000001");
  EXPECT_EQ(JsNumber(1e21), "1e+21");
  EXPECT_EQ(JsNumber(123456789012345680000.0), "123456789012345680000");
  EXPECT_EQ(JsNumber(8.700000000000001), "8.700000000000001");
  EXPECT_EQ(JsNumber(std::numeric_limits<double>::quiet_NaN()), "null");
  EXPECT_EQ(JsFixed(0.123456789, 4), "0.1235");
  EXPECT_EQ(JsFixed(2.0, 5), "2");
  // Exact ties round away from zero like Number.prototype.toFixed (printf would give 0.1562 / 0.12).
  EXPECT_EQ(JsFixed(0.15625, 4), "0.1563");
  EXPECT_EQ(JsFixed(-0.15625, 4), "-0.1563");
  EXPECT_EQ(JsFixed(0.125, 2), "0.13");
  EXPECT_EQ(JsFixed(0.1, 0), "0");
  EXPECT_EQ(JsFixed(0.15624, 4), "0.1562");
}

TEST(TraceWriterTest, WritesReferenceSchemaAndWorldPointChurn) {
  const std::string dir = MakeTempDir();
  TraceWriter w;
  std::string error;
  ASSERT_TRUE(w.Open(dir, 50, &error)) << error;
  FrameRow f0;
  f0.index = 0;
  f0.t = 0.002955;
  f0.video_time_ns = 1002955000;
  PublishedFrame out;
  out.position = Vec3f{0.f, 2.f, 0.f};
  out.rotation = QuatWxyz{1.f, 0.f, 0.f, 0.f};
  out.status = TrackingStatus::kNormal;
  out.reason = TrackingReason::kUnspecified;
  out.world_points = {{7, {1.f, 2.f, 3.f}, 0.5f}, {0, {0.f, 2.f, 0.f}, 0.f}, {0, {0.f, 2.f, 0.f}, 0.f}};
  w.WriteProcessed(f0, out);
  FrameRow f1 = f0;
  f1.index = 1;
  f1.t = 0.043745;
  w.WriteDropped(f1);
  FrameRow f2 = f0;
  f2.index = 2;
  out.repeated = true;
  out.world_points = {{8, {1.f, 2.f, 3.f}, 0.25f}};
  w.WriteProcessed(f2, out);
  ASSERT_TRUE(w.Close(&error)) << error;
  const std::string trace = ReadText(dir + "/trace.jsonl");
  const std::string first = trace.substr(0, trace.find('\n'));
  EXPECT_EQ(first.rfind(
                "{\"i\":0,\"t\":0.002955,\"videoTimeUs\":1002955,\"publishMs\":null,\"acceptedMs\":null,"
                "\"fresh\":true,\"callbackMs\":null,\"latencyMs\":null,\"status\":\"NORMAL\","
                "\"reason\":\"UNSPECIFIED\",\"p\":[0,2,0],\"q_wxyz\":[1,0,0,0],\"worldPoints\":3,"
                "\"wpAdded\":3,\"wpRemoved\":0,\"wpMeanConfidence\":0.16666666666666666,"
                "\"xrGpuStageMs\":null,\"xrCpuStageMs\":null,\"srRepeated\":false,",
                0),
            0u)
      << first;
  EXPECT_NE(trace.find("{\"i\":1,\"t\":0.043745,\"videoTimeUs\":1002955,\"publishMs\":null,"
                       "\"acceptedMs\":null,\"fresh\":false,\"callbackMs\":null,\"latencyMs\":null,"
                       "\"status\":null,\"reason\":null,\"p\":null,\"q_wxyz\":null,\"worldPoints\":null,"
                       "\"wpAdded\":null,\"wpRemoved\":null,\"wpMeanConfidence\":null,"
                       "\"xrGpuStageMs\":null,\"xrCpuStageMs\":null}"),
            std::string::npos)
      << trace;
  // Frame 2: ids {8} vs previous {7, 0}: added 1, removed 2 (page-runtime.js loop semantics).
  EXPECT_NE(trace.find("\"i\":2,"), std::string::npos);
  EXPECT_NE(trace.find("\"worldPoints\":1,\"wpAdded\":1,\"wpRemoved\":2,\"wpMeanConfidence\":0.25"),
            std::string::npos)
      << trace;
  EXPECT_NE(trace.find("\"srRepeated\":true"), std::string::npos);
  EXPECT_NE(trace.find("\"srMapTrackerState\":null"), std::string::npos);
  const std::string samples = ReadText(dir + "/worldpoints-sampled.jsonl");
  EXPECT_EQ(samples,
            "{\"frameIndex\":0,\"videoTime\":1.002955,\"points\":[[7,0.5,1,2,3],[0,0,0,2,0],[0,0,0,2,0]]}\n");
  // Existing run files are never overwritten.
  {
    TraceWriter again;
    EXPECT_FALSE(again.Open(dir, 50, &error));
  }
  RemoveDir(dir, {"trace.jsonl", "worldpoints-sampled.jsonl"});
}

}  // namespace
}  // namespace slam_refact::harness
