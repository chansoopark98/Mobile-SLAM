// FaceController event logic (US-018/US-019; docs/spec/08-face-ear.md §5 T6). Expected sequences
// follow face-controller.ts maybeEmitFaceStateUpdates / maybeWinkOrBlink* (JS semantics: undefined
// initial flags, 100 ms and 750 ms timers, state kept across facelost, cleared on detach).
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "face/face_events.h"

namespace slam_refact::face {
namespace {

FaceMsg Face(int id, FaceTrackingStatus status) {
  FaceMsg f;
  f.id = id;
  f.status = status;
  f.transform.rotation = {0.0f, 0.0f, 1.0f, 0.0f};
  f.transform.scale = 0.139f;
  f.transform.scaled_width = 1.0f;
  f.transform.scaled_height = 1.75f;
  f.transform.scaled_depth = 1.15f;
  return f;
}

FaceMsg Open(int id, FaceTrackingStatus status, bool left_eye, bool right_eye) {
  FaceMsg f = Face(id, status);
  f.left_eye_open = left_eye;
  f.right_eye_open = right_eye;
  return f;
}

FaceResponse Response(std::vector<FaceMsg> faces) {
  FaceResponse r;
  r.faces = std::move(faces);
  return r;
}

std::vector<std::string> Names(const std::vector<FaceEvent>& events) {
  std::vector<std::string> out;
  for (const auto& e : events) {
    out.push_back(e.name);
  }
  return out;
}

const std::string P = "facecontroller.";

TEST(FaceEvents, AttachEmitsLoadingThenScanning) {
  FaceEventEmitter emitter;
  ModelGeometryMsg mg;
  mg.max_detections = 1;
  mg.points_per_detection = 478;
  mg.indices = {{0, 1, 2}};
  mg.uvs = {{0.5f, 0.25f}};
  const auto events = emitter.OnAttach(mg);
  ASSERT_EQ(2u, events.size());
  EXPECT_EQ(P + "faceloading", events[0].name);
  EXPECT_EQ(P + "facescanning", events[1].name);
  EXPECT_EQ(
      "{\"maxDetections\":1,\"pointsPerDetection\":478,\"indices\":[{\"a\":0,\"b\":1,\"c\":2}],"
      "\"uvs\":[{\"u\":0.5,\"v\":0.25}]}",
      events[0].detail_json);
}

TEST(FaceEvents, NullResponseEmitsNothing) {
  FaceEventEmitter emitter;
  EXPECT_TRUE(emitter.OnProcessCpu(nullptr, 0.0).empty());
}

TEST(FaceEvents, FirstFrameEmitsEveryFlagOnce) {
  FaceEventEmitter emitter;
  const auto r = Response({Open(1, FaceTrackingStatus::kFound, true, true)});
  const auto events = emitter.OnProcessCpu(&r, 0.0);
  const std::vector<std::string> expected = {
      P + "facefound",          P + "mouthclosed",        P + "lefteyeopened",
      P + "righteyeopened",     P + "lefteyebrowlowered", P + "righteyebrowlowered",
      P + "earlost",            P + "earlost",            P + "earpointlost",
      P + "earpointlost",       P + "earpointlost",       P + "earpointlost",
      P + "earpointlost",       P + "earpointlost"};
  EXPECT_EQ(expected, Names(events));
  EXPECT_EQ("{\"id\":1,\"ear\":\"left\"}", events[6].detail_json);
  EXPECT_EQ("{\"id\":1,\"ear\":\"right\"}", events[7].detail_json);
  EXPECT_EQ("{\"id\":1,\"point\":\"leftLobe\"}", events[8].detail_json);
  EXPECT_EQ("{\"id\":1,\"point\":\"rightHelix\"}", events[13].detail_json);
  // Two 100 ms timers (eyes opened) fire without a wink/blink event.
  EXPECT_EQ(2u, emitter.pending_timers());
  EXPECT_TRUE(emitter.AdvanceTo(100.0).empty());
  EXPECT_EQ(0u, emitter.pending_timers());
  // Same flags again: only faceupdated.
  const auto r2 = Response({Open(1, FaceTrackingStatus::kUpdated, true, true)});
  EXPECT_EQ(std::vector<std::string>{P + "faceupdated"}, Names(emitter.OnProcessCpu(&r2, 133.0)));
}

TEST(FaceEvents, BothEyesClosedAndReopenedIsOneBlink) {
  FaceEventEmitter emitter;
  auto r = Response({Open(1, FaceTrackingStatus::kFound, true, true)});
  emitter.OnProcessCpu(&r, 0.0);
  r = Response({Open(1, FaceTrackingStatus::kUpdated, false, false)});
  EXPECT_EQ((std::vector<std::string>{P + "faceupdated", P + "lefteyeclosed",
                                      P + "righteyeclosed"}),
            Names(emitter.OnProcessCpu(&r, 200.0)));
  r = Response({Open(1, FaceTrackingStatus::kUpdated, true, true)});
  EXPECT_EQ((std::vector<std::string>{P + "faceupdated", P + "lefteyeopened",
                                      P + "righteyeopened"}),
            Names(emitter.OnProcessCpu(&r, 300.0)));
  const auto fired = emitter.AdvanceTo(400.0);
  ASSERT_EQ(1u, fired.size());
  EXPECT_EQ(P + "blinked", fired[0].name);
  EXPECT_EQ("{\"id\":1}", fired[0].detail_json);
  EXPECT_TRUE(emitter.AdvanceTo(2000.0).empty());
}

TEST(FaceEvents, OneEyeClosedAndReopenedIsAWink) {
  FaceEventEmitter emitter;
  auto r = Response({Open(1, FaceTrackingStatus::kFound, true, true)});
  emitter.OnProcessCpu(&r, 0.0);
  r = Response({Open(1, FaceTrackingStatus::kUpdated, false, true)});
  emitter.OnProcessCpu(&r, 200.0);
  r = Response({Open(1, FaceTrackingStatus::kUpdated, true, true)});
  emitter.OnProcessCpu(&r, 400.0);
  // The timer of the frame at 400 ms is due at 500 ms; it fires before the next frame's events.
  r = Response({Open(1, FaceTrackingStatus::kUpdated, true, true)});
  const auto events = emitter.OnProcessCpu(&r, 533.0);
  EXPECT_EQ((std::vector<std::string>{P + "lefteyewinked", P + "faceupdated"}), Names(events));
}

TEST(FaceEvents, EyeClosedLongerThan750MsIsNoWink) {
  FaceEventEmitter emitter;
  auto r = Response({Open(1, FaceTrackingStatus::kFound, true, true)});
  emitter.OnProcessCpu(&r, 0.0);
  r = Response({Open(1, FaceTrackingStatus::kUpdated, true, false)});
  emitter.OnProcessCpu(&r, 200.0);
  r = Response({Open(1, FaceTrackingStatus::kUpdated, true, true)});
  emitter.OnProcessCpu(&r, 1000.0);
  EXPECT_TRUE(emitter.AdvanceTo(5000.0).empty());
}

TEST(FaceEvents, InterpupillaryDistanceChangeOnly) {
  FaceEventEmitter emitter;
  auto f = Open(1, FaceTrackingStatus::kFound, true, true);
  auto r = Response({f});
  emitter.OnProcessCpu(&r, 0.0);
  f.status = FaceTrackingStatus::kUpdated;
  f.interpupillary_distance_in_mm = 63.5f;
  r = Response({f});
  const auto events = emitter.OnProcessCpu(&r, 33.0);
  ASSERT_EQ(2u, events.size());
  EXPECT_EQ(P + "interpupillarydistance", events[1].name);
  EXPECT_EQ("{\"interpupillaryDistance\":63.5,\"id\":1}", events[1].detail_json);
  r = Response({f});
  EXPECT_EQ(1u, emitter.OnProcessCpu(&r, 66.0).size());
}

TEST(FaceEvents, LostKeepsStateDetachClearsIt) {
  FaceEventEmitter emitter;
  auto r = Response({Open(1, FaceTrackingStatus::kFound, true, true)});
  emitter.OnProcessCpu(&r, 0.0);
  r = Response({Face(1, FaceTrackingStatus::kLost)});
  const auto lost = emitter.OnProcessCpu(&r, 1000.0);
  ASSERT_EQ(1u, lost.size());
  EXPECT_EQ(P + "facelost", lost[0].name);
  EXPECT_EQ("{\"id\":1}", lost[0].detail_json);
  r = Response({Open(1, FaceTrackingStatus::kFound, true, true)});
  EXPECT_EQ(std::vector<std::string>{P + "facefound"}, Names(emitter.OnProcessCpu(&r, 1033.0)));
  emitter.OnDetach();
  EXPECT_EQ(14u, emitter.OnProcessCpu(&r, 2000.0).size());
}

TEST(FaceEvents, EarFoundAndPointEvents) {
  FaceEventEmitter emitter;
  auto f = Open(2, FaceTrackingStatus::kFound, true, true);
  auto r = Response({f});
  emitter.OnProcessCpu(&r, 0.0);
  f.status = FaceTrackingStatus::kUpdated;
  f.left_ear_found = true;
  f.left_lobe_found = true;
  f.left_canal_found = true;
  f.left_helix_found = true;
  f.right_canal_found = true;
  r = Response({f});
  const auto events = emitter.OnProcessCpu(&r, 33.0);
  const std::vector<std::string> expected = {P + "faceupdated", P + "earfound",
                                             P + "earpointfound", P + "earpointfound",
                                             P + "earpointfound", P + "earpointfound"};
  EXPECT_EQ(expected, Names(events));
  EXPECT_EQ("{\"id\":2,\"ear\":\"left\"}", events[1].detail_json);
  EXPECT_EQ("{\"id\":2,\"point\":\"rightCanal\"}", events[5].detail_json);
}

TEST(FaceEvents, FaceDataJsonHasTheDeliveredFields) {
  FaceMsg f = Face(3, FaceTrackingStatus::kFound);
  f.vertices = {{0.25f, -0.5f, 1.0f}};
  f.normals = {{0.0f, 0.0f, -1.0f}};
  f.uvs_in_camera_frame = {{0.5f, 0.75f}};
  f.attachment_points = {{AttachmentName::FOREHEAD, {0.0f, 1.0f, 0.0f}, {}},
                         {AttachmentName::NOSE_TIP, {0.0f, 0.0f, 0.5f}, {}}};
  f.ear_attachment_points = {{AttachmentName::EAR_LEFT_LOBE, {1.0f, 0.0f, 0.0f}, {}}};
  f.mouth_open = true;
  f.left_ear_found = true;
  f.left_lobe_found = true;
  EXPECT_EQ(
      "{\"id\":3,\"transform\":{\"position\":{\"x\":0,\"y\":0,\"z\":0},\"rotation\":{\"w\":0,"
      "\"x\":0,\"y\":1,\"z\":0},\"scale\":0.138999999,\"scaledWidth\":1,\"scaledHeight\":1.75,"
      "\"scaledDepth\":1.14999998},\"vertices\":[{\"x\":0.25,\"y\":-0.5,\"z\":1}],\"normals\":[{"
      "\"x\":0,\"y\":0,\"z\":-1}],\"uvsInCameraFrame\":[{\"u\":0.5,\"v\":0.75}],"
      "\"attachmentPoints\":{\"forehead\":{\"position\":{\"x\":0,\"y\":1,\"z\":0}},\"noseTip\":{"
      "\"position\":{\"x\":0,\"y\":0,\"z\":0.5}},\"leftLobe\":{\"position\":{\"x\":1,\"y\":0,"
      "\"z\":0}}},\"mouthOpen\":true,\"leftEyeOpen\":false,\"rightEyeOpen\":false,"
      "\"leftEyebrowRaised\":false,\"rightEyebrowRaised\":false,\"interpupillaryDistanceInMM\":0,"
      "\"leftEarFound\":true,\"rightEarFound\":false}",
      FaceDataJson(f));
}

}  // namespace
}  // namespace slam_refact::face
