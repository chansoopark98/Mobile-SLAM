// Coordinate rewrite of face responses (orientation.cc FaceResponse/FrameworkResponse overloads)
// and the face model helpers (US-018/US-019).
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "face/face_controller.h"
#include "face/face_coordinates.h"
#include "face/face_models.h"

namespace slam_refact::face {
namespace {

FaceResponse OneFace() {
  FaceResponse r;
  r.camera.extrinsic_rotation = {1.0f, 0.0f, 0.0f, 0.0f};
  FaceMsg f;
  f.id = 1;
  f.status = FaceTrackingStatus::kFound;
  f.transform.position = {0.1f, -0.2f, 0.5f};
  f.transform.rotation = {0.0f, 0.0f, 1.0f, 0.0f};
  f.transform.scale = 0.139f;
  f.vertices = {{1.0f, 2.0f, 3.0f}};
  f.normals = {{0.0f, 0.0f, -1.0f}};
  f.attachment_points = {{AttachmentName::NOSE_TIP, {0.5f, 0.25f, 0.125f}, {0.5f, 0.5f, 0.5f, 0.5f}}};
  f.ear_vertices = {{-1.0f, 0.0f, 2.0f}};
  r.faces.push_back(f);
  return r;
}

CameraCoordinates Identity(Axes axes, bool mirrored) {
  CameraCoordinates c;
  c.origin_rotation = {1.0f, 0.0f, 0.0f, 0.0f};
  c.scale = 1.0f;
  c.axes = axes;
  c.mirrored_display = mirrored;
  return c;
}

TEST(FaceCoordinates, LeftHandedIdentityOriginKeepsValues) {
  auto r = OneFace();
  RewriteCoordinateSystemPostprocess(Identity(Axes::kLeftHanded, false), &r);
  EXPECT_FLOAT_EQ(0.1f, r.faces[0].transform.position.x);
  EXPECT_FLOAT_EQ(-0.2f, r.faces[0].transform.position.y);
  EXPECT_FLOAT_EQ(0.5f, r.faces[0].transform.position.z);
  EXPECT_FLOAT_EQ(1.0f, r.faces[0].transform.rotation.y);
  EXPECT_FLOAT_EQ(3.0f, r.faces[0].vertices[0].z);
  EXPECT_FLOAT_EQ(0.139f, r.faces[0].transform.scale);
}

TEST(FaceCoordinates, RightHandedNegatesZAndQuaternionWz) {
  auto r = OneFace();
  RewriteCoordinateSystemPostprocess(Identity(Axes::kRightHanded, false), &r);
  const auto& f = r.faces[0];
  EXPECT_FLOAT_EQ(-0.5f, f.transform.position.z);
  EXPECT_FLOAT_EQ(-3.0f, f.vertices[0].z);
  EXPECT_FLOAT_EQ(1.0f, f.normals[0].z);
  EXPECT_FLOAT_EQ(-0.125f, f.attachment_points[0].position.z);
  EXPECT_FLOAT_EQ(-0.5f, f.attachment_points[0].rotation.w);
  EXPECT_FLOAT_EQ(-0.5f, f.attachment_points[0].rotation.z);
  EXPECT_FLOAT_EQ(-2.0f, f.ear_vertices[0].z);
  // Camera rotation (1, 0, 0, 0) -> (-1, 0, 0, 0).
  EXPECT_FLOAT_EQ(-1.0f, r.camera.extrinsic_rotation.w);
}

TEST(FaceCoordinates, MirrorNegatesXAndQuaternionYz) {
  auto r = OneFace();
  RewriteCoordinateSystemPostprocess(Identity(Axes::kLeftHanded, true), &r);
  const auto& f = r.faces[0];
  EXPECT_FLOAT_EQ(-0.1f, f.transform.position.x);
  EXPECT_FLOAT_EQ(-1.0f, f.transform.rotation.y);
  EXPECT_FLOAT_EQ(-1.0f, f.vertices[0].x);
  EXPECT_FLOAT_EQ(1.0f, f.ear_vertices[0].x);
  EXPECT_FLOAT_EQ(-0.5f, f.attachment_points[0].rotation.y);
}

TEST(FaceCoordinates, OriginAndScaleMoveTheFaceAndCamera) {
  auto r = OneFace();
  CameraCoordinates c = Identity(Axes::kLeftHanded, false);
  c.origin_position = {0.0f, 2.0f, 0.0f};
  c.scale = 2.0f;
  RewriteCoordinateSystemPostprocess(c, &r);
  EXPECT_FLOAT_EQ(0.0f, r.camera.extrinsic_position.x);
  EXPECT_FLOAT_EQ(2.0f, r.camera.extrinsic_position.y);
  EXPECT_FLOAT_EQ(0.2f, r.faces[0].transform.position.x);
  EXPECT_FLOAT_EQ(1.6f, r.faces[0].transform.position.y);
  EXPECT_FLOAT_EQ(1.0f, r.faces[0].transform.position.z);
  EXPECT_FLOAT_EQ(0.278f, r.faces[0].transform.scale);
  // Mesh vertices stay in local space.
  EXPECT_FLOAT_EQ(1.0f, r.faces[0].vertices[0].x);
}

TEST(FaceCoordinates, FrameworkWindingAndUvs) {
  ModelGeometryMsg m;
  m.indices = {{0, 1, 2}};
  m.uvs = {{0.25f, 0.5f}};
  auto right = m;
  RewriteCoordinateSystemFrameworkPostprocess(Identity(Axes::kRightHanded, false), &right);
  EXPECT_EQ(2, right.indices[0].b);
  EXPECT_EQ(1, right.indices[0].c);
  EXPECT_FLOAT_EQ(0.25f, right.uvs[0].u);
  auto both = m;
  RewriteCoordinateSystemFrameworkPostprocess(Identity(Axes::kRightHanded, true), &both);
  EXPECT_EQ(1, both.indices[0].b);  // swapped twice
  EXPECT_FLOAT_EQ(0.75f, both.uvs[0].u);
}

TEST(FaceModels, Sha256KnownVectors) {
  EXPECT_EQ("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
            Sha256Hex(std::vector<uint8_t>{}));
  const std::string abc = "abc";
  EXPECT_EQ("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
            Sha256Hex(reinterpret_cast<const uint8_t*>(abc.data()), abc.size()));
  const std::string two_blocks = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  EXPECT_EQ("248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1",
            Sha256Hex(reinterpret_cast<const uint8_t*>(two_blocks.data()), two_blocks.size()));
}

TEST(FaceController, DefaultsMatchTheJsSide) {
  const auto o = DefaultFaceOptions();
  EXPECT_FLOAT_EQ(0.01f, o.near_clip);
  EXPECT_FLOAT_EQ(1000.0f, o.far_clip);
  EXPECT_EQ(1, o.max_detections);
  EXPECT_TRUE(o.use_standard_uvs);
  EXPECT_EQ(Axes::kRightHanded, o.coordinates.axes);
  ASSERT_EQ(1u, o.mesh_geometry.size());
  EXPECT_EQ(MeshGeometryOption::kFace, o.mesh_geometry[0]);
  EXPECT_FALSE(o.enable_ears);
  const auto k = DefaultDeviceIntrinsics();
  EXPECT_EQ(480, k.pixelsWidth);
  EXPECT_FLOAT_EQ(506.4104f, k.focalLengthHorizontal);
}

TEST(FaceController, RejectsBytesThatAreNotTheDeployedModel) {
  FaceController controller;
  std::string error;
  EXPECT_FALSE(controller.LoadFaceMeshModel(std::vector<uint8_t>{1, 2, 3}, &error));
  EXPECT_NE(std::string::npos, error.find("sha256"));
}

TEST(FaceController, FrameworkDataForFaceMeshGeometry) {
  FaceController controller;
  auto o = DefaultFaceOptions();
  o.coordinates.axes = Axes::kLeftHanded;
  controller.UpdateFaceOptions(o);
  const auto& m = controller.GetFrameworkData();
  EXPECT_EQ(478, m.points_per_detection);
  EXPECT_EQ(1, m.max_detections);
  EXPECT_EQ(478u, m.uvs.size());
  // FACEMESH_INDICES[0, FACEMESH_EYE_INDICES_START) = 852 triangles for meshGeometry [FACE].
  EXPECT_EQ(852u, m.indices.size());
}

}  // namespace
}  // namespace slam_refact::face
