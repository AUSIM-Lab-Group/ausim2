#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "ausim_msg/create_aabb.hpp"
#include "ausim_msg/msg/detection2_d_array.hpp"
#include "ausim_msg/msg/detection3_d_array.hpp"
#include "ausim_msg/msg/label_info.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void ExpectNear(double actual, double expected, double tolerance, const std::string& label) {
  if (std::fabs(actual - expected) > tolerance) {
    std::cerr << label << ": expected " << expected << ", got " << actual << '\n';
    std::exit(1);
  }
}

void TestCreateAABB2D() {
  const auto bbox = ausim_msg::createAABB2D(10, 20, 30, 40);
  ExpectNear(bbox.center.position.x, 25.0, 1e-9, "bbox2d.center.position.x");
  ExpectNear(bbox.center.position.y, 40.0, 1e-9, "bbox2d.center.position.y");
  ExpectNear(bbox.size_x, 30.0, 1e-9, "bbox2d.size_x");
  ExpectNear(bbox.size_y, 40.0, 1e-9, "bbox2d.size_y");
  ExpectNear(bbox.center.theta, 0.0, 1e-9, "bbox2d.center.theta");
}

void TestCreateAABB3D() {
  const auto bbox = ausim_msg::createAABB3D(1, 2, 3, 4, 5, 6);
  ExpectNear(bbox.center.position.x, 3.0, 1e-9, "bbox3d.center.position.x");
  ExpectNear(bbox.center.position.y, 4.5, 1e-9, "bbox3d.center.position.y");
  ExpectNear(bbox.center.position.z, 6.0, 1e-9, "bbox3d.center.position.z");
  ExpectNear(bbox.center.orientation.w, 1.0, 1e-9, "bbox3d.center.orientation.w");
  ExpectNear(bbox.size.x, 4.0, 1e-9, "bbox3d.size.x");
  ExpectNear(bbox.size.y, 5.0, 1e-9, "bbox3d.size.y");
  ExpectNear(bbox.size.z, 6.0, 1e-9, "bbox3d.size.z");
}

void TestNestedDetectionMessages() {
  ausim_msg::msg::Detection2DArray detections_2d;
  detections_2d.header.frame_id = "camera_link";

  ausim_msg::msg::Detection2D detection_2d;
  detection_2d.id = "det-2d";
  detection_2d.bbox = ausim_msg::createAABB2D(5, 6, 7, 8);
  detection_2d.results.resize(1);
  detection_2d.results[0].hypothesis.class_id = "target";
  detection_2d.results[0].hypothesis.score = 0.95;
  detection_2d.results[0].pose.pose.orientation.w = 1.0;
  detections_2d.detections.push_back(detection_2d);

  Expect(detections_2d.detections.size() == 1, "expected one 2D detection");
  Expect(detections_2d.detections[0].results.size() == 1, "expected one 2D hypothesis");
  Expect(detections_2d.detections[0].results[0].hypothesis.class_id == "target", "expected nested 2D class id");

  ausim_msg::msg::Detection3DArray detections_3d;
  detections_3d.header.frame_id = "map";

  ausim_msg::msg::Detection3D detection_3d;
  detection_3d.id = "det-3d";
  detection_3d.results.resize(1);
  detection_3d.results[0].hypothesis.class_id = "box";
  detection_3d.results[0].hypothesis.score = 0.9;
  detection_3d.results[0].pose.pose.orientation.w = 1.0;
  detection_3d.bbox = ausim_msg::createAABB3D(1, 2, 3, 4, 5, 6);
  detections_3d.detections.push_back(detection_3d);

  Expect(detections_3d.detections.size() == 1, "expected one 3D detection");
  ExpectNear(detections_3d.detections[0].bbox.center.position.z, 6.0, 1e-9, "expected nested 3D bbox center");
}

void TestLabelInfo() {
  ausim_msg::msg::LabelInfo info;
  info.header.frame_id = "vision_pipeline";
  info.threshold = 0.35f;

  ausim_msg::msg::VisionClass label;
  label.class_id = 7;
  label.class_name = "person";
  info.class_map.push_back(label);

  Expect(info.class_map.size() == 1, "expected one label");
  Expect(info.class_map[0].class_name == "person", "expected label name");
  ExpectNear(info.threshold, 0.35, 1e-6, "expected threshold value");
}

}  // namespace

int main() {
  TestCreateAABB2D();
  TestCreateAABB3D();
  TestNestedDetectionMessages();
  TestLabelInfo();
  return 0;
}
