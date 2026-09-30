// Copyright 2026 CorridorCourier contributors. Apache-2.0.
#include "cc_costmap_layers/perception_layer.hpp"

#include <algorithm>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cc_costmap_layers
{

ClassParams PerceptionLayer::loadClassParams(const std::string & cls, const ClassParams & d)
{
  auto node = node_.lock();
  const std::string p = name_ + "." + cls + ".";
  ClassParams out;
  declareParameter(cls + ".enabled", rclcpp::ParameterValue(d.enabled));
  declareParameter(cls + ".inflation_radius", rclcpp::ParameterValue(d.inflation_radius));
  declareParameter(cls + ".cost_scaling_factor", rclcpp::ParameterValue(d.cost_scaling_factor));
  declareParameter(cls + ".persistence", rclcpp::ParameterValue(d.persistence));
  declareParameter(cls + ".min_hits", rclcpp::ParameterValue(d.min_hits));
  declareParameter(cls + ".association_gate", rclcpp::ParameterValue(d.association_gate));
  node->get_parameter(p + "enabled", out.enabled);
  node->get_parameter(p + "inflation_radius", out.inflation_radius);
  node->get_parameter(p + "cost_scaling_factor", out.cost_scaling_factor);
  node->get_parameter(p + "persistence", out.persistence);
  node->get_parameter(p + "min_hits", out.min_hits);
  node->get_parameter(p + "association_gate", out.association_gate);
  return out;
}

void PerceptionLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PerceptionLayer: failed to lock node");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  global_frame_ = layered_costmap_->getGlobalFrameID();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("detections_topic", rclcpp::ParameterValue(std::string("/perception/detections")));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));
  declareParameter("min_score", rclcpp::ParameterValue(0.4));
  std::string topic;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".detections_topic", topic);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".min_score", min_score_);

  const CostModel defaults;
  for (auto cls : {ObjectClass::kPerson, ObjectClass::kCart, ObjectClass::kLowObstacle,
      ObjectClass::kGlass, ObjectClass::kUnknown})
  {
    model_.setClassParams(cls, loadClassParams(classToString(cls), defaults.classParams(cls)));
  }
  onFootprintChanged();

  sub_ = node->create_subscription<vision_msgs::msg::Detection3DArray>(
    topic, rclcpp::SensorDataQoS(),
    std::bind(&PerceptionLayer::detectionsCallback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(
    logger_, "PerceptionLayer '%s' on %s, person inflation %.2f m",
    name_.c_str(), topic.c_str(), model_.classParams(ObjectClass::kPerson).inflation_radius);
}

void PerceptionLayer::onFootprintChanged()
{
  std::lock_guard<std::mutex> lock(mutex_);
  model_.setInscribedRadius(layered_costmap_->getInscribedRadius());
}

void PerceptionLayer::detectionsCallback(vision_msgs::msg::Detection3DArray::ConstSharedPtr msg)
{
  if (!enabled_) {
    return;
  }
  const double stamp = rclcpp::Time(msg->header.stamp, clock_->get_clock_type()).seconds();
  std::vector<Observation> batch;
  batch.reserve(msg->detections.size());
  for (const auto & det : msg->detections) {
    if (det.results.empty()) {continue;}
    const auto best = std::max_element(
      det.results.begin(), det.results.end(),
      [](const auto & a, const auto & b) {return a.hypothesis.score < b.hypothesis.score;});
    if (best->hypothesis.score < min_score_) {continue;}

    geometry_msgs::msg::PoseStamped in, out;
    in.header = msg->header;
    in.pose = det.bbox.center;
    try {
      // Transform at the capture time so latency does not smear objects.
      tf_->transform(in, out, global_frame_, tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Detection dropped: %s", ex.what());
      continue;
    }
    Observation o;
    o.cls = classFromString(best->hypothesis.class_id);
    o.x = out.pose.position.x;
    o.y = out.pose.position.y;
    o.yaw = tf2::getYaw(out.pose.orientation);
    o.size_x = std::max(det.bbox.size.x, 0.05);
    o.size_y = std::max(det.bbox.size.y, 0.05);
    o.stamp = stamp;
    batch.push_back(o);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & o : batch) {
    model_.addObservation(o);
  }
}

void PerceptionLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  model_.prune(clock_->now().seconds());
  Bounds b = model_.costBounds();
  // Include last cycle's footprint so cells of vanished objects are reset.
  Bounds touched = b;
  if (last_bounds_.valid) {
    touched.expand(last_bounds_.min_x, last_bounds_.min_y, last_bounds_.max_x, last_bounds_.max_y);
  }
  last_bounds_ = b;
  if (!touched.valid) {
    return;
  }
  *min_x = std::min(*min_x, touched.min_x);
  *min_y = std::min(*min_y, touched.min_y);
  *max_x = std::max(*max_x, touched.max_x);
  *max_y = std::max(*max_y, touched.max_y);
}

void PerceptionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  GridView view{
    master_grid.getCharMap(), master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY(),
    master_grid.getOriginX(), master_grid.getOriginY(), master_grid.getResolution()};
  std::lock_guard<std::mutex> lock(mutex_);
  model_.stampCosts(view, min_i, min_j, max_i, max_j);
}

void PerceptionLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  model_.clear();
  current_ = false;
}

}  // namespace cc_costmap_layers

PLUGINLIB_EXPORT_CLASS(cc_costmap_layers::PerceptionLayer, nav2_costmap_2d::Layer)
