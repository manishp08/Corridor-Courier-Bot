// Copyright 2026 CorridorCourier contributors. Apache-2.0.
#ifndef CC_COSTMAP_LAYERS__PERCEPTION_LAYER_HPP_
#define CC_COSTMAP_LAYERS__PERCEPTION_LAYER_HPP_

#include <mutex>
#include <string>

#include "cc_costmap_layers/cost_model.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace cc_costmap_layers
{

// Nav2 costmap layer that turns 3D object detections (from cc_perception)
// into class-aware costs: people get a wider, softer inflation than static
// objects, and objects the 2D LiDAR cannot see (glass, low obstacles,
// cart bodies) become lethal.
//
// Threading: the detection callback runs on the costmap node's executor
// while updateBounds/updateCosts run on the costmap update thread, so all
// access to the CostModel goes through mutex_.
class PerceptionLayer : public nav2_costmap_2d::Layer
{
public:
  PerceptionLayer() = default;
  ~PerceptionLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return true;}
  void onFootprintChanged() override;

private:
  void detectionsCallback(vision_msgs::msg::Detection3DArray::ConstSharedPtr msg);
  ClassParams loadClassParams(const std::string & cls, const ClassParams & defaults);

  std::mutex mutex_;
  CostModel model_;
  Bounds last_bounds_;
  std::string global_frame_;
  double transform_tolerance_{0.2};
  double min_score_{0.4};
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("PerceptionLayer")};
};

}  // namespace cc_costmap_layers

#endif  // CC_COSTMAP_LAYERS__PERCEPTION_LAYER_HPP_
