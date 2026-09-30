// Copyright 2026 CorridorCourier contributors. Apache-2.0.
//
// ROS-independent core of the perception costmap layer.
//
// Takes object observations (already in the costmap's global frame), fuses
// repeated observations of the same object, forgets them after a
// class-specific persistence time and stamps class-specific costs into a
// grid.  The Nav2 plugin (perception_layer.cpp) is a thin, thread-safe
// wrapper around this class; the 2D surrogate simulator loads the very same
// code through the C API in cost_model_capi.cpp, so the costs measured in
// the ablation are the costs this code produces.
#ifndef CC_COSTMAP_LAYERS__COST_MODEL_HPP_
#define CC_COSTMAP_LAYERS__COST_MODEL_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace cc_costmap_layers
{

// Same numeric values as nav2_costmap_2d/cost_values.hpp.
constexpr unsigned char kNoInformation = 255;
constexpr unsigned char kLethal = 254;
constexpr unsigned char kInscribed = 253;
constexpr unsigned char kFree = 0;

enum class ObjectClass : std::uint8_t
{
  kPerson = 0,
  kCart = 1,
  kLowObstacle = 2,
  kGlass = 3,
  kUnknown = 4,
};
constexpr std::size_t kNumClasses = 5;

ObjectClass classFromString(const std::string & name);
const char * classToString(ObjectClass cls);

struct ClassParams
{
  bool enabled{true};
  // Distance (m) from the object's surface at which cost reaches zero.
  double inflation_radius{0.55};
  // Exponential decay rate, same meaning as nav2 inflation_layer.
  double cost_scaling_factor{3.0};
  // Seconds an object is kept after its last observation.
  double persistence{2.0};
  // Observations needed before the object is written to the grid
  // (suppresses single-frame false positives).
  int min_hits{1};
  // Association gate (m) for merging an observation into an existing object.
  double association_gate{0.4};
};

struct Observation
{
  ObjectClass cls{ObjectClass::kUnknown};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double size_x{0.1};
  double size_y{0.1};
  double stamp{0.0};
};

struct TrackedObject
{
  Observation obs;
  int hits{0};
  double first_seen{0.0};
};

struct Bounds
{
  double min_x, min_y, max_x, max_y;
  bool valid{false};
  void expand(double x0, double y0, double x1, double y1);
};

// Non-owning view of a row-major uint8 cost grid (index = my * size_x + mx),
// the layout of nav2_costmap_2d::Costmap2D::getCharMap().
struct GridView
{
  unsigned char * data;
  unsigned int size_x;
  unsigned int size_y;
  double origin_x;
  double origin_y;
  double resolution;
};

class CostModel
{
public:
  CostModel();

  void setInscribedRadius(double r) {inscribed_radius_ = r;}
  double inscribedRadius() const {return inscribed_radius_;}
  void setClassParams(ObjectClass cls, const ClassParams & p);
  const ClassParams & classParams(ObjectClass cls) const;

  // Merges into an existing object of the same class inside the gate,
  // otherwise starts a new object.
  void addObservation(const Observation & obs);
  // Drops objects older than their class persistence. Returns the number removed.
  std::size_t prune(double now);
  void clear() {objects_.clear();}

  const std::vector<TrackedObject> & objects() const {return objects_;}
  std::size_t confirmedCount() const;

  // World-frame box that stampCosts() may touch, including inflation.
  Bounds costBounds() const;

  // Writes costs with max-combination into the grid, limited to the cell
  // window [min_i, max_i) x [min_j, max_j). Returns the number of cells raised.
  std::size_t stampCosts(
    GridView & grid, int min_i, int min_j, int max_i,
    int max_j) const;
  std::size_t stampCosts(GridView & grid) const;

  // Nav2 inflation law as a function of the distance from the obstacle surface.
  static unsigned char costAtDistance(
    double dist, double inscribed_radius,
    double cost_scaling_factor, double inflation_radius);

private:
  static double distanceToBox(
    double px, double py, const Observation & o);

  std::array<ClassParams, kNumClasses> params_;
  std::vector<TrackedObject> objects_;
  double inscribed_radius_{0.25};
};

}  // namespace cc_costmap_layers

#endif  // CC_COSTMAP_LAYERS__COST_MODEL_HPP_
