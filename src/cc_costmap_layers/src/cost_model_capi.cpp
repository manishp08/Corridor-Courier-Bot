// Copyright 2026 CorridorCourier contributors. Apache-2.0.
//
// Plain C API over CostModel so non-ROS code (the 2D surrogate simulator,
// via Python ctypes) runs exactly the code the Nav2 plugin runs.
#include "cc_costmap_layers/cost_model.hpp"

using cc_costmap_layers::ClassParams;
using cc_costmap_layers::CostModel;
using cc_costmap_layers::GridView;
using cc_costmap_layers::ObjectClass;
using cc_costmap_layers::Observation;

extern "C" {

void * ccl_create(double inscribed_radius)
{
  auto * m = new CostModel();
  m->setInscribedRadius(inscribed_radius);
  return m;
}

void ccl_destroy(void * h) {delete static_cast<CostModel *>(h);}

void ccl_set_class(
  void * h, int cls, int enabled, double inflation_radius, double cost_scaling_factor,
  double persistence, int min_hits, double association_gate)
{
  ClassParams p;
  p.enabled = enabled != 0;
  p.inflation_radius = inflation_radius;
  p.cost_scaling_factor = cost_scaling_factor;
  p.persistence = persistence;
  p.min_hits = min_hits;
  p.association_gate = association_gate;
  static_cast<CostModel *>(h)->setClassParams(static_cast<ObjectClass>(cls), p);
}

void ccl_add(
  void * h, int cls, double x, double y, double yaw, double size_x, double size_y,
  double stamp)
{
  Observation o;
  o.cls = static_cast<ObjectClass>(cls);
  o.x = x; o.y = y; o.yaw = yaw; o.size_x = size_x; o.size_y = size_y; o.stamp = stamp;
  static_cast<CostModel *>(h)->addObservation(o);
}

int ccl_prune(void * h, double now) {return static_cast<int>(static_cast<CostModel *>(h)->prune(now));}

void ccl_clear(void * h) {static_cast<CostModel *>(h)->clear();}

int ccl_count(void * h) {return static_cast<int>(static_cast<CostModel *>(h)->objects().size());}

int ccl_confirmed(void * h) {return static_cast<int>(static_cast<CostModel *>(h)->confirmedCount());}

int ccl_stamp(
  void * h, unsigned char * data, unsigned int size_x, unsigned int size_y,
  double origin_x, double origin_y, double resolution)
{
  GridView g{data, size_x, size_y, origin_x, origin_y, resolution};
  return static_cast<int>(static_cast<CostModel *>(h)->stampCosts(g));
}

}  // extern "C"
