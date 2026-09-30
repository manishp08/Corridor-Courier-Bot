// Copyright 2026 CorridorCourier contributors. Apache-2.0.
#include "cc_costmap_layers/cost_model.hpp"

#include <algorithm>
#include <cmath>

namespace cc_costmap_layers
{

ObjectClass classFromString(const std::string & name)
{
  if (name == "person") {return ObjectClass::kPerson;}
  if (name == "cart") {return ObjectClass::kCart;}
  if (name == "low_obstacle") {return ObjectClass::kLowObstacle;}
  if (name == "glass") {return ObjectClass::kGlass;}
  return ObjectClass::kUnknown;
}

const char * classToString(ObjectClass cls)
{
  switch (cls) {
    case ObjectClass::kPerson: return "person";
    case ObjectClass::kCart: return "cart";
    case ObjectClass::kLowObstacle: return "low_obstacle";
    case ObjectClass::kGlass: return "glass";
    default: return "unknown";
  }
}

void Bounds::expand(double x0, double y0, double x1, double y1)
{
  if (!valid) {
    min_x = x0; min_y = y0; max_x = x1; max_y = y1; valid = true;
    return;
  }
  min_x = std::min(min_x, x0);
  min_y = std::min(min_y, y0);
  max_x = std::max(max_x, x1);
  max_y = std::max(max_y, y1);
}

CostModel::CostModel()
{
  // Defaults; the plugin and simulator override them from parameters.
  params_[static_cast<int>(ObjectClass::kPerson)] = {true, 1.2, 1.5, 0.5, 1, 0.6};
  params_[static_cast<int>(ObjectClass::kCart)] = {true, 0.55, 3.0, 5.0, 2, 0.4};
  params_[static_cast<int>(ObjectClass::kLowObstacle)] = {true, 0.55, 3.0, 10.0, 2, 0.4};
  params_[static_cast<int>(ObjectClass::kGlass)] = {true, 0.55, 3.0, 20.0, 2, 0.6};
  params_[static_cast<int>(ObjectClass::kUnknown)] = {false, 0.55, 3.0, 1.0, 3, 0.4};
}

void CostModel::setClassParams(ObjectClass cls, const ClassParams & p)
{
  params_[static_cast<int>(cls)] = p;
}

const ClassParams & CostModel::classParams(ObjectClass cls) const
{
  return params_[static_cast<int>(cls)];
}

void CostModel::addObservation(const Observation & obs)
{
  const ClassParams & p = classParams(obs.cls);
  if (!p.enabled) {
    return;
  }
  TrackedObject * best = nullptr;
  double best_d = p.association_gate;
  for (auto & t : objects_) {
    if (t.obs.cls != obs.cls) {continue;}
    const double d = std::hypot(t.obs.x - obs.x, t.obs.y - obs.y);
    if (d <= best_d) {
      best_d = d;
      best = &t;
    }
  }
  if (best == nullptr) {
    objects_.push_back({obs, 1, obs.stamp});
    return;
  }
  // People move: take the newest pose. Static objects: running average
  // (weight capped so a mis-association can still be corrected).
  if (obs.cls == ObjectClass::kPerson) {
    best->obs = obs;
  } else {
    const double w = 1.0 / std::min(best->hits + 1, 10);
    best->obs.x += w * (obs.x - best->obs.x);
    best->obs.y += w * (obs.y - best->obs.y);
    best->obs.size_x = std::max(best->obs.size_x, obs.size_x);
    best->obs.size_y = std::max(best->obs.size_y, obs.size_y);
    best->obs.yaw = obs.yaw;
    best->obs.stamp = obs.stamp;
  }
  best->hits += 1;
}

std::size_t CostModel::prune(double now)
{
  const auto before = objects_.size();
  objects_.erase(
    std::remove_if(
      objects_.begin(), objects_.end(),
      [&](const TrackedObject & t) {
        return now - t.obs.stamp > classParams(t.obs.cls).persistence;
      }),
    objects_.end());
  return before - objects_.size();
}

std::size_t CostModel::confirmedCount() const
{
  return static_cast<std::size_t>(std::count_if(
           objects_.begin(), objects_.end(), [&](const TrackedObject & t) {
             return t.hits >= classParams(t.obs.cls).min_hits;
           }));
}

Bounds CostModel::costBounds() const
{
  Bounds b;
  for (const auto & t : objects_) {
    const auto & o = t.obs;
    const double half_diag = 0.5 * std::hypot(o.size_x, o.size_y);
    const double r = half_diag + classParams(o.cls).inflation_radius;
    b.expand(o.x - r, o.y - r, o.x + r, o.y + r);
  }
  return b;
}

unsigned char CostModel::costAtDistance(
  double dist, double inscribed_radius, double cost_scaling_factor, double inflation_radius)
{
  if (dist <= 0.0) {return kLethal;}
  if (dist <= inscribed_radius) {return kInscribed;}
  if (dist > inflation_radius) {return kFree;}
  const double factor = std::exp(-cost_scaling_factor * (dist - inscribed_radius));
  return static_cast<unsigned char>((kInscribed - 1) * factor);
}

double CostModel::distanceToBox(double px, double py, const Observation & o)
{
  const double c = std::cos(o.yaw), s = std::sin(o.yaw);
  const double dx = px - o.x, dy = py - o.y;
  const double lx = std::abs(c * dx + s * dy) - 0.5 * o.size_x;
  const double ly = std::abs(-s * dx + c * dy) - 0.5 * o.size_y;
  return std::hypot(std::max(lx, 0.0), std::max(ly, 0.0));
}

std::size_t CostModel::stampCosts(
  GridView & g, int min_i, int min_j, int max_i, int max_j) const
{
  min_i = std::max(min_i, 0);
  min_j = std::max(min_j, 0);
  max_i = std::min(max_i, static_cast<int>(g.size_x));
  max_j = std::min(max_j, static_cast<int>(g.size_y));
  std::size_t raised = 0;
  for (const auto & t : objects_) {
    const ClassParams & p = classParams(t.obs.cls);
    if (!p.enabled || t.hits < p.min_hits) {continue;}
    const auto & o = t.obs;
    const double r = 0.5 * std::hypot(o.size_x, o.size_y) + p.inflation_radius;
    const int i0 = std::max(min_i, static_cast<int>(std::floor((o.x - r - g.origin_x) / g.resolution)));
    const int i1 = std::min(max_i, static_cast<int>(std::ceil((o.x + r - g.origin_x) / g.resolution)));
    const int j0 = std::max(min_j, static_cast<int>(std::floor((o.y - r - g.origin_y) / g.resolution)));
    const int j1 = std::min(max_j, static_cast<int>(std::ceil((o.y + r - g.origin_y) / g.resolution)));
    for (int j = j0; j < j1; ++j) {
      const double wy = g.origin_y + (j + 0.5) * g.resolution;
      unsigned char * row = g.data + static_cast<std::size_t>(j) * g.size_x;
      for (int i = i0; i < i1; ++i) {
        const double wx = g.origin_x + (i + 0.5) * g.resolution;
        // A cell counts as "on" the object if its centre is within half a
        // cell of the surface, so thin objects (glass) still get a lethal core.
        const double d = std::max(0.0, distanceToBox(wx, wy, o) - 0.5 * g.resolution);
        const unsigned char c = costAtDistance(
          d, inscribed_radius_, p.cost_scaling_factor, p.inflation_radius);
        if (c > row[i] || row[i] == kNoInformation) {
          if (c == kFree && row[i] == kNoInformation) {continue;}
          row[i] = c;
          ++raised;
        }
      }
    }
  }
  return raised;
}

std::size_t CostModel::stampCosts(GridView & g) const
{
  return stampCosts(g, 0, 0, static_cast<int>(g.size_x), static_cast<int>(g.size_y));
}

}  // namespace cc_costmap_layers
