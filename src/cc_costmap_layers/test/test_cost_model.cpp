// Copyright 2026 CorridorCourier contributors. Apache-2.0.
// Dependency-free unit tests for CostModel (run by colcon test and by `make test`).
#include <cmath>
#include <cstdio>
#include <vector>

#include "cc_costmap_layers/cost_model.hpp"

using namespace cc_costmap_layers;  // NOLINT

static int g_failures = 0;
#define CHECK(cond) do { if (!(cond)) { \
      std::fprintf(stderr, "%s:%d CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures; } } while (0)

struct Grid
{
  std::vector<unsigned char> data;
  GridView view;
  Grid(unsigned int nx, unsigned int ny, double res)
  : data(nx * ny, kFree), view{nullptr, nx, ny, 0.0, 0.0, res} {view.data = data.data();}
  unsigned char at(double x, double y) const
  {
    const auto i = static_cast<unsigned int>(x / view.resolution);
    const auto j = static_cast<unsigned int>(y / view.resolution);
    return data[j * view.size_x + i];
  }
};

static Observation obs(ObjectClass c, double x, double y, double t, double s = 0.4)
{
  Observation o;
  o.cls = c; o.x = x; o.y = y; o.size_x = s; o.size_y = s; o.stamp = t;
  return o;
}

static void test_cost_law()
{
  CHECK(CostModel::costAtDistance(0.0, 0.25, 3.0, 0.55) == kLethal);
  CHECK(CostModel::costAtDistance(0.2, 0.25, 3.0, 0.55) == kInscribed);
  CHECK(CostModel::costAtDistance(0.6, 0.25, 3.0, 0.55) == kFree);
  const auto a = CostModel::costAtDistance(0.3, 0.25, 3.0, 0.55);
  const auto b = CostModel::costAtDistance(0.5, 0.25, 3.0, 0.55);
  CHECK(a > b && b > kFree && a < kInscribed);
}

static void test_people_inflate_wider_than_static()
{
  CostModel m;
  m.addObservation(obs(ObjectClass::kPerson, 2.0, 2.0, 0.0));
  m.addObservation(obs(ObjectClass::kLowObstacle, 6.0, 2.0, 0.0));
  m.addObservation(obs(ObjectClass::kLowObstacle, 6.0, 2.0, 0.1));  // min_hits = 2
  Grid g(200, 80, 0.05);
  m.stampCosts(g.view);
  CHECK(g.at(2.0, 2.0) == kLethal);
  CHECK(g.at(6.0, 2.0) == kLethal);
  // 0.9 m from centre = 0.7 m from surface: inside person radius, outside static.
  CHECK(g.at(2.9, 2.0) > kFree);
  CHECK(g.at(6.9, 2.0) == kFree);
}

static void test_min_hits_and_persistence()
{
  CostModel m;
  m.addObservation(obs(ObjectClass::kGlass, 3.0, 2.0, 0.0));
  Grid g(200, 80, 0.05);
  m.stampCosts(g.view);
  CHECK(g.at(3.0, 2.0) == kFree);  // single hit not confirmed
  m.addObservation(obs(ObjectClass::kGlass, 3.05, 2.0, 0.1));
  CHECK(m.objects().size() == 1);  // associated, not duplicated
  m.stampCosts(g.view);
  CHECK(g.at(3.0, 2.0) == kLethal);
  CHECK(m.prune(10.0) == 0);
  CHECK(m.prune(30.0) == 1);
}

static void test_window_and_max_combine()
{
  CostModel m;
  m.addObservation(obs(ObjectClass::kPerson, 2.0, 2.0, 0.0));
  Grid g(200, 80, 0.05);
  g.data[40 * 200 + 40] = kLethal;  // pre-existing lethal cell elsewhere is kept
  m.stampCosts(g.view, 0, 0, 30, 80);  // window excludes the person centre column
  CHECK(g.at(2.0, 2.0) == kLethal);    // cell (40,40) is exactly the preset one
  CHECK(g.at(1.6, 2.0) == kFree);      // i = 32 is outside the window
  const Bounds b = m.costBounds();
  CHECK(b.valid && b.min_x < 1.0 && b.max_x > 3.0);
}

static void test_person_takes_latest_pose()
{
  CostModel m;
  m.addObservation(obs(ObjectClass::kPerson, 2.0, 2.0, 0.0));
  m.addObservation(obs(ObjectClass::kPerson, 2.3, 2.0, 0.1));
  CHECK(m.objects().size() == 1);
  CHECK(std::abs(m.objects()[0].obs.x - 2.3) < 1e-9);
}

int main()
{
  test_cost_law();
  test_people_inflate_wider_than_static();
  test_min_hits_and_persistence();
  test_window_and_max_combine();
  test_person_takes_latest_pose();
  if (g_failures == 0) {std::printf("cost_model tests passed\n");}
  return g_failures == 0 ? 0 : 1;
}
