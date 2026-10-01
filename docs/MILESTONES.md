# Milestones and acceptance tests

Each milestone has a command and a pass criterion. **Status** is honest about where it has been run:

* *Surrogate:* run in this repository's 2D simulator. The numbers are in [RESULTS.md](RESULTS.md).
* *Gazebo:* needs a ROS 2 Jazzy + Gazebo Harmonic machine (`make docker`). The commands are ready; run
  them there and paste the output into this file.

Every ROS command assumes `source install/setup.bash`. Every node runs with `use_sim_time:=true`.

---

## 1. Robot and world
**Goal:** the robot drives with teleop and the TF tree is clean.

```bash
ros2 launch cc_gazebo sim.launch.py world:=empty_corridor wheel_tf:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard            # publishes /cmd_vel
ros2 run tf2_tools view_frames                                  # writes frames_<date>.pdf
```
**Pass:** the robot moves. `frames.pdf` shows one tree,
`odom → base_footprint → base_link → {lidar_link, imu_link, camera_link → camera_optical_frame, *_wheel_link}`,
with no second parent for any frame. Every edge is recent.
**Status:** Gazebo (not run here).

## 2. Sensor sanity
**Goal:** LiDAR, IMU and camera verified, with frame timing checked.

```bash
ros2 run cc_eval sensor_sanity --ros-args -p use_sim_time:=true -p duration:=20.0
ros2 launch cc_navigation bringup.launch.py world:=cluttered_room ablation:=A rviz:=true   # visual check
```
**Pass:** every row prints `OK`. Rates are at least 80% of nominal, the p95 stamp lag is 200 ms or less,
and frame IDs are as expected. In RViz the scan lines up with the walls, pallets are *absent* from the
scan (they sit below its plane), and the camera image shows them.
**Status:** Gazebo (not run here).

## 3. EKF: 20 m loop drift, with and without the IMU
```bash
make drift                                                    # surrogate, 30 seeds, paired
ros2 run cc_eval drift_test --ros-args -p use_sim_time:=true -p side:=2.5 -p loops:=2   # Gazebo, glass_wall world
```
**Pass:** the tuned EKF has lower final error than wheel-only odometry. The deliverable is the number.
**Status:** Surrogate ✔. Final error fell from 1.19 m to 0.26 m, about 78% less (see RESULTS.md, "EKF drift").
The run also showed both pitfalls. Untuned covariances gave no gain, and an uncalibrated gyro made the
result worse than no fusion.

## 4. SLAM and AMCL
```bash
# Map on the empty layout (no clutter, glass or people), then save it
ros2 launch cc_gazebo sim.launch.py world:=kidnapped
ros2 launch cc_localization localization.launch.py mode:=slam ekf:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # drive the loop twice for loop closure
ros2 run nav2_map_server map_saver_cli -f src/cc_localization/maps/kidnapped
# Localize and compare with ground truth
ros2 launch cc_navigation bringup.launch.py world:=kidnapped ablation:=B
ros2 run cc_eval harness --ros-args -p use_sim_time:=true -p world:=kidnapped -p config:=B -p bag_dir:=bags
evo_ape tum bags/kidnapped_B_dwb_1000/gt.tum bags/kidnapped_B_dwb_1000/est.tum -p
```
**Pass:** the median ATE is below 0.15 m in the room-scale worlds (2, 3), and the robot re-localizes after the kidnap.
**Result so far:** the room-scale worlds pass (0.035–0.051 m). The corridor worlds do not: 0.20–0.32 m,
because position along a featureless corridor is unobservable. Kidnap recovery succeeded in 43–60% of runs.
**Status:** Surrogate ✔ for the AMCL part (ATE and kidnap tables in RESULTS.md). The surrogate uses a walls-only
reference map (`src/cc_localization/maps/*`, made by `make_maps.py`) in place of the slam_toolbox map. Gazebo:
the mapping pass is still to run.

## 5. Nav2 baseline plus the eval harness
```bash
make ablation SEEDS=30                                   # surrogate
ros2 run cc_eval run_ablation --seeds 30 --configs A     # Gazebo baseline first
```
**Pass:** the harness writes one CSV row per episode (schema in `cc_eval/results_io.py`) and a rosbag.
`make report` or `ros2 run cc_eval report <dir> <out.md>` turns the CSV into tables with confidence intervals.
Config A reaches the goal in the empty corridor (control).
**Status:** Surrogate ✔. A reached the goal in 93% (DWB) and 97% (MPPI) of corridor seeds. The failures
came from AMCL losing position along the featureless corridor, which happens in every config. Gazebo harness
written, not run here.

## 6. Perception and the costmap layer
```bash
make test                                                   # C++ layer core + projection tests
ros2 launch cc_navigation bringup.launch.py world:=glass_wall ablation:=D detector:=oracle rviz:=true
ros2 run cc_perception detector_node --ros-args -p backend:=onnx -p model_path:=models/yolov8n.onnx
```
**Pass:** the glass pane and pallets appear as lethal cost in `/global_costmap/costmap` while they are absent
from `/scan`. A person's inflation visibly exceeds a pallet's.
**Status:** Surrogate ✔ (the same C++ core, called through its C API). Unit tests ✔. The Gazebo plugin
build is covered by the `ros` CI job.

## 7. Ablation, writeup and demo video
```bash
make results                                  # drift + ablation (DWB and MPPI) + demo GIFs + RESULTS.md
ros2 run cc_eval run_ablation --seeds 30 --controllers dwb,mppi --bags   # the Gazebo version
```
**Status:** Surrogate ✔: [RESULTS.md](RESULTS.md) and `results/demo/*.gif`. For a Gazebo video, record the
RViz or Gazebo window during `bringup.launch.py` runs of the same seeds.
