# CorridorCourier: everything that runs WITHOUT ROS (surrogate simulator,
# C++ layer core, metrics, report). ROS/Gazebo targets are at the bottom and
# need a sourced ROS 2 Jazzy environment (or `make docker`).

PY ?= python3
SEEDS ?= 30
WORKERS ?= $(shell nproc)
export PYTHONPATH := $(CURDIR)/sim:$(CURDIR)/src/cc_eval:$(CURDIR)/src/cc_perception:$(PYTHONPATH)

.PHONY: setup lib test test-fast results drift ablation demo report worlds maps clean \
        ros-build ros-test docker docker-run

setup:
	$(PY) -m pip install numpy scipy matplotlib pillow pytest pyyaml onnxruntime opencv-python-headless

lib:                                   ## C++ cost model as a shared library (used by the simulator)
	$(PY) -c "from ccsim.costlib import build; print(build(force=True))"

test: lib                              ## all ROS-free tests, including full episodes
	g++ -std=c++17 -O2 -Wall -Wextra -Isrc/cc_costmap_layers/include \
	    src/cc_costmap_layers/src/cost_model.cpp src/cc_costmap_layers/test/test_cost_model.cpp \
	    -o sim/build/test_cost_model && sim/build/test_cost_model
	$(PY) -m pytest -q sim/tests src/cc_eval/test src/cc_perception/test

test-fast: lib
	$(PY) -m pytest -q -m "not slow" sim/tests src/cc_eval/test src/cc_perception/test

drift:                                 ## milestone 3: EKF drift study (30 seeds, ~10 s)
	cd sim && $(PY) -m ccsim.experiments drift --seeds $(SEEDS)

ablation:                              ## milestones 5-7: 5 worlds x 4 configs x 2 controllers x SEEDS
	cd sim && $(PY) -m ccsim.experiments ablation --seeds $(SEEDS) --controllers dwb,mppi --workers $(WORKERS)

demo:                                  ## demo GIFs (same seed, baseline vs full stack)
	cd sim && $(PY) -m ccsim.experiments demo

report:                                ## docs/RESULTS.md + results/figures from the CSVs
	cd sim && $(PY) -m ccsim.experiments report

results: lib drift ablation demo report

worlds:                                ## regenerate Gazebo SDF worlds + manifests from sim/ccsim/world.py
	$(PY) src/cc_gazebo/scripts/generate_worlds.py
	$(PY) src/cc_localization/scripts/make_maps.py

clean:
	rm -rf sim/build .pytest_cache

# ---------------------------------------------------------------- ROS 2 Jazzy
ros-build:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

ros-test:
	colcon test --packages-select cc_costmap_layers cc_perception cc_eval && colcon test-result --verbose

docker:
	docker build -t corridor-courier:jazzy -f docker/Dockerfile .

docker-run:
	xhost +local:docker >/dev/null 2>&1 || true
	docker run --rm -it --net=host -e DISPLAY=$$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
	    -v $(CURDIR)/results:/ws/results corridor-courier:jazzy
