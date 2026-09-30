"""ccsim: a 2D surrogate simulator for the CorridorCourier ablation.

It reproduces the failure modes the project is about (objects below the
LiDAR plane, LiDAR-transparent glass, scripted walking people, wheel slip on
polished floors) with enough fidelity to measure the effect of each
ablation config, and runs the same costmap-layer C++ code as the Nav2 plugin.
It is not a replacement for the Gazebo runs; see docs/RESULTS.md.
"""
