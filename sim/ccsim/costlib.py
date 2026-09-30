"""ctypes binding to the C++ CostModel (src/cc_costmap_layers), built on demand."""
import ctypes
import os
import subprocess
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
PKG = REPO / "src" / "cc_costmap_layers"
LIB = Path(os.environ.get("CC_COST_MODEL_LIB", REPO / "sim" / "build" / "libcc_cost_model.so"))
SOURCES = [PKG / "src" / "cost_model.cpp", PKG / "src" / "cost_model_capi.cpp"]


def build(force=False):
    newest = max(p.stat().st_mtime for p in SOURCES + [PKG / "include" / "cc_costmap_layers" / "cost_model.hpp"])
    if not force and LIB.exists() and LIB.stat().st_mtime >= newest:
        return LIB
    LIB.parent.mkdir(parents=True, exist_ok=True)
    tmp = LIB.with_suffix(f".{os.getpid()}.tmp")
    cmd = ["g++", "-std=c++17", "-O2", "-shared", "-fPIC", f"-I{PKG / 'include'}",
           *map(str, SOURCES), "-o", str(tmp)]
    subprocess.run(cmd, check=True)
    os.replace(tmp, LIB)
    return LIB


_lib = None


def lib():
    global _lib
    if _lib is None:
        L = ctypes.CDLL(str(build()))
        d, i, u, p = ctypes.c_double, ctypes.c_int, ctypes.c_uint, ctypes.c_void_p
        L.ccl_create.restype = p
        L.ccl_create.argtypes = [d]
        L.ccl_destroy.argtypes = [p]
        L.ccl_set_class.argtypes = [p, i, i, d, d, d, i, d]
        L.ccl_add.argtypes = [p, i, d, d, d, d, d, d]
        L.ccl_prune.argtypes = [p, d]
        L.ccl_prune.restype = i
        L.ccl_clear.argtypes = [p]
        L.ccl_count.argtypes = [p]
        L.ccl_count.restype = i
        L.ccl_confirmed.argtypes = [p]
        L.ccl_confirmed.restype = i
        L.ccl_stamp.argtypes = [p, ctypes.POINTER(ctypes.c_ubyte), u, u, d, d, d]
        L.ccl_stamp.restype = i
        _lib = L
    return _lib


class CostModel:
    def __init__(self, inscribed_radius):
        self._l = lib()
        self._h = self._l.ccl_create(inscribed_radius)

    def __del__(self):
        if getattr(self, "_h", None):
            self._l.ccl_destroy(self._h)
            self._h = None

    def set_class(self, cls_id, enabled=True, inflation_radius=0.55, cost_scaling_factor=3.0,
                  persistence=2.0, min_hits=1, association_gate=0.4):
        self._l.ccl_set_class(self._h, cls_id, int(enabled), inflation_radius, cost_scaling_factor,
                              persistence, min_hits, association_gate)

    def add(self, cls_id, x, y, yaw, sx, sy, stamp):
        self._l.ccl_add(self._h, cls_id, x, y, yaw, sx, sy, stamp)

    def prune(self, now):
        return self._l.ccl_prune(self._h, now)

    def clear(self):
        self._l.ccl_clear(self._h)

    def count(self):
        return self._l.ccl_count(self._h)

    def confirmed(self):
        return self._l.ccl_confirmed(self._h)

    def stamp(self, grid, origin, res):
        assert grid.dtype == np.uint8 and grid.flags.c_contiguous
        ny, nx = grid.shape
        ptr = grid.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))
        return self._l.ccl_stamp(self._h, ptr, nx, ny, origin[0], origin[1], res)
