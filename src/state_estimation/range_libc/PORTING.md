# ROS 2 Jazzy port

Vendored from [f1tenth/range_libc](https://github.com/f1tenth/range_libc), branch `foxy-devel`,
commit `781acfc`. The C++ library (`includes/RangeLib.h`) is unchanged. License stays Apache-2.0
(`LICENSE`, upstream's `license.txt`), Copyright 2017 Corey H. Walsh.

Kept: `includes/` (header-only library), `vendor/lodepng`, `vendor/distance_transform.h` and
`pywrapper/RangeLibc.pyx`. Dropped: the standalone C++ benchmark (`main.cpp`, gflags), the docs,
maps and plotting scripts, and the CUDA sources (`kernels.cu`, `CudaRangeLib.h`).

Changes:

- Build: an `ament_cmake` package. CMake runs `cython3` on the `.pyx` and builds the `range_libc`
  Python module into the workspace's site-packages. Upstream's `setup.py` used `distutils` and
  Cython's `old_build_ext`, which no longer exist on Python 3.12. Dependencies (`cython3`,
  `python3-dev`, `python3-numpy`) come from rosdep.
- CPU ray casting only: `bl`, `rm`, `cddt`/`pcddt`, `glt`. `PyRayMarchingGPU` stays in the
  wrapper but refuses to construct (USE_CUDA is 0). To add `rmgpu`, re-vendor the CUDA sources
  and compile them with `nvcc` when CUDA is available.
- `RangeLibc.pyx`: Python 3 (`range` instead of `xrange`). The map yaw comes from the quaternion
  directly, so `transforms3d` is not needed. `nav_msgs` is optional (numpy or PNG maps work
  without ROS). An `OccupancyGrid` is loaded by setting only its occupied cells, instead of
  looping over every cell in Python.
- Test: `test/test_range_libc.py` ray-casts a non-square box map (OccupancyGrid input, map
  coordinates) with every CPU method.
