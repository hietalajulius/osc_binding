# osc_binding

Python bindings for a C++ operational-space controller for robotic manipulation,
built with [pybind11](https://github.com/pybind/pybind11) and Eigen. The binding
passes robot state, dynamics, and Cartesian targets from Python to the controller
and returns joint torques for a seven-joint arm.

The main entry point, `step_controller`, supports position and velocity targets,
uses the initial end-effector orientation as its orientation target, and takes
position/rotation gains, a damping ratio, and a torque-change limit.

## Build requirements

- Python with development headers, a C++11 compiler, and Eigen headers available
  to the compiler.
- CMake and a build tool. The Python build configuration requests CMake >= 3.12
  and Ninja on non-Windows platforms.
- The `pybind11` Git submodule.
- The separate OSC controller source tree, including `osc/osc_step.h` and its
  dependencies. It is **not included in this repository**. The current build
  expects its include directory at
  `${ROBOTICS_PATH}/osc_ws/src/osc/include`.

Once that controller workspace and its dependencies are available:

```bash
git clone --recursive https://github.com/hietalajulius/osc_binding.git
cd osc_binding
export ROBOTICS_PATH=/absolute/path/to/your/robotics/workspace
python -m pip install .
```

`ROBOTICS_PATH` is read by `setup.py` and passed to CMake. Installing this
repository alone does not install the external controller or a robot model.

## Calling the controller

The binding accepts positional arguments. Matrices must be flattened in
**column-major order** because the C++ code maps them into Eigen's default
storage layout.

The following adapter shows the call using state and dynamics supplied by your
robot or simulator. It does not acquire state or send commands to hardware.

```python
import numpy as np
import osc_binding


def compute_torques(
    initial_transform, transform,       # 4 x 4 end-effector transforms
    initial_q, q, dq,                    # 7 joint positions/velocities each
    mass, jacobian,                      # 7 x 7 and 6 x 7
    coriolis, previous_desired_torques,  # 7 values each
    position_target, velocity_target,   # 3 Cartesian values each
    max_torque_change, kp_pos, kp_rot, damping_ratio,
):
    def flat(values):
        return np.asarray(values, dtype=np.float64).ravel(order="F").tolist()

    return osc_binding.step_controller(
        flat(initial_transform), flat(transform),
        flat(initial_q), flat(q), flat(dq),
        flat(mass), flat(jacobian),
        flat(coriolis), flat(previous_desired_torques),
        flat(position_target), flat(velocity_target),
        max_torque_change, kp_pos, kp_rot, damping_ratio,
    )
```

Use consistent coordinate frames and the conventions expected by the external
OSC implementation. The return value is the controller's joint torque matrix,
exposed as a NumPy array.

## Provenance and license

This repository was forked from
[pybind/cuda_example](https://github.com/pybind/cuda_example). The original
copyright notice and license are preserved in [LICENSE](LICENSE). Some legacy
template helpers and tests remain; the controller API is `step_controller`.
