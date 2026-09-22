#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <osc/osc_step.h>
#include <pybind11/stl.h>


#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

Eigen::MatrixXd step_controller(std::array<double, 16> initial_O_T_EE_array,
                                std::array<double, 16> O_T_EE_array,
                                std::array<double, 7> initial_q_array,
                                std::array<double, 7> q_array,
                                std::array<double, 7> dq_array,
                                std::array<double, 49> mass_array,
                                std::array<double, 42> jacobian_array,
                                std::array<double, 7> coriolis_array,
                                std::array<double, 7> tau_J_d_array,
                                std::array<double, 3> position_d_array,
                                std::array<double, 3> velocity_d_array,
                                double delta_tau_max_,
                                double kp_pos,
                                double kp_rot,
                                double damping_ratio) {

    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_O_T_EE_array.data()));
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(O_T_EE_array.data()));
    Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_q(initial_q_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(tau_J_d_array.data());
    Eigen::Vector3d position_d_(position_d_array.data());
    Eigen::Vector3d velocity_d_(velocity_d_array.data());
    Eigen::MatrixXd orientation_d_(initial_transform.linear());

    Eigen::Matrix<double, 3, 3> kp_pos_mat;
    Eigen::Matrix<double, 3, 3> kv_pos_mat;
    Eigen::Matrix<double, 3, 3> kp_rot_mat;
    Eigen::Matrix<double, 3, 3> kv_rot_mat;

    kp_pos_mat << kp_pos * Eigen::Matrix3d::Identity();
    kp_rot_mat << kp_rot * Eigen::Matrix3d::Identity();

    kv_pos_mat << 2 * sqrt(kp_pos) * damping_ratio * Eigen::Matrix3d::Identity();
    kv_rot_mat << 2 * sqrt(kp_rot) * damping_ratio * Eigen::Matrix3d::Identity();
    
    
    Eigen::MatrixXd torques = step(transform, jacobian, mass, coriolis, q, dq, initial_q, tau_J_d, position_d_, velocity_d_, orientation_d_, delta_tau_max_, kp_pos_mat, kp_rot_mat, kv_pos_mat, kv_rot_mat, true);
    return torques;
}



namespace py = pybind11;

PYBIND11_MODULE(osc_binding, m) {
    m.doc() = R"pbdoc(
        Python bindings for a C++ operational-space controller.
        --------------------------------------------------------

        .. currentmodule:: osc_binding

        .. autosummary::
           :toctree: _generate

           step_controller
    )pbdoc";

    m.def("step_controller", &step_controller, R"pbdoc(
        Compute a joint torque command with the external OSC implementation.

        Positional arguments, in order:
            initial_O_T_EE_array, O_T_EE_array: initial and current 4x4
                end-effector transforms, each flattened to 16 values.
            initial_q_array, q_array, dq_array: initial joint positions,
                current joint positions, and joint velocities (7 values each).
            mass_array: 7x7 joint-space mass matrix (49 values).
            jacobian_array: 6x7 end-effector Jacobian (42 values).
            coriolis_array: Coriolis torque vector (7 values).
            tau_J_d_array: previous desired joint torques (7 values).
            position_d_array, velocity_d_array: Cartesian position and
                velocity targets (3 values each).
            delta_tau_max_: maximum torque change passed to the controller.
            kp_pos, kp_rot: scalar position and rotation gains.
            damping_ratio: damping ratio used to derive velocity gains.

        Flatten matrices in column-major order. The orientation target comes
        from initial_O_T_EE_array. Coordinate frames and units must match the
        external controller's conventions.

        Returns the controller's joint torque matrix as a NumPy array.
    )pbdoc");

    m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two integers (legacy template helper).
    )pbdoc");

    class MyClass {
    Eigen::MatrixXd big_mat = Eigen::MatrixXd::Zero(10000, 10000);
    public:
        Eigen::MatrixXd &getMatrix() { return big_mat; }
        const Eigen::MatrixXd &viewMatrix() { return big_mat; }
    };

    // Later, in binding code:
    py::class_<MyClass>(m, "MyClass")
        .def(py::init<>())
        .def("copy_matrix", &MyClass::getMatrix) // Makes a copy!
        .def("get_matrix", &MyClass::getMatrix, py::return_value_policy::reference_internal)
        .def("view_matrix", &MyClass::viewMatrix, py::return_value_policy::reference_internal)
        ;

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
