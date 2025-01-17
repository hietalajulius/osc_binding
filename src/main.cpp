#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <osc/osc_step.h>
#include <pybind11/stl.h>


#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

//step_controller(np.zeros(16), np.zeros(16), np.zeros(7), np.zeros(7), np.zeros(7) np.zeros(49), np.zeros(42), np.zeros(7), np.zeros(7), np.zeros(3), np.zeros(3), 10.0, 10.0, 10.0, 10.0, 10.0, 1)

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
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: osc_binding

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

    m.def("step_controller", &step_controller, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");

    m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two numbers

        Some other explanation about the subtract function.
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
