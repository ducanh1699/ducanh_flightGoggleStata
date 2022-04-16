#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "pybind11/eigen.h"
#include "carDynamicsSim.hpp"

namespace py = pybind11;

void bind_CarDynamicsSim(py::module &m);

PYBIND11_MODULE(car_dynamics_sim, m) {
    m.doc() = "Python package of carDynamicsSim";
    bind_CarDynamicsSim(m);
}

void bind_CarDynamicsSim(py::module &m) {
    py::class_<CarDynamicsSim>(m, "CarDynamicsSim")
        .def(py::init<double, double, double, const Eigen::Vector2d &>(),
            py::arg("maxSteeringAngle")=1.0, 
            py::arg("minSpeed")=0.0, 
            py::arg("maxSpeed")=5.0, 
            py::arg("velProcNoiseAutoCorr")=Eigen::Vector2d(0.001,0.001))
        .def("setNoiseSeed", &CarDynamicsSim::setNoiseSeed, "set random seeds",
            py::arg("seed"))
        .def("setVehicleProperties", &CarDynamicsSim::setVehicleProperties, "set vehicle properties",
            py::arg("maxSteeringAngle"), 
            py::arg("minSpeed"), py::arg("maxSpeed"), 
            py::arg("velProcNoiseAutoCorr"))
        .def("setVehicleState", [](CarDynamicsSim & carSim,
            const Eigen::Vector2d & position, 
            const Eigen::Vector2d & velocity,
            double heading) {
            Eigen::Rotation2Dd heading_eigen(heading);
            carSim.setVehicleState(position, velocity, heading_eigen);
            }, "set vehicle state",
            py::arg("position"), py::arg("velocity"), py::arg("heading"))
        .def("getVehicleState", [](CarDynamicsSim & carSim) {
            Eigen::Vector2d position, velocity;
            Eigen::Rotation2Dd heading;
            Eigen::Matrix<double, 1, 1> heading_radian;

            carSim.getVehicleState(position, velocity, heading);
            heading_radian << heading.angle();
            
            std::map<std::string, Eigen::VectorXd> vec;
            vec.insert(std::pair<std::string, Eigen::VectorXd>("position",position));
            vec.insert(std::pair<std::string, Eigen::VectorXd>("velocity",velocity));
            vec.insert(std::pair<std::string, Eigen::VectorXd>("heading",heading_radian));
            // vec.insert(std::pair<std::string, Eigen::VectorXd>("heading",heading.angle()));
            return vec;
            }, "get vehicle state")

        .def("setVehiclePosition", &CarDynamicsSim::setVehiclePosition, "set vehicle position",
            py::arg("position"))
        .def("setVehicleVelocity", &CarDynamicsSim::setVehicleVelocity, "set vehicle velocity",
            py::arg("velocity"))
        .def("setVehicleHeading", [](CarDynamicsSim & carSim, double heading) {
            Eigen::Rotation2Dd rot(heading);
            carSim.setVehicleHeading(rot);
            }, "set vehicle heading",
            py::arg("heading"))
        
        .def("getVehiclePosition", &CarDynamicsSim::getVehiclePosition, "get vehicle position")
        .def("getVehicleVelocity", &CarDynamicsSim::getVehicleVelocity, "get vehicle velocity")
        .def("getVehicleHeading", [](CarDynamicsSim & carSim) {
            Eigen::Rotation2Dd heading = carSim.getVehicleHeading();
            return heading.angle();
            }, "get vehicle heading")
        
        .def("proceedState_ExplicitEuler", &CarDynamicsSim::proceedState_ExplicitEuler, 
            "proceed state with explicit euler method",
            py::arg("dt_secs"), py::arg("speed"), py::arg("steeringAngle"));
}