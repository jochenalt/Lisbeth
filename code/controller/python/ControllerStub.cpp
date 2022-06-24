#include "Controller.hpp"

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;


template <typename Controller>
struct ControllerPythonVisitor : public bp::def_visitor<ControllerPythonVisitor<Controller>>
{
    template <class PyClassGait>
    void visit(PyClassGait& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("initialize", &Controller::initialize, bp::args("params"))
            .def("compute", &Controller::compute, bp::args("imuLinearAcceleration", "imuGyroscopse", "imuAttitudeEuler",
            											  "jointPositions", "jointVelocities"))
            .def("command_speed", &Controller::command_speed, bp::args("vX", "vY","heightZ", "angSpeedZ", "rotX", "rotY"))
			.def("command_go", &Controller::command_go, bp::args("ok"))
			.def_readonly("vdes", &Controller::v_des)
			.def_readonly("qdes", &Controller::q_des)
			.def_readonly("tau_ff", &Controller::tau_ff)
			.def_readonly("P", &Controller::P)
			.def_readonly("D", &Controller::D);
    }

    static void expose()
    {
        bp::class_<Controller>("Controller", bp::no_init).def(ControllerPythonVisitor<Controller>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
        ENABLE_SPECIFIC_MATRIX_TYPE(VectorN);
        ENABLE_SPECIFIC_MATRIX_TYPE(Matrix242);
    }
};
void exposeController() { ControllerPythonVisitor<Controller>::expose(); }
