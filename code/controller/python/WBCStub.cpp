#include "QPWBC.hpp"
#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

#include "WbcController.hpp"
#include "Types.h"


namespace bp = boost::python;

/////////////////////////////////
/// Binding WbcWrapper class
/////////////////////////////////
template <typename WbcController>
struct WbcControllerPythonVisitor : public bp::def_visitor<WbcControllerPythonVisitor<WbcController>>
{
    template <class PyClassWbcConroller>
    void visit(PyClassWbcConroller& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("initialize", &WbcController::initialize, bp::args("params"), "Initialize WbcWrapper from Python.\n")

            .def("get_qdes", &WbcController::get_qdes, "Get qdes_.\n")
            .def("get_vdes", &WbcController::get_vdes, "Get vdes_.\n")
            .def("get_tau_ff", &WbcController::get_tau_ff, "Get tau_ff_.\n")
            .def_readonly("qdes", &WbcController::get_qdes)
            .def_readonly("vdes", &WbcController::get_vdes)
            .def_readonly("tau_ff", &WbcController::get_tau_ff)
            .def_readonly("f_with_delta", &WbcController::get_f_with_delta)
            .def_readonly("feet_pos", &WbcController::get_feet_pos)
            .def_readonly("feet_err", &WbcController::get_feet_err)
            .def_readonly("feet_vel", &WbcController::get_feet_vel)
            .def_readonly("feet_pos_target", &WbcController::get_feet_pos_target)
            .def_readonly("feet_vel_target", &WbcController::get_feet_vel_target)
            .def_readonly("feet_acc_target", &WbcController::get_feet_acc_target)

            // Run WbcWrapper from Python
            .def("compute", &WbcController::compute, bp::args("q", "dq", "f_cmd", "contacts", "pgoals", "vgoals", "agoals", "xgoals"), "Run WbcWrapper from Python.\n");
    }

    static void expose()
    {
        bp::class_<WbcController>("WbcController", bp::no_init).def(WbcControllerPythonVisitor<WbcController>());

        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};
void exposeWbcController() { WbcControllerPythonVisitor<WbcController>::expose(); }




