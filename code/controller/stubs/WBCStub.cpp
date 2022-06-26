#include "WBCSolver.hpp"

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

#include "WBCController.hpp"
#include "Types.h"


namespace bp = boost::python;

/////////////////////////////////
/// Binding WbcWrapper class
/////////////////////////////////
template <typename WBCController>
struct WbcControllerPythonVisitor : public bp::def_visitor<WbcControllerPythonVisitor<WBCController>>
{
    template <class PyClassWbcConroller>
    void visit(PyClassWbcConroller& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("initialize", &WBCController::initialize, bp::args("params"), "Initialize WbcWrapper from Python.\n")

            .def("get_qdes", &WBCController::get_qdes, "Get qdes_.\n")
            .def("get_vdes", &WBCController::get_vdes, "Get vdes_.\n")
            .def("get_tau_ff", &WBCController::get_tau_ff, "Get tau_ff_.\n")
            .def_readonly("qdes", &WBCController::get_qdes)
            .def_readonly("vdes", &WBCController::get_vdes)
            .def_readonly("tau_ff", &WBCController::get_tau_ff)
            .def_readonly("f_with_delta", &WBCController::get_f_with_delta)
            .def_readonly("feet_pos", &WBCController::get_feet_pos)
            .def_readonly("feet_err", &WBCController::get_feet_err)
            .def_readonly("feet_vel", &WBCController::get_feet_vel)
            .def_readonly("feet_pos_target", &WBCController::get_feet_pos_target)
            .def_readonly("feet_vel_target", &WBCController::get_feet_vel_target)
            .def_readonly("feet_acc_target", &WBCController::get_feet_acc_target)
            .def("compute", &WBCController::compute, bp::args("q", "dq", "f_cmd", "contacts", "pgoals", "vgoals", "agoals", "xgoals"), "Run WbcWrapper from Python.\n");
    }

    static void expose()
    {
        bp::class_<WBCController>("WbcController", bp::no_init).def(WbcControllerPythonVisitor<WBCController>());

        ENABLE_SPECIFIC_MATRIX_TYPE(Vector12);
        ENABLE_SPECIFIC_MATRIX_TYPE(Matrix34);
        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};
void exposeWbcController() { WbcControllerPythonVisitor<WBCController>::expose(); }




