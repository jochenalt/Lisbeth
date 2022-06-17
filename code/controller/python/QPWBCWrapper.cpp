#include "QPWBC.hpp"
#include "WbcWrapper.hpp"

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include "Types.h"


namespace bp = boost::python;


/////////////////////////////////
/// Binding QPWBC class
/////////////////////////////////
template <typename QPWBC>
struct QPWBCPythonVisitor : public bp::def_visitor<QPWBCPythonVisitor<QPWBC>>
{
    template <class PyClassQPWBC>
    void visit(PyClassQPWBC& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("get_f_res", &QPWBC::get_f_res, "Get velocity goals matrix.\n")
            .def("get_ddq_res", &QPWBC::get_ddq_res, "Get acceleration goals matrix.\n")
            .def("get_H", &QPWBC::get_H, "Get H weight matrix.\n")
            .def("initialize", &QPWBC::initialize, "initialize\n")

            // Run QPWBC from Python
            .def("run", &QPWBC::run, bp::args("M", "Jc", "f_cmd", "RNEA", "k_contacts"), "Run QPWBC from Python.\n");
    }

    static void expose()
    {
        bp::class_<QPWBC>("QPWBC", bp::no_init).def(QPWBCPythonVisitor<QPWBC>());

        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};
void exposeQPWBC() { QPWBCPythonVisitor<QPWBC>::expose(); }



/////////////////////////////////
/// Binding WbcWrapper class
/////////////////////////////////
template <typename WbcWrapper>
struct WbcWrapperPythonVisitor : public bp::def_visitor<WbcWrapperPythonVisitor<WbcWrapper>>
{
    template <class PyClassWbcWrapper>
    void visit(PyClassWbcWrapper& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("initialize", &WbcWrapper::initialize, bp::args("params"), "Initialize WbcWrapper from Python.\n")

            .def("get_qdes", &WbcWrapper::get_qdes, "Get qdes_.\n")
            .def("get_vdes", &WbcWrapper::get_vdes, "Get vdes_.\n")
            .def("get_tau_ff", &WbcWrapper::get_tau_ff, "Get tau_ff_.\n")
            .def_readonly("qdes", &WbcWrapper::get_qdes)
            .def_readonly("vdes", &WbcWrapper::get_vdes)
            .def_readonly("tau_ff", &WbcWrapper::get_tau_ff)
            .def_readonly("f_with_delta", &WbcWrapper::get_f_with_delta)
            .def_readonly("feet_pos", &WbcWrapper::get_feet_pos)
            .def_readonly("feet_err", &WbcWrapper::get_feet_err)
            .def_readonly("feet_vel", &WbcWrapper::get_feet_vel)
            .def_readonly("feet_pos_target", &WbcWrapper::get_feet_pos_target)
            .def_readonly("feet_vel_target", &WbcWrapper::get_feet_vel_target)
            .def_readonly("feet_acc_target", &WbcWrapper::get_feet_acc_target)

            // Run WbcWrapper from Python
            .def("compute", &WbcWrapper::compute, bp::args("q", "dq", "f_cmd", "contacts", "pgoals", "vgoals", "agoals"), "Run WbcWrapper from Python.\n");
    }

    static void expose()
    {
        bp::class_<WbcWrapper>("WbcWrapper", bp::no_init).def(WbcWrapperPythonVisitor<WbcWrapper>());

        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};
void exposeWbcWrapper() { WbcWrapperPythonVisitor<WbcWrapper>::expose(); }




