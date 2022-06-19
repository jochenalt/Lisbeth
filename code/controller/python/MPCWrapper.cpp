#include "MPC.hpp"

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;

template <typename MPC>
struct MPCPythonVisitor : public bp::def_visitor<MPCPythonVisitor<MPC>>
{
    template <class PyClassMPC>
    void visit(PyClassMPC& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))
            .def(bp::init<Params&>(bp::args("params"),
                                               "Constructor with parameters."))

            // Run MPC from Python
            .def("run", &MPC::run, bp::args("xref_in", "fsteps_in"), "Run MPC from Python.\n")
            .def("get_latest_result", &MPC::get_latest_result,
                 "Get latest result (predicted trajectory  forces to apply).\n")
            .def("get_gait", &MPC::get_gait, "Get gait matrix.\n")
            .def("get_Sgait", &MPC::get_Sgait, "Get S_gait matrix.\n");
    }

    static void expose()
    {
        bp::class_<MPC>("MPC", bp::no_init).def(MPCPythonVisitor<MPC>());

        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};

void exposeMPC() { MPCPythonVisitor<MPC>::expose(); }

#include "MpcController.hpp"

template <typename MpcController>
struct MpcControllerPythonVisitor : public bp::def_visitor<MpcControllerPythonVisitor<MpcController>>
{
    template <class PyClassGait>
    void visit(PyClassGait& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("initialize", &MpcController::initialize, bp::args("params"),
                 "Initialize MPC from Python.\n")
            .def("get_latest_result", &MpcController::get_latest_result)
        	.def("solve", &MpcController::solve, bp::args("fsteps","gait", "fsteps_in"));
    }

    static void expose()
    {
        bp::class_<MpcController>("MpcController", bp::no_init).def(MpcControllerPythonVisitor<MpcController>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
        ENABLE_SPECIFIC_MATRIX_TYPE(Matrix242);
    }
};
void exposeMpcController() { MpcControllerPythonVisitor<MpcController>::expose(); }
