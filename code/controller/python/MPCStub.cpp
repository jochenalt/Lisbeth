#include "MPC.hpp"

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;


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
