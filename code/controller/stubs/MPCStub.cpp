#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include "MPCController.hpp"

namespace bp = boost::python;


template <typename MPCController>
struct MpcControllerPythonVisitor : public bp::def_visitor<MpcControllerPythonVisitor<MPCController>>
{
    template <class PyClassGait>
    void visit(PyClassGait& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

        .def("initialize", &MPCController::initialize, bp::args("params"))
        .def("get_latest_result", &MPCController::get_latest_result)
        .def("solve", &MPCController::solve, bp::args("fsteps","gait", "fsteps_in"))
    	  .def("is_ready", &MPCController::is_ready)
		  .def("get_avr_time_us", &MPCController::get_avr_time_us);
    }

    static void expose()
    {
        bp::class_<MPCController>("MpcController", bp::no_init).def(MpcControllerPythonVisitor<MPCController>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
        ENABLE_SPECIFIC_MATRIX_TYPE(Matrix242);
        ENABLE_SPECIFIC_MATRIX_TYPE(Matrix12N);


    }
};
void exposeMpcController() { MpcControllerPythonVisitor<MPCController>::expose(); }
