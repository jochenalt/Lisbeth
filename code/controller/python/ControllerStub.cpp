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
            .def("compute", &Controller::compute);
    }

    static void expose()
    {
        bp::class_<Controller>("Controller", bp::no_init).def(ControllerPythonVisitor<Controller>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
        ENABLE_SPECIFIC_MATRIX_TYPE(Matrix242);
    }
};
void exposeController() { ControllerPythonVisitor<Controller>::expose(); }
