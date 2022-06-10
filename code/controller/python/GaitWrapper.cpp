#include "Gait.hpp"
#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;

template <typename Gait>
struct GaitPythonVisitor : public bp::def_visitor<GaitPythonVisitor<Gait>>
{
    template <class PyClassGait>
    void visit(PyClassGait& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("getCurrentGait", &Gait::getCurrentGait, "Get currentGait_ matrix.\n")
            .def("isNewPhase", &Gait::isNewPhase, "Get newPhase_ boolean.\n")
            .def("getIsStatic", &Gait::getIsStatic, "Get is_static_ boolean.\n")
            .def("getCurrentGaitType", &Gait::getCurrentGaitType, "Get getCurrentGaitType.\n")
            .def("getPrevGaitType", &Gait::getPrevGaitType, "Get getPrevGaitType.\n")


            .def("initialize", &Gait::initialize, bp::args("params"),
                 "Initialize Gait from Python.\n")

            // Update current gait matrix from Python
            .def("updateGait", &Gait::updateGait, bp::args("rollGait", "q", "targetGaitType"),
                 "Update current gait matrix from Python.\n");
    }

    static void expose()
    {
        bp::class_<Gait>("Gait", bp::no_init).def(GaitPythonVisitor<Gait>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposeGait() { GaitPythonVisitor<Gait>::expose(); }
