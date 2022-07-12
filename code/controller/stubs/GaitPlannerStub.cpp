#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

#include "GaitPlanner.hpp"

namespace bp = boost::python;

template <typename GaitPlanner>
struct GaitPythonVisitor : public bp::def_visitor<GaitPythonVisitor<GaitPlanner>>
{
    template <class PyClassGait>
    void visit(PyClassGait& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            //.def("getCurrentGait", &Gait::getCurrentGait, "Get currentGait_ matrix.\n")

			.def("isNewPhase", &GaitPlanner::isNewPhase)
            .def("getIsStatic", &GaitPlanner::getIsStatic)
            .def("getCurrentGaitType", &GaitPlanner::getCurrentGaitType)
            .def("getCurrentGaitTypeInt", &GaitPlanner::getCurrentGaitTypeInt)
				.def("getPrevGaitType", &GaitPlanner::getPrevGaitType)
            .def("getPrevGaitTypeInt", &GaitPlanner::getPrevGaitTypeInt)

            .def("initialize", &GaitPlanner::initialize, bp::args("params", "gait"))

            // Update current gait matrix from Python
            .def("update", &GaitPlanner::update, bp::args("rollGait","targetGaitType"))
            .def_readwrite("matrix", &GaitPlanner::currentGait);

    }

    static void expose()
    {
        bp::class_<GaitPlanner>("Gait", bp::no_init).def(GaitPythonVisitor<GaitPlanner>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposeGait() { GaitPythonVisitor<GaitPlanner>::expose(); }
