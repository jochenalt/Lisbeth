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

			.def("isNewPhase", &GaitPlanner::isNewPhase, "Get newPhase_ boolean.\n")
            .def("getIsStatic", &GaitPlanner::getIsStatic, "Get is_static_ boolean.\n")
            .def("getCurrentGaitType", &GaitPlanner::getCurrentGaitType, "Get getCurrentGaitType.\n")
            .def("getCurrentGaitTypeInt", &GaitPlanner::getCurrentGaitTypeInt, "Get getCurrentGaitType.\n")

			.def("getPrevGaitType", &GaitPlanner::getPrevGaitType, "Get getPrevGaitType.\n")
            .def("getPrevGaitTypeInt", &GaitPlanner::getPrevGaitTypeInt, "Get getPrevGaitType.\n")


            .def("initialize", &GaitPlanner::initialize, bp::args("params"),
                 "Initialize Gait from Python.\n")

            // Update current gait matrix from Python
            .def("update", &GaitPlanner::update, bp::args("rollGait","targetGaitType"),
                 "Update current gait matrix from Python.\n")
            .def_readwrite("matrix", &GaitPlanner::currentGait_);

    }

    static void expose()
    {
        bp::class_<GaitPlanner>("Gait", bp::no_init).def(GaitPythonVisitor<GaitPlanner>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposeGait() { GaitPythonVisitor<GaitPlanner>::expose(); }
