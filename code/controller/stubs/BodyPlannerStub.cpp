#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include "../include/BodyPlanner.hpp"

namespace bp = boost::python;


/////////////////////////////////
/// Binding StatePlanner class
/////////////////////////////////
template <typename BodyPlanner>
struct StatePlannerPythonVisitor : public bp::def_visitor<StatePlannerPythonVisitor<BodyPlanner>>
{
    template <class PyClassStatePlanner>
    void visit(PyClassStatePlanner& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("getReferenceStates", &BodyPlanner::getBodyTrajectory, "Get xref matrix.\n")

            .def("initialize", &BodyPlanner::setup, bp::args("params", "gait"),
                 "Initialize StatePlanner from Python.\n")

            // Run StatePlanner from Python
            .def("computeReferenceStates", &BodyPlanner::update, bp::args("q", "v", "b_vref"),
                 "Run StatePlanner from Python.\n");
    }

    static void expose()
    {
        bp::class_<BodyPlanner>("StatePlanner", bp::no_init).def(StatePlannerPythonVisitor<BodyPlanner>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
        ENABLE_SPECIFIC_MATRIX_TYPE(Matrix12N);

    }
};
void exposeStatePlanner() { StatePlannerPythonVisitor<BodyPlanner>::expose(); }
