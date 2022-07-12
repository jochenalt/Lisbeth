#include "FootstepPlanner.hpp"
#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;


template <typename FootstepPlanner>
struct FootstepPlannerPythonVisitor : public bp::def_visitor<FootstepPlannerPythonVisitor<FootstepPlanner>>
{
    template <class PyClassFootstepPlanner>
    void visit(PyClassFootstepPlanner& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))
            .def("getFootsteps", &FootstepPlanner::getFootsteps)
            .def("initialize", &FootstepPlanner::initialize, bp::args("params", "gaitIn"))
            .def("updateFootsteps", &FootstepPlanner::updateFootsteps, bp::args("refresh",  "q", "b_v", "b_vref"));

    }

    static void expose()
    {
        bp::class_<FootstepPlanner>("FootstepPlanner", bp::no_init).def(FootstepPlannerPythonVisitor<FootstepPlanner>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
        ENABLE_SPECIFIC_MATRIX_TYPE(Vector12);
        ENABLE_SPECIFIC_MATRIX_TYPE(Matrix34);
        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN12);

    }
};
void exposeFootstepPlanner() { FootstepPlannerPythonVisitor<FootstepPlanner>::expose(); }
