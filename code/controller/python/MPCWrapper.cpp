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
            .def("run", &MPC::run, bp::args("num_iter", "xref_in", "fsteps_in"), "Run MPC from Python.\n")
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

#include "MpcWrapper.hpp"

template <typename MpcWrapper>
struct MpcWrapperPythonVisitor : public bp::def_visitor<MpcWrapperPythonVisitor<MpcWrapper>>
{
    template <class PyClassGait>
    void visit(PyClassGait& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("initialize", &MpcWrapper::initialize, bp::args("params"),
                 "Initialize MPC from Python.\n")
            .def("get_latest_result", &MpcWrapper::get_latest_result)
        	.def("solve", &MpcWrapper::solve);
    }

    static void expose()
    {
        bp::class_<MpcWrapper>("MpcWrapper", bp::no_init).def(MpcWrapperPythonVisitor<MpcWrapper>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposeMpcWrapper() { MpcWrapperPythonVisitor<MpcWrapper>::expose(); }
