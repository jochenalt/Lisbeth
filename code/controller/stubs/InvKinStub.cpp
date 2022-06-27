#include "InvKin.hpp"

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;

/////////////////////////////////
/// Binding InvKin class
/////////////////////////////////
template <typename InvKin>
struct InvKinPythonVisitor : public bp::def_visitor<InvKinPythonVisitor<InvKin>>
{
    template <class PyClassInvKin>
    void visit(PyClassInvKin& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))


            .def("get_q_step", &InvKin::get_q_step, "Get velocity goals matrix.\n")
            .def("get_dq_cmd", &InvKin::get_dq_cmd, "Get acceleration goals matrix.\n")
            .def("get_ddq_cmd", &InvKin::get_ddq_cmd, "")
			.def("get_q_cmd", &InvKin::get_q_cmd, "")
			.def("get_foot_id", &InvKin::get_foot_id, "")


			.def("initialize", &InvKin::initialize, "initialize\n")

            // Run InvKin from Python
            .def("run", &InvKin::run,
            		bp::args("q","dq","contacts", "pgoals", "vgoals", "agoals"),
						"Run InvKin from Python.\n");
    }

    static void expose()
    {
        bp::class_<InvKin>("InvKin", bp::no_init).def(InvKinPythonVisitor<InvKin>());
        ENABLE_SPECIFIC_MATRIX_TYPE(Matrix43);

        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};

void exposeInvKin() { InvKinPythonVisitor<InvKin>::expose(); }

