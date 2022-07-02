#include "Params.hpp"
#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;

template <typename Params>
struct ParamsPythonVisitor : public bp::def_visitor<ParamsPythonVisitor<Params>>
{
    template <class PyClassParams>
    void visit(PyClassParams& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

        .def("initialize", &Params::initialize, bp::args("file_path"))

         .def_readwrite("interface", &Params::interface)
			.def_readwrite("DEMONSTRATION", &Params::DEMONSTRATION)
			.def_readwrite("SIMULATION", &Params::SIMULATION)
         .def_readwrite("h_ref", &Params::h_ref)
			.def_readwrite("dt_wbc", &Params::dt_wbc)
         .def_readwrite("N_gait", &Params::N_gait)
         .def_readwrite("envID", &Params::envID)
         .def_readwrite("velID", &Params::velID)
         .def_readwrite("dt_mpc", &Params::dt_mpc)
         .def_readwrite("T_mpc", &Params::T_mpc)
         .def_readwrite("N_SIMULATION", &Params::N_SIMULATION)
         .def_readwrite("use_flat_plane", &Params::use_flat_plane)
         .def_readwrite("predefined_vel", &Params::predefined_vel)
         .def_readwrite("enable_pyb_GUI", &Params::enable_pyb_GUI)
        	.def_readwrite("N_periods", &Params::N_periods)
        	.def_readwrite("gait", &Params::gait)
        	.def_readwrite("CoM_offset", &Params::CoM_offset)
			.def_readwrite("shoulders", &Params::shoulders)
        	.def_readwrite("footsteps_init", &Params::footsteps_init)
        	.def_readwrite("footsteps_under_shoulders", &Params::footsteps_under_shoulders)
        	.def_readwrite("I_mat", &Params::I_mat)

			.def_readwrite("q_init", &Params::q_init);

    }

    static void expose()
    {
        bp::class_<Params>("Params", bp::no_init).def(ParamsPythonVisitor<Params>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposeParams() { ParamsPythonVisitor<Params>::expose(); }
