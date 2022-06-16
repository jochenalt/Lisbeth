#include "InvKin.hpp"
#include "MPC.hpp"
#include "StatePlanner.hpp"
#include "Gait.hpp"
#include "Kinematics.hpp"
#include "Estimator.hpp"
#include "FootstepPlanner.hpp"
#include "FootTrajectoryGenerator.hpp"
#include "QPWBC.hpp"
#include "Params.hpp"

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
            .def(bp::init<double, int, double, int>(bp::args("dt_in", "n_steps_in", "T_gait_in", "N_gait"),
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

/////////////////////////////////
/// Binding StatePlanner class
/////////////////////////////////
template <typename StatePlanner>
struct StatePlannerPythonVisitor : public bp::def_visitor<StatePlannerPythonVisitor<StatePlanner>>
{
    template <class PyClassStatePlanner>
    void visit(PyClassStatePlanner& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("getReferenceStates", &StatePlanner::getReferenceStates, "Get xref matrix.\n")
            .def("getNSteps", &StatePlanner::getNSteps, "Get number of steps in prediction horizon.\n")

            .def("initialize", &StatePlanner::initialize, bp::args("dt_in", "T_mpc_in", "h_ref_in"),
                 "Initialize StatePlanner from Python.\n")

            // Run StatePlanner from Python
            .def("computeReferenceStates", &StatePlanner::computeReferenceStates, bp::args("q", "v", "b_vref", "z_average"),
                 "Run StatePlanner from Python.\n");
    }

    static void expose()
    {
        bp::class_<StatePlanner>("StatePlanner", bp::no_init).def(StatePlannerPythonVisitor<StatePlanner>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposeStatePlanner() { StatePlannerPythonVisitor<StatePlanner>::expose(); }


/////////////////////////////////
/// Binding WbcWrapper class
/////////////////////////////////
template <typename WbcWrapper>
struct WbcWrapperPythonVisitor : public bp::def_visitor<WbcWrapperPythonVisitor<WbcWrapper>>
{
    template <class PyClassWbcWrapper>
    void visit(PyClassWbcWrapper& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("initialize", &WbcWrapper::initialize, bp::args("params"), "Initialize WbcWrapper from Python.\n")

            .def("get_qdes", &WbcWrapper::get_qdes, "Get qdes_.\n")
            .def("get_vdes", &WbcWrapper::get_vdes, "Get vdes_.\n")
            .def("get_tau_ff", &WbcWrapper::get_tau_ff, "Get tau_ff_.\n")

            .def_readonly("qdes", &WbcWrapper::get_qdes)
            .def_readonly("vdes", &WbcWrapper::get_vdes)
            .def_readonly("tau_ff", &WbcWrapper::get_tau_ff)
            .def_readonly("f_with_delta", &WbcWrapper::get_f_with_delta)
            .def_readonly("feet_pos", &WbcWrapper::get_feet_pos)
            .def_readonly("feet_err", &WbcWrapper::get_feet_err)
            .def_readonly("feet_vel", &WbcWrapper::get_feet_vel)
            .def_readonly("feet_pos_target", &WbcWrapper::get_feet_pos_target)
            .def_readonly("feet_vel_target", &WbcWrapper::get_feet_vel_target)
            .def_readonly("feet_acc_target", &WbcWrapper::get_feet_acc_target)

            // Run WbcWrapper from Python
            .def("compute", &WbcWrapper::compute, bp::args("q", "dq", "f_cmd", "contacts", "pgoals", "vgoals", "agoals"), "Run WbcWrapper from Python.\n");
    }

    static void expose()
    {
        bp::class_<WbcWrapper>("WbcWrapper", bp::no_init).def(WbcWrapperPythonVisitor<WbcWrapper>());

        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};
void exposeWbcWrapper() { WbcWrapperPythonVisitor<WbcWrapper>::expose(); }


extern void exposeInvKin();
extern void exposeParams();
extern void exposeEstimator();
extern void exposeFootstepPlanner();
extern void exposeFootTrajectoryGenerator();
extern void exposeGait();
extern void exposeQPWBC();

/////////////////////////////////
/// Exposing classes
/////////////////////////////////
BOOST_PYTHON_MODULE(libcontroller_core)
{

    eigenpy::enableEigenPy();

    exposeMPC();
    exposeStatePlanner();
    exposeGait();
    exposeFootstepPlanner();
    exposeFootTrajectoryGenerator();
    exposeInvKin();
    exposeQPWBC();
    exposeParams();
    exposeEstimator();
    exposeWbcWrapper();
}
