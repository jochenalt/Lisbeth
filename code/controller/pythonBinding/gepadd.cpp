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
/// Binding Gait class
/////////////////////////////////
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
            .def("getRemainingTime", &Gait::getRemainingTime, "Get getRemainingTime.\n")




            .def("initialize", &Gait::initialize, bp::args("dt_in", "T_gait_in", "T_mpc_in", "N_gait", "GaitType"),
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


/////////////////////////////////
/// Estimator class
/////////////////////////////////
template <typename Estimator>
struct EstimatorPythonVisitor : public bp::def_visitor<EstimatorPythonVisitor<Estimator>>
{
    template <class PyClassEstimator>
    void visit(PyClassEstimator& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))
        	.def("initialize",&Estimator::initialize, bp::args("dT", "N_simulation", "h_init", "perfectEstimator"),
        		 "initialize")
		    .def("getQFiltered",&Estimator::getQFiltered, "getQFiltered")
		    .def("getVFiltered",&Estimator::getVFiltered, "getVFiltered")
		    .def("getImuRPY",&Estimator::getImuRPY, "getImuRPY")
		    .def("getVSecu",&Estimator::getVSecu, "getVSecu")
		    .def("isSteady",&Estimator::isSteady, "isSteady")

			// run one loop of estimator
			.def("run",&Estimator::run, bp::args("k", "gait", "goals",
												 "baseLinearAcceleration", "baseAngularVelocity", "baseOrientation",
												 "q", "v", "perfectPosition","perfectVelocity",
												 "baseHeight", "baseVelocity"),
				 "run");
    }

    static void expose()
    {
        bp::class_<Estimator>("Estimator", bp::no_init).def(EstimatorPythonVisitor<Estimator>());

        ENABLE_SPECIFIC_MATRIX_TYPE(VectorN);
        ENABLE_SPECIFIC_MATRIX_TYPE(Vector12);
        ENABLE_SPECIFIC_MATRIX_TYPE(Vector18);
        ENABLE_SPECIFIC_MATRIX_TYPE(Vector19);
        ENABLE_SPECIFIC_MATRIX_TYPE(Vector4);
        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);

    }
};

void exposeEstimator() { EstimatorPythonVisitor<Estimator>::expose(); }


/////////////////////////////////
/// Binding FootstepPlanner class
/////////////////////////////////
template <typename FootstepPlanner>
struct FootstepPlannerPythonVisitor : public bp::def_visitor<FootstepPlannerPythonVisitor<FootstepPlanner>>
{
    template <class PyClassFootstepPlanner>
    void visit(PyClassFootstepPlanner& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("getFootsteps", &FootstepPlanner::getFootsteps, "Get footsteps_ matrix.\n")
            .def("getRz", &FootstepPlanner::getRz, "Get rotation along z matrix.\n")

            .def("initialize", &FootstepPlanner::initialize, bp::args("dt_in", "dt_wbc_in", "T_mpc_in", "h_ref_in", "shouldersIn", "gaitIn", "N_gait"),
                 "Initialize FootstepPlanner from Python.\n")

            // Compute target location of footsteps from Python
            .def("updateFootsteps", &FootstepPlanner::updateFootsteps, bp::args("refresh", "k", "q", "b_v", "b_vref"),
                 "Update and compute location of footsteps from Python.\n");

    }

    static void expose()
    {
        bp::class_<FootstepPlanner>("FootstepPlanner", bp::no_init).def(FootstepPlannerPythonVisitor<FootstepPlanner>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposeFootstepPlanner() { FootstepPlannerPythonVisitor<FootstepPlanner>::expose(); }

/////////////////////////////////
/// Binding FootTrajectoryGenerator class
/////////////////////////////////
template <typename FootTrajectoryGenerator>
struct FootTrajectoryGeneratorPythonVisitor : public bp::def_visitor<FootTrajectoryGeneratorPythonVisitor<FootTrajectoryGenerator>>
{
    template <class PyClassFootTrajectoryGenerator>
    void visit(PyClassFootTrajectoryGenerator& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("getFootPosition", &FootTrajectoryGenerator::getFootPosition, "Get position_ matrix.\n")
            .def("getFootVelocity", &FootTrajectoryGenerator::getFootVelocity, "Get velocity_ matrix.\n")
            .def("getFootAcceleration", &FootTrajectoryGenerator::getFootAcceleration, "Get acceleration_ matrix.\n")

            .def("initialize", &FootTrajectoryGenerator::initialize, bp::args("maxHeightIn", "lockTimeIn", "targetFootstepIn",
                 "initialFootPosition", "dt_tsid_in", "k_mpc_in", "gaitIn"),
                 "Initialize FootTrajectoryGenerator from Python.\n")

            // Compute target location of footsteps from Python
            .def("update", &FootTrajectoryGenerator::update, bp::args("k", "targetFootstep"),
                 "Compute target location of footsteps from Python.\n");

    }

    static void expose()
    {
        bp::class_<FootTrajectoryGenerator>("FootTrajectoryGenerator", bp::no_init).def(FootTrajectoryGeneratorPythonVisitor<FootTrajectoryGenerator>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposeFootTrajectoryGenerator() { FootTrajectoryGeneratorPythonVisitor<FootTrajectoryGenerator>::expose(); }

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
            .def(bp::init<double>(bp::args("dt_in"), "Constructor with parameters."))

            .def("get_q_step", &InvKin::get_q_step, "Get velocity goals matrix.\n")
            .def("get_dq_cmd", &InvKin::get_dq_cmd, "Get acceleration goals matrix.\n")

            // Run InvKin from Python
            .def("refreshAndCompute", &InvKin::refreshAndCompute,
                 bp::args("contacts", "goals", "vgoals", "agoals", "posf", "vf", "wf", "af", "Jf"),
                 "Run InvKin from Python.\n");
    }

    static void expose()
    {
        bp::class_<InvKin>("InvKin", bp::no_init).def(InvKinPythonVisitor<InvKin>());

        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};

void exposeInvKin() { InvKinPythonVisitor<InvKin>::expose(); }

/////////////////////////////////
/// Binding QPWBC class
/////////////////////////////////
template <typename QPWBC>
struct QPWBCPythonVisitor : public bp::def_visitor<QPWBCPythonVisitor<QPWBC>>
{
    template <class PyClassQPWBC>
    void visit(PyClassQPWBC& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("get_f_res", &QPWBC::get_f_res, "Get velocity goals matrix.\n")
            .def("get_ddq_res", &QPWBC::get_ddq_res, "Get acceleration goals matrix.\n")
            .def("get_H", &QPWBC::get_H, "Get H weight matrix.\n")

            // Run QPWBC from Python
            .def("run", &QPWBC::run, bp::args("M", "Jc", "f_cmd", "RNEA", "k_contacts"), "Run QPWBC from Python.\n");
    }

    static void expose()
    {
        bp::class_<QPWBC>("QPWBC", bp::no_init).def(QPWBCPythonVisitor<QPWBC>());

        ENABLE_SPECIFIC_MATRIX_TYPE(matXd);
    }
};
void exposeQPWBC() { QPWBCPythonVisitor<QPWBC>::expose(); }

/////////////////////////////////
/// Binding Params class
/////////////////////////////////
template <typename Params>
struct ParamsPythonVisitor : public bp::def_visitor<ParamsPythonVisitor<Params>>
{
    template <class PyClassParams>
    void visit(PyClassParams& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))

            .def("initialize", &Params::initialize, bp::args("file_path"),
                 "Initialize Params from Python.\n")

            // Read Params from Python
            .def_readwrite("interface", &Params::interface)
            .def_readwrite("SIMULATION", &Params::SIMULATION)
            .def_readwrite("dt_wbc", &Params::dt_wbc)
            .def_readwrite("N_gait", &Params::N_gait)
            .def_readwrite("envID", &Params::envID)
            .def_readwrite("velID", &Params::velID)
            .def_readwrite("dt_mpc", &Params::dt_mpc)
            .def_readwrite("T_gait", &Params::T_gait)
            .def_readwrite("T_mpc", &Params::T_mpc)
            .def_readwrite("N_SIMULATION", &Params::N_SIMULATION)
            .def_readwrite("use_flat_plane", &Params::use_flat_plane)
            .def_readwrite("predefined_vel", &Params::predefined_vel)
            .def_readwrite("enable_pyb_GUI", &Params::enable_pyb_GUI);
    }

    static void expose()
    {
        bp::class_<Params>("Params", bp::no_init).def(ParamsPythonVisitor<Params>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposeParams() { ParamsPythonVisitor<Params>::expose(); }

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

}
