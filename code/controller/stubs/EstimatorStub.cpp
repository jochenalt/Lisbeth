#include "Estimator.hpp"
#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;

template <typename Estimator>
struct EstimatorPythonVisitor : public bp::def_visitor<EstimatorPythonVisitor<Estimator>>
{
    template <class PyClassEstimator>
    void visit(PyClassEstimator& cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))
        	  .def("initialize",&Estimator::initialize, bp::args("params", "gait"))
           .def("update_reference_state", &Estimator::updateReferenceState, bp::args("v_ref"))
			  .def("get_q_estimate", &Estimator::getQEstimate)
			  .def("get_v_estimate", &Estimator::getVEstimate)
			  .def("get_v_security", &Estimator::getVSecurity)
	        .def("get_feet_status", &Estimator::getFeetStatus)
	        .def("get_feet_targets", &Estimator::getFeetTargets)
			  .def("get_q_reference", &Estimator::getQReference)
			  .def("get_v_reference", &Estimator::getVReference)
			  .def("get_base_vel_ref", &Estimator::getBaseVelRef)
			  .def("get_base_acc_ref", &Estimator::getBaseAccRef)
	        .def("get_h_v", &Estimator::getHV, "")
	        .def("get_v_filtered", &Estimator::getVFiltered)
	        .def("get_h_v_filtered", &Estimator::getHVFiltered)

	        .def("get_oRh", &Estimator::getoRh)
	        .def("get_hRb", &Estimator::gethRb)
	        .def("get_oTh", &Estimator::getoTh)

		     .def("getImuRPY",&Estimator::getImuRPY)
		     .def("isSteady",&Estimator::isSteady)
			  .def("run",&Estimator::run,
					   bp::args("targets",
									"baseLinearAcceleration", "baseAngularVelocity", "baseOrientation","baseOrientationQuad",
									"q", "v"));
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

