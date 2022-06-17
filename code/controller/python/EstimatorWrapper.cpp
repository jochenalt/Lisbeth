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
        	.def("initialize",&Estimator::initialize, bp::args("params"),
        		 "initialize")
		    .def("getVEstimate",&Estimator::getVEstimate, "getVEstimate")
		    .def("getQEstimate",&Estimator::getQEstimate, "getQEstimate")
		    .def("getHVFiltered",&Estimator::getHVFiltered, "getHVFiltered")
		    .def("getBaseVelRef",&Estimator::getBaseVelRef, "getBaseVelRef")


		    .def("getImuRPY",&Estimator::getImuRPY, "getImuRPY")
		    .def("getVSecurity",&Estimator::getVSecurity, "getVSecurity")
		    .def("isSteady",&Estimator::isSteady, "isSteady")

			// run one loop of estimator
			.def("run",&Estimator::run, bp::args("k", "gait", "targets",
												 "baseLinearAcceleration", "baseAngularVelocity", "baseOrientation",
												 "q", "v", "perfectPosition","perfectVelocity"),
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

