#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>

extern void exposeInvKin();
extern void exposeParams();
extern void exposeEstimator();
extern void exposeFootstepPlanner();
extern void exposeFootTrajectoryGenerator();
extern void exposeGait();
extern void exposeQPWBC();
extern void exposeStatePlanner();
extern void exposeWbcController();
extern void exposeFilter();
extern void exposeMpcController();
extern void exposeController();


BOOST_PYTHON_MODULE(libcontroller_core)
{

    eigenpy::enableEigenPy();

    exposeMpcController();
    exposeStatePlanner();
    exposeGait();
    exposeFootstepPlanner();
    exposeFootTrajectoryGenerator();
    exposeInvKin();
    exposeParams();
    exposeEstimator();
    exposeWbcController();
    exposeFilter();
    exposeController();
}
