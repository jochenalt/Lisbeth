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
extern void exposeMPC();
extern void exposeWbcWrapper();
extern void exposeFilter();
extern void exposeMpcWrapper();


BOOST_PYTHON_MODULE(libcontroller_core)
{

    eigenpy::enableEigenPy();

    exposeMPC();
    exposeStatePlanner();
    exposeGait();
    exposeFootstepPlanner();
    exposeFootTrajectoryGenerator();
    exposeInvKin();
    exposeParams();
    exposeEstimator();
    exposeWbcWrapper();
    exposeFilter();
    exposeMpcWrapper();
}
