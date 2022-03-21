#include "Gait.hpp"

Gait::Gait()
    : pastGait_()
    , currentGait_()
    , desiredGait_()
    , dt_(0.0)
    , T_gait_(0.0)
    , T_mpc_(0.0)
    , remainingTime_(0.0)
    , newPhase_(false)
    , is_static_(true)
    , q_static_(VectorN::Zero(19))
    , currentGaitType_(GaitType::NoGait)
    , prevGaitType_(GaitType::NoGait)
    , subGait (GaitType::Walking)
{
}


void Gait::initialize(double dt_in, double T_gait_in, double T_mpc_in, int N_gait)
{
    dt_ = dt_in;
    T_gait_ = T_gait_in;
    T_mpc_ = T_mpc_in;
    n_steps_ = (int)std::lround(T_mpc_in / dt_in);

    pastGait_ = MatrixN::Zero(N_gait, 4);
    currentGait_ = MatrixN::Zero(N_gait, 4);
    desiredGait_ = MatrixN::Zero(N_gait, 4);

    if((n_steps_ > N_gait) || ((int)std::lround(T_gait_in / dt_in) > N_gait))
        throw std::invalid_argument("Sizes of matrices are too small for considered durations. Increase N_gait in config file.");

    create_trot();
    create_gait_f();

    is_static_ = false;
}


void Gait::create_walk()
{
    // Number of timesteps in 1/4th period of gait
    int N = (int)std::lround(0.25 * T_gait_ / dt_);

    desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);

    Eigen::Matrix<double, 1, 4> sequence;
    sequence << 0.0, 1.0, 1.0, 1.0;
    desiredGait_.block(0, 0, N, 4) = sequence.colwise().replicate(N);
    sequence << 1.0, 0.0, 1.0, 1.0;
    desiredGait_.block(N, 0, N, 4) = sequence.colwise().replicate(N);
    sequence << 1.0, 1.0, 0.0, 1.0;
    desiredGait_.block(2*N, 0, N, 4) = sequence.colwise().replicate(N);
    sequence << 1.0, 1.0, 1.0, 0.0;
    desiredGait_.block(3*N, 0, N, 4) = sequence.colwise().replicate(N);
}

void Gait::create_trot()
{
    // Number of timesteps in a half period of gait
    int N = (int)std::lround(0.5 * T_gait_ / dt_);

    desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);

    Eigen::Matrix<double, 1, 4> sequence;
    sequence << 1.0, 0.0, 0.0, 1.0;
    desiredGait_.block(0, 0, N, 4) = sequence.colwise().replicate(N);
    sequence << 0.0, 1.0, 1.0, 0.0;
    desiredGait_.block(N, 0, N, 4) = sequence.colwise().replicate(N);
}

void Gait::create_pacing()
{
    // Number of timesteps in a half period of gait
    int N = (int)std::lround(0.5 * T_gait_ / dt_);

    desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);

    Eigen::Matrix<double, 1, 4> sequence;
    sequence << 1.0, 0.0, 1.0, 0.0;
    desiredGait_.block(0, 0, N, 4) = sequence.colwise().replicate(N);
    sequence << 0.0, 1.0, 0.0, 1.0;
    desiredGait_.block(N, 0, N, 4) = sequence.colwise().replicate(N);
}

void Gait::create_bounding()
{
    // Number of timesteps in a half period of gait
    int N = (int)std::lround(0.5 * T_gait_ / dt_);

    desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);

    Eigen::Matrix<double, 1, 4> sequence;
    sequence << 1.0, 1.0, 0.0, 0.0;
    desiredGait_.block(0, 0, N, 4) = sequence.colwise().replicate(N);
    sequence << 0.0, 0.0, 1.0, 1.0;
    desiredGait_.block(N, 0, N, 4) = sequence.colwise().replicate(N);
}

void Gait::create_static()
{
    // Number of timesteps in a period of gait
    int N = (int)std::lround(T_gait_ / dt_);

    desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);

    Eigen::Matrix<double, 1, 4> sequence;
    sequence << 1.0, 1.0, 1.0, 1.0;
    desiredGait_.block(0, 0, N, 4) = sequence.colwise().replicate(N);
}

void Gait::create_gait_f()
{
    int i = 0;

    // Fill currrent gait matrix
    for (int j = 0; j < n_steps_; j++)
    {
        currentGait_.row(j) = desiredGait_.row(i);
        i++;
        if (desiredGait_.row(i).isZero())
        {
            i = 0;
        }  // Loop back if T_mpc_ longer than gait duration
    }

    // Get index of first empty line
    int index = 1;
    while (!desiredGait_.row(index).isZero())
    {
        index++;
    }

    // Age desired gait to take into account what has been put in the current gait matrix
    for (int k = 0; k < i; k++)  
    {
        for (int m = 0; m < index-1; m++) // TODO: Find optimized circular shift function
        {
            desiredGait_.row(m).swap(desiredGait_.row(m+1));
        }       
    }
}

double Gait::getPhaseDuration(int gaitPhaseIdx, int legNo, FootPhase footPhase)
{
    double t_phase = 1;
    int a = gaitPhaseIdx;

    // Looking for the end of the swing/stance phase in currentGait_
    while ((!currentGait_.row(gaitPhaseIdx + 1).isZero()) && (currentGait_(gaitPhaseIdx + 1, legNo) == (double)footPhase))
    {
    	gaitPhaseIdx++;
        t_phase++;
    }
    // If we reach the end of currentGait_ we continue looking for the end of the swing/stance phase in desiredGait_
    if (currentGait_.row(gaitPhaseIdx + 1).isZero())
    {
        int k = 0;
        while ((!desiredGait_.row(k).isZero()) && (desiredGait_(k, legNo) == (double)footPhase))
        {
            k++;
            t_phase++;
        }
    }
    // We suppose that we found the end of the swing/stance phase either in currentGait_ or desiredGait_
    remainingTime_ = t_phase;

    // Looking for the beginning of the swing/stance phase in currentGait_
    while ((a > 0) && (currentGait_(a - 1, legNo) == (double)footPhase))
    {
        a--;
        t_phase++;
    }
    // If we reach the end of currentGait_ we continue looking for the beginning of the swing/stance phase in pastGait_
    if (a == 0)
    {
        while ((!pastGait_.row(a).isZero()) && (pastGait_(a, legNo) == (double)footPhase))
        {
            a++;
            t_phase++;
        }
    }
    // We suppose that we found the beginning of the swing/stance phase either in currentGait_ or pastGait_
    return t_phase * dt_;  // Take into account time step value
}

bool Gait::updateGait(int const k,
                      int const k_mpc,
                      VectorN const& q,
					  int targetGaitType)
{
	if ((targetGaitType != GaitType::NoGait) && (currentGaitType_ != targetGaitType)) {
		changeGait (targetGaitType, q);
	}

	if (k % k_mpc == 0) {
        rollGait();
        return true;
    }

	return false;
}

bool Gait::changeGait(int targetGait, VectorN const& q)
{
    is_static_ = false;
    if (targetGait == GaitType::Pacing)
    {
    	std::cout << "change to pacing gait" << std::endl;
    	create_pacing();
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;
    }
    else if (targetGait == GaitType::Bounding)
    {
    	std::cout << "change to bounding gait" << std::endl;

    	prevGaitType_ = currentGaitType_;
    	create_bounding();
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;
    }
    else if (targetGait == GaitType::Trot)
    {
    	std::cout << "change to trot gait" << std::endl;
    	create_trot();
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;
    }
    else if (targetGait == GaitType::Walking)
    {
    	std::cout << "change to walking gait" << std::endl;
    	create_walk();
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;
    }
    else if (targetGait == GaitType::Static)
    {
        create_static();
        q_static_.head(7) = q.head(7);
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;

        // @JA is_static has some consequences that lead to hiccups in the gait change, state update is not done properly anymore
        // is_static_ = true;
    }

    // if we change from static to any gait,
    // do a fast forward in order to ensure that we start right away
    if ((prevGaitType_ == GaitType::Static) && (targetGait != GaitType::Static)) {
    	std::cout << "change from static to gait, fast forward" << std::endl;

    	while (!isNewPhase())
    		rollGait();
    }

    return is_static_;
}

void Gait::rollGait()
{
    // Transfer current gait into past gait
    for (int m = n_steps_; m > 0; m--) // TODO: Find optimized circular shift function
    {
        pastGait_.row(m).swap(pastGait_.row(m-1));
    }
    pastGait_.row(0) = currentGait_.row(0);

    
    // Entering new contact phase, store positions of feet that are now in contact
    newPhase_ =!currentGait_.row(0).isApprox(currentGait_.row(1));

    // Age current gait
    int index = 1;
    while (!currentGait_.row(index).isZero())
    {
    	currentGait_.row(index-1).swap(currentGait_.row(index));
        index++;
    }

    // Insert a new line from desired gait into current gait
    currentGait_.row(index-1) = desiredGait_.row(0);

    // Age desired gait
    index = 1;
    while (!desiredGait_.row(index).isZero())
    {
    	desiredGait_.row(index-1).swap(desiredGait_.row(index));
        index++;
    }
}

