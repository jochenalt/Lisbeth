#include "Gait.hpp"

Gait::Gait()
    : pastGait_()
    , currentGait_()
    , desiredGait_()
    , dt_(0.0)
	, nRows_(0)
    , T_gait_(0.0)
    , T_mpc_(0.0)
    , remainingTime_(0.0)
    , newPhase_(false)
    , is_static_(true)
    , currentGaitType_(GaitType::NoGait)
    , prevGaitType_(GaitType::NoGait)
    , subGait (GaitType::Walking)
{
}


void Gait::initialize(Params &params)
{

	GaitType gaitType = GaitType::NoMovement;
	if (params.dt_mpc == 0)
        throw std::invalid_argument("passed dt_in == 0");

    dt_ = params.dt_mpc;
    nRows_ = (int)params.gait.rows();
    T_gait_ = params.T_gait;
    T_mpc_ = params.T_mpc;
    n_steps_ = (int)std::lround(params.T_mpc / dt_);

    pastGait_ = MatrixN::Zero(params.N_gait, 4);
    currentGait_ = MatrixN::Zero(params.N_gait, 4);
    desiredGait_ = MatrixN::Zero(params.N_gait, 4);

    if((n_steps_ > params.N_gait) || ((int)std::lround(params.T_gait / dt_) > params.N_gait))
        throw std::invalid_argument("Sizes of matrices are too small for considered durations. Increase N_gait in config file.");

    is_static_ = false;
    if (gaitType == GaitType::Pacing)
    	createPacing();
    else if (gaitType == GaitType::Bounding)
     	createBounding();
    else if (gaitType == GaitType::Trot)
    	createTrot();
    else if (gaitType == GaitType::Walking)
    	createWalk();
    else if (gaitType == GaitType::WalkingTrot)
    	createWalkingTrot();
    else if (gaitType == GaitType::CustomGallop)
    	createCustomGallop();
    else if (gaitType == GaitType::NoMovement) {
    	createStatic();
    }

    create_gait_f();
    rollGait();
}

void Gait::createWalk()
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

void Gait::createTrot()
{
    // Number of timesteps in a half period of gait
    int N = (int)std::lround(0.5 * T_gait_ / dt_);
    desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);

    RowVector4 sequence;
    sequence << 1.0, 0.0, 0.0, 1.0;
    desiredGait_.block(0, 0, N, 4) = sequence.colwise().replicate(N);
    sequence << 0.0, 1.0, 1.0, 0.0;
    desiredGait_.block(N, 0, N, 4) = sequence.colwise().replicate(N);
}

void Gait::createPacing()
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

void Gait::createBounding()
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

void Gait::createWalkingTrot() {
  int N = (int)std::lround(0.25 * T_gait_ / dt_);
  desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);

  Eigen::Matrix<double, 1, 4> sequence;

  sequence << 1., 0., 0., 1.;
  desiredGait_.block(0, 0, N, 4) = sequence.colwise().replicate(N);
  sequence << 1., 1., 1., 1.;
  desiredGait_.block(N, 0, N, 4) = sequence.colwise().replicate(N);
  sequence << 0., 1., 1., 0.;
  desiredGait_.block(2*N, 0, N, 4) = sequence.colwise().replicate(N);
  sequence << 1., 1., 1., 1.;
  desiredGait_.block(3*N, 0, N, 4) = sequence.colwise().replicate(N);

}


void Gait::createCustomGallop() {
  int N = (int)std::lround(0.25 * T_gait_ / dt_);
  desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);

  Eigen::Matrix<double, 1, 4> sequence;
  sequence << 1., 0., 1., 0.;
  desiredGait_.block(0, 0, N, 4) = sequence.colwise().replicate(N);
  sequence << 1., 0., 0., 1.;
  desiredGait_.block(N, 0, N, 4) = sequence.colwise().replicate(N);
  sequence << 0., 1., 0., 1.;
  desiredGait_.block(2 * N, 0, N, 4) = sequence.colwise().replicate(N);
  sequence << 0., 1., 1., 0.;
  desiredGait_.block(3 * N, 0, N, 4) = sequence.colwise().replicate(N);
}

void Gait::createStatic()
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
double Gait::getElapsedTime(int i, int j) {
  double state = currentGait_(i, j);
  double nPhase = 0;
  int row = i;

  // Looking for the beginning of the swing/stance phase in currentGait_
  while ((row > 0) && (currentGait_(row - 1, j) == state)) {
    row--;
    nPhase++;
  }

  // If we reach the end of currentGait_ we continue looking for the beginning of the swing/stance phase in pastGait_
  if (row == 0) {
	  /*
    row = nRows_;
    while ((row > 0) && (pastGait_(row - 1, j) == state)) {
      row--;
      nPhase++;
    }
    */
    while ((!pastGait_.row(row).isZero()) && (pastGait_(row, j) == state))
    {
    	row++;
        nPhase++;
    }
  }
  return nPhase * dt_;
}

double Gait::getPhaseDurationCoeff(int i, int j) { return getElapsedTime(i, j) + getRemainingTimeCoeff(i, j); }

double Gait::getRemainingTimeCoeff(int i, int j) {
  double state = currentGait_(i, j);
  double nPhase = 1;
  int row = i;
  // Looking for the end of the swing/stance phase in currentGait_
  while ((row < nRows_ - 1) && (currentGait_(row + 1, j) == state)) {
    row++;
    nPhase++;
  }
  // If we reach the end of currentGait_ we continue looking for the end of the swing/stance phase in desiredGait_
  if (currentGait_.row(i + 1).isZero()) {
  // if (row == nRows_ - 1) {
    row = 0;
    while ((row < nRows_) && (desiredGait_(row, j) == state)) {
      row++;
      nPhase++;
    }
  }
  return nPhase * dt_;
}


bool Gait::updateGait(bool initiateNewGait,
					  int targetGaitType)
{
	if ((targetGaitType != GaitType::NoGait) && (currentGaitType_ != targetGaitType)) {
		changeGait (targetGaitType);
	}

	if (initiateNewGait) {
        rollGait();
        return true;
    }

	return false;
}

bool Gait::changeGait(int targetGait)
{
    is_static_ = false;
    if (targetGait == GaitType::Pacing)
    {
    	std::cout << "change to pacing gait" << std::endl;
    	createPacing();
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;
    }
    else if (targetGait == GaitType::Bounding)
    {
    	std::cout << "change to bounding gait" << std::endl;

    	prevGaitType_ = currentGaitType_;
    	createBounding();
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;
    }
    else if (targetGait == GaitType::Trot)
    {
    	std::cout << "change to trot gait" << std::endl;
    	createTrot();
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;
    }
    else if (targetGait == GaitType::Walking)
    {
    	std::cout << "change to walking gait" << std::endl;
    	createWalk();
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;
    }
    else if (targetGait == GaitType::WalkingTrot)
    {
    	std::cout << "change to walking trot " << std::endl;
    	createWalkingTrot();
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;
    }
    else if (targetGait == GaitType::CustomGallop)
    {
    	std::cout << "change to custom gallo" << std::endl;
    	createCustomGallop();
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;
    }
    else if (targetGait == GaitType::NoMovement)
    {
        createStatic();
    	prevGaitType_ = currentGaitType_;
        currentGaitType_ = (GaitType)targetGait;

        // @JA is_static has some consequences that lead to hiccups in the gait change, state update is not done properly anymore
        // is_static_ = true;
    }


    // if we change from static to any gait,
    // do a fast forward in order to ensure that we start right away
    if ((prevGaitType_ == GaitType::NoMovement) && (targetGait != GaitType::NoMovement )) {
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

