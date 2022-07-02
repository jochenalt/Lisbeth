#include "Gait.hpp"

Gait::Gait() :
		pastGait_(),
		currentGait_(),
		desiredGait_(),
		remainingTime_(0.0),
		newPhase_(false),
		is_static_(true),
		currentGaitType_(GaitType::NoGait),
		prevGaitType_(GaitType::NoGait),
		subGait(GaitType::Walking)
{
}

void Gait::initialize(Params &params_in)
{
	params = &params_in;

	// create the arrays big enough, during runtime we only need params->N_steps * params->N_periods
	pastGait_ = MatrixN4::Zero(params->N_gait, 4);
	currentGait_ = MatrixN4::Zero(params->N_gait, 4);
	desiredGait_ = MatrixN4::Zero(params->N_gait, 4);

	is_static_ = false;
	createStatic();
	currentGait_ = desiredGait_;
}

void Gait::setGait(int period, int pos,int sequences, MatrixN4 & gait, std::string sequence) {
	int repetition = params->get_N_steps() / sequences;

	RowVector4 gaitSequence;
	gaitSequence << (sequence[0]-'0'), sequence[1]-'0', sequence[2]-'0',sequence[3]-'0';

	gait.block(period*sequences*repetition + repetition*pos, 0, repetition, 4) = gaitSequence.colwise().replicate(repetition);
}

void Gait::createWalk()
{
	desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(i,0,4,desiredGait_, "0111");
		setGait(i,1,4,desiredGait_, "1011");
		setGait(i,2,4,desiredGait_, "1101");
		setGait(i,3,4,desiredGait_, "1110");
	}

	exit(1);
}

void Gait::createTrot()
{
	desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(i,0,2,desiredGait_, "1001");
		setGait(i,1,2,desiredGait_, "0110");
	}
}

void Gait::createPacing()
{
	desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(i,0,2,desiredGait_, "1010");
		setGait(i,1,2,desiredGait_, "0101");
	}
}

void Gait::createBounding()
{
	desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(i,0,2,desiredGait_, "1100");
		setGait(i,1,2,desiredGait_, "0011");
	}
}

void Gait::createWalkingTrot()
{
	desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(i,0,4,desiredGait_, "1001");
		setGait(i,1,4,desiredGait_, "1111");
		setGait(i,2,4,desiredGait_, "0110");
		setGait(i,3,4,desiredGait_, "1111");
	}
}

void Gait::createCustomGallop()
{
	desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(i,0,4,desiredGait_, "1010");
		setGait(i,1,4,desiredGait_, "1001");
		setGait(i,2,4,desiredGait_, "0101");
		setGait(i,3,4,desiredGait_, "0110");
	}
}

void Gait::createStatic()
{
	// Number of timesteps in a period of gait
	desiredGait_ = MatrixN::Zero(currentGait_.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(i,0,1,desiredGait_, "1111");
	}
}

double Gait::getPhaseDuration(int gaitPhaseIdx, int legNo, FootPhase footPhase)
{
	double t_phase = 1;
	int a = gaitPhaseIdx;

	// Looking for the end of the swing/stance phase in currentGait_
	while ((!currentGait_.row(gaitPhaseIdx + 1).isZero())
			&& (currentGait_(gaitPhaseIdx + 1, legNo) == (double) footPhase))
	{
		gaitPhaseIdx++;
		t_phase++;
	}
	// If we reach the end of currentGait_ we continue looking for the end of the swing/stance phase in desiredGait_
	if (currentGait_.row(gaitPhaseIdx + 1).isZero())
	{
		int k = 0;
		while ((!desiredGait_.row(k).isZero())
				&& (desiredGait_(k, legNo) == (double) footPhase))
		{
			k++;
			t_phase++;
		}
	}
	// We suppose that we found the end of the swing/stance phase either in currentGait_ or desiredGait_
	remainingTime_ = t_phase;

	// Looking for the beginning of the swing/stance phase in currentGait_
	while ((a > 0) && (currentGait_(a - 1, legNo) == (double) footPhase))
	{
		a--;
		t_phase++;
	}
	// If we reach the end of currentGait_ we continue looking for the beginning of the swing/stance phase in pastGait_
	if (a == 0)
	{
		while ((!pastGait_.row(a).isZero())
				&& (pastGait_(a, legNo) == (double) footPhase))
		{
			a++;
			t_phase++;
		}
	}
	// We suppose that we found the beginning of the swing/stance phase either in currentGait_ or pastGait_
	return t_phase * params->dt_mpc;  // Take into account time step value
}
double Gait::getElapsedTime(int i, int j)
{
	double state = currentGait_(i, j);
	double nPhase = 0;
	int row = i;

	// Looking for the beginning of the swing/stance phase in currentGait_
	while ((row > 0) && (currentGait_(row - 1, j) == state))
	{
		row--;
		nPhase++;
	}

	// If we reach the end of currentGait_ we continue looking for the beginning of the swing/stance phase in pastGait_
	if (row == 0)
	{
		while ((!pastGait_.row(row).isZero()) && (pastGait_(row, j) == state))
		{
			row++;
			nPhase++;
		}
	}
	return nPhase * params->dt_mpc;
}

double Gait::getPhaseDuration(int i, int j)
{
	return getElapsedTime(i, j) + getRemainingTime(i, j);
}

double Gait::getRemainingTime(int i, int j)
{
	double state = currentGait_(i, j);
	double nPhase = 1;
	int row = i;
	// Looking for the end of the swing/stance phase in currentGait_
	while ((row < params->get_N_steps()  - 1) && (currentGait_(row + 1, j) == state))
	{
		row++;
		nPhase++;
	}
	// If we reach the end of currentGait_ we continue looking for the end of the swing/stance phase in desiredGait_
	if (currentGait_.row(i + 1).isZero())
	{
		// if (row == nRows_ - 1) {
		row = 0;
		while ((row < params->get_N_steps() ) && (desiredGait_(row, j) == state))
		{
			row++;
			nPhase++;
		}
	}
	return nPhase * params->dt_mpc;
}

bool Gait::update(bool initiateNewGait, int targetGaitType)
{
	if ((targetGaitType != GaitType::NoGait)
			&& (currentGaitType_ != targetGaitType))
	{
		changeGait(targetGaitType);
	}

	if (initiateNewGait)
	{
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
		currentGaitType_ = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::Bounding)
	{
		std::cout << "change to bounding gait" << std::endl;

		prevGaitType_ = currentGaitType_;
		createBounding();
		prevGaitType_ = currentGaitType_;
		currentGaitType_ = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::Trot)
	{
		std::cout << "change to trot gait" << std::endl;
		createTrot();
		prevGaitType_ = currentGaitType_;
		currentGaitType_ = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::Walking)
	{
		std::cout << "change to walking gait" << std::endl;
		createWalk();
		prevGaitType_ = currentGaitType_;
		currentGaitType_ = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::WalkingTrot)
	{
		std::cout << "change to walking trot " << std::endl;
		createWalkingTrot();
		prevGaitType_ = currentGaitType_;
		currentGaitType_ = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::CustomGallop)
	{
		std::cout << "change to custom gallo" << std::endl;
		createCustomGallop();
		prevGaitType_ = currentGaitType_;
		currentGaitType_ = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::NoMovement)
	{
		createStatic();
		prevGaitType_ = currentGaitType_;
		currentGaitType_ = (GaitType) targetGait;
	}

	// if we change from static to any gait,
	// do a fast forward in order to ensure that we start right away
	if ((prevGaitType_ == GaitType::NoMovement)
			&& (targetGait != GaitType::NoMovement))
	{
		std::cout << "change from static to gait, fast forward" << std::endl;

		while (!isNewPhase())
			rollGait();
	}

	return is_static_;
}

void Gait::rollGait()
{
	// Transfer current gait into past gait
	// shift pastGait from [0..9] to [1..10]
	for (int m = params->get_N_steps(); m > 0; m--)
	{
		pastGait_.row(m).swap(pastGait_.row(m - 1));
	}
	// and assign current gait to [0]
	pastGait_.row(0) = currentGait_.row(0);

	// Entering new contact phase, store positions of feet that are now in contact
	newPhase_ = !currentGait_.row(0).isApprox(currentGait_.row(1));

	// Age current gait
	int index = 1;
	while (!currentGait_.row(index).isZero())
	{
		currentGait_.row(index - 1).swap(currentGait_.row(index));
		index++;
	}

	// Insert a new line from desired gait into current gait
	currentGait_.row(index - 1) = desiredGait_.row(0);

	// Age desired gait
	index = 1;
	while (!desiredGait_.row(index).isZero())
	{
		desiredGait_.row(index - 1).swap(desiredGait_.row(index));
		index++;
	}
}

