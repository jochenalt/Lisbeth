#include "GaitPlanner.hpp"

GaitPlanner::GaitPlanner() :
		pastGait(),
		currentGait(),
		desiredGait(),
		remainingTime_(0.0),
		newPhase(false),
		is_static(true),
		currentGaitType(GaitType::NoGait),
		prevGaitType_(GaitType::NoGait),
		subGait(GaitType::Walking)
{
}

void GaitPlanner::initialize(Params &params_in)
{
	params = &params_in;

	// create the arrays big enough, during runtime we only need params->N_steps * params->N_periods
	pastGait = MatrixN4::Zero(params->N_gait, 4);
	currentGait = MatrixN4::Zero(params->N_gait, 4);
	desiredGait = MatrixN4::Zero(params->N_gait, 4);

	is_static = false;
	createStatic();
	currentGait = desiredGait;
}

void GaitPlanner::setGait(int i,int n_sequences, MatrixN4 & gait, std::string sequence) {
	int repetition = params->get_N_steps() / (params->N_periods * n_sequences);

	RowVector4 gaitSequence;
	gaitSequence << (sequence[0]-'0'), sequence[1]-'0', sequence[2]-'0',sequence[3]-'0';

	for (int p = 0;p< params->N_periods;p++) {
		gait.block(p*n_sequences*repetition + repetition*i, 0, repetition, 4) = gaitSequence.colwise().replicate(repetition);
	}
}

void GaitPlanner::createWalk()
{
	desiredGait = MatrixN::Zero(currentGait.rows(), 4);
	setGait(0,4,desiredGait, "0111");
	setGait(1,4,desiredGait, "1011");
	setGait(2,4,desiredGait, "1101");
	setGait(3,4,desiredGait, "1110");
}

void GaitPlanner::createTrot()
{
	desiredGait = MatrixN::Zero(currentGait.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(0,2,desiredGait, "1001");
		setGait(1,2,desiredGait, "0110");
	}
}

void GaitPlanner::createPacing()
{
	desiredGait = MatrixN::Zero(currentGait.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(0,2,desiredGait, "1010");
		setGait(1,2,desiredGait, "0101");
	}
}

void GaitPlanner::createBounding()
{
	desiredGait = MatrixN::Zero(currentGait.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(0,2,desiredGait, "1100");
		setGait(1,2,desiredGait, "0011");
	}
}

void GaitPlanner::createWalkingTrot()
{
	desiredGait = MatrixN::Zero(currentGait.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(0,4,desiredGait, "1001");
		setGait(1,4,desiredGait, "1111");
		setGait(2,4,desiredGait, "0110");
		setGait(3,4,desiredGait, "1111");
	}
}

void GaitPlanner::createCustomGallop()
{
	desiredGait = MatrixN::Zero(currentGait.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(0,4,desiredGait, "1010");
		setGait(1,4,desiredGait, "1001");
		setGait(2,4,desiredGait, "0101");
		setGait(3,4,desiredGait, "0110");
	}
}

void GaitPlanner::createStatic()
{
	desiredGait = MatrixN::Zero(currentGait.rows(), 4);
	for (int i = 0;i<params->N_periods;i++) {
		setGait(0,1,desiredGait, "1111");
	}
}

double GaitPlanner::getPhaseDuration(int gaitPhaseIdx, int legNo, FootPhase footPhase)
{
	double t_phase = 1;
	int a = gaitPhaseIdx;

	// Looking for the end of the swing/stance phase in currentGait_
	while ((!currentGait.row(gaitPhaseIdx + 1).isZero())
			&& (currentGait(gaitPhaseIdx + 1, legNo) == (double) footPhase))
	{
		gaitPhaseIdx++;
		t_phase++;
	}
	// If we reach the end of currentGait_ we continue looking for the end of the swing/stance phase in desiredGait_
	if (currentGait.row(gaitPhaseIdx + 1).isZero())
	{
		int k = 0;
		while ((!desiredGait.row(k).isZero())
				&& (desiredGait(k, legNo) == (double) footPhase))
		{
			k++;
			t_phase++;
		}
	}
	// We suppose that we found the end of the swing/stance phase either in currentGait_ or desiredGait_
	remainingTime_ = t_phase;

	// Looking for the beginning of the swing/stance phase in currentGait_
	while ((a > 0) && (currentGait(a - 1, legNo) == (double) footPhase))
	{
		a--;
		t_phase++;
	}
	// If we reach the end of currentGait_ we continue looking for the beginning of the swing/stance phase in pastGait_
	if (a == 0)
	{
		while ((!pastGait.row(a).isZero())
				&& (pastGait(a, legNo) == (double) footPhase))
		{
			a++;
			t_phase++;
		}
	}
	// We suppose that we found the beginning of the swing/stance phase either in currentGait_ or pastGait_
	return t_phase * params->dt_mpc;  // Take into account time step value
}

double GaitPlanner::getElapsedTime(int i, int j)
{
	double state = currentGait(i, j);
	double nPhase = 0;
	int row = i;

	// Looking for the beginning of the swing/stance phase in currentGait_
	while ((row > 0) && (currentGait(row - 1, j) == state))
	{
		row--;
		nPhase++;
	}

	// If we reach the end of currentGait_ we continue looking for the beginning of the swing/stance phase in pastGait_
	if (row == 0)
	{
		while ((!pastGait.row(row).isZero()) && (pastGait(row, j) == state))
		{
			row++;
			nPhase++;
		}
	}
	return nPhase * params->dt_mpc;
}

double GaitPlanner::getPhaseDuration(int i, int j)
{
	return getElapsedTime(i, j) + getRemainingTime(i, j);
}

double GaitPlanner::getRemainingTime(int i, int j)
{
	double state = currentGait(i, j);
	double nPhase = 1;
	int row = i;
	// Looking for the end of the swing/stance phase in currentGait_
	while ((row < params->get_N_steps()  - 1) && (currentGait(row + 1, j) == state))
	{
		row++;
		nPhase++;
	}
	// If we reach the end of currentGait_ we continue looking for the end of the swing/stance phase in desiredGait_
	if (currentGait.row(i + 1).isZero())
	{
		// if (row == nRows_ - 1) {
		row = 0;
		while ((row < params->get_N_steps() ) && (desiredGait(row, j) == state))
		{
			row++;
			nPhase++;
		}
	}
	return nPhase * params->dt_mpc;
}

bool GaitPlanner::update(bool initiateNewStep, int targetGaitType)
{
	if ((targetGaitType != GaitType::NoGait)
			&& (currentGaitType != targetGaitType))
	{
		changeGait(targetGaitType);
	}

	if (initiateNewStep)
	{
		rollGait();
		return true;
	}

	return false;
}

bool GaitPlanner::changeGait(int targetGait)
{
	is_static = false;
	if (targetGait == GaitType::Pacing)
	{
		std::cout << "change to pacing gait" << std::endl;
		createPacing();
		prevGaitType_ = currentGaitType;
		currentGaitType = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::Bounding)
	{
		std::cout << "change to bounding gait" << std::endl;

		prevGaitType_ = currentGaitType;
		createBounding();
		prevGaitType_ = currentGaitType;
		currentGaitType = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::Trot)
	{
		std::cout << "change to trot gait" << std::endl;
		createTrot();
		prevGaitType_ = currentGaitType;
		currentGaitType = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::Walking)
	{
		std::cout << "change to walking gait" << std::endl;
		createWalk();
		prevGaitType_ = currentGaitType;
		currentGaitType = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::WalkingTrot)
	{
		std::cout << "change to walking trot " << std::endl;
		createWalkingTrot();
		prevGaitType_ = currentGaitType;
		currentGaitType = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::CustomGallop)
	{
		std::cout << "change to custom gallo" << std::endl;
		createCustomGallop();
		prevGaitType_ = currentGaitType;
		currentGaitType = (GaitType) targetGait;
	}
	else if (targetGait == GaitType::NoMovement)
	{
		createStatic();
		prevGaitType_ = currentGaitType;
		currentGaitType = (GaitType) targetGait;
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

	return is_static;
}

void GaitPlanner::rollGait()
{
	// Transfer current gait into past gait
	// shift pastGait from [0..9] to [1..10]
	for (int m = params->get_N_steps(); m > 0; m--)
	{
		pastGait.row(m).swap(pastGait.row(m - 1));
	}
	// and assign current gait to [0]
	pastGait.row(0) = currentGait.row(0);

	// Entering new contact phase, store positions of feet that are now in contact
	newPhase = !currentGait.row(0).isApprox(currentGait.row(1));

	// Age current gait
	int index = 1;
	while (!currentGait.row(index).isZero())
	{
		currentGait.row(index - 1).swap(currentGait.row(index));
		index++;
	}

	// Insert a new line from desired gait into current gait
	currentGait.row(index - 1) = desiredGait.row(0);

	// Age desired gait
	index = 1;
	while (!desiredGait.row(index).isZero())
	{
		desiredGait.row(index - 1).swap(desiredGait.row(index));
		index++;
	}
}



void GaitPlanner::getLoopsInSteps(int &loops_passed, int & loops_to_go) {
	// look forward how many loops we got
	int index = 0;
	loops_to_go = 0;
	while (currentGait.row(0) == currentGait.row(index+1)) {
		index++;
		loops_to_go++;
	}

	// look backwards how many loops we passed
	index = 0;
	loops_passed = 0;
	while (currentGait.row(0) == pastGait.row(index+1)) {
		index++;
		loops_passed++;
	}
}

