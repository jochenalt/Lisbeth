/**
 * Planner for Gait.
 *
 * output is an array representing if feet are touching the ground, e.g.
 * 	1 1 0 0
 * 	0 0 1 1
 *
 * represents a running cheetah-gait. The gait is the basis for the next steps like foot step planning and trajectory generation
 *
 */

#ifndef GAIT_H_
#define GAIT_H_

#include "Types.h"
#include "Params.hpp"


// Order of feet/legs: FL, FR, HL, HR

class Gait
{
public:

	enum FootPhase { SWING_PHASE = 0, STANCE_PHASE = 1 };
    Gait();
    virtual ~Gait() {}

    void initialize(Params& params);

    /**
     * Compute the remaining and total duration of a swing phase or a stance phase based
     * on the content of the gait matrix
     * 	gaitPhaseIdx 	considered gait phase (row of the gait matrix)
     * 	footIdx 		considered foot (col of the gait matrix)
     * 	phase 			considered foot phase, swing or stance
     */
    double getPhaseDuration(int gaitPhaseIdx, int footIdx, FootPhase phase);

    // Change the gait to the passed type
    bool changeGait(int gaitTypeInput);

    /** Move one step further in the gait cycle
     *
     * Decrease by 1 the number of remaining step for the current phase of the gait
     * Transfer current gait phase into past gait matrix
     * Insert future desired gait phase at the end of the gait matrix
     */
    bool update(bool const rollGait, int targetGaitType);


    void rollGait();

    MatrixN4 getCurrentGait() { return currentGait_; }
    double getCurrentGait(int i, int j) { return currentGait_(i, j); }
    bool getIsStatic() { return is_static_; }
    bool isNewPhase() { return newPhase_; }
    GaitType getCurrentGaitType() { return currentGaitType_; }
    int getCurrentGaitTypeInt() { return currentGaitType_; }

    GaitType getPrevGaitType() { return prevGaitType_; }
    int getPrevGaitTypeInt() { return prevGaitType_; }

    double getElapsedTime(int i, int j);
    double getRemainingTime(int i, int j);
    double getPhaseDuration(int i, int j);

private:
    void createWalk();
    void createTrot();
    void createPacing();
    void createBounding();
    void createWalkingTrot();
    void createCustomGallop();

    void createStatic();

    void setGait(int pos, int sequences, MatrixN4 & gait, std::string sequence);

    MatrixN4 pastGait_;     // Past gait

public:
    MatrixN4 currentGait_;  // Current and future gait. needs to be public to be used from python
private:
    MatrixN4 desiredGait_;  // Future desired gait

    Params* params;

    double remainingTime_;

    bool newPhase_;
    bool is_static_;

    GaitType currentGaitType_;
    GaitType prevGaitType_;

    GaitType subGait;

};

#endif  // GAIT_H_INCLUDED
