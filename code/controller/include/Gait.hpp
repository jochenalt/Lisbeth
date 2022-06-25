/**
 * header for Gait class
 * Planner that outputs current and future locations of footsteps, the reference
 * trajectory of the base and the position, velocity, acceleration commands for feet in
 * swing phase based on the reference velocity given by the user and the current
 * position/velocity of the base
 * */

#ifndef GAIT_H_INCLUDED
#define GAIT_H_INCLUDED

#include "Types.h"
#include "Params.hpp"


// Order of feet/legs: FL, FR, HL, HR

class Gait
{
public:

	enum FootPhase { SWING_PHASE = 0, STANCE_PHASE = 1 };
    Gait();
    virtual ~Gait() {}

    /**
     * initializes the gait manager
     *
     */
    void initialize(Params& params);

    /**
     * Compute the remaining and total duration of a swing phase or a stance phase based
     * on the content of the gait matrix
     * 	gaitPhaseIdx 	considered gait phase (row of the gait matrix)
     * 	footIdx 		considered foot (col of the gait matrix)
     * 	phase 			considered foot phase, swing or stance
     */
    double getPhaseDuration(int gaitPhaseIdx, int footIdx, FootPhase phase);

    /** Handle the joystick code to trigger events (change of gait for instance)
     * gaitTypeInput target gait, coming from GaitType, because of Python binding needs to be int
     * q current position vector of the flying base in world frame (linear and angular stacked)
     */
    bool changeGait(int gaitTypeInput);

    /** Move one step further in the gait cycle
     *
     * Decrease by 1 the number of remaining step for the current phase of the gait
     * Transfer current gait phase into past gait matrix
     * Insert future desired gait phase at the end of the gait matrix
     */
    bool update(bool const rollGait, int targetGaitType);


    void rollGait();

    MatrixN getCurrentGait() { return currentGait_; }
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


    MatrixN pastGait_;     // Past gait
public:
    MatrixN currentGait_;  // Current and future gait
private:
    MatrixN desiredGait_;  // Future desired gait

    double dt_;      // Time step of the contact sequence (time step of the MPC)
    int nRows_;  // number of rows in the gait matrix

    double T_gait_;  // Gait period
    double T_mpc_;   // MPC period (prediction horizon)

    double remainingTime_;

    bool newPhase_;
    bool is_static_;

    GaitType currentGaitType_;
    GaitType prevGaitType_;

    GaitType subGait;

};

#endif  // GAIT_H_INCLUDED
