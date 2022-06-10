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
    bool changeGait(int gaitTypeInput, VectorN const& q);

    /** Move one step further in the gait cycle
     *
     * Decrease by 1 the number of remaining step for the current phase of the gait
     * Transfer current gait phase into past gait matrix
     * Insert future desired gait phase at the end of the gait matrix
     */
    bool updateGait(bool const rollGait, VectorN const& q, int targetGaitType);


    void rollGait();

    MatrixN getCurrentGait() { return currentGait_; }
    double getCurrentGaitCoeff(int i, int j) { return currentGait_(i, j); }
    double getRemainingTime() { return remainingTime_; }
    bool getIsStatic() { return is_static_; }
    VectorN getQStatic() { return q_static_; }
    bool isNewPhase() { return newPhase_; }
    int getCurrentGaitType() { return currentGaitType_; }
    int getPrevGaitType() { return prevGaitType_; }

private:
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief  Create a slow walking gait, raising and moving only one foot at a time
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void createWalk();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Create a trot gait with diagonaly opposed legs moving at the same time
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void createTrot();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Create a pacing gait with legs on the same side (left or right) moving at the same time
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void createPacing();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Create a bounding gait with legs on the same side (front or hind) moving at the same time
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void createBounding();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Create a static gait with all legs in stance phase
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void createStatic();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// \brief Initialize content of the gait matrix based on the desired gait, the gait period and
    ///        the length of the prediciton horizon
    ///
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void create_gait_f();

    MatrixN pastGait_;     // Past gait
    MatrixN currentGait_;  // Current and future gait
    MatrixN desiredGait_;  // Future desired gait

    double dt_;      // Time step of the contact sequence (time step of the MPC)
    int nRows_;  // number of rows in the gait matrix

    double T_gait_;  // Gait period
    double T_mpc_;   // MPC period (prediction horizon)
    int n_steps_;        // Number of time steps in the prediction horizon

    double remainingTime_;

    bool newPhase_;
    bool is_static_;
    VectorN q_static_;

    GaitType currentGaitType_;
    GaitType prevGaitType_;

    GaitType subGait;

};

#endif  // GAIT_H_INCLUDED
