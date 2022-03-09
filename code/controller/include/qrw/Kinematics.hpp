#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/explog.hpp"

using namespace pinocchio;

class Kinematics
{
	const std::string urdf_path = "/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf";

public:
	Kinematics();
	~Kinematics() {};

	void initialize();

	void forwardKinematics1(Eigen::VectorXd q);
	void forwardKinematics2(Eigen::VectorXd q, Eigen::VectorXd v);

private:
	  Model model;
	  Data data;
};

#endif  // KINEMATICS_H_
