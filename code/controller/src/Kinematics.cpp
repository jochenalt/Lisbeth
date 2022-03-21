#include <fstream>

#include "Kinematics.hpp"

using namespace std;

inline bool file_exists (const std::string& name) {
    ifstream f(name.c_str());
    return f.good();
}


Kinematics::Kinematics()  {
}

void Kinematics::initialize() {

    // Load the urdf model
	if (!file_exists(urdf_path)) {
		std::cout << "Kinematics.initialize:" << urdf_path << " does not exist" << endl;
		exit(1);
	}

	pinocchio::JointModelFreeFlyer root_joint;
	pinocchio::urdf::buildModel(urdf_path,root_joint, model);
	cout << "Kinematics.initialize(" << urdf_path << ")" << endl;

	// Create data required by the algorithms
	data = Data(model);
}



void Kinematics::forwardKinematics1(Eigen::VectorXd q) {
	pinocchio::forwardKinematics(model,data,q);
}

void Kinematics::forwardKinematics2(Eigen::VectorXd q, Eigen::VectorXd v) {
	pinocchio::forwardKinematics(model,data,q, v);

}
