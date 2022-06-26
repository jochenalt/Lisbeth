#ifndef TYPES_H_INCLUDED
#define TYPES_H_INCLUDED

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <memory>

using Vector1 = Eigen::Matrix<double, 1, 1>;
using Vector2 = Eigen::Matrix<double, 2, 1>;
using Vector3 = Eigen::Matrix<double, 3, 1>;
using Vector3i = Eigen::Matrix<int, 3, 1>;

using Array3 = Eigen::Array<double, 3, 1>;

using Vector4 = Eigen::Matrix<double, 4, 1>;
using Vector5 = Eigen::Matrix<double, 5, 1>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Vector7 = Eigen::Matrix<double, 7, 1>;
using Vector8 = Eigen::Matrix<double, 8, 1>;

using Vector11 = Eigen::Matrix<double, 11, 1>;
using Vector12 = Eigen::Matrix<double, 12, 1>;
using Vector18 = Eigen::Matrix<double, 18, 1>;
using Vector19 = Eigen::Matrix<double, 19, 1>;
using VectorN = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using VectorNi = Eigen::Matrix<int, Eigen::Dynamic, 1>;

using RowVector4 = Eigen::Matrix<double, 1, 4>;
using RowVector3 = Eigen::Matrix<double, 1, 3>;
using RowVector12 = Eigen::Matrix<double, 1, 12>;

using Matrix242 = Eigen::Matrix<double, 24, 2>;

using Matrix3 = Eigen::Matrix<double, 3, 3>;
using Matrix4 = Eigen::Matrix<double, 4, 4>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Matrix31 = Eigen::Matrix<double, 3, 1>;
using Matrix13 = Eigen::Matrix<double, 1, 3>;
using Matrix112 = Eigen::Matrix<double, 1, 12>;
using Matrix12 = Eigen::Matrix<double, 12, 12>;
using Matrix18 = Eigen::Matrix<double, 18, 18>;

using Matrix34 = Eigen::Matrix<double, 3, 4>;
using Matrix43 = Eigen::Matrix<double, 4, 3>;
using Matrix64 = Eigen::Matrix<double, 6, 4>;
using Matrix3N = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using Matrix4N = Eigen::Matrix<double, 4, Eigen::Dynamic>;
using MatrixN4i = Eigen::Matrix<int, Eigen::Dynamic, 4>;
using Matrix6N = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Matrix12N = Eigen::Matrix<double, 12, Eigen::Dynamic>;
using MatrixN = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixNi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;

typedef Eigen::MatrixXd matXd;

// the legs or ordered in that sequence
enum Legs { FL=0, FR = 1, HL = 2, HR = 3 };
typedef int LegNoType;
#define LegNo 4

// Different Gait Types:
enum GaitType { NoGait = 0,
	            Pacing   	= 1, /* Camel */
				Bounding 	= 2, /* Cheetah */
				Walking  	= 3, /* One leg up */
				Trot     	= 4,/* Dog */
				WalkingTrot = 5,/* Dog */
				CustomGallop= 6,/* Dog */
				NoMovement  = 7, /* No Movement */
};
#define GaitNo 7


#endif  // TYPES_H_INCLUDED
