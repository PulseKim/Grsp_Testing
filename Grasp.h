#ifndef _GRASP_H_
#define	_GRASP_H_

#include <iostream>
#include "dart/dart.hpp"
#include "dart/utils/utils.hpp"


class Grasp{
protected:
	Eigen::MatrixXd mWrenchBasis;
	Eigen::MatrixXd mGraspMap;
	Eigen::VectorXd mContactForces;


public:
	Grasp();
	bool checkValidStiaticFrictionType();
	Eigen::MatrixXd computeGraspMap();
	Eigen::MatrixXd computeHandJacobian();
	

};

#endif