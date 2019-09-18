#ifndef _IKSOLVER_H
#define _IKSOLVER_H

#include <iostream>
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/utils/utils.hpp"
using namespace dart::common;
using namespace dart::dynamics;


class IkSolver{
public: 
	IkSolver();
	Eigen::VectorXd IKMultiple(const SkeletonPtr& hand, std::vector<std::pair<Eigen::Vector3d, std::string>> Ends, int total_iter);
	Eigen::VectorXd IKSubjectTo(const SkeletonPtr& hand, std::vector<std::pair<Eigen::Vector6d, std::string>> Ends, std::string relativeFrame, int total_iter);
	Eigen::VectorXd IKMultipleFixedWeighted(const SkeletonPtr& hand, std::vector<std::pair<Eigen::Vector3d, std::string>> Ends, const Eigen::VectorXd fixed_weight,int total_iter);
	Eigen::VectorXd ConstrainedIKSubjectTo(const dart::simulation::WorldPtr& world, std::string hand_name,std::string obj_name, std::vector<std::pair<Eigen::Vector6d, std::string>> Ends, std::string relativeFrame, int total_iter);
	Eigen::VectorXd stepByStep(const SkeletonPtr& hand, std::vector<std::pair<Eigen::Vector6d, std::string>> Ends, std::string relativeFrame, int total_iter);
	bool isSatisfyConstrain(const dart::simulation::WorldPtr& world, std::string hand_name,std::string obj_name);
};

#endif