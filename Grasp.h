#ifndef _GRASP_H_
#define	_GRASP_H_

#include <iostream>
#include "dart/dart.hpp"
#include "dart/utils/utils.hpp"
#include "IkSolver.h"
#include "Contact.h"


class Grasp{
protected:
	dart::simulation::WorldPtr& mWorld;
	SkeletonPtr mObject;
	SkeletonPtr mHand;
	std::unique_ptr<Contact> mContactPlanner;
	// Eigen::MatrixXd WB_concat;
	// Eigen::MatrixXd Jh_concat;


public:
	Grasp(dart::simulation::WorldPtr& world, SkeletonPtr Obj, SkeletonPtr hand);

	void forcePlanning();
	bool isContact();
	void contactPlanning();	
	Eigen::VectorXd testPlanning();

	/*! Functions that help main grasping */
	Eigen::MatrixXd config2Contact();
	Eigen::Vector3d localPt2GlobalPt(Eigen::Vector3d src);
	Eigen::Vector3d globalPt2LocalPt(Eigen::Vector3d src);

};

#endif