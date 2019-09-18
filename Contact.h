#ifndef _CONTACT_H_
#define	_CONTACT_H_

#include <iostream>
#include "dart/dart.hpp"
#include "dart/utils/utils.hpp"
#include <unsupported/Eigen/MatrixFunctions>
using namespace dart::dynamics;

enum frictionType{FL, PC, SF};

struct Wrench{
	Eigen::Vector3d force;
	Eigen::Vector3d torque;
};

/*! These senses are all defined on the a contact frame which is different from finger frame
	g_po stands for SE(3) transformation of object frame related to palm frame
	g_oc stands for SE(3) transformation of contact frame related to object frame
	g_xy = (p_xy, R_xy)
*/
struct singleContact{
	std::string cId;
	std::string contactEndEffector;
	frictionType fType;
	Eigen::MatrixXd wrenchBasis;
	Eigen::MatrixXd graspMap;
	Eigen::MatrixXd JacobianHand;
	Eigen::VectorXd contactForces;
	Eigen::MatrixXd g_oc;
	Eigen::MatrixXd g_po;
};


class Contact{

protected:
	SkeletonPtr mObject;
	SkeletonPtr mHand;

public:
	std::vector<singleContact> mContacts;
	double fs_coeff;
	double t_coeff;
	bool valid;

public:
	Contact();
	Contact(SkeletonPtr Obj, SkeletonPtr hand);
	void createNewContact(std::string id, frictionType ft, const Eigen::Vector3d contact_pos, std::string finger);
	void setContactForceMagnitude(singleContact& currentContact, Eigen::VectorXd force);
	bool checkValidStiaticFrictionType(std::string id);
	void computeRelativeMaps(singleContact& currentContact, const Eigen::Vector3d contact_pos);
	void computeGraspMap(singleContact& currentContact);
	Eigen::Vector3d getNormalVector(const Eigen::Vector3d contact_pos);
	void computeHandJacobian(singleContact& currentContact);
	singleContact getContactById(std::string id);

	//Useful functions
	Eigen::Matrix3d normal2Frame(const Eigen::Vector3d norm);
};

#endif