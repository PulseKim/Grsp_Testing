#include "Contact.h"

Contact::Contact(){}
Contact::Contact(SkeletonPtr obj, SkeletonPtr hand) : mObject(obj), mHand(hand)
{
	fs_coeff = mObject->getBodyNode(0)->getFrictionCoeff();
	valid = true;
}

void Contact::createNewContact(std::string id, frictionType ft, const Eigen::Vector3d contact_pos)
{
	singleContact currentContact;
	currentContact.cId = id;
	currentContact.fType = ft;
	Eigen::MatrixXd currentWrench;
	if(ft == FL)
	{
		Eigen::VectorXd freeWrench = Eigen::VectorXd::Zero(6);
		freeWrench[2] = 1;
		// freeWrench.resize(6, 1);
		currentWrench = freeWrench;
		currentContact.contactForces.resize(1);
		currentContact.contactForces.setZero();
	}
	else if(ft == PC)
	{
		Eigen::VectorXd temp = Eigen::VectorXd::Zero(18,1);
		temp[0] = 1;
		temp[7] = 1;
		temp[14] = 1;
		currentWrench = temp;
		currentWrench.resize(6, 3);
		currentContact.contactForces.resize(3);	
		currentContact.contactForces.setZero();
	}
	else if(ft == SF)
	{
		Eigen::VectorXd temp = Eigen::VectorXd::Zero(24,1);
		temp[0] = 1;
		temp[7] = 1;
		temp[14] = 1;
		temp[23] = 1;
		currentWrench = temp;
		currentWrench.resize(6, 4);
		currentContact.contactForces.resize(4);
		currentContact.contactForces.setZero();
	}
	currentContact.wrenchBasis = currentWrench;

	this->computeRelativeMaps(currentContact, contact_pos);
	this->computeGraspMap(currentContact);
	mContacts.push_back(currentContact);
}

void Contact::computeRelativeMaps(singleContact& currentContact, const Eigen::Vector3d contact_pos)
{
	//Set g_po
	dart::dynamics::Frame *world_frame = mObject->getBodyNode(0)->getParentFrame();
	dart::dynamics::Frame *obj_frame = mObject->getBodyNode(0);
	dart::dynamics::Frame *hand_frame = mHand->getBodyNode("palm");

	Eigen::Isometry3d Iso_po = obj_frame->getTransform(hand_frame, hand_frame);
	currentContact.g_po.resize(3, 4);
	currentContact.g_po.block(0,0,3,1) = Iso_po.translation();
	currentContact.g_po.block(0,1,3,3) = Iso_po.rotation();

	//SEt g_oc starting from getting frame orientation
	//Get Axis
	Eigen::Matrix3d obj_ori = obj_frame->getWorldTransform().rotation();
	Eigen::Matrix3d contact_ori;
	Eigen::Matrix3d rotation_oc;
	Eigen::Vector3d normal;

	normal = this->getNormalVector(contact_pos);
	contact_ori = this->normal2Frame(normal);
	rotation_oc = contact_ori * obj_ori.inverse();
	std::cout << "normal is " << normal.transpose() << std::endl;
	currentContact.g_oc.resize(3, 4);
	currentContact.g_oc.block(0,0,3,1) = contact_pos;
	currentContact.g_oc.block(0,1,3,3) = rotation_oc;
}

Eigen::Vector3d Contact::getNormalVector(const Eigen::Vector3d contact_pos)
{
	dart::dynamics::Shape* obj_shape = mObject->getShapeNode(0)->getShape().get();
	Eigen::Vector3d normal;
	if(obj_shape->is<CylinderShape>())
	{
		const auto* cylinder = static_cast<const CylinderShape*>(obj_shape);
		double rad = cylinder->getRadius();
		double height = cylinder->getHeight();

		//Invalid case
		if (contact_pos[2] > height/2 || contact_pos[2] < -height/2 || pow(contact_pos[0],2) +pow(contact_pos[1],2) > pow(rad,2))
		{
			valid = false;
			std::cout << "Invalid contact point" << std::endl;
			std::cout << "Please check the local position of object to contact" << std::endl;
		}
		//Top/Bottom case
		else if(contact_pos[2] == height/2 || contact_pos[2] == -height/2)
		{
			normal = (Eigen::Vector3d(0,0,0) - Eigen::Vector3d(0,0,contact_pos[2])).normalized();
		}
		//Invalid case
		else if(pow(contact_pos[0],2) +pow(contact_pos[1],2) < pow(rad,2) - 1E-5)
		{
			valid = false;
			std::cout << "Invalid contact point" << std::endl;
			std::cout << "Please check the local position of object to contact" << std::endl;
		}
		//Side case
		else
		{
			normal = (Eigen::Vector3d(0,0,0) - Eigen::Vector3d(contact_pos[0],contact_pos[1],0)).normalized();
		}
	}
	else if(obj_shape->is<BoxShape>())
	{
		const auto* box = static_cast<const BoxShape*>(obj_shape);
		Eigen::Vector3d size = box->getSize();

	}
	else if(obj_shape->is<SphereShape>())
	{
		normal = Eigen::Vector3d(0,0,0) - contact_pos;
	}
	else if(obj_shape->is<EllipsoidShape>())
	{
		const auto* ellipsoid = static_cast<const EllipsoidShape*>(obj_shape);
	}
	else
	{

	}

	return normal;
}

void Contact::computeGraspMap(singleContact& currentContact)
{
	Eigen::MatrixXd G_map;
	Eigen::Vector3d translate = currentContact.g_oc.col(0);
	Eigen::Matrix3d skew_translate = dart::math::makeSkewSymmetric(translate);
	Eigen::Matrix3d rotation =  currentContact.g_oc.block(0,1,3,3);
	Eigen::Matrix6d adjointT;
	adjointT.block(0, 0, 3, 3) = rotation;
	adjointT.block(0, 3, 3, 3) = Eigen::Matrix3d::Zero();
	adjointT.block(3, 0, 3, 3) = skew_translate * rotation;
	adjointT.block(3, 3, 3, 3) = rotation;
	// std::cout << adjointT << std::endl;
	G_map = adjointT * currentContact.wrenchBasis;
	std::cout << "Grasp map is : " << std::endl;
	std::cout << G_map << std::endl;
	currentContact.graspMap = G_map;

}

Eigen::Matrix3d Contact::normal2Frame(const Eigen::Vector3d norm)
{
	Eigen::Matrix3d frame;
	Eigen::Vector3d tangentX;
	Eigen::Vector3d tangentY;
	Eigen::Vector3d normal = norm.normalized();

	if (fabs(normal.dot(Eigen::Vector3d(1, 0, 0))) > 1.0 - 1E-8) 
	{
		tangentX = normal.cross(Eigen::Vector3d(0, 0, 1)).normalized();
	} 
	else 
	{
		tangentX = normal.cross(Eigen::Vector3d(1, 0, 0)).normalized();
	}
	tangentY = (normal.cross(tangentX)).normalized();
	frame.col(0) = tangentX;
	frame.col(1) = tangentY;
	frame.col(2) = normal;
	return frame;
}
