#include "Contact.h"

Contact::Contact(){}
Contact::Contact(SkeletonPtr obj, SkeletonPtr hand) : mObject(obj), mHand(hand)
{
	fs_coeff = mObject->getBodyNode(0)->getFrictionCoeff();
	valid = true;
}

void Contact::createNewContact(std::string id, frictionType ft, const Eigen::Vector3d contact_pos, std::string finger)
{
	singleContact currentContact;
	currentContact.cId = id;
	currentContact.fType = ft;
	currentContact.contactEndEffector = finger;
	Eigen::MatrixXd currentWrench;
	if(ft == FL)
	{
		Eigen::VectorXd freeWrench = Eigen::VectorXd::Zero(6);
		freeWrench[2] = 1;
		// freeWrench.resize(6, 1);
		currentWrench = freeWrench;
		currentContact.contactForces.resize(1);
		currentContact.contactForces.setOnes();
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
		currentContact.contactForces.setOnes();
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
		currentContact.contactForces.setOnes();
	}
	currentContact.wrenchBasis = currentWrench;

	this->computeRelativeMaps(currentContact, contact_pos);
	this->computeGraspMap(currentContact);
	this->computeHandJacobian(currentContact);
	mContacts.push_back(currentContact);
}

void Contact::setContactForceMagnitude(singleContact& currentContact, Eigen::VectorXd force)
{
	currentContact.contactForces = force;
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
		if (contact_pos[2] > height/2 || contact_pos[2] < -height/2 || pow(contact_pos[0],2) +pow(contact_pos[1],2) > pow(rad,2)+ 1E-5)
		{
			valid = false;
			std::cout << "Invalid contact point1" << std::endl;
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
			std::cout << "Invalid contact point2" << std::endl;
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
		if(false) //Vertex
		{

		}
		else if(false) //Edge
		{

		}
		else //Face
		{
			if(contact_pos[0] == size[0]/2 || contact_pos[0] == -size[0]/2)
				normal = (Eigen::Vector3d(0,0,0) - Eigen::Vector3d(contact_pos[0],0,0)).normalized();
			else if(contact_pos[1] == size[1]/2 || contact_pos[1] == -size[1]/2)
				normal = (Eigen::Vector3d(0,0,0) - Eigen::Vector3d(0,contact_pos[1],0)).normalized();
			else if(contact_pos[2] == size[2]/2 || contact_pos[2] == -size[2]/2)
				normal = (Eigen::Vector3d(0,0,0) - Eigen::Vector3d(0,0,contact_pos[2])).normalized();
		}


	}
	else if(obj_shape->is<SphereShape>())
	{
		normal = (Eigen::Vector3d(0,0,0) - contact_pos).normalized();
	}
	else if(obj_shape->is<EllipsoidShape>())
	{
		const auto* ellipsoid = static_cast<const EllipsoidShape*>(obj_shape);
		//Have to implement
	}
	else
	{	
		//Tetrahedral Cases
		//Cross product and choose shorter one compare to left vertex
	}
	normal.normalize();
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
	G_map = adjointT * currentContact.wrenchBasis;
	// std::cout << "Grasp map is : " << std::endl;
	// std::cout << G_map << std::endl;
	currentContact.graspMap = G_map;

	// std::cout << "original " <<std::endl;
	// std::cout << rotation << std::endl;
	// std::cout << "to vector" << std::endl;
	// std::cout << dart::math::logMap(rotation) <<std::endl;
	// std::cout << "to vec to rot" <<std::endl;
	// std::cout << dart::math::expMapRot(dart::math::logMap(rotation)) << std::endl;
}

void Contact::computeHandJacobian(singleContact& currentContact){
	dart::dynamics::Frame *hand_frame = mHand->getBodyNode("palm");
	BodyNode* palm_node = mHand->getBodyNode("palm");
	Eigen::MatrixXd Jh;
	Eigen::Matrix6d Ad_gpc_inv;
	Eigen::MatrixXd Js_pf;
	Eigen::Matrix3d rotation_pc = currentContact.g_po.block(0,1,3,3) * currentContact.g_oc.block(0,1,3,3);
	Eigen::Vector3d trans_pc = currentContact.g_po.col(0) + currentContact.g_po.block(0,1,3,3) * currentContact.g_oc.col(0);
	Eigen::Matrix3d skew_trans_pc = dart::math::makeSkewSymmetric(trans_pc);
	BodyNode* bn = mHand->getBodyNode(currentContact.contactEndEffector);

	Ad_gpc_inv.block(0, 0, 3, 3) = rotation_pc.transpose();
	Ad_gpc_inv.block(0, 3, 3, 3) = -rotation_pc.transpose() * skew_trans_pc;
	Ad_gpc_inv.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
	Ad_gpc_inv.block(3, 3, 3, 3) = rotation_pc.transpose();

	//Asume palm as spatial coordinate(Not moving)
	Js_pf = mHand->getJacobian(bn, palm_node, hand_frame);
	Jh = currentContact.wrenchBasis.transpose() * Ad_gpc_inv * Js_pf;
	currentContact.JacobianHand = Jh;
	// std::cout << "Hand Jacobian is" <<std::endl;
	// std::cout << Jh.transpose() <<std::endl;
	// std::cout << Jh * mHand->getVelocities() << std::endl;
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

singleContact Contact::getContactById(std::string id){
	singleContact temp;

	for(int i =0 ; i < mContacts.size(); ++i)
	{
		if(mContacts[i].cId == id)
		{
			temp = mContacts[i];
			break;
		}
	}

	return temp;
}