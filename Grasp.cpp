#include "Grasp.h"

Grasp::Grasp(dart::simulation::WorldPtr& world, SkeletonPtr Obj, SkeletonPtr hand)
:mWorld(world), mObject(Obj), mHand(hand)
{
	mContactPlanner = std::make_unique<Contact>(mObject, mHand);
}

void Grasp::contactPlanning()
{

}

Eigen::VectorXd Grasp::testPlanning()
{
	dart::dynamics::Shape* obj_shape = mObject->getShapeNode(0)->getShape().get();
	const auto* cylinder = static_cast<const CylinderShape*>(obj_shape);
	double rad = cylinder->getRadius();
	double height = cylinder->getHeight();
	double theta = 90 * M_PI / 180;
	double offset = height / 6;
	std::vector<std::pair<Eigen::Vector6d, std::string>> Ends;
	Eigen::Matrix3d rot_thumb;
	Eigen::Matrix3d rot_finger;
	Eigen::Vector6d temp;

	IkSolver* Ikea = new IkSolver();

	mContactPlanner->createNewContact("idthumb", PC, Eigen::Vector3d(rad*cos(-theta),rad*sin(-theta),-height/2 + offset), "thumbpatch");
	mContactPlanner->createNewContact("id0", PC, Eigen::Vector3d(rad*cos(theta),rad*sin(theta),-height/2 + offset), "patch0");
	mContactPlanner->createNewContact("id1", SF, Eigen::Vector3d(rad*cos(theta),rad*sin(theta),-height/2 + offset *2), "patch1");


	//Thumb Push Back
	Eigen::Vector3d current_normal = mContactPlanner->getNormalVector(Eigen::Vector3d(rad*cos(-theta),rad*sin(-theta),-height/2 + offset));

	rot_thumb.col(0) = - mContactPlanner->getContactById("idthumb").g_po.block(0,1,3,3) * current_normal;
	rot_thumb.col(1) = Eigen::Vector3d(0,1,0);
	rot_thumb.col(2) = rot_thumb.col(0).cross(rot_thumb.col(1)).normalized();

	temp.head<3>() = dart::math::logMap(rot_thumb);
	temp.tail<3>() = this->localPt2GlobalPt(mContactPlanner->getContactById("idthumb").g_oc.col(0));
	Ends.push_back(std::make_pair(temp, "thumbpatch"));

	//Fingers Push Back
	current_normal = mContactPlanner->getNormalVector(Eigen::Vector3d(rad*cos(theta),rad*sin(theta),-height/2 + offset));

	rot_finger.col(0) = Eigen::Vector3d(0,1,0);
	rot_finger.col(1) = - mContactPlanner->getContactById("id0").g_po.block(0,1,3,3) * current_normal;
	rot_finger.col(2) = rot_finger.col(0).cross(rot_finger.col(1)).normalized();

	temp.head<3>() = dart::math::logMap(rot_finger);
	temp.tail<3>() = this->localPt2GlobalPt(mContactPlanner->getContactById("id0").g_oc.col(0));
	Ends.push_back(std::make_pair(temp, "patch0"));

	current_normal = mContactPlanner->getNormalVector(Eigen::Vector3d(rad*cos(theta),rad*sin(theta),-height/2 + offset * 2));

	rot_finger.col(1) = - mContactPlanner->getContactById("id1").g_po.block(0,1,3,3) * current_normal;
	rot_finger.col(2) = rot_finger.col(0).cross(rot_finger.col(1)).normalized();

	temp.head<3>() = dart::math::logMap(rot_finger);
	temp.tail<3>() = this->localPt2GlobalPt(mContactPlanner->getContactById("id1").g_oc.col(0));
	Ends.push_back(std::make_pair(temp, "patch1"));

	this->forcePlanning();
	// std::cout << mHand->getLinearJacobian(mHand->getBodyNode("thumbpatch"), palm_node).transpose() <<std::endl;
	// Eigen::VectorXd result = Ikea->IKSubjectTo(mHand, Ends, "weld", 1000);
	Eigen::VectorXd result = Ikea->stepByStep(mHand, Ends, "weld", 2000);
	// Eigen::VectorXd result = Ikea->ConstrainedIKSubjectTo(mWorld, "hand", "object", Ends, "weld", 1000);
	std::cout << result.transpose() << std::endl;
	return result;
}

void Grasp::forcePlanning()
{
	Eigen::VectorXd jointTorque;
	Eigen::VectorXd contactForce_concat;
	Eigen::MatrixXd Jh_concat;
	int numContacts = mContactPlanner->mContacts.size();
	int n, m;
	std::vector<double> concat_force;
	for(int i = 0; i < numContacts; ++i)
	{
		for(int j = 0; j < mContactPlanner->mContacts[i].contactForces.rows(); ++j)
		{
			concat_force.push_back(mContactPlanner->mContacts[i].contactForces(j));
		}
	}
	m = concat_force.size();
	contactForce_concat.resize(m);
	for(int i =0; i < m; ++i)
		contactForce_concat(i) = concat_force[i];

	int cnt = 0;
	n = mContactPlanner->mContacts[0].JacobianHand.cols();
	Jh_concat.resize(m, n);
	for(int i = 0; i < numContacts; ++i)
	{
		Jh_concat.block(cnt, 0, mContactPlanner->mContacts[i].JacobianHand.rows(), mContactPlanner->mContacts[i].JacobianHand.cols()) = mContactPlanner->mContacts[i].JacobianHand;
		cnt += mContactPlanner->mContacts[i].JacobianHand.rows();
	}
	jointTorque = Jh_concat.transpose() * contactForce_concat;
}

Eigen::Vector3d Grasp::localPt2GlobalPt(Eigen::Vector3d src)
{
	Eigen::Vector3d dst;
	dart::dynamics::Frame *obj_frame = mObject->getBodyNode(0);
	Eigen::Isometry3d T_obj = obj_frame->getWorldTransform();
	Eigen::Matrix3d R_obj = T_obj.rotation();
	Eigen::Vector3d Tl_obj = T_obj.translation();

	dst = R_obj * src + Tl_obj;
	return dst;
}

Eigen::Vector3d Grasp::globalPt2LocalPt(Eigen::Vector3d src)
{
	Eigen::Vector3d dst;
	dart::dynamics::Frame *obj_frame = mObject->getBodyNode(0);
	Eigen::Isometry3d T_obj = obj_frame->getWorldTransform();
	Eigen::Matrix3d R_obj = T_obj.rotation();
	Eigen::Vector3d Tl_obj = T_obj.translation();

	dst = R_obj.inverse() * (src - Tl_obj);
	return dst;
}