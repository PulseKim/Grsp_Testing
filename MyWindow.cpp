#include "MyWindow.h"

MyWindow::MyWindow(const WorldPtr& world) : SimWindow()
{
	this->setWorld(world);	
	this->initParameters();
	this->initWindowSetting();
}

void MyWindow::initParameters()
{
	mWorld->setGravity(Eigen::Vector3d(0, -9.81, 0));
	// mWorld->setGravity(Eigen::Vector3d(0, -0.1, 0));
	rad_obj = 0.04;
	height_obj = 0.15;
}

void MyWindow::initWindowSetting()
{	
	mIKsolver = std::make_unique<IkSolver>();
	this->initSkeleton();
	this->setSkeleton();
	this->addSkeleton();
	mController = std::make_unique<Controller>(mHand);
}

void MyWindow::initSkeleton()
{
	mFloor = Skeleton::create("floor");
	mHand = Skeleton::create("hand");
	mObject = Skeleton::create("object");

	SkelParser skel;
	skel.weldBox(mFloor, "floor", Eigen::Vector3d(0.5, 0.03, 0.5), Eigen::Vector3d(0, 0.015,0), 0.5, dart::Color::White());
	skel.freeCylinder(mObject, "cylinder1", rad_obj, height_obj, Eigen::Vector3d(0,0, 0), 0.1, dart::Color::Red());
	mHandMaker = std::make_unique<HandMaker>(mHand, "right");
	mHandMaker->makeHand();
}

void MyWindow::setSkeleton()
{
	mObject->setPosition(0, this->degToRad(90));
	mObject->setPosition(4, height_obj/2 + 0.001);
	mHand->setPosition(2, this->degToRad(90));

	double offsetx = 0.1, offsetz = 0.05;
	Eigen::Vector3d initPosition(mObject->getCOM()[0]-offsetx, 0.1, mObject->getCOM()[2]-offsetz);
	mObject->getBodyNode(0)->setFrictionCoeff(0.8);


	//Transparency
	auto visualShapenodes = mObject->getBodyNode(0)->getShapeNodesWith<VisualAspect>();
	visualShapenodes[0]->getVisualAspect()->setColor(dart::Color::Red(0.2));
}

void MyWindow::addSkeleton()
{
	mWorld->addSkeleton(mFloor);
	mWorld->addSkeleton(mHand);
	mWorld->addSkeleton(mObject);
}

void MyWindow::testing()
{
	Eigen::Matrix3d isoRot;
	Eigen::Matrix3d tempRotation;
	tempRotation.col(0) = Eigen::Vector3d(1,0,0);
	tempRotation.col(1) = Eigen::Vector3d(0,cos(this->degToRad(60)), sin(this->degToRad(60))).normalized();
	tempRotation.col(2) = Eigen::Vector3d(0,-sin(this->degToRad(60)), cos(this->degToRad(60))).normalized();
	Eigen::Matrix3d tempRotation2;
	tempRotation2.col(0) = Eigen::Vector3d(1,0,0);
	tempRotation2.col(1) = Eigen::Vector3d(0,cos(this->degToRad(20)), sin(this->degToRad(20))).normalized();
	tempRotation2.col(2) = Eigen::Vector3d(0,-sin(this->degToRad(20)), cos(this->degToRad(20))).normalized();
	isoRot.col(0) = Eigen::Vector3d(0,1,0);
	isoRot.col(1) = Eigen::Vector3d(-1,0,0);
	isoRot.col(2) = Eigen::Vector3d(0,0,1);

	std::cout << dart::math::logMap(tempRotation).transpose() << std::endl;
	std::cout << dart::math::logMap(tempRotation2).transpose() << std::endl;
	std::cout << dart::math::logMap(isoRot).transpose() << std::endl;
	std::cout << dart::math::logMap(tempRotation*isoRot).transpose() << std::endl;
	std::cout << dart::math::logMap(tempRotation2*isoRot).transpose() << std::endl;
	std::cout << dart::math::logMap(isoRot*tempRotation).transpose() << std::endl;
	std::cout << dart::math::logMap(isoRot*tempRotation2).transpose() << std::endl;

	// Grasp* gg = new Grasp(mWorld, mObject, mHand);
	// Eigen::MatrixXd target = gg->testPlanning();
	// mHand->setPositions(target);
	// mController->setTargetPosition(target);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
		//Implement Here 
		case 'q':
		this->testing();
		break;
		case 'a':
		std::cout << mHand->getPositions().transpose() << std::endl;
		break;
		case 's':
		mObject->getBodyNode(0)->setExtForce(Eigen::Vector3d(0,0,1));
		break;
		case 'd':
		std::cout <<"palm world rotation matrix" <<std::endl;
		std::cout << mHand->getBodyNode("thumb metacarpal")->getWorldTransform().rotation() <<std::endl;
		std::cout << "distphalanx world rotation" <<std::endl;
		std::cout << mHand->getBodyNode("thumb distphalanx")->getWorldTransform().rotation() <<std::endl;
		std::cout << "distphalanx relative rotation world" << std::endl;
		std::cout << mHand->getBodyNode("thumb distphalanx")->getTransform(mHand->getBodyNode("thumb metacarpal"), mHand->getBodyNode(0)->getParentFrame()).rotation() <<std::endl;
		std::cout << "distphalanx relative rotation" << std::endl;
		std::cout << mHand->getBodyNode("thumb distphalanx")->getTransform(mHand->getBodyNode("thumb metacarpal"), mHand->getBodyNode("thumb metacarpal")).rotation() <<std::endl;
		std::cout << "inv * world" << std::endl;
		std::cout << mHand->getBodyNode("thumb metacarpal")->getWorldTransform().rotation().inverse() * mHand->getBodyNode("thumb distphalanx")->getWorldTransform().rotation() <<std::endl;


		break;
		default:
		SimWindow::keyboard(key, x, y);
	}	
}


void MyWindow::timeStepping()
{
	mController->clearForces();
	mController->addSPDForces();
	SimWindow::timeStepping();
	mController->setFreeJointPosition();
}

void MyWindow::draw()
{
	glDisable(GL_LIGHTING);
	// glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	if (!mSimulating) {
		if (mPlayFrame < mWorld->getRecording()->getNumFrames()) {
			std::size_t nSkels = mWorld->getNumSkeletons();
			for (std::size_t i = 0; i < nSkels; i++) {
	        // std::size_t start = mWorld->getIndex(i);
	        // std::size_t size = mWorld->getSkeleton(i)->getNumDofs();
				mWorld->getSkeleton(i)->setPositions(mWorld->getRecording()->getConfig(mPlayFrame, i));
			}	      
			if (mShowMarkers) {
	        // std::size_t sumDofs = mWorld->getIndex(nSkels);
				int nContact = mWorld->getRecording()->getNumContacts(mPlayFrame);
				for (int i = 0; i < nContact; i++) {
					Eigen::Vector3d v = mWorld->getRecording()->getContactPoint(mPlayFrame, i);
					Eigen::Vector3d f = mWorld->getRecording()->getContactForce(mPlayFrame, i);

					glBegin(GL_LINES);
					glVertex3f(v[0], v[1], v[2]);
					glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
					glEnd();
					mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
					mRI->pushMatrix();
					glTranslated(v[0], v[1], v[2]);
					mRI->drawSphere(0.01);
					mRI->popMatrix();
				}
			}
		}
	} 
	else {glPointSize(10.0);  
		if (mShowMarkers) {
			const auto result =
			mWorld->getConstraintSolver()->getLastCollisionResult();
			for (const auto& contact : result.getContacts()) {
				Eigen::Vector3d v = contact.point;
				Eigen::Vector3d f = contact.force / 10.0;
				glBegin(GL_LINES);
				glVertex3f(v[0], v[1], v[2]);
				glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
				glEnd();
				mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
				mRI->pushMatrix();
				glTranslated(v[0], v[1], v[2]);
				mRI->drawSphere(0.01);
				mRI->popMatrix();
			}
		}
	}

	this->drawNormals();
	glDisable(GL_LIGHTING);
	//Implement 2D factors inside here
	this->drawFrame();
	glEnable(GL_LIGHTING);
	drawWorld();

	  // display the frame count in 2D text
	char buff[64];
	if (!mSimulating)
	#ifdef _WIN32
		_snprintf(buff, sizeof(buff), "%d", mPlayFrame);
	#else
	std::snprintf(buff, sizeof(buff), "%d", mPlayFrame);
	#endif
	else
	#ifdef _WIN32
		_snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
	#else
	std::snprintf(buff, sizeof(buff), "%d", mWorld->getSimFrames());
	#endif
	std::string frame(buff);
	glColor3f(0.0, 0.0, 0.0);
	drawStringOnScreen(0.02f, 0.02f, frame);
	glEnable(GL_LIGHTING);
}

void MyWindow::drawNormals()
{
	glEnable(GL_LIGHTING);
	this->showDirection(true, Eigen::Vector3d(0,0.15,0), Eigen::Vector3d(0,-0.1,0));
}

Eigen::VectorXd MyWindow::ForwardKinematicsMovement(int current_idx, int total_steps, const Eigen::VectorXd original, const Eigen::VectorXd target)
{
	Eigen::VectorXd pose = original;
	for(int j = 0 ; j < target.size(); ++j){
		pose[j] = original[j] + (target[j] - original[j]) * current_idx / total_steps;
	}
	return pose;
}


void MyWindow::drawFrame()
{
	dart::dynamics::Frame *hand_frame = mHand->getBodyNode("patch0");
	Eigen::Matrix3d rot = hand_frame->getWorldTransform().rotation();
	Eigen::Vector3d trans = hand_frame->getWorldTransform().translation();
	glLineWidth(3.0);
	for(int i = 0; i < 3; i++){
		Eigen::Vector3d current_axis = rot.col(i).normalized() * 0.1;
		glColor3f(0.0,i * 0.5,0.0);
		glBegin(GL_LINES);
		glVertex3f(trans[0], trans[1], trans[2]);
		glVertex3f(trans[0] + current_axis[0], trans[1] + current_axis[1], trans[2] + current_axis[2]);
		glEnd();
	}
}

//Depricated Currently
void MyWindow::showDirection(bool flag, Eigen::Vector3d begin, Eigen::Vector3d dir)
{
	if(!flag) return;
	dir.normalize();
	dir = 0.03 * dir;
	glColor3f(0.0, 1.0, 1.0); 
	glLineWidth(4.0);
	glBegin(GL_LINES);
	glVertex3f(begin[0], begin[1], begin[2]);
	glVertex3f(begin[0] + dir[0], begin[1] + dir[1], begin[2] + dir[2]);
	glEnd();
	mRI->setPenColor(Eigen::Vector3d(0.0, 1.0, 1.0));
	mRI->pushMatrix();
	glTranslated(begin[0], begin[1], begin[2]);
	mRI->drawSphere(0.005);
	mRI->popMatrix();

}


void MyWindow::tempCollision()
{
	// std::cout << mWorld->getSimFrames() << std::endl;
	auto cont = mWorld->getLastCollisionResult().getContacts();
	// std::cout << cont[0].force <<std::endl;
	// std::cout << mWorld->getLastCollisionResult().getContact(0).force << std::endl;
	// for(int i= 0; i < cont.size(); ++i){
	// 	std::cout << mWorld->getLastCollisionResult().getContact(i).collisionObject1->getShapeFrame()->getName() <<std::endl;
	// 	std::cout << mWorld->getLastCollisionResult().getContact(i).collisionObject2->getShapeFrame()->getName() <<std::endl;
	// }
	// auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
	// auto objGroup = collisionEngine->createCollisionGroup(mObj.get());
	// auto finger1 = collisionEngine->createCollisionGroup((mArm_r->getBodyNode("distphalanx2")));
	// auto handGroup = collisionEngine->createCollisionGroup();
	// handGroup->addShapeFramesOf(mArm_r->getBodyNode("thumb distphalanx"));	
	// handGroup->distance(objGroup.get());
	// handGroup->removeShapeFramesOf(mArm_r->getBodyNode("thumb distphalanx"));
	// for(int i =0 ; i < 4;++i){
	// 	handGroup->addShapeFramesOf(mArm_r->getBodyNode("distphalanx"+ std::to_string(i)));
	// 	std::cout <<"collide" << handGroup->collide(finger1.get()) << std::endl;
	// 	handGroup->removeShapeFramesOf(mArm_r->getBodyNode("distphalanx"+ std::to_string(i)));
	// }
}

double MyWindow::degToRad(double degree)
{
	return degree * M_PI / 180;
}