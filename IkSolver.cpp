#include "IkSolver.h"

IkSolver::IkSolver(){

}


Eigen::VectorXd IkSolver::IKMultiple
(const SkeletonPtr& hand, std::vector<std::pair<Eigen::Vector3d, std::string>> Ends, int total_iter)
{
	SkeletonPtr dummyHand = hand->cloneSkeleton();
	// std::cout << "current" <<std::endl;
	// for(int i = 0; i < Ends.size(); ++i)
	// 	std::cout << dummyHand->getBodyNode(Ends[i].second)->getCOM().transpose()<<std::endl;
	// std::cout << "target" << std::endl;
	// for(int i =0 ; i < Ends.size(); ++i)
	// 	std::cout << Ends[i].first.transpose() << std::endl;

	Eigen::MatrixXd J_stack = Eigen::MatrixXd::Zero(Ends[0].first.rows()*Ends.size(),hand->getPositions().size());
	Eigen::VectorXd dev_stack;
	dev_stack = Eigen::VectorXd::Zero(Ends.size()*3); 
	Eigen::VectorXd newPose;
	double epsilon = 0.001;

	for(int current_iter = 0; current_iter < total_iter ;current_iter++)
	{
		Eigen::VectorXd currentPose = dummyHand->getPositions();
		double total_dev = 0;
		for(int i = 0 ; i < Ends.size() ; i ++){
			BodyNode* currentPart;
			currentPart = dummyHand->getBodyNode(Ends[i].second);
			Eigen::Vector3d deviation = (Ends[i].first - currentPart->getCOM());
			total_dev += deviation.norm();
			Eigen::MatrixXd J = dummyHand->getLinearJacobian(currentPart);
			J_stack.block(J.rows()*i,0,J.rows(),J.cols()) = J;
			dev_stack(3*i) = deviation[0];
			dev_stack(3*i+1) = deviation[1];
			dev_stack(3*i+2) = deviation[2];
		}
		if(total_dev < epsilon) break;
		Eigen::MatrixXd JJT = J_stack*J_stack.transpose();
		Eigen::MatrixXd pseudoJ = J_stack.transpose() * (JJT+ 0.0025 * Eigen::MatrixXd::Identity(JJT.rows(), JJT.cols())).inverse();
		
		Eigen::VectorXd tempPose = pseudoJ * dev_stack * 0.05;
		dummyHand->setPositions(currentPose+tempPose);	
		dummyHand->computeForwardKinematics(true,false,false);	
	}
	newPose = dummyHand->getPositions();
	// std::cout << "setting pose" <<std::endl;
	// for(int i = 0; i < Ends.size(); ++i)
	// 	std::cout << dummyHand->getBodyNode(Ends[i].second)->getCOM().transpose()<<std::endl;	
	return newPose;
}

Eigen::VectorXd IkSolver::ConstrainedIKSubjectTo
(const dart::simulation::WorldPtr& world, std::string hand_name,std::string obj_name, std::vector<std::pair<Eigen::Vector6d, std::string>> Ends, std::string relativeFrame, int total_iter)
{
	dart::simulation::WorldPtr currentWorld = world->clone();
	SkeletonPtr dummyHand = currentWorld->getSkeleton(hand_name);
	Eigen::VectorXd initGuess = dummyHand->getPositions();
	Eigen::VectorXd target;
	int bookmark = dummyHand->getBodyNode("metacarpal2")->getIndexInSkeleton();

	for(int i = 0; i < total_iter; ++i)
	{
		std::cout << i << "iter " << std::endl;
		dummyHand->setPositions(initGuess);
		Eigen::VectorXd pose = this->IKSubjectTo(dummyHand, Ends, relativeFrame, 1000);
		dummyHand->setPositions(pose);
		if(this->isSatisfyConstrain(currentWorld, hand_name, obj_name))
		{
			target = dummyHand->getPositions();
			break;
		}
		initGuess = dart::math::Random::uniform<Eigen::VectorXd>(initGuess.size(), -M_PI/10, M_PI/10);
		for(int i = bookmark; i < initGuess.size();++i)
			initGuess(i) = 0;

	}
	return target;
}

bool IkSolver::isSatisfyConstrain(const dart::simulation::WorldPtr& world, std::string hand_name,std::string obj_name)
{
	
	bool checker = true;
	//Collision Constraint
	SkeletonPtr currentHand = world->getSkeleton(hand_name);
	auto collisionEngine = world->getConstraintSolver()->getCollisionDetector();
	auto hand_wo_finger = collisionEngine->createCollisionGroup(currentHand.get());
	auto hand_w_finger = collisionEngine->createCollisionGroup();
	auto objectGroup = collisionEngine->createCollisionGroup(world->getSkeleton(obj_name).get());
	hand_wo_finger->removeShapeFramesOf(currentHand->getBodyNode("thumb distphalanx"));
	hand_wo_finger->removeShapeFramesOf(currentHand->getBodyNode("thumb dummy"));
	hand_wo_finger->removeShapeFramesOf(currentHand->getBodyNode("thumbpatch"));
	hand_w_finger->addShapeFramesOf(currentHand->getBodyNode("thumb distphalanx"));
	hand_w_finger->addShapeFramesOf(currentHand->getBodyNode("thumb dummy"));
	hand_w_finger->addShapeFramesOf(currentHand->getBodyNode("thumbpatch"));

	double error = 0.02;

	checker = checker && hand_w_finger->distance(objectGroup.get()) < error ;

	for(int i =0; i < 4; ++i)
	{
		hand_wo_finger->removeShapeFramesOf(currentHand->getBodyNode("distphalanx" + std::to_string(i)));
		hand_wo_finger->removeShapeFramesOf(currentHand->getBodyNode("dummy" + std::to_string(i)));
		hand_wo_finger->removeShapeFramesOf(currentHand->getBodyNode("patch" + std::to_string(i)));
	}

	for(int i = 0; i < 2; ++i)
	{
		hand_w_finger->removeShapeFramesOf(currentHand->getBodyNode("thumb distphalanx"));
		hand_w_finger->removeShapeFramesOf(currentHand->getBodyNode("thumb dummy"));
		hand_w_finger->removeShapeFramesOf(currentHand->getBodyNode("thumbpatch"));
		hand_w_finger->addShapeFramesOf(currentHand->getBodyNode("distphalanx" + std::to_string(i)));
		hand_w_finger->addShapeFramesOf(currentHand->getBodyNode("dummy" + std::to_string(i)));
		hand_w_finger->addShapeFramesOf(currentHand->getBodyNode("patch" + std::to_string(i)));
		checker = checker && hand_w_finger->distance(objectGroup.get()) <error ;
	}

	checker = checker && !hand_wo_finger->collide(objectGroup.get());
	return checker;
}


Eigen::VectorXd IkSolver::stepByStep
(const SkeletonPtr& hand, std::vector<std::pair<Eigen::Vector6d, std::string>> Ends, std::string relativeFrame, int total_iter)
{
	SkeletonPtr dummyHand = hand->cloneSkeleton();
	std::vector<Eigen::Vector3d> originalCOMs;
	std::vector<Eigen::Vector3d> originalAngs;

	for(int i=0; i< Ends.size();++i)
	{
		originalCOMs.push_back(dummyHand->getBodyNode(Ends[i].second)->getCOM());
		originalAngs.push_back(dart::math::logMap(dummyHand->getBodyNode(Ends[i].second)->getWorldTransform().rotation()));
	}


	std::vector<std::pair<Eigen::Vector6d, std::string>> interEnds;
	for(int i = 0; i < 10; ++i)
	{
		for(int j = 0; j < Ends.size(); ++j)
		{
			Eigen::Vector6d inter_pose;
			inter_pose.head<3>() = (Ends[j].first.head<3>() - originalAngs[j]) *(i+1) / 10 + originalAngs[j];	
			inter_pose.tail<3>() = (Ends[j].first.tail<3>() - originalCOMs[j]) *(i+1) / 10 + originalCOMs[j];
			interEnds.push_back(std::make_pair(inter_pose, Ends[j].second));
		}
		Eigen::VectorXd temp = this->IKSubjectTo(dummyHand, interEnds, relativeFrame, total_iter);
		dummyHand->setPositions(temp);
		interEnds.clear();
	}
	return dummyHand->getPositions();
}

Eigen::VectorXd IkSolver::IKSubjectTo
(const SkeletonPtr& hand, std::vector<std::pair<Eigen::Vector6d, std::string>> Ends, std::string relativeFrame, int total_iter)
{
	SkeletonPtr dummyHand = hand->cloneSkeleton();
	// dummyHand->setSelfCollisionCheck(false);
	// std::cout << "current" <<std::endl;
	// for(int i = 0; i < Ends.size(); ++i)
	// 	std::cout << dummyHand->getBodyNode(Ends[i].second)->getCOM().transpose()<<std::endl;
	// std::cout << "target" << std::endl;
	// for(int i =0 ; i < Ends.size(); ++i)
	// 	std::cout << Ends[i].first.transpose() << std::endl;

	Eigen::MatrixXd J_stack = Eigen::MatrixXd::Zero(Ends[0].first.rows()*Ends.size(),hand->getPositions().size());
	Eigen::VectorXd dev_stack;
	dev_stack = Eigen::VectorXd::Zero(Ends.size()*6); 
	Eigen::VectorXd newPose;
	double epsilon = 0.001;

	for(int current_iter = 0; current_iter < total_iter ;current_iter++)
	{
		Eigen::VectorXd currentPose = dummyHand->getPositions();
		double total_dev = 0;
		for(int i = 0 ; i < Ends.size() ; i ++){
			BodyNode* currentPart;
			BodyNode* spatialNode;
			spatialNode = dummyHand->getBodyNode(relativeFrame);
			// dart::dynamics::Frame *worldFrame  = dummyHand->getBodyNode(0)->getParentFrame();
			dart::dynamics::Frame *worldFrame  = dart::dynamics::Frame::World();
			
			currentPart = dummyHand->getBodyNode(Ends[i].second);
			Eigen::Vector3d ang_dev = (Ends[i].first.head<3>() - dart::math::logMap(currentPart->getWorldTransform().rotation()));
			Eigen::Vector3d deviation = (Ends[i].first.tail<3>() - currentPart->getCOM());
			total_dev += deviation.norm();
			total_dev += ang_dev.norm();
			Eigen::MatrixXd J;
			if(relativeFrame == "world")
				J = dummyHand->getJacobian(currentPart);
			else
				J = dummyHand->getJacobian(currentPart, spatialNode, worldFrame);
			// Eigen::MatrixXd J = dummyHand->getJacobian(currentPart);
			J_stack.block(J.rows()*i,0,J.rows(),J.cols()) = J;
			dev_stack(6*i) = ang_dev[0];
			dev_stack(6*i+1) = ang_dev[1];
			dev_stack(6*i+2) = ang_dev[2];
			dev_stack(6*i+3) = deviation[0];
			dev_stack(6*i+4) = deviation[1];
			dev_stack(6*i+5) = deviation[2];
		}
		if(total_dev < epsilon) break;
		Eigen::MatrixXd JJT = J_stack*J_stack.transpose();
		Eigen::MatrixXd pseudoJ = J_stack.transpose() * (JJT+ 0.0025 * Eigen::MatrixXd::Identity(JJT.rows(), JJT.cols())).inverse();
		
		Eigen::VectorXd tempPose = pseudoJ * dev_stack * 0.02;
		dummyHand->setPositions(currentPose+tempPose);
		dummyHand->computeForwardKinematics(true,false,false);
	}
	newPose = dummyHand->getPositions();
	// std::cout << "setting pose" <<std::endl;
	// for(int i = 0; i < Ends.size(); ++i)
	// 	std::cout << dummyHand->getBodyNode(Ends[i].second)->getCOM().transpose()<<std::endl;	
	return newPose;
}

Eigen::VectorXd IkSolver::IKMultipleFixedWeighted(const SkeletonPtr& hand, std::vector<std::pair<Eigen::Vector3d, std::string>> Ends, const Eigen::VectorXd fixed_weight,int total_iter)
{
	SkeletonPtr dummyHand = hand->cloneSkeleton();
	Eigen::MatrixXd J_stack = Eigen::MatrixXd::Zero(Ends[0].first.rows()*Ends.size(),hand->getPositions().size());
	Eigen::VectorXd dev_stack;
	dev_stack = Eigen::VectorXd::Zero(Ends.size()*3); 
	Eigen::VectorXd newPose;
	double epsilon = 0.005;

	for(int current_iter = 0; current_iter < total_iter ;current_iter++)
	{
		Eigen::VectorXd currentPose = dummyHand->getPositions();
		double total_dev = 0;
		for(int i = 0 ; i < Ends.size() ; i ++){
			BodyNode* currentPart;
			currentPart = dummyHand->getBodyNode(Ends[i].second);
			Eigen::Vector3d deviation = (Ends[i].first - currentPart->getCOM());
			total_dev += deviation.norm();
			Eigen::MatrixXd J = dummyHand->getLinearJacobian(currentPart);
			//Weight Parameterization with fixed weights
			for(int j = 0; j <hand->getPositions().size(); ++j)
			{
				J.col(j) *= fixed_weight[j];
			}
			J_stack.block(J.rows()*i,0,J.rows(),J.cols()) = J;
			dev_stack(3*i) = deviation[0];
			dev_stack(3*i+1) = deviation[1];
			dev_stack(3*i+2) = deviation[2];
		}
		if(total_dev < epsilon) break;
		Eigen::MatrixXd JJT = J_stack*J_stack.transpose();
		Eigen::MatrixXd pseudoJ = J_stack.transpose() * (JJT+ 0.0025 * Eigen::MatrixXd::Identity(JJT.rows(), JJT.cols())).inverse();

		Eigen::VectorXd tempPose = pseudoJ * dev_stack * 0.05;
		dummyHand->setPositions(currentPose+tempPose);	
	}
	newPose = dummyHand->getPositions();

	return newPose;
}
