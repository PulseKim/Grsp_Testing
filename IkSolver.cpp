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
			//Weight Parameterize for arm and palm
			for(int j = 0; j <7 ; ++j){
				J.col(j) *=2;
				// J.col(i) = J.col(i) * (2.5 *(total_iter-current_iter)/total_iter);
			}

			// //Weight Parameterize for thumb
			for(int j = 0; j <5; ++j)
				J.col(7+j) = J.col(7+j) * (current_iter)/(total_iter);
			//Weight Parameterize for fingers
			for(int k = 0; k <4 ; ++k){
				for(int j = 0; j <4; ++j)
					J.col(4*k+12+j) = J.col(4*k+12+j) * (current_iter)/(total_iter);
			}

			J.col(dummyHand->getPositions().size()-1) = J.col(dummyHand->getPositions().size()-1) * (2.3*(total_iter-current_iter+2)/total_iter);

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