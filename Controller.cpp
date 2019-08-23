#include "Controller.h"

const double rad = M_PI /180.0;

Controller::Controller(const SkeletonPtr& finger): mFinger(finger)
{
    this->jointControlSetter();
}

void Controller::jointControlSetter()
{
	int nDofs = mFinger->getNumDofs();
    int mag_Kp = 1000;

    mForces = Eigen::VectorXd::Zero(nDofs);
    mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
    mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);

    // for(std::size_t i = 0; i < 6; ++i){
    //     mKp(i,i) = 0;
    //     mKd(i,i) = 0;
    // }
    for(std::size_t i = 0; i < nDofs; ++i){
        mKp(i,i) = mag_Kp;
        mKd(i,i) = 2 * std::sqrt(mag_Kp);
    }
    this->setTargetPosition(mFinger->getPositions());
}

void Controller::setTargetPosition(const Eigen::VectorXd& pose)
{
	mTargetPositions = pose;
}

void Controller::clearForces()
{
	mForces.setZero();
}

Eigen::VectorXd Controller::getForces()
{
    return mForces;
}

void Controller::addSPDForces()
{
	Eigen::VectorXd q = mFinger->getPositions();
    Eigen::VectorXd dq = mFinger->getVelocities();
    for(int i = 0; i < q.size(); ++i){
        if(q[i]-mTargetPositions[i] > M_PI){
            q[i]-=2*M_PI;
            dq[i]-=2*M_PI;
        }
        else if(q[i]-mTargetPositions[i] < -M_PI){
            q[i]+=2*M_PI;
            dq[i]+=2*M_PI;
        } 
    }

    Eigen::MatrixXd invM = (mFinger->getMassMatrix()
        + mKd * mFinger->getTimeStep()).inverse();
    Eigen::VectorXd p =
    -mKp * (q + dq * mFinger->getTimeStep() - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    Eigen::VectorXd qddot =
    invM * (-mFinger->getCoriolisAndGravityForces()
        + p + d + mFinger->getConstraintForces());
    
    mForces += p + d - mKd * qddot * mFinger->getTimeStep();
    mFinger->setForces(mForces);
    
}

void Controller::setFreeJointPosition()
{

    for(int i = 0; i < 6; i++){
        mFinger->setPosition(i, mTargetPositions(i));
        mFinger->setVelocity(i, 0);
    }
}

void Controller::addPDForces()
{
    Eigen::VectorXd q = mFinger->getPositions();
    Eigen::VectorXd dq = mFinger->getVelocities();
    int cnt = 1;
    while(cnt !=0){
        cnt = 0;
        for(int i = 6; i < q.size(); ++i){
            if(q[i]-mTargetPositions[i] > M_PI) {
                q[i]-=2*M_PI;
                dq[i]-=2*M_PI;
                cnt =1;
            }
            else if(q[i]-mTargetPositions[i] < -M_PI){
                q[i]+=2*M_PI;
                dq[i]+=2*M_PI;
                cnt =1;
            } 
        }
    }
    Eigen::VectorXd p = -mKp * (q - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;

    mForces += p+d;
    std::cout <<"q" << q.transpose() <<std::endl;
    std::cout <<"p" << p.transpose() <<std::endl;
    // std::cout << "d"  << d.transpose() << std::endl;
    mFinger->setForces(mForces);
}