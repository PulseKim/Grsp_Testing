#ifndef	_HANDMAKER_H_
#define	_HANDMAKER_H_

#include "SkelParser.h"
#include <iostream>

using namespace dart::simulation;
using namespace dart::gui::glut;

class HandMaker{
public: 
	HandMaker(const SkeletonPtr& inHand, std::string dir);	
	void makeHand();
	void makePalm();
	void makeFingers();
	void makeSingleFinger(int idx);
	void makeThumb();

public:
	SkeletonPtr mHand;
	SkelParser skel;
	int currentDirection;
	BodyNode* currentParent;
	Eigen::Vector3d palmSize;
};

#endif