#include "HandMaker.h"

HandMaker::HandMaker(const SkeletonPtr& inHand, std::string dir) : mHand(inHand)
{
	if(dir == "left") currentDirection = 0;
	else if(dir == "right") currentDirection = 1;
	else currentDirection = -1;
}

void HandMaker::makeHand()
{
	this->makePalm();
	this->makeFingers();
	int indicator = currentDirection == 0 ? 1 : -1;
	int index_thumb = mHand->getIndexOf(mHand->getBodyNode("thumb metacarpal")->getParentJoint()->getDof(0));
	mHand->setPosition(index_thumb, 30*M_PI/180);
	mHand->setPosition(index_thumb+1, indicator* -60*M_PI/180);
	mHand->setPosition(index_thumb+3, indicator* 20*M_PI/180);
}

void HandMaker::makePalm()
{
	BodyNode* bn;
	Eigen::Vector3d boxsize(0.1, 0.02, 0.09);
	palmSize = boxsize;
	Eigen::Vector3d color = dart::Color::Green();
	double mass = 0.1;
	bn = skel.freeBox(mHand, "palm", boxsize, Eigen::Vector3d(0,-0.3,0), mass, color);
	currentParent = bn;
}

void HandMaker::makeFingers()
{
	this->makeThumb();
	for(int i = 0; i < 4; ++i)
		this->makeSingleFinger(i);
}


void HandMaker::makeSingleFinger(int idx)
{
	BodyNode* bn;
	Eigen::Vector3d color = dart::Color::Green();
	double gap = 0.001;
	double fing_x = palmSize[0] / 4 - gap;
	double fing_len = 0.07;
	double mass_long = 0.04;
	double mass_short = 0.02;
	double epsilon = 1E-10;
	Eigen::Vector3d patch(0.005, epsilon, 0.005);
	double offset;

	if(currentDirection == 0)
		offset = -palmSize[0]/2 + gap*idx + fing_x/2 + fing_x * idx;
	else if(currentDirection == 1)
		offset = palmSize[0]/2 -(gap*idx + fing_x/2 + fing_x * idx);
	Eigen::Vector3d boxsize(fing_x, palmSize[1], fing_len);
	bn = skel.univBox(mHand, currentParent, "metacarpal"+std::to_string(idx), Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), boxsize, Eigen::Vector3d(0,0,-boxsize[2]/2), Eigen::Vector3d(offset, 0, palmSize[2]/2), mass_long , color);
	boxsize[2] = 0.05;
	bn = skel.revolBox(mHand, bn, "proxphalanx"+std::to_string(idx), Eigen::Vector3d::UnitX(), boxsize, Eigen::Vector3d(0,0,-boxsize[2]/2), Eigen::Vector3d(0,0,fing_len/2), mass_short, color);
	fing_len = 0.05;
	boxsize[2] = 0.03;
	bn = skel.revolBox(mHand, bn, "distphalanx"+std::to_string(idx), Eigen::Vector3d::UnitX(), boxsize, Eigen::Vector3d(0,0,-boxsize[2]/2), Eigen::Vector3d(0,0,fing_len/2), mass_short, color);		
	bn = skel.weldBox(mHand, bn, "patch" + std::to_string(idx), patch, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,-boxsize[1]/2, boxsize[2]/2-patch[2]/2), epsilon, dart::Color::Orange());
}


void HandMaker::makeThumb()
{
	BodyNode* bn;
	Eigen::Vector3d color = dart::Color::Green();
	double fing_x = palmSize[0] / 3;
	double fing_len = 0.06;
	double theta = 30 * M_PI / 180;
	double mass_long = 0.04;
	double mass_short = 0.02;
	double epsilon = 1E-10;
	Eigen::Vector3d patch(epsilon, 0.005, 0.005);
	Eigen::Vector3d offset;
	if(currentDirection == 0)
		offset = Eigen::Vector3d(-palmSize[0]/2, 0, -palmSize[2]/2);
	else if(currentDirection == 1)
		offset = Eigen::Vector3d(palmSize[0]/2, 0, -palmSize[2]/2);
	Eigen::Vector3d boxsize(palmSize[1], fing_x,  fing_len);

	bn = skel.univBox(mHand, currentParent, "thumb metacarpal", Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), boxsize, Eigen::Vector3d(0,0,-boxsize[2]/2), offset,mass_long ,color);
	boxsize[2] = 0.05;
	bn = skel.univBox(mHand, bn, "thumb proxphalanx", Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), boxsize, Eigen::Vector3d(0,0,-boxsize[2]/2), Eigen::Vector3d(0,0,fing_len/2),mass_short ,color);	
	fing_len = 0.05;
	boxsize[2] = 0.03;
	bn = skel.revolBox(mHand, bn, "thumb distphalanx", Eigen::Vector3d::UnitY(), boxsize, Eigen::Vector3d(0,0,-boxsize[2]/2), Eigen::Vector3d(0,0,fing_len/2), mass_short, color);		
	if(currentDirection == 0)
		bn = skel.weldBox(mHand, bn, "thumbpatch", patch, Eigen::Vector3d(0,0,0), Eigen::Vector3d(boxsize[0]/2,0, boxsize[2]/2-patch[2]/2), epsilon, dart::Color::Orange());
	else
		bn = skel.weldBox(mHand, bn, "thumbpatch", patch, Eigen::Vector3d(0,0,0), Eigen::Vector3d(-boxsize[0]/2,0, boxsize[2]/2-patch[2]/2), epsilon, dart::Color::Orange());


}