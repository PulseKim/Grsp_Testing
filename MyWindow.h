#include <iostream>
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/utils/utils.hpp"
#include "HandMaker.h"
#include "Controller.h"
#include "IkSolver.h"
#include "Contact.h"

using namespace dart::simulation;
using namespace dart::gui::glut;

class MyWindow: public SimWindow
{
public:
	MyWindow(const WorldPtr& world);

	//Initialization
	void initParameters();
	void initWindowSetting();
	void initSkeleton();
	void setSkeleton();
	void addSkeleton();

	//Overriding functions
	void keyboard(unsigned char key, int x, int y) override;
	void timeStepping() override;
	void draw() override;

	//Useful functions
	Eigen::VectorXd ForwardKinematicsMovement(int current_idx, int total_steps, const Eigen::VectorXd original, const Eigen::VectorXd target);
	void playSavedMovement(std::string filePath);
	double degToRad(double degree);
	void tempCollision();
	void testing();
	void drawFrame();
	void showDirection(bool flag, Eigen::Vector3d begin, Eigen::Vector3d dir);

protected:
	SkeletonPtr	mFloor;
	SkeletonPtr mHand;
	SkeletonPtr mObject;
	std::unique_ptr<HandMaker> mHandMaker;
	std::unique_ptr<Controller> mController;
	std::unique_ptr<IkSolver> mIKsolver;

public:
	double rad_obj;
	double height_obj;
};