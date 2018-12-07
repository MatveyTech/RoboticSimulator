
//#include "..\include\Utils_my.h"
#include "..\include\RSApp.h"
#include <Eigen/Dense>


using namespace Eigen;

RSApp::RSApp(void)
{
	const char* fName = "C:/Users/matvey/Documents/CS2/Graphics project/RoboticSimulator/data/rbs/yumi/yumi.rbs";//TODOMATVEY:Change this
	loadFile(fName);
	LoadMeshModelsIntoViewer();
	viewer.launch();
}

void RSApp::loadFile(const char* fName) {
	//Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

	if (fNameExt.compare("rbs") == 0)
		loadRobot(fName);
	if (fNameExt.compare("rs") == 0) {
		if (robot) {
			robot->loadReducedStateFromFile(fName);
			startState = RobotState(robot);
			ikSolver->ikPlan->setTargetIKStateFromRobot();
		}
	}
}

void RSApp::loadRobot(const char* fName) {
	delete robot;
	delete rbEngine;
	
	rbEngine = new RBEngine();
	rbEngine->loadRBsFromFile(fName);
	robot = new Robot(rbEngine->rbs[0]);
	//robot = make_shared<Robot*>(new Robot(rbEngine->rbs[0]));
	startState = RobotState(robot);
	setupSimpleRobotStructure(robot);

	//rbEngine->GetNumOfrbs();

	delete ikSolver;
	ikSolver = new IK_Solver(robot, true);
}

void RSApp::LoadMeshModelsIntoViewer()
{
	for (auto&& i : rbEngine->rbs)
	{
		cout << i->meshFileName << endl;
		igl::readOBJ(i->meshFileName, V, F);
		break;
	}
	viewer.data_list[0].set_mesh(V, F);
}
