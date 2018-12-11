
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

MatrixXd TransformP(Eigen::MatrixXd &V, Eigen::Matrix4d &tr)
{
	MatrixXd th = V.rowwise().homogeneous().transpose();
	return (tr * th).transpose().leftCols(3);
}

void RSApp::LoadMeshModelsIntoViewer()
{
	int ii = 0;
	for (auto&& i : rbEngine->rbs)
	{
		cout << i->meshFileName << endl;
		igl::readOBJ(i->meshFileName, V, F);
		if (ii != 0)
		{
			Matrix4d tr(Matrix4d::Identity());
			tr(2,3) = 0.3;
			//tr(3, 1) = 5;
			//tr(3, 2) = p(2);
			V = TransformP(V, tr);
		}
		viewer.data_list[ii].set_mesh(V, F);
		viewer.append_mesh();
		ii++;

		/*if (ii == 3)
			break;*/
	}	
}
