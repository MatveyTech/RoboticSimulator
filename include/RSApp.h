#pragma once

#include <stdio.h>
#include <memory>
#include "AbstractRBEngine.h"
#include "RBEngine.h"
#include "Robot.h"
#include "IK_Solver.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/serialize.h>
#include <igl/readOBJ.h>
#include <GLFW/glfw3.h>


using namespace std;
using namespace igl::opengl::glfw;
using namespace Eigen;
using namespace std;

class RSApp
{
private:
	Robot* robot = nullptr;
	AbstractRBEngine* rbEngine = nullptr;
	RobotState startState = RobotState(14);
	IK_Solver* ikSolver = nullptr;
	Viewer viewer;
	
	//shared_ptr<Robot*> robot;


public:
	RSApp(void);
	virtual void loadFile(const char* fName);
	void loadRobot(const char* fName);
	void LoadMeshModelsIntoViewer();
	void DrawAll();
};

