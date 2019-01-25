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
#include <chrono>


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
	chrono::steady_clock::time_point last_rendered;
	
	//shared_ptr<Robot*> robot;
	void PrintRendeingTime();
	void MoveActiveLink(P3D delta);
	void DefineViewerCallbacks();

public:
	
	void CreateIKSolver();

	RSApp(void);
	virtual void loadFile(const char* fName);
	void loadRobot(const char* fName);
	void LoadMeshModelsIntoViewer(bool useSerializedModels);
	void DrawAll();
};

