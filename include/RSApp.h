#pragma once

#include <stdio.h>
#include <memory>
#include "AbstractRBEngine.h"
#include "RBEngine.h"
#include "Robot.h"
#include "IK_Solver.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/serialize.h>
#include <igl/readOBJ.h>
#include <GLFW/glfw3.h>
#include <chrono>
#include "Simulation.h"
//#include <Eigen/Sparse>


using namespace std;
using namespace igl::opengl::glfw;
using namespace Eigen;
using namespace std;
using namespace igl::opengl::glfw::imgui;


class RSApp
{
private:
	Robot* robot = nullptr;
	AbstractRBEngine* rbEngine = nullptr;
	RobotState startState = RobotState(14);
	IK_Solver* ikSolver = nullptr;
	ImGuiMenu menu;
	Viewer viewer;
	Simulation* simulation;
	chrono::steady_clock::time_point last_rendered;
	
	//shared_ptr<Robot*> robot;
	void PrintRenderingTime(); 
	void MoveActiveLink(P3D point, bool isAbsolute=false);
	void DefineViewerCallbacks();
	void CreateMenu();
	bool CartMode = false;
	int PathSize = 15;
	//int joints[7] = { 0,0,0,0,0,0,0 };
public:
	
	void CreateIKSolver();
	void CreateSimulation();

	RSApp(void);
	virtual void loadFile(const char* fName);
	void loadRobot(const char* fName);
	void LoadMeshModelsIntoViewer(bool useSerializedModels); 
	void DrawAll();
};

