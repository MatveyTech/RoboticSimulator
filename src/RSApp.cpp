
//#include "..\include\Utils_my.h"
#include "..\include\RSApp.h"
#include <Eigen/Dense>
#include <fstream>

#define COLOR(r,g,b) RowVector3d(r / 255., g / 255., b / 225.)


using namespace Eigen;

RigidBody * GetRigidBody(Robot* robot)
{
	for (int i = 0; i < robot->getRigidBodyCount(); i++)
		if (robot->getRigidBody(i)->name.compare("link_2_l") == 0)
			return robot->getRigidBody(i);
	return nullptr;
}

RSApp::RSApp(void)
{
	const char* fName = "C:/Users/matvey/Documents/CS2/Graphics project/RoboticSimulator/data/rbs/yumi/yumi.rbs";//TODOMATVEY:Change this
	loadFile(fName);
	LoadMeshModelsIntoViewer();
	//viewer.core.camera_eye = Vector3f(3,3,0);

	viewer.callback_pre_draw =
		[&](igl::opengl::glfw::Viewer & v)
	{
		//DrawAll();
		return false;
	};
	viewer.core.is_animating = true;

	viewer.callback_key_down =
		[&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
	{
		if (key == GLFW_KEY_J)
		{
			rbEngine->drawRBs();
			return true;
		}

		if (key == GLFW_KEY_D)
		{
			DrawAll();
			return true;
		}

		if (key == GLFW_KEY_ENTER)
		{
			RigidBody* rb = GetRigidBody(robot);
			ikSolver->ikPlan->endEffectors.clear();
			ikSolver->ikPlan->endEffectors.push_back(IK_EndEffector());
			ikSolver->ikPlan->endEffectors.back().endEffectorLocalCoords = P3D(0, 0, 0);
			ikSolver->ikPlan->endEffectors.back().endEffectorRB = rb;
			ikSolver->ikPlan->endEffectors.back().targetEEPos = P3D(0.5, 0.5, 0);
			ikSolver->ikEnergyFunction->regularizer = 100;
			ikSolver->ikOptimizer->checkDerivatives = true;
			ikSolver->solve(10, false, false);

			rbEngine->drawRBs();
			return true;
		}
		return false;
	};

	

	viewer.launch();
}

bool isFileExists(const std::string& filename) {
	std::ifstream ifile(filename.c_str());
	return (bool)ifile;
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
		Eigen::Matrix4d m = i->meshtransformation;
		cout << i->meshFileName << endl;
		string VFile = "../RoboticSimulator/data/rbs/yumi/meshes_ser/V_link_" + std::to_string(ii);
		string FFile = "../RoboticSimulator/data/rbs/yumi/meshes_ser/F_link_" + std::to_string(ii);

		//if (!isFileExists(VFile))
		if(false)
		{
			igl::readOBJ(i->meshFileName, i->vertices, i->faces); 
			/*igl::serialize(i->vertices, "V", VFile);
			igl::serialize(i->faces, "F", FFile);*/
		}
		else
		{
			igl::deserialize(i->vertices, "V", VFile);
			igl::deserialize(i->faces, "F", FFile);
		}

		MatrixXd transformed_vert = TransformP(i->vertices, i->meshtransformation); 
		viewer.data_list[ii].set_mesh(transformed_vert, i->faces);
		viewer.data_list[ii].show_lines = false;
		if (ii==0)
			//viewer.data_list[ii].set_colors(COLOR(105, 105, 105));
			viewer.data_list[ii].set_colors(COLOR(227, 100, 33));//orange
		else if (ii<8 && ii%2==1 || ii>=8 && ii % 2 == 0)
			viewer.data_list[ii].set_colors(COLOR(230, 230, 227));
		else
			viewer.data_list[ii].set_colors(COLOR(227, 100, 33));//orange

		viewer.append_mesh();
		ii++;

		/*if (ii == 1)
			break;*/
	}
	DrawAll();
}

void RSApp::DrawAll()
{
	int ii = 0;
	for (auto&& i : rbEngine->rbs)
	{
		QuaternionR q = i->state.orientation;		
		Matrix3x3 rotationM = q.getRotationMatrix();
		Matrix4x4 m = Matrix4x4::Identity();
		m.topLeftCorner(3,3) = rotationM;
		m.topRightCorner(3, 1) = Vector3d(i->state.position[0], i->state.position[1], i->state.position[2]);
		Matrix4x4 res = m * i->meshtransformation;
		MatrixXd transformed_vert = TransformP(i->vertices, res);
		viewer.data_list[ii].set_mesh(transformed_vert, i->faces);
		ii++;
	}
}
