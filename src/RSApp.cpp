
#include "..\include\RSApp.h"
#include <Eigen/Dense>
#include <fstream>
#include <chrono>

#define COLOR(r,g,b) RowVector3d(r / 255., g / 255., b / 225.)


using namespace Eigen;

//always returns 7nth left link
RigidBody * GetCurrentActiveLink(Robot* robot)
{
	for (int i = 0; i < robot->getRigidBodyCount(); i++)
		if (robot->getRigidBody(i)->name.compare("link_7_l") == 0)
			return robot->getRigidBody(i);
	return nullptr;
}

void RSApp::PrintRenderingTime()
{
	auto now = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, milli> diff_ms = now - last_rendered;
	last_rendered = now;
	cout << diff_ms.count() << endl;
}

void RSApp::MoveActiveLink(P3D point, bool isAbsolute)
{
	RigidBody* rb = GetCurrentActiveLink(robot);
	ikSolver->ikPlan->endEffectors.clear();
	ikSolver->ikPlan->endEffectors.push_back(IK_EndEffector());
	ikSolver->ikPlan->endEffectors.back().endEffectorLocalCoords = P3D(0, 0, 0);
	ikSolver->ikPlan->endEffectors.back().endEffectorRB = rb;

	P3D cart = rb->getWorldCoordinates(P3D(0, 0, 0));
	cout << "new point:(" << cart.x() << ";" << cart.y() << ";" << cart.z() << ")." << endl;

	ikSolver->ikEnergyFunction->regularizer = 100;
	ikSolver->ikOptimizer->checkDerivatives = true;
	P3D newPoint = isAbsolute ? point : cart + point;
	ikSolver->ikPlan->endEffectors.back().targetEEPos = newPoint;
}

void RSApp::DefineViewerCallbacks()
{
	//viewer.core.is_animating = true;
	viewer.callback_pre_draw = 
	[&](igl::opengl::glfw::Viewer & v)
	{
		//ikSolver->solve(10, false, false);
		//PrintRenderingTime();
		DrawAll();
		return false;
	};
	if (CartMode)
	{
		viewer.callback_key_down =
			[&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
		{
			double step = 0.035;

			switch (key)
			{
			case GLFW_KEY_UP:
				MoveActiveLink(P3D(0, step, 0));
				return true;
			case GLFW_KEY_DOWN:
				MoveActiveLink(P3D(0, -step, 0));
				return true;
			case GLFW_KEY_LEFT:
				MoveActiveLink(P3D(0, 0, step));
				return true;
			case GLFW_KEY_RIGHT:
				MoveActiveLink(P3D(0, 0, -step));
				return true;
			case GLFW_KEY_PAGE_UP:
				MoveActiveLink(P3D(-step, 0, 0));
				return true;
			case GLFW_KEY_PAGE_DOWN:
				MoveActiveLink(P3D(step, 0, 0));
			case GLFW_KEY_S:
				ikSolver->solve(100, false, false);
				return true;
			case GLFW_KEY_P:
				//MoveActiveLink(P3D(0.38,0.44,-0.57),true);
				//rbEngine->drawRBs();
				robot->PrintJointsValues();
				return true;
			case GLFW_KEY_A:
			{
				DynamicArray<double> newJoints;
				newJoints.resize(14, 0);
				//newJoints[1] = 58; newJoints[3] = 15; newJoints[5] = 68; newJoints[7] = 300;
				//newJoints[9] = 258; newJoints[11] = 72; newJoints[13] = 72;
				newJoints[0] = 90; newJoints[2] = 50;
				robot->MoveByJoints(newJoints);
				return true;
			}
			case GLFW_KEY_R:
			{
				DynamicArray<double> newJoints;
				newJoints.resize(14, 0);
				robot->MoveByJoints(newJoints);
				return true;
			}
			default:
				return false;
			}
			return false;
		};
	}
	else
	{
		viewer.callback_key_down =
			[&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
		{
			double step = 0.035;

			switch (key)
			{
			case GLFW_KEY_LEFT:
				MoveActiveLink(P3D(0, 0, step));
				return true;
			case GLFW_KEY_RIGHT:
				MoveActiveLink(P3D(0, 0, -step));
				return true;
			case GLFW_KEY_S:
				ikSolver->solve(100, false, false);
				return true;
			case GLFW_KEY_P:
				robot->PrintJointsValues();
				return true;
			case GLFW_KEY_A:
			{
				VectorXd v(7);
				v << 0,90,0,0,0,0,0;
				robot->MoveByJointsR(v);
				robot->PrintJointsValues();
				return true;

				//DynamicArray<double> newJoints;
				//newJoints.resize(14, 0);
				////newJoints[1] = 58; newJoints[3] = 15; newJoints[5] = 68; newJoints[7] = 300;
				////newJoints[9] = 258; newJoints[11] = 72; newJoints[13] = 72;
				//newJoints[0] = 90; newJoints[2] = 50;
				//robot->MoveByJoints(newJoints);
				//return true;
			}
			case GLFW_KEY_R:
			{
				DynamicArray<double> newJoints;
				newJoints.resize(14, 0);
				robot->MoveByJoints(newJoints);
				return true;
			}
			default:
				return false;
			}
			return false;
		};
	}

}

void RSApp::CreateIKSolver()
{
	delete ikSolver;
	ikSolver = new IK_Solver(robot, true);
}

RSApp::RSApp(void)
{
	bool useSimpleRobot = true;
	bool useSerializedModels = false;
	char* fName;
	if (useSimpleRobot)
		fName = "C:/Users/matvey/Documents/CS2/Graphics project/RoboticSimulator/data/rbs/yumi/yumi_simplified.rbs";//TODOMATVEY:Change this
	else
		fName = "C:/Users/matvey/Documents/CS2/Graphics project/RoboticSimulator/data/rbs/yumi/yumi.rbs";//TODOMATVEY:Change this
	loadFile(fName);
	LoadMeshModelsIntoViewer(useSerializedModels);
	DefineViewerCallbacks();

	CreateIKSolver();
	
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

	
}

MatrixXd TransformP(Eigen::MatrixXd &V, Eigen::Matrix4d &tr)
{
	MatrixXd th = V.rowwise().homogeneous().transpose();
	return (tr * th).transpose().leftCols(3);
}

void RSApp::LoadMeshModelsIntoViewer(bool useSerializedModels)
{
	int ii = 0;
	for (auto&& i : rbEngine->rbs)
	{
		Eigen::Matrix4d m = i->meshtransformation;
		cout << i->meshFileName << endl;
		string VFile = "../RoboticSimulator/data/rbs/yumi/meshes_ser/V_link_" + std::to_string(ii);
		string FFile = "../RoboticSimulator/data/rbs/yumi/meshes_ser/F_link_" + std::to_string(ii);

		if (useSerializedModels)
		{
			igl::deserialize(i->vertices, "V", VFile);
			igl::deserialize(i->faces, "F", FFile);			
		}
		else
		{
			igl::readOBJ(i->meshFileName, i->vertices, i->faces);
			/*igl::serialize(i->vertices, "V", VFile);
			igl::serialize(i->faces, "F", FFile);*/
		}

		MatrixXd transformed_vert = TransformP(i->vertices, i->meshtransformation);
		viewer.data_list[ii].set_mesh(transformed_vert, i->faces);
		viewer.data_list[ii].show_lines = false;
		if (ii == 0)
			//viewer.data_list[ii].set_colors(COLOR(105, 105, 105));
			viewer.data_list[ii].set_colors(COLOR(227, 100, 33));//orange
		else if (ii < 8 && ii % 2 == 1 || ii >= 8 && ii % 2 == 0)
			viewer.data_list[ii].set_colors(COLOR(230, 230, 227));
		else
			viewer.data_list[ii].set_colors(COLOR(227, 100, 33));//orange

		viewer.append_mesh();
		ii++;
	}
	DrawAll();
}

void RSApp::DrawAll()
{
	int ii = 0;
	for (auto&& i : rbEngine->rbs)
	{
		if (ii > 0) // base isn't moving so no need to redraw it
		{
			QuaternionR q = i->state.orientation;
			Matrix3x3 rotationM = q.getRotationMatrix();
			Matrix4x4 m = Matrix4x4::Identity();
			m.topLeftCorner(3, 3) = rotationM;
			m.topRightCorner(3, 1) = Vector3d(i->state.position[0], i->state.position[1], i->state.position[2]);
			Matrix4x4 res = m * i->meshtransformation;
			MatrixXd transformed_vert = TransformP(i->vertices, res);
			viewer.data_list[ii].set_mesh(transformed_vert, i->faces);
		}
		ii++;
	}
}