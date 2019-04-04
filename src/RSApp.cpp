
#include "..\include\RSApp.h"
#include "..\include\StartFromFirstObjective.h"
#include "..\include\FinishAtLastObjective.h"
#include "..\include\EqualDistributionObjective.h"
#include <fstream>
#include <chrono>
#include <igl/readOFF.h>

#define COLOR(r,g,b) RowVector3d(r / 255., g / 255., b / 225.)


//always returns 7nth right link
RigidBody * GetCurrentActiveLink(Robot* robot)
{
	for (int i = 0; i < robot->getRigidBodyCount(); i++)
		if (robot->getRigidBody(i)->name.compare("link_7_r") == 0)
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

	P3D cart = rb->getWorldCoordinates(kSolver->GetGripLocalCoordinates());
	cout << "new point:(" << cart.x() << ";" << cart.y() << ";" << cart.z() << ")." << endl;

	ikSolver->ikEnergyFunction->regularizer = 100;
	ikSolver->ikOptimizer->checkDerivatives = true;
	P3D newPoint = isAbsolute ? point : cart + point;
	ikSolver->ikPlan->endEffectors.back().targetEEPos = newPoint;
}


void RSApp::DefineViewerCallbacks()
{
	viewer.core.is_animating = true;
	viewer.callback_pre_draw = 
	[&](igl::opengl::glfw::Viewer & v)
	{
		if (CartMode)
			ikSolver->solve(10, false, false);
		//PrintRenderingTime();
		DrawPoint();
		DrawRobot();
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
				robot->MoveByJointsR(simulation->MoveToPrevAndGet());
				return true;
			case GLFW_KEY_RIGHT:
				robot->MoveByJointsR(simulation->MoveToNextAndGet());
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
				v << 50 ,-50,0,-50,0,0,0;
				robot->MoveByJointsR(v);
				robot->PrintJointsValues();
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

}

void RSApp::DrawPoint()
{
	P3D p = kSolver->CalcForwardKinematics(robot->GetQ());
	pointToDraw.row(0) = p;	
	viewer.data().set_points(pointToDraw, Eigen::RowVector3d(220/255.0, 220 / 255.0, 220 / 255.0));
}

void RSApp::CreateIKSolver()
{
	delete ikSolver;
	ikSolver = new IK_Solver(robot, true);
}

void RSApp::RecreateSimulation(std::vector<double> weights, MinimizerType mt)
{
	VectorXd v1(7); v1 << 0, 0, 0, 0, 0, 0, 0;
	VectorXd v2(7); v2 << 50, -50, 0, -50, 0, 0, 0;
	if (simulation != nullptr)
		delete simulation;
	//simulation = new BasicSimulation(v1, v2, PathSize);
	simulation = new AdvancedSimulation(v1, v2, PathSize, weights,(int)mt,robot,m_finalCart,m_onlyFinalCart);
}

RSApp::RSApp(void)
{
	bool useSimpleRobot = true;
	bool useSerializedModels = false;
	char* fName;
	if (useSimpleRobot)
		fName = "../RoboticSimulator/data/rbs/yumi/yumi_simplified.rbs";
	else
		fName = "../RoboticSimulator/data/rbs/yumi/yumi.rbs";
	m_eeLocalCoord = kSolver->GetGripLocalCoordinates();
	loadFile(fName);
	kSolver = new SingleArmKinematicsSolver(robot);
	LoadMeshModelsIntoViewer(useSerializedModels);
	DefineViewerCallbacks();

	CreateIKSolver();
	std::vector<double> weights = { 1,1,1};
	RecreateSimulation(weights,MinimizerType::GD);
	CreateMenu();
	viewer.selected_data_index = 0;
	viewer.core.camera_zoom = 0.5;
	//viewer.core.camera_eye = Eigen::Vector3f(5, 3, 0);
	//viewer.core.camera_eye += Eigen::Vector3f(0, 5, 0);
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
	DrawRobot();
	AddSphere(ii);
}

void RSApp::AddSphere(int ii)
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	string sphereFile = "../RoboticSimulator/data/models/sphere.off";

	igl::readOFF(sphereFile, V, F);
	Eigen::Matrix4d translation_matrix;
	translation_matrix << 
		0.07, 0, 0, 0.68,
		0, 0.07, 0, 0.78,
		0, 0, 0.07, 0.33,
		0, 0, 0, 1;
	viewer.data_list[ii].set_mesh(TransformP(V,translation_matrix), F);
	viewer.data_list[ii].show_lines = false;
	viewer.data_list[ii].set_colors(COLOR(0, 255, 0));
	
}

void RSApp::DrawRobot()
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
			Eigen::Matrix4d res = m * i->meshtransformation;
			MatrixXd transformed_vert = TransformP(i->vertices, res);
			viewer.data_list[ii].set_mesh(transformed_vert, i->faces);
		}
		ii++;
	}
}

void RSApp::CreateMenu()
{
	

	viewer.plugins.push_back(&menu);
	double doubleVariable = 0.1f; // Shared between two menus
	//menu.post_resize(200, 200);
	
	/*ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(0.0, 0.0));
	ImGui::SetWindowFontScale(1.2);*/

	menu.callback_draw_viewer_menu = [&]()
	{
		
		//ImGui::Checkbox("bool", &CartMode);
		// Draw parent menu content
		//menu.draw_viewer_menu();

		// Add new group
		/*if (ImGui::CollapsingHeader("New Group", ImGuiTreeNodeFlags_DefaultOpen))
		{*/
		// Expose variable directly ...
		//ImGui::InputDouble("double", &doubleVariable, 0, 0, "%.4f");

		// ... or using a custom callback
		//static bool boolVariable = true;
		//if (ImGui::Checkbox("bool", &boolVariable))
		//{
		//	// do something
		//	std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
		//}

		static int simulationPos = 1;

		ImGui::SliderInt("", &simulationPos, 1, PathSize);
		if (!CartMode)
			robot->MoveByJointsR(simulation->MoveToIndAndGet(simulationPos - 1));
		ImGui::SameLine();
		if (ImGui::Button("-"))
		{
			simulationPos = simulation->DecreaseCurrentIndex()+1;
			robot->MoveByJointsR(simulation->GetCurrent());
		}
		ImGui::SameLine();
		if (ImGui::Button("+"))
		{
			simulationPos = simulation->IncreaseCurrentIndex()+1;
			robot->MoveByJointsR(simulation->GetCurrent());
		}
		ImGui::SameLine();
		ImGui::Text("Step");
		static int w1 = 0;
		static int w2 = 0;
		static int w3 = 0;	

		auto int2char = [&](int val) -> char*
		{
			if (val == 0)
				return "1";
			else if (val == 1)
				return "10";
			else if (val == 2)
				return "100";
			else if (val == 3)
				return "1000";
			else if (val == 4)
				return "10000";
			else 
				return "BAD";
		};
		ImGui::Text("Weights:");
		int max_w = 4;
		ImGui::SliderInt("First", &w1, 0,max_w,int2char(w1));
		ImGui::SliderInt("Last", &w2, 0,max_w,int2char(w2));
		ImGui::SliderInt("Equal", &w3, 0,max_w,int2char(w3));
		ImGui::NewLine();
		
		static MinimizerType minimizerType = simulation->MinimizerType;
		const char* cc = minimizerType == MinimizerType::GD ? "GradDesc" : "BFGS";
		ImGui::SliderInt("Minimizer", &((int)minimizerType), 0, 1, cc);

		std::vector<double> weights = { (double)pow(10,w1),(double)pow(10,w2),(double)pow(10,w3) };
		
		ImGui::Checkbox("Only final position", &m_onlyFinalCart);
		if (m_onlyFinalCart)
		{
			static float fl[3];
			ImGui::InputFloat3("", fl, 2);
			m_finalCart = P3D(fl[0], fl[1], fl[2]);
			ImGui::SameLine();
			if (ImGui::Button("Reset", ImVec2(-1, 0)))
			{
				for (size_t i = 0; i < 3; i++)
					fl[i] = 0;
			}
		}

		if (ImGui::Button("Rebuild simulation", ImVec2(-1, 0)))
		{
			RecreateSimulation(weights, minimizerType);
		}
		

		if (ImGui::Button("Debug button", ImVec2(-1, 0)))
		{
			//
		}
#pragma region		ee_adjustments
		/*double step = 0.003;
		if (ImGui::Button("1+", ImVec2(-1, 0)))
		{
			m_eeLocalCoord.x() += step;
			cout << endl << m_eeLocalCoord << endl;
			P3D p = m_gcRobotRepresentation->getWorldCoordinatesFor(m_eeLocalCoord, robot->getRigidBody(13));
			pointToDraw << p(0), p(1), p(2);
			DrawPoint();			
		}
		if (ImGui::Button("1-", ImVec2(-1, 0)))
		{
			m_eeLocalCoord.x() += -step;
			cout << endl << m_eeLocalCoord << endl;
			P3D p = m_gcRobotRepresentation->getWorldCoordinatesFor(m_eeLocalCoord, robot->getRigidBody(13));
			pointToDraw << p(0), p(1), p(2);
			DrawPoint();
		}		
		if (ImGui::Button("2+", ImVec2(-1, 0)))
		{
			m_eeLocalCoord.y() += step;
			cout << endl << m_eeLocalCoord << endl;
			P3D p = m_gcRobotRepresentation->getWorldCoordinatesFor(m_eeLocalCoord, robot->getRigidBody(13));
			pointToDraw << p(0), p(1), p(2);
			DrawPoint();
		}
		if (ImGui::Button("2-", ImVec2(-1, 0)))
		{
			m_eeLocalCoord.y() += -step;
			cout << endl << m_eeLocalCoord << endl;
			P3D p = m_gcRobotRepresentation->getWorldCoordinatesFor(m_eeLocalCoord, robot->getRigidBody(13));
			pointToDraw << p(0), p(1), p(2);
			DrawPoint();
		}
		if (ImGui::Button("3+", ImVec2(-1, 0)))
		{
			m_eeLocalCoord.z() += step;
			cout << endl << m_eeLocalCoord << endl;
			P3D p = m_gcRobotRepresentation->getWorldCoordinatesFor(m_eeLocalCoord, robot->getRigidBody(13));
			pointToDraw << p(0), p(1), p(2);
			DrawPoint();
		}
		if (ImGui::Button("3-", ImVec2(-1, 0)))
		{
			m_eeLocalCoord.z() += -step;
			cout << endl << m_eeLocalCoord << endl;
			P3D p = m_gcRobotRepresentation->getWorldCoordinatesFor(m_eeLocalCoord, robot->getRigidBody(13));
			pointToDraw << p(0), p(1), p(2);
			DrawPoint();
		}*/
#pragma endregion 
		
		ImGui::SetNextWindowPos(ImVec2(340,0), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(0.0, 0.0));
		ImGui::SetWindowFontScale(1.2);
		// 1. Show a simple window.
		// Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets automatically appears in a window called "Debug".
		ImGui::Begin("Right Arm Joints");
		ImGui::SetWindowFontScale(1.2);
		ImGui::Text("%5.2f ", robot->GetJointValueR(0));
		ImGui::SameLine();
		ImGui::Text("%5.2f", robot->GetJointValueR(1));
		ImGui::SameLine();
		ImGui::Text("%5.2f", robot->GetJointValueR(2));
		ImGui::SameLine();
		ImGui::Text("%5.2f", robot->GetJointValueR(3));
		ImGui::SameLine();
		ImGui::Text("%5.2f", robot->GetJointValueR(4));
		ImGui::SameLine();
		ImGui::Text("%5.2f", robot->GetJointValueR(5));
		ImGui::SameLine();
		ImGui::Text("%5.2f", robot->GetJointValueR(6));
		
		ImGui::End();

		ImGui::SetNextWindowPos(ImVec2(800, 0), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(0.0, 0.0));
		
		// 1. Show a simple window.
		// Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets automatically appears in a window called "Debug".
		ImGui::Begin("Right Arm Cart");
		ImGui::SetWindowFontScale(1.2);
		P3D p2 = kSolver->CalcForwardKinematics(robot->GetQ());
		ImGui::Text("x:%5.2f y:%5.2f z:%5.2f", p2.x(), p2.y(), p2.z());
		ImGui::End();

		
		ImGui::SetNextWindowPos(ImVec2(0, 350), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(0.0, 0.0));
		ImGui::SetWindowFontScale(1.2);
		ImGui::Begin("Optimization");
		ImGui::SetWindowFontScale(1.2);
		if (ImGui::Button("Make step", ImVec2(-1, 0)))
		{
			simulation->MakeStep();
		}
		static bool autostep = false;
		if (ImGui::Button("Auto step", ImVec2(-1, 0)))
		{
			autostep = !autostep;
		}
		if (autostep)
			simulation->MakeStep();
		ImGui::Text("Iteration # %d", simulation->IterationNum);
		ImGui::Text("Value %E", simulation->ComputeValueInCurrentPoint());
		ImGui::Text("Grad  %E", simulation->ComputeGradientInCurrentPoint());
		ImGui::Text("Iterations:  %d", simulation->GetLastNumOfIterations());
		ImGui::End();


		//}
	};

}
