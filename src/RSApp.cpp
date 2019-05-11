
#include "..\include\RSApp.h"
#include "..\include\CollisionObjective.h"
#include "..\include\CloseToPointObjective.h"
#include "..\include\Utilities.h"
#include <fstream>
#include <chrono>
#include <igl/readOFF.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/unproject.h"



//always returns 7nth right link
RigidBody * GetCurrentActiveLink(Robot* robot)
{
	for (int i = 0; i < robot->getRigidBodyCount(); i++)
		if (robot->getRigidBody(i)->name.compare("link_7_r") == 0)
			return robot->getRigidBody(i);
	return nullptr;
}

double Rad(double x) // the functor we want to apply
{
	return RAD(x);
}

Vector2f RSApp::GetCurrentMousePosition()
{
	return Vector2f(viewer.current_mouse_x, viewer.core.viewport(3) - viewer.current_mouse_y);
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
	//cout << "new point:(" << cart.x() << ";" << cart.y() << ";" << cart.z() << ")." << endl;

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
		m_cartLocation = kSolver->CalcForwardKinematics(robot->GetQ());
		for (auto&& obstacle : m_obstacles)
		{
			if (obstacle->CollidesRobot(m_cartLocation))
				viewer.data_list[obstacle->IndexInViewer].set_colors(COLOR(255, 0, 0));
			else
				viewer.data_list[obstacle->IndexInViewer].set_colors(COLOR(0, 255, 0));
		}
		if (m_selectedSphere)
		{
			m_selectedSphere->MoveAlongAxis(m_draggingDirection);
		}
		
		if (m_state == SState::AdjustStart)
		{
			double res = 0;
			for (int i = 0; i<5; ++i)
				m_ikMinimizer->minimize(m_ikStartPositionObjective, m_startPosition, res);
			robot->MoveByJointsR(m_startPosition, false);
		}

		if (m_state == SState::AdjustEnd)
		{
			double res = 0;
			for (int i = 0; i<5; ++i)
				m_ikMinimizer->minimize(m_ikEndPositionObjective, m_endPosition, res);
			robot->MoveByJointsR(m_endPosition, false);
		}		

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
				robot->MoveByJoints(newJoints,true);
				return true;
			}
			case GLFW_KEY_R:
			{
				DynamicArray<double> newJoints;
				newJoints.resize(14, 0);
				robot->MoveByJoints(newJoints,true);
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
			m_startDragger->SetVisibility(false);
			m_endDragger->SetVisibility(false);
			//m_state == SState::Simulation;
			switch (key)
			{
			case GLFW_KEY_LEFT:
				robot->MoveByJointsR(simulation->MoveToPrevAndGet(),false);
				return true;
			case GLFW_KEY_RIGHT:
				robot->MoveByJointsR(simulation->MoveToNextAndGet(), false);
				return true;
			case GLFW_KEY_S:
				m_state = SState::AdjustStart;
				m_startDragger->SetVisibility(true);
				return true;
			case GLFW_KEY_E:
				m_state = SState::AdjustEnd;
				m_endDragger->SetVisibility(true);
				return true;
			/*case GLFW_KEY_S:
				ikSolver->solve(100, false, false);
				return true;*/
			case GLFW_KEY_P:
				robot->PrintJointsValues();
				return true;
			case GLFW_KEY_A:
			{
				VectorXd v(7);
				v << 50 ,-50,0,-50,0,0,0;
				robot->MoveByJointsR(v,true);
				robot->PrintJointsValues();
				return true;
			}
			case GLFW_KEY_R:
			{
				DynamicArray<double> newJoints;
				newJoints.resize(14, 0);
				robot->MoveByJoints(newJoints,true);
				return true;
			}
			default:
				return false;
			}
			return false;
		};
		viewer.callback_key_up =
			[&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
		{
			m_state = SState::Simulation;
			m_startDragger->SetVisibility(false);
			m_endDragger->SetVisibility(false);
			return true;
		};
	}
	
	viewer.callback_mouse_move =
		[&](igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y)->bool
	{
		if (m_selectedSphere == nullptr)
		{
			VerifyHighlightAll();
		}
			
		else
		{
			
			Vector2f currenPos = GetCurrentMousePosition();
			m_draggingDirection = currenPos.x() - m_dragStartPosition.x();
			
			//m_dragStartPosition = currenPos;
			/*if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view,
				viewer.core.proj, viewer.core.viewport, viewer.data_list[i].V, viewer.data_list[i].F, fid, bc))*/
			//Vector3f s;
			//igl::unproject(Vector3f(mouse_x, mouse_y, 0), viewer.core.view, viewer.core.proj, viewer.core.viewport,s);
			//P3D xx(s.x(), s.y(), s.z());
			//cout << xx.x() << " " << xx.y() << " " << xx.z() << endl ;
			////P3D xx(1,1,1);
			//pointToDraw.row(0) = xx;
			//viewer.data().set_points(pointToDraw, Eigen::RowVector3d(220 / 255.0, 220 / 255.0, 220 / 255.0));
			//Vec3 win_s(pos(0), pos(1), 0);
			//Vec3 win_d(pos(0), pos(1), 1);
			//// Source, destination and direction in world
			//Vec3 d;
			//igl::unproject(win_s, model, proj, viewport, s);
			//igl::unproject(win_d, model, proj, viewport, d);
			//dir = d - s;
		}

		return (m_selectedSphere != nullptr);
		
		/*if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view,
			viewer.core.proj, viewer.core.viewport, viewer.data_list[sphereIndexInViewer].V, viewer.data_list[sphereIndexInViewer].F, fid, bc))
		{
			cout << "boom!\n";
			return true;
		}*/
	};

	viewer.callback_mouse_down =
		[&](igl::opengl::glfw::Viewer& viewer, int button, int modifier)->bool
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			m_selectedSphere = m_highlightedSphere;
			if (m_selectedSphere)
			{
				m_selectedSphere->SetSelected();
				m_dragStartPosition = GetCurrentMousePosition();
			}
			
		}
		return false;
	};

	viewer.callback_mouse_up =
		[&](igl::opengl::glfw::Viewer& viewer, int button, int modifier)->bool
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			if (m_selectedSphere)
				m_selectedSphere->ClearSelection();
			m_selectedSphere = nullptr;
			m_draggingDirection = 0;
		}
		VerifyHighlightAll();
		return false;
	};
}

void RSApp::VerifyHighlightAll()
{
	m_highlightedSphere = nullptr;
	for (auto&& ds : DraggableSphere::AllDS)
	{
		VerifyHighlight(ds);
	}
}

bool RSApp::VerifyHighlight(DraggableSphere* ds)
{
	int fid;
	Eigen::Vector3f bc;

	double x = viewer.current_mouse_x;
	double y = viewer.core.viewport(3) - viewer.current_mouse_y;

	ds->ClearHighlight();
	for (int i = ds->IndexInViewer+1; i <= ds->LastIndexInViewer; ++i)
	{
		if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view,
			viewer.core.proj, viewer.core.viewport, viewer.data_list[i].V, viewer.data_list[i].F, fid, bc))
		{
			ds->SetHighlighted(i);
			m_highlightedSphere = ds;
			return true;
		}
	}
	return false;
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

void RSApp::RecreateSimulation(std::vector<int> weights, MinimizerType mt)
{
	VectorXd v1(7); v1 << 0, 0, 0, 0, 0, 0, 0;
	v1 = v1.unaryExpr(&Rad);
	VectorXd v2(7); v2 << 50, -50, 0, -50, 0, 0, 0;
	v2 = v2.unaryExpr(&Rad);
	if (simulation != nullptr)
		delete simulation;
	//simulation = new BasicSimulation(v1, v2, PathSize);
	

	simulation = new AdvancedSimulation(m_startPosition, m_endPosition, PathSize, weights,(int)mt,robot,m_finalCart,m_onlyFinalCart, m_obstacles);
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

	P3D startingPoint = kSolver->CalcForwardKinematics(robot->GetQ());
	m_startDragger = new DraggableSphere(startingPoint, 0.04, &viewer);
	m_startDragger->SetVisibility(false);
	P3D endingPoint(0.59,0.69,0.43);
	m_endDragger = new DraggableSphere(endingPoint, 0.04, &viewer);
	m_endDragger->SetVisibility(false);
	m_ikMinimizer = new BFGSFunctionMinimizer(1);
	m_ikStartPositionObjective = new CloseToPointObjective(7, 1, m_startDragger->Location, robot);
	m_ikEndPositionObjective  = new CloseToPointObjective(7, 1, m_endDragger->Location, robot);
	m_ikMinimizer = new BFGSFunctionMinimizer(1);

	m_startPosition = VectorXd(7);
	m_startPosition.setConstant(RAD(0));

	double res = 0;
	for (int i = 0; i<5; ++i)
		m_ikMinimizer->minimize(m_ikStartPositionObjective, m_startPosition, res);

	m_endPosition = VectorXd(7);
	m_endPosition.setConstant(RAD(0));

	res = 0;
	for (int i = 0; i<5; ++i)
		m_ikMinimizer->minimize(m_ikEndPositionObjective, m_endPosition, res);

	
	CreateIKSolver();
	std::vector<int> weights = { w_first,w_last,w_equal,w_close2point,w_collision};
	RecreateSimulation(weights,MinimizerType::BFGS);
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



void RSApp::LoadMeshModelsIntoViewer(bool useSerializedModels)
{
	AddRobotModels(useSerializedModels);
	AddCollisionSpheres();
	
	
}

void RSApp::AddRobotModels(bool useSerializedModels)
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
}

void RSApp::AddCollisionSpheres()
{
	m_obstacles.push_back(new CollisionSphere(P3D(0.72, 0.60, 0.34), 0.07, &viewer));
	//m_obstacles.push_back(new CollisionSphere(P3D(0.3, 0.78, 0.33), 0.07, &viewer));
	
}

void RSApp::SetCollisionSpheresVisibility(bool val)
{
	for (auto&& cs : m_obstacles)
		cs->SetVisibility(val);
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
		if (m_state==SState::Simulation)
			robot->MoveByJointsR(simulation->MoveToIndAndGet(simulationPos - 1), false);
		ImGui::SameLine();
		if (ImGui::Button("-"))
		{
			simulationPos = simulation->DecreaseCurrentIndex()+1;
			robot->MoveByJointsR(simulation->GetCurrent(), false);
		}
		ImGui::SameLine();
		if (ImGui::Button("+"))
		{
			simulationPos = simulation->IncreaseCurrentIndex()+1;
			robot->MoveByJointsR(simulation->GetCurrent(), false);
		}
		ImGui::SameLine();
		ImGui::Text("Step");
		

		auto int2char = [&](int val) -> std::string
		{
			//quick and dirty
			if (val == -1)
				return "0";
			std::string out_string;
			std::stringstream ss;
			ss << pow(10, val);
			return ss.str();
		};
		auto calcWeight = [&](int val) -> int
		{
			//quick and dirty
			if (val == -1)
				return 0;
			else
				return pow(10, val);
		};
		ImGui::Text("Weights:");
		int max_w = 9;
		int min_w = -1;
		ImGui::SliderInt("First", &w_first, min_w,max_w,int2char(w_first).data());
		ImGui::SliderInt("Last", &w_last, min_w,max_w,int2char(w_last).data());
		ImGui::SliderInt("Equal", &w_equal, min_w,max_w,int2char(w_equal).data());
		ImGui::SliderInt("Close2Point", &w_close2point, min_w,max_w,int2char(w_close2point).data());
		ImGui::SliderInt("Collision", &w_collision, min_w,max_w,int2char(w_collision).data());
		ImGui::SameLine();
		ImGui::Checkbox("B", &CollisionObjective::UseBaseAddGradient);
		ImGui::NewLine();
		
		static MinimizerType minimizerType = simulation->MinimizerType;
		const char* cc = minimizerType == MinimizerType::GD ? "GradDesc" : "BFGS";
		ImGui::SliderInt("Minimizer", &((int)minimizerType), 0, 1, cc);

		std::vector<int> weights = { calcWeight(w_first),calcWeight(w_last),calcWeight(w_equal),calcWeight(w_close2point),calcWeight(w_collision) };
		
		if (ImGui::Checkbox("Only final position", &m_onlyFinalCart))
		{
			if (!m_onlyFinalCart)
			{
				w_first = 2; w_last = 2; w_equal = 0; w_collision = 0;
				w_close2point = -1;
			}
			else
			{
				w_first = -1; w_last = -1; w_equal = -1; w_collision = -1;
				w_close2point = 0;
			}
		}

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

		static bool showPathOutput = false;
		ImGui::Checkbox("Show path", &showPathOutput);
		
		static bool showCollisionSpheres = true;
		if (ImGui::Checkbox("Show collision spheres", &showCollisionSpheres))
		{
			SetCollisionSpheresVisibility(showCollisionSpheres);
		}

		static bool autostep = false;
		if (ImGui::Button("Rebuild simulation", ImVec2(-1, 0)))
		{
			RecreateSimulation(weights, minimizerType);
			//autostep = true;
		}
		

		if (ImGui::Button("Debug button", ImVec2(-1, 0)))
		{
			static bool b = true;
			b = !b;
			for (auto&& ds : DraggableSphere::AllDS)
			{
				ds->ShowDraggers(b);
			}
			
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
		ImGui::Text("x:%5.2f y:%5.2f z:%5.2f", m_cartLocation.x(), m_cartLocation.y(), m_cartLocation.z());
		ImGui::End();

		if (showPathOutput)
		{
			ImGui::SetWindowFontScale(1.2);
			ImGui::Begin("Path");
			ImGui::SetWindowPos(ImVec2(830, 350), ImGuiCond_Once);
			ImGui::SetWindowFontScale(1.2);
			int n = 7;
			for (size_t i = 0; i < simulation->path.size() / n; i++)
			{
				ImGui::Text("%5.2f  %5.2f  %5.2f  %5.2f  %5.2f  %5.2f  %5.2f  ", DEG(simulation->path(i*n+0)), DEG(simulation->path(i * n + 1)), DEG(simulation->path(i * n + 2)), DEG(simulation->path(i * n + 3)), DEG(simulation->path(i * n + 4)), DEG(simulation->path(i * n + 5)), DEG(simulation->path(i * n + 6)));
			}
			ImGui::End();
		}
		
		
		ImGui::SetNextWindowPos(ImVec2(0, 480), ImGuiCond_Once);
		ImGui::SetNextWindowSize(ImVec2(0.0, 0.0));
		ImGui::SetWindowFontScale(1.2);
		ImGui::Begin("Optimization");
		ImGui::SetWindowFontScale(1.2);
		if (ImGui::Button("Make step", ImVec2(-1, 0)))
		{
			simulation->MakeStep();
		}
		
		if (ImGui::Button("Auto step", ImVec2(-1, 0)))
		{
			autostep = !autostep;
		}
		/*static int stopAfter = 1000;
		ImGui::InputInt("Stop after", &stopAfter);
		if (simulation->IterationNum == stopAfter)
			autostep = false;*/
		if (autostep)
			simulation->MakeStep();
		ImGui::Text("Iteration # %d", simulation->IterationNum);
		ImGui::Text("Value %E", simulation->ComputeValueAll());
		ImGui::Text("Grad  %E", simulation->ComputeGradientAll());
		
		/*CollisionSphere cs = m_obstacles.front();
		CollisionObjective co(7, (double)pow(10, w4), cs.Location, cs.Radius, robot);
		CloseToPointObjective ctp(7, 1.0, cs.Location, robot);		
		co.testGradientWithFD(simulation->GetCurrent().unaryExpr(&Rad));*/
		//ctp.testGradientWithFD(j3);

		ImGui::Text("Curr. Value %E", simulation->ComputeValueCurrent());		
		ImGui::Text("Curr. Grad  %E", simulation->ComputeGradientCurrent());

		static bool testGradient = false;
		ImGui::Checkbox("Test gradient", &testGradient);
		if (testGradient)
		{
			static bool contTestAll = false;
			ImGui::Checkbox("Continuously Test All", &contTestAll);
			if (contTestAll)
				simulation->testGradient(0);
			if (ImGui::Button("Test1", ImVec2(-1, 0)))
				simulation->testGradient(1);
			if (ImGui::Button("Test2", ImVec2(-1, 0)))
				simulation->testGradient(2);
			if (ImGui::Button("Test3", ImVec2(-1, 0)))
				simulation->testGradient(3);
			if (ImGui::Button("Test4", ImVec2(-1, 0)))
				simulation->testGradient(4);
			if (ImGui::Button("Test5", ImVec2(-1, 0)))
				simulation->testGradient(5);

		}
			

		ImGui::Text("Iterations:  %d", simulation->GetLastNumOfIterations());
		ImGui::End();


		//}
	};

}
