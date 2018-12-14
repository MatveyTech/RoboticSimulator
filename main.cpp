#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <igl/opengl/gl.h>

#include "include/AbstractRBEngine.h"
#include "include/RBEngine.h"
#include "include/Robot.h"
#include "include/IK_Solver.h"

#include "include\RSApp.h"

#include <memory>


using namespace Eigen;
using namespace std;




void TransformPoints(Eigen::MatrixXd &V, Eigen::Matrix4d &tr)
{
	for (int i = 0; i < V.count() / 3; ++i)
	{
		Vector4d vec(V(i, 0), V(i, 1), V(i, 2), 1);
		Vector4d transformed = tr*vec;
		V(i, 0) = transformed(0);
		V(i, 1) = transformed(1);
		V(i, 2) = transformed(2);
	}
}

void DrawFirst2Joints()
{
	igl::opengl::glfw::Viewer viewer;
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	Eigen::MatrixXd V2;
	Eigen::MatrixXi F2;
	Eigen::MatrixXd V3;
	Eigen::MatrixXi F3;
	igl::readOBJ("C:/Users/matvey/Documents/CS2/Graphics project/SCP/SCP/data/rbs/yumi/meshes/simplified/body.obj", V, F);
	igl::readOBJ("C:/Users/matvey/Documents/CS2/Graphics project/SCP/SCP/data/rbs/yumi/meshes/simplified/link_1_r.obj", V2, F2);
	igl::readOBJ("C:/Users/matvey/Documents/CS2/Graphics project/SCP/SCP/data/rbs/yumi/meshes/simplified/link_2_r.obj", V3, F3);
	
	char *base_transformation = "0.70711 -0.70711 0.0 0.0 -0.076778 -0.453103 -0.084942";
	char *link_1_r = "0.70711 -0.70711 0.0 0.0 0.0 0.0 0.0";
	char *link_2_r = "0.70711 -0.70711 0.0 0.0 -0.185025 -0.515599 -0.130436";

	Vector3d p;
	
	double q1, q2, q3, q4;
	sscanf(base_transformation, "%lf %lf %lf %lf %lf %lf %lf",
		&q1, &q2, &q3, &q4, &p(0), &p(1), &p(2));
	Quaterniond d(q1, q2, q3, q4);
	d.normalize();
	Matrix3d rot = d.toRotationMatrix();
	/*Matrix4d tr(Matrix4d::Identity());
	tr.block<3, 3>(0, 0) = rot;
	tr(3, 0) = p(0);
	tr(3, 1) = p(1);
	tr(3, 2) = p(2);
	TransformPoints(V, tr);
	viewer.data_list[0].set_mesh(V, F);*/
	

	sscanf(link_1_r, "%lf %lf %lf %lf %lf %lf %lf",
		&q1, &q2, &q3, &q4, &p(0), &p(1), &p(2));
	Quaterniond d2(q1, q2, q3, q4);
	d2.normalize();
	Matrix3d rot2 = d2.toRotationMatrix();
	/*Matrix4d tr2(Matrix4d::Identity());
	tr2.block<3, 3>(0, 0) = rot2;
	tr2(3, 0) = p(0);
	tr2(3, 1) = p(1);
	tr2(3, 2) = p(2);
	TransformPoints(V2, tr2);*/

	viewer.append_mesh();
	viewer.data_list[1].set_mesh(V2, F2);

	sscanf(link_2_r, "%lf %lf %lf %lf %lf %lf %lf",
		&q1, &q2, &q3, &q4, &p(0), &p(1), &p(2));
	Quaterniond d3(q1, q2, q3, q4);
	d3.normalize();
	Matrix3d rot3 = d3.toRotationMatrix();
	/*Matrix4d tr3(Matrix4d::Identity());
	tr3.block<3, 3>(0, 0) = rot3;
	tr3(3, 0) = p(0);
	tr3(3, 1) = p(1);
	tr3(3, 2) = p(2);
	TransformPoints(V3, tr3);*/

	viewer.append_mesh();
	viewer.data_list[2].set_mesh(V3, F3);
	
	RowVector3d color(211 / 255., 211 / 255., 211 / 255.);
	viewer.data_list[0].set_colors(color);
	viewer.data_list[1].set_colors(color);
	viewer.data_list[2].set_colors(color);
	
	const Eigen::MatrixXd P = (Eigen::MatrixXd(2, 3) <<
		0,0,0).finished();
	viewer.data().add_points(P, Eigen::RowVector3d(255, 0, 0));

	viewer.launch();
}

RigidBody * GetRigidBody(Robot* robot)
{
	for (int i = 0; i < robot->getRigidBodyCount(); i++)
		if (robot->getRigidBody(i)->name.compare("link_2_l") == 0)
			return robot->getRigidBody(i);
	return nullptr;
}

MatrixXd TransformP2(Eigen::MatrixXd &V, Eigen::Matrix4d &tr)
{
	MatrixXd th = V.rowwise().homogeneous().transpose();
	return (tr * th).transpose().leftCols(3);
}

int main(int argc, char *argv[])
{
	RSApp app;
	return 0;
	Eigen::MatrixXd V3;
	Eigen::MatrixXi F3;
	igl::readOBJ("C:/Users/matvey/Documents/CS2/Graphics project/SCP/SCP/data/rbs/yumi/meshes/simplified/link_1_r.obj", V3, F3);
	igl::opengl::glfw::Viewer viewer2;
	viewer2.data_list[0].set_mesh(V3, F3);
	viewer2.launch();
	return 0;

	
	/*viewer2.callback_key_down =
		[&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
	{
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
	viewer2.launch();*/

	//DrawFirst2Joints();
	

	/*viewer.callback_key_down =
		[&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
	{
		/*if (key == GLFW_KEY_ENTER)
		{
			tr(0, 3) = tr(0, 3) + 0.15;
			TransformPoints(V, tr);
			viewer.data_list[0].set_mesh(V, F);
			return true;
	
		return false;
	};*/
	
	//2;
	//viewer.data_list[0].set_colors(Eigen::RowVector3d(1, 0, 0));
	//viewer.append_mesh();
	

}
