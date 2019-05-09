#pragma once

//#include "ObjectiveFunction.h"
//#include "Robot.h"
//#include "SingleArmKinematicsSolver.h"
//
#include "..\include\Utilities.h"
#include "..\include\P3D.h"

#include <igl/opengl/glfw/Viewer.h>

using namespace igl::opengl::glfw;

class DraggableSphere
{
private:
	MatrixXd m_transformedV;
	MatrixXd m_transformedDragX;

	RowVector3d m_SphereColor = COLOR(0, 255, 0);
	RowVector3d m_DraggerColor = COLOR(255, 255, 0);
	RowVector3d m_DraggerHighlightedColor = COLOR(255,255, 255);
	RowVector3d m_DraggerSelectedColor = COLOR(0,0,255);

	double m_draggableStep = 0.15;

	int DraggerXIndex() { return IndexInViewer + 1; }
	int DraggerYIndex() { return IndexInViewer + 2; }
	int DraggerZIndex() { return IndexInViewer + 3; }
	

	Viewer* m_viewer;

	Eigen::MatrixXd V;
	Eigen::MatrixXi F;

	Eigen::Matrix4d GetTranslationMatrix();
	Eigen::Matrix4d GetScaleMatrix();
	Eigen::Matrix4d GetDraggerTranslationMatrix(int axis);
	Eigen::Matrix4d GetDraggerScaleMatrix();

	void RecalculatePositions();

	double m_moveAlomgAxisStep = 0.0065;
	

public:
	P3D Location;
	double Radius;
	int IndexInViewer;
	int LastIndexInViewer;

	DraggableSphere(P3D loc, double radius, int ind, Viewer* viewer);

	void SetHighlighted(int index);
	void SetSelected();
	
	void ClearHighlight();
	void ClearSelection();

	int m_highlightedIndex = 0;
	int m_selectedIndex = 0;

	void MoveAlongAxis(int direction);
	void ShowDraggers(bool val);

};


class CollisionSphere : public DraggableSphere
{
public:
	CollisionSphere(P3D loc, double rad, int ind, Viewer* viewer);
	bool CollidesRobot(P3D eePosition);
};

//using namespace Eigen;
