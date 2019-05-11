#include "..\include\DraggableSphere.h"
#include "..\include\V3D.h"
#include <igl/readOFF.h>
#include <stdio.h>

using namespace std;

vector<DraggableSphere*> DraggableSphere::AllDS;

Eigen::Matrix4d DraggableSphere::GetTranslationMatrix()
{
	Eigen::Matrix4d tm;

	tm <<
		1, 0, 0, Location.x(),
		0, 1, 0, Location.y(),
		0, 0, 1, Location.z(),
		0, 0, 0, 1;

	return tm;
}

Eigen::Matrix4d DraggableSphere::GetScaleMatrix()
{
	Eigen::Matrix4d sm;

	sm <<
		Radius, 0, 0, 0,
		0, Radius, 0, 0,
		0, 0, Radius, 0,
		0, 0, 0, 1;

	return sm;
}

Eigen::Matrix4d DraggableSphere::GetDraggerTranslationMatrix(int axis)
{
	Eigen::Matrix4d tm;

	if (axis == 0)
	{
		tm <<
			1, 0, 0, m_draggableStep,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}
	else if (axis == 1)
	{
		tm <<
			1, 0, 0, 0,
			0, 1, 0, m_draggableStep,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}
	else if (axis == 2)
	{
		tm <<
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, m_draggableStep,
			0, 0, 0, 1;
	}
	else
		throw ("Not supported");


	return tm;
}

Eigen::Matrix4d DraggableSphere::GetDraggerScaleMatrix()
{
	Eigen::Matrix4d sm;
	double small_r = Radius / 3;
	sm <<
		small_r, 0, 0, 0,
		0, small_r, 0, 0,
		0, 0, small_r, 0,
		0, 0, 0, 1;

	return sm;
}

void DraggableSphere::RecalculatePositions()
{
	Eigen::Matrix4d mat = GetTranslationMatrix()*GetScaleMatrix();
	m_transformedV = TransformP(V, mat);
	m_viewer->data_list[IndexInViewer].set_mesh(m_transformedV, F);

	Eigen::Matrix4d tm = GetTranslationMatrix()*GetDraggerTranslationMatrix(0)*GetDraggerScaleMatrix();
	m_transformedDragX = TransformP(MatrixXd(V), tm);
	m_viewer->data_list[DraggerXIndex()].set_mesh(m_transformedDragX, F);

	tm = GetTranslationMatrix()*GetDraggerTranslationMatrix(1)*GetDraggerScaleMatrix();
	Eigen::MatrixXd transformedDragY = TransformP(MatrixXd(V), tm);
	m_viewer->data_list[DraggerYIndex()].set_mesh(transformedDragY, F);

	tm = GetTranslationMatrix()*GetDraggerTranslationMatrix(2)*GetDraggerScaleMatrix();
	Eigen::MatrixXd transformedDragZ = TransformP(V, tm);
	m_viewer->data_list[DraggerZIndex()].set_mesh(transformedDragZ, F);
}

void DraggableSphere::MoveAlongAxis(int direction)
{
	double step = direction > 0 ? m_moveAlomgAxisStep : direction < 0 ? -m_moveAlomgAxisStep : 0;
	if (m_selectedIndex == DraggerXIndex())
	{
		Location.x() += step;
	}
	if (m_selectedIndex == DraggerYIndex())
	{
		Location.y() += step;
	}
	if (m_selectedIndex == DraggerZIndex())
	{
		Location.z() += step;
	}
	RecalculatePositions();
}

void DraggableSphere::ShowDraggers(bool val)
{
	m_viewer->data_list[DraggerXIndex()].show_faces = val;
	m_viewer->data_list[DraggerYIndex()].show_faces = val;
	m_viewer->data_list[DraggerZIndex()].show_faces = val;
}

void DraggableSphere::SetVisibility(bool val)
{
	m_viewer->data_list[IndexInViewer].show_faces = val;
	ShowDraggers(val);
}

DraggableSphere::DraggableSphere(P3D loc, double radius, Viewer* viewer) :
	Location(loc),
	Radius(radius),
	IndexInViewer(viewer->data_list.size()),
	m_viewer(viewer),
	m_draggableStep(radius*2),
	m_SphereColor (COLOR(0, 0, 255))
{
	std::string sphereFile = "../RoboticSimulator/data/models/sphere.off";
	igl::readOFF(sphereFile, V, F);

	for (int i = 0; i <= 3; ++i)
		m_viewer->append_mesh();

	RecalculatePositions();

	for (int i = 0; i <= 3; ++i)
	{
		m_viewer->data_list[IndexInViewer + i].show_lines = false;
		if (i == 0)
			m_viewer->data_list[IndexInViewer + i].set_colors(m_SphereColor);
		else
			m_viewer->data_list[IndexInViewer + i].set_colors(m_DraggerColor);
	}

	LastIndexInViewer = DraggerZIndex();
	AllDS.push_back(this);
	
}

DraggableSphere::~DraggableSphere()
{
	auto it = std::find(AllDS.begin(), AllDS.end(), this);
	if (it != AllDS.end())
	{
		AllDS.erase(it);
	}
}

void DraggableSphere::SetHighlighted(int index)
{
	m_viewer->data_list[index].set_colors(m_DraggerHighlightedColor);
	m_highlightedIndex = index;
}

void DraggableSphere::SetSelected()
{
	m_viewer->data_list[m_highlightedIndex].set_colors(m_DraggerSelectedColor);
	m_selectedIndex = m_highlightedIndex;
}


void DraggableSphere::ClearHighlight()
{
	if (m_highlightedIndex != 0)
	{
		RowVector3d colorBack = (m_highlightedIndex == IndexInViewer) ? m_SphereColor : m_DraggerColor;
		m_viewer->data_list[m_highlightedIndex].set_colors(colorBack);
		m_highlightedIndex = 0;
	}
}

void DraggableSphere::ClearSelection()
{
	m_viewer->data_list[m_selectedIndex].set_colors(m_SphereColor);
	m_selectedIndex = 0;
}

CollisionSphere::CollisionSphere(P3D loc, double rad, Viewer* viewer) :
	DraggableSphere(loc, rad, viewer)	
{
	m_SphereColor = COLOR(0, 255, 0);
}

bool CollisionSphere::CollidesRobot(P3D eePosition)
{
	double d = (eePosition - Location).norm();
	return d < Radius;
}