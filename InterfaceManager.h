#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"

class InterfaceManager
{
private:
	bool				mouseIsPressed = false;
	bool				moveSelectionMode = false;
	std::vector<int>	selection;
	
	Eigen::Vector3d		firstMoveDirection = Eigen::Vector3d(1, 0, 0);
	Eigen::Vector3d		secondMoveDirection = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d		lastProjectedPoint = Eigen::Vector3d(0, 0, 0);

private:
	std::vector<ControlPoint*> getSelectedControlPoints(Mesh& mesh);
	std::vector<int> getSelectedControlPointsIndex(const Mesh& mesh, bool invert = false);	// if invert = false, returns the control points that are in the selection ; if = true, returns the selected points that are not control points
	std::vector<int> getNonSelectedControlPointsIndex(const Mesh& mesh);						// returns not selected control points
	void displaySelectedPoints(igl::opengl::glfw::Viewer& viewer, Mesh& mesh);
	void projectOnMoveDirection(igl::opengl::glfw::Viewer& viewer, Eigen::Vector3d& projectionReceiver);

public:
	void onMousePressed(igl::opengl::glfw::Viewer& viewer, Mesh& mesh, bool isShiftPressed);
	void onMouseReleased();
	bool onMouseMoved(igl::opengl::glfw::Viewer& viewer, Mesh& mesh);
	void onKeyPressed(igl::opengl::glfw::Viewer& viewer, Mesh& mesh, unsigned char key, bool isShiftPressed);
};