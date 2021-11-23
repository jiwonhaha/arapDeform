#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"

class InterfaceManager
{
private:
	bool				mouseIsPressed = false;
	bool				moveSelectionMode = false;
	std::vector<int>	selection;
	
	Eigen::RowVector3d	firstMoveDirection = Eigen::RowVector3d(1, 0, 0);
	Eigen::RowVector3d	secondMoveDirection = Eigen::RowVector3d(0, 0, 0);

private:
	std::vector<int> getSelectedControlPoints(const Mesh& mesh, bool invert = false);	// if invert = false, returns the control points that are in the selection ; if = true, returns the selected points that are not control points
	std::vector<int> getNonSelectedControlPoints(const Mesh& mesh);						// returns not selected control points
	void displaySelectedPoints(igl::opengl::glfw::Viewer& viewer, Mesh& mesh);

public:
	void onMousePressed(igl::opengl::glfw::Viewer& viewer, Mesh& mesh, bool isShiftPressed);
	void onMouseReleased();
	bool onMouseMoved();
	void onKeyPressed(igl::opengl::glfw::Viewer& viewer, Mesh& mesh, unsigned char key, bool isShiftPressed);
};