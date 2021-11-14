#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"

class InterfaceManager
{
private:
	bool				mouseIsPressed = false;
	bool				moveSelectionMode = false;
	std::vector<int>	selection;

private:
	void displaySelectedPoints(igl::opengl::glfw::Viewer& viewer, Mesh& mesh);

public:
	void onMousePressed(igl::opengl::glfw::Viewer& viewer, Mesh& mesh, bool isShiftPressed);
	void onMouseReleased();
	bool onMouseMoved();
	void onKeyPressed(igl::opengl::glfw::Viewer& viewer, Mesh& mesh, unsigned char key);
};