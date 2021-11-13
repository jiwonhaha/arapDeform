#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"

class MouseManager
{
private:
	bool mouseIsPressed = false;

public:
	bool onMousePressed(igl::opengl::glfw::Viewer& viewer, Mesh& mesh);
	bool onMouseReleased();
	bool onMouseMoved();
};