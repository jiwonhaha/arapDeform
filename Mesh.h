#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include "ControlPoint.h"

struct Mesh
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	std::vector<ControlPoint> controlPoints;
};