#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include "ControlPoint.h"

struct Mesh
{
private:
	std::vector<ControlPoint> controlPoints;

public:
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;

	Eigen::MatrixXd getVerticesFromIndex(const std::vector<int>& indexes) const;

	const std::vector<ControlPoint>& getControlPoints() const { return controlPoints; }
	std::vector<int> getControlPointsIndex() const;
	Eigen::MatrixXd getControlPointsWantedPosition() const;
	bool isAControlPoint(int vertexIndex) const;
	ControlPoint* getControlPoint(int vertexIndex);		// be careful : changing controlPoints vector may change its memory location => would invalidate the pointer
	
	void addControlPoint(int vertexIndex);
	void addControlPoint(int vertexIndex, Eigen::RowVector3d position);
	void removeControlPoint(int vertexIndex);

	void printControlPoints() const;
};