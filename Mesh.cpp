#include "Mesh.h"

std::vector<int> Mesh::getControlPointsIndex()
{
	std::vector<int> indexes;
	indexes.reserve(controlPoints.size());
	for (const auto& cp : controlPoints)
		indexes.push_back(cp.vertexIndexInMesh);
	return indexes;
}

Eigen::MatrixXd Mesh::getControlPointsWantedPosition()
{
	Eigen::MatrixXd cpPosition = Eigen::MatrixXd::Zero(controlPoints.size(), 3);
	for (int i = 0; i < cpPosition.rows(); i++)
		cpPosition.row(i) = controlPoints[i].wantedVertexPosition;
	return cpPosition;
}

bool Mesh::isAControlPoint(int vertexIndex)
{
	for (const auto& cp : controlPoints)
		if (cp.vertexIndexInMesh == vertexIndex)
			return true;
	return false;
}

ControlPoint* Mesh::getControlPoint(int vertexIndex)
{
	for (auto& cp : controlPoints)
		if (cp.vertexIndexInMesh == vertexIndex)
			return &cp;
	return nullptr;
}


void Mesh::addControlPoint(int vertexIndex)
{
	if (vertexIndex < 0 || vertexIndex >= V.rows() || isAControlPoint(vertexIndex))
		return;
	controlPoints.push_back(ControlPoint(vertexIndex, V.row(vertexIndex)));
}

void Mesh::addControlPoint(int vertexIndex, Eigen::RowVector3d position)
{
	if (vertexIndex < 0 || vertexIndex >= V.rows())
		return;

	// If already exists, change its control position to position
	for (auto& cp : controlPoints)
		if (cp.vertexIndexInMesh == vertexIndex)
		{
			cp.wantedVertexPosition = position;
			return;
		}

	// Else just add it
	controlPoints.push_back(ControlPoint(vertexIndex, V.row(vertexIndex)));
}

void Mesh::removeControlPoint(int vertexIndex)
{
	if (vertexIndex < 0 || vertexIndex >= V.rows())
		return;

	int index = -1;
	for (int i = 0; i < controlPoints.size(); i++)
		if (controlPoints[i].vertexIndexInMesh == vertexIndex)
		{
			index = i;
			break;
		}

	if (index != -1)
		controlPoints.erase(std::next(controlPoints.begin(), index));
}

void Mesh::printControlPoints()
{
	std::cout << "Control Points:\n";
	for (auto& cp : controlPoints)
	{
		std::cout << cp.vertexIndexInMesh << " : " << V.row(cp.vertexIndexInMesh) << " -> " << cp.wantedVertexPosition << "\n";
	}
	std::cout << std::endl;
}