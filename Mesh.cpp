#include "Mesh.h"


Eigen::MatrixXd Mesh::getVerticesFromIndex(const std::vector<int>& indexes) const
{
	Eigen::MatrixXd verts = Eigen::MatrixXd::Zero(indexes.size(), 3);
	for (int i = 0; i < verts.rows(); i++)
		verts.row(i) = V.row(indexes[i]);
	return verts;
}

std::vector<ControlPoint*> Mesh::getControlPointsW()
{
	std::vector<ControlPoint*> cpp = std::vector<ControlPoint*>();
	cpp.reserve(controlPoints.size());
	for (auto& cp : controlPoints)
		cpp.push_back(&cp);
	return cpp;
}

std::vector<int> Mesh::getControlPointsIndex() const
{
	std::vector<int> indexes;
	indexes.reserve(controlPoints.size());
	for (const auto& cp : controlPoints)
		indexes.push_back(cp.vertexIndexInMesh);
	return indexes;
}

Eigen::MatrixXd Mesh::getControlPointsWantedPosition() const
{
	Eigen::MatrixXd cpPosition = Eigen::MatrixXd::Zero(controlPoints.size(), 3);
	for (int i = 0; i < cpPosition.rows(); i++)
		cpPosition.row(i) = controlPoints[i].wantedVertexPosition;
	return cpPosition;
}

bool Mesh::isAControlPoint(int vertexIndex) const
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
	controlPoints.push_back(ControlPoint(vertexIndex, position));
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

void Mesh::printControlPoints() const
{
	std::cout << "Control Points:\n";
	for (auto& cp : controlPoints)
	{
		std::cout << cp.vertexIndexInMesh << " : " << V.row(cp.vertexIndexInMesh) << " -> " << cp.wantedVertexPosition << "\n";
	}
	std::cout << std::endl;
}