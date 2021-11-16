#pragma once
#include <igl/opengl/glfw/Viewer.h>
using namespace Eigen;

void find_neighbors(const MatrixXd V, const MatrixXi F);
void compute_edges_weight(const MatrixXd& V, const MatrixXi& F);
void compute_laplacian_matrix(std::list<std::pair<int, Vector3d>> C);
void arap(const MatrixXd& V, const MatrixXi& F, const std::list<std::pair<int, Vector3d>> C, MatrixXd new_V);
