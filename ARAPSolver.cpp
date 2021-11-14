#include "ARAPSolver.h"


// /!\ Ne prend pas en compte les boundaries !!!

std::vector<std::list<int>> neighbors;
MatrixXd weights;
MatrixXd L;


/* Find one-ring neighbors of all the vertices in V.
 * V : Matrix of vertices
 * F : Matrix of faces
 *
 * Out : Vector of list of neigbors indices
 */
void find_neighbors(const MatrixXd V, const MatrixXi F) {
    std::vector<std::list<int>> myNeighbors(V.rows());

    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < F.cols(); j++) {
            myNeighbors[F(i, j)].push_back(F(i, (j + 1) % F.cols()));
            myNeighbors[F(i, j)].push_back(F(i, (j + 2) % F.cols()));
        }
    }

    for (int i = 0; i < V.rows(); i++) {
        myNeighbors[i].sort();
        myNeighbors[i].unique();
    }

    neighbors = myNeighbors;

    // DEBUG
    // Print out the vector 
    std::cout << "neighbors = { ";
    for (std::list<int> neighbor : neighbors) {
        std::cout << "{ ";
        for (int n : neighbor) {
            std::cout << n << ", ";
        }
        std::cout << "}; \n";
    }
    std::cout << "}; \n";
}

/* Compute weights wij
 * V : Matrix of vertices
 * F : Matrix of faces
 *
 * Out : Weights wij = 1/2 * (cot(aij) + cot(bij))
 */
void compute_edges_weight(const MatrixXd& V, const MatrixXi& F) {
    weights = MatrixXd::Zero(V.rows(), V.rows());

    for (int i = 0; i < F.rows(); i++) {
        // Compute edges vectors CCW
        Vector3d v1 = V.row(F(i, 0)) - V.row(F(i, 1));
        Vector3d v2 = V.row(F(i, 2)) - V.row(F(i, 1));
        Vector3d v3 = V.row(F(i, 0)) - V.row(F(i, 2));

        // Compute the angles
        double a201 = acos(v1.dot(-v3) / (v1.norm() * v3.norm()));
        double a012 = acos(-v1.dot(-v2) / (v1.norm() * v2.norm()));
        double a120 = acos(-v2.dot(v3) / (v2.norm() * v3.norm()));

        // Add the angles
        weights(F(i, 0), F(i, 1)) += cos(a120) / sin(a120);
        weights(F(i, 1), F(i, 0)) += cos(a120) / sin(a120);

        weights(F(i, 1), F(i, 2)) += cos(a201) / sin(a201);
        weights(F(i, 2), F(i, 1)) += cos(a201) / sin(a201);

        weights(F(i, 2), F(i, 0)) += cos(a012) / sin(a012);
        weights(F(i, 0), F(i, 2)) += cos(a012) / sin(a012);
    }

    // Divide all the weights by 2
    weights = (float) 1 / 2 * weights;

    // DEBUG
    std::cout << weights << std::endl;
}

void compute_laplacian_matrix() {
    L = weights;

    for (int i = 0; i < L.rows(); i++) {
        L(i, i) = -L.row(i).sum();
    }

    // DEBUG
    std::cout << L << std::endl;
}

/* Apply arap algo for one iteration
 * V : Matrix of initial points (previous frame)
 * C : Constraints vertices 
 *
 * Out : Update V 
 */
void arap(const MatrixXd &V, const MatrixXi F, const MatrixXd &C, MatrixXd new_V) {
    // Compute weights
    compute_edges_weight(V, F);

    // Precompute Laplacian-Beltrami matrix
    compute_laplacian_matrix();

    // Find optimal R

    // Find optimal p'
}