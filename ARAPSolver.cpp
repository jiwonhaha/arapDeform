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
    /*std::cout << "neighbors = { ";
    for (std::list<int> neighbor : neighbors) {
        std::cout << "{ ";
        for (int n : neighbor) {
            std::cout << n << ", ";
        }
        std::cout << "}; \n";
    }
    std::cout << "}; \n";*/
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
        Vector3d v1 = V.row(F(i, 1)) - V.row(F(i, 0));
        Vector3d v2 = V.row(F(i, 2)) - V.row(F(i, 1));
        Vector3d v3 = V.row(F(i, 0)) - V.row(F(i, 2));

        // Compute the angles
        double a201 = acos(v1.dot(-v3) / (v1.norm() * v3.norm()));
        double a012 = acos(-v1.dot(v2) / (v1.norm() * v2.norm()));
        double a120 = acos(-v2.dot(v3) / (v2.norm() * v3.norm())); //seems ok

        // Add the angles
        // J'ai dû mettre abs pour que ca marche... réflechir à pourquoi

        /*std::cout << "a120" << std::endl;
        std::cout << a120 * 180 / 3.14 << std::endl;
        std::cout << "a201" << std::endl;
        std::cout << a201 * 180 / 3.14 << std::endl;
        std::cout << "a012" << std::endl;
        std::cout << a012 * 180 / 3.14 << std::endl;*/

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
    //std::cout << weights << std::endl;
}

void compute_laplacian_matrix(std::list<std::pair<int, Vector3d>> C) {
    L = weights;

    // Add constraints
    for (std::list<std::pair<int, Vector3d>>::iterator it = C.begin(); it != C.end(); ++it) {
        int index = (*it).first;
        L.row(index) = VectorXd::Zero(L.cols());
        L(index, index) = 1;
    }

    // Add diagonal value
    for (int i = 0; i < L.rows(); i++) {
        L(i, i) = -L.row(i).sum();
    }

    // DEBUG
    //std::cout << L << std::endl;
}

MatrixXd compute_covariance_matrix(MatrixXd V, MatrixXd new_V, int index) {
    MatrixXd Si(V.cols(), V.cols());

    // Retrieve neighbors of v
    std::list<int> neighbors_v = neighbors[index];   

    /*std::cout << "neighbors = { ";
    for (int n : neighbors_v) {
            std::cout << n << ", ";
    }
    std::cout << "}; \n";*/

    MatrixXd P_init = MatrixXd::Zero(V.cols(), neighbors_v.size());
    MatrixXd P_new = MatrixXd::Zero(V.cols(), neighbors_v.size());
    DiagonalMatrix<double, Eigen::Dynamic> D(neighbors_v.size());

    // Vertex of initial and final mesh
    Vector3d v_init = V.row(index);
    Vector3d v_new = new_V.row(index);

    // For each neighbor compute edges (in initial and result meshes)
    int k = 0;
    for (std::list<int>::iterator it = neighbors_v.begin(); it != neighbors_v.end(); ++it, ++k) {
        // Initial mesh
        Vector3d neighbor_init = V.row(*it);
        Vector3d edge_init = v_init - neighbor_init;
        P_init.col(k) = edge_init;

        /*std::cout << "edge_init" << std::endl;
        std::cout << edge_init << std::endl;*/

        // Updated mesh
        Vector3d neighbor_new = new_V.row(*it);
        Vector3d edge_new = v_new - neighbor_new;
        P_new.col(k) = edge_new;

        /*std::cout << "edge_new" << std::endl;
        std::cout << edge_new << std::endl;*/

        // Diagonal mesh
        D.diagonal()[k] = weights(index, *it);
    }



    Si = P_init * D * P_new.transpose();

    // DEBUG
    /*std::cout << "D" << std::endl;
    std::cout << D.diagonal() << std::endl;

    std::cout << "P_init" << std::endl;
    std::cout << P_init << std::endl;

    std::cout << "P_new" << std::endl;
    std::cout << P_new << std::endl;*/

    /*std::cout << "Si" << std::endl;
    std::cout << Si << std::endl;*/

    return Si;
}

/* Apply arap algo for one iteration
 * V : Matrix of initial points (previous frame)
 * C : Constraints vertices 
 *
 * Out : Update V 
 */
void arap(const MatrixXd &V, const MatrixXi &F, const std::list<std::pair<int, Vector3d>> C, MatrixXd new_V) {
    // Initialize new V vertices
    // Do it here or before ?

    // Center meshes
    MatrixXd V_centered = V.rowwise() - V.colwise().mean();
    MatrixXd new_V_centered = new_V.rowwise() - new_V.colwise().mean();

    /*std::cout << V.colwise().mean() << std::endl;
    std::cout << new_V.colwise().mean() << std::endl;*/

    // Compute weights
    compute_edges_weight(V, F);

    // Precompute Laplacian-Beltrami matrix
    compute_laplacian_matrix(C);

    // ITERATE

    // Find optimal Ri for each cell
    std::vector<MatrixXd> R(V.rows()); // Matrix of local rotations
    for (int i = 0; i < V.rows(); i++) {
        MatrixXd Si = compute_covariance_matrix(V_centered, new_V_centered, i);

        JacobiSVD<MatrixXd> svd(Si, ComputeThinU | ComputeThinV);

        DiagonalMatrix<double, 3> D(1, 1, (svd.matrixV() * svd.matrixU().transpose()).determinant());
        MatrixXd Ri = svd.matrixV() * D * svd.matrixU().transpose();

        // Store Ri
        R[i] = Ri;

        // DEBUG
        std::cout << Ri << std::endl;
    }
    

    // Find optimal p'
}