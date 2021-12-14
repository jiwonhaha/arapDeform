#include "ARAPSolver.h"


// /!\ Ne prend pas en compte les boundaries !!!

std::vector<std::list<int>> neighbors;
MatrixXd weights;
MatrixXd L;

float eps = 1e-10;
float tol = 1e-3;


/* Find one-ring neighbors of all the vertices in V.
 * V : Matrix of vertices
 * F : Matrix of faces
 *
 * Out : Vector of list of neigbors indices
 */
void find_neighbors(const MatrixXd& V, const MatrixXi& F) {
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
        double a120 = acos(-v2.dot(v3) / (v2.norm() * v3.norm())); 

        
        // DEBUG
        /*std::cout << "a120" << std::endl;
        std::cout << a120 * 180 / 3.14 << std::endl;
        std::cout << "a201" << std::endl;
        std::cout << a201 * 180 / 3.14 << std::endl;
        std::cout << "a012" << std::endl;
        std::cout << a012 * 180 / 3.14 << std::endl;*/

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

    // Put small values to 0
    for (int i = 0; i < weights.rows(); i++) {
        for (int j = 0; j < weights.cols(); j++) {
            if (weights(i, j) < eps) {
                weights(i, j) = 0;
            }
        }
    }

    // DEBUG
    /*std::cout << "weights" << std::endl;
    std::cout << weights << std::endl;*/
}

void compute_laplacian_matrix(const std::vector<ControlPoint>& C) {
    L = -weights;

    // Add diagonal value
    for (int i = 0; i < L.rows(); i++) {
        L(i, i) += -L.row(i).sum();
    }

    // Add constraints (Article)
    for (const ControlPoint& c : C) {
        int index = c.vertexIndexInMesh;
        L.row(index) = VectorXd::Zero(L.cols());
        L.col(index) = VectorXd::Zero(L.rows());
        L(index, index) = 1;
    }

    // DEBUG
    /*std::cout << 'L' << std::endl;
    std::cout << L << std::endl;*/
}

MatrixXd laplacian_init(const MatrixXd& V, const std::vector<ControlPoint>& C) {
    MatrixXd laplacian = -weights;

    // Add diagonal value
    for (int i = 0; i < laplacian.rows(); i++) {
        laplacian(i, i) += -laplacian.row(i).sum();
    }

    // Build A and B
    MatrixXd A = MatrixXd::Zero(V.rows(), V.rows() - C.size());
    MatrixXd B = MatrixXd::Zero(V.rows(), C.size());

    int a = 0;
    int b = 0;
    for (int i = 0; i < V.rows(); i++) {
        std::pair<bool, Vector3d> constraint = isConstraint(C, i);

        if (constraint.first) {
            B.col(b) = laplacian.col(i);
            b++;
        }
        else {
            A.col(a) = laplacian.col(i);
            a++;
        }
    }

    // Build A' * A
    MatrixXd left = A.transpose() * A;

    // Build A' * (-By + L * p)
    MatrixXd y = MatrixXd::Zero(C.size(), V.cols());
    for (int i = 0; i < C.size(); i++) {
        y.row(i) = C[i].wantedVertexPosition;
    }


    MatrixXd right = A.transpose() * (-B * y + laplacian * V);

    MatrixXd x = left.ldlt().solve(right);

    MatrixXd new_V = MatrixXd::Zero(V.rows(), V.cols());
    a = 0;
    b = 0;
    for (int i = 0; i < V.rows(); i++) {
        std::pair<bool, Vector3d> constraint = isConstraint(C, i);

        if (constraint.first) {
            new_V.row(i) = y.row(b);
            b++;
        }
        else {
            new_V.row(i) = x.row(a);
            a++;
        }
    }

    return new_V;
}

MatrixXd compute_covariance_matrix(const MatrixXd& V, const MatrixXd& new_V, const int& index) {
    MatrixXd Si(V.cols(), V.cols());

    // Retrieve neighbors of v
    std::list<int> neighbors_v = neighbors[index];   

    // DEBUG
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
    std::cout << D.diagonal() << std::endl;*/

    /*std::cout << "P_init" << std::endl;
    std::cout << P_init << std::endl;

    std::cout << "P_new" << std::endl;
    std::cout << P_new << std::endl;

    std::cout << "Si" << std::endl;
    std::cout << Si << std::endl;*/

    return Si;
}

std::pair<bool, Vector3d> isConstraint(const std::vector<ControlPoint>& C, int index) {

    for (const ControlPoint& c : C) {
        if (index == c.vertexIndexInMesh) {
            return std::pair<bool, Vector3d>(true, c.wantedVertexPosition);
        }
    }
    return std::pair<bool, Vector3d>(false, Vector3d(0,0,0));
}

MatrixXd compute_b(const MatrixXd& V, const std::vector<MatrixXd>& R, const std::vector<ControlPoint>& C) {

    MatrixXd b = MatrixXd::Zero(V.rows(), V.cols());

    for (int i = 0; i < V.rows(); i++) {
        VectorXd vi = V.row(i);

        // Retrieve neighbors of v
        std::list<int> neighbors_v = neighbors[i];

        // Rotation matrix of i-th vertex
        MatrixXd Ri = R[i];

        // For each neighbor add the corresponding term
        // Check if the point is a constraint
        std::pair<bool, Vector3d> constraint = isConstraint(C, i);

        if (constraint.first) {
            b.row(i) = (VectorXd) constraint.second;
        }
        else {
            // For each neighbor
            for (std::list<int>::iterator it = neighbors_v.begin(); it != neighbors_v.end(); ++it) {
                std::pair<bool, Vector3d> constraint_n = isConstraint(C, *it);

                VectorXd neighbor = V.row(*it);
                
                if (constraint_n.first) {
                    b.row(i) += (double)weights(i, *it) * constraint_n.second;
                }

                // Weight of the edge
                double wij = weights(i, *it);

                // Neighbor Rotation matrix
                MatrixXd Rj = R[*it];

                b.row(i) += (double)wij / 2 * (Ri + Rj) * (vi - neighbor);

            }
        }
    }

    return b;
}

float compute_energy(const MatrixXd& V, const MatrixXd& new_V, const std::vector<MatrixXd>& R) {
    float energy = 0;
    for (int i = 0; i < V.rows(); i++) {
        VectorXd vi = V.row(i);
        VectorXd vi_prime = new_V.row(i);

        // Retrieve neighbors of vi
        std::list<int> neighbors_v = neighbors[i];

        // Rotation matrix of i-th vertex
        MatrixXd Ri = R[i];

        for (std::list<int>::iterator it = neighbors_v.begin(); it != neighbors_v.end(); ++it) {

            VectorXd neighbor = V.row(*it);
            VectorXd neighbor_prime = new_V.row(*it);

            // Weight of the edge
            double wij = weights(i, *it);

            energy += (float)wij * pow(((vi_prime - neighbor_prime) - Ri * (vi - neighbor)).norm(), 2);

        }
    }
    return energy;
}


/* Apply arap algo for one iteration
 * V : Matrix of initial points (previous frame)
 * C : Constraints vertices 
 *
 * Out : Update V 
 */
MatrixXd arap(const MatrixXd &V, const MatrixXi &F, const std::vector<ControlPoint>& C, const int& kmax, const EInitialisationType& init) {

    MatrixXd previous_V = V;

   
    // User interaction
    if (init == EInitialisationType::e_LastFrame) {
        previous_V = V;
        std::cout << "Initiated with last frame" << std::endl;
    }
    // Laplacian initialization
    else if (init == EInitialisationType::e_Laplace) {
        previous_V = laplacian_init(V, C);
        std::cout << "Initiated with laplacian" << std::endl;
    }
    
    MatrixXd new_V = previous_V;

    /*float old_energy = 0;
    float new_energy = 0;

    // ITERATE
    int k = 0;
    do {

        // Find optimal Ri for each cell
        std::vector<MatrixXd> R(V.rows()); // Matrix of local rotations
        for (int i = 0; i < V.rows(); i++) {
            MatrixXd Si = compute_covariance_matrix(previous_V, new_V, i);

            JacobiSVD<MatrixXd> svd(Si, ComputeThinU | ComputeThinV);

            DiagonalMatrix<double, 3> D(1, 1, (svd.matrixV() * svd.matrixU().transpose()).determinant());
            MatrixXd Ri = svd.matrixV() * D * svd.matrixU().transpose();

            // =====    Alternative for Ri determination    =====
            /*MatrixXd svdU = svd.matrixU();
            MatrixXd svdV = svd.matrixV();
            MatrixXd Ri = svdV * svdU.transpose();    // Initial Guest
            int a;
            if (Ri.determinant() <= 0)
            {
                // Get the smallest singular value
                const VectorXd& singVals = svd.singularValues();
                double i_SmallestSingVal = 0;
                for (int i = 1; i < singVals.rows(); i++)
                    if (singVals(i) < singVals(i_SmallestSingVal))
                        i_SmallestSingVal = i;

                // change the corresponding U's col sign
                svdU.col(i_SmallestSingVal) = -1 * svdU.col(i_SmallestSingVal);

                // recompute Ri
                Ri = svdV * svdU.transpose();
            }

            // Store Ri
            R[i] = Ri;

            // DEBUG
            /*std::cout << "Ri" << std::endl;
            std::cout << Ri << std::endl;
        }

        // Find optimal p'
        MatrixXd b = compute_b(previous_V, R, C);

        previous_V = new_V;
        new_V = L.ldlt().solve(b);

        old_energy = new_energy;
        new_energy = compute_energy(previous_V, new_V, R);

        k++;
    } while (k < kmax && abs(old_energy - new_energy) > tol);

    std::cout << k << std::endl;*/

    return new_V;
}
