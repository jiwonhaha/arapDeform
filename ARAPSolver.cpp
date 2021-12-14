#include "ARAPSolver.h"


// /!\ Ne prend pas en compte les boundaries !!!

#define tol 1e-3




MatrixXd laplacian_init(const Mesh& mesh) {
    #define V mesh.V
    #define W mesh.W
    const std::vector<ControlPoint>& C = mesh.getControlPoints();

    MatrixXd laplacian = -W;

    // Add diagonal value
    for (int i = 0; i < laplacian.rows(); i++) {
        laplacian(i, i) += -laplacian.row(i).sum();
    }

    // Build A, B and y
    MatrixXd A = MatrixXd::Zero(V.rows(), V.rows() - C.size());
    MatrixXd B = MatrixXd::Zero(V.rows(), C.size());
    MatrixXd y = MatrixXd::Zero(C.size(), V.cols());

    int a = 0;
    int b = 0;
    for (int i = 0; i < V.rows(); i++) {
        std::pair<bool, Vector3d> constraint = isConstraint(C, i);

        if (constraint.first) {
            B.col(b) = laplacian.col(i);
            y.row(b) = constraint.second;
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
    MatrixXd right = A.transpose() * (-B * y + laplacian * V);

    MatrixXd x = left.ldlt().solve(right);

    MatrixXd new_V = MatrixXd::Zero(V.rows(), V.cols());
    a = 0;
    for (int i = 0; i < V.rows(); i++) {
        std::pair<bool, Vector3d> constraint = isConstraint(C, i);

        if (constraint.first) {
            new_V.row(i) = constraint.second;
        }
        else {
            new_V.row(i) = x.row(a);
            a++;
        }
    }

    return new_V;
    #undef V
    #undef W
}

MatrixXd compute_covariance_matrix(const Eigen::MatrixXd& W, const std::vector<std::list<int>>& N, const MatrixXd& V, const MatrixXd& new_V, const int& index) {
    MatrixXd Si(V.cols(), V.cols());

    // Retrieve neighbors of v
    std::list<int> neighbors_v = N[index];

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

        // Updated mesh
        Vector3d neighbor_new = new_V.row(*it);
        Vector3d edge_new = v_new - neighbor_new;
        P_new.col(k) = edge_new;


        // Diagonal mesh
        D.diagonal()[k] = W(index, *it);
    }    

    Si = P_init * D * P_new.transpose();
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

MatrixXd compute_b(const Eigen::MatrixXd& W, const std::vector<std::list<int>>& N, const MatrixXd& V, const std::vector<MatrixXd>& R, const std::vector<ControlPoint>& C) {

    MatrixXd b = MatrixXd::Zero(V.rows(), V.cols());

    for (int i = 0; i < V.rows(); i++) {
        VectorXd vi = V.row(i);

        // Retrieve neighbors of v
        std::list<int> neighbors_v = N[i];

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
                    b.row(i) += (double)W(i, *it) * constraint_n.second;
                }

                // Weight of the edge
                double wij = W(i, *it);

                // Neighbor Rotation matrix
                MatrixXd Rj = R[*it];

                b.row(i) += (double)wij / 2 * (Ri + Rj) * (vi - neighbor);

            }
        }
    }

    return b;
}

float compute_energy(const Eigen::MatrixXd& W, const std::vector<std::list<int>>& N, const MatrixXd& V, const MatrixXd& new_V, const std::vector<MatrixXd>& R) {
    float energy = 0;
    for (int i = 0; i < V.rows(); i++) {
        VectorXd vi = V.row(i);
        VectorXd vi_prime = new_V.row(i);

        // Retrieve neighbors of vi
        std::list<int> neighbors_v = N[i];

        // Rotation matrix of i-th vertex
        MatrixXd Ri = R[i];

        for (std::list<int>::iterator it = neighbors_v.begin(); it != neighbors_v.end(); ++it) {

            VectorXd neighbor = V.row(*it);
            VectorXd neighbor_prime = new_V.row(*it);

            // Weight of the edge
            double wij = W(i, *it);

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
MatrixXd arap(const Mesh& mesh, const int& kmax, const EInitialisationType& init, int* outInterationNumber, float* outInitialEnergy, float* outFinalEnergy)
{
    #define V mesh.V
    #define N mesh.N
    #define W mesh.W
    const std::vector<ControlPoint>&  C = mesh.getControlPoints();
    MatrixXd L = mesh.getL_withCP();

    MatrixXd previous_V = V;

    // Make an intial guess of new_V according to the initialisation type choosen:
    MatrixXd new_V;
    // User interaction
    if (init == EInitialisationType::e_LastFrame) {
        new_V = V;
        //std::cout << "Initiated with last frame" << std::endl;
    }
    // Laplacian initialization
    else if (init == EInitialisationType::e_Laplace) {
        new_V = laplacian_init(mesh);
        //std::cout << "Initiated with laplacian" << std::endl;
    }

    float old_energy = 0;
    float new_energy = 0;


    // ITERATE
    int k = 0;
    do {

        // Find optimal Ri for each cell
        std::vector<MatrixXd> R(V.rows()); // Matrix of local rotations
        for (int i = 0; i < V.rows(); i++) {
            MatrixXd Si = compute_covariance_matrix(W, N, previous_V, new_V, i);

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
            }*/

            // Store Ri
            R[i] = Ri;
        }

        // Find optimal p'
        MatrixXd b = compute_b(W, N, previous_V, R, C);

        previous_V = new_V;
        new_V = L.ldlt().solve(b);

        old_energy = new_energy;
        new_energy = compute_energy(W, N, previous_V, new_V, R);

        if (k == 0 && outInitialEnergy != nullptr)
            *outInitialEnergy = new_energy;

        k++;
    } while (k < kmax && abs(old_energy - new_energy) > tol);

    if (outInterationNumber != nullptr)
        *outInterationNumber = k;
    if (outFinalEnergy != nullptr)
        *outFinalEnergy = new_energy;

    return new_V;
    #undef V
    #undef N
    #undef W
}
