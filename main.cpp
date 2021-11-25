#include <igl/opengl/glfw/Viewer.h>
#include "ARAPSolver.h"
#include "Mesh.h"
#include "InterfaceManager.h"


int main(int argc, char *argv[])
{
    // Inline mesh of a cube
    Mesh mesh = Mesh();
    mesh.V = (Eigen::MatrixXd(8,3)<<
                0.0,0.0,0.0,
                0.0,0.0,1.0,
                0.0,1.0,0.0,
                0.0,1.0,1.0,
                1.0,0.0,0.0,
                1.0,0.0,1.0,
                1.0,1.0,0.0,
                1.0,1.0,1.0).finished();
    mesh.F = (Eigen::MatrixXi(12,3)<<
                1,7,5,
                1,3,7,
                1,4,3,
                1,2,4,
                3,8,7,
                3,4,8,
                5,7,8,
                5,8,6,
                1,5,6,
                1,6,2,
                2,6,8,
                2,8,4).finished().array()-1;

    // Find one-ring neighbors
    find_neighbors(mesh.V, mesh.F);
    
    // DEBUG
    /*// Compute edges weights
    compute_edges_weight(mesh.V, mesh.F);
    // Compute Laplacian matrix
    compute_laplacian_matrix();*/

    MatrixXd new_V(mesh.V.rows(), mesh.V.cols());

    /*MatrixXd R(3, 3);
    float theta = (float) 3.14 / 4;
    R << cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;

    std::cout << R << std::endl;*/

    /*for (int i = 0; i < mesh.V.rows(); i++) {
        new_V.row(i) = mesh.V.row(i) * R.transpose();
    }*/

    new_V = mesh.V;
    //new_V.row(0) << -1.0, 0.0, 0.0;


    //std::cout << new_V << std::endl;
    //mesh.addControlPoint(5, RowVector3d(-1.0, 0.0, 0.0));
    std::vector<ControlPoint> C = mesh.getControlPoints();

    /*std::cout << "C = { ";
    for (ControlPoint c : C) {
            std::cout << c.vertexIndexInMesh << ": ";
            std::cout << c.wantedVertexPosition << ", ";
    }
    std::cout << "}; \n";*/

    arap(mesh.V, mesh.F, C, new_V);

    // Setup the interface
    igl::opengl::glfw::Viewer viewer;
    InterfaceManager interfaceManager = InterfaceManager();
    viewer.callback_mouse_down = [&interfaceManager, &mesh](igl::opengl::glfw::Viewer& viewer, int, int modifier)->bool
    {
        interfaceManager.onMousePressed(viewer, mesh, modifier & 0x00000001);
        return false;
    };
    viewer.callback_mouse_up = [&interfaceManager](igl::opengl::glfw::Viewer& viewer, int, int)->bool
    {
        interfaceManager.onMouseReleased();
        return false;
    };
    viewer.callback_mouse_move = [&interfaceManager](igl::opengl::glfw::Viewer& viewer, int, int modifier)->bool
    {
        return interfaceManager.onMouseMoved();
    };
    viewer.callback_key_down = [&interfaceManager, &mesh](igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)->bool
    {
        interfaceManager.onKeyPressed(viewer, mesh, key, modifier & 0x00000001);
        return false;
    };

    // Plot the mesh
    viewer.data().set_mesh(mesh.V, mesh.F);
    //viewer.data().set_mesh(new_V, mesh.F);
    viewer.data().set_face_based(true);
    viewer.launch();
}
