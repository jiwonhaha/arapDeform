#include <igl/opengl/glfw/Viewer.h>
#include "ARAPSolver.h"
#include "Mesh.h"
#include "MouseManager.h"

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

    // Setup the ARAP related objects
    MouseManager mouseManager = MouseManager();

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.callback_mouse_down = [&mesh, &mouseManager](igl::opengl::glfw::Viewer& viewer, int, int)->bool
    {
        return mouseManager.onMousePressed(viewer, mesh);
    };
    viewer.callback_mouse_up = [&mouseManager](igl::opengl::glfw::Viewer& viewer, int, int)->bool
    {
        return mouseManager.onMouseReleased();
    };
    viewer.callback_mouse_move = [&mouseManager](igl::opengl::glfw::Viewer& viewer, int, int)->bool
    {
        mouseManager.onMouseMoved();
        return false;
    };
    viewer.data().set_mesh(mesh.V, mesh.F);
    viewer.data().set_face_based(true);
    viewer.launch();
}
