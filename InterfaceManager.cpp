#include "InterfaceManager.h"
#include <igl/unproject_onto_mesh.h>
#include <igl/point_mesh_squared_distance.h>


void InterfaceManager::displaySelectedPoints(igl::opengl::glfw::Viewer& viewer, Mesh& mesh)
{
    Eigen::MatrixXd CV = Eigen::MatrixXd::Zero(selection.size(), 3);
    for (int i = 0; i < CV.rows(); i++)
        CV.row(i) = mesh.V.row(selection[i]);
    
    if (moveSelectionMode)
        viewer.data().set_points(CV, Eigen::RowVector3d(1, 0.5, 0));
    else
        viewer.data().set_points(CV, Eigen::RowVector3d(1, 0, 0));
}


void InterfaceManager::onMousePressed(igl::opengl::glfw::Viewer& viewer, Mesh& mesh, bool isShiftPressed)
{
    mouseIsPressed = true;

    if (moveSelectionMode)
        return;

    int fid;
    Eigen::Vector3d bc;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view,
        viewer.core().proj, viewer.core().viewport, mesh.V, mesh.F, fid, bc))
    {
        // paint hit in orange
        Eigen::MatrixXd C = Eigen::MatrixXd::Constant(mesh.F.rows(), 3, 1);
        C.row(fid) << 0.94, 0.69, 0.36;
        viewer.data().set_colors(C);


        int closestVertex = 0;
        for (int i = 1; i < 3; i++)
            if (bc[i] > bc[closestVertex])
                closestVertex = i;

        if (!isShiftPressed)
            selection.clear();

        selection.push_back(mesh.F.row(fid)[closestVertex]);
    }
    else if (isShiftPressed)
        selection.clear();

    displaySelectedPoints(viewer, mesh);
}

void InterfaceManager::onMouseReleased()
{
    mouseIsPressed = false;
}

bool InterfaceManager::onMouseMoved()
{
    if (!mouseIsPressed)
        return false;

    if (moveSelectionMode)
    {
        // move control points that are in the selection
        // ....
        return true;
    }
    else
        return false;   // simply move the viewpoint    
}

void InterfaceManager::onKeyPressed(igl::opengl::glfw::Viewer& viewer, Mesh& mesh, unsigned char key)
{
    std::cout << "pressed Key: " << key << " " << (unsigned int)key << std::endl;
    if (key == 'M')
    {
        // Swap mode : move or select
        moveSelectionMode = !moveSelectionMode;
        displaySelectedPoints(viewer, mesh);
    }
    else if (key == 'C')
        int a = 1;  // add selected points to control points
    else if (key == 'R')
        int a = 1;  // remove selected points of control points
    else if (key == 'X')
        int a = 1;  // set move axis to X
    else if (key == 'Y')
        int a = 1;  // set move axis to Y
    else if (key == 'Z')
        int a = 1;  // set move axis to Z
}