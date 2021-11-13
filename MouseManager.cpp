#include "MouseManager.h"
#include <igl/unproject_onto_mesh.h>
#include <igl/point_mesh_squared_distance.h>


bool MouseManager::onMousePressed(igl::opengl::glfw::Viewer& viewer, Mesh& mesh)
{
    mouseIsPressed = true;

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

        Eigen::MatrixXd CV = Eigen::MatrixXd::Zero(2,3);
        CV.row(0) = mesh.V.row(mesh.F.row(fid)[closestVertex]);
        viewer.data().set_points(CV, Eigen::RowVector3d(1,0,0));
        return true;
    }
    return false;
}

bool MouseManager::onMouseReleased()
{
    mouseIsPressed = false;
    return true;
}

bool MouseManager::onMouseMoved()
{
    if (!mouseIsPressed)
        return false;

    // ... drag the selected control points
    std::cout << "Draging Mouse" << std::endl;
    return true;
}