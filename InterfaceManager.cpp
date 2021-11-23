#include "InterfaceManager.h"
#include <igl/unproject_onto_mesh.h>
#include <igl/point_mesh_squared_distance.h>


std::vector<int> InterfaceManager::getSelectedControlPoints(const Mesh& mesh, bool invert)
{
    std::vector<int> selection_cp = std::vector<int>();
    for (const auto& i : selection)
        if (mesh.isAControlPoint(i) ^ invert)
            selection_cp.push_back(i);

    return selection_cp;
}
std::vector<int> InterfaceManager::getNonSelectedControlPoints(const Mesh& mesh)
{
    std::vector<int> selection_cp = std::vector<int>();
    std::vector<int> all_cp = mesh.getControlPointsIndex();
    for (const auto& i : all_cp)
    {
        bool notInSelection = true;
        for (const auto& j : selection)
            if (i == j)
            {
                notInSelection = false;
                break;
            }
        if(notInSelection)
            selection_cp.push_back(i);
    }
    
    return selection_cp;
}

void InterfaceManager::displaySelectedPoints(igl::opengl::glfw::Viewer& viewer, Mesh& mesh)
{
    // retrieve the control points not selected
    Eigen::MatrixXd cpNotSelected = mesh.getVerticesFromIndex(getNonSelectedControlPoints(mesh));
    // retrieve the control points selected
    Eigen::MatrixXd cpSelected = mesh.getVerticesFromIndex(getSelectedControlPoints(mesh));
    // retrieve the standard points selected
    Eigen::MatrixXd notCpSelected = mesh.getVerticesFromIndex(getSelectedControlPoints(mesh, true));

    
    if (moveSelectionMode)
    {
        viewer.data().set_points(cpNotSelected, Eigen::RowVector3d(0, 0.5, 0));
        viewer.data().add_points(cpSelected, Eigen::RowVector3d(0, 1, 0.5));
        viewer.data().add_points(notCpSelected, Eigen::RowVector3d(1, 0.5, 0));
    }
    else
    {
        viewer.data().set_points(cpNotSelected, Eigen::RowVector3d(0, 0.5, 0));
        viewer.data().add_points(cpSelected, Eigen::RowVector3d(0, 1, 0));
        viewer.data().add_points(notCpSelected, Eigen::RowVector3d(1, 0, 0));
    }
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

        if (isShiftPressed)
        {
            int selectedVertexIndex = mesh.F.row(fid)[closestVertex];
            int indexOnVectorIfExists = -1;
            for (int i = 0; i < selection.size(); i++)
                if (selection[i] == selectedVertexIndex)
                {
                    indexOnVectorIfExists = i;
                    break;
                }
            if (indexOnVectorIfExists < 0)
                selection.push_back(selectedVertexIndex);   // not in selection : add it
            else
                selection.erase(std::next(selection.begin(), indexOnVectorIfExists));   // already in the selection : remove it
        }
        else
        {
            selection.clear();
            selection.push_back(mesh.F.row(fid)[closestVertex]);
        }
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
        // use unproject on plane or line depending on the move direction
        return true;
    }
    else
        return false;   // simply move the viewpoint    
}

void InterfaceManager::onKeyPressed(igl::opengl::glfw::Viewer& viewer, Mesh& mesh, unsigned char key, bool isShiftPressed)
{
    std::cout << "pressed Key: " << key << " " << (unsigned int)key << std::endl;
    if (key == 'G')
    {
        // Swap mode : move or select
        moveSelectionMode = !moveSelectionMode;
        displaySelectedPoints(viewer, mesh);
    }
    else if (key == 'C')
    {
        for (const auto& i : selection)
            mesh.addControlPoint(i);
        displaySelectedPoints(viewer, mesh);
        mesh.printControlPoints();
    }
    else if (key == 'R')
    {
        for (const auto& i : selection)
            mesh.removeControlPoint(i);
        displaySelectedPoints(viewer, mesh);
        mesh.printControlPoints();
    }
    else if (key == 'X')
    {
        if (isShiftPressed)
        {
            firstMoveDirection = Eigen::RowVector3d(0, 1, 0);
            secondMoveDirection = Eigen::RowVector3d(0, 0, 1);
        }
        else
        {
            firstMoveDirection = Eigen::RowVector3d(1, 0, 0);
            secondMoveDirection = Eigen::RowVector3d(0, 0, 0);
        }
    }
    else if (key == 'Y')
    {
        if (isShiftPressed)
        {
            firstMoveDirection = Eigen::RowVector3d(1, 0, 0);
            secondMoveDirection = Eigen::RowVector3d(0, 0, 1);
        }
        else
        {
            firstMoveDirection = Eigen::RowVector3d(0, 1, 0);
            secondMoveDirection = Eigen::RowVector3d(0, 0, 0);
        }
    }
    else if (key == 'Z')
    {
        if (isShiftPressed)
        {
            firstMoveDirection = Eigen::RowVector3d(1, 0, 0);
            secondMoveDirection = Eigen::RowVector3d(0, 1, 0);
        }
        else
        {
            firstMoveDirection = Eigen::RowVector3d(0, 0, 1);
            secondMoveDirection = Eigen::RowVector3d(0, 0, 0);
        }
    }
}