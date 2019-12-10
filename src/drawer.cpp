#include "drawer.hpp"
#include <igl/jet.h>
#include <igl/list_to_matrix.h>

void SimulationResultsDrawer::computeVerticesColors(
    const std::vector<std::vector<double>> &vertexDataValues) {
  Eigen::MatrixXd eigenVertexDataValues(vcgMesh.VN(), 6);
  igl::list_to_matrix(vertexDataValues, eigenVertexDataValues);
  for (int verticesDataFieldIndex = 0;
       verticesDataFieldIndex < eigenVertexDataValues.cols();
       verticesDataFieldIndex++) {
    Eigen::VectorXd verticesDataField(
        eigenVertexDataValues.col(verticesDataFieldIndex));
    Eigen::MatrixX3d colors(vcgMesh.VN(), 3);
    igl::jet(verticesDataField, true, colors);
    verticesColors.push_back(colors);
  }
}

void SimulationResultsDrawer::vcgToEigenMesh(const VCGMesh &vcgMesh,
                                             Eigen::MatrixX3d &vertices,
                                             Eigen::MatrixX3i &triFaces) {
  vertices.resize(vcgMesh.VN(), 3);
  const std::vector<MyVertex> &meshVerts = vcgMesh.vert;
  for (size_t vertexIndex = 0; vertexIndex < meshVerts.size(); vertexIndex++) {
    vertices.row(static_cast<Eigen::Index>(vertexIndex)) = Eigen::Vector3d(
        meshVerts[vertexIndex].P().X(), meshVerts[vertexIndex].P().Y(),
        meshVerts[vertexIndex].P().Z());
  }

  triFaces.resize(vcgMesh.FN(), 3);
  const std::vector<MyFace> &meshFaces = vcgMesh.face;
  for (size_t faceIndex = 0; faceIndex < meshFaces.size(); faceIndex++) {
    int vertex0Index = static_cast<int>(
        vcg::tri::Index<VCGMesh>(vcgMesh, meshFaces[faceIndex].cV(0)));
    int vertex1Index = static_cast<int>(
        vcg::tri::Index<VCGMesh>(vcgMesh, meshFaces[faceIndex].cV(1)));
    int vertex2Index = static_cast<int>(
        vcg::tri::Index<VCGMesh>(vcgMesh, meshFaces[faceIndex].cV(2)));
    triFaces.row(static_cast<Eigen::Index>(faceIndex)) =
        Eigen::Vector3i(vertex0Index, vertex1Index, vertex2Index);
  }
}

void SimulationResultsDrawer::addMesh(const Eigen::MatrixX3d &vertices,
                                      const Eigen::MatrixX3i &triFaces,
                                      const bool shouldBeVisible,
                                      igl::opengl::glfw::Viewer &viewer) const {
  igl::opengl::ViewerData data;
  data.set_mesh(vertices, triFaces);
  data.set_visible(shouldBeVisible);
  data.compute_normals();
  viewer.data_list.push_back(data);
}

void SimulationResultsDrawer::createMenu(
    igl::opengl::glfw::Viewer &viewer,
    igl::opengl::glfw::imgui::ImGuiMenu &menu) const {
  menu.callback_draw_viewer_menu = [&]() {
    menu.draw_viewer_menu();
    static bool shouldDrawDisplacedMesh = false;

    // Add new group
    if (ImGui::CollapsingHeader("Simulation Results",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Checkbox("Show displaced mesh", &shouldDrawDisplacedMesh)) {
        assert(viewer.data_list.size() == 2);
        viewer.data_list[1].set_visible(shouldDrawDisplacedMesh);
        viewer.data_list[0].set_visible(!shouldDrawDisplacedMesh);
      }
      enum ChosenForce { Fx = 0, Fy, Fz, Mx, My, Mz };
      static ChosenForce force = Fx;
      ImGui::Combo("Force visualization", reinterpret_cast<int *>(&force),
                   "Fx\0Fy\0Fz\0Mx\0My\0Mz\0");
      viewer.data_list[1].set_colors(
          verticesColors[static_cast<size_t>(force)]);
    }
  };
}

void SimulationResultsDrawer::computeDisplacedVertices(
    const Eigen::MatrixX3d &vertices, Eigen::MatrixX3d &displacedVertices,
    const float displacementFactor = 30) const {
  displacedVertices = vertices;
  for (size_t vertexIndex = 0;
       vertexIndex < static_cast<size_t>(vertices.rows()); vertexIndex++) {
    displacedVertices.row(static_cast<Eigen::Index>(vertexIndex)) +=
        displacementFactor *
        Eigen::Vector3d(vertexDisplacements[vertexIndex][0],
                        vertexDisplacements[vertexIndex][1],
                        vertexDisplacements[vertexIndex][2]);
  }
}

SimulationResultsDrawer::SimulationResultsDrawer(
    const VCGMesh &m, std::vector<std::vector<double>> &vDisplacements,
    std::vector<std::vector<double>> &vForces)
    : vcgMesh(m), vertexDisplacements(vDisplacements) {
  computeVerticesColors(vForces);
}

void SimulationResultsDrawer::draw() const {
  igl::opengl::glfw::Viewer viewer;
  viewer.data_list.clear();
  Eigen::MatrixX3d vertices;
  Eigen::MatrixX3i triFaces;
  vcgToEigenMesh(vcgMesh, vertices, triFaces);
  addMesh(vertices, triFaces, true, viewer);
  Eigen::MatrixX3d displacedVertices;
  computeDisplacedVertices(vertices, displacedVertices);
  addMesh(displacedVertices, triFaces, false, viewer);
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  createMenu(viewer, menu);
  viewer.plugins.push_back(&menu);

  viewer.launch();
}
