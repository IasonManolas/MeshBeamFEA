#include "gui.hpp"
#include <igl/file_dialog_open.h>
#include <igl/list_to_matrix.h>
#include <igl/matrix_to_list.h>
#include <regex>

using Menu = igl::opengl::glfw::imgui::ImGuiMenu;

void parseIntegers(const std::string &str, std::vector<int> &result) {
  typedef std::regex_iterator<std::string::const_iterator> re_iterator;
  typedef re_iterator::value_type re_iterated;

  std::regex re("(\\d+)");

  re_iterator rit(str.begin(), str.end(), re);
  re_iterator rend;

  std::transform(rit, rend, std::back_inserter(result),
                 [](const re_iterated &it) { return std::stoi(it[1]); });
}

GUI::GUI() {
  createMenu();
  viewer.launch();
}

GUI::~GUI() {}

void GUI::createMenu() {
  Menu *pMenu = new Menu;
  pMenu->callback_draw_viewer_menu = [&]() {
    const float w = ImGui::GetContentRegionAvailWidth();
    const float p = ImGui::GetStyle().FramePadding.x;

    // Workspace tab
    if (ImGui::CollapsingHeader("Workspace", ImGuiTreeNodeFlags_None)) {
      if (ImGui::Button("Clear", ImVec2(-1, 0))) {
        viewer.clearDrawingData();
        entries.simulation.nodalForces.clear();
      }
    }

    // Edge mesh tab
    if (ImGui::CollapsingHeader("Edge Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Load##Edge Mesh", ImVec2((w - p) / 2.f, 0))) {
        if (viewer.hasDrawingData(drawingDataIDs.edgeMeshID)) {
          viewer.deleteDrawingData(drawingDataIDs.edgeMeshID);
        }
        if (loadEdgeMesh()) {
          entries.shouldDrawEdgeMesh = true;
          drawEdgeMesh();
        }
      }
      if (ImGui::Checkbox("Show Edge Mesh", &entries.shouldDrawEdgeMesh)) {
        if (entries.shouldDrawEdgeMesh) {
          drawEdgeMesh();
        } else {
          viewer.setDrawingDataVisibility(drawingDataIDs.edgeMeshID, false);
        }
        viewer.setDrawingDataVisibility(drawingDataIDs.nodalForcesID,
                                        entries.shouldDrawEdgeMesh);
      }
    }

    // Viewing options tab
    if (ImGui::CollapsingHeader("Viewing Options",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Center object", ImVec2((w - p) / 2.f, 0))) {
        viewer.centerCamera(drawingDataIDs.edgeMeshID);
      }
      ImGui::SameLine(0, p);
      if (ImGui::Button("Snap canonical view", ImVec2((w - p) / 2.f, 0))) {
        viewer.snapCanonicalView();
      }
      // Select rotation type
      static RotationType rotationType{viewer.getRotationType()};
      if (ImGui::Combo("Camera Type", reinterpret_cast<int *>(&rotationType),
                       "Trackball\0Two Axes\0002D Mode\0\0")) {
        viewer.setRotationType(rotationType);
      }
      // Orthographic view
      static bool shouldUseOrthographicView =
          viewer.getShouldUseOrthographicView();
      if (ImGui::Checkbox("Orthographic view", &(shouldUseOrthographicView))) {
        viewer.setUseOrthographicView(shouldUseOrthographicView);
      }
      // Invert normals
      static bool shouldInvertNormals = false;
      if (ImGui::Checkbox("Invert normals", &(shouldInvertNormals))) {
        viewer.setShouldInvertNormals(drawingDataIDs.edgeMeshID,
                                      shouldInvertNormals);
        viewer.setShouldInvertNormals(drawingDataIDs.displacedEdgeMeshID,
                                      shouldInvertNormals);
      }
      // Draw axis
      static bool shouldDrawAxis = false;
      if (ImGui::Checkbox("Show axis", &shouldDrawAxis)) {
        if (viewer.hasDrawingData(drawingDataIDs.worldAxisID)) {
          viewer.setDrawingDataVisibility(drawingDataIDs.worldAxisID,
                                          shouldDrawAxis);
        } else {
          drawer.drawWorldAxis(drawingDataIDs.worldAxisID, viewer,
                               shouldDrawAxis);
        }
      }
      // Choose force component
      static NodalForceComponent force =
          entries.viewingOptions.chosenForceComponent;
      ImGui::Combo("Force visualization", reinterpret_cast<int *>(&force),
                   "N\0Ty\0Tx\0Mx\0My\0Mz\0");
      if (force != entries.viewingOptions.chosenForceComponent) {
        entries.viewingOptions.chosenForceComponent = force;
        if (viewer.hasDrawingData(drawingDataIDs.displacedEdgeMeshID)) {
          drawer.setBeamColors(drawingDataIDs.displacedEdgeMeshID,
                               entries.viewingOptions.chosenForceComponent,
                               viewer);
        }
      }
      // Draw force component picker
      drawColorTypeCombo();
      // Draw colorbar if there are simulation data
      if (viewer.hasDrawingData(drawingDataIDs.displacedEdgeMeshID)) {
        drawColorbar();
      }
    }

    // Simulation tab
    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      // Execute simulation
      if (ImGui::Button("Execute \n Simulation", ImVec2((w - p) / 2.f, 0))) {
        if (edgeMesh.IsEmpty())
          return;
        executeSimulation();
      }
      // Fix vertices
      static std::string strfixedVertices;
      if (ImGui::InputText("Fixed Vertices", strfixedVertices)) {
        if (edgeMesh.IsEmpty())
          return;
        entries.simulation.fixedVertices.clear();
        parseIntegers(strfixedVertices, entries.simulation.fixedVertices);
        if (viewer.hasDrawingData(drawingDataIDs.fixedNodesID)) {
          viewer.deleteDrawingData(drawingDataIDs.fixedNodesID);
        }
        if (!entries.simulation.fixedVertices.empty()) {
          Eigen::MatrixX3d fixedNodePositions(
              entries.simulation.fixedVertices.size(), 3);
          for (gsl::index fixedNodeIndex = 0;
               fixedNodeIndex < entries.simulation.fixedVertices.size();
               fixedNodeIndex++) {
            fixedNodePositions.row(fixedNodeIndex) =
                VCGTriMesh::convertToEigenVector(
                    edgeMesh
                        .vert[entries.simulation.fixedVertices[fixedNodeIndex]]
                        .cP());
          }

          drawer.markPositions(drawingDataIDs.fixedNodesID, fixedNodePositions,
                               viewer);
        }
      }
      // Set nodal force
      if (ImGui::CollapsingHeader("Nodal Force",
                                  ImGuiTreeNodeFlags_DefaultOpen)) {
        // Set nodal force vertex index
        static int vertexIndex = entries.simulation.force.vertexIndex;
        if (ImGui::InputInt("Vertex Index", &vertexIndex)) {
          entries.simulation.force.vertexIndex = vertexIndex;
          if (edgeMesh.IsEmpty())
            return;
          if (entries.simulation.force.vertexIndex < 0) {
            std::cerr << "Only positive values can be used as vertex indices."
                      << std::endl;
          }
        }
        // Set nodal force DoF [0,5]
        if (ImGui::InputInt("Force DoF", &entries.simulation.force.dof)) {
          if (edgeMesh.IsEmpty())
            return;
          if (entries.simulation.force.dof > 5 ||
              entries.simulation.force.dof < 0) {
            std::cerr << "DoF must be in the range [0,5]" << std::endl;
          }
        }
        // Set nodal force magnitude
        if (ImGui::InputFloat("Force Magnitude",
                              &entries.simulation.force.magnitude)) {
        }
        // Add nodal force
        if (ImGui::Button("Add nodal \n force", ImVec2((w - p) / 2.f, 0))) {
          if (edgeMesh.IsEmpty())
            return;
          addNodalForce();
          drawNodalForces();
        }
        ImGui::SameLine(0, p);
        // Clear nodal forces
        if (ImGui::Button("Clear nodal \n forces", ImVec2((w - p) / 2.f, 0))) {
          if (edgeMesh.IsEmpty())
            return;
          entries.simulation.nodalForces.clear();
          viewer.deleteDrawingData(drawingDataIDs.nodalForcesID);
        }
      }
    }
  };
  viewer.addPlugin("Main menu", pMenu);
}

bool GUI::loadEdgeMesh() {
  const std::string meshFilenameString = igl::file_dialog_open();
  size_t last_dot = meshFilenameString.rfind('.');
  if (last_dot == std::string::npos) {
    std::cerr << "Error: No file extension found in " << meshFilenameString
              << std::endl;
    return false;
  }

  std::string extension = meshFilenameString.substr(last_dot + 1);

  if (extension != "ply" && extension != "PLY") {
    std::cerr << "Error: Only ply file types are supported" << std::endl;
    return false;
  }
  return edgeMesh.loadFromPly(meshFilenameString);
}

void GUI::setSimulation() {
  Eigen::VectorXi fixedNodes;
  igl::list_to_matrix(entries.simulation.fixedVertices, fixedNodes);

  const Eigen::MatrixX2i elements = edgeMesh.getEigenEdges();
  const std::vector<BeamDimensions> beamDimensions =
      edgeMesh.getBeamDimensions();
  const std::vector<BeamMaterial> beamMaterial = edgeMesh.getBeamMaterial();

  const Eigen::MatrixX3d nodes = edgeMesh.getEigenVertices();
  const Eigen::MatrixX3d elementNormals = edgeMesh.getEigenEdgeNormals();
  simulator.setSimulation(nodes, elements, elementNormals, fixedNodes,
                          entries.simulation.nodalForces, beamDimensions,
                          beamMaterial);
}

void GUI::drawEdgeMesh() {
  if (viewer.hasDrawingData(drawingDataIDs.edgeMeshID)) {
    viewer.setDrawingDataVisibility(drawingDataIDs.edgeMeshID, true);

  } else {
    Eigen::MatrixX3d vertices = edgeMesh.getEigenVertices();
    Eigen::MatrixX2i edges = edgeMesh.getEigenEdges();
    drawer.drawEdges(drawingDataIDs.edgeMeshID, vertices, edges,
                     edgeMesh.getEigenEdgeNormals(),
                     edgeMesh.getBeamDimensions(), viewer);
  }
}

void GUI::executeSimulation() {
  assert(!edgeMesh.IsEmpty());
  if (viewer.hasDrawingData(drawingDataIDs.displacedEdgeMeshID)) {
    viewer.deleteDrawingData(drawingDataIDs.displacedEdgeMeshID);
  }
  setSimulation();

  const auto simulationResults = simulator.executeSimulation();

  drawSimulationResults(simulationResults.nodal_displacements,
                        simulationResults.element_forces);
}

void GUI::drawSimulationResults(
    const std::vector<std::vector<double>> &nodalDisplacements,
    const std::vector<std::vector<double>> &elementForces) {
  drawDisplacedEdgeMesh(nodalDisplacements);
  drawEdgeForces(elementForces);
}

void GUI::addNodalForce() {
  if (entries.simulation.force.vertexIndex >= 0 &&
      entries.simulation.force.dof < 6 && entries.simulation.force.dof >= 0) {
    NodalForce force;
    force.dof = entries.simulation.force.dof;
    force.index = entries.simulation.force.vertexIndex;
    force.magnitude = static_cast<double>(entries.simulation.force.magnitude);
    entries.simulation.nodalForces.push_back(force);
    std::cout << "Force <" << force.dof << "," << force.index << ","
              << force.magnitude << "> was added." << std::endl;
  } else {
    std::cerr << "Force could not be created." << std::endl;
  }
}

void GUI::drawNodalForces() {
  if (viewer.hasDrawingData(drawingDataIDs.nodalForcesID)) {
    viewer.deleteDrawingData(drawingDataIDs.nodalForcesID);
  }

  std::unordered_map<gsl::index, Eigen::Vector3d> nodeIndexForceMap;
  for (gsl::index nodeIndex = 0;
       nodeIndex < entries.simulation.nodalForces.size(); nodeIndex++) {
    const NodalForce &nodalForce = entries.simulation.nodalForces[nodeIndex];
    switch (nodalForce.dof) {
    case 0:
      nodeIndexForceMap[nodalForce.index] +=
          Eigen::Vector3d(nodalForce.magnitude, 0, 0);
      break;
    case 1:
      nodeIndexForceMap[nodalForce.index] +=
          Eigen::Vector3d(0, nodalForce.magnitude, 0);
      break;
    case 2:
      nodeIndexForceMap[nodalForce.index] +=
          Eigen::Vector3d(0, 0, nodalForce.magnitude);
      break;
    default:
      std::cerr << "Force with dof of " << nodalForce.dof
                << " is not being visualized." << std::endl;
    }
  }
  if (nodeIndexForceMap.empty())
    return;

  Eigen::MatrixX3d nodePositions(nodeIndexForceMap.size(), 3),
      edgeEndPositions(nodeIndexForceMap.size(), 3);
  Eigen::VectorXd forcesMagnitude(nodeIndexForceMap.size());
  gsl::index forceIndex = 0;
  for (auto it = nodeIndexForceMap.begin(); it != nodeIndexForceMap.end();
       it++) {
    const Eigen::Vector3d &forceVector = it->second;
    forcesMagnitude(forceIndex++) = forceVector.norm();
  }
  const float edgeLength = 1;
  forceIndex = 0;
  const double maxForce = forcesMagnitude.maxCoeff();
  for (auto it = nodeIndexForceMap.begin(); it != nodeIndexForceMap.end();
       it++) {
    const gsl::index &nodeIndex = it->first;
    const Eigen::Vector3d &forceVector = it->second;
    // fill position
    nodePositions.row(forceIndex) =
        VCGTriMesh::convertToEigenVector(edgeMesh.vert[nodeIndex].cP());
    edgeEndPositions.row(forceIndex) =
        nodePositions.row(forceIndex) +
        edgeLength * (forceVector / maxForce).transpose();
    forceIndex++;
  }
  Eigen::MatrixX3d color(1, 3);
  color.row(0) = Eigen::Vector3d(1, 1, 1);
  drawer.drawEdges(drawingDataIDs.nodalForcesID, nodePositions,
                   edgeEndPositions, color, viewer);
}

void GUI::drawDisplacedEdgeMesh(
    const std::vector<std::vector<double>> &nodalDisplacements) {
  Eigen::MatrixXd eigenNodalDisplacements;
  igl::list_to_matrix(nodalDisplacements, eigenNodalDisplacements);
  Eigen::MatrixX3d nodes = edgeMesh.getEigenVertices();
  Eigen::MatrixX3d eigenNodalDisplacementsXYZ(nodes.rows(), 3);
  eigenNodalDisplacementsXYZ.col(0) = eigenNodalDisplacements.col(0);
  eigenNodalDisplacementsXYZ.col(1) = eigenNodalDisplacements.col(1);
  eigenNodalDisplacementsXYZ.col(2) = eigenNodalDisplacements.col(2);
  assert(eigenNodalDisplacementsXYZ.rows() == nodes.rows());
  const Eigen::MatrixX3d displacedVertices = eigenNodalDisplacementsXYZ + nodes;
  //  viewer.setDrawingDataVisibility(drawingDataIDs.edgeMeshID, false);
  //  drawEdgeMesh();
  Eigen::MatrixX2i elements = edgeMesh.getEigenEdges();
  drawer.drawEdges(drawingDataIDs.displacedEdgeMeshID, displacedVertices,
                   elements, edgeMesh.getEigenEdgeNormals(),
                   edgeMesh.getBeamDimensions(), viewer);
}

void GUI::drawEdgeForces(const std::vector<std::vector<double>> &edgeForces) {
  std::vector<Eigen::VectorXd> eigenEdgeForces;
  convertToEigen(edgeForces, eigenEdgeForces);
  minMaxForcesPerForceComponent.clear();
  minMaxForcesPerForceComponent.resize(
      NodalForceComponent::NumberOfForceComponents);
  for (size_t fc = Fx; fc < NodalForceComponent::NumberOfForceComponents;
       fc++) {
    minMaxForcesPerForceComponent[fc] = std::make_pair(
        eigenEdgeForces[fc].minCoeff(), eigenEdgeForces[fc].maxCoeff());
  }
  std::vector<Eigen::MatrixXd> edgeColors;
  convertToColors(eigenEdgeForces, edgeColors);
  Eigen::MatrixXd &beamMeshVertices =
      viewer.getDrawingData(drawingDataIDs.displacedEdgeMeshID).V;
  drawer.computeBeamColors(edgeMesh.getEigenVertices(),
                           edgeMesh.getEigenEdges(), edgeColors,
                           beamMeshVertices);
  drawer.setBeamColors(drawingDataIDs.displacedEdgeMeshID,

                       entries.viewingOptions.chosenForceComponent, viewer);
  init_colormaps(eigenEdgeForces, entries.viewingOptions.chosenColormapType);
}

void GUI::convertToEigen(
    const std::vector<std::vector<double>> &forcesPerEdgePerComponent,
    std::vector<Eigen::VectorXd> &forcesPerComponentPerEdge) {

  // Convert to vector of eigen matrices of the form force component-> per
  // Edge
  // force values does global su
  const int numDof = 6;
  const size_t numberOfEdges = forcesPerEdgePerComponent.size();
  forcesPerComponentPerEdge =
      std::vector<Eigen::VectorXd>(numDof, Eigen::VectorXd(2 * numberOfEdges));
  for (gsl::index edgeIndex = 0; edgeIndex < numberOfEdges; edgeIndex++) {
    for (gsl::index forceComponentIndex = NodalForceComponent::Fx;
         forceComponentIndex < numDof; forceComponentIndex++) {
      (forcesPerComponentPerEdge[forceComponentIndex])(2 * edgeIndex) =
          forcesPerEdgePerComponent[edgeIndex][forceComponentIndex];
      (forcesPerComponentPerEdge[forceComponentIndex])(2 * edgeIndex + 1) =
          forcesPerEdgePerComponent[edgeIndex][numDof + forceComponentIndex];
    }
  }
}

void GUI::convertToColors(const std::vector<Eigen::VectorXd> &edgeForces,
                          std::vector<Eigen::MatrixXd> &edgeColors) const {

  const int numDof = NodalForceComponent::NumberOfForceComponents;
  edgeColors.resize(numDof);
  // compute the color of the vertex of each edge
  for (gsl::index forceComponentIndex = NodalForceComponent::Fx;
       forceComponentIndex < numDof; forceComponentIndex++) {
    igl::colormap(entries.viewingOptions.chosenColormapType,
                  edgeForces[forceComponentIndex], true,
                  edgeColors[forceComponentIndex]);
  }
}

void GUI::drawColorbar() {
  Eigen::Vector4f backgroundColor(0.3f, 0.3f, 0.5f, 1.0f);
  backgroundColor *= 255;
  draw_colorbar(
      entries.viewingOptions.chosenForceComponent,
      minMaxForcesPerForceComponent[entries.viewingOptions.chosenForceComponent]
          .first,
      minMaxForcesPerForceComponent[entries.viewingOptions.chosenForceComponent]
          .second,
      backgroundColor);
}

void GUI::drawColorTypeCombo() {
  const std::vector<std::string> items{
      "Inferno", "Jet", "Magma", "Parula", "Plasma", "Viridis",
  };
  static gsl::index selected_index = entries.viewingOptions.chosenColormapType;
  static std::string current_item = items[selected_index];
  ImVec2 combo_pos = ImGui::GetCursorScreenPos();
  if (ImGui::BeginCombo("Coloring Type##combo", "")) {
    for (gsl::index n = 0; n < items.size(); ++n) {
      // You can store your selection however you want, outside or inside your
      // objects
      bool is_selected = (current_item == items[n]);
      ImGui::PushID(n);
      if (ImGui::Selectable("", is_selected)) {
        current_item = items[n];
        selected_index = n;
      }
      ImGui::SameLine(0, 0);
      ImGui::Text("%s", items[n].c_str());
      if (is_selected) {
        ImGui::SetItemDefaultFocus();
      }
      ImGui::PopID();
    }
    ImGui::EndCombo();
  }

  ImVec2 backup_pos = ImGui::GetCursorScreenPos();
  ImGuiStyle &style = ImGui::GetStyle();
  ImGui::SetCursorScreenPos(ImVec2(combo_pos.x + style.FramePadding.x,
                                   combo_pos.y + style.FramePadding.y));
  float h = ImGui::GetTextLineHeight();
  ImGui::Text("%s", current_item.c_str());
  ImGui::SetCursorScreenPos(backup_pos);

  if (selected_index != entries.viewingOptions.chosenColormapType) {
    entries.viewingOptions.chosenColormapType =
        static_cast<igl::ColorMapType>(selected_index);
    drawEdgeForces(simulator.getResults().element_forces);
  }
}
