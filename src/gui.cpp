#include "gui.hpp"
#include "utilities.hpp"
#include <ctype.h>
#include <filesystem>
#include <igl/file_dialog_open.h>
#include <igl/list_to_matrix.h>
#include <igl/matrix_to_list.h>

using Menu = igl::opengl::glfw::imgui::ImGuiMenu;

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
      if (ImGui::Button("Load", ImVec2((w - p) / 2.f, 0))) {
        const std::string jsonFilepath = igl::file_dialog_open();
        if (!Utilities::hasExtension(jsonFilepath, ".json") ||
            !std::filesystem::exists(jsonFilepath)) {
          std::cerr << "Scenario could not be loaded because of an invalid "
                       "file scenario file was chosen."
                    << std::endl;
          return;
        }
        if (!Utilities::hasExtension(jsonFilepath, ".json")) {
          std::cerr << "File " + jsonFilepath +
                           " has not a json extension. Can't load workspace."
                    << std::endl;
          return;
        }
        // Parse ply filename from json, load and draw the edge mesh
        std::string plyFilename;
        ConfigurationFile::getPlyFilename(jsonFilepath, plyFilename);
        if (!loadEdgeMesh(plyFilename)) {
          std::cerr << "Loading of " + plyFilename + " failed." << std::endl;
          return;
        }
        shouldDrawEdgeMesh = true;
        clearViewer();
        // Parse the fixed nodes of the simulation scenario,update gui and draw
        // them
        std::vector<gsl::index> fixedVertices;
        ConfigurationFile::getFixedVertices(jsonFilepath, fixedVertices);
        if (!fixedVertices.empty()) {
          entries.simulation.fixedVertices = fixedVertices;
          entries.simulation.strFixedVertices = "";
          for (gsl::index vertexIndex : fixedVertices) {
            const bool vertexIndexIsInBounds =
                vertexIndex >= 0 && vertexIndex < edgeMesh.VN();
            Expects(vertexIndexIsInBounds);
            entries.simulation.strFixedVertices +=
                std::to_string(vertexIndex) + ",";
          }
          if (!fixedVertices.empty()) {
            entries.simulation.strFixedVertices.erase(
                entries.simulation.strFixedVertices.end() - 1);
          }
          shouldDrawFixedVertices = true;
        }
        // Parse nodal forces and draw them
        std::vector<NodalForce> nodalForces;
        ConfigurationFile::getNodalForces(jsonFilepath, nodalForces);
        if (!nodalForces.empty()) {
          entries.simulation.nodalForces = nodalForces;
          shouldDrawNodalForces = true;
        }

        updateViewer();
      }

      ImGui::SameLine(0, p);

      if (ImGui::Button("Save", ImVec2((w - p) / 2.f, 0))) {
        const std::string jsonFilepath = igl::file_dialog_save();
        if (jsonFilepath.empty()) {
          return;
        }
        if (!Utilities::hasExtension(jsonFilepath, ".json")) {
          std::cerr << "File " + jsonFilepath +
                           " has not a json extension. Can't load workspace."
                    << std::endl;
          return;
        }

        // Fill a json struct with the simulation scenario
        ConfigurationFile::SimulationScenario scenario{
            entries.plyFilename, entries.simulation.fixedVertices,
            entries.simulation.nodalForces};
        nlohmann::json jsonFile(scenario);
        // Export the contents of the json struct into a file
        std::ofstream file(jsonFilepath);
        file << jsonFile;
      }

      if (ImGui::Button("Clear", ImVec2(-1, 0))) {
        clearViewer();
      }
    }

    // Edge mesh tab
    if (ImGui::CollapsingHeader("Edge Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Load##Edge Mesh", ImVec2((w - p) / 2.f, 0))) {
        const std::string meshFilenameString = igl::file_dialog_open();
        if (loadEdgeMesh(meshFilenameString)) {
          clearViewer();
          entries.shouldDrawEdgeMesh = true;
          shouldDrawEdgeMesh = true;
          updateViewer();
        }
      }
      //      ImGui::SameLine(0, p);
      // Saves the displaced mesh
      //      if (ImGui::Button("Save##Edge Mesh", ImVec2((w - p) / 2.f, 0))) {
      //        const std::string meshFilenameString = igl::file_dialog_open();
      //        if (loadEdgeMesh(meshFilenameString)) {
      //          entries.shouldDrawEdgeMesh = true;
      //          shouldDrawEdgeMesh = true;
      //          updateViewer();
      //        }
      //      }

      if (ImGui::Checkbox("Show Edge Mesh", &entries.shouldDrawEdgeMesh)) {
        shouldDrawEdgeMesh = entries.shouldDrawEdgeMesh;
        if (entries.shouldDrawEdgeMesh) {
          updateViewer();
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
      if (ImGui::Checkbox("Show axis",
                          &entries.viewingOptions.shouldDrawWorldAxis)) {
        if (entries.viewingOptions.shouldDrawWorldAxis) {
          drawWorldAxis();
        } else {
          viewer.setDrawingDataVisibility(drawingDataIDs.worldAxisID, false);
        }
      }
      // Choose force component
      static NodalForceComponent force =
          entries.viewingOptions.chosenForceComponent;
      ImGui::Combo("Force visualization", reinterpret_cast<int *>(&force),
                   "N\0Ty\0Tz\0Mx\0My\0Mz\0");
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
      if (ImGui::InputText("Fixed Vertices",
                           entries.simulation.strFixedVertices)) {
        if (edgeMesh.IsEmpty())
          return;
        entries.simulation.fixedVertices.clear();
        Utilities::parseIntegers(entries.simulation.strFixedVertices,
                                 entries.simulation.fixedVertices);
        if (viewer.hasDrawingData(drawingDataIDs.fixedNodesID)) {
          viewer.deleteDrawingData(drawingDataIDs.fixedNodesID);
        }
        shouldDrawFixedVertices = true;
        updateViewer();
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
          shouldDrawNodalForces = true;
          updateViewer();
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
void GUI::drawFixedVertices() {
  drawFixedVertices(entries.simulation.fixedVertices);
}
void GUI::drawFixedVertices(const std::vector<gsl::index> &vertices) {
  if (!vertices.empty()) {
    Eigen::MatrixX3d fixedNodePositions(vertices.size(), 3);
    for (gsl::index fixedNodeIndex = 0; fixedNodeIndex < vertices.size();
         fixedNodeIndex++) {
      gsl::index vertexIndex = vertices[fixedNodeIndex];
      const bool vertexExists = vertexIndex < edgeMesh.VN();
      if (!vertexExists) {
        std::cerr << "Vertex with index " + std::to_string(vertexIndex) +
                         " does not exist."
                  << std::endl;
        continue;
      }
      fixedNodePositions.row(fixedNodeIndex) =
          VCGTriMesh::convertToEigenVector(edgeMesh.vert[vertexIndex].cP());
    }

    drawer.markPositions(drawingDataIDs.fixedNodesID, fixedNodePositions,
                         viewer);
  }
}

void GUI::updateViewer() {
  if (shouldDrawEdgeMesh) {
    shouldDrawEdgeMesh = false;
    drawEdgeMesh();
  }
  if (shouldDrawFixedVertices) {
    shouldDrawFixedVertices = false;
    drawFixedVertices();
  }
  if (shouldDrawNodalForces) {
    shouldDrawNodalForces = false;
    drawNodalForces();
  }
}

void GUI::drawWorldAxis() {
  if (viewer.hasDrawingData(drawingDataIDs.worldAxisID)) {
    viewer.setDrawingDataVisibility(drawingDataIDs.worldAxisID, true);
  } else {
    drawer.drawWorldAxis(drawingDataIDs.worldAxisID, viewer, true);
  }
}

void GUI::clearViewer() {
  viewer.clearDrawingData();
  entries.simulation.clear();
  if (entries.viewingOptions.shouldDrawWorldAxis) {
    drawWorldAxis();
  }
}

bool GUI::loadEdgeMesh(const std::string &meshFilenameString) {
  if (!Utilities::hasExtension(meshFilenameString, ".ply") ||
      !std::filesystem::exists(meshFilenameString)) {
    return false;
  }
  entries.plyFilename = meshFilenameString;
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
  SimulationJob simulationJob{nodes,
                              elements,
                              elementNormals,
                              fixedNodes,
                              entries.simulation.nodalForces,
                              beamDimensions,
                              beamMaterial};
  simulator.setSimulation(simulationJob);

  const std::filesystem::path resultsFolderPath =
      std::filesystem::current_path().append("Results");
  std::cout << resultsFolderPath.string() << std::endl;
  std::filesystem::create_directory(resultsFolderPath);
  simulator.setResultsNodalDisplacementCSVFilepath(
      std::filesystem::path(resultsFolderPath)
          .append("nodal_displacements.csv")
          .string());
  simulator.setResultsElementalForcesCSVFilepath(
      std::filesystem::path(resultsFolderPath)
          .append("elemental_displacements.csv")
          .string());
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
  Expects(!edgeMesh.IsEmpty());
  if (viewer.hasDrawingData(drawingDataIDs.displacedEdgeMeshID)) {
    viewer.deleteDrawingData(drawingDataIDs.displacedEdgeMeshID);
  }
  setSimulation();

  const auto simulationResults = simulator.executeSimulation();

  drawSimulationResults(simulationResults);
}

void GUI::drawSimulationResults(const SimulationResults &simulationResults) {
  drawDisplacedEdgeMesh(simulationResults.nodalDisplacements);
  drawEdgeForces(simulationResults.edgeForces);
}

void GUI::addNodalForce() {
  Expects(entries.simulation.force.vertexIndex >= 0 &&
          entries.simulation.force.vertexIndex < edgeMesh.VN() &&
          entries.simulation.force.dof < 6 &&
          entries.simulation.force.dof >= 0);
  NodalForce force{entries.simulation.force.vertexIndex,
                   entries.simulation.force.dof,
                   static_cast<double>(entries.simulation.force.magnitude)};
  entries.simulation.nodalForces.push_back(force);
  std::cout << "Force <" << force.dof << "," << force.index << ","
            << force.magnitude << "> was added." << std::endl;
}

void GUI::drawNodalForces() {
  if (viewer.hasDrawingData(drawingDataIDs.nodalForcesID)) {
    viewer.deleteDrawingData(drawingDataIDs.nodalForcesID);
  }

  // Combine all dof for the same vertices into a signle force vector
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

void GUI::drawDisplacedEdgeMesh(const Eigen::MatrixXd &nodalDisplacements) {
  Eigen::MatrixX3d nodes = edgeMesh.getEigenVertices();
  Eigen::MatrixX3d eigenNodalDisplacementsXYZ(nodes.rows(), 3);
  eigenNodalDisplacementsXYZ.col(0) = nodalDisplacements.col(0);
  eigenNodalDisplacementsXYZ.col(1) = nodalDisplacements.col(1);
  eigenNodalDisplacementsXYZ.col(2) = nodalDisplacements.col(2);
  Expects(eigenNodalDisplacementsXYZ.rows() == nodes.rows());
  const Eigen::MatrixX3d displacedVertices = eigenNodalDisplacementsXYZ + nodes;
  Eigen::MatrixX2i elements = edgeMesh.getEigenEdges();
  drawer.drawEdges(drawingDataIDs.displacedEdgeMeshID, displacedVertices,
                   elements, edgeMesh.getEigenEdgeNormals(),
                   edgeMesh.getBeamDimensions(), viewer);
}

void GUI::drawEdgeForces(const std::vector<Eigen::VectorXd> &edgeForces) {
  // Compute the color of each edge.
  std::vector<Eigen::MatrixXd> edgeColors;
  convertToColors(edgeForces, edgeColors);
  Eigen::MatrixXd &beamMeshVertices =
      viewer.getDrawingData(drawingDataIDs.displacedEdgeMeshID).V;
  // Compute the color of each beam.
  drawer.computeBeamColors(edgeMesh.getEigenVertices(),
                           edgeMesh.getEigenEdges(), edgeColors,
                           beamMeshVertices);
  drawer.setBeamColors(drawingDataIDs.displacedEdgeMeshID,
                       entries.viewingOptions.chosenForceComponent, viewer);
  for (gsl::index fc = Fx; fc < NodalForceComponent::NumberOfForceComponents;
       fc++) {
    colormaps.push_back(
        Colormap{edgeForces[fc], entries.viewingOptions.chosenColormapType});
  }
}

void GUI::convertToEigen(
    const std::vector<std::vector<double>> &forcesPerEdgePerComponent,
    std::vector<Eigen::VectorXd> &forcesPerComponentPerEdge) {}

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
  Eigen::Vector3f backgroundColor(0.3, 0.3, 0.5);
  backgroundColor *= 255;
  colormaps[entries.viewingOptions.chosenForceComponent].drawColorbar(
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

    colormaps.clear();
    for (gsl::index fc = Fx; fc < NodalForceComponent::NumberOfForceComponents;
         fc++) {
      colormaps.push_back(Colormap{simulator.getResults().edgeForces[fc],
                                   entries.viewingOptions.chosenColormapType});
    }
    drawEdgeForces(simulator.getResults().edgeForces);
  }
}
