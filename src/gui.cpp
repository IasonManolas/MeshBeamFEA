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
  Drawer drawer;
  //  drawer.draw(viewer.iglViewer);
  viewer.launch();
}

GUI::~GUI() {}

void GUI::createMenu() {
  Menu *pMenu = new Menu;
  pMenu->callback_draw_viewer_menu = [&]() {
    if (ImGui::CollapsingHeader("Edge Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
      //      static bool shouldDrawDisplacedMesh = false;
      //      if (ImGui::Checkbox("Show displaced mesh",
      //      &shouldDrawDisplacedMesh)) {
      //        viewer.data_list[DataIndices::DisplacedMesh].set_visible(
      //            shouldDrawDisplacedMesh);
      //      }
      float w = ImGui::GetContentRegionAvailWidth();
      float p = ImGui::GetStyle().FramePadding.x;
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
        // viewer.data_list[DataIndices::BeamMesh].set_visible(shouldDrawBeamMesh);
        // viewer.data_list[DataIndices::BeamMesh].set_colors(
        //    beamVerticesColors[static_cast<size_t>(currentlyChosenForce)]);
      }
    }

    //     //Workspace
    //    if (ImGui::CollapsingHeader("Workspace",
    //    ImGuiTreeNodeFlags_DefaultOpen)) {
    //      float w = ImGui::GetContentRegionAvailWidth();
    //      float p = ImGui::GetStyle().FramePadding.x;
    //      if (ImGui::Button("Load##Workspace", ImVec2((w - p) / 2.f, 0)))
    //      {
    //        viewer.load_scene();
    //      }
    //      ImGui::SameLine(0, p);
    //      if (ImGui::Button("Save##Workspace", ImVec2((w - p) / 2.f, 0)))
    //      {
    //        viewer.save_scene();
    //      }
    //    }

    //        Eigen::MatrixX3d vertices;
    //        originalMesh.getVertices(vertices);
    //        Eigen::VectorXi edges;
    //        originalMesh.getEdges(edges);
    //        Eigen::MatrixX3d vertexNormals;
    //        originalMesh.getNormals(vertexNormals);

    //        simulator.setSimulation(vertices, edges, vertexNormals,
    //        nodalForces); initializeDrawer();
    //        simulator.setMesh(originalMesh);
    //                ImGui::SameLine(0, p);
    //                if (ImGui::Button("Save##Mesh", ImVec2((w - p)
    //                / 2.f, 0))) {
    //                  viewer.open_dialog_save_mesh();
    //                }
    //      }

    //    // Viewing options
    if (ImGui::CollapsingHeader("Viewing Options",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
      //      if (ImGui::Button("Center object", ImVec2(-1, 0))) {
      //        viewer.core().align_camera_center(viewer.data().V,
      //        viewer.data().F);
      //      }
      //      if (ImGui::Button("Snap canonical view", ImVec2(-1, 0))) {
      //        viewer.snap_to_canonical_quaternion();
      //      }
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

      float w = ImGui::GetContentRegionAvailWidth();
      float p = ImGui::GetStyle().FramePadding.x;
      if (ImGui::Button("Clear", ImVec2((w - p) / 2.f, 0))) {
        viewer.clearDrawingData();
        entries.simulation.nodalForces.clear();
      }
      static NodalForceComponent force =
          entries.viewingOptions.chosenForceComponent;
      ImGui::Combo("Force visualization", reinterpret_cast<int *>(&force),
                   "N\0Ty\0Tx\0Mx\0My\0Mz\0");
      if (force != entries.viewingOptions.chosenForceComponent) {
        entries.viewingOptions.chosenForceComponent = force;
        drawer.setEdgeColors(drawingDataIDs.displacedEdgeMeshID,
                             entries.simulation.b * entries.simulation.h,
                             entries.viewingOptions.chosenForceComponent,
                             viewer);
      }
    }
    drawColorTypeCombo();
    if (viewer.hasDrawingData(drawingDataIDs.displacedEdgeMeshID)) {
      drawColorbar();
    }

    //      // Zoom
    //      ImGui::PushItemWidth(80 * pMenu->menu_scaling());
    //      ImGui::DragFloat("Zoom", &(viewer.core().camera_zoom), 0.05f,
    //      0.1f,
    //                       20.0f);

    //      // Select rotation type
    //      int rotation_type =
    //      static_cast<int>(viewer.core().rotation_type); static
    //      Eigen::Quaternionf trackball_angle =
    //          Eigen::Quaternionf::Identity();
    //      static bool orthographic = true;
    //      if (ImGui::Combo("Camera Type", &rotation_type,
    //                       "Trackball\0Two Axes\0002D Mode\0\0")) {
    //        using RT = igl::opengl::ViewerCore::RotationType;
    //        auto new_type = static_cast<RT>(rotation_type);
    //        if (new_type != viewer.core().rotation_type) {
    //          if (new_type == RT::ROTATION_TYPE_NO_ROTATION) {
    //            trackball_angle = viewer.core().trackball_angle;
    //            orthographic = viewer.core().orthographic;
    //            viewer.core().trackball_angle =
    //            Eigen::Quaternionf::Identity();
    //            viewer.core().orthographic = true;
    //          } else if (viewer.core().rotation_type ==
    //                     RT::ROTATION_TYPE_NO_ROTATION) {
    //            viewer.core().trackball_angle = trackball_angle;
    //            viewer.core().orthographic = orthographic;
    //          }
    //          viewer.core().set_rotation_type(new_type);
    //        }

    //      // Orthographic view
    //      ImGui::Checkbox("Orthographic view",
    //      &(viewer.core().orthographic)); ImGui::PopItemWidth();
    //    }

    //    // Helper for setting viewport specific mesh options
    //    auto make_checkbox = [&](const char *label, unsigned int &option)
    //    {
    //      return ImGui::Checkbox(
    //          label, [&]() { return viewer.core().is_set(option); },
    //          [&](bool value) { return viewer.core().set(option, value);
    //          });
    //    };

    //    // Draw options
    //    if (ImGui::CollapsingHeader("Draw Options",
    //                                ImGuiTreeNodeFlags_DefaultOpen)) {
    //      if (ImGui::Checkbox("Face-based", &(viewer.data().face_based)))
    //      {
    //        viewer.data().dirty = igl::opengl::MeshGL::DIRTY_ALL;
    //      }
    //      make_checkbox("Show texture", viewer.data().show_texture);
    //      if (ImGui::Checkbox("Invert normals",
    //      &(viewer.data().invert_normals))) {
    //        viewer.data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
    //      }
    //      make_checkbox("Show overlay", viewer.data().show_overlay);
    //      make_checkbox("Show overlay depth",
    //      viewer.data().show_overlay_depth);
    //      ImGui::ColorEdit4("Background",
    //      viewer.core().background_color.data(),
    //                        ImGuiColorEditFlags_NoInputs |
    //                            ImGuiColorEditFlags_PickerHueWheel);
    //      ImGui::ColorEdit4("Line color", viewer.data().line_color.data(),
    //                        ImGuiColorEditFlags_NoInputs |
    //                            ImGuiColorEditFlags_PickerHueWheel);
    //      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
    //      ImGui::DragFloat("Shininess", &(viewer.data().shininess), 0.05f,
    //      0.0f,
    //                       100.0f);
    //      ImGui::PopItemWidth();
    //    }

    //    // Overlays
    //    if (ImGui::CollapsingHeader("Overlays",
    //    ImGuiTreeNodeFlags_DefaultOpen)) {
    //      make_checkbox("Wireframe", viewer.data().show_lines);
    //      make_checkbox("Fill", viewer.data().show_faces);
    //      ImGui::Checkbox("Show vertex labels",
    //      &(viewer.data().show_vertid)); ImGui::Checkbox("Show faces
    //      labels", &(viewer.data().show_faceid));
    //    }

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      const float w = ImGui::GetContentRegionAvailWidth();
      const float p = ImGui::GetStyle().FramePadding.x;
      if (ImGui::Button("Execute \n Simulation", ImVec2((w - p) / 2.f, 0))) {
        if (edgeMesh.IsEmpty())
          return;
        executeSimulation();
      }
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
          for (int fixedNodeIndex = 0;
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

      if (ImGui::CollapsingHeader("Nodal Force",
                                  ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::InputInt("Vertex Index",
                            &entries.simulation.force.vertexIndex)) {
          if (edgeMesh.IsEmpty())
            return;
          if (entries.simulation.force.vertexIndex < 0) {
            std::cerr << "Only positive values can be used as vertex indices."
                      << std::endl;
          }
        }
        if (ImGui::InputInt("Force DoF", &entries.simulation.force.dof)) {
          if (edgeMesh.IsEmpty())
            return;
          if (entries.simulation.force.dof > 5 ||
              entries.simulation.force.dof < 0) {
            std::cerr << "DoF must be in the range [0,5]" << std::endl;
          }
        }
        if (ImGui::InputFloat("Force Magnitude",
                              &entries.simulation.force.magnitude)) {
        }

        if (ImGui::Button("Add nodal \n force", ImVec2((w - p) / 2.f, 0))) {
          if (edgeMesh.IsEmpty())
            return;
          addNodalForce();
          drawNodalForces();
        }
        if (ImGui::Button("Clear nodal \n forces", ImVec2((w - p) / 2.f, 0))) {
          if (edgeMesh.IsEmpty())
            return;
          entries.simulation.nodalForces.clear();
          viewer.deleteDrawingData(drawingDataIDs.nodalForcesID);
        }
      }
      //    if (ImGui::CollapsingHeader("Results",
      //    ImGuiTreeNodeFlags_DefaultOpen)) {
    }
  };
  viewer.addPlugin("Main menu", pMenu);
}

bool GUI::loadEdgeMesh() {
  const std::string meshFilenameString = igl::file_dialog_open();
  //      "/home/iason/Models/Line segments/lineSegments.ply";
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
  if (viewer.hasDrawingData(drawingDataIDs.beamForcesID)) {
    viewer.deleteDrawingData(drawingDataIDs.beamForcesID);
  }
  setSimulation();

  const auto simulationResults = simulator.executeSimulation();

  drawDisplacedEdgeMesh(simulationResults.nodal_displacements);

  drawEdgeForces(simulationResults.element_forces);
}

void GUI::addNodalForce() {
  if (entries.simulation.force.vertexIndex >= 0 &&
      entries.simulation.force.dof < 6 && entries.simulation.force.dof >= 0) {
    NodalForce force;
    force.dof = entries.simulation.force.dof;
    force.index = static_cast<size_t>(entries.simulation.force.vertexIndex);
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

  std::unordered_map<int, Eigen::Vector3d> nodeIndexForceMap;
  for (int nodeIndex = 0; nodeIndex < entries.simulation.nodalForces.size();
       nodeIndex++) {
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
  int forceIndex = 0;
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
    const int &nodeIndex = it->first;
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
  if (entries.simulation.b * entries.simulation.h > 0) {
    Eigen::MatrixXd &beamMeshVertices =
        viewer.getDrawingData(drawingDataIDs.displacedEdgeMeshID).V;
    //    assert(beamMeshVertices.rows() == numberOfVerticesPerBeam *
    //    edgeMesh.EN());
    drawer.computeBeamColors(edgeMesh.getEigenVertices(),
                             edgeMesh.getEigenEdges(), edgeColors,
                             beamMeshVertices);
    drawer.setEdgeColors(drawingDataIDs.displacedEdgeMeshID,
                         entries.simulation.b * entries.simulation.h,
                         entries.viewingOptions.chosenForceComponent, viewer);
  }
  colorbar.init_colormaps(eigenEdgeForces,
                          entries.viewingOptions.chosenColormapType);
}

void GUI::convertToEigen(
    const std::vector<std::vector<double>> &forcesPerEdgePerComponent,
    std::vector<Eigen::VectorXd> &forcesPerComponentPerEdge) {

  // Convert to vector of eigen matrices of the form force component-> per
  // Edge
  // force value
  const int numDof = 6;
  const size_t numberOfEdges = forcesPerEdgePerComponent.size();
  forcesPerComponentPerEdge =
      std::vector<Eigen::VectorXd>(numDof, Eigen::VectorXd(2 * numberOfEdges));
  for (int edgeIndex = 0; edgeIndex < numberOfEdges; edgeIndex++) {
    for (int forceComponentIndex = NodalForceComponent::Fx;
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
  for (int forceComponentIndex = NodalForceComponent::Fx;
       forceComponentIndex < numDof; forceComponentIndex++) {
    igl::colormap(entries.viewingOptions.chosenColormapType,
                  edgeForces[forceComponentIndex], true,
                  edgeColors[forceComponentIndex]);
  }
}

void GUI::drawColorbar() {
  //  ColorbarPlugin colorbar;
  //  igl::ColorMapType newColormapType =
  //      static_cast<igl::ColorMapType>(colorbar.draw_colormap_combo());
  //  if (newColormapType != colormapType) {
  //    colormapType = newColormapType;
  //    drawNodalForces();
  //  }
  Eigen::Vector4f backgroundColor(0.3f, 0.3f, 0.5f, 1.0f);
  backgroundColor *= 255;
  colorbar.draw_colorbar(
      entries.viewingOptions.chosenForceComponent,
      minMaxForcesPerForceComponent[entries.viewingOptions.chosenForceComponent]
          .first,
      minMaxForcesPerForceComponent[entries.viewingOptions.chosenForceComponent]
          .second,
      backgroundColor);
}

void GUI::drawColorTypeCombo() {
  const char *items[]{
      "Inferno", "Jet", "Magma", "Parula", "Plasma", "Viridis",
  };
  static int selected_index = entries.viewingOptions.chosenColormapType;
  static const char *current_item = items[selected_index];
  ImVec2 combo_pos = ImGui::GetCursorScreenPos();
  if (ImGui::BeginCombo("Coloring Type##combo", "")) {
    for (int n = 0; n < IM_ARRAYSIZE(items); ++n) {
      // You can store your selection however you want, outside or inside your
      // objects
      bool is_selected = (current_item == items[n]);
      ImGui::PushID(n);
      if (ImGui::Selectable("", is_selected)) {
        current_item = items[n];
        selected_index = n;
      }
      ImGui::SameLine(0, 0);
      //      ImGui::Image(
      //          reinterpret_cast<ImTextureID>(colormaps_[n]),
      //          ImVec2(ImGui::GetTextLineHeight(),
      //          ImGui::GetTextLineHeight()), ImVec2(0, 0), ImVec2(1, 1));
      //      ImGui::SameLine();
      ImGui::Text("%s", items[n]);
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
  //  ImGui::Image(reinterpret_cast<ImTextureID>(colormaps_[selected_index]),
  //               ImVec2(h, h));
  //  ImGui::SameLine();
  ImGui::Text("%s", current_item);
  ImGui::SetCursorScreenPos(backup_pos);

  if (selected_index != entries.viewingOptions.chosenColormapType) {
    entries.viewingOptions.chosenColormapType =
        static_cast<igl::ColorMapType>(selected_index);
    std::vector<Eigen::VectorXd> eigenEdgeForces;
    convertToEigen(simulator.getResults().element_forces, eigenEdgeForces);
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
    colorbar.init_colormaps(eigenEdgeForces,
                            entries.viewingOptions.chosenColormapType);

    if (entries.simulation.b * entries.simulation.h > 0) {
      Eigen::MatrixXd &beamMeshVertices =
          viewer.getDrawingData(drawingDataIDs.displacedEdgeMeshID).V;
      drawer.computeBeamColors(edgeMesh.getEigenVertices(),
                               edgeMesh.getEigenEdges(), edgeColors,
                               beamMeshVertices);
      drawer.setEdgeColors(drawingDataIDs.displacedEdgeMeshID,
                           entries.simulation.b * entries.simulation.h,
                           entries.viewingOptions.chosenForceComponent, viewer);
    }
  }
}

void ColorbarPlugin::init_colormaps(std::vector<Eigen::VectorXd> vectorOfValues,
                                    const igl::ColorMapType &colormapType) {
  colormaps_.clear();
  for (Eigen::VectorXd &values : vectorOfValues) {
    std::sort(values.data(), values.data() + values.size());
    Eigen::MatrixX3d rgbColors;
    Eigen::MatrixXd valuesMatrix(values);
    igl::colormap(colormapType, valuesMatrix, true, rgbColors);
    GLuint id = 0;
    texture_from_colormap(rgbColors, id);
    colormaps_.push_back(id);
  }
}

// Draws a combo box for selecting the colormap
int ColorbarPlugin::draw_colormap_combo() const {}

// Draws the actual colorbar with min/max values
void ColorbarPlugin::draw_colorbar(
    const int colormapIndex, float xmin, float xmax,
    const Eigen::Vector4f &background_color) const {
  ImVec4 color(0, 0, 0, 1);
  auto rgb = background_color;
  // http://stackoverflow.com/a/3943023/112731
  if (rgb[0] * 0.299 + rgb[1] * 0.587 + rgb[2] * 0.114 <= 186) {
    color = ImVec4(1, 1, 1, 1);
  }
  float w = 100;
  float h = 20;
  ImGui::BeginGroup();
  ImGui::BeginGroup();
  ImGui::Image(reinterpret_cast<ImTextureID>(colormaps_[colormapIndex]),
               ImVec2(w, h));
  ImGui::EndGroup();
  ImGui::TextColored(color, "%.3g", xmin);
  ImGui::SameLine();
  ImGui::Dummy(ImVec2(w - 40, 0)); // TODO: -40 is wrong since the space depends
                                   // of the digits of min and max
  ImGui::SameLine();
  ImGui::TextColored(color, "%.3g", xmax);
  ImGui::EndGroup();
}