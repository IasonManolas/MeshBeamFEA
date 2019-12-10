#ifndef DRAWER_HPP
#define DRAWER_HPP

#include "mesh.hpp"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

class SimulationResultsDrawer {
  const VCGMesh &vcgMesh;
  const std::vector<std::vector<double>> &vertexDisplacements;
  using VerticesColors = std::vector<Eigen::MatrixX3d>;
  VerticesColors verticesColors;

  void computeVerticesColors(
      const std::vector<std::vector<double>> &vertexDataValues);
  static void vcgToEigenMesh(const VCGMesh &vcgMesh, Eigen::MatrixX3d &vertices,
                             Eigen::MatrixX3i &triFaces);
  void addMesh(const Eigen::MatrixX3d &vertices,
               const Eigen::MatrixX3i &triFaces, const bool shouldBeVisible,
               igl::opengl::glfw::Viewer &viewer) const;

  void createMenu(igl::opengl::glfw::Viewer &viewer,
                  igl::opengl::glfw::imgui::ImGuiMenu &menu) const;
  void computeDisplacedVertices(const Eigen::MatrixX3d &vertices,
                                Eigen::MatrixX3d &displacedVertices,
                                const float displacementFactor) const;

public:
  SimulationResultsDrawer(const VCGMesh &m,
                          std::vector<std::vector<double>> &vDisplacements,
                          std::vector<std::vector<double>> &vForces);
  void draw() const;
};

#endif // DRAWER_HPP
