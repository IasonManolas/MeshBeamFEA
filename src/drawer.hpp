#ifndef DRAWER_HPP
#define DRAWER_HPP

#include "edgemesh.hpp"
#include "mesh.hpp"
#include "viewer.hpp"
#include <stdexcept>

enum NodalForceComponent {
  Fx = 0,
  Fy,
  Fz,
  Mx,
  My,
  Mz,
  NumberOfForceComponents
};

class Drawer {
  float edgeThicknessEpsilon{0.0000005};
  std::vector<Eigen::MatrixX3d> beamVerticesColors;

private:
  void generateDrawingData(const Eigen::MatrixX3d &edgeStartPoints,
                           const Eigen::MatrixX3d &edgeEndPoints,
                           const Eigen::MatrixX3d &colors,
                           ViewerData &drawingData) const;
  void generateDrawingData(const Eigen::MatrixX3d &vertices,
                           const Eigen::MatrixX3i &triangleFaces,
                           ViewerData &drawingData) const;

  void generateDrawingData(const Eigen::MatrixX3d &vertices,
                           const Eigen::MatrixX2i &edges,
                           ViewerData &drawingData) const;

  void generateDrawingData(const Eigen::MatrixX3d &vertices,
                           const Eigen::MatrixX3i &triangleFaces,
                           const Eigen::RowVector3d &color,
                           ViewerData &drawingData) const;

public:
  Drawer();
  void drawWorldAxis(const std::string &drawingDataID, Viewer &viewer,
                     const double &axisLength = 1);
  void drawBeamMesh(const std::string drawingDataID,
                    const VCGEdgeMesh &edgeMesh, const float &beamThickness,
                    Viewer &viewer) const;
  void setVertices(const std::string drawingDataID,
                   const Eigen::MatrixX3d &newPositions, Viewer &viewer) const;
  void drawEdges(const std::string drawingDataID,
                 const Eigen::MatrixX3d &vertices, const Eigen::MatrixX2i edges,
                 const Eigen::MatrixX3d &edgeNormals,
                 const std::vector<BeamDimensions> &beamDimensions,
                 Viewer &viewer) const;
  void markPositions(const std::string drawingDataID,
                     const Eigen::MatrixX3d &spherePositions,
                     Viewer &viewer) const;
  void drawEdges(const string &drawingDataID,
                 const Eigen::MatrixX3d &edgeStartPoints,
                 const Eigen::MatrixX3d &edgeEndPoints,
                 const Eigen::MatrixX3d &colors, Viewer &viewer) const;
  void computeBeamColors(const Eigen::MatrixX3d &edgeVertices,
                         const Eigen::MatrixX2i &edges,
                         const std::vector<Eigen::MatrixXd> &edgeColors,
                         const Eigen::MatrixX3d &beamMeshVertices);
  void setBeamColors(const std::string &drawingDataID,
                     const gsl::index &colorIndex, Viewer &viewer) const;
};

#endif // DRAWER_HPP
