#ifndef DRAWER_HPP
#define DRAWER_HPP

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
  void computeBeamColors(
      const std::vector<Eigen::MatrixXd> &edgeColors,
      const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
          &edgesNodes,
      const Eigen::MatrixX3d &beamMeshVertices);
  void computeBeamColors(const Eigen::Vector3d &edgeVertex1,
                         const Eigen::Vector3d &edgeVertex2,
                         const Eigen::MatrixX3d &beamVertices,
                         Eigen::MatrixX3d &beamVerticesColors) const;
  void computeBeamColors(const Eigen::MatrixX3d &edgeVertices,
                         const Eigen::MatrixX3d &beamVertices,
                         const Eigen::MatrixXd &edgeVerticesColors,
                         Eigen::MatrixX3d beamVerticesColors) const;
  static void vcgToEigenMesh(const VCGTriMesh &vcgMesh,
                             Eigen::MatrixX3d &vertices,
                             Eigen::MatrixX3i &triFaces);
  static void computeDisplacedVertices(
      const Eigen::MatrixX3d &vertices,
      const std::vector<std::vector<double>> vectorDisplacements,
      Eigen::MatrixX3d &displacedVertices, const float displacementFactor = 1);

  static void extractEdgeNodes(
      const VCGTriMesh &mesh, const Eigen::MatrixX3d &vertices,
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edgeNodes);

  void getBeamMesh(const VCGTriMesh &vcgMesh, const Eigen::MatrixX3d &vertices,
                   IGLMesh &beamMesh);
  void getBeamMesh(const VCGEdgeMesh &edgeNodes, const float &beamThickness,
                   IGLMesh &beamMesh) const;
  void addDrawingData(Viewer &viewer, const std::string &dataIdentifier,
                      const ViewerData &drawingData);
  void drawMesh(const std::string meshIdentifier,
                const Eigen::MatrixX3d &vertices,
                const Eigen::MatrixX3i &triFaces, Viewer &viewer) const;

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
  // Drawer(const VCGMesh &m,
  //       const std::vector<std::vector<double>> &vDisplacements,
  //       const std::vector<std::vector<double>> &edgeForces);
  Drawer();
  void drawWorldAxis(const std::string &drawingDataID, Viewer &viewer,
                     const double &axisLength = 1);
  void drawBeamMesh(const std::string drawingDataID,
                    const VCGEdgeMesh &edgeMesh, const float &beamThickness,
                    Viewer &viewer) const;
  void setVertices(const std::string drawingDataID,
                   const Eigen::MatrixX3d &newPositions, Viewer &viewer) const;
  void setEdgePositions(const std::string &drawingDataID,
                        const Eigen::MatrixX3d &newPositions,
                        Viewer &viewer) const;
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
  void setBeamColors(const std::string &drawingDataID, const int &colorIndex,
                     Viewer &viewer) const;
};

#endif // DRAWER_HPP
