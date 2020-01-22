#include "drawer.hpp"
#include <Eigen/Geometry>
#include <igl/combine.h>
#include <igl/jet.h>
#include <igl/list_to_matrix.h>
#include <vcg/complex/algorithms/create/platonic.h>

void Drawer::generateDrawingData(const Eigen::MatrixX3d &vertices,
                                 const Eigen::MatrixX3i &triangleFaces,
                                 ViewerData &drawingData) const {
  drawingData.set_mesh(vertices, triangleFaces);
  drawingData.compute_normals();
}

void Drawer::generateDrawingData(const Eigen::MatrixX3d &vertices,
                                 const Eigen::MatrixX3i &triangleFaces,
                                 const Eigen::RowVector3d &color,
                                 ViewerData &drawingData) const {
  generateDrawingData(vertices, triangleFaces, drawingData);
  drawingData.set_colors(color);
}

void Drawer::generateDrawingData(const Eigen::MatrixX3d &edgeStartPoints,
                                 const Eigen::MatrixX3d &edgeEndPoints,
                                 const Eigen::MatrixX3d &colors,
                                 ViewerData &drawingData) const {
  drawingData.add_edges(edgeStartPoints, edgeEndPoints, colors);
}

void Drawer::generateDrawingData(const Eigen::MatrixX3d &vertices,
                                 const Eigen::MatrixX2i &edges,
                                 ViewerData &drawingData) const {
  Eigen::MatrixX3d C(1, 3);
  C.row(0) = Eigen::Vector3d{1, 0, 0};
  drawingData.set_edges(vertices, edges, C);
}

void Drawer::drawEdges(const std::string drawingDataID,
                       const Eigen::MatrixX3d &vertices,
                       const Eigen::MatrixX2i edges,
                       const Eigen::MatrixX3d &edgeNormals,
                       const std::vector<BeamDimensions> &beamDimensions,
                       Viewer &viewer) const {
  assert(!viewer.hasDrawingData(drawingDataID));
  const bool drawEdgesAsBeams = true;
  if (drawEdgesAsBeams) {
    VCGTriMesh beamMesh;
    VCGTriMesh::constructBeamMesh(vertices, edges, edgeNormals, beamDimensions,
                                  beamMesh);
    const int verticesPerBeam = beamMesh.VN() / (edges.rows());
    IGLMesh iglBeamMesh = beamMesh.getIGLMesh();
    assert(iglBeamMesh.vertices.rows() == verticesPerBeam * edges.rows());
    ViewerData drawingData;
    generateDrawingData(iglBeamMesh.vertices, iglBeamMesh.triFaces,
                        drawingData);
    viewer.addDrawingData(drawingDataID, drawingData);
    viewer.setDrawingDataVisibility(drawingDataID, true);
  } else {
    ViewerData drawingData;
    generateDrawingData(vertices, edges, drawingData);
    viewer.addDrawingData(drawingDataID, drawingData);
  }
}

void Drawer::markPositions(const string drawingDataID,
                           const Eigen::MatrixX3d &spherePositions,
                           Viewer &viewer) const {
  assert(!viewer.hasDrawingData(drawingDataID));
  VCGTriMesh squares;
  for (int sphereIndex = 0; sphereIndex < spherePositions.rows();
       sphereIndex++) {
    Eigen::Vector3d position = spherePositions.row(sphereIndex);
    VCGTriMesh square;
    vcg::tri::Icosahedron<VCGTriMesh>(square);
    vcg::tri::UpdatePosition<VCGTriMesh>::Scale(square, 0.01);
    vcg::tri::UpdatePosition<VCGTriMesh>::Translate(
        square, VCGTriMesh::convertToVCGPoint(position));
    vcg::tri::Append<VCGTriMesh, VCGTriMesh>::Mesh(squares, square);
  }
  IGLMesh iglSquares = squares.getIGLMesh();
  ViewerData drawingData;
  Eigen::RowVector3d redColor(1, 0, 0);
  generateDrawingData(iglSquares.vertices, iglSquares.triFaces, redColor,
                      drawingData);
  viewer.addDrawingData(drawingDataID, drawingData);
}

void Drawer::drawWorldAxis(const std::string &drawingDataID, Viewer &viewer,
                           const double &axisLength /*=1*/) {
  assert(!viewer.hasDrawingData(drawingDataID));
  Eigen::MatrixXd startingPositions(3, 3);
  startingPositions.row(0) = Eigen::Vector3d(0, 0, 0);
  startingPositions.row(1) = Eigen::Vector3d(0, 0, 0);
  startingPositions.row(2) = Eigen::Vector3d(0, 0, 0);
  Eigen::MatrixXd axisColors(3, 3);
  Eigen::MatrixXd endingPositions(3, 3);
  endingPositions.row(0) = Eigen::Vector3d(axisLength, 0, 0);
  endingPositions.row(1) = Eigen::Vector3d(0, axisLength, 0);
  endingPositions.row(2) = Eigen::Vector3d(0, 0, axisLength);
  axisColors = endingPositions;
  ViewerData axisDrawingData;
  axisDrawingData.add_edges(startingPositions, endingPositions, axisColors);
  viewer.addDrawingData(drawingDataID, axisDrawingData);
}

void Drawer::setBeamColors(const string &drawingDataID, const int &colorIndex,
                           Viewer &viewer) const {
  assert(!beamVerticesColors.empty());
  viewer.setColors(drawingDataID,
                   beamVerticesColors[static_cast<size_t>(colorIndex)]);
}

void Drawer::drawEdges(const string &drawingDataID,
                       const Eigen::MatrixX3d &edgeStartPoints,
                       const Eigen::MatrixX3d &edgeEndPoints,
                       const Eigen::MatrixX3d &colors, Viewer &viewer) const {
  ViewerData drawingData;
  generateDrawingData(edgeStartPoints, edgeEndPoints, colors, drawingData);
  viewer.addDrawingData(drawingDataID, drawingData);
}

void Drawer::computeBeamColors(const Eigen::MatrixX3d &edgeVertices,
                               const Eigen::MatrixX2i &edges,
                               const std::vector<Eigen::MatrixXd> &edgeColors,
                               const Eigen::MatrixX3d &beamMeshVertices) {
  beamVerticesColors.clear();
  const int numDoF = 6;
  beamVerticesColors.resize(numDoF);
  const int numberOfEdges = edges.rows();
  const int numberOfVerticesPerBeam = beamMeshVertices.rows() / numberOfEdges;
  for (int forceComponentIndex = NodalForceComponent::Fx;
       forceComponentIndex < numDoF; forceComponentIndex++) {
    beamVerticesColors[forceComponentIndex].resize(beamMeshVertices.rows(), 3);
    for (int edgeIndex = 0; edgeIndex < numberOfEdges; edgeIndex++) {
      const int vertexIndex0 = edges(edgeIndex, 0);
      const int vertexIndex1 = edges(edgeIndex, 1);
      const Eigen::Vector3d &p0 = edgeVertices.row(vertexIndex0);
      const Eigen::Vector3d &p1 = edgeVertices.row(vertexIndex1);
      const Eigen::Vector3d &edgeV0Color =
          edgeColors[forceComponentIndex].row(2 * edgeIndex);
      const Eigen::Vector3d &edgeV1Color =
          edgeColors[forceComponentIndex].row(2 * edgeIndex + 1);
      for (int beamVertexIndex = 0; beamVertexIndex < numberOfVerticesPerBeam;
           beamVertexIndex++) {
        const Eigen::Vector3d &p = beamMeshVertices.row(
            numberOfVerticesPerBeam * edgeIndex + beamVertexIndex);
        const bool isOnTheSideOfP0 = Eigen::Vector3d(p - p0).squaredNorm() <
                                     Eigen::Vector3d(p - p1).squaredNorm();

        if (isOnTheSideOfP0) {
          beamVerticesColors[forceComponentIndex].row(
              numberOfVerticesPerBeam * edgeIndex + beamVertexIndex) =
              edgeV0Color;
        } else {
          beamVerticesColors[forceComponentIndex].row(
              numberOfVerticesPerBeam * edgeIndex + beamVertexIndex) =
              edgeV1Color;
        }
      }
    }
  }
}

void Drawer::drawBeamMesh(const std::string drawingDataID,
                          const VCGEdgeMesh &edgeMesh,
                          const float &edgeThickness, Viewer &viewer) const {
  if (edgeMesh.IsEmpty()) {
    cerr << "Error: No edge mesh is given. Beam mesh can't be rendered."
         << std::endl;
    return;
  }
  assert(!viewer.hasDrawingData(drawingDataID));
  VCGTriMesh beamMesh;
  edgeMesh.getBeamMesh(edgeThickness, beamMesh);
  IGLMesh iglBeamMesh = beamMesh.getIGLMesh();
  ViewerData drawingData;
  generateDrawingData(iglBeamMesh.vertices, iglBeamMesh.triFaces, drawingData);
  viewer.addDrawingData(drawingDataID, drawingData);
  viewer.setDrawingDataVisibility(drawingDataID, true);
}

void Drawer::setVertices(const std::string drawingDataID,
                         const Eigen::MatrixX3d &newPositions,
                         Viewer &viewer) const {
  ViewerData &drawingData = viewer.getDrawingData(drawingDataID);
  assert(drawingData.V.rows() == newPositions.rows());
  drawingData.set_vertices(newPositions);
}

Drawer::Drawer() {}
