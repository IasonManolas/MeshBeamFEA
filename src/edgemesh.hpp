#ifndef EDGEMESH_HPP
#define EDGEMESH_HPP
#include "mesh.hpp"
#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/nanoply/include/nanoplyWrapper.hpp>

class VCGEdgeMeshEdgeType;
class VCGEdgeMeshVertexType;

struct VCGEdgeMeshUsedTypes
    : public vcg::UsedTypes<vcg::Use<VCGEdgeMeshVertexType>::AsVertexType,
                            vcg::Use<VCGEdgeMeshEdgeType>::AsEdgeType> {};

class VCGEdgeMeshVertexType
    : public vcg::Vertex<VCGEdgeMeshUsedTypes, vcg::vertex::Coord3d,
                         vcg::vertex::Normal3d, vcg::vertex::BitFlags,
                         vcg::vertex::VEAdj> {};
class VCGEdgeMeshEdgeType
    : public vcg::Edge<VCGEdgeMeshUsedTypes, vcg::edge::VertexRef,
                       vcg::edge::BitFlags, vcg::edge::EEAdj,
                       vcg::edge::VEAdj> {};

class VCGEdgeMesh : public vcg::tri::TriMesh<std::vector<VCGEdgeMeshVertexType>,
                                             std::vector<VCGEdgeMeshEdgeType>> {
  const std::string plyPropertyEdgeNormalID{"edge_normal"};
  const std::string plyPropertyBeamDimensionsID{"beam_dimensions"};
  const std::string plyPropertyBeamMaterialID{"beam_material"};
  VCGEdgeMesh::PerEdgeAttributeHandle<vcg::Point3d> handleEdgeNormals;
  VCGEdgeMesh::PerEdgeAttributeHandle<BeamDimensions> handleBeamDimensions;
  VCGEdgeMesh::PerEdgeAttributeHandle<BeamMaterial> handleBeamMaterial;
  Eigen::MatrixX2i eigenEdges;
  Eigen::MatrixX3d eigenVertices;
  Eigen::MatrixX3d eigenEdgeNormals;

  void getEdges(Eigen::MatrixX2i &edges);
  void getVertices(Eigen::MatrixX3d &vertices);

public:
  VCGEdgeMesh();

  void setDefaultAttributes();

  static Eigen::Vector3d convertToEigenVector(const VCGEdgeMesh::CoordType &p) {
    return Eigen::Vector3d(p.X(), p.Y(), p.Z());
  }
  void getEdges(Eigen::MatrixX3d &edgeStartingPoints,
                Eigen::MatrixX3d &edgeEndingPoints) const;

  void getBeamMesh(const float &beamThickness, VCGTriMesh &beamMesh) const;

  void getNormals(Eigen::MatrixX3d &normals) const;

  bool loadUsingDefaultLoader(const std::string &plyFilename);
  bool hasProperty(const std::vector<nanoply::PlyProperty> &v,
                   const std::string &propertyName);

  bool
  hasPlyEdgeProperty(const std::string &plyFilename,
                     const std::vector<nanoply::PlyProperty> &edgeProperties,
                     const std::string &plyEdgePropertyName);

  bool plyFileHasAllRequiredFields(const std::string &plyFilename);

  bool loadUsingNanoply(const std::string &plyFilename);

  bool loadFromPly(const std::string plyFilename);

  bool savePly(const std::string plyFilename);

  // TODO: finish this function
  bool savePly(const std::string plyFilename,
               const Eigen::MatrixX3d &V /*,const Eigen::MatrixX2i& E*/);

  Eigen::MatrixX2i getEigenEdges() const;
  Eigen::MatrixX3d getEigenVertices() const;
  Eigen::MatrixX3d getEigenEdgeNormals() const;
  std::vector<BeamDimensions> getBeamDimensions() const;
  std::vector<BeamMaterial> getBeamMaterial() const;
};

#endif // EDGEMESH_HPP
