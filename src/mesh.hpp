#ifndef MESHSTRUCTS_HPP
#define MESHSTRUCTS_HPP

#include "beam.hpp"
#include <Eigen/Dense>
#include <gsl/gsl_util>
#include <igl/matrix_to_list.h>
#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/nanoply/include/nanoplyWrapper.hpp>

struct IGLMesh {
  Eigen::MatrixX3d vertices;
  Eigen::MatrixX3i triFaces;
};

class VCGVertex;
class VCGFace;
class VCGEdge;

class VCGUsedTypes : public vcg::UsedTypes<vcg::Use<VCGVertex>::AsVertexType,
                                           vcg::Use<VCGEdge>::AsEdgeType,
                                           vcg::Use<VCGFace>::AsFaceType> {};
class VCGVertex
    : public vcg::Vertex<VCGUsedTypes, vcg::vertex::Coord3d,
                         vcg::vertex::Normal3d, vcg::vertex::Color4b,
                         vcg::vertex::BitFlags> {};

class VCGEdge : public vcg::Edge<VCGUsedTypes, vcg::edge::VertexRef,
                                 vcg::edge::BitFlags, vcg::edge::EEAdj> {};

class VCGFace : public vcg::Face<VCGUsedTypes, vcg::face::VertexRef,
                                 vcg::face::Normal3d> {};

class VCGTriMesh
    : public vcg::tri::TriMesh<std::vector<VCGVertex>, std::vector<VCGFace>,
                               std::vector<VCGEdge>> {

public:
  static Eigen::Vector3d convertToEigenVector(const VCGTriMesh::CoordType &p) {
    return Eigen::Vector3d(p.X(), p.Y(), p.Z());
  }

  void printInfo() const {
    const bool printDetailedInfo = false;
    if (printDetailedInfo) {
      std::cout << "~~Vertices info~~" << std::endl;
      size_t vertexIndex = 0;
      for (VCGVertex v : vert) {
        std::cout << "Vertex Index:" << vertexIndex++ << endl;
        std::cout << "Position:" << v.cP().X() << " " << v.cP().Y() << " "
                  << v.cP().Z() << endl;
        std::cout << "Normal:" << v.cN().X() << " " << v.cN().Y() << " "
                  << v.cN().Z() << endl;
        std::cout << "Color:" << static_cast<int>(v.cC().X()) << " "
                  << static_cast<int>(v.cC().Y()) << " "
                  << static_cast<int>(v.cC().Z()) << endl;
      }
      std::cout << "~~Faces info~~" << std::endl;
      size_t faceIndex = 0;
      for (VCGFace f : face) {
        std::cout << "Face Index:" << faceIndex++ << std::endl;
        std::cout << "Vertices:";
        for (int vertexIndexInFace = 0; vertexIndexInFace < f.VN();
             vertexIndexInFace++) {
          size_t vertexIndexInMesh =
              vcg::tri::Index<VCGTriMesh>(*this, f.cV(vertexIndexInFace));
          std::cout << " " << vertexIndexInMesh;
        }
        std::cout << std::endl;
      }
    } else {
      std::cout << "Number of vertices:" << VN() << endl;
      std::cout << "Number of faces:" << FN() << endl;
      std::cout << "Number of edges:" << EN() << endl;
    }
  }

  IGLMesh getIGLMesh() const {
    const VCGTriMesh &vcgMesh = *this;
    IGLMesh iglMesh;
    iglMesh.vertices.resize(vcgMesh.VN(), 3);
    const std::vector<VCGVertex> &meshVerts = vcgMesh.vert;
    for (gsl::index vertexIndex = 0; vertexIndex < meshVerts.size();
         vertexIndex++) {
      iglMesh.vertices.row(vertexIndex) = Eigen::Vector3d(
          meshVerts[vertexIndex].P().X(), meshVerts[vertexIndex].P().Y(),
          meshVerts[vertexIndex].P().Z());
    }

    iglMesh.triFaces.resize(vcgMesh.FN(), 3);
    const std::vector<VCGFace> &meshFaces = vcgMesh.face;
    for (gsl::index faceIndex = 0; faceIndex < meshFaces.size(); faceIndex++) {
      int vertex0Index = static_cast<int>(
          vcg::tri::Index<VCGTriMesh>(vcgMesh, meshFaces[faceIndex].cV(0)));
      int vertex1Index = static_cast<int>(
          vcg::tri::Index<VCGTriMesh>(vcgMesh, meshFaces[faceIndex].cV(1)));
      int vertex2Index = static_cast<int>(
          vcg::tri::Index<VCGTriMesh>(vcgMesh, meshFaces[faceIndex].cV(2)));
      iglMesh.triFaces.row(faceIndex) =
          Eigen::Vector3i(vertex0Index, vertex1Index, vertex2Index);
    }
    return iglMesh;
  }

  void getVertices(Eigen::MatrixX3d &vertices) const {
    vertices.resize(VN(), 3);
    for (int vertexIndex = 0; vertexIndex < VN(); vertexIndex++) {
      VCGTriMesh::CoordType vertexCoordinates =
          vert[static_cast<size_t>(vertexIndex)].cP();
      vertices.row(vertexIndex) = convertToEigenVector(vertexCoordinates);
    }
  }

  void getEdges(Eigen::MatrixX2i &edges) const {
    edges.resize(EN(), 2);
    for (int edgeIndex = 0; edgeIndex < EN(); edgeIndex++) {
      const VCGTriMesh::EdgeType &edge = this->edge[edgeIndex];
      const size_t nodeIndex0 = vcg::tri::Index<VCGTriMesh>(*this, edge.cV(0));
      const size_t nodeIndex1 = vcg::tri::Index<VCGTriMesh>(*this, edge.cV(1));
      edges.row(edgeIndex) = Eigen::Vector2i(nodeIndex0, nodeIndex1);
    }
  }

  void getNormals(Eigen::MatrixX3d &normals) const {
    normals.resize(VN(), 3);
    for (int vertexIndex = 0; vertexIndex < VN(); vertexIndex++) {
      VCGTriMesh::CoordType vertexNormal =
          vert[static_cast<size_t>(vertexIndex)].cN();
      normals.row(vertexIndex) = convertToEigenVector(vertexNormal);
    }
  }
  static VCGTriMesh::CoordType convertToVCGPoint(const Eigen::Vector3d &p) {
    return {p.x(), p.y(), p.z()};
  }

  static void constructBeamMesh(const Eigen::MatrixX3d &nodes,
                                const Eigen::MatrixX2i &edges,
                                const float &beamThickness,
                                VCGTriMesh &beamMesh, int slices = 10,
                                int stacks = 1) {
    for (int edgeIndex = 0; edgeIndex < edges.rows(); edgeIndex++) {
      const VCGTriMesh::CoordType &p0 =
          convertToVCGPoint(nodes.row(edges(edgeIndex, 0)));
      const VCGTriMesh::CoordType &p1 =
          convertToVCGPoint(nodes.row(edges(edgeIndex, 1)));
      VCGTriMesh beam;
      vcg::tri::OrientedCylinder(beam, p0, p1, beamThickness, true, slices,
                                 stacks);
      vcg::tri::Append<VCGTriMesh, VCGTriMesh>::Mesh(beamMesh, beam);
    }
  }

  static void constructBeamMesh(
      const Eigen::MatrixX3d &nodes, const Eigen::MatrixX2i &edges,
      const Eigen::MatrixX3d &edgeNormals,
      const std::vector<BeamDimensions> &beamDimensions, VCGTriMesh &beamMesh) {
    for (int edgeIndex = 0; edgeIndex < edges.rows(); edgeIndex++) {
      const Eigen::Vector3d &p0 = nodes.row(edges(edgeIndex, 0));
      const Eigen::Vector3d &p1 = nodes.row(edges(edgeIndex, 1));
      VCGTriMesh beam;
      VCGTriMesh::BoxType bb;
      const Eigen::Vector3d edgeVector(p1 - p0);
      const float &b = beamDimensions[edgeIndex].b;
      const float &h = beamDimensions[edgeIndex].h;
      bb.max = {edgeVector.norm(), h / 2, b / 2};
      bb.min = {0, -h / 2, -b / 2};
      vcg::tri::Box(beam, bb);
      std::cout << "b=" + std::to_string(b) + " h=" + std::to_string(h)
                << std::endl;

      vcg::Matrix44<double> R;
      R.SetIdentity();
      const Eigen::Vector3d localX = edgeVector.normalized();
      const Eigen::Vector3d localY = edgeNormals.row(edgeIndex).normalized();
      const Eigen::Vector3d localZ = localX.cross(localY).normalized();
      R.SetColumn(0, {localX.x(), localX.y(), localX.z(), 0});
      R.SetColumn(1, {localY.x(), localY.y(), localY.z(), 0});
      R.SetColumn(2, {localZ.x(), localZ.y(), localZ.z(), 0});
      vcg::Matrix44<double> T;
      T.SetTranslate({p0.x(), p0.y(), p0.z()});
      // TODO: It would be faster if I called the UpdatePosition
      // function once
      // for all vertices of the beam mesh
      vcg::tri::UpdatePosition<VCGTriMesh>::Matrix(beam, T * R);
      vcg::tri::Append<VCGTriMesh, VCGTriMesh>::Mesh(beamMesh, beam);
    }
  }
};

inline std::string convertToLowercase(const std::string &s) {
  std::string lowercaseString = s;
  std::transform(s.begin(), s.end(), lowercaseString.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return lowercaseString;
}

#endif // MESHSTRUCTS_HPP
