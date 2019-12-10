#ifndef MESHSTRUCTS_HPP
#define MESHSTRUCTS_HPP

#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>

class MyVertex;
class MyFace;
class MyEdge;

class MyUsedTypes : public vcg::UsedTypes<vcg::Use<MyVertex>::AsVertexType,
                                          vcg::Use<MyEdge>::AsEdgeType,
                                          vcg::Use<MyFace>::AsFaceType> {};
class MyVertex : public vcg::Vertex<MyUsedTypes, vcg::vertex::Coord3d,
                                    vcg::vertex::Normal3d, vcg::vertex::Color4b,
                                    vcg::vertex::BitFlags> {};

class MyEdge : public vcg::Edge<MyUsedTypes, vcg::edge::VertexRef,
                                vcg::edge::BitFlags, vcg::edge::EEAdj> {};

class MyFace
    : public vcg::Face<MyUsedTypes, vcg::face::VertexRef, vcg::face::Normal3f> {
};

class VCGMesh
    : public vcg::tri::TriMesh<std::vector<MyVertex>, std::vector<MyFace>,
                               std::vector<MyEdge>> {
public:
  void printInfo() const {
    const bool printDetailedInfo = false;
    if (printDetailedInfo) {
      std::cout << "~~Vertices info~~" << std::endl;
      size_t vertexIndex = 0;
      for (MyVertex v : vert) {
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
      for (MyFace f : face) {
        std::cout << "Face Index:" << faceIndex++ << std::endl;
        std::cout << "Vertices:";
        for (int vertexIndexInFace = 0; vertexIndexInFace < f.VN();
             vertexIndexInFace++) {
          size_t vertexIndexInMesh =
              vcg::tri::Index<VCGMesh>(*this, f.V(vertexIndexInFace));
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
};

#endif // MESHSTRUCTS_HPP
