#include <filesystem>
#include <iostream>
#include <vector>

#include <threed_beam_fea.h>

#include <json.hpp>

#include <Eigen/Core>

#include <vcg/complex/algorithms/create/platonic.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>

struct Properties
{
  double EA;
  double EIz;
  double EIy; // Francesco's properties
  double GJ;

  Properties()
  {
    const double crossArea = 0.001; //(m^2)
    const double I2 =
      4.189828 * std::pow(10, -8); //(m^3) I2=I3 for round section
    const double I3 = 4.189828 * std::pow(10, -8);           //(m^3)
    const double polarInertia = 8.379656 * std::pow(10, -8); //(m^3)
    const double ni = 0.3;
    const double youngsModulus = 210000000; //(kN/m^2)
    const double G = youngsModulus / (2 * (1 + ni));

    // Properties used by fea
    EA = youngsModulus * crossArea; // Young's modulus * cross
    EIz = youngsModulus * I3;       // Young's modulus* I3
    EIy = youngsModulus * I2;       // Young's modulus* I2
    GJ = G * polarInertia;          // G * Polar Inertia
  }
};
struct VertexForce
{
  size_t vertexIndex;
  Eigen::Vector3d force;
};

class BeamSimulator
{

  class MyVertex;
  class MyFace;
  class MyEdge;

  class MyUsedTypes
    : public vcg::UsedTypes<vcg::Use<MyVertex>::AsVertexType,
                            vcg::Use<MyEdge>::AsEdgeType,
                            vcg::Use<MyFace>::AsFaceType>
  {};
  class MyVertex
    : public vcg::Vertex<MyUsedTypes,
                         vcg::vertex::Coord3d,
                         vcg::vertex::Normal3d,
                         vcg::vertex::Color4b,
                         vcg::vertex::BitFlags>
  {};

  class MyEdge
    : public vcg::Edge<MyUsedTypes,
                       vcg::edge::VertexRef,
                       vcg::edge::BitFlags,
                       vcg::edge::EEAdj>
  {};

  class MyFace
    : public vcg::Face<MyUsedTypes, vcg::face::VertexRef, vcg::face::Normal3f>
  {};

  class MyMesh
    : public vcg::tri::
        TriMesh<std::vector<MyVertex>, std::vector<MyFace>, std::vector<MyEdge>>
  {};

  std::vector<fea::Elem> elements;
  std::vector<fea::Node> nodes;
  std::vector<fea::BC> boundaryConditions;
  std::vector<fea::Force> nodalForces;
  Properties simulationProperties;
  std::string nodalDisplacementOutputFilepath;
  std::string nodalForcesOutputFilepath;

public:
  BeamSimulator() {}

  void setNodes(const std::vector<fea::Node>& nodes)
  {
    this->nodes.clear();
    this->nodes = nodes;
  }

  void setElements(const std::vector<fea::Elem>& elements)
  {
    this->elements.clear();
    this->elements = elements;
  }

  static void printInfo(const MyMesh& mesh)
  {
    const bool printDetailedInfo = false;
    if (printDetailedInfo) {
      std::cout << "~~Vertices info~~" << std::endl;
      size_t vertexIndex = 0;
      for (MyVertex v : mesh.vert) {
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
      for (MyFace f : mesh.face) {
        std::cout << "Face Index:" << faceIndex++ << std::endl;
        std::cout << "Vertices:";
        for (int vertexIndexInFace = 0; vertexIndexInFace < f.VN();
             vertexIndexInFace++) {
          size_t vertexIndexInMesh =
            vcg::tri::Index<MyMesh>(mesh, f.V(vertexIndexInFace));
          std::cout << " " << vertexIndexInMesh;
        }
        std::cout << std::endl;
      }
    } else {
      std::cout << "Number of vertices:" << mesh.VN() << endl;
      std::cout << "Number of faces:" << mesh.FN() << endl;
      std::cout << "Number of edges:" << mesh.EN() << endl;
    }
  }

  void loadPLY(const std::string plyFilename, MyMesh& mesh)
  {
    int returnValue =
      vcg::tri::io::ImporterPLY<MyMesh>::Open(mesh, plyFilename.c_str());
    if (returnValue != 0) {
      std::cout << "Unable to open %s for '%s'\n" + plyFilename +
                     vcg::tri::io::ImporterPLY<MyMesh>::ErrorMsg(returnValue)
                << std::endl;
      throw std::runtime_error{ "Unable to load the ply file." };
    }
    std::cout << plyFilename << " was loaded successfuly." << std::endl;
    vcg::tri::UpdateTopology<MyMesh>::AllocateEdge(mesh);
    vcg::tri::UpdateNormal<MyMesh>::PerVertexNormalized(mesh);
    // printInfo(mesh);
  }

  void populateNodes(const MyMesh& mesh)
  {
    nodes.clear();
    nodes.resize(static_cast<size_t>(mesh.VN()));
    for (int vertexIndex = 0; vertexIndex < mesh.VN(); vertexIndex++) {
      MyMesh::CoordType vertexCoordinates =
        mesh.vert[static_cast<size_t>(vertexIndex)].cP();
      nodes[static_cast<size_t>(vertexIndex)] = Eigen::Vector3d(
        vertexCoordinates.X(), vertexCoordinates.Y(), vertexCoordinates.Z());
    }
  }

  void populateElements(const MyMesh& mesh)
  {
    elements.clear();
    elements.resize(static_cast<size_t>(mesh.EN()));
    for (size_t edgeIndex = 0; edgeIndex < static_cast<size_t>(mesh.EN());
         edgeIndex++) {
      const MyMesh::EdgeType& edge = mesh.edge[edgeIndex];
      const vcg::Point3d nAverage =
        ((edge.cV(0)->cN() + edge.cV(1)->cN()) / 2).Normalize();
      const std::vector<double> nAverageVector{ nAverage.X(),
                                                nAverage.Y(),
                                                nAverage.Z() };
      fea::Props feaProperties(simulationProperties.EA,
                               simulationProperties.EIz,
                               simulationProperties.EIy,
                               simulationProperties.GJ,
                               nAverageVector);
      const size_t nodeIndex0 = vcg::tri::Index<MyMesh>(mesh, edge.cV(0));
      const size_t nodeIndex1 = vcg::tri::Index<MyMesh>(mesh, edge.cV(1));
      fea::Elem element(static_cast<unsigned int>(nodeIndex0),
                        static_cast<unsigned int>(nodeIndex1),
                        feaProperties);
      elements[edgeIndex] = element;
    }
  }

  void populateElementsAndNodes(const std::string plyFilename)
  {
    MyMesh mesh;
    loadPLY(plyFilename, mesh);
    populateNodes(mesh);
    populateElements(mesh);
  }

  void fixVertices(const std::vector<unsigned int> vertices)
  {
    boundaryConditions.clear();
    boundaryConditions.resize(6 * vertices.size());
    unsigned int DoFIndex = 0;
    for (unsigned int i = 0; i < vertices.size(); i++) {
      const unsigned int vertexIndex = vertices[i];
      fea::BC bcTX(vertexIndex, fea::DOF::DISPLACEMENT_X, 0);
      boundaryConditions[DoFIndex++] = bcTX;
      fea::BC bcTY(vertexIndex, fea::DOF::DISPLACEMENT_Y, 0);
      boundaryConditions[DoFIndex++] = bcTY;
      fea::BC bcTZ(vertexIndex, fea::DOF::DISPLACEMENT_Z, 0);
      boundaryConditions[DoFIndex++] = bcTZ;
      fea::BC bcRX(vertexIndex, fea::DOF::ROTATION_X, 0);
      boundaryConditions[DoFIndex++] = bcRX;
      fea::BC bcRY(vertexIndex, fea::DOF::ROTATION_Y, 0);
      boundaryConditions[DoFIndex++] = bcRY;
      fea::BC bcRZ(vertexIndex, fea::DOF::ROTATION_Z, 0);
      boundaryConditions[DoFIndex++] = bcRZ;
    }
  }

  void setNodalForces(const std::vector<std::vector<unsigned int>> vertexForces)
  {
    nodalForces.clear();
    nodalForces.resize(vertexForces.size());
    for (size_t forceIndex = 0; forceIndex < vertexForces.size();
         forceIndex++) {
      const std::vector<unsigned int>& forceVector = vertexForces[forceIndex];
      const unsigned int& nodeIndex = forceVector[0];
      const fea::DOF dof(static_cast<fea::DOF>(forceVector[1]));
      const double& forceMagnitude(static_cast<double>(forceVector[2]));
      nodalForces[forceIndex] = fea::Force(nodeIndex, dof, forceMagnitude);
    }
  }

  static void printInfo(const fea::Job& job)
  {
    std::cout << "Details regarding the fea::Job:" << std::endl;
    std::cout << "Nodes:" << std::endl;
    for (fea::Node n : job.nodes) {
      std::cout << n << std::endl;
    }
    std::cout << "Elements:" << std::endl;
    for (Eigen::Vector2i e : job.elems) {
      std::cout << e << std::endl;
    }
  }

  void reset()
  {
    elements.clear();
    nodes.clear();
    boundaryConditions.clear();
    nodalForces.clear();
  }

  void parseConfigurationFile(const std::string configurationFilename)
  {
    ifstream inFile(configurationFilename);
    std::string jsonContents((std::istreambuf_iterator<char>(inFile)),
                             std::istreambuf_iterator<char>());
    nlohmann::json jsonFile(nlohmann::json::parse(jsonContents));

    const std::string plyFilename = jsonFile["plyFilename"];
    if (!std::filesystem::exists(plyFilename)) {
      throw std::runtime_error{ "Input ply file does not exist." };
    }
    populateElementsAndNodes(filesystem::absolute(plyFilename).string());

    const std::string jsonNodalDisplacementOutputPath =
      jsonFile["nodalDisplacemntsCSV"];
    if (!jsonNodalDisplacementOutputPath.empty()) {
      nodalDisplacementOutputFilepath =
        filesystem::absolute(jsonNodalDisplacementOutputPath).string();
    }
    const std::string jsonNodalForcesOutputPath = jsonFile["nodalForcesCSV"];
    if (!jsonNodalForcesOutputPath.empty()) {
      nodalForcesOutputFilepath =
        filesystem::absolute(jsonNodalForcesOutputPath).string();
    }

    std::vector<unsigned int> fixedVertices = jsonFile["fixedVertices"];
    fixVertices(fixedVertices);

    std::vector<std::vector<unsigned int>> forces = jsonFile["forces"];
    setNodalForces(forces);
  }

  void executeSimulation(const std::string configurationFilename)
  {
    reset();
    parseConfigurationFile(configurationFilename);
    fea::Job job(nodes, elements);
    // printInfo(job);

    // create the default options
    fea::Options opts;
    if (!nodalForcesOutputFilepath.empty()) {
      opts.save_nodal_forces = true;
      opts.nodal_forces_filename = nodalForcesOutputFilepath;
    }
    if (!nodalDisplacementOutputFilepath.empty()) {
      opts.save_nodal_displacements = true;
      opts.nodal_displacements_filename = nodalDisplacementOutputFilepath;
    }

    // have the program output status updates
    opts.verbose = true;

    // form an empty vector of ties since none were prescribed
    std::vector<fea::Tie> ties;

    // also create an empty list of equations as none were prescribed
    std::vector<fea::Equation> equations;

    fea::Summary summary =
      fea::solve(job, boundaryConditions, nodalForces, ties, equations, opts);

    // print a report of the analysis
    std::cout << summary.FullReport() << std::endl;
  }
};

int
main(int argc, char* argv[])
{
  if (argc != 2) {
    throw std::runtime_error{ "Wrong number of arguments." };
  }
  const std::string configurationFilename = argv[1];
  // check if file exists using std::filesystem
  if (!std::filesystem::exists(configurationFilename)) {
    throw std::runtime_error{ "Configuration filepath does not exist." };
    return 1;
  }

  BeamSimulator simulator;
  simulator.executeSimulation(configurationFilename);
  return 0;
}
