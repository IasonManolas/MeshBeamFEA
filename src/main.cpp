#include <iostream>
#include <vector>

#include "threed_beam_fea.h"

#include "Eigen/Core"

#include "igl/edges.h"
#include "igl/list_to_matrix.h"
#include "igl/matrix_to_list.h"
#include "igl/readPLY.h"

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

class BeamSimulator
{

  std::vector<fea::Elem> elements;
  std::vector<fea::Node> nodes;
  std::vector<fea::BC> boundaryConditions;
  std::vector<fea::Force> nodalForces;
  Properties simulationProperties;

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

  void loadElementsAndNodesFromPLY(const std::string plyFilename)
  {
    std::vector<std::vector<double>> v, n, uv;
    std::vector<std::vector<int>> f;
    igl::readPLY(plyFilename, v, f, n, uv);
    Eigen::MatrixX3d V, N;
    Eigen::MatrixX3i F;
    igl::list_to_matrix(v, V);
    igl::list_to_matrix(f, F);
    igl::list_to_matrix(n, N);
    assert(V.size() != 0 && N.size() != 0 && F.size() != 0);

    nodes.resize(static_cast<size_t>(V.rows()));
    for (int vertexIndex = 0; vertexIndex < V.rows(); vertexIndex++) {
      nodes[static_cast<size_t>(vertexIndex)] = V.row(vertexIndex);
    }

    Eigen::MatrixX2i E;
    igl::edges(F, E);
    using Edge = std::vector<int>;
    std::vector<Edge> e;
    igl::matrix_to_list(E, e);
    std::transform(
      e.begin(), e.end(), std::back_inserter(elements), [&](const Edge& edge) {
        const unsigned int nodeIndex1 = static_cast<unsigned int>(edge[0]);
        const unsigned int nodeIndex2 = static_cast<unsigned int>(edge[1]);
        const Eigen::Vector3d n1(N.row(nodeIndex1));
        const Eigen::Vector3d n2(N.row(nodeIndex2));
        const Eigen::Vector3d nAverage = (n1 + n2) / 2;
        const std::vector<double> nAverageVector = { nAverage.x(),
                                                     nAverage.y(),
                                                     nAverage.z() };
        fea::Props feaProperties(simulationProperties.EA,
                                 simulationProperties.EIz,
                                 simulationProperties.EIy,
                                 simulationProperties.GJ,
                                 nAverageVector);
        return fea::Elem(nodeIndex1, nodeIndex2, feaProperties);
      });
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

  void setNodalForces(const std::vector<fea::Force> forces)
  {
    nodalForces.clear();
    nodalForces = forces;
  }

  void executeSimulation()
  {
    // create the default options
    fea::Options opts;

    // request nodal forces and displacements
    opts.save_nodal_forces = true;
    opts.save_nodal_displacements = true;

    // set custom name for nodal forces output
    opts.nodal_forces_filename = "cantilever_beam_forces.csv";

    // have the program output status updates
    opts.verbose = true;

    // form an empty vector of ties since none were prescribed
    std::vector<fea::Tie> ties;

    // also create an empty list of equations as none were prescribed
    std::vector<fea::Equation> equations;

    fea::Job job(nodes, elements);
    fea::Summary summary =
      fea::solve(job, boundaryConditions, nodalForces, ties, equations, opts);

    // print a report of the analysis
    std::cout << summary.FullReport() << std::endl;
  }
};

int
main()
{
  bool usePLY = true;
  BeamSimulator simulator;
  if (usePLY) {
    const std::string filename("/home/iason/Coding/Projects/Euler-Bernoulli "
                               "Beam simulation/tetrahedron.ply");
    simulator.loadElementsAndNodesFromPLY(filename);
  } else {
    fea::Node node1, node2;
    // place the first node at (0, 0, 0)
    node1 << 0, 0, 0;
    // place the second node at (1, 0, 0)
    node2 << 1, 0, 0;
    std::vector<fea::Node> nodes{ node1, node2 };
    simulator.setNodes(nodes);

    Properties materialProperties;
    std::vector<double> normal_vec = { 0.0, 0.0, 1.0 };
    fea::Props props(materialProperties.EA,
                     materialProperties.EIz,
                     materialProperties.EIy,
                     materialProperties.GJ,
                     normal_vec);
    // define the indices of the node list that form the element
    unsigned int nn1 = 0;
    unsigned int nn2 = 1;
    fea::Elem elem(nn1, nn2, props);
    std::vector<fea::Elem> elements{ elem };
    simulator.setElements(elements);
  }

  std::vector<unsigned int> fixedVertices{ 1, 2, 3 };
  simulator.fixVertices(fixedVertices);
  simulator.setNodalForces(
    std::vector<fea::Force>{ fea::Force(0, fea::DOF::DISPLACEMENT_X, 1000) });
  simulator.executeSimulation();
  return 0;
}
