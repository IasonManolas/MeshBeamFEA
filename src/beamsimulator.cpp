#include "beamsimulator.hpp"

#include <Eigen/Core>

#include <vcg/complex/algorithms/create/platonic.h>
#include <vcg/complex/algorithms/update/normal.h>

#include <iostream>

BeamSimulator::BeamSimulator(const VCGMesh &m, const SimulationOptions &props)
    : mesh(m), properties(props) {
  populateNodes(mesh);
  populateElements(mesh);
  fixVertices(properties.fixedVertices);
  setNodalForces(properties.nodalForces);
}

void BeamSimulator::setNodes(const std::vector<fea::Node> &nodes) {
  this->nodes.clear();
  this->nodes = nodes;
}

void BeamSimulator::setElements(const std::vector<fea::Elem> &elements) {
  this->elements.clear();
  this->elements = elements;
}

void BeamSimulator::populateNodes(const VCGMesh &mesh) {
  nodes.clear();
  nodes.resize(static_cast<size_t>(mesh.VN()));
  for (int vertexIndex = 0; vertexIndex < mesh.VN(); vertexIndex++) {
    VCGMesh::CoordType vertexCoordinates =
        mesh.vert[static_cast<size_t>(vertexIndex)].cP();
    nodes[static_cast<size_t>(vertexIndex)] = Eigen::Vector3d(
        vertexCoordinates.X(), vertexCoordinates.Y(), vertexCoordinates.Z());
  }
}

void BeamSimulator::populateElements(const VCGMesh &mesh) {
  elements.clear();
  elements.resize(static_cast<size_t>(mesh.EN()));
  for (size_t edgeIndex = 0; edgeIndex < static_cast<size_t>(mesh.EN());
       edgeIndex++) {
    const VCGMesh::EdgeType &edge = mesh.edge[edgeIndex];
    const vcg::Point3d nAverage =
        ((edge.cV(0)->cN() + edge.cV(1)->cN()) / 2).Normalize();
    const std::vector<double> nAverageVector{nAverage.X(), nAverage.Y(),
                                             nAverage.Z()};
    fea::Props feaProperties(properties.beamProperties.EA,
                             properties.beamProperties.EIz,
                             properties.beamProperties.EIy,
                             properties.beamProperties.GJ, nAverageVector);
    const size_t nodeIndex0 = vcg::tri::Index<VCGMesh>(mesh, edge.cV(0));
    const size_t nodeIndex1 = vcg::tri::Index<VCGMesh>(mesh, edge.cV(1));
    fea::Elem element(static_cast<unsigned int>(nodeIndex0),
                      static_cast<unsigned int>(nodeIndex1), feaProperties);
    elements[edgeIndex] = element;
  }
}

void BeamSimulator::fixVertices(const std::vector<int> vertices) {
  boundaryConditions.clear();
  boundaryConditions.resize(6 * vertices.size());
  unsigned int DoFIndex = 0;
  for (unsigned int i = 0; i < vertices.size(); i++) {
    if (vertices[i] < 0 ||
        static_cast<size_t>(vertices[i]) > nodes.size() - 1) {
      throw std::runtime_error{"Vertex index is outside of valid range."};
    }
    const unsigned int vertexIndex = static_cast<unsigned int>(vertices[i]);
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

void BeamSimulator::setNodalForces(
    const std::vector<std::vector<int>> vertexForces) {
  nodalForces.clear();
  nodalForces.resize(vertexForces.size());
  for (size_t forceIndex = 0; forceIndex < vertexForces.size(); forceIndex++) {
    const std::vector<int> &forceVector = vertexForces[forceIndex];

    const int &nodeIndex = forceVector[0];
    if (nodeIndex < 0 ||
        static_cast<unsigned int>(nodeIndex) > nodes.size() - 1) {
      throw std::range_error{"Node index is out of valid range."};
    }

    const int &jsonDoF = forceVector[1];
    if (jsonDoF > 5 || jsonDoF < 0) {
      throw std::range_error{"DoF given is outside of valid range."};
    }
    const fea::DOF DoF(static_cast<fea::DOF>(jsonDoF));

    const double &forceMagnitude(static_cast<double>(forceVector[2]));

    nodalForces[forceIndex] =
        fea::Force(static_cast<unsigned int>(nodeIndex), DoF, forceMagnitude);
  }
}

void BeamSimulator::printInfo(const fea::Job &job) {
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

void BeamSimulator::reset() {
  elements.clear();
  nodes.clear();
  boundaryConditions.clear();
  nodalForces.clear();
}

fea::Summary BeamSimulator::executeSimulation() {
  fea::Job job(nodes, elements);
  // printInfo(job);

  // create the default options
  fea::Options opts;
  if (!properties.nodalForcesOutputFilepath.empty()) {
    opts.save_nodal_forces = true;
    opts.nodal_forces_filename = properties.nodalForcesOutputFilepath;
  }
  if (!properties.nodalDisplacementsOutputFilepath.empty()) {
    opts.save_nodal_displacements = true;
    opts.nodal_displacements_filename =
        properties.nodalDisplacementsOutputFilepath;
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

  return summary;
}
