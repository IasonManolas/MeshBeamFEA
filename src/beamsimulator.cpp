#include "beamsimulator.hpp"
#include <Eigen/Core>
#include <filesystem>
#include <gsl/gsl>
#include <iostream>
#include <vcg/complex/algorithms/create/platonic.h>
#include <vcg/complex/algorithms/update/normal.h>

BeamSimulator::BeamSimulator() {}

void BeamSimulator::setSimulation(
    const Eigen::MatrixX3d &nodes, const Eigen::MatrixX2i &elements,
    const Eigen::MatrixX3d &elementNormals, const Eigen::VectorXi &fixedNodes,
    const std::vector<NodalForce> &nodalForces,
    const std::vector<BeamDimensions> &beamDimensions,
    const std::vector<BeamMaterial> &beamMaterial) {
  assert(elementNormals.rows() == elements.rows());
  reset();
  setNodes(nodes);
  setElements(elements, elementNormals, beamDimensions, beamMaterial);
  setFixedNodes(fixedNodes);
  setNodalForces(nodalForces);
}

void BeamSimulator::setResultsNodalDisplacementCSVFilepath(
    const string &outputFilepath) {
  this->nodalDisplacementsOutputFilepath = outputFilepath;
}

void BeamSimulator::setResultsElementalForcesCSVFilepath(
    const string &outputFilepath) {
  this->elementalForcesOutputFilepath = outputFilepath;
}

void BeamSimulator::setNodes(const Eigen::MatrixX3d &nodes) {
  const long int numberOfNodes = nodes.rows();
  this->nodes.resize(gsl::narrow_cast<int>(numberOfNodes));
  for (gsl::index nodeIndex = 0; nodeIndex < nodes.rows(); nodeIndex++) {
    this->nodes[nodeIndex] = static_cast<fea::Node>(nodes.row(nodeIndex));
  }
}

void BeamSimulator::setElements(
    const Eigen::MatrixX2i &elements, const Eigen::MatrixX3d &elementNormals,
    const std::vector<BeamDimensions> &beamDimensions,
    const std::vector<BeamMaterial> &beamMaterial) {
  assert(elements.rows() == beamDimensions.size());
  assert(elements.rows() == beamMaterial.size());
  const long int numberOfEdges = elements.rows();
  this->elements.clear();
  this->elements.resize(static_cast<size_t>(numberOfEdges));
  for (gsl::index edgeIndex = 0; edgeIndex < numberOfEdges; edgeIndex++) {
    const gsl::index nodeIndex0 = elements(edgeIndex, 0);
    const gsl::index nodeIndex1 = elements(edgeIndex, 1);
    const bool nodeIndicesAreInRange =
        nodeIndex0 < nodes.size() && nodeIndex1 < nodes.size();
    assert(nodeIndicesAreInRange);
    const std::vector<double> nAverageVector{elementNormals(edgeIndex, 0),
                                             elementNormals(edgeIndex, 1),
                                             elementNormals(edgeIndex, 2)};
    BeamSimulationProperties prop(beamDimensions[edgeIndex],
                                  beamMaterial[edgeIndex]);
    fea::Props feaProperties(prop.EA, prop.EIz, prop.EIy, prop.GJ,
                             nAverageVector);
    fea::Elem element(nodeIndex0, nodeIndex1, feaProperties);
    this->elements[edgeIndex] = element;
  }
}

void BeamSimulator::setFixedNodes(const Eigen::VectorXi &fixedNodes) {
  boundaryConditions.clear();
  boundaryConditions.resize(static_cast<size_t>(6 * fixedNodes.rows()));
  gsl::index boundaryConditionIndex = 0;
  for (gsl::index fixedNodeIndex = 0; fixedNodeIndex < fixedNodes.rows();
       fixedNodeIndex++) {
    if (fixedNodes(fixedNodeIndex) < 0 ||
        static_cast<size_t>(fixedNodes(fixedNodeIndex)) > nodes.size() - 1) {
      throw std::runtime_error{"Vertex index is outside of valid range."};
    }
    const gsl::index vertexIndex = fixedNodes(fixedNodeIndex);
    fea::BC bcTX(vertexIndex, fea::DOF::DISPLACEMENT_X, 0);
    boundaryConditions[boundaryConditionIndex++] = bcTX;
    fea::BC bcTY(vertexIndex, fea::DOF::DISPLACEMENT_Y, 0);
    boundaryConditions[boundaryConditionIndex++] = bcTY;
    fea::BC bcTZ(vertexIndex, fea::DOF::DISPLACEMENT_Z, 0);
    boundaryConditions[boundaryConditionIndex++] = bcTZ;
    fea::BC bcRX(vertexIndex, fea::DOF::ROTATION_X, 0);
    boundaryConditions[boundaryConditionIndex++] = bcRX;
    fea::BC bcRY(vertexIndex, fea::DOF::ROTATION_Y, 0);
    boundaryConditions[boundaryConditionIndex++] = bcRY;
    fea::BC bcRZ(vertexIndex, fea::DOF::ROTATION_Z, 0);
    boundaryConditions[boundaryConditionIndex++] = bcRZ;
  }
}

void BeamSimulator::setNodalForces(const std::vector<NodalForce> &nodalForces) {
  this->nodalForces.clear();
  this->nodalForces.resize(nodalForces.size());
  for (gsl::index forceIndex = 0; forceIndex < nodalForces.size();
       forceIndex++) {
    const NodalForce &force = nodalForces[forceIndex];

    if (force.index > nodes.size() - 1) {
      throw std::range_error{"Node index is out of valid range."};
    }

    this->nodalForces[forceIndex] =
        fea::Force(force.index, force.dof, force.magnitude);
  }
}

fea::Summary BeamSimulator::getResults() const { return results; }

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
  printInfo(job);

  // create the default options
  fea::Options opts;
  const bool outputDisplacements = true;
  const bool outputForces = true;
  if (!elementalForcesOutputFilepath.empty()) {
    opts.save_elemental_forces = true;
    opts.elemental_forces_filename = elementalForcesOutputFilepath;
  }
  if (!nodalDisplacementsOutputFilepath.empty()) {
    opts.save_nodal_displacements = true;
    opts.nodal_displacements_filename = nodalDisplacementsOutputFilepath;
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
  //  std::cout << summary.FullReport() << std::endl;
  results = summary;

  return summary;
}
