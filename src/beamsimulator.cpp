#include "beamsimulator.hpp"
#include <Eigen/Core>
#include <filesystem>
#include <gsl/gsl>
#include <gsl/gsl_assert>
#include <iostream>
#include <vcg/complex/algorithms/create/platonic.h>
#include <vcg/complex/algorithms/update/normal.h>

BeamSimulator::BeamSimulator() {}

void BeamSimulator::setSimulation(const SimulationJob &simulationJob) {
  this->simulationJob = FeaSimulationJob(simulationJob);
}

void BeamSimulator::setResultsNodalDisplacementCSVFilepath(
    const string &outputFilepath) {
  this->nodalDisplacementsOutputFilepath = outputFilepath;
}

void BeamSimulator::setResultsElementalForcesCSVFilepath(
    const string &outputFilepath) {
  this->elementalForcesOutputFilepath = outputFilepath;
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

fea::Summary BeamSimulator::executeSimulation() {
  Expects(!simulationJob.isEmpty());
  fea::Job job(simulationJob.nodes, simulationJob.elements);
  printInfo(job);

  // create the default options
  fea::Options opts;
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

  // form an empty vector of ties
  std::vector<fea::Tie> ties;

  // also create an empty list of equations
  std::vector<fea::Equation> equations;

  results = fea::solve(job, simulationJob.boundaryConditions,
                       simulationJob.nodalForces, ties, equations, opts);

  return results;
}

void FeaSimulationJob::setElements(const SimulationJob &job) {
  const long int numberOfEdges = job.elements.rows();
  Expects(numberOfEdges == job.beamDimensions.size());
  Expects(numberOfEdges == job.beamMaterial.size());
  Expects(numberOfEdges == job.elementalNormals.rows());
  elements.clear();
  elements.resize(static_cast<size_t>(numberOfEdges));
  for (gsl::index edgeIndex = 0; edgeIndex < numberOfEdges; edgeIndex++) {
    const gsl::index nodeIndex0 = job.elements(edgeIndex, 0);
    const gsl::index nodeIndex1 = job.elements(edgeIndex, 1);
    const bool nodeIndicesAreInRange =
        nodeIndex0 < job.nodes.size() && nodeIndex1 < job.nodes.size();
    Expects(nodeIndicesAreInRange);
    const std::vector<double> nAverageVector{
        job.elementalNormals(edgeIndex, 0), job.elementalNormals(edgeIndex, 1),
        job.elementalNormals(edgeIndex, 2)};
    BeamSimulationProperties prop(job.beamDimensions[edgeIndex],
                                  job.beamMaterial[edgeIndex]);
    fea::Props feaProperties(prop.EA, prop.EIz, prop.EIy, prop.GJ,
                             nAverageVector);
    fea::Elem element(nodeIndex0, nodeIndex1, feaProperties);
    elements[edgeIndex] = element;
  }
}

void FeaSimulationJob::setNodes(const SimulationJob &job) {
  const long int numberOfNodes = job.nodes.rows();
  this->nodes.resize(gsl::narrow_cast<int>(numberOfNodes));
  for (gsl::index nodeIndex = 0; nodeIndex < numberOfNodes; nodeIndex++) {
    this->nodes[nodeIndex] = static_cast<fea::Node>(job.nodes.row(nodeIndex));
  }

  setFixedNodes(job.fixedNodes);
  setNodalForces(job.nodalForces);
}

FeaSimulationJob::FeaSimulationJob(const SimulationJob &job) {
  clear();
  setElements(job);
  setNodes(job);
}

FeaSimulationJob::FeaSimulationJob() {}

bool FeaSimulationJob::isEmpty() const {
  return elements.empty() || nodes.empty();
}

void FeaSimulationJob::clear() {
  elements.clear();
  nodes.clear();
  boundaryConditions.clear();
  nodalForces.clear();
}

void FeaSimulationJob::setFixedNodes(const Eigen::VectorXi &fixedNodes) {
  const bool nodesHaveBeenSet{!nodes.empty()};
  Expects(nodesHaveBeenSet);
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

void FeaSimulationJob::setNodalForces(
    const std::vector<NodalForce> &nodalForces) {
  const bool nodesHaveBeenSet{!nodes.empty()};
  Expects(nodesHaveBeenSet);
  this->nodalForces.clear();
  this->nodalForces.resize(nodalForces.size());
  for (gsl::index forceIndex = 0; forceIndex < nodalForces.size();
       forceIndex++) {
    const NodalForce &force = nodalForces[forceIndex];

    if (force.index > this->nodes.size() - 1) {
      throw std::range_error{"Node index is out of valid range."};
    }

    this->nodalForces[forceIndex] =
        fea::Force(force.index, force.dof, force.magnitude);
  }
}

BeamSimulationProperties::BeamSimulationProperties(
    const BeamDimensions &dimensions, const BeamMaterial &material) {
  crossArea = (dimensions.b * dimensions.h);
  I2 = dimensions.b * std::pow(dimensions.h, 3) / 12;
  I3 = dimensions.h * std::pow(dimensions.b, 3) / 12;
  polarInertia = (I2 + I3);

  const float youngsModulusPascal =
      material.youngsModulusGPascal * std::pow(10, 9);

  G = youngsModulusPascal / (2 * (1 + material.poissonsRatio));
  EA = youngsModulusPascal * crossArea;
  EIy = youngsModulusPascal * I3;
  EIz = youngsModulusPascal * I2;
  GJ = G * polarInertia;
}
