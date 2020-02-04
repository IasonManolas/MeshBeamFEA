#include "beamsimulatortester.hpp"
#include "edgemesh.hpp"
#include <Eigen/Geometry>
#include <filesystem>
#include <gsl/gsl_util>
#include <sstream>

void BeamSimulatorTester::launchRotationTest() {
  CantileverBeam cantileverBeam;
  const std::filesystem::path resultsFolderPath{
      std::filesystem::current_path().append("Results")};
  std::filesystem::create_directory(resultsFolderPath);
  const std::filesystem::path forceResultsPath{
      std::filesystem::path(resultsFolderPath).append("Forces")};
  std::filesystem::create_directory(forceResultsPath);
  const std::filesystem::path displacementResultsPath{
      std::filesystem::path(resultsFolderPath).append("Displacements")};
  std::filesystem::create_directory(displacementResultsPath);
  const Eigen::Vector3d initialXAxis(cantileverBeam.XAxis);
  const Eigen::Vector3d initialYAxis(cantileverBeam.YAxis);
  Eigen::Vector3d rotationAxis{Eigen::Vector3d::UnitZ()};
  do {
    const float rotationAngle = M_PI_2;
    const int numberOfRotationSteps = std::ceil(2 * M_PI / rotationAngle);
    for (gsl::index rotationStep = 1; rotationStep < numberOfRotationSteps;
         rotationStep++) {
      const Eigen::AngleAxis<double> R{rotationStep * rotationAngle,
                                       rotationAxis};
      cantileverBeam.XAxis = R.toRotationMatrix() * initialXAxis;
      cantileverBeam.YAxis = R.toRotationMatrix() * initialYAxis;
      for (size_t numberOfBeams = 1; numberOfBeams < 6; numberOfBeams++) {
        cantileverBeam.numberOfBeams = numberOfBeams;
        setSimulation(cantileverBeam);
        setSimulationResultsCSV(cantileverBeam.XAxis, cantileverBeam.YAxis,
                                numberOfBeams, forceResultsPath,
                                displacementResultsPath);
        beamSimulator.executeSimulation();
      }
    }
  } while (std::next_permutation(rotationAxis.data(), rotationAxis.data() + 3));
}

// void BeamSimulatorTester::launchRotationTest(
//    const std::string &plyFilename, const Eigen::VectorXi &fixedNodes,
//    const std::vector<NodalForces> &nodalForces) {
//  createResultFolders();
//}

void BeamSimulatorTester::setSimulation(
    const BeamSimulatorTester::CantileverBeam &cantileverBeam) {
  const Eigen::MatrixX3d nodes(cantileverBeam.getNodes());
  const Eigen::MatrixX2i elements(cantileverBeam.getElements());
  const Eigen::MatrixX3d elementNormals(cantileverBeam.getNormals());
  const Eigen::VectorXi fixedNodes(cantileverBeam.getFixedNodes());
  const std::vector<NodalForce> nodeForces(cantileverBeam.getNodalForces());
  const std::vector<BeamDimensions> beamDimensions(
      cantileverBeam.getBeamDimensions());
  const std::vector<BeamMaterial> beamMaterial(
      cantileverBeam.getBeamMaterial());
  SimulationJob simulationJob{nodes,       elements,   elementNormals,
                              fixedNodes,  nodeForces, beamDimensions,
                              beamMaterial};
  beamSimulator.setSimulation(simulationJob);
}

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 2) {
  std::ostringstream out;
  out.precision(n);
  out << std::fixed << a_value;
  return out.str();
}

void BeamSimulatorTester::setSimulationResultsCSV(
    const Eigen::Vector3d &beamXAxis, const Eigen::Vector3d &beamYAxis,
    const size_t &numberOfBeams, const filesystem::path &forceResultPath,
    const filesystem::path &displacementResultPath) {
  const std::string resultFileID{to_string_with_precision(numberOfBeams) +
                                 "_(" + to_string_with_precision(beamXAxis(0)) +
                                 "," + to_string_with_precision(beamXAxis(1)) +
                                 "," + to_string_with_precision(beamXAxis(2)) +
                                 ")" + "_(" +
                                 to_string_with_precision(beamYAxis(0)) + "," +
                                 to_string_with_precision(beamYAxis(1)) + "," +
                                 to_string_with_precision(beamYAxis(2)) + ")"};
  const std::string elementalForcesFilename{resultFileID + "_ef.csv"};
  const std::string nodalDisplacementsFilename{resultFileID + "_nd.csv"};
  beamSimulator.setResultsElementalForcesCSVFilepath(
      std::filesystem::path(forceResultPath)
          .append(elementalForcesFilename)
          .string());
  beamSimulator.setResultsNodalDisplacementCSVFilepath(
      std::filesystem::path(displacementResultPath)
          .append(nodalDisplacementsFilename)
          .string());
}

void BeamSimulatorTester::createResultFolders() const {}

BeamSimulatorTester::BeamSimulatorTester() {}

// void BeamSimulatorTester::rotationTests(const string &plyFilename) {
//  Eigen::VectorXi fixedNodes(1);
//  fixedNodes(0) = 0;
//  launchRotationTest(plyFilename, );
//}

void BeamSimulatorTester::launchTests() { launchRotationTest(); }

Eigen::MatrixX3d BeamSimulatorTester::CantileverBeam::getNodes() const {
  const size_t numberOfNodes = numberOfBeams + 1;
  Eigen::MatrixX3d nodes(numberOfNodes, 3);
  const Eigen::Vector3d stepVector(XAxis.normalized() / numberOfBeams);
  for (gsl::index nodeIndex = 0; nodeIndex < numberOfNodes; nodeIndex++) {
    nodes.row(nodeIndex) = nodeIndex * stepVector;
  }
  return nodes;
}

Eigen::MatrixX2i BeamSimulatorTester::CantileverBeam::getElements() const {
  Eigen::MatrixX2i elements(numberOfBeams, 2);
  for (gsl::index beamIndex = 0; beamIndex < numberOfBeams; beamIndex++) {
    elements(beamIndex, 0) = beamIndex;
    elements(beamIndex, 1) = beamIndex + 1;
  }
  return elements;
}

Eigen::MatrixX3d BeamSimulatorTester::CantileverBeam::getNormals() const {
  Eigen::MatrixX3d elementNormals(numberOfBeams, 3);
  for (gsl::index beamIndex = 0; beamIndex < numberOfBeams; beamIndex++) {
    elementNormals(beamIndex, 0) = YAxis(0);
    elementNormals(beamIndex, 1) = YAxis(1);
    elementNormals(beamIndex, 2) = YAxis(2);
  }
  return elementNormals;
}

Eigen::VectorXi BeamSimulatorTester::CantileverBeam::getFixedNodes() const {
  Eigen::VectorXi fixedNodes(1);
  fixedNodes(0) = 0;
  return fixedNodes;
}

std::vector<NodalForce>
BeamSimulatorTester::CantileverBeam::getNodalForces() const {
  const gsl::index lastNodeIndex = numberOfBeams;
  NodalForce xForce{lastNodeIndex, 0, YAxis(0) * forceMagnitude};
  NodalForce yForce{lastNodeIndex, 1, YAxis(1) * forceMagnitude};
  NodalForce zForce{lastNodeIndex, 2, YAxis(2) * forceMagnitude};
  return {xForce, yForce, zForce};
}

std::vector<BeamDimensions>
BeamSimulatorTester::CantileverBeam::getBeamDimensions() const {
  return std::vector<BeamDimensions>(numberOfBeams, BeamDimensions(b, h));
}

std::vector<BeamMaterial>
BeamSimulatorTester::CantileverBeam::getBeamMaterial() const {
  return std::vector<BeamMaterial>(
      numberOfBeams, BeamMaterial(poissonsRatio, youngsModulusGPascal));
}
