#ifndef BEAMSIMULATOR_HPP
#define BEAMSIMULATOR_HPP

#include "beam.hpp"
#include "mesh.hpp"
#include <filesystem>
#include <gsl/gsl>
#include <igl/list_to_matrix.h>
#include <nlohmann/json.hpp>
#include <threed_beam_fea.h>
#include <vector>

struct BeamSimulationProperties {
  float crossArea;
  float I2;
  float I3;
  float polarInertia;
  float G;
  // Properties used by fea
  float EA;
  float EIz;
  float EIy;
  float GJ;

  BeamSimulationProperties(const BeamDimensions &dimensions,
                           const BeamMaterial &material) {
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
};

struct NodalForce {
  gsl::index index;
  gsl::index dof;
  double magnitude;
};

struct SimulationJob {
  Eigen::MatrixX3d nodes;
  Eigen::MatrixX2i elements;
  Eigen::MatrixX3d elementalNormals;
  Eigen::VectorXi fixedNodes;
  std::vector<NodalForce> nodalForces;
  std::vector<BeamDimensions> beamDimensions;
  std::vector<BeamMaterial> beamMaterial;
};

class BeamSimulator {
private:
  std::vector<fea::Elem> elements;
  std::vector<fea::Node> nodes;
  std::vector<fea::BC> boundaryConditions;
  std::vector<fea::Force> nodalForces;
  std::vector<Eigen::Vector3d> nodeNormals;
  std::string nodalDisplacementsOutputFilepath{"nodal_displacement.csv"};
  std::string elementalForcesOutputFilepath{"elemental_forces.csv"};
  fea::Summary results;

  static void printInfo(const fea::Job &job);
  void reset();
  void setMesh(VCGTriMesh &mesh);
  void setNodes(const Eigen::MatrixX3d &nodes);
  void setElements(const Eigen::MatrixX2i &elements,
                   const Eigen::MatrixX3d &elementNormals,
                   const std::vector<BeamDimensions> &beamDimensions,
                   const std::vector<BeamMaterial> &beamMaterial);
  void setFixedNodes(const Eigen::VectorXi &fixedNodes);
  void setNodalForces(const std::vector<NodalForce> &nodalForces);
  void setNodeNormals(const Eigen::MatrixX3d &nodeNormals);

public:
  BeamSimulator();

  fea::Summary executeSimulation();
  fea::Summary getResults() const;
  void setSimulation(const SimulationJob &simulationJob);
  void setResultsNodalDisplacementCSVFilepath(const std::string &outputPath);
  void setResultsElementalForcesCSVFilepath(const std::string &outputPath);
};

#endif // BEAMSIMULATOR_HPP
