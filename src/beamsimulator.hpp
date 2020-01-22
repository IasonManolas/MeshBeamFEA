#ifndef BEAMSIMULATOR_HPP
#define BEAMSIMULATOR_HPP

#include "beam.hpp"
#include "mesh.hpp"
#include <threed_beam_fea.h>
#include <vector>

struct SimulationOptions {
  std::vector<int> fixedVertices;
  std::vector<std::vector<int>> nodalForces;
};

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
    I2 = (dimensions.b * std::pow(dimensions.h, 3) / 12);
    I3 = (dimensions.h * std::pow(dimensions.b, 3) / 12);
    polarInertia = (I2 + I3);
    G = (material.youngsModulusGPascal * std::pow(10, 9) /
         (2 * (1 + material.poissonsRatio)));
    EA = (material.youngsModulusGPascal * std::pow(10, 9) * crossArea);
    EIz = (material.youngsModulusGPascal * std::pow(10, 9) * I3);
    EIy = (material.youngsModulusGPascal * std::pow(10, 9) * I2);
    GJ = (G * polarInertia);
  }
};

struct NodalForce {
  size_t index;
  int dof;
  double magnitude;
};

class BeamSimulator {
private:
  std::vector<fea::Elem> elements;
  std::vector<fea::Node> nodes;
  std::vector<fea::BC> boundaryConditions;
  std::vector<fea::Force> nodalForces;
  std::vector<Eigen::Vector3d> nodeNormals;
  std::string nodalDisplacementsOutputFilepath;
  std::string nodalForcesOutputFilepath;
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
  void setSimulation(const Eigen::MatrixX3d &nodes,
                     const Eigen::MatrixX2i &elements,
                     const Eigen::MatrixX3d &edgeNormals,
                     const Eigen::VectorXi &fixedNodes,
                     const std::vector<NodalForce> &nodalForces,
                     const std::vector<BeamDimensions> &beamDimensions,
                     const std::vector<BeamMaterial> &beamMaterial);
};

#endif // BEAMSIMULATOR_HPP
