#ifndef BEAMSIMULATOR_HPP
#define BEAMSIMULATOR_HPP

#include "beam.hpp"
#include "mesh.hpp"
#include <filesystem>
#include <gsl/gsl>
#include <igl/list_to_matrix.h>
#include <json.hpp>
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
                           const BeamMaterial &material);
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

struct SimulationResults {
  std::vector<Eigen::VectorXd> edgeForces; ///< Force values per force component
                                           ///< #force components x #edges
  Eigen::MatrixXd
      nodalDisplacements; ///< The displacement of each node #nodes x 3
  SimulationResults(const fea::Summary &feaSummary);
  SimulationResults() {}
};

/** \struct FeaSimulationJob
 *  \brief A struct for grouping together the simulation data used by the
 * threed-beam-fea library
 */
struct FeaSimulationJob {
  std::vector<fea::Elem> elements;
  std::vector<fea::Node> nodes;
  std::vector<fea::BC> boundaryConditions;
  std::vector<fea::Force> nodalForces;

  void setElements(const SimulationJob &job);

  void setNodes(const SimulationJob &job);

  FeaSimulationJob(const SimulationJob &job);
  FeaSimulationJob();
  bool isEmpty() const;
  void clear();

private:
  void setFixedNodes(const Eigen::VectorXi &fixedNodes);
  void setNodalForces(const std::vector<NodalForce> &nodalForces);
};

class BeamSimulator {
public:
  BeamSimulator();

  SimulationResults executeSimulation();
  SimulationResults getResults() const;
  void setSimulation(const SimulationJob &simulationJob);
  void setResultsNodalDisplacementCSVFilepath(const std::string &outputPath);
  void setResultsElementalForcesCSVFilepath(const std::string &outputPath);

private:
  FeaSimulationJob simulationJob;
  std::string nodalDisplacementsOutputFilepath{"nodal_displacement.csv"};
  std::string elementalForcesOutputFilepath{"elemental_forces.csv"};
  SimulationResults results;

  static void printInfo(const fea::Job &job);
};

#endif // BEAMSIMULATOR_HPP
