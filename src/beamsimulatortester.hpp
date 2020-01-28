#ifndef BEAMSIMULATORTESTER_HPP
#define BEAMSIMULATORTESTER_HPP
#include "beamsimulator.hpp"
#include <filesystem>

class BeamSimulatorTester {
  struct CantileverBeamProperties {
    float beamLength{1};
    size_t numberOfBeams{1};
    float forceMagnitude{1000};
    float youngsModulusGPascal{210};
    float poissonsRatio{0.3};
    float b{0.1};
    float h{0.2};
    Eigen::Vector3d XAxis{1, 0, 0};
    Eigen::Vector3d YAxis{0, 1, 0};
  };
  struct CantileverBeam : public CantileverBeamProperties {
    Eigen::MatrixX3d getNodes() const;
    Eigen::MatrixX2i getElements() const;
    Eigen::MatrixX3d getNormals() const;
    Eigen::VectorXi getFixedNodes() const;
    std::vector<NodalForce> getNodalForces() const;
    std::vector<BeamDimensions> getBeamDimensions() const;
    std::vector<BeamMaterial> getBeamMaterial() const;
  };
  BeamSimulator beamSimulator;
  void launchCantileverTest();
  /**
   */
  void setSimulation(const CantileverBeam &cantileverBeam);
  void
  setSimulationResultsCSV(const Eigen::Vector3d &beamXAxis,
                          const size_t &numberOfBeams,
                          const filesystem::__cxx11::path &resultsFolderPath);

public:
  BeamSimulatorTester();
  void launchTests();
};

#endif // BEAMSIMULATORTESTER_HPP
