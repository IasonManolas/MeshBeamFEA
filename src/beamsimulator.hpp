#ifndef BEAMSIMULATOR_HPP
#define BEAMSIMULATOR_HPP

#include <vector>

#include "mesh.hpp"
#include <threed_beam_fea.h>

struct BeamProperties {
  double EA;
  double EIz;
  double EIy;
  double GJ;

  BeamProperties() {
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

struct SimulationOptions {
  BeamProperties beamProperties;
  std::vector<int> fixedVertices;
  std::vector<std::vector<int>> nodalForces;
  std::string nodalDisplacementsOutputFilepath;
  std::string nodalForcesOutputFilepath;
  SimulationOptions() {}
};

class BeamSimulator {

  const VCGMesh &mesh;
  std::vector<fea::Elem> elements;
  std::vector<fea::Node> nodes;
  std::vector<fea::BC> boundaryConditions;
  std::vector<fea::Force> nodalForces;
  const SimulationOptions &properties;

  void setNodes(const std::vector<fea::Node> &nodes);
  void setElements(const std::vector<fea::Elem> &elements);
  void populateNodes(const VCGMesh &mesh);
  void populateElements(const VCGMesh &mesh);
  void fixVertices(const std::vector<int> vertices);
  void setNodalForces(const std::vector<std::vector<int>> vertexForces);
  static void printInfo(const fea::Job &job);
  void reset();

public:
  BeamSimulator(const VCGMesh &m, const SimulationOptions &props);

  fea::Summary executeSimulation();
};

#endif // BEAMSIMULATOR_HPP
