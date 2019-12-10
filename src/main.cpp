#include "beamsimulator.hpp"
#include "drawer.hpp"
#include "mesh.hpp"
#include <filesystem>
#include <iostream>
#include <json.hpp>
#include <stdexcept>

struct ConfigurationFile {
  SimulationOptions simulationProperties;
  bool shouldDrawResults = false;
  std::string plyFilename;
};

void populateMesh(const std::string plyFilename, VCGMesh &mesh) {
  int returnValue =
      vcg::tri::io::ImporterPLY<VCGMesh>::Open(mesh, plyFilename.c_str());
  if (returnValue != 0) {
    std::cout << "Unable to open %s for '%s'\n" + plyFilename +
                     vcg::tri::io::ImporterPLY<VCGMesh>::ErrorMsg(returnValue)
              << std::endl;
    throw std::runtime_error{"Unable to load the ply file."};
  }
  std::cout << plyFilename << " was loaded successfuly." << std::endl;
  vcg::tri::UpdateTopology<VCGMesh>::AllocateEdge(mesh);
  vcg::tri::UpdateNormal<VCGMesh>::PerVertexNormalized(mesh);
  mesh.printInfo();
}

void parseConfigurationFile(const std::string configurationFilename,
                            ConfigurationFile &configurationFile) {
  ifstream inFile(configurationFilename);
  std::string jsonContents((std::istreambuf_iterator<char>(inFile)),
                           std::istreambuf_iterator<char>());
  nlohmann::json jsonFile(nlohmann::json::parse(jsonContents));

  const std::string jsonPlyFilename = jsonFile["plyFilename"];
  if (!std::filesystem::exists(jsonPlyFilename) || jsonPlyFilename.empty()) {
    throw std::runtime_error{"Input ply file does not exist."};
  }
  configurationFile.plyFilename = jsonPlyFilename;

  const std::string jsonNodalDisplacementsOutputPath =
      jsonFile["nodalDisplacementsCSV"];
  if (!jsonNodalDisplacementsOutputPath.empty()) {
    configurationFile.simulationProperties.nodalDisplacementsOutputFilepath =
        filesystem::absolute(jsonNodalDisplacementsOutputPath).string();
  }
  const std::string jsonNodalForcesOutputPath = jsonFile["nodalForcesCSV"];
  if (!jsonNodalForcesOutputPath.empty()) {
    configurationFile.simulationProperties.nodalForcesOutputFilepath =
        filesystem::absolute(jsonNodalForcesOutputPath).string();
  }

  std::vector<int> fixedVertices = jsonFile["fixedVertices"];
  configurationFile.simulationProperties.fixedVertices = fixedVertices;

  std::vector<std::vector<int>> forces = jsonFile["forces"];
  configurationFile.simulationProperties.nodalForces = forces;

  bool jsonShouldDrawResults = jsonFile["shouldDrawResults"];
  configurationFile.shouldDrawResults = jsonShouldDrawResults;
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    throw std::runtime_error{"Wrong number of arguments."};
  }
  const std::string configurationFilename = argv[1];
  // check if file exists using std::filesystem
  if (!std::filesystem::exists(configurationFilename)) {
    throw std::runtime_error{"Configuration filepath does not exist."};
    return 1;
  }

  ConfigurationFile configurationFile;
  parseConfigurationFile(configurationFilename, configurationFile);

  VCGMesh mesh;
  populateMesh(configurationFile.plyFilename, mesh);
  configurationFile.simulationProperties.fixedVertices.clear();
  for (const MyVertex &v : mesh.vert) {
    vcg::Color4b red = vcg::Color4b::Red;
    if (v.C().operator==(red)) {
      configurationFile.simulationProperties.fixedVertices.push_back(
          static_cast<int>(vcg::tri::Index(mesh, v)));
    }
  }

  BeamSimulator simulator(mesh, configurationFile.simulationProperties);
  fea::Summary simulationSummary = simulator.executeSimulation();

  if (configurationFile.shouldDrawResults) {
    SimulationResultsDrawer drawer(mesh, simulationSummary.nodal_displacements,
                                   simulationSummary.nodal_forces);
    drawer.draw();
  }

  return 0;
}
