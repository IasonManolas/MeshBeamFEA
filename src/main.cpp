#include "gui.hpp"
#include <filesystem>
#include <iostream>
#include <json.hpp>
#include <stdexcept>
#include <string>

struct ConfigurationFile {
  SimulationOptions simulationProperties;
  bool shouldDrawResults = false;
  std::string plyFilename;
  std::string nodalDisplacementsOutputFilepath;
  std::string nodalForcesOutputFilepath;
};

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
    configurationFile.nodalDisplacementsOutputFilepath =
        filesystem::absolute(jsonNodalDisplacementsOutputPath).string();
  }
  const std::string jsonNodalForcesOutputPath = jsonFile["nodalForcesCSV"];
  if (!jsonNodalForcesOutputPath.empty()) {
    configurationFile.nodalForcesOutputFilepath =
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
  GUI simulationGui;
  //  if (argc != 2) {
  //    throw std::runtime_error{"Wrong number of arguments."};
  //  }
  //  const std::string configurationFilename = argv[1];
  //  // check if file exists using std::filesystem
  //  if (!std::filesystem::exists(configurationFilename)) {
  //    throw std::runtime_error{"Configuration filepath does not exist."};
  //    return 1;
  //  }

  //  ConfigurationFile configurationFile;
  //  parseConfigurationFile(configurationFilename, configurationFile);

  //  VCGMesh mesh;
  //  populateMesh(configurationFile.plyFilename, mesh);

  //  // configurationFile.simulationProperties.fixedVertices.clear();
  //  // for (const VCGVertex &v : mesh.vert) {
  //  //  vcg::Color4b red = vcg::Color4b::Red;
  //  //  if (v.C().operator==(red)) {
  //  //    configurationFile.simulationProperties.fixedVertices.push_back(
  //  //        static_cast<int>(vcg::tri::Index(mesh, v)));
  //  //  }
  //  //}

  //  BeamSimulator simulator(mesh, configurationFile.simulationProperties);
  //  fea::Summary simulationSummary = simulator.executeSimulation();

  //  if (configurationFile.shouldDrawResults) {
  //    Drawer drawer(mesh, simulationSummary.nodal_displacements,
  //                  simulationSummary.element_forces);
  //    drawer.draw();
  //  }

  return 0;
}
