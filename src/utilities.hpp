#ifndef UTILITIES_H
#define UTILITIES_H

#include <gsl/gsl_util>
#include <regex>

namespace Utilities {
inline void parseIntegers(const std::string &str,
                          std::vector<gsl::index> &result) {
  typedef std::regex_iterator<std::string::const_iterator> re_iterator;
  typedef re_iterator::value_type re_iterated;

  std::regex re("(\\d+)");

  re_iterator rit(str.begin(), str.end(), re);
  re_iterator rend;

  std::transform(rit, rend, std::back_inserter(result),
                 [](const re_iterated &it) { return std::stoi(it[1]); });
}

std::string convertToLowercase(const std::string &s) {
  std::string lowercase;
  std::transform(s.begin(), s.end(), lowercase.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return lowercase;
}
bool hasExtension(const std::string &filename, const std::string &extension) {
  const std::filesystem::path path(filename);
  if (!path.has_extension()) {
    std::cerr << "Error: No file extension found in " << filename << std::endl;
    return false;
  }

  const std::string detectedExtension = path.extension().string();

  if (convertToLowercase(detectedExtension) != convertToLowercase(extension)) {
    std::cerr << "Error: detected extension is " + detectedExtension +
                     " and not " + extension
              << std::endl;
    return false;
  }
  return true;
}

} // namespace Utilities

namespace ConfigurationFile {

inline void getPlyFilename(const std::string jsonFilepath,
                           std::string &plyFilename) {
  ifstream inFile(jsonFilepath);
  std::string jsonContents((std::istreambuf_iterator<char>(inFile)),
                           std::istreambuf_iterator<char>());
  nlohmann::json jsonFile(nlohmann::json::parse(jsonContents));

  if (jsonFile.contains("plyFilename")) {
    plyFilename = jsonFile["plyFilename"];
  }
}

inline void getNodalForces(const std::string jsonFilepath,
                           std::vector<NodalForce> &nodalForces) {
  ifstream inFile(jsonFilepath);
  std::string jsonContents((std::istreambuf_iterator<char>(inFile)),
                           std::istreambuf_iterator<char>());
  nlohmann::json jsonFile(nlohmann::json::parse(jsonContents));

  nodalForces.clear();
  if (jsonFile.contains("forces")) {
    std::vector<std::vector<double>> forces = jsonFile["forces"];
    nodalForces.resize(forces.size());
    for (gsl::index forceIndex = 0; forceIndex < forces.size(); forceIndex++) {
      const NodalForce nf{static_cast<gsl::index>(forces[forceIndex][0]),
                          static_cast<gsl::index>(forces[forceIndex][1]),
                          forces[forceIndex][2]};
      Expects(nf.dof >= 0 && nf.dof < 6 && nf.index >= 0 && nf.magnitude >= 0);
      nodalForces[forceIndex] = nf;
    }
  }
}

inline void getFixedVertices(const std::string jsonFilepath,
                             std::vector<gsl::index> &fixedVertices) {
  ifstream inFile(jsonFilepath);
  std::string jsonContents((std::istreambuf_iterator<char>(inFile)),
                           std::istreambuf_iterator<char>());
  nlohmann::json jsonFile(nlohmann::json::parse(jsonContents));

  fixedVertices.clear();
  if (jsonFile.contains("fixedVertices")) {
    fixedVertices =
        static_cast<std::vector<gsl::index>>(jsonFile["fixedVertices"]);
  }
}

struct SimulationScenario {
  std::string edgeMeshFilename;
  std::vector<gsl::index> fixedVertices;
  std::vector<NodalForce> nodalForces;
};

void to_json(nlohmann::json &json, const SimulationScenario &scenario) {
  json["plyFilename"] = scenario.edgeMeshFilename;
  if (!scenario.fixedVertices.empty()) {
    json["fixedVertices"] = scenario.fixedVertices;
  }
  if (!scenario.nodalForces.empty()) {
    std::vector<std::tuple<gsl::index, gsl::index, float>> forces;
    std::transform(scenario.nodalForces.begin(), scenario.nodalForces.end(),
                   std::back_inserter(forces), [](const NodalForce &f) {
                     return std::tuple<gsl::index, gsl::index, float>{
                         f.index, f.dof, f.magnitude};
                   });
    json["forces"] = forces;
  }
}
} // namespace ConfigurationFile

#endif // UTILITIES_H
