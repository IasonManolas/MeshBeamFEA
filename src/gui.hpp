#ifndef GUI_HPP
#define GUI_HPP

#include "beamsimulator.hpp"
#include "colorbar.hpp"
#include "drawer.hpp"
#include <igl/colormap.h>

class GUI : public Colorbar {
  struct Entries {
    bool shouldDrawEdgeMesh{false};
    std::string plyFilename;
    struct Simulation {
      struct Force {
        gsl::index vertexIndex{0};
        int dof{0};
        float magnitude{1};
      } force;
      void clear() {
        fixedVertices.clear();
        strFixedVertices.clear();
        nodalForces.clear();
      }
      std::vector<gsl::index> fixedVertices;
      std::string strFixedVertices;
      std::vector<NodalForce> nodalForces;
    } simulation;
    struct ViewingOptions {
      igl::ColorMapType chosenColormapType{igl::COLOR_MAP_TYPE_VIRIDIS};
      NodalForceComponent chosenForceComponent{Fx};
      bool shouldDrawWorldAxis{false};
    } viewingOptions;
  } entries;

  struct DrawingDataIDs {
    const std::string worldAxisID = "world axis";
    const std::string edgeMeshID = "original mesh";
    const std::string displacedEdgeMeshID = "displaced mesh";
    const std::string fixedNodesID = "fixed vertices";
    const std::string nodalForcesID = "nodal forces";
  } drawingDataIDs;

private:
  Drawer drawer;
  BeamSimulator simulator;
  Viewer viewer;
  VCGEdgeMesh edgeMesh;
  std::vector<std::pair<double, double>> minMaxForcesPerForceComponent;
  bool shouldDrawEdgeMesh = false;
  bool shouldDrawFixedVertices = false;
  bool shouldDrawNodalForces = false;

  void createMenu();
  void drawWorldAxis();
  void clearViewer();
  bool loadEdgeMesh();
  bool loadEdgeMesh(const string &meshFilenameString);
  void setSimulation();
  void drawEdgeMesh();
  void executeSimulation();
  void addNodalForce();
  void drawNodalForces();
  void drawDisplacedEdgeMesh(
      const std::vector<std::vector<double>> &nodalDisplacements);
  void drawEdgeForces(const std::vector<std::vector<double>> &edgeForces);
  void drawColorbar();
  void drawColorTypeCombo();
  void convertToEigen(const std::vector<std::vector<double>> &edgeForces,
                      std::vector<Eigen::VectorXd> &eigenEdgeForces);
  // edgeForces: force component->pair of forces for each edge
  void convertToColors(const std::vector<Eigen::VectorXd> &edgeForces,
                       std::vector<Eigen::MatrixXd> &edgeColors) const;

  void drawSimulationResults(
      const std::vector<std::vector<double>> &nodalDisplacements,
      const std::vector<std::vector<double>> &elementForces);

  void drawFixedVertices(const std::vector<gsl::index> &vertices);
  void updateViewer();

  void drawFixedVertices();

public:
  GUI();
  ~GUI();
};

#endif // GUI_HPP
