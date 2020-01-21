#ifndef GUI_HPP
#define GUI_HPP

#include "beamsimulator.hpp"
#include "drawer.hpp"
#include <glad/glad.h>
#include <igl/colormap.h>

class ColorbarPlugin {
public:
  int draw_colormap_combo() const;

  void draw_colorbar(const int colormapIndex, float xmin, float xmax,
                     const Eigen::Vector4f &background_color) const;
  void texture_from_colormap(const Eigen::MatrixX3d &rgb, GLuint &id) {
    assert(rgb.cols() == 3);
    const bool isVertical = false;
    int imageWidth;
    int imageHeight;
    unsigned char *imagePtr;
    if (isVertical) {
      imageWidth = 1;
      imageHeight = static_cast<int>(rgb.rows());

      Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic,
                    Eigen::RowMajor>
          cmap;
      if (rgb.maxCoeff() > 1.0) {
        cmap = rgb.cast<unsigned char>();
      } else {
        cmap = (rgb.array() * 255.f).cast<unsigned char>();
      }
      size_t imageDataSize = sizeof(unsigned char) * cmap.size();
      imagePtr = static_cast<unsigned char *>(malloc(imageDataSize));
      memcpy(imagePtr, cmap.data(), imageDataSize);
    } else {
      auto rgbT = rgb.transpose();
      imageWidth = static_cast<int>(rgbT.cols());
      imageHeight = 1;

      Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic,
                    Eigen::ColMajor>
          cmap;
      if (rgbT.maxCoeff() > 1.0) {
        cmap = rgbT.cast<unsigned char>();
      } else {
        cmap = (rgbT.array() * 255.f).cast<unsigned char>();
      }
      assert(cmap.rows() == 3);
      size_t imageDataSize = sizeof(unsigned char) * cmap.size();
      imagePtr = static_cast<unsigned char *>(malloc(imageDataSize));
      memcpy(imagePtr, cmap.data(), imageDataSize);
    }

    if (id == 0) {
      glGenTextures(1, &id);
    }
    glBindTexture(GL_TEXTURE_2D, id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                    GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageWidth, imageHeight, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, imagePtr);
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  void init_colormaps(std::vector<Eigen::VectorXd> vectorOfValues,
                      const igl::ColorMapType &colormapType);

protected:
  std::vector<GLuint> colormaps_;
};

class GUI {
  struct Entries {
    bool shouldDrawEdgeMesh{false};
    struct Simulation {
      struct Force {
        int vertexIndex{0};
        int dof{0};
        float magnitude{1};
      } force;
      std::vector<int> fixedVertices;
      std::vector<NodalForce> nodalForces;
    } simulation;
    struct ViewingOptions {
      igl::ColorMapType chosenColormapType{igl::COLOR_MAP_TYPE_INFERNO};
      NodalForceComponent chosenForceComponent{Fx};
    } viewingOptions;
  } entries;

  struct DrawingDataIDs {
    const std::string worldAxisID = "world axis";
    const std::string edgeMeshID = "original mesh";
    const std::string displacedEdgeMeshID = "displaced mesh";
    const std::string fixedNodesID = "fixed vertices";
    const std::string nodalForcesID = "nodal forces";
    const std::string beamForcesID = "beam forces";
  } drawingDataIDs;

private:
  Drawer drawer;
  BeamSimulator simulator;
  Viewer viewer;
  VCGEdgeMesh edgeMesh;
  ColorbarPlugin colorbar;
  std::vector<std::pair<double, double>> minMaxForcesPerForceComponent;

  void createMenu();
  bool loadEdgeMesh();
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

  // TODO: Could this function incorporate loading triangular meshes? Is there
  // use for triangular meshes in this application?
  static bool populateMesh(const std::string plyFilename,
                           VCGEdgeMesh &edgeMesh);

  void setSimulation(const Eigen::MatrixX3d &nodes,
                     const Eigen::MatrixX2i &elements,
                     const Eigen::VectorXi strfixedVertices,
                     const std::vector<fea::Force> nodalForces);
  void getVertices(const VCGTriMesh &mesh, Eigen::MatrixX3d &vertices) const;
  void getNormals(const VCGTriMesh &mesh, Eigen::MatrixX3d &normals);

  // edgeColors: force component->pairs of colors for each edge
  void computeBeamColors(const std::vector<Eigen::MatrixXd> &edgeColors,
                         const Eigen::MatrixX3d &beamMeshVertices);

  void convertToEigen(const std::vector<std::vector<double>> &edgeForces,
                      std::vector<Eigen::VectorXd> &eigenEdgeForces);
  // edgeForces: force component->pair of forces for each edge
  void convertToColors(const std::vector<Eigen::VectorXd> &edgeForces,
                       std::vector<Eigen::MatrixXd> &edgeColors) const;

public:
  GUI();
  ~GUI();
};

#endif // GUI_HPP
