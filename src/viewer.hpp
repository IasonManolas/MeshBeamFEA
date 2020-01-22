#ifndef VIEWER_HPP
#define VIEWER_HPP
#include <gsl/gsl>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

using ViewerType = igl::opengl::glfw::Viewer;
using ViewerData = igl::opengl::ViewerData;
using ViewerPlugin = igl::opengl::glfw::ViewerPlugin;
using RotationType = igl::opengl::ViewerCore::RotationType;

class Viewer {
  ViewerType iglViewer;
  /*Holds the pairs of data entry keys that are present in the iglViewer and
   * their corresponding indices */
  std::unordered_map<std::string, size_t> generatedDrawingDataMap;
  std::unordered_map<std::string, size_t> generatedPluginsMap;
  Eigen::Quaternionf trackballAngle{Eigen::Quaternionf::Identity()};
  bool shouldUseOrthographicView{true};

public:
  ViewerData &getDrawingData(const std::string &drawingDataIdentifier);
  ~Viewer();
  void addDrawingData(const std::string &dataIdentifier,
                      const ViewerData drawingData);
  bool hasDrawingData(const std::string &drawingDataID);
  bool hasPlugin(const std::string &pluginID);
  void setDrawingDataVisibility(const std::string &drawingDataID,
                                const bool &shouldBeVisible);
  void addPlugin(const std::string &pluginIdentifier,
                 gsl::owner<ViewerPlugin *> pPlugin);
  void deleteDrawingData(const std::string &drawingDataID);
  void setColors(const std::string &drawingDataID,
                 const Eigen::MatrixX3d &colors);
  void launch();
  void draw();
  void clearDrawingData();
  bool loadScene();
  bool saveScene();
  void centerCamera(const std::string &drawingDataID);
  void snapCanonicalView();
  RotationType getRotationType();
  void setRotationType(const RotationType &newRotationType);
  bool getShouldUseOrthographicView() const;
  void setUseOrthographicView(bool value);
  void setShouldInvertNormals(const std::string &drawingDataID,
                              const bool &shouldInvertNormals);
};

#endif // VIEWER_HPP
