#ifndef VIEWER_HPP
#define VIEWER_HPP
#include <gsl/gsl>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

using ViewerType = igl::opengl::glfw::Viewer;
using ViewerData = igl::opengl::ViewerData;
using ViewerPlugin = igl::opengl::glfw::ViewerPlugin;

class Viewer {
  ViewerType iglViewer;
  /*Holds the pairs of data entry keys that are present in the iglViewer and
   * their corresponding indices */
  std::unordered_map<std::string, size_t> generatedDrawingDataMap;
  std::unordered_map<std::string, size_t> generatedPluginsMap;

public:
  ViewerData &getDrawingData(const std::string &drawingDataIdentifier) {
    return iglViewer.data_list[generatedDrawingDataMap[drawingDataIdentifier]];
  }
  ~Viewer() {
    for (ViewerPlugin *pPlugin : iglViewer.plugins) {
      delete pPlugin;
    }
  }
  void addDrawingData(const std::string &dataIdentifier,
                      const ViewerData drawingData) {
    if (hasDrawingData(dataIdentifier)) {
      const std::string error{
          "Error: Data identifier " + dataIdentifier +
          " is not present in the viewer. Can't set visibility."};
      std::cerr << error << std::endl;
      return;
    }
    iglViewer.data_list.push_back(drawingData);
    generatedDrawingDataMap[dataIdentifier] = iglViewer.data_list.size() - 1;
  }

  bool hasDrawingData(const std::string &drawingDataID) {
    return generatedDrawingDataMap.count(drawingDataID) != 0;
  }

  bool hasPlugin(const std::string &pluginID) {
    return generatedDrawingDataMap.count(pluginID) != 0;
  }

  void setDrawingDataVisibility(const std::string &drawingDataID,
                                const bool &shouldBeVisible) {
    if (!hasDrawingData(drawingDataID)) {
      const std::string error{
          "Error: Data identifier " + drawingDataID +
          " is not present in the viewer. Can't set visibility."};
      std::cerr << error << std::endl;
      return;
    }
    getDrawingData(drawingDataID).set_visible(shouldBeVisible);
  }

  void addPlugin(const std::string &pluginIdentifier,
                 gsl::owner<ViewerPlugin *> pPlugin) {
    if (hasPlugin(pluginIdentifier)) {
      const std::string error{"Error: Plugin identifier " + pluginIdentifier +
                              " is already present in the viewer."};
      std::cerr << error << std::endl;
      return;
    }
    iglViewer.plugins.push_back(pPlugin);
    generatedPluginsMap[pluginIdentifier] = iglViewer.plugins.size() - 1;
  }

  void deleteDrawingData(const std::string &drawingDataID) {
    if (!hasDrawingData(drawingDataID)) {
      std::cerr << "Error: The drawing data identifier " + drawingDataID +
                       " was not found. Entry can not be deleted."
                << std::endl;
      return;
    }
    // decrease index of elements in map that come after the element to be
    // deleted in the vector of drawing data
    const size_t drawingDataIndexToBeDeleted =
        generatedDrawingDataMap[drawingDataID];
    for (auto &drawingDataEntryPair : generatedDrawingDataMap) {
      if (drawingDataEntryPair.second > drawingDataIndexToBeDeleted) {
        drawingDataEntryPair.second--;
      }
    }
    // delete map entry
    generatedDrawingDataMap.erase(drawingDataID);
    // delete drawing data
    iglViewer.data_list.erase(iglViewer.data_list.begin() +
                              drawingDataIndexToBeDeleted);
  }

  void setColors(const std::string &drawingDataID,
                 const Eigen::MatrixX3d &colors) {
    if (!hasDrawingData(drawingDataID))
      return;
    getDrawingData(drawingDataID).set_colors(colors);
  }

  void launch() { iglViewer.launch(); }
  void draw() { iglViewer.draw(); }
  void clearDrawingData() {
    iglViewer.data_list.clear();
    iglViewer.data_list.push_back(ViewerData());
    generatedDrawingDataMap.clear();
  }
};

#endif // VIEWER_HPP
