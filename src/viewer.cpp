#include "viewer.hpp"

void Viewer::setUseOrthographicView(bool value) {
  shouldUseOrthographicView = value;
}

void Viewer::setShouldInvertNormals(const std::string &drawingDataID,
                                    const bool &shouldInvertNormals) {
  if (!hasDrawingData(drawingDataID)) {
    return;
  }
  std::cout << "Inverting " + drawingDataID << std::endl;
  getDrawingData(drawingDataID).invert_normals = shouldInvertNormals;
  getDrawingData(drawingDataID).dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
}

ViewerData &Viewer::getDrawingData(const std::string &drawingDataIdentifier) {
  return iglViewer.data_list[generatedDrawingDataMap[drawingDataIdentifier]];
}

Viewer::~Viewer() {
  for (ViewerPlugin *pPlugin : iglViewer.plugins) {
    delete pPlugin;
  }
}

void Viewer::addDrawingData(const std::string &dataIdentifier,
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

bool Viewer::hasDrawingData(const std::string &drawingDataID) {
  return generatedDrawingDataMap.count(drawingDataID) != 0;
}

bool Viewer::hasPlugin(const std::string &pluginID) {
  return generatedDrawingDataMap.count(pluginID) != 0;
}

void Viewer::setDrawingDataVisibility(const std::string &drawingDataID,
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

void Viewer::addPlugin(const std::string &pluginIdentifier,
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

void Viewer::deleteDrawingData(const std::string &drawingDataID) {
  if (!hasDrawingData(drawingDataID)) {
    std::cerr << "Error: The drawing data identifier " + drawingDataID +
                     " was not found. Entry can not be deleted."
              << std::endl;
    return;
  }
  // decrease index of elements in map that come after the element to be
  // deleted in the vector of drawing data
  const gsl::index drawingDataIndexToBeDeleted =
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

void Viewer::setColors(const std::string &drawingDataID,
                       const Eigen::MatrixX3d &colors) {
  if (!hasDrawingData(drawingDataID))
    return;
  getDrawingData(drawingDataID).set_colors(colors);
}

void Viewer::launch() { iglViewer.launch(); }

void Viewer::draw() { iglViewer.draw(); }

void Viewer::clearDrawingData() {
  iglViewer.data_list.clear();
  iglViewer.data_list.push_back(ViewerData());
  generatedDrawingDataMap.clear();
}

void Viewer::centerCamera(const std::string &drawingDataID) {
  iglViewer.core().align_camera_center(getDrawingData(drawingDataID).V,
                                       getDrawingData(drawingDataID).F);
}

void Viewer::snapCanonicalView() { iglViewer.snap_to_canonical_quaternion(); }

RotationType Viewer::getRotationType() {
  return iglViewer.core().rotation_type;
}

void Viewer::setRotationType(const RotationType &newRotationType) {
  if (newRotationType != iglViewer.core().rotation_type) {
    if (newRotationType == RotationType::ROTATION_TYPE_NO_ROTATION) {
      trackballAngle = iglViewer.core().trackball_angle;
      shouldUseOrthographicView = iglViewer.core().orthographic;
      iglViewer.core().trackball_angle = Eigen::Quaternionf::Identity();
      iglViewer.core().orthographic = true;
    } else if (iglViewer.core().rotation_type ==
               RotationType::ROTATION_TYPE_NO_ROTATION) {
      iglViewer.core().trackball_angle = trackballAngle;
      iglViewer.core().orthographic = shouldUseOrthographicView;
    }
    iglViewer.core().set_rotation_type(newRotationType);
  }
}

bool Viewer::getShouldUseOrthographicView() const {
  return shouldUseOrthographicView;
}
