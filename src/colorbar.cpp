#include "colorbar.hpp"
#include <algorithm>

// void Colormap::createColormaps(std::vector<Eigen::VectorXd> vectorOfValues,
//                               const igl::ColorMapType &colormapType) {
//  colormaps_.clear();
//  for (Eigen::VectorXd &values : vectorOfValues) {
//    std::sort(values.data(), values.data() + values.size());
//    Eigen::MatrixX3d rgbColors;
//    Eigen::MatrixXd valuesMatrix(values);
//    igl::colormap(colormapType, valuesMatrix, true, rgbColors);
//    GLuint id = 0;
//    texture_from_colormap(rgbColors, id);
//    colormaps_.push_back(id);
//  }
//}

void Colormap::createVerticalColormapImage(const Eigen::MatrixX3d &rgb) {
  int imageWidth;
  int imageHeight;
  Expects(ptrColormapImage == nullptr);
  Expects(rgb.cols() == 3);

  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      cmap;
  if (rgb.maxCoeff() > 1.0) {
    cmap = rgb.cast<unsigned char>();
  } else {
    cmap = (rgb.array() * 255.f).cast<unsigned char>();
  }
  Ensures(cmap.cols() == 3);
  const size_t imageDataSize = sizeof(unsigned char) * cmap.size();
  ptrColormapImage = std::make_unique<unsigned char[]>(imageDataSize);
  memcpy(ptrColormapImage.get(), cmap.data(), imageDataSize);
  Ensures(ptrColormapImage != nullptr);
}

void Colormap::createHorizontalColormapImage(const Eigen::MatrixX3d &rgb) {
  Expects(ptrColormapImage == nullptr);
  auto rgbT = rgb.transpose();
  const int imageWidth = rgbT.cols();
  const int imageHeight = 1;

  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>
      cmap;
  if (rgbT.maxCoeff() > 1.0) {
    cmap = rgbT.cast<unsigned char>();
  } else {
    cmap = (rgbT.array() * 255.f).cast<unsigned char>();
  }
  Ensures(cmap.rows() == 3);
  const size_t imageDataSize = sizeof(unsigned char) * cmap.size();
  ptrColormapImage = std::make_unique<unsigned char[]>(imageDataSize);
  memcpy(ptrColormapImage.get(), cmap.data(), imageDataSize);
  Ensures(ptrColormapImage != nullptr);
}

void Colormap::generateImageTexture(const int &imageWidth,
                                    const int &imageHeight) {
  const bool hasPositiveImageDimensions = imageWidth > 0 && imageHeight > 0;
  const bool noTextureWasGenerated = colormapTextureID == 0;
  Expects(hasPositiveImageDimensions && noTextureWasGenerated);

  glGenTextures(1, &colormapTextureID);
  glBindTexture(GL_TEXTURE_2D, colormapTextureID);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                  GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imageWidth, imageHeight, 0, GL_RGB,
               GL_UNSIGNED_BYTE, ptrColormapImage.get());
  glGenerateMipmap(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, 0);
}

Colormap::Colormap() {}

Colormap::Colormap(Eigen::VectorXd values,
                   const igl::ColorMapType &colormapType) {
  minValue = values.minCoeff();
  maxValue = values.maxCoeff();
  std::sort(values.data(), values.data() + values.size());
  Eigen::MatrixX3d rgbColors;
  igl::colormap(colormapType, values, true, rgbColors);
  generateImage(rgbColors);
}

// Draws the colorbar in the ImGui
void Colormap::drawColorbar(const Eigen::Vector3f &backgroundColor) const {
  ImVec4 color(0, 0, 0, 1);
  // http://stackoverflow.com/a/3943023/112731
  if (backgroundColor[0] * 0.299 + backgroundColor[1] * 0.587 +
          backgroundColor[2] * 0.114 <=
      186) {
    color = ImVec4(1, 1, 1, 1);
  }
  float w = 100;
  float h = 20;
  ImGui::BeginGroup();
  ImGui::BeginGroup();
  ImGui::Image(reinterpret_cast<ImTextureID>(colormapTextureID), ImVec2(w, h));
  ImGui::EndGroup();
  ImGui::TextColored(color, "%.3g", minValue);
  ImGui::SameLine();
  ImGui::Dummy(ImVec2(w - 40, 0)); // TODO: -40 is wrong since the space
                                   // depends of the digits of min and max
  ImGui::SameLine();
  ImGui::TextColored(color, "%.3g", maxValue);
  ImGui::EndGroup();
}

void Colormap::generateImage(const Eigen::MatrixX3d &rgb,
                             const bool isVertical /*=false*/) {
  if (isVertical) {
    createVerticalColormapImage(rgb);
    generateImageTexture(1, rgb.rows());
  } else {
    createHorizontalColormapImage(rgb);
    generateImageTexture(rgb.rows(), 1);
  }
}
