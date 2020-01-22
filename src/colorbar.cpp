#include "colorbar.hpp"

void Colorbar::init_colormaps(std::vector<Eigen::VectorXd> vectorOfValues,
                              const igl::ColorMapType &colormapType) {
  colormaps_.clear();
  for (Eigen::VectorXd &values : vectorOfValues) {
    std::sort(values.data(), values.data() + values.size());
    Eigen::MatrixX3d rgbColors;
    Eigen::MatrixXd valuesMatrix(values);
    igl::colormap(colormapType, valuesMatrix, true, rgbColors);
    GLuint id = 0;
    texture_from_colormap(rgbColors, id);
    colormaps_.push_back(id);
  }
}

Colorbar::Colorbar() {}

// Draws a combo box for selecting the colormap
int Colorbar::draw_colormap_combo() const {}

// Draws the actual colorbar with min/max values
void Colorbar::draw_colorbar(const int colormapIndex, float xmin, float xmax,
                             const Eigen::Vector4f &background_color) const {
  ImVec4 color(0, 0, 0, 1);
  auto rgb = background_color;
  // http://stackoverflow.com/a/3943023/112731
  if (rgb[0] * 0.299 + rgb[1] * 0.587 + rgb[2] * 0.114 <= 186) {
    color = ImVec4(1, 1, 1, 1);
  }
  float w = 100;
  float h = 20;
  ImGui::BeginGroup();
  ImGui::BeginGroup();
  ImGui::Image(reinterpret_cast<ImTextureID>(colormaps_[colormapIndex]),
               ImVec2(w, h));
  ImGui::EndGroup();
  ImGui::TextColored(color, "%.3g", xmin);
  ImGui::SameLine();
  ImGui::Dummy(ImVec2(w - 40, 0)); // TODO: -40 is wrong since the space
                                   // depends of the digits of min and max
  ImGui::SameLine();
  ImGui::TextColored(color, "%.3g", xmax);
  ImGui::EndGroup();
}

void Colorbar::texture_from_colormap(const Eigen::MatrixX3d &rgb, GLuint &id) {
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
