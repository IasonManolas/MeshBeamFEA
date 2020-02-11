#ifndef COLORBAR_HPP
#define COLORBAR_HPP

#include <Eigen/Dense>
#include <glad/glad.h>
#include <gsl/gsl_util>
#include <igl/colormap.h>
#include <imgui/imgui.h>
#include <memory>
#include <vector>

class Colormap {
  float minValue{0};
  float maxValue{0};
  std::unique_ptr<unsigned char[]> ptrColormapImage;
  GLuint colormapTextureID{0};

  void createVerticalColormapImage(const Eigen::MatrixX3d &rgb);
  void createHorizontalColormapImage(const Eigen::MatrixX3d &rgb);
  void generateImageTexture(const int &imageWidth, const int &imageHeight);

public:
  Colormap();
  Colormap(Eigen::VectorXd values, const igl::ColorMapType &colormapType);
  void generateImage(const Eigen::MatrixX3d &rgb,
                     const bool isVertical = false);
  //  void createColormaps(std::vector<Eigen::VectorXd> vectorOfValues,
  //                       const igl::ColorMapType &colormapType);

  void drawColorbar(const Eigen::Vector3f &backgroundColor) const;
};

#endif // COLORBAR_HPP
