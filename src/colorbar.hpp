#ifndef COLORBAR_HPP
#define COLORBAR_HPP

#include <Eigen/Dense>
#include <glad/glad.h>
#include <igl/colormap.h>
#include <imgui/imgui.h>

class Colorbar {
public:
  Colorbar();
  int draw_colormap_combo() const;

  void draw_colorbar(const int colormapIndex, float xmin, float xmax,
                     const Eigen::Vector4f &background_color) const;
  void texture_from_colormap(const Eigen::MatrixX3d &rgb, GLuint &id);

  void init_colormaps(std::vector<Eigen::VectorXd> vectorOfValues,
                      const igl::ColorMapType &colormapType);

protected:
  std::vector<GLuint> colormaps_;
};

#endif // COLORBAR_HPP
