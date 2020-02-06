#ifndef BEAM_HPP
#define BEAM_HPP
#include <Eigen/Dense>
#include <assert.h>
#include <cmath>
#include <gsl/gsl_assert>
#include <iostream>

struct BeamDimensions {
  float b;
  float h;
  BeamDimensions(const float &width, const float &height)
      : b(width), h(height) {}
  BeamDimensions() : b(1), h(1) {}
};

struct BeamMaterial {
  float poissonsRatio;
  float youngsModulusGPascal;
  BeamMaterial(const float &poissonsRatio, const float &youngsModulusGPascal)
      : poissonsRatio(poissonsRatio),
        youngsModulusGPascal(youngsModulusGPascal) {
    Expects(poissonsRatio <= 0.5 && poissonsRatio >= -1);
  }
  BeamMaterial() : poissonsRatio(0.3), youngsModulusGPascal(210) {}
};

#endif // BEAM_HPP
