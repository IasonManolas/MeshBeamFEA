#ifndef BEAM_HPP
#define BEAM_HPP
#include <assert.h>
#include <cmath>
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
    assert(poissonsRatio <= 0.5 && poissonsRatio >= -1);
    std::cout << "non-default constructor was called" << std::endl;
  }
  BeamMaterial() : poissonsRatio(0.3), youngsModulusGPascal(210) {
    std::cout << "Default constructor was called" << std::endl;
  }
};

#endif // BEAM_HPP
