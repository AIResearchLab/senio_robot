#pragma once

#include <string>

namespace senio_hardware {

struct Joint {
  int id;
  std::string name;
  double pos; //state
  double vel; //state
  double eff; //state
  double cmd; //position control
};

} // namespace senio_hardware
