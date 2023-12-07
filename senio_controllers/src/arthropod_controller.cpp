#include <senio_controllers/arthropod_controller.hpp>
#include <pluginlib/class_list_macros.h>

namespace position_controllers
{
using ArthropodCartesianTrajectoryController =
    senio_controllers::ArthropodController<hardware_interface::PositionJointInterface>;
}  // namespace position_controllers

// register this class as a controller
PLUGINLIB_EXPORT_CLASS(position_controllers::ArthropodCartesianTrajectoryController,
                       controller_interface::ControllerBase)
