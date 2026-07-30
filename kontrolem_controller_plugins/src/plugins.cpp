// M16 pluginlib exports: register each shipped control-law factory under the
// kontrolem_control::ControllerFactory base class so the L4 runtime can discover
// and instantiate it by name — no hardcoded if/else. The factory classes and
// their logic live in the ROS-free kontrolem_controllers package; only these
// export macros (and pluginlib) live here, so the impl package stays ROS-free.
#include <pluginlib/class_list_macros.hpp>

#include "kontrolem_control/controller_factory.hpp"
#include "kontrolem_controllers/factories.hpp"

PLUGINLIB_EXPORT_CLASS(kontrolem_controllers::LqrFactory, kontrolem_control::ControllerFactory)
PLUGINLIB_EXPORT_CLASS(kontrolem_controllers::LqgFactory, kontrolem_control::ControllerFactory)
PLUGINLIB_EXPORT_CLASS(kontrolem_controllers::LpvFactory, kontrolem_control::ControllerFactory)
PLUGINLIB_EXPORT_CLASS(kontrolem_controllers::MpcFactory, kontrolem_control::ControllerFactory)
PLUGINLIB_EXPORT_CLASS(kontrolem_controllers::QpFactory, kontrolem_control::ControllerFactory)
PLUGINLIB_EXPORT_CLASS(kontrolem_controllers::WbcFactory, kontrolem_control::ControllerFactory)
PLUGINLIB_EXPORT_CLASS(
  kontrolem_controllers::KinematicGaitFactory, kontrolem_control::ControllerFactory)
