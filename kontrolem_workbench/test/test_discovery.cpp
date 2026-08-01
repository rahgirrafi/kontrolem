// Integration: the workbench's discovery sees the SAME registry the runtime
// uses — all seven shipped laws plus the out-of-tree example must be found by
// name, each with a non-empty parameter spec (except laws that genuinely have
// none). Run with the workspace sourced:
//   source install/setup.bash && ./build/kontrolem_workbench/test_discovery
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>

#include "kontrolem_control/controller_factory.hpp"

using kontrolem_control::ControllerFactory;

int main()
{
  pluginlib::ClassLoader<ControllerFactory> loader(
    "kontrolem_control", "kontrolem_control::ControllerFactory");

  std::map<std::string, std::shared_ptr<ControllerFactory>> laws;
  for (const auto & cls : loader.getDeclaredClasses()) {
    auto f = loader.createSharedInstance(cls);
    laws[f->name()] = f;
  }

  int failures = 0;
  const std::vector<std::string> expected = {
    "lqr", "lqg", "lpv", "mpc", "qp", "wbc", "kinematic_gait", "example"};
  for (const auto & name : expected) {
    if (laws.find(name) == laws.end()) {
      std::cout << "  FAIL: law '" << name << "' not discovered\n";
      ++failures;
    } else if (laws[name]->parameter_spec().empty()) {
      std::cout << "  FAIL: law '" << name << "' has an empty parameter spec\n";
      ++failures;
    }
  }
  if (laws.size() < expected.size()) {
    std::cout << "  FAIL: discovered only " << laws.size() << " laws\n";
    ++failures;
  }

  std::cout << "test_discovery: " << (failures == 0 ? "PASS" : "FAIL") << " (" << laws.size()
            << " laws found)\n";
  return failures == 0 ? 0 : 1;
}
