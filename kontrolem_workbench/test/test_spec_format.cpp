// Offline (no ROS, no pluginlib): the describe/docs formatting renders every
// ParamValue alternative correctly, and a real shipped spec (LPV — the one with
// the most type variety) comes out with all its rows.
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "kontrolem_controllers/factories.hpp"
#include "kontrolem_workbench/spec_format.hpp"

using namespace kontrolem_control;
using namespace kontrolem_workbench;

static int failures = 0;
static void check(bool cond, const std::string & what)
{
  if (!cond) {
    std::cout << "  FAIL: " << what << "\n";
    ++failures;
  }
}

int main()
{
  // type_name: one per alternative.
  check(type_name(ParamValue{1.5}) == "double", "type double");
  check(type_name(ParamValue{std::int64_t{3}}) == "int", "type int");
  check(type_name(ParamValue{true}) == "bool", "type bool");
  check(type_name(ParamValue{std::string{"x"}}) == "string", "type string");
  check(type_name(ParamValue{std::vector<double>{}}) == "double[]", "type double[]");
  check(type_name(ParamValue{std::vector<std::int64_t>{}}) == "int[]", "type int[]");
  check(type_name(ParamValue{std::vector<std::string>{}}) == "string[]", "type string[]");

  // value_str: YAML-compatible literals.
  check(value_str(ParamValue{100.0}) == "100.0", "double keeps a decimal point");
  check(value_str(ParamValue{0.5}) == "0.5", "plain double");
  check(value_str(ParamValue{std::int64_t{30}}) == "30", "int literal");
  check(value_str(ParamValue{false}) == "false", "bool literal");
  check(value_str(ParamValue{std::string{"trot"}}) == "\"trot\"", "string quoted");
  check(value_str(ParamValue{std::vector<double>{1.0, 2.5}}) == "[1.0, 2.5]", "double array");
  check(value_str(ParamValue{std::vector<std::int64_t>{9, 9}}) == "[9, 9]", "int array");
  check(
    value_str(ParamValue{std::vector<std::string>{"a", "b"}}) == "[\"a\", \"b\"]", "string array");
  check(value_str(ParamValue{std::vector<double>{}}) == "[]", "empty array");

  // A real shipped spec renders every row (LPV: doubles, double[], string[], int[]).
  const auto spec = kontrolem_controllers::LpvFactory{}.parameter_spec();
  const std::string out = format_spec("lpv", spec);
  check(out.find("control_law: lpv") != std::string::npos, "header names the law");
  for (const auto & d : spec) {
    check(out.find(d.name) != std::string::npos, "row present: " + d.name);
    check(out.find(d.description) != std::string::npos, "description present: " + d.name);
  }
  check(out.find("string[]") != std::string::npos, "lpv shows a string[] type");
  check(out.find("int[]") != std::string::npos, "lpv shows an int[] type");

  std::cout << "test_spec_format: " << (failures == 0 ? "PASS" : "FAIL") << " (" << failures
            << " failures)\n";
  return failures == 0 ? 0 : 1;
}
