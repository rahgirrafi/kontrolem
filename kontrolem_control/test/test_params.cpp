// Proves the ROS-free config-as-data types (M16): ParameterMap seeds from a
// spec's defaults, overlaid values win, every variant type round-trips, and the
// error paths (missing key, wrong type) throw. All with NO ROS present — the
// point being that a controller's configuration is testable off-middleware.
#include <iostream>
#include <string>
#include <vector>

#include "kontrolem_control/params.hpp"

using namespace kontrolem_control;

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
  // A schema exercising every ParamValue alternative.
  const ParameterSpec spec = {
    {"kp", ParamValue{100.0}, "a double"},
    {"horizon", ParamValue{std::int64_t{30}}, "an int"},
    {"enabled", ParamValue{true}, "a bool"},
    {"mode", ParamValue{std::string{"trot"}}, "a string"},
    {"q_diag", ParamValue{std::vector<double>{1.0, 2.0, 3.0}}, "a double array"},
    {"nodes", ParamValue{std::vector<std::int64_t>{5, 7}}, "an int array"},
    {"joints", ParamValue{std::vector<std::string>{"a", "b"}}, "a string array"},
  };

  // 1) Seeded from the spec -> every default resolves.
  ParameterMap m(spec);
  check(m.double_at("kp") == 100.0, "seeded double default");
  check(m.int_at("horizon") == 30, "seeded int default");
  check(m.bool_at("enabled") == true, "seeded bool default");
  check(m.string_at("mode") == "trot", "seeded string default");
  check(m.double_array_at("q_diag").size() == 3 && m.double_array_at("q_diag")[2] == 3.0,
        "seeded double-array default");
  check(m.int_array_at("nodes").size() == 2 && m.int_array_at("nodes")[1] == 7,
        "seeded int-array default");
  check(m.string_array_at("joints").size() == 2 && m.string_array_at("joints")[1] == "b",
        "seeded string-array default");

  // 2) Overlaid values win over defaults.
  m.set("kp", ParamValue{250.0});
  m.set("mode", ParamValue{std::string{"crawl"}});
  m.set("q_diag", ParamValue{std::vector<double>{9.0}});
  m.set("nodes", ParamValue{std::vector<std::int64_t>{11, 13, 17}});
  check(m.int_array_at("nodes").size() == 3 && m.int_array_at("nodes")[2] == 17,
        "overlaid int-array wins");
  // An int array and a double array are DIFFERENT parameter types (the M17 LPV
  // schema relies on this): reading one as the other must throw, not coerce.
  {
    bool coerced = true;
    try {
      (void)m.double_array_at("nodes");
    } catch (const std::runtime_error &) {
      coerced = false;
    }
    check(!coerced, "int-array is not readable as a double-array");
  }
  check(m.double_at("kp") == 250.0, "overlaid double wins");
  check(m.string_at("mode") == "crawl", "overlaid string wins");
  check(m.double_array_at("q_diag").size() == 1 && m.double_array_at("q_diag")[0] == 9.0,
        "overlaid array wins");

  // 3) has() reflects presence.
  check(m.has("kp"), "has(kp)");
  check(!m.has("nope"), "!has(nope)");

  // 4) Missing key throws.
  bool threw = false;
  try {
    (void)m.double_at("nope");
  } catch (const std::out_of_range &) {
    threw = true;
  }
  check(threw, "missing key throws out_of_range");

  // 5) Wrong-type access throws.
  threw = false;
  try {
    (void)m.double_at("mode");  // "mode" is a string
  } catch (const std::runtime_error &) {
    threw = true;
  }
  check(threw, "wrong-type access throws");

  // 6) An empty map (no seed) throws on any access — the runtime always seeds.
  ParameterMap empty;
  threw = false;
  try {
    (void)empty.int_at("horizon");
  } catch (const std::out_of_range &) {
    threw = true;
  }
  check(threw, "empty map access throws");

  std::cout << "test_params: " << (failures == 0 ? "PASS" : "FAIL") << " (" << failures
            << " failures)\n";
  return failures == 0 ? 0 : 1;
}
