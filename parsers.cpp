#include "parsers.hpp"
#include "engine/api/base_result.hpp"
#include "json_helpers.hpp"

#include <iostream>
#include <optional>
#include <variant>

std::optional<WaypointResult>
ParseNearest(const osrm::engine::api::ResultT &result) {
  using namespace jsonutil;

  auto *root = std::get_if<Object>(&result);
  if (!root)
    return std::nullopt;

  const Array *waypoints = getArray(*root, "waypoints");
  if (!waypoints || waypoints->values.empty())
    return std::nullopt;

  const Object *wp = std::get_if<Object>(&waypoints->values[0]);
  if (!wp)
    return std::nullopt;

  const String *hintStr = getString(*wp, "hint");
  const Array *location = getArray(*wp, "location");

  if (!hintStr || !location)
    return std::nullopt;

  const Number *lon = getNumber(*location, 0);
  const Number *lat = getNumber(*location, 1);

  if (!lon || !lat)
    return std::nullopt;

  WaypointResult out{osrm::engine::Hint::FromBase64(hintStr->value),
                     {osrm::util::FloatLongitude{lon->value},
                      osrm::util::FloatLatitude{lat->value}}};

  return out;
}

std::optional<std::tuple<float, float, std::string>>
ParseRoute(const osrm::engine::api::ResultT &result) {
  using namespace jsonutil;

  auto *root = std::get_if<Object>(&result);
  if (!root)
    return std::nullopt;

  const Array *routes = jsonutil::getArray(*root, "routes");
  if (!routes || routes->values.empty())
    return std::nullopt;

  const Object *route0 = std::get_if<Object>(&routes->values[0]);
  if (!route0)
    return std::nullopt;

  const Number *duration = jsonutil::getNumber(*route0, "duration");
  const Number *distance = jsonutil::getNumber(*route0, "distance");
  const String *polyline = jsonutil::getString(*route0, "geometry");

  if (!duration || !distance || !polyline)
    return std::nullopt;

  return std::make_tuple(static_cast<float>(duration->value),
                         static_cast<float>(distance->value), polyline->value);
}

static std::vector<float> ParseFloatMatrix(const Array &matrix) {
  std::vector<float> out;

  for (auto const &rowVal : matrix.values) {
    auto *row = std::get_if<Array>(&rowVal);
    if (!row)
      continue;
    for (auto const &v : row->values) {
      if (auto *num = std::get_if<Number>(&v))
        out.push_back(static_cast<float>(num->value));
      else
        out.push_back(NAN);
    }
  }
  return out;
}

std::optional<TableResult>
ParseTable(const osrm::engine::api::ResultT &result) {
  auto *root = std::get_if<Object>(&result);
  if (!root)
    return std::nullopt;

  auto durationsIt = root->values.find("durations");
  auto distancesIt = root->values.find("distances");
  if (durationsIt == root->values.end() || distancesIt == root->values.end())
    return std::nullopt;

  auto *durations = std::get_if<Array>(&durationsIt->second);
  auto *distances = std::get_if<Array>(&distancesIt->second);
  if (!durations || durations->values.empty() || !distances ||
      distances->values.empty())
    return std::nullopt;

  return TableResult{ParseFloatMatrix(*durations),
                     ParseFloatMatrix(*distances)};
}
