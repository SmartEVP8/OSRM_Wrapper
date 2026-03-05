#pragma once

#if __has_include(<osrm/engine/api/base_result.hpp>)
#include <osrm/engine/api/base_result.hpp>
#else
#include "engine/api/base_result.hpp"
#endif

#if __has_include(<osrm/engine/hint.hpp>)
#include <osrm/engine/hint.hpp>
#else
#include "engine/hint.hpp"
#endif

#if __has_include(<osrm/util/coordinate.hpp>)
#include <osrm/util/coordinate.hpp>
#else
#include "util/coordinate.hpp"
#endif

#include <optional>
#include <vector>

struct WaypointResult {
  std::optional<osrm::engine::Hint> hint;
  osrm::util::Coordinate coord;
};

struct TableResult {
  std::vector<float> durations;
  std::vector<float> distances;
};

std::optional<WaypointResult>
ParseNearest(const osrm::engine::api::ResultT &result);

std::optional<TableResult>
ParseTable(const osrm::engine::api::ResultT &result);

std::optional<std::pair<float, std::string>>
ParseRoute(const osrm::engine::api::ResultT &result);
