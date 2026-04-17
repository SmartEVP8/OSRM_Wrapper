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

struct TableData {
  std::vector<float> durations;
  std::vector<float> distances;
};

struct TableLeg {
  float durations;
  float distances;
};

struct TableResult {
  TableLeg srcToStation;
  TableLeg stationToDest;
};

std::optional<WaypointResult>
ParseNearest(const osrm::engine::api::ResultT &result);

std::optional<TableData> ParseTable(const osrm::engine::api::ResultT &result);

std::optional<std::tuple<float, float, std::string>>
ParseRoute(const osrm::engine::api::ResultT &result);
