#include "osrm/engine_config.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/osrm.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"

#include "json_helpers.hpp"
#include "parsers.hpp"

#include <cstdlib>
#include <cstring>
#include <optional>
#include <vector>

extern "C" {
struct RouteResult {
  float duration;
  char *polyline;
};
}

static std::vector<std::optional<osrm::engine::Hint>> persistentHints;
static std::vector<osrm::util::Coordinate> persistentCoords;

extern "C" {

osrm::OSRM *InitializeOSRM(const char *basePath) {
  osrm::EngineConfig config;
  config.storage_config = osrm::storage::StorageConfig(basePath);
  config.use_shared_memory = false;
  return new osrm::OSRM(config);
}

void RegisterStations(osrm::OSRM *osrm, double *coords, int numStations) {
  persistentHints.clear();
  persistentCoords.clear();

  persistentHints.reserve(numStations);
  persistentCoords.reserve(numStations);

  for (int i = 0; i < numStations; ++i) {

    osrm::util::Coordinate input{osrm::util::FloatLongitude{coords[i * 2]},
                                 osrm::util::FloatLatitude{coords[i * 2 + 1]}};

    osrm::NearestParameters params;
    params.coordinates.push_back(input);
    params.radiuses.push_back(100.0);

    osrm::engine::api::ResultT result;

    if (osrm->Nearest(params, result) == osrm::Status::Ok) {
      auto parsed = ParseNearest(result);
      if (parsed) {
        persistentHints.push_back(parsed->hint);
        persistentCoords.push_back(parsed->coord);
        continue;
      }
    }

    // fallback
    persistentHints.push_back(std::nullopt);
    persistentCoords.push_back(input);
  }
}

RouteResult *ComputeSrcToDest(osrm::OSRM *osrm, double evLon, double evLat,
                              double destLon, double destLat) {
  osrm::NearestParameters evParams;
  evParams.coordinates.push_back(
      {osrm::util::FloatLongitude{evLon}, osrm::util::FloatLatitude{evLat}});
  osrm::engine::api::ResultT evResult;
  if (osrm->Nearest(evParams, evResult) != osrm::Status::Ok) {
    return nullptr;
  }
  auto evSnap = ParseNearest(evResult);
  if (!evSnap) {
    return nullptr;
  }

  osrm::NearestParameters destParams;
  destParams.coordinates.push_back({osrm::util::FloatLongitude{destLon},
                                    osrm::util::FloatLatitude{destLat}});
  osrm::engine::api::ResultT destResult;
  if (osrm->Nearest(destParams, destResult) != osrm::Status::Ok) {
    return nullptr;
  }
  auto destSnap = ParseNearest(destResult);
  if (!destSnap) {
    return nullptr;
  }

  osrm::RouteParameters routeParams;
  routeParams.coordinates.push_back(evSnap->coord);
  routeParams.coordinates.push_back(destSnap->coord);
  osrm::engine::api::ResultT routeResult;
  if (osrm->Route(routeParams, routeResult) != osrm::Status::Ok) {
    return nullptr;
  }

  auto routeData = ParseRoute(routeResult);
  if (!routeData) {
    return nullptr;
  }

  RouteResult *ret = static_cast<RouteResult *>(malloc(sizeof(RouteResult)));
  if (!ret) {
    return nullptr;
  }

  ret->duration = routeData->first;

  const std::string &polylineStr = routeData->second;
  ret->polyline = static_cast<char *>(malloc(polylineStr.size() + 1));
  std::memcpy(ret->polyline, polylineStr.c_str(), polylineStr.size() + 1);
  return ret;
}

void ComputeTableIndexed(osrm::OSRM *osrm, double evLon, double evLat,
                         int *stationIndices, int numIndices, float *out) {

  osrm::TableParameters params;
  params.hints.push_back(std::nullopt);
  params.coordinates.push_back(
      {osrm::util::FloatLongitude{evLon}, osrm::util::FloatLatitude{evLat}});

  for (int i = 0; i < numIndices; ++i) {
    int idx = stationIndices[i];
    params.coordinates.push_back(persistentCoords[idx]);
    params.hints.push_back(persistentHints[idx]);
  }

  params.sources = {0};
  params.destinations.resize(numIndices);

  for (int i = 0; i < numIndices; ++i)
    params.destinations[i] = i + 1;

  osrm::engine::api::ResultT result;
  if (osrm->Table(params, result) != osrm::Status::Ok)
    return;

  auto parsed = ParseTable(result);
  if (!parsed)
    return;

  std::memcpy(out, parsed->durations.data(), sizeof(float) * numIndices);
}

void FreeMemory(void *ptr) { free(ptr); }

void DeleteOSRM(osrm::OSRM *osrm) { delete osrm; }

} // extern "C"
