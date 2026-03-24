#include "osrm/engine_config.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/osrm.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"

#include "json_helpers.hpp"
#include "parsers.hpp"

#include <cstdint>
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

RouteResult *ComputeSrcToDestWithStops(osrm::OSRM *osrm, double *coords,
                                       int numCoords) {
  osrm::RouteParameters routeParams;

  for (int i = 0; i < numCoords; ++i) {
    double lon = coords[i * 2];
    double lat = coords[i * 2 + 1];

    osrm::NearestParameters params;
    params.coordinates.push_back(
        {osrm::util::FloatLongitude{lon}, osrm::util::FloatLatitude{lat}});
    osrm::engine::api::ResultT result;
    if (osrm->Nearest(params, result) != osrm::Status::Ok) {
      return nullptr;
    }
    auto snap = ParseNearest(result);
    if (!snap) {
      return nullptr;
    }
    routeParams.coordinates.push_back(snap->coord);
  }

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
                         uint16_t *stationIndices, int numIndices,
                         float *outDurations, float *outDistances) {

  osrm::TableParameters params;
  params.annotations =
      osrm::TableParameters::AnnotationsType::All; // request both durations and
                                                   // distances
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

  std::memcpy(outDurations, parsed->durations.data(),
              sizeof(float) * numIndices);
  std::memcpy(outDistances, parsed->distances.data(),
              sizeof(float) * numIndices);
}

void ComputeTableIndexedWithDest(osrm::OSRM *osrm, double evLon, double evLat,
                                 double destLon, double destLat,
                                 uint16_t *stationIndices, int numIndices,
                                 float *outDurations, float *outDistances) {

  osrm::TableParameters params;
  params.annotations = osrm::TableParameters::AnnotationsType::All;

  params.coordinates.push_back(
      {osrm::util::FloatLongitude{evLon}, osrm::util::FloatLatitude{evLat}});
  params.hints.push_back(std::nullopt);

  for (int i = 0; i < numIndices; ++i) {
    int idx = stationIndices[i];
    params.coordinates.push_back(persistentCoords[idx]);
    params.hints.push_back(persistentHints[idx]);
  }

  params.coordinates.push_back({osrm::util::FloatLongitude{destLon},
                                osrm::util::FloatLatitude{destLat}});
  params.hints.push_back(std::nullopt);

  params.sources.resize(numIndices + 1);
  for (int i = 0; i <= numIndices; ++i)
    params.sources[i] = i;

  params.destinations.resize(numIndices + 1);
  for (int i = 0; i < numIndices; ++i)
    params.destinations[i] = i + 1;
  params.destinations[numIndices] = numIndices + 1;

  osrm::engine::api::ResultT result;
  if (osrm->Table(params, result) != osrm::Status::Ok)
    return;

  auto parsed = ParseTable(result);
  if (!parsed)
    return;

  // Matrix is (numIndices+1) rows x (numIndices+1) cols
  // Row 0       = EV      → each station (cols 0..N-1) — leg 1
  // Row i (1..N)= station → final dest   (col N)       — leg 2

  const int cols = numIndices + 1;
  for (int i = 0; i < numIndices; ++i) {
    int evToStation = i;
    int stationToDest = (i + 1) * cols + numIndices;

    outDurations[i] =
        parsed->durations[evToStation] + parsed->durations[stationToDest];
    outDistances[i] =
        parsed->distances[evToStation] + parsed->distances[stationToDest];
  }
}

void FreeMemory(void *ptr) { free(ptr); }

void DeleteOSRM(osrm::OSRM *osrm) { delete osrm; }

void PointsToPoints(osrm::OSRM *osrm, double *srcCoords, int numSrcs,
                    double *dstCoords, int numDsts, float *outDurations,
                    float *outDistances) {

  osrm::TableParameters params;
  params.annotations = osrm::TableParameters::AnnotationsType::All;

  for (int i = 0; i < numSrcs; ++i) {
    params.coordinates.push_back(
        {osrm::util::FloatLongitude{srcCoords[i * 2]},
         osrm::util::FloatLatitude{srcCoords[i * 2 + 1]}});
    params.hints.push_back(std::nullopt);
  }

  for (int i = 0; i < numDsts; ++i) {
    params.coordinates.push_back(
        {osrm::util::FloatLongitude{dstCoords[i * 2]},
         osrm::util::FloatLatitude{dstCoords[i * 2 + 1]}});
    params.hints.push_back(std::nullopt);
  }

  params.sources.resize(numSrcs);
  for (int i = 0; i < numSrcs; ++i)
    params.sources[i] = i;

  params.destinations.resize(numDsts);
  for (int i = 0; i < numDsts; ++i)
    params.destinations[i] = numSrcs + i;

  osrm::engine::api::ResultT result;
  if (osrm->Table(params, result) != osrm::Status::Ok)
    return;

  auto parsed = ParseTable(result);
  if (!parsed)
    return;

  std::memcpy(outDurations, parsed->durations.data(),
              sizeof(float) * numSrcs * numDsts);
  std::memcpy(outDistances, parsed->distances.data(),
              sizeof(float) * numSrcs * numDsts);
}

} // extern "C"
