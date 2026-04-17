#include "osrm/engine_config.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/osrm.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"

#include "json_helpers.hpp"
#include "parsers.hpp"
#include "util/coordinate.hpp"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <vector>

extern "C" {
struct RouteResult {
  float duration;
  float distance;
  char *polyline;
};

struct InstanceState {
  osrm::OSRM *osrm;
  std::vector<std::optional<osrm::engine::Hint>> persistentHints;
  std::vector<osrm::util::Coordinate> persistentCoords;
};
}

extern "C" {

InstanceState *InitializeOSRM(const char *basePath) {
  osrm::EngineConfig config;
  config.storage_config = osrm::storage::StorageConfig(basePath);
  config.use_shared_memory = false;

  auto *state = new InstanceState();
  state->osrm = new osrm::OSRM(config);
  return state;
}

bool RegisterStations(InstanceState *state, double *coords, int numStations,
                      double *outSnappedCoords) {
  state->persistentHints.clear();
  state->persistentCoords.clear();
  state->persistentHints.reserve(numStations);
  state->persistentCoords.reserve(numStations);

  for (int i = 0; i < numStations; ++i) {

    osrm::util::Coordinate input{osrm::util::FloatLongitude{coords[i * 2]},
                                 osrm::util::FloatLatitude{coords[i * 2 + 1]}};

    osrm::NearestParameters params;
    params.coordinates.push_back(input);
    params.radiuses.push_back(212.5);

    osrm::engine::api::ResultT result;

    if (state->osrm->Nearest(params, result) == osrm::Status::Ok) {
      auto parsed = ParseNearest(result);
      if (parsed) {
        state->persistentHints.push_back(parsed->hint);
        state->persistentCoords.push_back(parsed->coord);
        outSnappedCoords[i * 2] =
            static_cast<double>(osrm::util::toFloating(parsed->coord.lon));
        outSnappedCoords[i * 2 + 1] =
            static_cast<double>(osrm::util::toFloating(parsed->coord.lat));
        continue;
      }
    }

    return false;
  }

  return true;
}

RouteResult *ComputeSrcToDest(InstanceState *state, double evLon, double evLat,
                              double destLon, double destLat) {
  osrm::NearestParameters evParams;
  evParams.coordinates.push_back(
      {osrm::util::FloatLongitude{evLon}, osrm::util::FloatLatitude{evLat}});
  osrm::engine::api::ResultT evResult;
  if (state->osrm->Nearest(evParams, evResult) != osrm::Status::Ok) {
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
  if (state->osrm->Nearest(destParams, destResult) != osrm::Status::Ok) {
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
  if (state->osrm->Route(routeParams, routeResult) != osrm::Status::Ok) {
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

  ret->duration = std::get<0>(*routeData);
  ret->distance = std::get<1>(*routeData);

  const std::string &polylineStr = std::get<2>(*routeData);
  ret->polyline = static_cast<char *>(malloc(polylineStr.size() + 1));
  std::memcpy(ret->polyline, polylineStr.c_str(), polylineStr.size() + 1);
  return ret;
}

RouteResult *ComputeSrcToDestWithStop(InstanceState *state, double evLon,
                                      double evLat, double stationLon,
                                      double stationLat, double destLon,
                                      double destLat, ushort index) {
  osrm::RouteParameters routeParams;

  // Snap source (EV position) - first coordinate
  osrm::NearestParameters srcParams;
  srcParams.coordinates.push_back(
      {osrm::util::FloatLongitude{evLon}, osrm::util::FloatLatitude{evLat}});
  osrm::engine::api::ResultT srcResult;
  if (state->osrm->Nearest(srcParams, srcResult) != osrm::Status::Ok) {
    return nullptr;
  }
  auto srcSnap = ParseNearest(srcResult);
  if (!srcSnap) {
    return nullptr;
  }
  routeParams.coordinates.push_back(srcSnap->coord);
  routeParams.hints.push_back(srcSnap->hint);

  if (index < state->persistentCoords.size()) {
    // Use pre-computed hints for intermediate station stops
    int idx = index;
    routeParams.coordinates.push_back(state->persistentCoords[idx]);
    routeParams.hints.push_back(state->persistentHints[idx]);

  } else {
    // Snap intermediate station stop if index is invalid
    osrm::NearestParameters stationParams;
    stationParams.coordinates.push_back(
        {osrm::util::FloatLongitude{stationLon},
         osrm::util::FloatLatitude{stationLat}});
    osrm::engine::api::ResultT stationResult;
    if (state->osrm->Nearest(stationParams, stationResult) !=
        osrm::Status::Ok) {
      return nullptr;
    }
    auto stationSnap = ParseNearest(stationResult);
    if (!stationSnap) {
      return nullptr;
    }
    routeParams.coordinates.push_back(stationSnap->coord);
    routeParams.hints.push_back(stationSnap->hint);
  }

  // Snap destination - last coordinate
  osrm::NearestParameters destParams;
  destParams.coordinates.push_back({osrm::util::FloatLongitude{destLon},
                                    osrm::util::FloatLatitude{destLat}});
  osrm::engine::api::ResultT destResult;
  if (state->osrm->Nearest(destParams, destResult) != osrm::Status::Ok) {
    return nullptr;
  }
  auto destSnap = ParseNearest(destResult);
  if (!destSnap) {
    return nullptr;
  }
  routeParams.coordinates.push_back(destSnap->coord);
  routeParams.hints.push_back(destSnap->hint);

  osrm::engine::api::ResultT routeResult;
  if (state->osrm->Route(routeParams, routeResult) != osrm::Status::Ok) {
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

  ret->duration = std::get<0>(*routeData);
  ret->distance = std::get<1>(*routeData);

  const std::string &polylineStr = std::get<2>(*routeData);
  ret->polyline = static_cast<char *>(malloc(polylineStr.size() + 1));
  std::memcpy(ret->polyline, polylineStr.c_str(), polylineStr.size() + 1);
  return ret;
}

void ComputeTableIndexedWithDest(InstanceState *state, double evLon,
                                 double evLat, double destLon, double destLat,
                                 uint16_t *stationIndices, int numIndices,
                                 TableResult *outResults) {

  osrm::TableParameters params;
  params.annotations = osrm::TableParameters::AnnotationsType::All;

  params.coordinates.push_back(
      {osrm::util::FloatLongitude{evLon}, osrm::util::FloatLatitude{evLat}});
  params.hints.push_back(std::nullopt);

  for (int i = 0; i < numIndices; ++i) {
    int idx = stationIndices[i];
    params.coordinates.push_back(state->persistentCoords[idx]);
    params.hints.push_back(state->persistentHints[idx]);
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
  if (state->osrm->Table(params, result) != osrm::Status::Ok)
    return;

  auto parsed = ParseTable(result);
  if (!parsed)
    return;

  // Matrix: (N x N) where N = Source + Stations + Destination
  // Leg 1: Source → Station i     | Index: [Row 0, Col i]
  // Leg 2: Station i → Destination | Index: [Row i, Col N-1]

  const int cols = numIndices - 1;
  const int numStations = cols - 2;
  const int destinationIdx = cols - 1;

  for (int i = 0; i < numStations; ++i) {
    int stationIdx = i + 1;
    int evToStation = (0 * cols) + i;
    int stationToDest = (stationIdx * cols) + destinationIdx;
    
    outResults[i].srcToStation.durations = parsed->durations[evToStation];
    outResults[i].srcToStation.distances = parsed->distances[evToStation];
    outResults[i].stationToDest.durations = parsed->durations[stationToDest];
    outResults[i].stationToDest.distances = parsed->distances[stationToDest];
  }
}

void FreeMemory(void *ptr) { free(ptr); }

void DeleteOSRM(InstanceState *state) {
  delete state->osrm;
  delete state;
}
void PointsToPoints(InstanceState *state, double *srcCoords, int numSrcs,
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
  if (state->osrm->Table(params, result) != osrm::Status::Ok)
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
