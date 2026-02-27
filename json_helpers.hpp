#include "osrm/nearest_parameters.hpp"
#include "osrm/osrm.hpp"
#include <variant>

using namespace osrm::util::json;
//
// ---------- JSON HELPERS ----------
//

namespace jsonutil {

inline const Object *getObject(const Object &obj, const char *key) {
  auto it = obj.values.find(key);
  if (it == obj.values.end())
    return nullptr;
  return std::get_if<Object>(&it->second);
}

inline const Array *getArray(const Object &obj, const char *key) {
  auto it = obj.values.find(key);
  if (it == obj.values.end())
    return nullptr;
  return std::get_if<Array>(&it->second);
}

inline const String *getString(const Object &obj, const char *key) {
  auto it = obj.values.find(key);
  if (it == obj.values.end())
    return nullptr;
  return std::get_if<String>(&it->second);
}

inline const Number *getNumber(const Object &obj, const char *key) {
  auto it = obj.values.find(key);
  if (it == obj.values.end())
    return nullptr;
  return std::get_if<Number>(&it->second);
}

inline const Number *getNumber(const Array &arr, size_t idx) {
  if (idx >= arr.values.size())
    return nullptr;
  return std::get_if<Number>(&arr.values[idx]);
}

} // namespace jsonutil
