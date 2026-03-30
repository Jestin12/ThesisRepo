#include "CreateJson.h"

void addFinger(JsonObject parent, const char *name) {
  JsonObject f = parent.createNestedObject(name);
  f["ax"] = nullptr;
  f["ay"] = nullptr;
  f["az"] = nullptr;
  f["yaw"] = nullptr;
  f["pitch"] = nullptr;
  f["roll"] = nullptr;
  f["flex_mcp"] = nullptr;
  f["flex_pip"] = nullptr;
}

void buildFingerData(JsonObject fingerData) {
  addFinger(fingerData, "Palm");
  addFinger(fingerData, "Index");
  addFinger(fingerData, "Middle");
  addFinger(fingerData, "Ring");
  addFinger(fingerData, "Pinky");
}