#include "CreateJson.h"
#include "Globals.h"

void addFinger(JsonObject parent, const char *name) {
  JsonObject f = parent.createNestedObject(name);
  f["ax_prox"] = nullptr;
  f["ay_prox"] = nullptr;
  f["az_prox"] = nullptr;
  f["yaw_prox"] = nullptr;
  f["pitch_prox"] = nullptr;
  f["roll_prox"] = nullptr;
  f["flex_mcp"] = nullptr;
  f["flex_pip"] = nullptr;
  f["ax_prox"] = nullptr;
  f["ay_prox"] = nullptr;
  f["az_prox"] = nullptr;
  f["yaw_prox"] = nullptr;
  f["pitch_prox"] = nullptr;
  f["roll_prox"] = nullptr;
}

void buildFingerData(JsonObject fingerData) {

  for (int i = 0; i < int(sizeof(HandChannels)/sizeof(HandChannels[0])); i++)
  {
    addFinger(fingerData, HandChannels[i].label.c_str());
  }
  // addFinger(fingerData, "Palm");
  // addFinger(fingerData, "Index");
  // addFinger(fingerData, "Middle");
  // addFinger(fingerData, "Ring");
  // addFinger(fingerData, "Pinky");
}