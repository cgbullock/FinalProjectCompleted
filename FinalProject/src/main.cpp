#include <Arduino.h>
#include <Arduino.h>
#include "Behaviors.h"

Behaviors FinalLap;

void setup() {
  FinalLap.Init();
}

void loop() {
  FinalLap.Run();
}