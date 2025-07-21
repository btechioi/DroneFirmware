#pragma once

#include "globals.h"

bool checkSignalLoss();
bool checkCriticalSensorFailure();
bool checkBatteryStatus();
bool checkGeoFence();
void checkFailsafes();
