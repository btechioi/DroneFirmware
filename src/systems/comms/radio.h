#pragma once

#include "globals.h"

void initRadio();
void updateRadio();
void sendTelemetry();
void processCommand(uint8_t* data, int packetSize);