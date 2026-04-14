#pragma once
#include <Arduino.h>
#include <LoRa.h>
#include "../../main/flight_mode.h"
#include "../../main/globals.h"
#include "../../../config/pins.h"

void initRadio();
void updateRadio();
void sendTelemetry();
void processCommand(uint8_t* data, size_t len);
