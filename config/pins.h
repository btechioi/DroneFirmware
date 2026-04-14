#pragma once

#include <Arduino.h>

constexpr uint8_t MOTOR_PINS[] = {12, 13, 14, 15};

constexpr uint8_t RC_PINS[] = {16, 17, 18, 19, 20, 21, 22, 26};

constexpr uint8_t LED_STATUS = 25;

constexpr uint8_t LORA_SS = 17;
constexpr uint8_t LORA_RST = 20;
constexpr uint8_t LORA_DIO0 = 21;

constexpr uint8_t IMU_SDA = 4;
constexpr uint8_t IMU_SCL = 5;

constexpr uint8_t RADIO_SPI_CS = 9;
constexpr uint8_t RADIO_SPI_MISO = 8;
constexpr uint8_t RADIO_SPI_MOSI = 11;
constexpr uint8_t RADIO_SPI_SCK = 10;
constexpr uint32_t SPI_SPEED = 125000000;
