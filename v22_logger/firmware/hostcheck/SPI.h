#pragma once
#include "Arduino.h"
#define MSBFIRST 1
#define SPI_MODE1 1
struct SPISettings { SPISettings(unsigned long, int, int); };
struct SPIClass { void begin(); void beginTransaction(SPISettings); void endTransaction(); uint16_t transfer16(uint16_t); };
extern SPIClass SPI;
