// 호스트 문법검사용 최소 스텁 (OpenCR + Dynamixel2Arduino API 표면만)
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#define OUTPUT 1
#define HIGH 1
#define LOW 0
#define HEX 16
#define BDPIN_DXL_PWR_EN 32
void pinMode(int, int); void digitalWrite(int, int); void delay(unsigned long); void delayMicroseconds(unsigned int);
unsigned long millis(); unsigned long micros();
struct HardwareSerial {
  void begin(unsigned long); int available(); int read();
  size_t print(const char*); size_t print(char); size_t print(int); size_t print(unsigned int); size_t print(long); size_t print(unsigned long); size_t print(float, int = 2); size_t print(double, int = 2);
  size_t println(); size_t println(const char*); size_t println(char); size_t println(int); size_t println(unsigned int); size_t println(long); size_t println(unsigned long); size_t println(float, int = 2); size_t println(double, int = 2);
  size_t print(int, int); size_t println(int, int); size_t print(long, int); size_t println(long, int);
};
extern HardwareSerial Serial, Serial3;
