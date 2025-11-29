#pragma once

//
// Choose your platform, M5STICKC or M5ATOM
//

#define M5STICKC
#include "utilities.h"
// #define M5ATOM

//
// Choose your I2C pins, SDA_PIN and SCL_PIN
// Leave them by default if you use the Grove connector
//

// e.g. M5STICKC hat pins
// #define SDA_PIN 26
// #define SCL_PIN 0

// e.g. M5ATOM back pins (= default I2C)
// #define SDA_PIN 21
// #define SCL_PIN 22

#if !defined(SDA_PIN) || !defined(SCL_PIN)
#if defined(M5STICKC)
#define SDA_PIN I2C_SDA
#define SCL_PIN I2C_SCL
#elif defined(M5ATOM)
#define SDA_PIN 26
#define SCL_PIN 32
#endif
#endif

// Sanity check
#if !defined(M5STICKC) && !defined(M5ATOM)
#error Define either M5STICKC or M5ATOM
#elif defined(M5STICKC) && defined(M5ATOM)
#error Define either M5STICKC or M5ATOM, not both
#endif