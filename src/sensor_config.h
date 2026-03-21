#ifndef __SENSOR_CONFIG_H
#define __SENSOR_CONFIG_H
#include <iostream>

#define ACC_NOISE_DENSITY 1.86e-02  // unit: m/(s^2) / sqrt(Hz), continuous-time
#define ACC_RANDOM_WALK 4.33e-03    // unit: m/(s^3) / sqrt(Hz)
#define ACC_CONST_BIAS_MAX 0.01     // unit: m / (s^2)

#define GYRO_NOISE_DENSITY 1.87e-03  // unit: rad/s/sqrt(Hz), continuous-time
#define GYRO_RANDOM_WALK 2.66e-03    // unit: rad/(s^2)/sqrt(Hz)
#define GYRO_CONST_BIAS_MAX 0.015    // unit: rad/s

#define IMU_FREQ 200  // uinit: Hz

#define IMU_CNT_PER_GPS 20
#define GPS_POS_NOISE_DENSITY 0.5  // m/sqrt(Hz)
#define GPS_POS_RANDOM_WALK 0      //
#define GPS_VEL_NOISE_DENSITY 0.01
#define GPS_VEL_RANDOM_WALK 0

#define IMU_CNT_PER_MAG 4
#define MAG_NOISE_DENSITY 1e-2
#define MAG_RANDOM_WALK 0
#define MAG_DECLINATION 0  //(2.78 * M_PI / 180.0)
#define MAG_INCLINATION 0  //(-33.58 * M_PI / 180.0)

#endif