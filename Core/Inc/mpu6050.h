#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

// Libs
#include <stdint.h>
#include <math.h>
#include "stm32f1xx_hal.h"

// Constants
#define RAD2DEG 57.2957795131

// Defines
#define WHO_AM_I_6050_ANS 0x68
#define WHO_AM_I_9250_ANS 0x71
#define WHO_AM_I          0x75
#define AD0_LOW           0x68
#define AD0_HIGH          0x69
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define PWR_MGMT_1        0x6B
#define ACCEL_XOUT_H      0x3B
#define I2C_TIMOUT_MS     1000

// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

// Structures
struct RawData
{
    int16_t ax, ay, az, gx, gy, gz;
};
extern struct RawData rawData;

struct SensorData
{
    float ax, ay, az, gx, gy, gz;
};
extern struct SensorData sensorData;

struct GyroCal
{
    float x, y, z;
};
extern struct GyroCal gyroCal;

struct Attitude
{
    float r, p, y;
};
extern struct Attitude attitude;

// Variables
extern uint8_t _addr;
extern float _dt, _tau;
extern float aScaleFactor, gScaleFactor;

// Functions
uint8_t MPU6050_init(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt);
void MPU6050_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints);
void MPU6050_calcAttitude(I2C_HandleTypeDef *I2Cx, float dt);
void MPU6050_readRawData(I2C_HandleTypeDef *I2Cx);
void MPU6050_readProcessedData(I2C_HandleTypeDef *I2Cx);
void MPU6050_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale);
void MPU6050_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale);




#endif /* INC_MPU6050_H_ */
