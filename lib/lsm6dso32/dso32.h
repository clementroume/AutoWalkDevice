/* Prevent recursive inclusion -----------------------------------------------*/
#ifndef __DSO32_H__
#define __DSO32_H__

#include "SPI.h"
#include "Wire.h"
#include "lsm6dso32_reg.h"

#ifdef ESP32
#ifndef MSBFIRST
#define MSBFIRST SPI_MSBFIRST
#endif
#endif

typedef enum { DSO32_OK = 0, DSO32_ERROR = -1 } DSO32_fct;

typedef enum {
  X_HIGH_PERF,
  X_LOW_OR_NORMAL,
  X_ULTRA_LOW,
} x_perf_t;

typedef enum {
  G_HIGH_PERF,
  G_LOW_OR_NORMAL,
} g_perf_t;

class DSO32 {
 public:
  DSO32(TwoWire* i2c, uint8_t address = LSM6DSO32_I2C_ADD_H);
  DSO32_fct begin();

  DSO32_fct Set_X(lsm6dso32_odr_xl_t Odr);
  DSO32_fct Set_X_Perf(x_perf_t Perf);
  DSO32_fct Set_X_FS(lsm6dso32_fs_xl_t Fs);
  DSO32_fct Get_X_DRDY_Status(uint8_t* Status);
  DSO32_fct Get_X_Axes(float_t* Acceleration);

  DSO32_fct Set_G(lsm6dso32_odr_g_t Odr);
  DSO32_fct Set_G_Perf(g_perf_t Perf);
  DSO32_fct Set_G_FS(lsm6dso32_fs_g_t Fs);
  DSO32_fct Get_G_DRDY_Status(uint8_t* Status);
  DSO32_fct Get_G_Axes(float_t* AngularRate);

  DSO32_fct Get_T_DRDY_Status(uint8_t* Status);
  DSO32_fct Get_Temp(float_t* Temperature);

  DSO32_fct Enable_Pedometer();

  uint8_t IO_Read(uint8_t RegAddr, uint8_t* pBuff, uint16_t nByte);
  uint8_t IO_Write(uint8_t reg, uint8_t* bufp, uint16_t len);

 private:
  // i2c
  TwoWire* dev_i2c;
  uint8_t address;

  stmdev_ctx_t reg_ctx;
  lsm6dso32_odr_xl_t acc_odr;
  lsm6dso32_odr_g_t gyro_odr;
  uint8_t acc_is_enabled;
  uint8_t gyro_is_enabled;
};

int32_t platform_read(void* handle, uint8_t reg, uint8_t* buffp, uint16_t len);
int32_t platform_write(void* handle, uint8_t reg, uint8_t* buffp, uint16_t len);

#endif