#include "dso32.h"

// lsm6dso32_timestamp_set(stmdev_ctx_t* ctx, uint8_t val);
// lsm6dso32_timestamp_get(stmdev_ctx_t* ctx, uint8_t* val);
// lsm6dso32_timestamp_raw_get(stmdev_ctx_t* ctx, uint32_t* val);

// lsm6dso32_number_of_steps_get(stmdev_ctx_t* ctx, uint16_t* val);
// lsm6dso32_steps_reset(stmdev_ctx_t* ctx);

// typedef enum {
//   LSM6DSO32_PEDO_DISABLE = 0x00,
//   LSM6DSO32_PEDO_BASE_MODE = 0x01,
//   LSM6DSO32_PEDO_ADV_MODE = 0x03,
//   LSM6DSO32_FALSE_STEP_REJ = 0x13,
//   LSM6DSO32_FALSE_STEP_REJ_ADV_MODE = 0x33,
// } lsm6dso32_pedo_md_t;
// int32_t lsm6dso32_pedo_sens_set(stmdev_ctx_t* ctx, lsm6dso32_pedo_md_t val);
// int32_t lsm6dso32_pedo_sens_get(stmdev_ctx_t* ctx, lsm6dso32_pedo_md_t* val);
// int32_t lsm6dso32_pedo_step_detect_get(stmdev_ctx_t* ctx, uint8_t* val);
// int32_t lsm6dso32_pedo_debounce_steps_set(stmdev_ctx_t* ctx, uint8_t* buff);
// int32_t lsm6dso32_pedo_debounce_steps_get(stmdev_ctx_t* ctx, uint8_t* buff);
// int32_t lsm6dso32_pedo_steps_period_set(stmdev_ctx_t* ctx, uint16_t val);
// int32_t lsm6dso32_pedo_steps_period_get(stmdev_ctx_t* ctx, uint16_t* val);

DSO32::DSO32(TwoWire* i2c, uint8_t address) : dev_i2c(i2c), address(address) {
  reg_ctx.write_reg = platform_write;
  reg_ctx.read_reg = platform_read;
  reg_ctx.handle = (void*)this;
  acc_is_enabled = 0U;
  gyro_is_enabled = 0U;
}

DSO32_fct DSO32::begin() {
  if (lsm6dso32_i3c_disable_set(&reg_ctx, LSM6DSO32_I3C_DISABLE))
    return DSO32_ERROR;
  if (lsm6dso32_auto_increment_set(&reg_ctx, PROPERTY_ENABLE))
    return DSO32_ERROR;
  if (lsm6dso32_block_data_update_set(&reg_ctx, PROPERTY_ENABLE))
    return DSO32_ERROR;
  if (lsm6dso32_xl_data_rate_set(&reg_ctx, LSM6DSO32_XL_ODR_OFF))
    return DSO32_ERROR;
  if (lsm6dso32_gy_data_rate_set(&reg_ctx, LSM6DSO32_GY_ODR_OFF))
    return DSO32_ERROR;
  if (lsm6dso32_rounding_mode_set(&reg_ctx, LSM6DSO32_ROUND_GY_XL))
    return DSO32_ERROR;

  acc_is_enabled = 0;
  gyro_is_enabled = 0;

  return DSO32_OK;
}

DSO32_fct DSO32::Set_X(lsm6dso32_odr_xl_t xOdr) {
  if (xOdr == LSM6DSO32_XL_ODR_6Hz5_LOW_PW ||
      xOdr == LSM6DSO32_XL_ODR_12Hz5_LOW_PW ||
      xOdr == LSM6DSO32_XL_ODR_26Hz_LOW_PW ||
      xOdr == LSM6DSO32_XL_ODR_52Hz_LOW_PW ||
      xOdr == LSM6DSO32_XL_ODR_104Hz_NORMAL_MD ||
      xOdr == LSM6DSO32_XL_ODR_208Hz_NORMAL_MD)
    Set_X_Perf(X_LOW_OR_NORMAL);

  if (xOdr == LSM6DSO32_XL_ODR_12Hz5_HIGH_PERF ||
      xOdr == LSM6DSO32_XL_ODR_26Hz_HIGH_PERF ||
      xOdr == LSM6DSO32_XL_ODR_52Hz_HIGH_PERF ||
      xOdr == LSM6DSO32_XL_ODR_104Hz_HIGH_PERF ||
      xOdr == LSM6DSO32_XL_ODR_208Hz_HIGH_PERF ||
      xOdr == LSM6DSO32_XL_ODR_417Hz_HIGH_PERF ||
      xOdr == LSM6DSO32_XL_ODR_833Hz_HIGH_PERF ||
      xOdr == LSM6DSO32_XL_ODR_1667Hz_HIGH_PERF ||
      xOdr == LSM6DSO32_XL_ODR_3333Hz_HIGH_PERF ||
      xOdr == LSM6DSO32_XL_ODR_6667Hz_HIGH_PERF)
    Set_X_Perf(X_HIGH_PERF);

  if (xOdr == LSM6DSO32_XL_ODR_6Hz5_ULTRA_LOW_PW ||
      xOdr == LSM6DSO32_XL_ODR_12Hz5_ULTRA_LOW_PW ||
      xOdr == LSM6DSO32_XL_ODR_26Hz_ULTRA_LOW_PW ||
      xOdr == LSM6DSO32_XL_ODR_52Hz_ULTRA_LOW_PW ||
      xOdr == LSM6DSO32_XL_ODR_104Hz_ULTRA_LOW_PW ||
      xOdr == LSM6DSO32_XL_ODR_208Hz_ULTRA_LOW_PW)
    Set_X_Perf(X_ULTRA_LOW);

  if (xOdr == LSM6DSO32_XL_ODR_OFF)
    acc_is_enabled = 0;
  else
    acc_is_enabled = 1;

  if (lsm6dso32_xl_data_rate_set(&reg_ctx, xOdr))
    return DSO32_ERROR;

  return DSO32_OK;
}
DSO32_fct DSO32::Set_X_Perf(x_perf_t Perf) {
  switch (Perf) {
    case X_HIGH_PERF: {
      lsm6dso32_ctrl5_c_t ULP;
      lsm6dso32_ctrl6_c_t LaN;

      /*Shutdowm Ultra Low Power*/
      if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_CTRL5_C, (uint8_t*)&ULP, 1))
        return DSO32_ERROR;
      if (ULP.xl_ulp_en) {
        if (acc_is_enabled == 1U) /*Shutdowm Accelerometer*/
          if (lsm6dso32_xl_data_rate_set(&reg_ctx, LSM6DSO32_XL_ODR_OFF))
            return DSO32_ERROR;
        ULP.xl_ulp_en = 0; /*Set ULP pin to 0*/
        if (lsm6dso32_write_reg(&reg_ctx, LSM6DSO32_CTRL5_C, (uint8_t*)&ULP, 1))
          return DSO32_ERROR;
      }

      /*Activate High Performance mode*/
      if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_CTRL6_C, (uint8_t*)&LaN, 1))
        return DSO32_ERROR;
      if (LaN.xl_hm_mode) {
        LaN.xl_hm_mode =
            0; /*Set Low and Normal pin to 0 == Activate Highr perf*/
        if (lsm6dso32_write_reg(&reg_ctx, LSM6DSO32_CTRL6_C, (uint8_t*)&LaN, 1))
          return DSO32_ERROR;
      }
      break;
    }
    case X_LOW_OR_NORMAL: {
      lsm6dso32_ctrl5_c_t ULP;
      lsm6dso32_ctrl6_c_t LaN;

      /*Shutdowm Ultra Low Power*/
      if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_CTRL5_C, (uint8_t*)&ULP, 1))
        return DSO32_ERROR;
      if (ULP.xl_ulp_en) {
        if (acc_is_enabled == 1U) /*Shutdowm Accelerometer*/
          if (lsm6dso32_xl_data_rate_set(&reg_ctx, LSM6DSO32_XL_ODR_OFF))
            return DSO32_ERROR;
        ULP.xl_ulp_en = 0; /*Set ULP pin to 0*/
        if (lsm6dso32_write_reg(&reg_ctx, LSM6DSO32_CTRL5_C, (uint8_t*)&ULP, 1))
          return DSO32_ERROR;
      }

      /*Check Low Power and Normal mode activation*/
      if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_CTRL6_C, (uint8_t*)&LaN, 1))
        return DSO32_ERROR;
      if (!LaN.xl_hm_mode) {
        LaN.xl_hm_mode = 1U; /*Set Low and Normal pin to 1*/
        if (lsm6dso32_write_reg(&reg_ctx, LSM6DSO32_CTRL6_C, (uint8_t*)&LaN, 1))
          return DSO32_ERROR;
      }
      break;
    }
    case X_ULTRA_LOW: {
      /* and check the Ultra Low Power bit if it is unchecked             */
      /* We must switch off gyro otherwise Ultra Low Power does not work  */
      lsm6dso32_ctrl5_c_t ULP;
      lsm6dso32_ctrl6_c_t LaN;

      /*Shutdown Low Power and Normal mode*/
      if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_CTRL6_C, (uint8_t*)&LaN, 1))
        return DSO32_ERROR;
      if (LaN.xl_hm_mode)
        LaN.xl_hm_mode = 0;
      if (lsm6dso32_write_reg(&reg_ctx, LSM6DSO32_CTRL6_C, (uint8_t*)&LaN, 1))
        return DSO32_ERROR;

      /* Disable Gyro */
      if (gyro_is_enabled == 1U)
        if (lsm6dso32_gy_data_rate_get(&reg_ctx, &gyro_odr))
          return DSO32_ERROR;
      if (lsm6dso32_gy_data_rate_set(&reg_ctx, LSM6DSO32_GY_ODR_OFF))
        return DSO32_ERROR;

      /*Activate Ultra Low Power*/
      if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_CTRL5_C, (uint8_t*)&ULP, 1))
        return DSO32_ERROR;
      if (!ULP.xl_ulp_en)
        if (acc_is_enabled == 1U) /*Shutdowm Accelerometer*/
          if (lsm6dso32_xl_data_rate_set(&reg_ctx, LSM6DSO32_XL_ODR_OFF))
            return DSO32_ERROR;
      ULP.xl_ulp_en = 1U; /*Set ULP pin to 0*/
      if (lsm6dso32_write_reg(&reg_ctx, LSM6DSO32_CTRL5_C, (uint8_t*)&ULP, 1))
        return DSO32_ERROR;
      break;
    }
    default:
      return DSO32_ERROR;
      break;
  }
  return DSO32_OK;
}
DSO32_fct DSO32::Set_X_FS(lsm6dso32_fs_xl_t Fs) {
  if (lsm6dso32_xl_full_scale_set(&reg_ctx, Fs))
    return DSO32_ERROR;
  return DSO32_OK;
}
DSO32_fct DSO32::Get_X_DRDY_Status(uint8_t* Status) {
  if (lsm6dso32_xl_flag_data_ready_get(&reg_ctx, Status))
    return DSO32_ERROR;
  return DSO32_OK;
}
DSO32_fct DSO32::Get_X_Axes(float_t* Acceleration) {
  int16_t data_raw[3];
  lsm6dso32_fs_xl_t full_scale;

  if (lsm6dso32_acceleration_raw_get(&reg_ctx, data_raw))
    return DSO32_ERROR;

  if (lsm6dso32_xl_full_scale_get(&reg_ctx, &full_scale))
    return DSO32_ERROR;
  switch (full_scale) {
    case LSM6DSO32_4g: {
      Acceleration[0] = lsm6dso32_from_fs4_to_mg(data_raw[0]);
      Acceleration[1] = lsm6dso32_from_fs4_to_mg(data_raw[1]);
      Acceleration[2] = lsm6dso32_from_fs4_to_mg(data_raw[2]);
      break;
    }
    case LSM6DSO32_8g: {
      Acceleration[0] = lsm6dso32_from_fs8_to_mg(data_raw[0]);
      Acceleration[1] = lsm6dso32_from_fs8_to_mg(data_raw[1]);
      Acceleration[2] = lsm6dso32_from_fs8_to_mg(data_raw[2]);
      break;
    }
    case LSM6DSO32_16g: {
      Acceleration[0] = lsm6dso32_from_fs16_to_mg(data_raw[0]);
      Acceleration[1] = lsm6dso32_from_fs16_to_mg(data_raw[1]);
      Acceleration[2] = lsm6dso32_from_fs16_to_mg(data_raw[2]);
      break;
    }
    case LSM6DSO32_32g: {
      Acceleration[0] = lsm6dso32_from_fs32_to_mg(data_raw[0]);
      Acceleration[1] = lsm6dso32_from_fs32_to_mg(data_raw[1]);
      Acceleration[2] = lsm6dso32_from_fs32_to_mg(data_raw[2]);
      break;
    }
    default:
      return DSO32_ERROR;
  }
  return DSO32_OK;
}

DSO32_fct DSO32::Set_G(lsm6dso32_odr_g_t gOdr) {
  if (gOdr == LSM6DSO32_GY_ODR_12Hz5_LOW_PW ||
      gOdr == LSM6DSO32_GY_ODR_26Hz_LOW_PW ||
      gOdr == LSM6DSO32_GY_ODR_52Hz_LOW_PW ||
      gOdr == LSM6DSO32_GY_ODR_104Hz_NORMAL_MD ||
      gOdr == LSM6DSO32_GY_ODR_208Hz_NORMAL_MD)
    Set_G_Perf(G_LOW_OR_NORMAL);

  if (gOdr == LSM6DSO32_GY_ODR_12Hz5_HIGH_PERF ||
      gOdr == LSM6DSO32_GY_ODR_26Hz_HIGH_PERF ||
      gOdr == LSM6DSO32_GY_ODR_52Hz_HIGH_PERF ||
      gOdr == LSM6DSO32_GY_ODR_104Hz_HIGH_PERF ||
      gOdr == LSM6DSO32_GY_ODR_208Hz_HIGH_PERF ||
      gOdr == LSM6DSO32_GY_ODR_417Hz_HIGH_PERF ||
      gOdr == LSM6DSO32_GY_ODR_833Hz_HIGH_PERF ||
      gOdr == LSM6DSO32_GY_ODR_1667Hz_HIGH_PERF ||
      gOdr == LSM6DSO32_GY_ODR_3333Hz_HIGH_PERF ||
      gOdr == LSM6DSO32_GY_ODR_6667Hz_HIGH_PERF)
    Set_G_Perf(G_HIGH_PERF);

  if (gOdr == LSM6DSO32_GY_ODR_OFF)
    gyro_is_enabled = 0;
  else
    gyro_is_enabled = 1;

  if (lsm6dso32_gy_data_rate_set(&reg_ctx, gOdr))
    return DSO32_ERROR;

  return DSO32_OK;
}
DSO32_fct DSO32::Set_G_Perf(g_perf_t Perf) {
  switch (Perf) {
    case G_HIGH_PERF: {
      lsm6dso32_ctrl7_g_t mod;
      if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_CTRL7_G, (uint8_t*)&mod, 1))
        return DSO32_ERROR;
      if (mod.g_hm_mode) {
        mod.g_hm_mode = 0;
        if (lsm6dso32_write_reg(&reg_ctx, LSM6DSO32_CTRL7_G, (uint8_t*)&mod, 1))
          return DSO32_ERROR;
      }
      break;
    }
    case G_LOW_OR_NORMAL: {
      lsm6dso32_ctrl7_g_t mod;
      if (lsm6dso32_read_reg(&reg_ctx, LSM6DSO32_CTRL7_G, (uint8_t*)&mod, 1))
        return DSO32_ERROR;
      if (!mod.g_hm_mode) {
        mod.g_hm_mode = 1U;
        if (lsm6dso32_write_reg(&reg_ctx, LSM6DSO32_CTRL7_G, (uint8_t*)&mod, 1))
          return DSO32_ERROR;
      }
      break;
    }
    default:
      return DSO32_ERROR;
  }
  return DSO32_OK;
}
DSO32_fct DSO32::Set_G_FS(lsm6dso32_fs_g_t Fs) {
  if (lsm6dso32_gy_full_scale_set(&reg_ctx, Fs))
    return DSO32_ERROR;
  return DSO32_OK;
}
DSO32_fct DSO32::Get_G_DRDY_Status(uint8_t* Status) {
  if (lsm6dso32_gy_flag_data_ready_get(&reg_ctx, Status))
    return DSO32_ERROR;
  return DSO32_OK;
}
DSO32_fct DSO32::Get_G_Axes(float_t* AngularRate) {
  int16_t data_raw[3];
  lsm6dso32_fs_g_t full_scale;

  if (lsm6dso32_angular_rate_raw_get(&reg_ctx, data_raw))
    return DSO32_ERROR;

  if (lsm6dso32_gy_full_scale_get(&reg_ctx, &full_scale))
    return DSO32_ERROR;
  switch (full_scale) {
    case LSM6DSO32_250dps: {
      AngularRate[0] = lsm6dso32_from_fs125_to_mdps(data_raw[0]);
      AngularRate[1] = lsm6dso32_from_fs125_to_mdps(data_raw[1]);
      AngularRate[2] = lsm6dso32_from_fs125_to_mdps(data_raw[2]);
      break;
    }
    case LSM6DSO32_125dps: {
      AngularRate[0] = lsm6dso32_from_fs250_to_mdps(data_raw[0]);
      AngularRate[1] = lsm6dso32_from_fs250_to_mdps(data_raw[1]);
      AngularRate[2] = lsm6dso32_from_fs250_to_mdps(data_raw[2]);
      break;
    }
    case LSM6DSO32_500dps: {
      AngularRate[0] = lsm6dso32_from_fs500_to_mdps(data_raw[0]);
      AngularRate[1] = lsm6dso32_from_fs500_to_mdps(data_raw[1]);
      AngularRate[2] = lsm6dso32_from_fs500_to_mdps(data_raw[2]);
      break;
    }
    case LSM6DSO32_1000dps: {
      AngularRate[0] = lsm6dso32_from_fs1000_to_mdps(data_raw[0]);
      AngularRate[1] = lsm6dso32_from_fs1000_to_mdps(data_raw[1]);
      AngularRate[2] = lsm6dso32_from_fs1000_to_mdps(data_raw[2]);
      break;
    }
    case LSM6DSO32_2000dps: {
      AngularRate[0] = lsm6dso32_from_fs2000_to_mdps(data_raw[0]);
      AngularRate[1] = lsm6dso32_from_fs2000_to_mdps(data_raw[1]);
      AngularRate[2] = lsm6dso32_from_fs2000_to_mdps(data_raw[2]);
      break;
    }
    default:
      return DSO32_ERROR;
  }
  return DSO32_OK;
}

DSO32_fct DSO32::Get_T_DRDY_Status(uint8_t* Status) {
  if (lsm6dso32_temp_flag_data_ready_get(&reg_ctx, Status))
    return DSO32_ERROR;
  return DSO32_OK;
}
DSO32_fct DSO32::Get_Temp(float_t* Temperature) {
  int16_t data_raw;

  if (lsm6dso32_temperature_raw_get(&reg_ctx, &data_raw))
    return DSO32_ERROR;
  Temperature[0] = lsm6dso32_from_lsb_to_celsius(data_raw);

  return DSO32_OK;
}

DSO32_fct DSO32::Enable_Pedometer() {
  lsm6dso32_pin_int1_route_t val;
  lsm6dso32_emb_sens_t emb_sens;

  Set_X(LSM6DSO32_XL_ODR_26Hz_LOW_PW);
  Set_X_FS(LSM6DSO32_4g);
  lsm6dso32_embedded_sens_get(&reg_ctx, &emb_sens);
  lsm6dso32_embedded_sens_off(&reg_ctx);
  delay(10);
  emb_sens.step = PROPERTY_ENABLE;
  lsm6dso32_pedo_sens_set(&reg_ctx, LSM6DSO32_PEDO_BASE_MODE);
  lsm6dso32_embedded_sens_set(&reg_ctx, &emb_sens);
  lsm6dso32_pin_int1_route_get(&reg_ctx, &val);
  val.emb_func_int1.int1_step_detector = PROPERTY_ENABLE;
  lsm6dso32_pin_int1_route_set(&reg_ctx, &val);

  return DSO32_OK;
}

uint8_t DSO32::IO_Read(uint8_t RegAddr, uint8_t* pBuff, uint16_t nByte) {
  if (dev_i2c) {
    dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
    dev_i2c->write(RegAddr);
    dev_i2c->endTransmission(false);
    dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t)nByte);
    int i = 0;
    while (dev_i2c->available()) {
      pBuff[i] = dev_i2c->read();
      i++;
    }
    return 0;
  }
  return 1;
}
int32_t platform_read(void* handle,
                      uint8_t RegAddr,
                      uint8_t* pBuff,
                      uint16_t nByte) {
  return ((DSO32*)handle)->IO_Read(RegAddr, pBuff, nByte);
}
uint8_t DSO32::IO_Write(uint8_t RegAddr, uint8_t* pBuff, uint16_t nByte) {
  if (dev_i2c) {
    dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
    dev_i2c->write(RegAddr);
    for (uint16_t i = 0; i < nByte; i++) {
      dev_i2c->write(pBuff[i]);
    }
    dev_i2c->endTransmission(true);
    return 0;
  }
  return 1;
}
int32_t platform_write(void* handle,
                       uint8_t RegAddr,
                       uint8_t* pBuff,
                       uint16_t nByte) {
  return ((DSO32*)handle)->IO_Write(RegAddr, pBuff, nByte);
}
