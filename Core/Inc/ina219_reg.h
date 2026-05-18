/**
 * @file    ina219_reg.h
 * @brief   INA219 current/power monitor low-level driver.
 *
 * API modelled after the ST MEMS C-driver pattern (stmdev_ctx_t).
 * The driver is independent of the HAL — it only calls the function pointers
 * stored in ina219_ctx_t.  Platform glue (HAL I2C) lives in ina219_driver.c.
 *
 * Reference: Texas Instruments INA219 datasheet (SBOS448G).
 * Inspired by the mindThomas C++ INA219 library:
 *   https://github.com/mindThomas/STM32-libraries/blob/master/Drivers/INA219/
 */

#ifndef INA219_REG_H_
#define INA219_REG_H_

#include <stdint.h>
#include <stddef.h>

/* --------------------------------------------------------------------------
 * Shared MEMS context type (same guard used by lsm6ds3tr_c_reg.h /
 * lis3mdl_reg.h so only one copy ends up in the translation unit).
 * -------------------------------------------------------------------------- */
#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef int32_t (*stmdev_write_ptr)(void *handle, uint8_t reg,
                                     const uint8_t *buf, uint16_t len);
typedef int32_t (*stmdev_read_ptr) (void *handle, uint8_t reg,
                                     uint8_t *buf, uint16_t len);
typedef void    (*stmdev_mdelay_ptr)(uint32_t millisec);

typedef struct {
    stmdev_write_ptr  write_reg;
    stmdev_read_ptr   read_reg;
    stmdev_mdelay_ptr mdelay;
    void             *handle;
    void             *priv_data;
} stmdev_ctx_t;

#endif /* MEMS_SHARED_TYPES */

/* INA219 reuses the shared context — identical function-pointer layout. */
typedef stmdev_ctx_t ina219_ctx_t;

/* --------------------------------------------------------------------------
 * I2C device addresses
 * 8-bit HAL format: (7-bit address << 1) | R/W  — HAL ignores the LSB.
 * -------------------------------------------------------------------------- */
#define INA219_I2C_ADD_GND_GND   0x80U  /* A0=GND, A1=GND  (7-bit 0x40) */
#define INA219_I2C_ADD_VCC_GND   0x82U  /* A0=VCC, A1=GND  (7-bit 0x41) */
#define INA219_I2C_ADD_GND_VCC   0x88U  /* A0=GND, A1=VCC  (7-bit 0x44) */
#define INA219_I2C_ADD_VCC_VCC   0x8AU  /* A0=VCC, A1=VCC  (7-bit 0x45) */

/* --------------------------------------------------------------------------
 * Register map
 * -------------------------------------------------------------------------- */
#define INA219_REG_CONFIG        0x00U
#define INA219_REG_SHUNT_VOLTAGE 0x01U
#define INA219_REG_BUS_VOLTAGE   0x02U
#define INA219_REG_POWER         0x03U
#define INA219_REG_CURRENT       0x04U
#define INA219_REG_CALIBRATION   0x05U

/* CONFIG register — reset value 0x399F */
#define INA219_CONFIG_RESET      0x8000U  /* Software reset bit */

/* --------------------------------------------------------------------------
 * CONFIG field enumerations (values pre-shifted to their bit positions)
 * -------------------------------------------------------------------------- */

/** Bus voltage range, bit 13 */
typedef enum {
    INA219_BUS_RANGE_16V = 0x0000U,   /**< 0..16 V  */
    INA219_BUS_RANGE_32V = 0x2000U,   /**< 0..32 V  */
} ina219_bus_range_t;

/** Programmable gain (shunt voltage range), bits [12:11] */
typedef enum {
    INA219_GAIN_1_40MV  = 0x0000U,   /**< Gain 1,  ±40 mV  */
    INA219_GAIN_2_80MV  = 0x0800U,   /**< Gain 2,  ±80 mV  */
    INA219_GAIN_4_160MV = 0x1000U,   /**< Gain 4,  ±160 mV */
    INA219_GAIN_8_320MV = 0x1800U,   /**< Gain 8,  ±320 mV */
} ina219_gain_t;

/** Bus ADC resolution / averaging, bits [10:7] */
typedef enum {
    INA219_BADC_9BIT    = 0x0000U,   /**< 9-bit,  84 µs   */
    INA219_BADC_10BIT   = 0x0080U,   /**< 10-bit, 148 µs  */
    INA219_BADC_11BIT   = 0x0100U,   /**< 11-bit, 276 µs  */
    INA219_BADC_12BIT   = 0x0180U,   /**< 12-bit, 532 µs  */
} ina219_badc_t;

/** Shunt ADC resolution / averaging, bits [6:3] */
typedef enum {
    INA219_SADC_9BIT_1S    = 0x0000U,  /**< 9-bit,  1 sample  84 µs   */
    INA219_SADC_10BIT_1S   = 0x0008U,  /**< 10-bit, 1 sample  148 µs  */
    INA219_SADC_11BIT_1S   = 0x0010U,  /**< 11-bit, 1 sample  276 µs  */
    INA219_SADC_12BIT_1S   = 0x0018U,  /**< 12-bit, 1 sample  532 µs  */
    INA219_SADC_12BIT_2S   = 0x0048U,  /**< 12-bit, 2 samples 1.06 ms */
    INA219_SADC_12BIT_4S   = 0x0050U,  /**< 12-bit, 4 samples 2.13 ms */
    INA219_SADC_12BIT_8S   = 0x0058U,  /**< 12-bit, 8 samples 4.26 ms */
    INA219_SADC_12BIT_16S  = 0x0060U,  /**< 12-bit, 16 samples 8.51 ms */
    INA219_SADC_12BIT_32S  = 0x0068U,  /**< 12-bit, 32 samples 17 ms  */
    INA219_SADC_12BIT_64S  = 0x0070U,  /**< 12-bit, 64 samples 34 ms  */
    INA219_SADC_12BIT_128S = 0x0078U,  /**< 12-bit, 128 samples 69 ms */
} ina219_sadc_t;

/** Operating mode, bits [2:0] */
typedef enum {
    INA219_MODE_POWERDOWN            = 0x0000U,
    INA219_MODE_SHUNT_TRIGGERED      = 0x0001U,
    INA219_MODE_BUS_TRIGGERED        = 0x0002U,
    INA219_MODE_SHUNT_BUS_TRIGGERED  = 0x0003U,
    INA219_MODE_ADC_OFF              = 0x0004U,
    INA219_MODE_SHUNT_CONTINUOUS     = 0x0005U,
    INA219_MODE_BUS_CONTINUOUS       = 0x0006U,
    INA219_MODE_SHUNT_BUS_CONTINUOUS = 0x0007U,
} ina219_mode_t;

/* --------------------------------------------------------------------------
 * Function prototypes
 * -------------------------------------------------------------------------- */

/** Raw 16-bit register read (big-endian conversion handled internally). */
int32_t ina219_read_reg (const ina219_ctx_t *ctx, uint8_t reg, uint16_t *val);

/** Raw 16-bit register write (big-endian conversion handled internally). */
int32_t ina219_write_reg(const ina219_ctx_t *ctx, uint8_t reg, uint16_t  val);

/** Software reset — waits one read cycle so the device settles. */
int32_t ina219_reset(const ina219_ctx_t *ctx);

/** Write calibration register (determines current LSB). */
int32_t ina219_calibration_set(const ina219_ctx_t *ctx, uint16_t cal);

/** Write CONFIG register (bus range, gain, ADC resolution, mode). */
int32_t ina219_config_set(const ina219_ctx_t *ctx,
                           ina219_bus_range_t  brng,
                           ina219_gain_t       gain,
                           ina219_badc_t       badc,
                           ina219_sadc_t       sadc,
                           ina219_mode_t       mode);

/** Read bus voltage register, shift out OVF/CNVR flags → raw mV value.
 *  Physical voltage = *val × 4 mV. */
int32_t ina219_bus_voltage_raw_get  (const ina219_ctx_t *ctx, int16_t  *val);

/** Read shunt voltage register.  Physical voltage = *val × 10 µV. */
int32_t ina219_shunt_voltage_raw_get(const ina219_ctx_t *ctx, int16_t  *val);

/** Read current register (requires prior calibration).
 *  Physical current [A] = *val × CurrentLSB. */
int32_t ina219_current_raw_get      (const ina219_ctx_t *ctx, int16_t  *val);

/** Read power register.
 *  Physical power [W] = *val × 20 × CurrentLSB. */
int32_t ina219_power_raw_get        (const ina219_ctx_t *ctx, uint16_t *val);

#endif /* INA219_REG_H_ */
