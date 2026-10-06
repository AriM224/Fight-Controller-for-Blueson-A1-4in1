//#include <Wire.h>
#include <SPI.h>
extern "C" {
  #include "lsm6dsv320x_reg.h"
}

// I²C device address (default 0x6A if SDO = GND, 0x6B if SDO = VDD)
#define LSM6DSV320X_I2C_ADDRESS  0x6A

// ST driver requires a handle for communication
#define DEV_ADDR NULL

#define LSM_CS 25

SPISettings imuSPI(1000000, MSBFIRST, SPI_MODE0);

// --- Platform read/write functions for the ST driver ---
int32_t spi_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    (void)handle;

    SPI.beginTransaction(imuSPI);
    digitalWrite(LSM_CS, LOW);

    // For SPI write, reg address MSB must be 0 (auto-inc works)
    SPI.transfer(reg & 0x7F);

    for (uint16_t i = 0; i < len; i++) {
        SPI.transfer(bufp[i]);
  }

    digitalWrite(LSM_CS, HIGH);
    SPI.endTransaction();
    return 0;
}

int32_t spi_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    (void)handle;

    SPI.beginTransaction(imuSPI);
    digitalWrite(LSM_CS, LOW);

    // For SPI read, set MSB = 1
    SPI.transfer(reg | 0x80);

    for (uint16_t i = 0; i < len; i++) {
        bufp[i] = SPI.transfer(0x00);
  }

    digitalWrite(LSM_CS, HIGH);
    SPI.endTransaction();
    return 0;
}

void platform_delay(uint32_t ms) {
  delay(ms);
}

// --- Global driver context ---
stmdev_ctx_t dev_ctx;

void setup() {
  Serial.begin(115200);

//  Wire.begin(); // SDA=D2 (GPIO4), SCL=D1 (GPIO5)
//  Wire.setClock(400000L);


    
    
  pinMode(LSM_CS, OUTPUT);
    digitalWrite(LSM_CS, HIGH);


//  SPI.begin(14, 12, 13, 25); /* begin SPI */
//  SPI.begin(14, 12, 13); /* begin S/PI */
  SPI.begin();
//    pinMode(12, PULLDOWN);
//    pinMode(14, INPUT_PULLUP);
//    pinMode(13, INPUT_PULLUP);
//    pinMode(33, INPUT_PULLUP);
  Serial.println("SPI start");
  // Initialize driver context
  dev_ctx.read_reg = spi_read;
  dev_ctx.write_reg = spi_write;
  dev_ctx.mdelay    = platform_delay;
  dev_ctx.handle    = DEV_ADDR;

  // Wait for sensor boot time
  delay(500);
  Serial.println("TEST");
  Serial.println("TEST");
  Serial.println("TEST");

  uint8_t whoamI = 0;

//  Serial.println(lsm6dsv320x_device_id_get(&dev_ctx, &whoamI));
  lsm6dsv320x_device_id_get(&dev_ctx, &whoamI);
//  while (whoamI != LSM6DSV320X_ID) {
//    Serial.println("Device not found!");
//    lsm6dsv320x_device_id_get(&dev_ctx, &whoamI);
    Serial.print("WHOAMI = 0x");
    Serial.println(whoamI, HEX);
//    delay(500);
//  }

//  Serial.print("reset 1: ");
//  Serial.println(lsm6dsv320x_sh_reset_set(&dev_ctx, 1));
  lsm6dsv320x_sh_reset_set(&dev_ctx, 1);
  delay(10);
//  Serial.print("reset 2: ");
//  Serial.println(lsm6dsv320x_sh_reset_set(&dev_ctx, 0));
  lsm6dsv320x_sh_reset_set(&dev_ctx, 0);
  delay(10);

//  lsm6dsv320x_xl_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_480Hz );
//  Serial.print("low accel rate: ");
//  Serial.println(lsm6dsv320x_xl_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_7680Hz ));
  lsm6dsv320x_xl_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_7680Hz );
//  Serial.print("low accel range: ");
//  Serial.println(lsm6dsv320x_xl_full_scale_set(&dev_ctx, LSM6DSV320X_4g));
  lsm6dsv320x_xl_full_scale_set(&dev_ctx, LSM6DSV320X_4g);

//  lsm6dsv320x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV320X_HG_XL_ODR_AT_480Hz, 1 );
//  Serial.print("high accel rate: ");
//  Serial.println(lsm6dsv320x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV320X_HG_XL_ODR_AT_7680Hz, 1 ));
  lsm6dsv320x_hg_xl_data_rate_set(&dev_ctx, LSM6DSV320X_HG_XL_ODR_AT_7680Hz, 1 );
//  Serial.print("high accel range: ");
//  Serial.println(lsm6dsv320x_hg_xl_full_scale_set(&dev_ctx, LSM6DSV320X_320g));
  lsm6dsv320x_hg_xl_full_scale_set(&dev_ctx, LSM6DSV320X_320g);

//  Serial.print("gyro rate: ");
//  Serial.println(lsm6dsv320x_gy_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_7680Hz ));
  lsm6dsv320x_gy_data_rate_set(&dev_ctx, LSM6DSV320X_ODR_AT_7680Hz );
//  Serial.print("gyro range: ");
//  Serial.println(lsm6dsv320x_gy_full_scale_set(&dev_ctx, LSM6DSV320X_4000dps));
  lsm6dsv320x_gy_full_scale_set(&dev_ctx, LSM6DSV320X_4000dps);

  Serial.println("LSM6DSV320X initialized!");
}

void loop() {
    int16_t data_raw_accel[3];
    float accel_mg[3];
    int16_t data_raw_accel_hg[3];
    float accel_mg_hg[3];
    int16_t data_raw_gyro[3];
    float gyro_mdps[3];

    lsm6dsv320x_acceleration_raw_get(&dev_ctx, data_raw_accel);

    accel_mg[0] = lsm6dsv320x_from_fs4_to_mg(data_raw_accel[0]);
    accel_mg[1] = lsm6dsv320x_from_fs4_to_mg(data_raw_accel[1]);
    accel_mg[2] = lsm6dsv320x_from_fs4_to_mg(data_raw_accel[2]);

    lsm6dsv320x_hg_acceleration_raw_get(&dev_ctx, data_raw_accel_hg);
  
    accel_mg_hg[0] = lsm6dsv320x_from_fs320_to_mg(data_raw_accel_hg[0]);
    accel_mg_hg[1] = lsm6dsv320x_from_fs320_to_mg(data_raw_accel_hg[1]);
    accel_mg_hg[2] = lsm6dsv320x_from_fs320_to_mg(data_raw_accel_hg[2]);

    lsm6dsv320x_angular_rate_raw_get(&dev_ctx, data_raw_gyro);

    gyro_mdps[0] = lsm6dsv320x_from_fs4000_to_mdps(data_raw_gyro[0]);
    gyro_mdps[1] = lsm6dsv320x_from_fs4000_to_mdps(data_raw_gyro[1]);
    gyro_mdps[2] = lsm6dsv320x_from_fs4000_to_mdps(data_raw_gyro[2]);

    
    Serial.print(accel_mg[0], 2);
    Serial.print(",");
    Serial.print(accel_mg[1], 2);
    Serial.print(",");
    Serial.println(accel_mg[2], 2);
    Serial.print(",");
    

//    Serial.print(accel_mg_hg[0], 2);
//    Serial.print(",");
//    Serial.print(accel_mg_hg[1], 2);
//    Serial.print(",");
//    Serial.println(accel_mg_hg[2], 2);
    
//    Serial.print(gyro_mdps[0], 2);
//    Serial.print(",");
//    Serial.print(gyro_mdps[1], 2);
//    Serial.print(",");
//    Serial.println(gyro_mdps[2], 2);

  delayMicroseconds(10000);
}
