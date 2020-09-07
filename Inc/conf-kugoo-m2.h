
#ifndef _CONF_KUGOO_M2_
#define _CONF_KUGOO_M2_

#define CONTROL_JX_168                // use JX-168 like (Kugoo m2-3-4) controller

#define VARIANT_USART

// #define CONTROL_SERIAL_USART2      // left cable, disable if ADC or PPM is used!
// #define FEEDBACK_SERIAL_USART2     // left cable, disable if ADC or PPM is used!

#define CONTROL_SERIAL_USART3         // right cable, disable if I2C is used!
#define FEEDBACK_SERIAL_USART3        // right cable, disable if I2C is used!

#define USART3_BAUD 9600                              // UART3 baud rate (short wired cable)



typedef struct {
  uint8_t mag_01_1;
  uint8_t mag_14;
  uint8_t mag_01_2;
  uint8_t mag_01_3;

  uint8_t spd_md;  // 0x05 - 1 spd, 0x0a - 2 spd, 0x0f - 3 spd

  uint8_t dk_c0;
  uint8_t dk_18;
  uint8_t dk_5a;
  uint8_t dk_00_1;
  uint8_t dk_01;
  uint8_t dk_05_1;
  uint8_t dk_64;
  uint8_t dk_00_2;
  uint8_t dk_0c;
  uint8_t dk_00_3;
  uint8_t dk_00_4;

  uint8_t spd_1;
  uint8_t spd_2;

  uint8_t dk_05_2;

  uint8_t checksum;
} SerialCommand;

typedef struct {
  uint8_t mag_02;
  uint8_t mag_0e;

  uint8_t dk_01;
  uint8_t dk_00_1;
  uint8_t dk_80;
  uint8_t dk_00_2;
  uint8_t dk_00_3;

  uint8_t spd_1;
  uint8_t spd_2;
  uint8_t spd_3;

  uint8_t dk_00_4;
  uint8_t dk_00_5;
  uint8_t dk_ff;

  uint8_t checksum;
} SerialResp;



#endif

