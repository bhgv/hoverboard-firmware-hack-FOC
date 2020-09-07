
#ifndef _CONF_KUGOO_M2_
#define _CONF_KUGOO_M2_

#define CONTROL_S3                // use JX-168 like (Kugoo m2-3-4) controller

#define VARIANT_USART

// #define CONTROL_SERIAL_USART2      // left cable, disable if ADC or PPM is used!
// #define FEEDBACK_SERIAL_USART2     // left cable, disable if ADC or PPM is used!

#define CONTROL_SERIAL_USART3         // right cable, disable if I2C is used!
#define FEEDBACK_SERIAL_USART3        // right cable, disable if I2C is used!

#define USART3_BAUD 9600                              // UART3 baud rate (short wired cable)



typedef struct {
  uint8_t mag_3e; // 0
  uint8_t mag_04; // 1

  uint8_t md;  // mode 1, 2, 3 // 2
  uint8_t night; // 0 or 0x80 // 3
  uint8_t spd; // 4
  uint8_t brk; // 5

  uint8_t checksum_h; // 6
  uint8_t checksum_l; // 7
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

