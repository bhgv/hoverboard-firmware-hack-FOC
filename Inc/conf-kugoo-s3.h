
#ifndef _CONF_KUGOO_M2_
#define _CONF_KUGOO_M2_

#define CONTROL_S3  // use JX-168 like (Kugoo m2-3-4) controller

#define VARIANT_USART

// #define CONTROL_SERIAL_USART2            // left cable, disable if ADC or PPM
// is used! #define FEEDBACK_SERIAL_USART2     // left cable, disable if ADC or
// PPM is used!

#define CONTROL_SERIAL_USART3   // right cable, disable if I2C is used!
#define FEEDBACK_SERIAL_USART3  // right cable, disable if I2C is used!

#define USART3_BAUD 9600  // UART3 baud rate (short wired cable)

typedef struct {
  uint8_t mag_3e;  // 0
  uint8_t mag_04;  // 1

  uint8_t md;     // mode 1, 2, 3 // 2
  uint8_t night;  // 0 or 0x80    // 3
  uint8_t spd;    // 4
  uint8_t brk;    // 5

  uint8_t checksum_h;  // 6
  uint8_t checksum_l;  // 7
} SerialCommand;

typedef struct {
  uint8_t mag_03c;
  uint8_t mag_07;

  uint8_t status;     // 0 - blocked, 1 - normal, 3 - configs accepted
  uint8_t svc_flags;  // 1 - "M", 2 - "ECU", 4 - "!"
  uint8_t current;
  uint8_t spd_h;  // time between hal pulses
  uint8_t spd_l;  //

  uint8_t dk_00_1;
  uint8_t dk_00_2;

  uint8_t checksum_h;  // 6
  uint8_t checksum_l;  // 7
} SerialResp;

#endif
