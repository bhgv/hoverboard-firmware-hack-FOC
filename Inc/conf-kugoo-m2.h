
#ifndef _CONF_KUGOO_M2_
#define _CONF_KUGOO_M2_

#define CONTROL_JX_168                // use JX-168 like (Kugoo m2-3-4) controller

#define VARIANT_USART

//#define _4x4_

#ifdef _4x4_
#define CONTROL_SERIAL_USART2      // left cable
#endif
// #define FEEDBACK_SERIAL_USART2     // left cable, disable if ADC or PPM is used!

#define CONTROL_SERIAL_USART3         // right cable, disable if I2C is used!
#define FEEDBACK_SERIAL_USART3        // right cable, disable if I2C is used!

#define USART3_BAUD 9600                              // UART3 baud rate (short wired cable)

#ifdef _4x4_
#define USART2_BAUD 9600                              // UART2 baud rate (long wired cable)
#endif


typedef struct {
  uint8_t mag_01_1; // begin of packet
  uint8_t mag_14;   // len of packet
  uint8_t mag_01_2; // ?ver of protocol?
  uint8_t mag_01_3; // ?ver of protocol?

  uint8_t spd_md;  // 0x05 - 1 spd, 0x0a - 2 spd, 0x0f - 3 spd

  uint8_t dk_c0;   // flags: 6bt - zero start, 5bt - light en
  uint8_t poles;   // motor poles (0x18)
  uint8_t diam_lo;   // diam in inches * 10. lo (90)
  uint8_t diam_hi; // diam in inches * 10. hi (0)
  uint8_t dk_01;
  uint8_t dk_05_1;
  uint8_t dk_64;   // ?max speed?
  uint8_t dk_00_2;
  uint8_t dk_0c;
  uint8_t dk_00_3;
  uint8_t dk_00_4;

  uint8_t spd_1;   // speed 0-991. hi
  uint8_t spd_2;   // speed 0-991. lo

  uint8_t dk_05_2;

  uint8_t checksum;
} SerialCommand;

typedef struct {
  uint8_t mag_02;  // begin of packet
  uint8_t mag_0e;  // len of packet

  uint8_t dk_01;   // 
  uint8_t dk_00_1; // bt6 - E001, bt4 - E003, bt3 - E005
  uint8_t dk_80;   // bt5 - brk endswitch status, bt4 - display conn error (E007)
  uint8_t dk_00_2;
  uint8_t dk_00_3;

  uint8_t curr;    // motor current * 10
  uint8_t spd_2;
  uint8_t spd_3;

  uint8_t dk_00_4;
  uint8_t dk_00_5;
  uint8_t dk_ff;

  uint8_t checksum;
} SerialResp;



#endif

