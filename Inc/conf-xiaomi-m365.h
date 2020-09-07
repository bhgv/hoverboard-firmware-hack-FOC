
#ifndef _CONF_XIAOMI_M365_
#define _CONF_XIAOMI_M365_

#define CONTROL_M365                  // use xiaomi-m365 controller

#define VARIANT_USART

// #define CONTROL_SERIAL_USART2      // left cable, disable if ADC or PPM is used!
// #define FEEDBACK_SERIAL_USART2     // left cable, disable if ADC or PPM is used!

#define CONTROL_SERIAL_USART3         // right cable, disable if I2C is used!
#define FEEDBACK_SERIAL_USART3        // right cable, disable if I2C is used!

#define USART3_BAUD 115200            // UART3 baud rate (short wired cable)



// ############################### Xiaomi m365 defs ################################

//#define SPD1 0x999   // 1024 * 60 / 100
#define SPD1 0x500   // 1024 * 45 / 100
#define SPD2 0x800   // 1024 * 45 / 100
//#define SPD2 0xccc   // 4096 * 80 / 100
#define SPD3 0xb00   // 4096 * 70 / 100
#define SPD4 0xfff   // 4096 * 100 / 100 - 1 (max)

#define LEDS_MAX 8

#define MAX_ACC 0xa0 //0x78  // 0x78 -- 3.3v  0xB2 -- 5v
#define MIN_ACC 0x30 //0x78  // 0x78 -- 3.3v  0xB2 -- 5v

enum {
    ST_OFF = 0,
    ST_ON,
    ST_LT,
};

enum {
    SPD_MD_1 = 0,
    SPD_MD_2,
    SPD_MD_3,
    SPD_MD_4,
};


typedef struct {
    uint8_t mag_55;
    uint8_t mag_aa;

    uint8_t len_7or9;
    uint8_t fl_20;
    uint8_t fl_65;
    uint8_t fl_00_1;
    uint8_t fl_04;
    uint8_t spd;
    uint8_t brk;
    uint8_t fl_00_2;
    uint8_t fl_00_3;

    uint8_t checksum7_l;
    uint8_t checksum7_h;

    uint8_t checksum9_l;
    uint8_t checksum9_h;
} SerialCommand;

typedef struct {
    uint8_t mag_55;
    uint8_t mag_aa;

    uint8_t len_06;
    uint8_t fl_21;
    uint8_t fl_64;
    uint8_t fl_00;
    uint8_t y_led_md; // 0 - off, 2 - on
    uint8_t leds_cnt; // 0-7
    uint8_t night_md; // 0x64 - on, 0x00 - off
    uint8_t dk_00;

    uint8_t checksum_l;
    uint8_t checksum_h;
} SerialResp;




#endif
