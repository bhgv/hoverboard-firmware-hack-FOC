
// Includes
#include <stdlib.h> // for abs()
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "comms.h"
#include "eeprom.h"
#include "util.h"
#include "BLDC_controller.h"
#include "rtwtypes.h"

#ifdef CONTROL_M365

extern ExtY    rtY_Left;                /* External outputs */
extern ExtY    rtY_Right;               /* External outputs */

extern int16_t batVoltage;
extern uint8_t backwardDrive;
extern uint8_t buzzerFreq;              // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern;           // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable;                  // global variable for motor enable

extern volatile uint32_t timeoutCnt;    // global variable for general timeout counter
extern volatile uint32_t main_loop_counter;

extern int16_t  cmd1;                   // normalized input value. -1000 to 1000
extern int16_t  cmd2;                   // normalized input value. -1000 to 1000

extern uint16_t timeoutCntSerial_R;  	// Timeout counter for Rx Serial command
extern uint8_t  timeoutFlagSerial_R;  	// Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

extern uint8_t timeoutFlagADC;          // Timeout Flag for ADC Protection:    0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)
extern uint8_t timeoutFlagSerial;       // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern SerialCommand command;
extern SerialCommand command_raw;
extern uint32_t command_len;

extern uint8_t  ctrlModReqRaw;
extern uint8_t  ctrlModReq;             // Final control mode request

extern uint8_t rx_buffer_R[]; // USART Rx DMA circular buffer
extern uint32_t rx_buffer_R_len;

static uint8_t spd_md = SPD_MD_1;


/* =========================== xiaomi-m365 calculateChecksum Function =========== */
uint16_t calculateChecksum(uint8_t *data) {
  uint8_t len = data[2] + 2;
  uint16_t sum = 0;
  for (int i = 0; i < len; i++)
    sum += data[2+i];
  sum ^= 0xFFFF;
  return sum;
}


/* =========================== Send Response Function =========================== */
SerialResp fb;

uint16_t max_n_mot = 0;

void sendRespUart(void) {
/*
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
*/

  fb.mag_55 = 0x55;
  fb.mag_aa = 0xaa;

  fb.len_06 = 0x06;
  fb.fl_21  = 0x21;
  fb.fl_64  = 0x64;
  fb.fl_00  = 0x00;

  fb.y_led_md = 0; // 0 - off, 2 - on
  fb.leds_cnt = 5; // 0-7
  fb.night_md = 0; // 0x64 - on, 0x00 - off
  fb.dk_00  = 0;

  uint16_t checksum = calculateChecksum((uint8_t*)&fb);

  fb.checksum_h = (uint8_t)(checksum >> 8);
  fb.checksum_l = (uint8_t)(checksum & 0xff);

  while (__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {}

  HAL_HalfDuplex_EnableTransmitter(&huart3);                  // half duplex to Tx
  int i;
  for (i = 0; i < 3; i++)
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&fb, sizeof(SerialResp));
  HAL_HalfDuplex_EnableReceiver(&huart3);                     // half duplex to Rx
}



void poweroffPressCheck(void) {
  static int isOn = 0;
  if (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
    enable = 0;  // disable motors
    poweroff();  // release power-latch
  }
}


uint16_t max_cmd2 = 0;

void readCommand(void) {
    #if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
      /*
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
      */

      int spd = (int)command.spd;
      int stp = (int)command.brk;

      if(spd > MAX_ACC) spd = MAX_ACC;
      if(stp > MAX_ACC) stp = MAX_ACC;

      if(spd < MIN_ACC) spd = MIN_ACC;
      if(stp < MIN_ACC) stp = MIN_ACC;

      uint32_t ac = (uint32_t)spd - MIN_ACC; //0x27 -- MAX_ACC - sniffed max
      uint32_t st = (uint32_t)stp - MIN_ACC; //0x29 -- MAX_ACC - sniffed max

      uint32_t max_pwm_spd = 0;
      switch(spd_md){
        case SPD_MD_1:
          max_pwm_spd = SPD1;
          break;

        case SPD_MD_2:
          max_pwm_spd = SPD2;
          break;

        case SPD_MD_3:
          max_pwm_spd = SPD3;
          break;

        case SPD_MD_4:
          max_pwm_spd = SPD4;
          break;

      }

      uint32_t pwm_spd = max_pwm_spd * ac;
      cmd2 = (int16_t)(pwm_spd / (MAX_ACC - MIN_ACC));

      uint32_t pwm_stp = (uint32_t)N_MOT_MAX * st;
      cmd1 = (int16_t)(pwm_stp / (MAX_ACC - MIN_ACC));

      if(cmd2 > max_cmd2) max_cmd2 = cmd2;
      #endif

      timeoutCnt = 0;

    #if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
      if (timeoutCntSerial_L++ >= SERIAL_TIMEOUT) {     // Timeout qualification
        timeoutFlagSerial_L = 1;                        // Timeout detected
        timeoutCntSerial_L  = SERIAL_TIMEOUT;           // Limit timout counter value
      }
      timeoutFlagSerial = timeoutFlagSerial_L;
    #endif
    #if defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
      if (timeoutCntSerial_R++ >= SERIAL_TIMEOUT) {     // Timeout qualification
        timeoutFlagSerial_R = 1;                        // Timeout detected
        timeoutCntSerial_R  = SERIAL_TIMEOUT;           // Limit timout counter value
      }
      timeoutFlagSerial = timeoutFlagSerial_R;
    #endif

    if (timeoutFlagADC || timeoutFlagSerial || timeoutCnt > TIMEOUT) {  // In case of timeout bring the system to a Safe State
      ctrlModReq  = OPEN_MODE;                                          // Request OPEN_MODE. This will bring the motor power to 0 in a controlled way
      cmd1        = 0;
      cmd2        = 0;
    } else {
      ctrlModReq  = ctrlModReqRaw;                                      // Follow the Mode request
    }

}


void usart3_rx_check(void)
{
  #if defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  static uint32_t old_pos = 0;
  uint32_t pos;
  pos = rx_buffer_R_len - __HAL_DMA_GET_COUNTER(huart3.hdmarx);         // Calculate current position in buffer

  if (old_pos > pos) old_pos = 0;
  #endif

  #if defined(DEBUG_SERIAL_USART3)
  if (pos != old_pos) {                                                 // Check change in received data
    if (pos > old_pos) {                                                // "Linear" buffer mode: check if current position is over previous one
      usart_process_debug(&rx_buffer_R[old_pos], pos - old_pos);        // Process data
    } else {                                                            // "Overflow" buffer mode
      usart_process_debug(&rx_buffer_R[old_pos], rx_buffer_R_len - old_pos); // First Process data from the end of buffer
      if (pos > 0) {                                                    // Check and continue with beginning of buffer
        usart_process_debug(&rx_buffer_R[0], pos);                      // Process remaining data
      }
    }
  }
  #endif // DEBUG_SERIAL_USART3

  #ifdef CONTROL_SERIAL_USART3
  uint8_t *ptr;

  static uint8_t st = 0;
  uint8_t len;

  if (pos != old_pos) {                                                 // Check change in received data
    ptr = (uint8_t *)&command_raw;                                      // Initialize the pointer with command_raw address
    int i;
    for(; st < 10 && old_pos < pos; old_pos++) {
      uint8_t c = rx_buffer_R[old_pos];
      ptr[st] = c;
      switch(st) {
        case 0:
          if(c == 0x55) st = 1;
          break;

        case 1:
          if(c == 0xaa) st = 2;
          else st = 0;
          break;

        case 2:
          if(c == 0x07 || c == 0x09) {
            len = c;
            st = 10;
          }
          else st = 0;
          break;

      };
    }
    if (st >= 10) {
      command_len = len + 2 + 2;
      if (pos > old_pos && (pos - old_pos) >= command_len) {              // "Linear" buffer mode: check if current position is over previous one AND data length equals expected length
        memcpy(&ptr[3], &rx_buffer_R[old_pos], command_len);              // Copy data. This is possible only if command_raw is contiguous! (meaning all the structure members have the same size)
        usart_process_command(&command_raw, &command, 3);                 // Process data
        st = 0;
        old_pos += command_len;
      } else if ((rx_buffer_R_len - old_pos + pos) == command_len) {      // "Overflow" buffer mode: check if data length equals expected length
        memcpy(ptr, &rx_buffer_R[old_pos], rx_buffer_R_len - old_pos);    // First copy data from the end of buffer
        if (pos > 0) {                                                    // Check and continue with beginning of buffer
          ptr += rx_buffer_R_len - old_pos;                               // Move to correct position in command_raw
          memcpy(ptr, &rx_buffer_R[0], pos);                              // Copy remaining data
        }
        usart_process_command(&command_raw, &command, 3);                 // Process data
        st = 0;
        old_pos += command_len;
      }
    }
  }
  #endif // CONTROL_SERIAL_USART3

  #if defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
    if (old_pos >= rx_buffer_R_len) {                                     // Check and manually update if we reached end of buffer
      old_pos = 0;
    }
  #endif
}


int usart_process_command(SerialCommand *command_in, SerialCommand *command_out, uint8_t usart_idx)
{
  int ret = sizeof(SerialCommand);

  uint16_t checksum = 0;
/*
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
*/
  if (
    command_in->mag_55 != 0x55
    || command_in->mag_aa != 0xaa
    || !(command_in->len_7or9 == 0x07 && command_in->len_7or9 == 0x09)
  ) {
    ret = 1;
  } else {
    checksum = calculateChecksum((uint8_t*)command_in);
    if ( (command_in->len_7or9 == 7 &&
          checksum == ((uint16_t)command_in->checksum7_h << 8) + (uint16_t)command_in->checksum7_l
        ) || (command_in->len_7or9 == 9 &&
          checksum == ((uint16_t)command_in->checksum9_h << 8) + (uint16_t)command_in->checksum9_l
        )
    ) {
      *command_out = *command_in;
      if (usart_idx == 2) {             // Sideboard USART2
  #ifdef CONTROL_SERIAL_USART2
        timeoutCntSerial_L  = 0;        // Reset timeout counter
        timeoutFlagSerial_L = 0;        // Clear timeout flag
  #endif
      } else if (usart_idx == 3) {      // Sideboard USART3
  #ifdef CONTROL_SERIAL_USART3
        timeoutCntSerial_R  = 0;        // Reset timeout counter
        timeoutFlagSerial_R = 0;        // Clear timeout flag
  #endif
      }
    }
  }

  return ret;
}

#endif // #ifdef CONTROL_M365

