

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

#ifdef CONTROL_S3

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


/* =========================== Send Response Function =========================== */
SerialResp fb;

uint16_t max_n_mot = 0;

void sendRespUart(void) {
//  SerialResp fb;
/*
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
*/

  fb.mag_03c = 0x3c;
  fb.mag_07 = 0x07;

  fb.status = 0x01;
  fb.svc_flags = 0x00;
  fb.current = 0x80;

  if (rtY_Left.n_mot > max_n_mot) max_n_mot = rtY_Left.n_mot;

  uint16_t tspd_i = rtY_Left.n_mot;

  fb.spd_l = (uint8_t)(tspd_i & 0xff);
  fb.spd_h = (uint8_t)(tspd_i >> 8);

  fb.dk_00_1 = 0x00;
  fb.dk_00_2 = 0x00;

  uint16_t sum = 0;
  int i;
  for (i = 0; i < sizeof(SerialResp) - 2; i++)
    sum += ((uint8_t*)&fb)[i];

  fb.checksum_h = (uint8_t)(sum >> 8);
  fb.checksum_l = (uint8_t)(sum & 0xff);

  //#if defined(FEEDBACK_SERIAL_USART3)
  if(__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {

    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&fb, sizeof(SerialResp));
  }
  //#endif
}


void poweroffPressCheck(void) {
    if (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      enable = 0;                                             // disable motors
      poweroff();                                             // release power-latch
    }
}


uint16_t max_cmd2 = 0;

void readCommand(void) {
#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
  /*
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
  */

    cmd1 = command.brk;

    cmd2 = command.spd;

    if(cmd2 > max_cmd2) max_cmd2 = cmd2;

    timeoutCnt = 0;
  #endif

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

  if (pos != old_pos) {                                                 // Check change in received data
    ptr = (uint8_t *)&command_raw;                                      // Initialize the pointer with command_raw address
    int i;
    for(; st < 10 && old_pos < pos; old_pos++) {
      uint8_t c = rx_buffer_R[old_pos];
      ptr[st] = c;
      switch(st) {
        case 0:
          if(c == 0x3e) st = 1;
          break;

        case 1:
          if(c == 0x04) st = 10;
          else st = 0;
          break;

      };
    }
    if (st >= 10) {
      if (pos > old_pos && (pos - old_pos) >= command_len - 4) {          // "Linear" buffer mode: check if current position is over previous one AND data length equals expected length
        memcpy(&ptr[4], &rx_buffer_R[old_pos], command_len - 4);          // Copy data. This is possible only if command_raw is contiguous! (meaning all the structure members have the same size)
        usart_process_command(&command_raw, &command, 3);                 // Process data
        st = 0;
        old_pos += command_len - 4;
      } else if ((rx_buffer_R_len - (old_pos - 4) + pos) == command_len) {      // "Overflow" buffer mode: check if data length equals expected length
        memcpy(ptr, &rx_buffer_R[old_pos], rx_buffer_R_len - old_pos);    // First copy data from the end of buffer
        if (pos > 0) {                                                    // Check and continue with beginning of buffer
          ptr += rx_buffer_R_len - old_pos;                               // Move to correct position in command_raw
          memcpy(ptr, &rx_buffer_R[0], pos);                              // Copy remaining data
        }
        usart_process_command(&command_raw, &command, 3);                 // Process data
        st = 0;
        old_pos += command_len - 4;
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
  uint8_t mag_3e; // 0
  uint8_t mag_04; // 1

  uint8_t md;  // mode 1, 2, 3 // 2
  uint8_t night; // 0 or 0x80 // 3
  uint8_t spd; // 4
  uint8_t brk; // 5

  uint8_t checksum_h; // 6
  uint8_t checksum_l; // 7
} SerialCommand;
*/
  if (
    command_in->mag_3e != 0x3e
    || command_in->mag_04 != 0x04
  ) {
    ret = 1;
  } else {
    int i;
    for (i = 0; i < sizeof(SerialCommand) - 2; i++)
      checksum += ((uint8_t*)command_in)[i];
    if (checksum == ((uint16_t)command_in->checksum_h << 8) + command_in->checksum_l) {
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





#endif // #ifdef CONTROL_S3
