

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

#ifdef CONTROL_JX_168

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

#ifdef CONTROL_SERIAL_USART2
extern uint16_t timeoutCntSerial_L;  	// Timeout counter for Rx Serial command
extern uint8_t  timeoutFlagSerial_L;  	// Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
#endif

extern uint8_t timeoutFlagADC;          // Timeout Flag for ADC Protection:    0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)
extern uint8_t timeoutFlagSerial;       // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern SerialCommand command;
extern SerialCommand command_raw;
extern uint32_t command_len;

#if defined(CONTROL_SERIAL_USART2) && defined(_4x4_MASTER)
extern SerialResp command2;
extern SerialResp command2_raw;
extern uint32_t command2_len;
#endif

extern uint8_t  ctrlModReqRaw;
extern uint8_t  ctrlModReq;             // Final control mode request

extern uint8_t rx_buffer_R[]; // USART Rx DMA circular buffer
extern uint32_t rx_buffer_R_len;

#ifdef CONTROL_SERIAL_USART2
extern uint8_t rx_buffer_L[]; // USART Rx DMA circular buffer
extern uint32_t rx_buffer_L_len;
#endif

extern int16_t curR_DC, curL_DC;

extern int16_t INPUT_MAX;             // [-] Input target maximum limitation
extern int16_t INPUT_MIN;             // [-] Input target minimum limitation

#ifdef CONTROL_ADC
  extern uint16_t ADC1_MIN_CAL;
  extern uint16_t ADC1_MAX_CAL;
  extern uint16_t ADC2_MIN_CAL;
  extern uint16_t ADC2_MAX_CAL;

  extern volatile adc_buf_t adc_buffer;
#endif

uint8_t  poles = 30;
uint16_t wl_diam_inch = 80;


int32_t k_brk = 0;


int usart2_process_command(SerialResp *command_in, SerialResp *command_out, uint8_t usart_idx);


/* =========================== Send Response Function =========================== */
SerialResp fb;

uint16_t max_n_mot = 0;

void sendRespUart(void) {
//  SerialResp fb;

  fb.mag_02 = 0x02;
  fb.mag_0e = 0x0e;

  fb.dk_01 = 0x01;
  fb.dk_00_1 = 0x00;
  fb.dk_80 = 0x80;
  fb.dk_00_2 = 0x00;
  fb.dk_00_3 = 0x00;

  if (rtY_Left.n_mot > max_n_mot) max_n_mot = rtY_Left.n_mot;

//  double   tspd_f = 10. * 1000000.0 / (200. * (double)rtY_Left.n_mot);
  double    tspd_f = 60. * 60. *
                     3.1415 * ((double)wl_diam_inch / 10.) * 0.025 * 0.001 /
                     ((double)poles * (double)rtY_Left.n_mot * 0.000001);
  uint16_t  tspd_i = 0x1770; //rtY_Right.n_mot; //0x1770;

  if (tspd_i > (uint16_t)tspd_f)
    tspd_i = (uint16_t)tspd_f;

  fb.curr = (uint8_t)((uint32_t)10 * ((uint32_t)ABS(curL_DC) + (uint32_t)ABS(curR_DC)) / A2BIT_CONV);
  fb.spd_2 = (uint8_t)(tspd_i >> 8);
  fb.spd_3 = (uint8_t)(tspd_i & 0xff);

  uint16_t tmp_k_brk = (uint16_t)k_brk;
  fb.dk_00_4 = (uint8_t)(tmp_k_brk >> 8);   //0x00;
  fb.dk_00_5 = (uint8_t)(tmp_k_brk & 0xff); //0x00;
  fb.dk_ff = 0xff;

  fb.checksum = 0x00;

  int i;
  for (i = 0; i < sizeof(SerialResp) - 1; i++)
    fb.checksum ^= ((uint8_t*)&fb)[i];

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
    poles = command.poles;

    wl_diam_inch = ((uint16_t)command.diam_hi << 8) + (uint16_t)command.diam_lo;

    cmd1 = 0;

    uint32_t tmp = (uint32_t)(((uint32_t)command.spd_1 << 8) | (uint32_t)command.spd_2);
    cmd2 = (int16_t)(((uint32_t)N_MOT_MAX * tmp) / (uint32_t)991 /*0x03df*/);

    if(cmd2 > max_cmd2) max_cmd2 = cmd2;

    timeoutCnt = 0;
  #endif

  #if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
/*
    if (timeoutCntSerial_L++ >= SERIAL_TIMEOUT) {     // Timeout qualification
      timeoutFlagSerial_L = 1;                        // Timeout detected
      timeoutCntSerial_L  = SERIAL_TIMEOUT;           // Limit timout counter value
    }
    timeoutFlagSerial = timeoutFlagSerial_L;
*/
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

#if !defined(_4x4_MASTER)
  #ifdef CONTROL_ADC
    // ADC values range: 0-4095, see ADC-calibration in config.h
    #ifdef ADC1_MID_POT
      cmd1 = CLAMP((adc_buffer.l_tx2 - ADC1_MID_CAL) * INPUT_MAX / (ADC1_MAX_CAL - ADC1_MID_CAL), 0, INPUT_MAX)
            +CLAMP((ADC1_MID_CAL - adc_buffer.l_tx2) * INPUT_MIN / (ADC1_MID_CAL - ADC1_MIN_CAL), INPUT_MIN, 0);    // ADC1
    #else
      static int32_t tmp_s = 0;
      static int32_t tmp_i = 0;

      static int32_t tmp_adc = 0, tmp_adc_o = 0;

      tmp_adc = adc_buffer.l_tx2;
      tmp_s += tmp_adc;
      tmp_i++;

      if (tmp_i >= 5) {
        /*cmd1*/
        tmp_adc = (tmp_s / tmp_i);
        if (tmp_adc > INPUT_MAX - 20 && tmp_adc - tmp_adc_o > (INPUT_MAX - INPUT_MIN) / 5)
          tmp_adc = tmp_adc_o + (INPUT_MAX - INPUT_MIN) / 5;
        tmp_adc_o = tmp_adc;

#ifndef _4x4_no_break
        int32_t k_brk_tmp =
                      (tmp_adc - (int32_t)ADC1_MIN_CAL) 
                      * (int32_t)INPUT_MAX 
                      / ((int32_t)ADC1_MAX_CAL - (int32_t)ADC1_MIN_CAL);
        k_brk = CLAMP(
                    k_brk_tmp,
                    (int32_t)0, 
                    (int32_t)INPUT_MAX
                  );    // ADC1
#else
        k_brk = 0;
#endif
        tmp_i = 0;
        tmp_s = 0;
      }
//      if (cmd1 < 5) {
//        k_brk = 0;
//      } else {
//        k_brk = (uint32_t)cmd1; //(float)cmd1 / (float)INPUT_MAX;
//      }
    #endif

    cmd1 = 0;

#if 0 //{}
    #ifdef ADC2_MID_POT
      cmd2 = CLAMP((adc_buffer.l_rx2 - ADC2_MID_CAL) * INPUT_MAX / (ADC2_MAX_CAL - ADC2_MID_CAL), 0, INPUT_MAX)
            +CLAMP((ADC2_MID_CAL - adc_buffer.l_rx2) * INPUT_MIN / (ADC2_MID_CAL - ADC2_MIN_CAL), INPUT_MIN, 0);    // ADC2
    #else
      cmd2 = CLAMP((adc_buffer.l_rx2 - ADC2_MIN_CAL) * INPUT_MAX / (ADC2_MAX_CAL - ADC2_MIN_CAL), 0, INPUT_MAX);    // ADC2
    #endif
#endif //{}

    #ifdef ADC_PROTECT_ENA
      if (adc_buffer.l_tx2 >= (ADC1_MIN_CAL - ADC_PROTECT_THRESH) && adc_buffer.l_tx2 <= (ADC1_MAX_CAL + ADC_PROTECT_THRESH) &&
          adc_buffer.l_rx2 >= (ADC2_MIN_CAL - ADC_PROTECT_THRESH) && adc_buffer.l_rx2 <= (ADC2_MAX_CAL + ADC_PROTECT_THRESH)) {
        if (timeoutFlagADC) {                         // Check for previous timeout flag
          if (timeoutCntADC-- <= 0)                   // Timeout de-qualification
            timeoutFlagADC  = 0;                      // Timeout flag cleared
        } else {
          timeoutCntADC     = 0;                      // Reset the timeout counter
        }
      } else {
        if (timeoutCntADC++ >= ADC_PROTECT_TIMEOUT) { // Timeout qualification
          timeoutFlagADC    = 1;                      // Timeout detected
          timeoutCntADC     = ADC_PROTECT_TIMEOUT;    // Limit timout counter value
        }
      }
    #endif

    timeoutCnt = 0;
  #endif
#else
  cmd1 = 0;
#endif
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
          if(c == 0x01) st = 1;
          break;

        case 1:
          if(c == 0x14) st = 2;
          else st = 0;
          break;

        case 2:
          if(c == 0x01) st = 3;
          else st = 0;
          break;

        case 3:
          if(c == 0x01 || c == 0x02) st = 10;
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

  uint8_t checksum = 0;

  if (
    command_in->mag_01_1 != 0x01
    || command_in->mag_14 != 0x14
    || command_in->mag_01_2 != 0x01
    || (command_in->mag_01_3 != 0x01 && command_in->mag_01_3 != 0x02)
  ) {
    ret = 1;
  } else {
    int i;
    for (i = 0; i < sizeof(SerialCommand); i++)
      checksum ^= ((uint8_t*)command_in)[i];
    if (checksum == 0) {
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
    #ifdef _4x4_MASTER
        if(__HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {
          HAL_UART_Transmit_DMA(&huart2, (uint8_t *)command_in, sizeof(SerialCommand));
        }
    #endif
  #endif
      }
    }
  }

  return ret;
}



// -- master - UART2 -- VVV --

void usart2_rx_check(void)
{
  #if defined(CONTROL_SERIAL_USART2)
  static uint32_t old_pos = 0;
  uint32_t pos;
  pos = rx_buffer_L_len - __HAL_DMA_GET_COUNTER(huart2.hdmarx);         // Calculate current position in buffer

  if (old_pos > pos) old_pos = 0;

  uint8_t *ptr;

  static uint8_t st = 0;

//  fb.mag_02 = 0x02;
//  fb.mag_0e = 0x0e;

  if (pos != old_pos) {                                                 // Check change in received data
    ptr = (uint8_t *)&command2_raw;                                      // Initialize the pointer with command_raw address
    int i;
    for(; st < 10 && old_pos < pos; old_pos++) {
      uint8_t c = rx_buffer_L[old_pos];
      ptr[st] = c;
      switch(st) {
        case 0:
          if(c == 0x02) st = 1;
          break;

        case 1:
          if(c == 0x0e) st = 10;
          else st = 0;
          break;

      };
    }
    if (st >= 10) {
      if (pos > old_pos && (pos - old_pos) >= command2_len - 2) {          // "Linear" buffer mode: check if current position is over previous one AND data length equals expected length
        memcpy(&ptr[2], &rx_buffer_L[old_pos], command2_len - 2);          // Copy data. This is possible only if command_raw is contiguous! (meaning all the structure members have the same size)
        usart2_process_command(&command2_raw, &command2, 2);                 // Process data
        st = 0;
        old_pos += command2_len - 2;
      } else if ((rx_buffer_L_len - (old_pos - 2) + pos) == command2_len) {      // "Overflow" buffer mode: check if data length equals expected length
        memcpy(ptr, &rx_buffer_L[old_pos], rx_buffer_L_len - old_pos);    // First copy data from the end of buffer
        if (pos > 0) {                                                    // Check and continue with beginning of buffer
          ptr += rx_buffer_L_len - old_pos;                               // Move to correct position in command_raw
          memcpy(ptr, &rx_buffer_L[0], pos);                              // Copy remaining data
        }
        usart2_process_command(&command2_raw, &command2, 2);                 // Process data
        st = 0;
        old_pos += command2_len - 2;
      }
    }
  }

  if (old_pos >= rx_buffer_L_len) {                                     // Check and manually update if we reached end of buffer
    old_pos = 0;
  }
  #endif // CONTROL_SERIAL_USART2
}

#if defined(_4x4_MASTER)
int usart2_process_command(SerialResp *command_in, SerialResp *command_out, uint8_t usart_idx)
{
  int ret = sizeof(SerialCommand);

  uint8_t checksum = 0;

/*
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
*/

  if (
    command_in->mag_02 != 0x02
    || command_in->mag_0e != 0x0e
  ) {
    ret = 1;
  } else {
    int i;
    for (i = 0; i < sizeof(SerialResp); i++)
      checksum ^= ((uint8_t*)command_in)[i];
    if (checksum == 0) {
      //*command_out = *command_in;

      k_brk = (((uint32_t)command_in->dk_00_4 << 8) | ((uint32_t)command_in->dk_00_5 & 0xff)) & 0xffff;
      if (k_brk > STOP_MIN_NUM_TO)
        k_brk = 51 * k_brk / 50;

      if (usart_idx == 2) {             // Sideboard USART2
  #ifdef CONTROL_SERIAL_USART2
        timeoutCntSerial_L  = 0;        // Reset timeout counter
        timeoutFlagSerial_L = 0;        // Clear timeout flag
  #endif
      }
    }
  }

  return ret;
}
#endif

#endif // #ifdef CONTROL_JX_168
