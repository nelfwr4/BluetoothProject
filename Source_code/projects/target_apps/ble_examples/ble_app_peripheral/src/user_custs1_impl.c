/**
 ****************************************************************************************
 *
 * @file user_custs1_impl.c
 *
 * @brief Peripheral project Custom1 Server implementation source code.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "gpio.h"
#include "app_api.h"
#include "app.h"
#include "prf_utils.h"
#include "custs1.h"
#include "custs1_task.h"
#include "user_custs1_def.h"
#include "user_custs1_impl.h"
#include "user_peripheral.h"
#include "user_periph_setup.h"
#include "adc.h"
#include "adc_531.h"
#include "arch_console.h"
#include "spi_flash.h"
#include "sw_aes.h"
#include "aes.h"
#include "wkupct_quadec.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

extern struct aes_env_tag aes_env;
extern uint8_t IV[ENC_DATA_LEN];

bool my_m1_state __SECTION_ZERO("retention_mem_area0");            // @RETENTION MEMORY
bool my_m2_state __SECTION_ZERO("retention_mem_area0");            // @RETENTION MEMORY
bool my_en_state __SECTION_ZERO("retention_mem_area0");            // @RETENTION MEMORY
uint16_t indication_counter __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
uint16_t non_db_val_counter __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
ke_msg_id_t led_timer __SECTION_ZERO("retention_mem_area0");       //@RETENTION MEMORY
ke_msg_id_t led_timer1 __SECTION_ZERO("retention_mem_area0");      //@RETENTION MEMORY
ke_msg_id_t led_timer2 __SECTION_ZERO("retention_mem_area0");      //@RETENTION MEMORY
ke_msg_id_t led_timer3 __SECTION_ZERO("retention_mem_area0");      //@RETENTION MEMORY
ke_msg_id_t led_timer4 __SECTION_ZERO("retention_mem_area0");      //@RETENTION MEMORY
ke_msg_id_t timer_used __SECTION_ZERO("retention_mem_area0");      //@RETENTION MEMORY
ke_msg_id_t handle_timer __SECTION_ZERO("retention_mem_area0");    //@RETENTION MEMORY
/* -------------------------------------------------------------------------- */
/*                Global variable Declarations and Definitions                */
/* -------------------------------------------------------------------------- */
uint8_t firmwareVersion = 0x01;
uint8_t max_Rec[2] =
    {
        0x00, // MSB
        0x32  // LSB
};
uint8_t key[ENC_DATA_LEN];                     // AES key
uint8_t aes_result[ENC_DATA_LEN];              // Store AES Decrypt result
uint16_t motorRunTime = 0x03E8;                // Motor Working time
uint16_t lockBackTime = 0x0032;                // Motor pause time
uint8_t keyString[8];                          // 16 Digit Hex number
uint8_t keyID[4];                              // 8 Digit Hex number
uint8_t blockdKeyID1[4];                       // storing block Key Id1
uint8_t blockdKeyID2[4];                       // storing block Key Id2
uint8_t start = 0x89;                          // Start byte of unlock frame
uint8_t end = 0x69;                            // End byte of unlock frame
uint8_t settingStart = 0x69;                   // SettingStart byte of settings frame
uint8_t settingsEnd = 0x89;                    // SettingsEnd byte of settings frame
uint8_t keyUserFlag = 0x55;                    // 0x55 -> Key User not blocked
uint8_t dateTime_flag = 0x55;                  // 0x55 -> DateTime flag valid
uint8_t keyType = 0x55;                        // 0x55 -> can override Deadbolt
uint8_t refNo[2];                              // 4 Digit Hex number
uint8_t ResultCode = 0x00;                     // Return Error Code/Success Code
extern bool my_button;                         // Storing Handle Flag data
bool handleFlag = 0;                           // Handle Flag
bool boltFlag = 0;                             // BoltFlag
uint16_t n = 0x00;                             // Increment Number each time when send function is called
uint16_t mvADC;                                // Battery voltage
uint8_t adcData[2];                            // Store Adc data
uint8_t X[50] __SECTION_ZERO("free_area");     // Using Mainly in Send function to store datas
uint8_t encX1[16];                             // Split and send AES Encrypted data
uint8_t encX2[16];                             // Split and send AES Encrypted data
uint8_t val_1[16] __SECTION_ZERO("free_area"); // Split and store AES Decrypted data
uint8_t val_2[16] __SECTION_ZERO("free_area"); // Split and store AES Decrypted data
uint8_t accessLogID[5];                        // 5 bytes of access log

uint8_t identifier[8] =
    {
        0x01, // unlock request frame
        0x02, // keyString request frame
        0x03, // lockBack request frame
        0x04, // Blocked key Id request frame
        0x05, // motorRunTime request frame
        0x06, // AES Key ID request frame
        0x07, // Access log request frame
        0x08, // Access log request frame
};

/* -------------------------------------------------------------------------- */
/*                             Function Definition                            */
/* -------------------------------------------------------------------------- */

void send(void)
{
  n++;
  if (n >= 0x32)
  {
    n = 0x01;
  }
  uint8_t tempN[2];
  tempN[0] = n >> 8;
  tempN[1] = n;
  GPIO_Disable_HW_Reset();
  uint32_t bytes_written;
  spi_flash_write_data(&tempN[0], 0x1F010, 2, &bytes_written);
  GPIO_Enable_HW_Reset();
  // Update boaltflag data
  if (GPIO_GetPinStatus(GPIO_BOLT_PORT, GPIO_BOLT_PIN) == 0)
  {
    // assume that deadbolt lock active means it will give zero
    boltFlag = 1;
  }
  else
  {
    boltFlag = 0;
  }
  // Response data
  X[0] = start;             // start
  X[1] = refNo[0];          // ref no1
  X[2] = refNo[1];          // ref no2
  X[3] = ResultCode;        // ResultCode
  X[4] = handleFlag;        // was handle moved?
  X[5] = boltFlag;          // boltFlag
  X[6] = adcData[0];        // voltage MSB
  X[7] = adcData[1];        // voltage LSB
  X[8] = lockBackTime >> 8; // lockbackTime MSB
  X[9] = lockBackTime;      // lockBackTime LSB
  X[10] = accessLogID[0];   // Access Log ID
  X[11] = accessLogID[1];   // Access Log ID
  X[12] = accessLogID[2];   // Access Log ID
  X[13] = accessLogID[3];   // Access Log ID
  X[14] = accessLogID[4];   // Access Log ID//Key ID
  X[15] = keyID[0];         // Key ID
  X[16] = keyID[1];         // Key ID
  X[17] = keyID[2];         // Key ID
  X[18] = keyID[3];         // Key ID
  X[19] = end;              // end
  X[20] = 0x00;             // Dummy
  X[21] = 0x00;             // Dummy
  X[22] = 0x00;             // Dummy
  X[23] = 0x00;             // Dummy
  X[24] = 0x00;             // Dummy
  X[25] = 0x00;             // Dummy
  X[26] = 0x00;             // Dummy
  X[27] = 0x00;             // Dummy
  X[28] = 0x00;             // Dummy
  X[29] = 0x00;             // Dummy
  X[30] = 0x00;             // Dummy
  X[31] = 0x00;             // Dummy

  /* --------------------------- Write to SPI Flash --------------------------- */
  GPIO_Disable_HW_Reset();
  // uint32_t bytes_written;
  spi_flash_write_data(&X[0], 0x1F035 + (0x14 * (n - 0x01) + 0x01) /* + (n - 1)*/, 20, &bytes_written);
  GPIO_Enable_HW_Reset();
  // split the X
  encX1[0] = X[0];
  encX1[1] = X[1];
  encX1[2] = X[2];
  encX1[3] = X[3];
  encX1[4] = X[4];
  encX1[5] = X[5];
  encX1[6] = X[6];
  encX1[7] = X[7];
  encX1[8] = X[8];
  encX1[9] = X[9];
  encX1[10] = X[10];
  encX1[11] = X[11];
  encX1[12] = X[12];
  encX1[13] = X[13];
  encX1[14] = X[14];
  encX1[15] = X[15];

  encX2[0] = X[16];
  encX2[1] = X[17];
  encX2[2] = X[18];
  encX2[3] = X[19];
  encX2[4] = X[20];
  encX2[5] = X[21];
  encX2[6] = X[22];
  encX2[7] = X[23];
  encX2[8] = X[24];
  encX2[9] = X[25];
  encX2[10] = X[26];
  encX2[11] = X[27];
  encX2[12] = X[28];
  encX2[13] = X[29];
  encX2[14] = X[30];
  encX2[15] = X[31];

  // AES encryption
  aes_operation(key, ENC_DATA_LEN, encX1, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();

  X[0] = aes_out[0];
  X[1] = aes_out[1];
  X[2] = aes_out[2];
  X[3] = aes_out[3];
  X[4] = aes_out[4];
  X[5] = aes_out[5];
  X[6] = aes_out[6];
  X[7] = aes_out[7];
  X[8] = aes_out[8];
  X[9] = aes_out[9];
  X[10] = aes_out[10];
  X[11] = aes_out[11];
  X[12] = aes_out[12];
  X[13] = aes_out[13];
  X[14] = aes_out[14];
  X[15] = aes_out[15];

  aes_operation(key, ENC_DATA_LEN, encX2, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();

  X[16] = aes_out[0];
  X[17] = aes_out[1];
  X[18] = aes_out[2];
  X[19] = aes_out[3];
  X[20] = aes_out[4];
  X[21] = aes_out[5];
  X[22] = aes_out[6];
  X[23] = aes_out[7];
  X[24] = aes_out[8];
  X[25] = aes_out[9];
  X[26] = aes_out[10];
  X[27] = aes_out[11];
  X[28] = aes_out[12];
  X[29] = aes_out[13];
  X[30] = aes_out[14];
  X[31] = aes_out[15];

  // AES encryption

  // Sent to app
  send_to_app();
  // reset handleFlag to zero
  handleFlag = 0x00;
  // reset boltFlag to zero
  boltFlag = 0x00;
  //	 GPIO_SetInactive(GPIO_EN_PORT, GPIO_EN);
  //	my_en_state = 0;
  //	my_m1_state = 0;
  //	my_m2_state = 0;
}
void stop(void)
{
  // Disabling EN pin to turn OFF motor driver and voltage divider
  GPIO_SetInactive(GPIO_EN_PORT, GPIO_EN);

  // declare a boolean to retained during sleep
  my_en_state = 0;
  my_m1_state = 0;
  my_m2_state = 0;

  // Stop motor
  GPIO_SetInactive(GPIO_M1_PORT, GPIO_M1_PIN);
  GPIO_SetInactive(GPIO_M2_PORT, GPIO_M2_PIN);

  // go to send function after 1Sec
  led_timer4 = app_easy_timer(100, send);

  //	 app_easy_timer_cancel(led_timer1);
  //	 app_easy_timer_cancel(led_timer2);
  //	 app_easy_timer_cancel(led_timer3);
}
void lock(void)
{
  // Enabling EN pin to turn ON motor driver and voltage divider
  GPIO_SetActive(GPIO_EN_PORT, GPIO_EN);
  /* Perform single ADC conversion */
  uint16_t result = gpadc_read();
  uint16_t mvADC = gpadc_sample_to_mv(result);
  // Right shifit 8 bits data to first index of array
  adcData[0] = (mvADC >> 8);
  adcData[1] = (mvADC >> 0);
  // declare a boolean to retained during sleep
  my_en_state = 1;
  my_m1_state = 0;
  my_m2_state = 1;

  // lock the door
  GPIO_SetInactive(GPIO_M1_PORT, GPIO_M1_PIN);
  GPIO_SetActive(GPIO_M2_PORT, GPIO_M2_PIN);

  // Go to stop function
  led_timer3 = app_easy_timer(motorRunTime, stop);
}
void stay(void)
{
  // Disabling EN pin to turn OFF motor driver and voltage divider
  GPIO_SetInactive(GPIO_EN_PORT, GPIO_EN);

  // declare a boolean to retained during sleep
  my_en_state = 0;
  my_m1_state = 0;
  my_m2_state = 0;

  // Time to enter the user
  GPIO_SetInactive(GPIO_M1_PORT, GPIO_M1_PIN);
  GPIO_SetInactive(GPIO_M2_PORT, GPIO_M2_PIN);

  // Go to lock function
  led_timer = app_easy_timer(lockBackTime, lock);
}
void unlock(void)
{
  // Enabling EN pin to turn ON motor driver and voltage divider
  GPIO_SetActive(GPIO_EN_PORT, GPIO_EN);

  // declare a boolean to retained during sleep
  my_en_state = 1;
  my_m1_state = 1;
  my_m2_state = 0;

  //	//
  //	adcData[0] = (mvADC>>8);
  //	adcData[1] = (mvADC>>0);

  // Unlock the door
  GPIO_SetActive(GPIO_M1_PORT, GPIO_M1_PIN);
  GPIO_SetInactive(GPIO_M2_PORT, GPIO_M2_PIN);

  // Go to stay function
  led_timer1 = app_easy_timer(motorRunTime, stay);
}
void handle_cb(void)
{
  handleFlag = 1;
  my_button = handleFlag;
  led_timer4 = app_easy_timer(100, send);
}

// while turning on the notification why the above function is working?
// Enable push button. Register callback function for button press event
void handle_button_enable(void)
{
  wkupct_register_callback(handle_cb);
  wkupct_enable_irq(WKUPCT_PIN_SELECT(GPIO_HANDLE_PORT, GPIO_HANDLE_PIN),                            // select pin
                    WKUPCT_PIN_POLARITY(GPIO_HANDLE_PORT, GPIO_HANDLE_PIN, WKUPCT_PIN_POLARITY_LOW), // polarity low
                    1,                                                                               // 1 event
                    10);                                                                             // debouncing time = 0
}

void user_svc1_led_wr_ind_handler(ke_msg_id_t const msgid,
                                  struct custs1_val_write_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
  /* -------------------------------------------------------------------------- */
  /*                        Reading all non voltile data                        */
  /* -------------------------------------------------------------------------- */
  GPIO_Disable_HW_Reset();
  uint32_t bytes_read1;
  spi_flash_read_data(&X[0], 0x1F012, 36, &bytes_read1);
  GPIO_Enable_HW_Reset();

  keyString[0] = X[0];
  keyString[1] = X[1];
  keyString[2] = X[2];
  keyString[3] = X[3];
  keyString[4] = X[4];
  keyString[5] = X[5];
  keyString[6] = X[6];
  keyString[7] = X[7];

  blockdKeyID1[0] = X[8];
  blockdKeyID1[1] = X[9];
  blockdKeyID1[2] = X[10];
  blockdKeyID1[3] = X[11];

  blockdKeyID2[0] = X[12];
  blockdKeyID2[1] = X[13];
  blockdKeyID2[2] = X[14];
  blockdKeyID2[3] = X[15];

  uint16_t temp_data;

  lockBackTime = X[17];
  temp_data = X[16];
  lockBackTime |= temp_data << 8;

  motorRunTime = X[19];
  temp_data = X[18];
  motorRunTime |= temp_data << 8;

  key[0] = X[20];
  key[1] = X[21];
  key[2] = X[22];
  key[3] = X[23];
  key[4] = X[24];
  key[5] = X[25];
  key[6] = X[26];
  key[7] = X[27];
  key[8] = X[28];
  key[9] = X[29];
  key[10] = X[30];
  key[11] = X[31];
  key[12] = X[32];
  key[13] = X[33];
  key[14] = X[34];
  key[15] = X[35];
  // Code is starting from here
  uint8_t val[50]; // MTU limit is set to 50 bytes
  memset(val, 0x00, 50);
  memcpy(&val, &param->value[0], param->length);

  val_1[0] = val[0];
  val_1[1] = val[1];
  val_1[2] = val[2];
  val_1[3] = val[3];
  val_1[4] = val[4];
  val_1[5] = val[5];
  val_1[6] = val[6];
  val_1[7] = val[7];
  val_1[8] = val[8];
  val_1[9] = val[9];
  val_1[10] = val[10];
  val_1[11] = val[11];
  val_1[12] = val[12];
  val_1[13] = val[13];
  val_1[14] = val[14];
  val_1[15] = val[15];

  val_2[0] = val[16];
  val_2[1] = val[17];
  val_2[2] = val[18];
  val_2[3] = val[19];
  val_2[4] = val[20];
  val_2[5] = val[21];
  val_2[6] = val[22];
  val_2[7] = val[23];
  val_2[8] = val[24];
  val_2[9] = val[25];
  val_2[10] = val[26];
  val_2[11] = val[27];
  val_2[12] = val[28];
  val_2[13] = val[29];
  val_2[14] = val[30];
  val_2[15] = val[31];

  // Copy the default init vec where ENC_DATA_LEN = 0x10 = 16 (DECIMAL)
  memcpy(aes_env.aes_key.iv, IV, ENC_DATA_LEN);

  rwip_schedule();
  // AES start ////
  //  Init TASK_AES
  aes_init(false, NULL);
  // start AES decryption1
  aes_operation(key, ENC_DATA_LEN, val_1, ENC_DATA_LEN, aes_result, ENC_DATA_LEN, AES_DECRYPT, NULL, 0); /* the last 0 means synchronous call */

  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();

  val_1[0] = aes_result[0];
  val_1[1] = aes_result[1];
  val_1[2] = aes_result[2];
  val_1[3] = aes_result[3];
  val_1[4] = aes_result[4];
  val_1[5] = aes_result[5];
  val_1[6] = aes_result[6];
  val_1[7] = aes_result[7];
  val_1[8] = aes_result[8];
  val_1[9] = aes_result[9];
  val_1[10] = aes_result[10];
  val_1[11] = aes_result[11];
  val_1[12] = aes_result[12];
  val_1[13] = aes_result[13];
  val_1[14] = aes_result[14];
  val_1[15] = aes_result[15];

  // start AES decryption 2
  aes_operation(key, ENC_DATA_LEN, val_2, ENC_DATA_LEN, aes_result, ENC_DATA_LEN, AES_DECRYPT, NULL, 0); /* the last 0 means synchronous call */

  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();
  rwip_schedule();

  val_2[0] = aes_result[0];
  val_2[1] = aes_result[1];
  val_2[2] = aes_result[2];
  val_2[3] = aes_result[3];
  val_2[4] = aes_result[4];
  val_2[5] = aes_result[5];
  val_2[6] = aes_result[6];
  val_2[7] = aes_result[7];
  val_2[8] = aes_result[8];
  val_2[9] = aes_result[9];
  val_2[10] = aes_result[10];
  val_2[11] = aes_result[11];
  val_2[12] = aes_result[12];
  val_2[13] = aes_result[13];
  val_2[14] = aes_result[14];
  val_2[15] = aes_result[15];

  /* --------------------------- AES Decryption end --------------------------- */

  /* -------------------------------------------------------------------------- */
  /*                            1.0 Unlock request frame                        */
  /* -------------------------------------------------------------------------- */
  if (identifier[0] == val_2[7])
  {
    if (start == val_1[0])
    {
      if (end == val_2[8])
      {
        // store the referance numbers
        refNo[0] = val_1[1];
        refNo[1] = val_1[2];
        // store the KeyId
        keyID[0] = val_1[4];
        keyID[1] = val_1[5];
        keyID[2] = val_1[6];
        keyID[3] = val_1[7];

        // store the accessLogID
        accessLogID[0] = val_2[2];
        accessLogID[1] = val_2[3];
        accessLogID[2] = val_2[4];
        accessLogID[3] = val_2[5];
        accessLogID[4] = val_2[6];

        if (dateTime_flag == val_1[8])
        {
          // dateflag passed
          if (keyString[0] == val_1[9] &&
              keyString[1] == val_1[10] &&
              keyString[2] == val_1[11] &&
              keyString[3] == val_1[12])
          {
            // keystring passed
            if (keyUserFlag == val_1[3])
            {
              // Keyuser flag passed
              // scan blocked key id 1
              GPIO_Disable_HW_Reset();
              uint32_t bytes_read;
              spi_flash_read_data(&blockdKeyID1[0], 0x1F01A, sizeof(blockdKeyID1), &bytes_read);
              GPIO_Enable_HW_Reset();
              if (blockdKeyID1[0] != keyID[0] &&
                  blockdKeyID1[1] != keyID[1] &&
                  blockdKeyID1[2] != keyID[2] &&
                  blockdKeyID1[3] != keyID[3])
              {
                // not blocked key 1
                // scan blocked key id 2
                GPIO_Disable_HW_Reset();
                uint32_t bytes_read;
                spi_flash_read_data(&blockdKeyID2[0], 0x1F01E, sizeof(blockdKeyID2), &bytes_read);
                GPIO_Enable_HW_Reset();
                if (blockdKeyID2[0] != keyID[0] &&
                    blockdKeyID2[1] != keyID[1] &&
                    blockdKeyID2[2] != keyID[2] &&
                    blockdKeyID2[3] != keyID[3])
                {
                  if (0x00 == val_2[1])
                  {
                    // store deadbolt value into deadbolt flag
                    boltFlag = 0x00;
                    // keytype = 00
                    // check dead bolt status
                    if (GPIO_GetPinStatus(GPIO_BOLT_PORT, GPIO_BOLT_PIN) == 0)
                    {
                      // assume that deadbolt lock active means it will give zero
                      boltFlag = 1;
                    }
                    else
                    {
                      boltFlag = 0;
                    }
                    if (boltFlag == 0) //
                    {
                      // dead bolt is not locked
                      // door unlocked
                      // Door unlocking
                      uint8_t data[2];
                      uint16_t temp_data;
                      GPIO_Disable_HW_Reset();
                      // Reading the delay value
                      uint32_t bytes_read;
                      spi_flash_read_data(&data[0], 0x1F022, sizeof(lockBackTime), &bytes_read);
                      lockBackTime = data[1];
                      temp_data = data[0];
                      lockBackTime |= temp_data << 8;
                      GPIO_Enable_HW_Reset();
                      // unlock
                      ResultCode = 0xA0;
                      led_timer = app_easy_timer(5, unlock);
                    }
                    else
                    {
                      // bolt lock is acctive, lock the door
                      // Stop motor
                      // GPIO_SetInactive(GPIO_M1_PORT, GPIO_M1_PIN);
                      // GPIO_SetInactive(GPIO_M2_PORT, GPIO_M2_PIN);

                      ResultCode = 0xF5;
                    }
                  }
                  else if (keyType == val_2[1])
                  {
                    if (GPIO_GetPinStatus(GPIO_BOLT_PORT, GPIO_BOLT_PIN) == 0)
                    {
                      // assume that deadbolt lock active means it will give zero
                      boltFlag = 1;
                    }
                    else
                    {
                      boltFlag = 0;
                    }
                    // keytype = 55
                    // emergency key applied, unlock the door
                    // Door unlocking
                    uint8_t data[2];
                    uint16_t temp_data;
                    GPIO_Disable_HW_Reset();
                    // Reading the delay value
                    uint32_t bytes_read;
                    spi_flash_read_data(&data[0], 0x1F022, sizeof(lockBackTime), &bytes_read);
                    lockBackTime = data[1];
                    temp_data = data[0];
                    lockBackTime |= temp_data << 8;

                    spi_flash_read_data(&data[0], 0x1F024, sizeof(motorRunTime), &bytes_read);
                    // read motor run time MSB from data[1]
                    motorRunTime = data[1];
                    temp_data = data[0];
                    // combine motor run time MSB with LSB
                    motorRunTime |= temp_data << 8;
                    GPIO_Enable_HW_Reset();
                    ResultCode = 0xA1;
                    led_timer = app_easy_timer(5, unlock);
                  }
                }
                else
                {
                  // blocked key id 2 is  not matching
                  ResultCode = 0xF4;
                }
              }
              else
              {
                // blocked key id 1 is  not matching
                ResultCode = 0xF3;
              }
            }
            else
            {
              // key user flag is  not matching
              ResultCode = 0xF2;
            }
          }
          else
          {
            // key string is  not matching
            ResultCode = 0xF1;
          }
        }
        else
        {
          // dateTime_flag is not matching
          ResultCode = 0xF0;
        }
        send();
      }
    }
  }

  /* ----------------------------- Settings frame ----------------------------- */

  /* -------------------------------------------------------------------------- */
  /*                            2.0 key change frame                            */
  /* -------------------------------------------------------------------------- */
  if (identifier[1] == val_2[3])
  {
    if (settingStart == val_1[0])
    {
      if (settingsEnd == val_2[4])
      {
        refNo[0] = val_1[1];
        refNo[1] = val_1[2];

        if (keyString[0] == val_1[3] &&
            keyString[1] == val_1[4] &&
            keyString[2] == val_1[5] &&
            keyString[3] == val_1[6] &&
            keyString[4] == val_1[7] &&
            keyString[5] == val_1[8] &&
            keyString[6] == val_1[9] &&
            keyString[7] == val_1[10])
        {

          /// Enter the new key
          keyString[0] = val_1[11]; // New Key1
          keyString[1] = val_1[12]; // New Key2
          keyString[2] = val_1[13]; // New Key3
          keyString[3] = val_1[14]; // New Key4
          keyString[4] = val_1[15]; // New Key5
          keyString[5] = val_2[0];  // New Key6
          keyString[6] = val_2[1];  // New Key7
          keyString[7] = val_2[2];  // New Key8
          /// writing the new key to spi flash
          GPIO_Disable_HW_Reset();
          // spi_flash_block_erase(0x7000, SPI_FLASH_OP_SE);
          uint32_t bytes_written;
          // uint8_t data[8] = { 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00 };
          spi_flash_write_data(&keyString[0], 0x1F012, sizeof(keyString), &bytes_written);
          GPIO_Enable_HW_Reset();
          /// Response

          X[0] = val_1[0];      // start
          X[1] = refNo[0];      // ref no1
          X[2] = val_1[2];      // ref no2
          X[3] = keyString[0];  // New key1
          X[4] = keyString[1];  // New key2
          X[5] = keyString[2];  // New Key3
          X[6] = keyString[3];  // New key4
          X[7] = keyString[4];  // New key5
          X[8] = keyString[5];  // New key6
          X[9] = keyString[6];  // New key7
          X[10] = keyString[7]; // New key8
          X[11] = val_2[4];     // end
          X[12] = 0x00;
          X[13] = 0x00;
          X[14] = 0x00;
          X[15] = 0x00;

          aes_operation(key, ENC_DATA_LEN, X, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();

          X[0] = aes_out[0];
          X[1] = aes_out[1];
          X[2] = aes_out[2];
          X[3] = aes_out[3];
          X[4] = aes_out[4];
          X[5] = aes_out[5];
          X[6] = aes_out[6];
          X[7] = aes_out[7];
          X[8] = aes_out[8];
          X[9] = aes_out[9];
          X[10] = aes_out[10];
          X[11] = aes_out[11];
          X[12] = aes_out[12];
          X[13] = aes_out[13];
          X[14] = aes_out[14];
          X[15] = aes_out[15];
        }
        send_to_app(); // send
      }
    }
  }

  /* -------------------------------------------------------------------------- */
  /*                              3.0 lockback time                             */
  /* -------------------------------------------------------------------------- */
  if (identifier[2] == val_1[13])
  {
    if (settingStart == val_1[0])
    {
      if (settingsEnd == val_1[14])
      {
        refNo[0] = val_1[1];
        refNo[1] = val_1[2];

        if (
            keyString[0] == val_1[3] &&
            keyString[1] == val_1[4] &&
            keyString[2] == val_1[5] &&
            keyString[3] == val_1[6] &&
            keyString[4] == val_1[7] &&
            keyString[5] == val_1[8] &&
            keyString[6] == val_1[9] &&
            keyString[7] == val_1[10])
        {

          /// change lock back time
          // Probably We need a typecasting here
          // lockBackTime = 10 * val[7];
          uint8_t lockBack_time1 = val_1[11];
          uint8_t lockBack_time2 = val_1[12];
          // add MSB to lockBackTime
          lockBackTime = lockBack_time2;
          // add LSB to lockBackTime
          lockBackTime |= lockBack_time1 << 8;
          uint32_t bytes_written;
          uint8_t data[2];

          data[0] = (lockBackTime >> 8);
          data[1] = (lockBackTime >> 0);

          GPIO_Disable_HW_Reset();
          //  uint32_t bytes_read;
          spi_flash_write_data(&data[0], 0x1F022, sizeof(lockBackTime), &bytes_written);
          GPIO_Enable_HW_Reset();
          /// Response
          X[0] = val_1[0];  // start
          X[1] = refNo[0];  // ref no1
          X[2] = refNo[1];  // ref no2
          X[3] = data[0];   // New delay
          X[4] = data[1];   // New delay
          X[5] = val_1[14]; // end
          X[6] = val_1[0];  // start;
          X[7] = blockdKeyID1[0];
          X[8] = blockdKeyID1[1];
          X[9] = blockdKeyID1[2];
          X[10] = blockdKeyID1[3];
          X[11] = blockdKeyID2[0];
          X[12] = blockdKeyID2[1];
          X[13] = blockdKeyID2[2];
          X[14] = blockdKeyID2[3];
          X[15] = val_1[14]; // end

          aes_operation(key, ENC_DATA_LEN, X, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();

          X[0] = aes_out[0];
          X[1] = aes_out[1];
          X[2] = aes_out[2];
          X[3] = aes_out[3];
          X[4] = aes_out[4];
          X[5] = aes_out[5];
          X[6] = aes_out[6];
          X[7] = aes_out[7];
          X[8] = aes_out[8];
          X[9] = aes_out[9];
          X[10] = aes_out[10];
          X[11] = aes_out[11];
          X[12] = aes_out[12];
          X[13] = aes_out[13];
          X[14] = aes_out[14];
          X[15] = aes_out[15];
        }
        send_to_app(); // send
      }
    }
  }

  /* -------------------------------------------------------------------------- */
  /*                           4.0 Blocked key Id set                           */
  /* -------------------------------------------------------------------------- */
  if (identifier[3] == val_2[3])
  {
    if (settingStart == val_1[0])
    {
      if (settingsEnd == val_2[4])
      {
        refNo[0] = val_1[1];
        refNo[1] = val_1[2];

        if (
            keyString[0] == val_1[3] &&
            keyString[1] == val_1[4] &&
            keyString[2] == val_1[5] &&
            keyString[3] == val_1[6] &&
            keyString[4] == val_1[7] &&
            keyString[5] == val_1[8] &&
            keyString[6] == val_1[9] &&
            keyString[7] == val_1[10])
        {

          /// change lock back time
          blockdKeyID1[0] = val_1[11];
          blockdKeyID1[1] = val_1[12];
          blockdKeyID1[2] = val_1[13];
          blockdKeyID1[3] = val_1[14];

          blockdKeyID2[0] = val_1[15];
          blockdKeyID2[1] = val_2[00];
          blockdKeyID2[2] = val_2[01];
          blockdKeyID2[3] = val_2[02];

          /// writing the new BKID1 to spi flash
          GPIO_Disable_HW_Reset();
          uint32_t bytes_written3;
          spi_flash_write_data(&blockdKeyID1[0], 0x1F01A, sizeof(blockdKeyID1), &bytes_written3);
          GPIO_Enable_HW_Reset();

          /// writing the new BKID2 to spi flash
          GPIO_Disable_HW_Reset();
          uint32_t bytes_written4;
          spi_flash_write_data(&blockdKeyID2[0], 0x1F01E, sizeof(blockdKeyID2), &bytes_written4);
          GPIO_Enable_HW_Reset();

          /// Response

          X[0] = val_1[0];         // start
          X[1] = refNo[0];         // ref no1
          X[2] = refNo[1];         // ref no2
          X[3] = blockdKeyID1[0];  // New BKID1
          X[4] = blockdKeyID1[1];  // New BKID1
          X[5] = blockdKeyID1[2];  // New BKID1
          X[6] = blockdKeyID1[3];  // New BKID1
          X[7] = blockdKeyID2[0];  // New BKID2
          X[8] = blockdKeyID2[1];  // New BKID2
          X[9] = blockdKeyID2[2];  // New BKID2
          X[10] = blockdKeyID2[3]; // New BKID2
          X[11] = val_2[4];        // end
          X[12] = 0x00;
          X[13] = 0x00;
          X[14] = 0x00;
          X[15] = 0x00;

          aes_operation(key, ENC_DATA_LEN, X, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();

          X[0] = aes_out[0];
          X[1] = aes_out[1];
          X[2] = aes_out[2];
          X[3] = aes_out[3];
          X[4] = aes_out[4];
          X[5] = aes_out[5];
          X[6] = aes_out[6];
          X[7] = aes_out[7];
          X[8] = aes_out[8];
          X[9] = aes_out[9];
          X[10] = aes_out[10];
          X[11] = aes_out[11];
          X[12] = aes_out[12];
          X[13] = aes_out[13];
          X[14] = aes_out[14];
          X[15] = aes_out[15];
        }
        send_to_app(); // send
      }
    }
  }

  /* -------------------------------------------------------------------------- */
  /*                             5.0 Motor run Time                             */
  /* -------------------------------------------------------------------------- */
  if (identifier[4] == val_1[13])
  {
    if (settingStart == val_1[0])
    {
      if (settingsEnd == val_1[14])
      {
        refNo[0] = val_1[1];
        refNo[1] = val_1[2];
        if (
            keyString[0] == val_1[3] &&
            keyString[1] == val_1[4] &&
            keyString[2] == val_1[5] &&
            keyString[3] == val_1[6] &&
            keyString[4] == val_1[7] &&
            keyString[5] == val_1[8] &&
            keyString[6] == val_1[9] &&
            keyString[7] == val_1[10])
        {
          uint16_t temp_data = 0x00;
          motorRunTime = val_1[12];
          temp_data = val_1[11];
          motorRunTime |= temp_data << 8;

          uint32_t bytes_written;
          uint8_t data[2];
          data[0] = (motorRunTime >> 8);
          data[1] = (motorRunTime >> 0);

          GPIO_Disable_HW_Reset();
          //  uint32_t bytes_read;
          spi_flash_write_data(&data[0], 0x1F024, sizeof(motorRunTime), &bytes_written);
          GPIO_Enable_HW_Reset();

          X[0] = val_1[0];  // start
          X[1] = refNo[0];  // ref no1
          X[2] = refNo[1];  // ref no2
          X[3] = data[0];   // Motor runtime MSB
          X[4] = data[1];   // Motor runtime LSB
          X[5] = val_1[14]; // end
          X[6] = 0x00;
          X[7] = 0x00;
          X[8] = 0x00;
          X[9] = 0x00;
          X[10] = 0x00;
          X[11] = 0x00;
          X[12] = 0x00;
          X[13] = 0x00;
          X[14] = 0x00;
          X[15] = 0x00;

          aes_operation(key, ENC_DATA_LEN, X, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();

          X[0] = aes_out[0];
          X[1] = aes_out[1];
          X[2] = aes_out[2];
          X[3] = aes_out[3];
          X[4] = aes_out[4];
          X[5] = aes_out[5];
          X[6] = aes_out[6];
          X[7] = aes_out[7];
          X[8] = aes_out[8];
          X[9] = aes_out[9];
          X[10] = aes_out[10];
          X[11] = aes_out[11];
          X[12] = aes_out[12];
          X[13] = aes_out[13];
          X[14] = aes_out[14];
          X[15] = aes_out[15];
        }
        send_to_app(); // send
      }
    }
  }

  /* -------------------------------------------------------------------------- */
  /*                             6.0 AES Key change                             */
  /* -------------------------------------------------------------------------- */
  if (identifier[5] == val_2[11])
  {
    if (settingStart == val_1[0])
    {
      if (settingsEnd == val_2[12])
      {
        refNo[0] = val_1[1];
        refNo[1] = val_1[2];

        if (keyString[0] == val_1[3] &&
            keyString[1] == val_1[4] &&
            keyString[2] == val_1[5] &&
            keyString[3] == val_1[6] &&
            keyString[4] == val_1[7] &&
            keyString[5] == val_1[8] &&
            keyString[6] == val_1[9] &&
            keyString[7] == val_1[10])
        {

          key[0] = val_1[11];
          key[1] = val_1[12];
          key[2] = val_1[13];
          key[3] = val_1[14];
          key[4] = val_1[15];
          key[5] = val_2[0];
          key[6] = val_2[1];
          key[7] = val_2[2];
          key[8] = val_2[3];
          key[9] = val_2[4];
          key[10] = val_2[5];
          key[11] = val_2[6];
          key[12] = val_2[7];
          key[13] = val_2[8];
          key[14] = val_2[9];
          key[15] = val_2[10];

          /// writing the new key to spi flash
          GPIO_Disable_HW_Reset();
          // spi_flash_block_erase(0x7000, SPI_FLASH_OP_SE);
          uint32_t bytes_written;
          // uint8_t data[8] = { 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00 };
          spi_flash_write_data(&key[0], 0x1F026, sizeof(key), &bytes_written);
          GPIO_Enable_HW_Reset();
          /// Response

          encX1[0] = val_1[0]; // start
          encX1[1] = val_1[1]; // ref no1
          encX1[2] = val_1[2]; // ref no2
          encX1[3] = key[0];   // New key1
          encX1[4] = key[1];   // New key2
          encX1[5] = key[2];   // New Key3
          encX1[6] = key[3];   // New key4
          encX1[7] = key[4];   // New key5
          encX1[8] = key[5];   // New key6
          encX1[9] = key[6];   // New key7
          encX1[10] = key[7];  // New key8
          encX1[11] = key[8];  // New key9
          encX1[12] = key[9];  // New key10
          encX1[13] = key[10]; // New key11
          encX1[14] = key[11]; // New key12
          encX1[15] = key[12]; // New key13
          encX2[0] = key[13];  // New key14
          encX2[1] = key[14];  // New key15
          encX2[2] = key[15];  // New key16
          encX2[3] = val_2[3]; // end

          // first encryption
          aes_operation(key, ENC_DATA_LEN, encX1, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();

          X[0] = aes_out[0];
          X[1] = aes_out[1];
          X[2] = aes_out[2];
          X[3] = aes_out[3];
          X[4] = aes_out[4];
          X[5] = aes_out[5];
          X[6] = aes_out[6];
          X[7] = aes_out[7];
          X[8] = aes_out[8];
          X[9] = aes_out[9];
          X[10] = aes_out[10];
          X[11] = aes_out[11];
          X[12] = aes_out[12];
          X[13] = aes_out[13];
          X[14] = aes_out[14];
          X[15] = aes_out[15];
          // second encryption
          aes_operation(key, ENC_DATA_LEN, encX2, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();

          X[16] = aes_out[0];
          X[17] = aes_out[1];
          X[18] = aes_out[2];
          X[19] = aes_out[3];
          X[20] = aes_out[4];
          X[21] = aes_out[5];
          X[22] = aes_out[6];
          X[23] = aes_out[7];
          X[24] = aes_out[8];
          X[25] = aes_out[9];
          X[26] = aes_out[10];
          X[27] = aes_out[11];
          X[28] = aes_out[12];
          X[29] = aes_out[13];
          X[30] = aes_out[14];
          X[31] = aes_out[15];
        }
        send_to_app(); // send
      }
    }
  }

  /* -------------------------------------------------------------------------- */
  /*                        7.0 Access log request frame                        */
  /* -------------------------------------------------------------------------- */
  if (identifier[6] == val_1[11])
  {
    if (settingStart == val_1[0])
    {
      if (settingsEnd == val_1[12])
      {
        refNo[0] = val_1[1];
        refNo[1] = val_1[2];

        if (keyString[0] == val_1[3] &&
            keyString[1] == val_1[4] &&
            keyString[2] == val_1[5] &&
            keyString[3] == val_1[6] &&
            keyString[4] == val_1[7] &&
            keyString[5] == val_1[8] &&
            keyString[6] == val_1[9] &&
            keyString[7] == val_1[10])
        {
          uint8_t tempN[2];
          uint16_t temp_data = 0x00;
          // Read latest LogRequest number n
          GPIO_Disable_HW_Reset();
          uint32_t bytes_read;
          spi_flash_read_data(&tempN[0], 0x1F010, 2, &bytes_read);
          GPIO_Enable_HW_Reset();
          n = tempN[1];
          temp_data = tempN[0];
          n |= temp_data << 8;

          X[0] = val_1[0];        // start
          X[1] = refNo[0];        // ref No 1
          X[2] = refNo[1];        // ref No 2
          X[3] = n >> 8;          // ACK for logRequest as lateast log number MSB
          X[4] = n;               // ACK for log LSB
          X[5] = firmwareVersion; //
          X[6] = max_Rec[0];      //
          X[7] = max_Rec[1];
          X[8] = val_1[12]; // end

          aes_operation(key, ENC_DATA_LEN, X, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();
          rwip_schedule();

          X[0] = aes_out[0];
          X[1] = aes_out[1];
          X[2] = aes_out[2];
          X[3] = aes_out[3];
          X[4] = aes_out[4];
          X[5] = aes_out[5];
          X[6] = aes_out[6];
          X[7] = aes_out[7];
          X[8] = aes_out[8];
          X[9] = aes_out[9];
          X[10] = aes_out[10];
          X[11] = aes_out[11];
          X[12] = aes_out[12];
          X[13] = aes_out[13];
          X[14] = aes_out[14];
          X[15] = aes_out[15];
          send_to_app(); // send
        }
      }
    }
  }
  /* ----------------- continuation of the access log request ----------------- */
  /* -------------------------------------------------------------------------- */
  /*                             Access log request                             */
  /* -------------------------------------------------------------------------- */
  if (identifier[7] == val_1[5])
  {
    if (settingStart == val_1[0])
    {
      if (settingsEnd == val_1[6])
      {
        uint16_t address;
        uint16_t temp_data;
        uint8_t flashVal[20];
        address = val_1[4];
        temp_data = val_1[3];
        address |= temp_data << 8;
        GPIO_Disable_HW_Reset();
        uint32_t bytes_read;
        spi_flash_read_data(&flashVal[0], 0x1F035 + (0x14 * (address - 0x01) + 0x01), sizeof(flashVal), &bytes_read);
        GPIO_Enable_HW_Reset();

        encX1[0] = flashVal[0];   // Start
        encX1[1] = flashVal[1];   // RefNo1
        encX1[2] = flashVal[2];   // RefNo2
        encX1[3] = flashVal[3];   // ResultCode
        encX1[4] = flashVal[4];   // Handle Flag
        encX1[5] = flashVal[5];   // BoltFlag
        encX1[6] = flashVal[6];   // Battery level
        encX1[7] = flashVal[7];   // Battery Level
        encX1[8] = flashVal[8];   // Lockback time
        encX1[9] = flashVal[9];   // Lockback time
        encX1[10] = flashVal[10]; // Access LogID
        encX1[11] = flashVal[11]; // Access LogID
        encX1[12] = flashVal[12]; // Access LogID
        encX1[13] = flashVal[13]; // Access LogID
        encX1[14] = flashVal[14]; // Access LogID
        encX1[15] = flashVal[15]; // Key ID

        encX2[0] = flashVal[16]; // Key ID
        encX2[1] = flashVal[17]; // Key ID
        encX2[2] = flashVal[18]; // Key ID
        encX2[3] = flashVal[19]; // End
        encX2[4] = 0x00;
        encX2[5] = 0x00;
        encX2[6] = 0x00;
        encX2[7] = 0x00;
        encX2[8] = 0x00;
        encX2[9] = 0x00;
        encX2[10] = 0x00;
        encX2[11] = 0x00;
        encX2[12] = 0x00;
        encX2[13] = 0x00;
        encX2[14] = 0x00;
        encX2[15] = 0x00;
        aes_operation(key, ENC_DATA_LEN, encX1, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

        rwip_schedule();
        // rwip_schedule();
        // rwip_schedule();
        // rwip_schedule();
        // rwip_schedule();
        // rwip_schedule();

        X[0] = aes_out[0];
        X[1] = aes_out[1];
        X[2] = aes_out[2];
        X[3] = aes_out[3];
        X[4] = aes_out[4];
        X[5] = aes_out[5];
        X[6] = aes_out[6];
        X[7] = aes_out[7];
        X[8] = aes_out[8];
        X[9] = aes_out[9];
        X[10] = aes_out[10];
        X[11] = aes_out[11];
        X[12] = aes_out[12];
        X[13] = aes_out[13];
        X[14] = aes_out[14];
        X[15] = aes_out[15];

        aes_operation(key, ENC_DATA_LEN, encX2, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

        rwip_schedule();
        // rwip_schedule();
        // rwip_schedule();
        // rwip_schedule();
        // rwip_schedule();
        // rwip_schedule();

        X[16] = aes_out[0];
        X[17] = aes_out[1];
        X[18] = aes_out[2];
        X[19] = aes_out[3];
        X[20] = aes_out[4];
        X[21] = aes_out[5];
        X[22] = aes_out[6];
        X[23] = aes_out[7];
        X[24] = aes_out[8];
        X[25] = aes_out[9];
        X[26] = aes_out[10];
        X[27] = aes_out[11];
        X[28] = aes_out[12];
        X[29] = aes_out[13];
        X[30] = aes_out[14];
        X[31] = aes_out[15];
        send_to_app(); // send
      }
    }
  }

  /* -------------------------------------------------------------------------- */
  /*                            8.0 Master rest frame                           */
  /* -------------------------------------------------------------------------- */
  if (val_1[0] == 0xce &&
      val_1[1] == 0x05 &&
      val_1[2] == 0x62 &&
      val_1[3] == 0x89 &&
      val_1[4] == 0x68 &&
      val_1[5] == 0xda /*&&
      val_1[6] == 0xaf &&
      val_1[7] == 0x90 &&
      val_1[8] == 0xec &&
      val_1[9] == 0xb5 &&
      val_1[10] == 0x35 &&
      val_1[11] == 0xb0 &&
      val_1[12] == 0x60 &&
      val_1[13] == 0xd7*/
  )
  {

    /// writing the default key string,BKID1,BKID2,Lock back time, motor run time and AES key to spi flash
    uint32_t bytes_written;
    uint8_t reset_data[36] =
        {
            0x4d, // keystring
            0x7d, // keystring
            0x23, // keystring
            0x17, // keystring
            0x24, // keystring
            0xFD, // keystring
            0xE3, // keystring
            0xA3, // keystring

            0xFF, // Blocked key Id 1
            0xFF, // Blocked key Id 1
            0xFF, // Blocked key Id 1
            0xFF, // Blocked key Id 1

            0xFF, // Blocked key id 2
            0xFF, // Blocked key id 2
            0xFF, // Blocked key id 2
            0xFF, // Blocked key id 2

            0x03, // lock back time
            0xE8, // lock back time

            0x00, // motor run time
            0x32, // motor run time

            0x23, // AES key
            0xAE, // AES key
            0x32, // AES key
            0xFA, // AES key
            0x73, // AES key
            0x96, // AES key
            0x3F, // AES key
            0x9D, // AES key
            0xA3, // AES key
            0x49, // AES key
            0x82, // AES key
            0x7D, // AES key
            0x20, // AES key
            0x4C, // AES key
            0xB4, // AES key
            0x32  // AES key
        };
    GPIO_Disable_HW_Reset();
    spi_flash_write_data(&reset_data[0], 0x1F012, sizeof(reset_data), &bytes_written);
    GPIO_Enable_HW_Reset();
    uint8_t success[5] = {0xce, val_1[1], val_1[2], 0x1, 0x3e};
    // AES encryption 1
    aes_operation(key, ENC_DATA_LEN, success, ENC_DATA_LEN, aes_out, ENC_DATA_LEN, AES_ENCRYPT, NULL, 0);

    rwip_schedule();
    rwip_schedule();
    rwip_schedule();
    rwip_schedule();
    rwip_schedule();
    rwip_schedule();

    X[0] = aes_out[0]; // start
    X[1] = aes_out[1]; // refNo1
    X[2] = aes_out[2]; // refNo2
    X[3] = aes_out[3]; // success
    X[4] = aes_out[4]; // end

    send_to_app();
  }

} // end of the all frames
void send_to_app(void)
{
  struct custs1_val_ntf_ind_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_NTF_REQ,
                                                        prf_get_task_from_id(TASK_ID_CUSTS1),
                                                        TASK_APP,
                                                        custs1_val_ntf_ind_req,
                                                        DEF_SVC1_ADC_VAL_1_CHAR_LEN);

  req->handle = SVC1_IDX_ADC_VAL_1_VAL;
  req->length = DEF_SVC1_ADC_VAL_1_CHAR_LEN;
  req->notification = true;
  memcpy(req->value, X, DEF_SVC1_ADC_VAL_1_CHAR_LEN);

  ke_msg_send(req);
  handle_button_enable(); // Enable callback again.
}
uint16_t gpadc_read(void)
{
  /* Initialize the ADC */
  adc_config_t adc_cfg = {
      .input_mode = ADC_INPUT_MODE_SINGLE_ENDED,
      .input = ADC_INPUT_SE_P0_7,
      .smpl_time_mult = 2,
      .continuous = false,
      .interval_mult = 0,
      .input_attenuator = ADC_INPUT_ATTN_4X,
      .chopping = false,
      .oversampling = 0,
  };
  adc_init(&adc_cfg);

  /* Perform offset calibration of the ADC */
  adc_offset_calibrate(ADC_INPUT_MODE_SINGLE_ENDED);

  adc_start();
  uint16_t result = adc_correct_sample(adc_get_sample());
  adc_disable();

  return (result);
}
uint16_t gpadc_sample_to_mv(uint16_t sample)
{
  /* Resolution of ADC sample depends on oversampling rate */
  uint32_t adc_res = 10 + ((6 < adc_get_oversampling()) ? 6 : adc_get_oversampling());

  /* Reference voltage is 900mv but scale based in input attenation */
  uint32_t ref_mv = 900 * (GetBits16(GP_ADC_CTRL2_REG, GP_ADC_ATTN) + 1);

  return (uint16_t)((((uint32_t)sample) * ref_mv) >> adc_res);
}

// ADC Handler
void user_svc1_adc_val_1_cfg_ind_handler(ke_msg_id_t const msgid,
                                         struct custs1_val_write_ind const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{

  /* Perform single ADC conversion */
  uint16_t result = gpadc_read();
  mvADC = gpadc_sample_to_mv(result);
  if (ke_state_get(TASK_APP) == APP_CONNECTED)
  {
    // GPIO_SetActive(GPIO_EN_PORT, GPIO_EN);
    // my_en_state = 1;
    // GPIO_SetInactive(GPIO_EN_PORT, GPIO_EN);
    // my_en_state = 0;
    //  Set it once again until Stop command is received in Control Characteristic
    timer_used = app_easy_timer(100, user_svc1_adc_val_1_cfg_ind_handler);
  }
  else
  {
    GPIO_SetInactive(GPIO_EN_PORT, GPIO_EN);
    my_en_state = 0;
  }
}
// handle function
// void handle_fn(void) {
//   handleFlag = 1;
//   my_button = handleFlag;
//   led_timer4 = app_easy_timer(100, send);
// }
