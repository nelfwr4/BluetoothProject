
#include "user_custs1_impl.h"
#include "unlock_frame.h"
#include "user_periph_setup.h"

extern uint8_t  refNo[2];
extern uint8_t start;
extern uint8_t end;
extern uint8_t unlockF1;
extern uint8_t unlockF2;
extern uint8_t keyString[4];
extern uint8_t dateTime_flag;
extern uint8_t keyFlag;
extern uint8_t blockdKeyID1[3];
extern uint8_t blockdKeyID2[3];
extern uint8_t keyID[3];
extern uint8_t keyType;
extern uint8_t lockBackTime;

void unlock_frame(uint8_t val_0,val_1,val_2,val_3,val_4,val_5,val_6,val_7,
									val_8,val_9,val_10,val_11,val_12,val_13,val_14,val_15){
//Unlock door	
	if(start == val_0 && 0 == val_13 && 0 == val_14 && end == val_15){
		// store the referance numbers
		refNo[0] = val_1;
		refNo[1] = val_2;
		if(refNo[0] == unlockF1 && refNo[1] == unlockF2){
		
		if(dateTime_flag == val[7]){
			//dateflag passed
				if (keyString[0] == val[8] &&  
						keyString[1] == val[9] && 
						keyString[2] == val[10] && 
						keyString[3] == val[11]  ){
					//keystring passed
								if(keyFlag == val[3]){
									//Keyuser flag passed
									
									//scan blocked key id 1
											GPIO_Disable_HW_Reset();
											uint32_t bytes_read;
											spi_flash_read_data(&blockdKeyID1[0], 0x7004, sizeof(blockdKeyID1), &bytes_read);
											GPIO_Enable_HW_Reset();
									if(blockdKeyID1[0] != keyID[0] &&
										 blockdKeyID1[1] != keyID[1] &&
										 blockdKeyID1[2] != keyID[2]){
											 //not blocked key 1 
											 //scan blocked key id 2
											GPIO_Disable_HW_Reset();
											uint32_t bytes_read;
											spi_flash_read_data(&blockdKeyID2[0], 0x7007, sizeof(blockdKeyID2), &bytes_read);
											GPIO_Enable_HW_Reset();
											 if(blockdKeyID2[0] != keyID[0] &&
												  blockdKeyID2[1] != keyID[1] &&
												  blockdKeyID2[2] != keyID[2]){
											 if(keyType == val[12]){
												 //keytype = 0
												 //check dead bolt status 
												 	if(GPIO_GetPinStatus(GPIO_BOLT_PORT, GPIO_BOLT_PIN) == 0){
														//assume that deadbolt lock active means it will give zero
														boltFlag = 1;
													}
													else{
														boltFlag = 0;
													}
												 if(boltFlag ==0)//
													 {
													 // dead bolt is not locked 
													 //door unlocked 
													 //Door unlocking 
											uint8_t data[2];	
                      uint16_t temp_data;											 
											GPIO_Disable_HW_Reset();
														 // Reading the delay value
											uint32_t bytes_read;
											spi_flash_read_data(&data[0], 0x7010, sizeof(lockBackTime), &bytes_read);
														 lockBackTime=data[1];
														 temp_data=data[0];
														 lockBackTime|=temp_data<<8;
											GPIO_Enable_HW_Reset();
														 intFlag =0;
//unlock
													led_timer=app_easy_timer(5,unlock);
													 
														
	 
												 }
													 else{
														 //bolt lock is acctive, lock the door 
														 //Stop motor
													GPIO_SetInactive(GPIO_M1_PORT, GPIO_M1_PIN);
													GPIO_SetInactive(GPIO_M2_PORT, GPIO_M2_PIN);
													 }
												 
											 }
											 else{
											//emergency key applied, unlock the door
											//Door unlocking 
											uint8_t data[2];	
                      uint16_t temp_data;											 
											GPIO_Disable_HW_Reset();
											// Reading the delay value
											uint32_t bytes_read;
											spi_flash_read_data(&data[0], 0x7010, sizeof(lockBackTime), &bytes_read);
											lockBackTime=data[1];
											temp_data=data[0];
											lockBackTime|=temp_data<<8;
											GPIO_Enable_HW_Reset();

													led_timer=app_easy_timer(5,unlock);
													 //
													 /* Perform single ADC conversion */
													uint16_t result = gpadc_read();
													uint16_t mvADC =	gpadc_sample_to_mv(result);
													uint8_t adcData[2];	
													adcData[0] = (mvADC>>8);
													adcData[1] = (mvADC>>0);
												
														//Response 5
														X[0] = start; // start
														X[1] = refNo[0]; //ref no1
														X[2] = refNo[1]; // ref no2
														X[3] = doorunlockedStatus; //did motor unlock?
														X[4] = adcData[0];
														X[5] = adcData[1];
														X[6] =	boltFlag;// boltFlag
														X[7] =  handleFlag; // was handle moved?
														X[8] = 0;
														X[9] = 0;
														X[10] = 0;
														X[11] = 0;
														X[12] = 0;
														X[13] = 0;
														X[14] = 0;
														X[15] = end;
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
											 }
											 }
										 
else{
/// blocked key id 2 is  not matching
errorCode = 0xF5;
//Response 5
	X[0] = start; // start
		X[1] = refNo[0]; //ref no1
		X[2] = refNo[1]; // ref no2
		X[3] = doorunlockedStatus; //did motor unlock?
		X[4] = errorCode; //Error code
		X[5] = boltFlag;// boltFlag
		X[6] = handleFlag; // was handle moved?
		X[7] = 0;
		X[8] = 0;
		X[9] = 0;
		X[10] = 0;
		X[11] = 0;
		X[12] = 0; 
		X[13] = 0;
		X[14] = 0;
		X[15] = end; // end
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
											 
										 }
else{
/// blocked key id 1 is  not matching
errorCode = 0xF4;
//Response 4
	X[0] = start; // start
		X[1] = refNo[0]; //ref no1
		X[2] = refNo[1]; // ref no2
		X[3] = doorunlockedStatus; //did motor unlock?
		X[4] = errorCode; //Error code
		X[5] = boltFlag;// boltFlag
		X[6] = handleFlag; // was handle moved?
		X[7] = 0;
		X[8] = 0;
		X[9] = 0;
		X[10] = 0;
		X[11] = 0;
		X[12] = 0; 
		X[13] = 0;
		X[14] = 0;
		X[15] = end; // end
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
										 
								}
else{
/// key userflag is  not matching
errorCode = 0xF3;
//Response 3
	X[0] = start; // start
		X[1] = refNo[0]; //ref no1
		X[2] = refNo[1]; // ref no2
		X[3] = doorunlockedStatus; //did motor unlock?
		X[4] = errorCode; //Error code
		X[5] = boltFlag;// boltFlag
		X[6] = handleFlag; // was handle moved?
		X[7] = 0;
		X[8] = 0;
		X[9] = 0;
		X[10] = 0;
		X[11] = 0;
		X[12] = 0; 
		X[13] = 0;
		X[14] = 0;
		X[15] = end; // end
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

				}
else{
/// key string is  not matching
errorCode = 0xF2;
//Response 2
	X[0] = start; // start
		X[1] = refNo[0]; //ref no1
		X[2] = refNo[1]; // ref no2
		X[3] = doorunlockedStatus; //did motor unlock?
		X[4] = errorCode; //Error code
		X[5] = boltFlag;// boltFlag
		X[6] = handleFlag; // was handle moved?
		X[7] = 0;
		X[8] = 0;
		X[9] = 0;
		X[10] = 0;
		X[11] = 0;
		X[12] = 0; 
		X[13] = 0;
		X[14] = 0;
		X[15] = end; // end
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
				
		
		
	
}
else{
			/// Date and time flag is  not matching
			errorCode = 0xF1;
			//Response 1
			X[0] = start; // start
		X[1] = refNo[0]; //ref no1
		X[2] = refNo[1]; // ref no2
		X[3] = doorunlockedStatus; //did motor unlock?
		X[4] = errorCode; //Error code
		X[5] = boltFlag;// boltFlag
		X[6] = handleFlag; // was handle moved?
		X[7] = 0;
		X[8] = 0;
		X[9] = 0;
		X[10] = 0;
		X[11] = 0;
		X[12] = 0; 
		X[13] = 0;
		X[14] = 0;
		X[15] = end; // end
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

	//}
else {
/// Start or end is  not matching
	errorCode = 0xF0;
	//Response 0
		X[0] = start; // start
		X[1] = refNo[0]; //ref no1
		X[2] = refNo[1]; // ref no2
		X[3] = doorunlockedStatus; //did motor unlock?
		X[4] = errorCode; //Error code
		X[5] = boltFlag;// boltFlag
		X[6] = handleFlag; // was handle moved?
		X[7] = 0;
		X[8] = 0;
		X[9] = 0;
		X[10] = 0;
		X[11] = 0;
		X[12] = 0; 
		X[13] = 0;
		X[14] = 0;
		X[15] = end; // end
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
}