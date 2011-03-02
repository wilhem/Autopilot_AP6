/******************************************************
*
*   The whole procedure must be esecuted only once
*
*   The following procedure selects the correct channel for
*   the correct sensor and starts the bias procedure
*
*   Those values have been taken considering the axis COMPLETELY
*   LEVELLED referred to ground. Those are really the bias Values
*
******************************************************/
void calibration(void){  
  
#if CALIBRATION == 0

  // Those are default values from variuos experiments it should be made a calibration for every restart
//  biasX = EEPROMReadInt(0);
//  biasY = EEPROMReadInt(2);
//  biasGyro = EEPROMReadInt(4);

  biasX = bias(accXoutPin);  // Letzter Wert: 505;
  biasY = bias(accYoutPin);  // Letzter Wert: 516;
  biasZ = 607;
  biasPitch = bias(gyroPitchOutPin);  // 382;
  biasRoll = bias(gyroRollOutPin);  // 381;
  biasYaw = bias(gyroYawOutPin);  // 383;
  

#endif

#if CALIBRATION == 1

  // Measure and store the bias value for X accelerometer
  biasX = bias(accXoutPin);
  
  // Measure and store the bias value for Y accelerometer
  biasY = bias(accYoutPin);
  
  // Measure and store the bias value for Y accelerometer
  biasZ = bias(accZoutPin);

#endif
  
}



unsigned int bias(int pin){
  
  long sumSample = 0;
  
  for(int i = 0; i < 1024; i++){
    sumSample += analogRead(pin);
    delayMicroseconds(100);
  }
  
  return sumSample >> 10;
}


// This function will write a 2 byte integer to the specific adress to the EEPROM
void EEPROMWriteInt(int p_address, int p_value){
  byte lox = ((p_value >> 0) & 0xFF);
  byte hix = ((p_value >> 8) & 0xFF);
  
  EEPROM.write(p_address, lox);
  EEPROM.write(p_address + 1, hix);
}

// This function will read a 2 byte from the EEPROM and will convert it to a integer
unsigned int EEPROMReadInt(int p_address){
  byte lox = EEPROM.read(p_address);
  byte hix = EEPROM.read(p_address + 1);
  
  return ((lox << 0) & 0xFF) + ((hix << 8) & 0xFF00);
}

