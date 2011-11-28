/*******************************************************************************
*
* Autopilot AP6 "Mizard"
*
* "Copyright (C) 2011 Davide Picchi"  mailto: paveway@gmail.com
* 
* This program is distributed under the terms of the GNU General Public License.
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
* Autopilot AP6 "Mizard" is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.

* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.

* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
* (see COPYING file)
*
*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*    
*    Davide Picchi
*    
*    Starting date: 16 Juli 2010
*
*+++++++++++++++++++++++++++++++++++++++++++++++++
*
*  Diese Version verwendet PORTD, damit der Empfänger eingelesen wird
* 
*  Einsatz: Razor 6DOF + Arduino Nano
*
*  Diese Version ist bereits zum fliegen...
*
*************************************************/
#include <avr/interrupt.h>
#include <avr/io.h>

#include <EEPROM.h>
#include <Kalman2.h>
#include <Servo.h>
#include <PID_v1.h>
#include "define_AP6.h"

#define ToDeg(x) (x*57.2957795131)  // * 180/pi



/***********************
* GLOBAL SCOPE VARIABLES
***********************/
float Xout = 0.0f, Yout = 0.0f, Zout = 0.0f, PitchOut = 0.0f, RollOut = 0.0f, YawOut = 0.0f;

/********************************************************************
* Variables in which are stored values from the calibration procedure
********************************************************************/
unsigned int biasX, biasY, biasZ, biasPitch, biasRoll, biasYaw;


/***********************************
* Definitions for analog sampling
***********************************/   
const int accXoutPin = A1;        // Acceleremoter X output pin, ratiometric
const int accYoutPin = A0;        // Accelerometer Y output pin, ratiometric
const int accZoutPin = A2;        // Accelerometer Z output pin, ratiometric
const int gyroPitchOutPin = A5;   // Gyro Output, pin not ratiometric
const int gyroRollOutPin = A4;    // Gyro Output, pin not ratiometric
const int gyroYawOutPin = A3;     // Gyro Output, pin not ratiometric

const float hacc[] = {0.080423184, 0.1021727, 0.1194973, 0.13065423, 0.1345051, 0.13065423, 0.1194973, 0.1021727, 0.080423184};
float YoutOld[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float XoutOld[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float ZoutOld[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int Xpointer = 0;
int Ypointer = 0;
int Zpointer = 0;

const float hgyro[] = {0.15486716, 0.22164172, 0.24698229, 0.22164172, 0.15486716};
float PitchOutOld[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float RollOutOld[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float YawOutOld[] = {0.0, 0.0, 0.0, 0.0, 0.0};
int PitchPointer = 0;
int RollPointer = 0;
int YawPointer = 0;

const int ntapsAcc = 9;
const int ntapsGyro = 5;


/****************************************
* Variablen, die durch den ausgeführten Interrupt gelesen werden
****************************************/
volatile unsigned int radioIn[NUM_CHANNEL];
unsigned int servoOut[NUM_CHANNEL];
volatile unsigned int aileronSignalLength, aileronSignalDiff, elevatorSignalLength, elevatorSignalDiff, throttleSignalLength, throttleSignalDiff, rudderSignalLength, rudderSignalDiff, autopilotSignalLength, autopilotSignalDiff;
volatile unsigned int cnt;
volatile char CH = 1;           // Erster Kanal, der aus dem Empfänger eingelesen wird
volatile char FLAG_ail = 0, FLAG_elv = 0, FLAG_thr = 0, FLAG_rud = 0, FLAG_aut = 0;
int autoPilot = 0;
	
/********************
* Variable for angles
*
* Values in degree to store angle in the body frame reference
********************/
  double pitchAngle = 0.0f;     // Body frame reference
  double rollAngle = 0.0f;       // Body frame reference
//  float yawAngle = 0.0f;        // Body frame reference
  
  double outputElevator = 0.0f;
  double pitchSetPoint = 0.0f;
  double outputAileron = 0.0f;
  double rollSetPoint = 0.0f;
  

/*******************************
* Define the axis to be filtered
*******************************/
Kalman2 pitch, roll;


/****************************
* Define the PID
****************************/
PID pitchPID(&pitchAngle, &outputElevator, &pitchSetPoint, Kp_pitch, Ki_pitch, Kd_pitch, REVERSE);
PID rollPID(&rollAngle, &outputAileron, &rollSetPoint, Kp_roll, Ki_roll, Kd_roll, DIRECT);


/********************************************************
 * Variable to space every loop routine exactly of 20 mSec
 ********************************************************/
unsigned long timeElapsedFastLoop, timeElapsedSlowLoop, serialTime;

/************************************
* Define Servos and pin configuration
************************************/
Servo aileronServo, elevatorServo, rudderServo, throttleServo;

const int aileronServoPin = 8;
const int elevatorServoPin = 9;
const int throttleServoPin = 10;
const int rudderServoPin = 11;

/******************************************************
* Definition von Variablen, die den Autopilot ansteuern
******************************************************/
char task = 1;

/***************************
* Output chars and variables
***************************/
const int ledCalibration = 13;


#if DEBUGLEVEL == 1

  char bufferOutput[8];
  char *s1;
  const char *s2 = "kp";
  const char *s3 = "kr";
  
#endif


/**************
* Initial Setup
**************/
void setup(){
   
   /*****************
   * Servo definition
   *****************/
   aileronServo.attach(aileronServoPin);
   elevatorServo.attach(elevatorServoPin);
   throttleServo.attach(throttleServoPin);
   throttleServo.writeMicroseconds(lengthPulseMin-lengthPulseMin);
   rudderServo.attach(rudderServoPin);

   pinMode(ledCalibration, OUTPUT);    // Led on digital pin 13 is used as indicator to show that the calibration is on the way...
   
   analogReference(EXTERNAL);
  
   pitch.init(0.5, 0.2);   // Pass to the the kalman filter the value of the noise from datasheet
   roll.init(0.5, 0.2);   // Pass to the the kalman filter the value of the noise from datasheet 
   
   set_interrupt();   // Erforderlich. Durch diese Funktion die einzelnen Interrupts angeschaltet werden
   
   //************************************************************//
   digitalWrite(ledCalibration, HIGH); // Led on, before the end of the cycle the aircraft must be parallel with the ground to calibrate the sensor
   
   // ESC muss NICHT gearmed werden.... der Regler ist intelligent genüg
   #if ESC_ARM == 0
   
   aileronServo.writeMicroseconds((maxPulseServo + minPulseServo)/2);
   elevatorServo.writeMicroseconds((maxPulseServo + minPulseServo)/2);
   rudderServo.writeMicroseconds((maxPulseServo + minPulseServo)/2);
   
   delay(5000);
     
     for(int j = 0; j < 250; j++){
       read_radio();
       delay(20);
     }
     
   #endif
     
   // ESC muss gearmed werden...
   #if ESC_ARM == 1
   
   delay(6000);
     
     for(int j = 0; j < 50; j++){
       read_radio();
       delay(20);
     }
     
     // Nutze diese Gelegenheit um den Motor zu armen
     for(int j = 0; j < 50; j++){
       throttleServo.writeMicroseconds(minPulseServo);
       delay(20);
     }
     
     for(int j = 0; j < 50; j++){
       throttleServo.writeMicroseconds(maxPulseServo);
       delay(20);
     }
     for(int j = 0; j < 50; j++){
       throttleServo.writeMicroseconds(minPulseServo);
       delay(20);
     }

   #endif
   
   digitalWrite(ledCalibration, LOW);
   //************************************************************//
   
   calibration();     // Accelerometers and Gyros bias calibration   
   
   /*******************
   * PID Initialization
   *******************/
   
   pitchPID.SetMode(MANUAL);
//   pitchPID.SetInputLimits(-90, 90);
   pitchPID.SetOutputLimits(-45, 45);
   pitchPID.SetSampleTime(20);
 
   rollPID.SetMode(MANUAL);
//   rollPID.SetInputLimits(-90, 90);
   rollPID.SetOutputLimits(-45, 45);
   rollPID.SetSampleTime(20);
   

   Serial.begin(BAUDRATE);      // Serial communication with the pc
   Serial.flush();
   
   // Jetzt zeige durch das Led-Licht, dass das ausgeführte Kalibrieren zum Ende gekommen ist
     
   for(int i = 0; i < 3; i++){
     digitalWrite(ledCalibration, HIGH);
     delay(100);
     digitalWrite(ledCalibration, LOW);
     delay(100);
   }
   
}


void loop(){
  
  if(((millis() - timeElapsedSlowLoop) > 19)){
    timeElapsedSlowLoop = millis();
    timeElapsedFastLoop = timeElapsedSlowLoop;

    digitalFilter();

    kalmanRoutine();
       
    task = 1;
    

#if PROCESSING == 1

  //send-receive with processing if it's time 
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
  
#endif

#if (DEBUGLEVEL == 1)

    itoa(((int)(pitchAngle)), bufferOutput, 10);
    s1 = bufferOutput;
    strcat(s1, s2);
    Serial.println(s1);
    
    itoa(((int)(rollAngle)), bufferOutput, 10);
    s1 = bufferOutput;
    strcat(s1, s3);
    Serial.println(s1);
    
#endif

  }


  if(((millis() - timeElapsedFastLoop) > 4)){
    timeElapsedFastLoop = millis();

    switch(task){
      case 1:
       digitalFilter();
               
       rollAngle = ToDeg(roll.getAngle());
       pitchAngle = ToDeg(pitch.getAngle());
       task++;
       break;
       
      case 2:
       digitalFilter();

       read_radio();
       task++;
       break;
       
      case 3:
       digitalFilter();
    
       servo();    // Function to calculate the correct output to be transmitted to the PIC
       task++;
       break;
    
      default:
       break;
       
    }

  }
    
}


void set_interrupt(void){
  
   DDRD &= ~((1 << PIND2) | (1 << PIND3) | (1 << PIND4) | (1 << PIND5) | (1 << PIND6));   // Input Pin
   PORTB &= ~((1 << PIND2) | (1 << PIND3) | (1 << PIND4) | (1 << PIND5) | (1 << PIND6));  // Pull - up disabled
   
   // Configure external interrupts on PCINT2 (from PD0 to PD7)
   PCMSK2 = 0;                 // Disable interrupts on overa single port
   PCMSK2 |= ((1 << PCINT18)); // At the moment only PD2 enabled
   PCICR |= (1 << PCIE2);      // Enabled interrupts on PCINT23-16

}


void kalmanRoutine(void){
  
    /*****************************
    * KALMAN 
    *****************************/
    pitch.correction(gyro_angle(PitchOut, biasPitch), accel_angle(Xout, biasX, Zout, biasZ));
    roll.correction(gyro_angle(RollOut, biasRoll), accel_angle(Yout, biasY, Zout, biasZ));
    
    pitch.prediction(gyro_angle(PitchOut, biasPitch));
    roll.prediction(gyro_angle(RollOut, biasRoll));

}

/****************************
Auf dem Empfänger:

1 => Aileron
2 => Throttle
3 => Elevator
4 => Rudder
5 => Autopilot/Ldg
*******************/
