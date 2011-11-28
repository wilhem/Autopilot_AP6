#ifndef define_h
#define define_h

#include "WProgram.h"

#define PROCESSING 0
#define CALIBRATION 0
#define DEBUGLEVEL 0
#define RADIO_TYPE 1  // 0 => Die Empfängers Outputs sind parallel zueinander: zB: Futaba
                      // 1 => Die Empfängers Outputs überliegen sich nicht, sind nacheinander: zB: 2.4 GHz
#define ESC_ARM 0     // 0 Der Regler wird nicht gearmt
                      // 1 Der Regler wird gearmt

#define BAUDRATE 115200

// Aus dem Empfänger
#define lengthPulseMin 700
#define lengthPulseMax 2100
#define lengthPulseMed 1400
// Für die Servos
#define minPulseServo 1100  // 650
#define maxPulseServo 1900 // 2300
// Für die Radio
#define NUM_CHANNEL 5
#define CH_AIL 0
#define CH_ELV 1
#define CH_THR 2
#define CH_RUD 3
#define CH_LDG 4

#define Kp_pitch 3.0 //-3.0  // Pitch verstärker Proportional
#define Ki_pitch 3.0   // Pitch verstärker Integral
#define Kd_pitch 0.15  // Pitch verstärker Derivative
#define Kp_roll 2.0
#define Ki_roll 0.5
#define Kd_roll 0.35

#endif
