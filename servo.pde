
void servo(void){
    
    if(rollPID.GetMode()){
      rollSetPoint = (int)map(radioIn[CH_AIL], lengthPulseMin, lengthPulseMax, -45, 45);
      rollPID.Compute();
      aileronServo.write(((int)outputAileron + 90));
  } else {
      servoOut[CH_AIL] = constrain(radioIn[CH_AIL], lengthPulseMin, lengthPulseMax);
      aileronServo.writeMicroseconds(servoOut[CH_AIL]);
    }
    
    if(pitchPID.GetMode()){
      pitchSetPoint = (int)map(radioIn[CH_ELV], lengthPulseMin, lengthPulseMax, 45, -45);   // Reverse because it has been reversed via radio (due to servos position)
      pitchPID.Compute();
      elevatorServo.write(((int)outputElevator + 90));
    } else {
      servoOut[CH_ELV] = constrain(radioIn[CH_ELV], minPulseServo, maxPulseServo);
      elevatorServo.writeMicroseconds(servoOut[CH_ELV]);
    }

    servoOut[CH_THR] = constrain(radioIn[CH_THR], lengthPulseMin, lengthPulseMax);
    throttleServo.writeMicroseconds(servoOut[CH_THR]);

    servoOut[CH_RUD] = constrain(radioIn[CH_RUD], lengthPulseMin, lengthPulseMax);
    rudderServo.writeMicroseconds(servoOut[CH_RUD]);


#if PROCESSING == 0

   if(radioIn[CH_LDG] < 1400){
     
     autoPilot = 1;   
      
   } else {
     
     autoPilot = 0;
     
   }

   if(autoPilot != pitchPID.GetMode()){
        (autoPilot == 1)?pitchPID.SetMode(AUTO):pitchPID.SetMode(MANUAL);
   }
    
   if(autoPilot != rollPID.GetMode()){
     
        (autoPilot == 1)?rollPID.SetMode(AUTO):rollPID.SetMode(MANUAL);
           
   }
  
#endif
}
