
void servo(void){

    
    servoOut[CH_AIL] = constrain(radioIn[CH_AIL], lengthPulseMin, lengthPulseMax);
    
    if(rollPID.GetMode()){
      rollSetPoint = map(servoOut[CH_AIL], lengthPulseMin, lengthPulseMax, -90, 90);
      rollPID.Compute();
      aileronServo.write((outputAileron + 90));
    } else {
      aileronServo.writeMicroseconds(servoOut[CH_AIL]);
    }
    
    servoOut[CH_ELV] = constrain(radioIn[CH_ELV], lengthPulseMin, lengthPulseMax);
    
    if(pitchPID.GetMode()){
      pitchSetPoint = map(servoOut[CH_ELV], lengthPulseMin, lengthPulseMax, 90, -90);   // Reverse because it has been reversed via radio (due to servos position)
      pitchPID.Compute();
      elevatorServo.write((outputElevator + 90));
    } else {
      elevatorServo.writeMicroseconds(servoOut[CH_ELV]);
    }

    servoOut[CH_THR] = constrain(radioIn[CH_THR], lengthPulseMin, lengthPulseMax);
    throttleServo.writeMicroseconds((servoOut[CH_THR] - 200));

    servoOut[CH_RUD] = constrain(radioIn[CH_RUD], lengthPulseMin, lengthPulseMax);
    rudderServo.writeMicroseconds(servoOut[CH_RUD]);

 

#if PROCESSING == 0

   if(radioIn[CH_LDG] < 1500){      // smaller than because otherwise the landing gear cannot be used!!
      
      if(!pitchPID.GetMode()){
        pitchPID.SetMode(AUTO);
      }
     
      if(!rollPID.GetMode()){
        rollPID.SetMode(AUTO);
      }
      
    } else {
     
      if(pitchPID.GetMode()){
        pitchPID.SetMode(MANUAL);
      }

      if(rollPID.GetMode()){
        rollPID.SetMode(MANUAL);
      }
  }
  
#endif
}
