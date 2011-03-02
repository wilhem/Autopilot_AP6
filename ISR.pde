

#if RADIO_TYPE == 0

ISR(PCINT2_vect){
  
  cnt = micros();
  
  switch(CH){
    
    case 1:
      // CH1 is selected Aileron value to be measured
      if((PIND & (1 << PIND2) && (!FLAG_ail))){   // PD2 as Input pin right now...
      
        aileronSignalLengthOld = aileronSignalLength;    // save the old value
        aileronSignalLength = cnt;
        
        FLAG_ail = 1;
  
        break;      
      }
     
      if(FLAG_ail){
        
        aileronSignalLength = cnt - aileronSignalLength;
        
        FLAG_ail = 0;  // No receiving for the moment (for this channel)...

        CH = 2;    // Listen to CH2

        PCMSK2 = 0x00;    // Disable interrupts on this pin
        PCMSK2 |= (1 << PCINT20);     // Enable the interrupts for the next channel

      }
      
      break;


    case 2:
     // CH2 is selected Elevator value to be measured
      if((PIND & (1 << PIND4) && (!FLAG_elv))){   // PD4 as Input pin right now...
      
        elevatorSignalLengthOld = elevatorSignalLength;
        elevatorSignalLength = cnt;
        
        FLAG_elv = 1;
        
        break;
      }
     
      if(FLAG_elv){
        
        elevatorSignalLength = cnt - elevatorSignalLength;

        FLAG_elv = 0;  // No receiving for the moment...

        CH = 3;    // Listen to CH3

        PCMSK2 = 0x00;    // Disable interrupts on this pin
        PCMSK2 |= (1 << PCINT19);     // Interrupt für den nächsten Kanal wird nun angeschaltet (Throttle wird aber gesprungen), deshalb => PCINT21 statt PCINT20
      }
      
      break;


    case 3:
      // CH3 is selected Throttle value to be measured
      if((PIND & (1 << PIND3) && (!FLAG_thr))){   // PD3 as Input pin right now...
      
        throttleSignalLengthOld = throttleSignalLength;
        throttleSignalLength = cnt;
        
        FLAG_thr = 1;
        
        break;
      }
     
      if(FLAG_thr){
        
        throttleSignalLength = cnt - throttleSignalLength;
        
        FLAG_thr = 0;  // No receiving for the moment...

        CH = 4;    // Listen to CH4

        PCMSK2 = 0x00;    // Disable interrupts on this pin
        PCMSK2 |= (1 << PCINT21);
      }
      
      break;



    case 4:
      // CH4 is selected Rudder value to be measured
      if((PIND & (1 << PIND5) && (!FLAG_rud))){   // PD5 as Input pin right now...
      
        rudderSignalLengthOld = rudderSignalLength;
        rudderSignalLength = cnt;
        
        FLAG_rud = 1;
        
        break;
      }
     
      if(FLAG_rud){
        
        rudderSignalLength = cnt - rudderSignalLength;
        
        FLAG_rud = 0;  // No receiving for the moment...

        CH = 5;    // Listen to CH5

        PCMSK2 = 0x00;    // Disable interrupts on this pin
        PCMSK2 |= (1 << PCINT22);     // Enable the interrupts for the next channel

      }
      
      break;
       
      

    case 5:
      // CH5 is selected AutoPilot value to be measured
      if((PIND & (1 << PIND6) && (!FLAG_aut))){   // PD6 as Input pin right now...
        autopilotSignalLength = cnt;
        
        FLAG_aut = 1;
        
        break;
      }
     
      if(FLAG_aut){
        
        autopilotSignalLength = cnt - autopilotSignalLength;
        
        FLAG_aut = 0;  // No receiving for the moment...

        CH = 1;    // Listen to CH1

        PCMSK2 = 0x00;    // Disable interrupts on this pin
        PCMSK2 |= (1 << PCINT18);     // Enable the interrupts for the next channel

      }
      
      break;     
      
     default:
      break;   
  }
  
}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if RADIO_TYPE == 1

ISR(PCINT2_vect){
  
  cnt = micros();
  
  switch(CH){
    
    case 1:
      // CH1 is selected Aileron value to be measured
      if(((PIND & (1 << PIND2)) && (!FLAG_ail))){   // PD2 as Input pin right now...
      
        aileronSignalLength = cnt;
        
        FLAG_ail = 1;
  
        break;      
      }
     
      if(FLAG_ail){
        
        aileronSignalDiff = cnt - aileronSignalLength;
        
        FLAG_ail = 0;  // No receiving for the moment (for this channel)...

        CH = 2;    // Listen to CH2

        PCMSK2 = 0x00;    // Disable interrupts on this pin
        PCMSK2 |= (1 << PCINT19);     // Enable the interrupts for the next channel

      }
      
      break;


    case 2:
     // CH2 is selected Elevator value to be measured
      if(((PIND & (1 << PIND3)) && (!FLAG_elv))){   // PD4 as Input pin right now...

        elevatorSignalLength = cnt;
        
        FLAG_elv = 1;
        
        break;
      }
     
      if(FLAG_elv){
        
        elevatorSignalDiff = cnt - elevatorSignalLength;

        FLAG_elv = 0;  // No receiving for the moment...

        CH = 3;    // Listen to CH3

        PCMSK2 = 0x00;    // Disable interrupts on this pin
        PCMSK2 |= (1 << PCINT20);     // Interrupt für den nächsten Kanal wird nun angeschaltet (Throttle wird aber gesprungen), deshalb => PCINT21 statt PCINT20
      }
      
      break;


    case 3:
      // CH3 is selected Throttle value to be measured
      if(((PIND & (1 << PIND4)) && (!FLAG_thr))){   // PD3 as Input pin right now...

        throttleSignalLength = cnt;
        
        FLAG_thr = 1;
        
        break;
      }
     
      if(FLAG_thr){
        
        throttleSignalDiff = cnt - throttleSignalLength;
        
        FLAG_thr = 0;  // No receiving for the moment...

        CH = 4;    // Listen to CH4

        PCMSK2 = 0x00;    // Disable interrupts on this pin
        PCMSK2 |= (1 << PCINT21);
      }
      
      break;



    case 4:
      // CH4 is selected Rudder value to be measured
      if(((PIND & (1 << PIND5)) && (!FLAG_rud))){   // PD5 as Input pin right now...

        rudderSignalLength = cnt;
        
        FLAG_rud = 1;
        
        break;
      }
     
      if(FLAG_rud){
        
        rudderSignalDiff = cnt - rudderSignalLength;
        
        FLAG_rud = 0;  // No receiving for the moment...

        CH = 5;    // Listen to CH5

        PCMSK2 = 0x00;    // Disable interrupts on this pin
        PCMSK2 |= (1 << PCINT22);     // Enable the interrupts for the next channel

      }
      
      break;
       
      

    case 5:
      // CH5 is selected AutoPilot value to be measured
      if(((PIND & (1 << PIND6)) && (!FLAG_aut))){   // PD6 as Input pin right now...
      
        autopilotSignalLength = cnt;
        
        FLAG_aut = 1;
        
        break;
      }
     
      if(FLAG_aut){
        
        autopilotSignalDiff = cnt - autopilotSignalLength;
        
        FLAG_aut = 0;  // No receiving for the moment...

        CH = 1;    // Listen to CH1

        PCMSK2 = 0x00;    // Disable interrupts on this pin
        PCMSK2 |= (1 << PCINT18);     // Enable the interrupts for the next channel

      }
      
      break;     
      
     default:
      break;   
  }
  
}

#endif
