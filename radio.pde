
void read_radio(){
  
  radioIn[CH_AIL] = radioIn[CH_AIL] * 0.7 + aileronSignalDiff * 0.3;
  radioIn[CH_ELV] = radioIn[CH_ELV] * 0.7 + elevatorSignalDiff * 0.3;
  radioIn[CH_THR] = radioIn[CH_THR] * 0.7 + throttleSignalDiff * 0.3;
  radioIn[CH_RUD] = radioIn[CH_RUD] * 0.7 + rudderSignalDiff * 0.3;
  radioIn[CH_LDG] = radioIn[CH_LDG] * 0.9 + autopilotSignalDiff * 0.1;
  
}
