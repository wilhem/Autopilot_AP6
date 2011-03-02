/*********** FIR DIGITAL FILTER************************
*  http://www.dsptutor.freeuk.com/FIRFilterDesign/FIRFilterDesign.html
*
*  The sampling is done @ 200 Hz, it means that every 5 milliSec this function is
*  from the main loop
*  Since the frequency is 200 Hz the cut off frequency will be 70 Hz
*
*  Rectangular window FIR filter
*
*  Filter type: LP
*  Passband: 0 - 70 Hz
*  Order: 8
*  Transition band: 920 Hz
*  Stopband attenuation: 21 dB
*
*  Coefficients:
*
*  a[0] =	0.080423184
*  a[1] =	0.1021727
*  a[2] =	0.1194973
*  a[3] =	0.13065423
*  a[4] =	0.1345051
*  a[5] =	0.13065423
*  a[6] =	0.1194973
*  a[7] =	0.1021727
*  a[8] =	0.080423184
*
* Since this is a FIR filter circular it takes as argument:
* the value to be stored as result, the pointer of the last position saved, the delay line,
* the number of taps, and the sample pin
*
*********************************************************************/
void FIR(float *input, int *pointer, float z[], const float h[], const int ntaps, const int samplePin){
  
  float sum = 0.0f;
  int pos = *pointer;       // remember last position saved
  int ii;
  
  
  if(--pos < 0) pos = ntaps - 1;
  
  z[pos] = analogRead(samplePin);
  
  for(ii = 0; ii < ntaps; ii++){
    
    sum += h[ii] * z[pos];
    
    if(++pos >= ntaps) pos = 0;
    
  }
  
  *input = sum;
  
  *pointer = pos;
  
}

/*********** FIR DIGITAL FILTER************************
*  http://www.dsptutor.freeuk.com/FIRFilterDesign/FIRFilterDesign.html
*
*  The sampling is done @ 200 Hz, it means that every 5 milliSec this function is
*  from the main loop
*  Since the frequency is 200 Hz the cut off frequency will be 70 Hz
*
*  Rectangular window FIR filter
*
* Filter type: LP
* Passband: 0 - 95 Hz
* Order: 4
* Transition band: 1840 Hz
* Stopband attenuation: 21 dB
*
* Coefficients:
*
* a[0] =        0.15486716
* a[1] =	0.22164172
* a[2] =	0.24698229
* a[3] =	0.22164172
* a[4] =	0.15486716
*
* Since this is a FIR filter circular it takes as argument:
* the value to be stored as result, the pointer of the last position saved, the delay line,
* the number of taps, and the sample pin
*
*********************************************************************/

void digitalFilter(void){
    
    FIR(&Yout, &Ypointer, YoutOld, hacc, ntapsAcc, accYoutPin);  // Execute it every 5 milliSecs
    FIR(&Xout, &Xpointer, XoutOld, hacc, ntapsAcc, accXoutPin);  // Execute it every 5 milliSecs
    FIR(&Zout, &Zpointer, ZoutOld, hacc, ntapsAcc, accZoutPin);  // Execute it every 5 milliSecs    
    FIR(&PitchOut, &PitchPointer, PitchOutOld, hgyro, ntapsGyro, gyroPitchOutPin);  // Execute it every 5 milliSecs
    FIR(&RollOut, &RollPointer, RollOutOld, hgyro, ntapsGyro, gyroRollOutPin);  // Execute it every 5 milliSecs
//    FIR(&YawOut, &YawPointer, YawOutOld, hgyro, ntapsGyro, gyroYawOutPin);  // Execute it every 5 milliSecs    
}

