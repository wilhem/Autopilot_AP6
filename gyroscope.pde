/*********************************************************************
*							
*  This function calculates the Theta angle with the   
*  following equation:                            
*						
*	Theta(t) = Theta(t0) + (Vout - Voffset) * dt	
*                               --------------		
*                                 Sensitivity		
*							
*  Where:						
*  - Sensitivity is NOT RATIOMETRIC (constant for every value of power)
*     and is equal to: Sensitivity = TO BE CALIBRATED		
*  - adc is the conversion from ADC value to real Volt value
**********************************************************************/
#define adc 3.38      // adc = 3300/1023
#define sens 190.79    // 190.79 mV/rad/sec


float gyro_angle(float gyroOutput, float gyroOffsetNull)
{
	return (((gyroOutput - gyroOffsetNull) * adc) / sens);
}
