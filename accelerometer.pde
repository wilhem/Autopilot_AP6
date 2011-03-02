/*************************************************************
*	This function calculates the value of the Theta angle between 
*	the absolut vertical and the reading of the accelerometer
*
*	The Sensitivity in output in mV per g 
*	From the datasheet it should be (With exaclty 
*	Vdd = 5V) iDeltaOutput = 312 mV/g (~ 64 in ADC conv)
*	For this test we assume Vdd exactly @ 5.00 V
*
*
*   X and Y refers to the real axis through the breadboard, and not to what is displayed on the board
*   It returns the accelerometers reading in RADIANTS
*
**************************************************************/


float accel_angle(float a_value, float bias_a, float b_value, float bias_b)
{    
	return (-atan2((a_value - bias_a), (b_value - bias_b)));
}
