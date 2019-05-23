/* =================================================================================
File name:  HVACI_Sensorless-Settings.H                     
Description:Incremental Build Level control file.
=================================================================================  */
#ifndef PROJ_SETTINGS_H

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/
#define LEVEL1  1      		// Module check out, duty cycle waveforms and PWM update  
#define LEVEL2  2           // Verify ADC, park/clarke, calibrate the offset 
#define LEVEL3  3           // Two current PI regulator test, speed measurement 
#define LEVEL4  4           // Flux and speed estimator tests 
#define LEVEL5  5           // Speed PI regulator test (Sensored closed-loop FOC system) 
#define LEVEL6  6           // Sensorless closed-loop FOC system

/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/
#define   BUILDLEVEL LEVEL6


#ifndef BUILDLEVEL    
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL
//------------------------------------------------------------------------------


#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI 3.14159265358979

// Define the system frequency (MHz)
#if (DSP2803x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 60
#elif (DSP280x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 100
#elif (DSP2833x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 150
#endif


//Define system Math Type
// Select Floating Math Type for 2833x
// Select IQ Math Type for 2803x 
#if (DSP2803x_DEVICE_H==1)
#define MATH_TYPE 0 
#elif (DSP2833x_DEVICE_H==1)
#define MATH_TYPE 1
#endif



// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10

// Define the electrical motor parametes (1/4 hp Marathon Motor)
#define RS 		11.05		        // Stator resistance (ohm) 
#define RR   	6.11		        // Rotor resistance (ohm) 
#define LS   	0.316423    	  	// Stator inductance (H) 
#define LR   	0.316423	  		// Rotor inductance (H) 	
#define LM   	0.293939	   		// Magnatizing inductance (H)
#define POLES  	4					// Number of poles

// Define the base quantites for PU system conversion
#define BASE_VOLTAGE    236.174     // Base peak phase voltage (volt)
#define BASE_CURRENT    10          // Base peak phase current (amp)
#define BASE_TORQUE         		// Base torque (N.m)
#define BASE_FLUX       		    // Base flux linkage (volt.sec/rad)
#define BASE_FREQ      	120         // Base electrical frequency (Hz) 
									// Note that 0.5 pu (1800 rpm) is max for Marathon motor 
									// Above 1800 rpm, field weakening is needed.
#endif

