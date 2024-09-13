/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by Luca Namestnik
            Last Update: March 17, 2020
						
						Last Update: April 1, 2024
						Updated by Luca Namestnik
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"



#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

#define MOTOR_DELAY 30000
#define MOTOR_DELAY_FAST 20000
#define SCANS 32	//512 must be divisible by this
#define PLANES 3

#define X_DISP 100		//in mm


void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0 
//not needed yet.
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
//This is not necessary will not be used for this right now.
void VL53L1X_XSHUT(void){
  GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
  GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
  FlashAllLEDs();
  SysTick_Wait10ms(10);
  GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void PortM_Init(void){
	//Use PortM pins (PM0-PM3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x1F;        								// configure Port M pins (PM0-PM4) as output
  GPIO_PORTM_AFSEL_R &= ~0x1F;     								// disable alt funct on Port M pins (PM0-PM4)
  GPIO_PORTM_DEN_R |= 0x1F;        								// enable digital I/O on Port M pins (PM0-PM4)
																									// configure Port M as GPIO
  GPIO_PORTM_AMSEL_R &= ~0x1F;     								// disable analog functionality on Port M	pins (PM0-PM4)	
	return;
}

void spin_cw(uint16_t steps) {
	for (int i = 0; i < steps; i++) {
		GPIO_PORTM_DATA_R = 0b0011;
		SysTick_Wait(MOTOR_DELAY);
		GPIO_PORTM_DATA_R = 0b0110;
		SysTick_Wait(MOTOR_DELAY);
		GPIO_PORTM_DATA_R = 0b1100;
		SysTick_Wait(MOTOR_DELAY);
		GPIO_PORTM_DATA_R = 0b1001;
		SysTick_Wait(MOTOR_DELAY);
	}
	GPIO_PORTM_DATA_R = 0;
}

void spin_ccw(uint16_t steps) {
	for (int i = 0; i < steps; i++) {
		GPIO_PORTM_DATA_R = 0b0011;
		SysTick_Wait(MOTOR_DELAY_FAST);
		GPIO_PORTM_DATA_R = 0b1001;
		SysTick_Wait(MOTOR_DELAY_FAST);
		GPIO_PORTM_DATA_R = 0b1100;
		SysTick_Wait(MOTOR_DELAY_FAST);
		GPIO_PORTM_DATA_R = 0b0110;
		SysTick_Wait(MOTOR_DELAY_FAST);
	}
	GPIO_PORTM_DATA_R = 0;
}

//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status = 0;

void scan_plane(uint8_t direction, uint8_t *RangeStatus, uint16_t *Distance, uint16_t *SignalRate, uint16_t *AmbientRate, uint16_t *SpadNum) {
	uint8_t dataReady = 0;
	status = VL53L1X_StartRanging(dev);
	for (int i = 0; i < SCANS; i++) {
		while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
			FlashLED3(1);
      VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
		
		status = VL53L1X_GetRangeStatus(dev, RangeStatus);
	  status = VL53L1X_GetDistance(dev, Distance);
		status = VL53L1X_GetSignalRate(dev, SignalRate);
		status = VL53L1X_GetAmbientRate(dev, AmbientRate);
		status = VL53L1X_GetSpadNb(dev, SpadNum);
		
		FlashLED1(1);
		
		status = VL53L1X_ClearInterrupt(dev);
		if (direction) spin_cw(512/SCANS);
		else spin_cw(512/SCANS);
		
		sprintf(printf_buffer,"%u, %u, %u, %u, %u, %u\r\n", i, *RangeStatus, *Distance, *SignalRate, *AmbientRate, *SpadNum);
		UART_printf(printf_buffer);
		
	}
	spin_ccw(512);
	VL53L1X_StopRanging(dev);
}

int main(void) {
  uint8_t sensorState = 0;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum;
  uint8_t RangeStatus;
	
	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortM_Init();
	PortG_Init();
	VL53L1X_XSHUT();
	//CHECKING THE PWM BELOW
	while(1) {
		GPIO_PORTM_DATA_R ^= 0x10;
		SysTick_Wait10ms(1);
		GPIO_PORTM_DATA_R ^= 0x10;
		SysTick_Wait10ms(1);
	}
	
	// 1 Wait for device booted
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(2);
  }
	FlashLED1(1);
	FlashLED2(2);
	FlashLED3(3);
	FlashLED4(4);
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* 2 Initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);

	
  /* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
  //status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
  //status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */
	
	
	while(1) if (UART_InChar() == 's') break;
	sprintf(printf_buffer, "%u %u %u", PLANES, X_DISP, SCANS);
	UART_printf(printf_buffer);
	
	
	for (int i = 0; i < PLANES; i++) {
		while(1) if (UART_InChar() == 's') break;
		scan_plane(i%2, &RangeStatus, &Distance, &SignalRate, &AmbientRate, &SpadNum);
	}

}

