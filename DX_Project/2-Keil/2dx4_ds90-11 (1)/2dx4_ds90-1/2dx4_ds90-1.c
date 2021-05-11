//Last digit: 0 -> Bus speed: 80Mhz
//Second Significant Digit: 8 -> Distance: PN1

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
//#include "sensor.h"



uint16_t	dev=0x52;


int status=0;

volatile int IntCount;

//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

//capture values from VL53L1X for inspection
uint16_t debugArray[100];

void spin(int steps, int direction){
	if(direction==0){
		for(int i=0; i<steps; i++){                 //when the input of direction is 0, rotate clockwisely   
			GPIO_PORTH_DATA_R = 0b00001001;
		  SysTick_Wait10ms(5);
		  GPIO_PORTH_DATA_R = 0b00000011;
		  SysTick_Wait10ms(5);
		  GPIO_PORTH_DATA_R = 0b0000110;
		  SysTick_Wait10ms(5);
		  GPIO_PORTH_DATA_R = 0b00001100;
		  SysTick_Wait10ms(5);
				}
			}
	if(direction==1){
		for(int i=0; i<steps; i++){                 //when the input of direction is 1, rotate counter clockwisely   
			GPIO_PORTH_DATA_R = 0b00001100;
		  SysTick_Wait10ms(5);
		  GPIO_PORTH_DATA_R = 0b0000110;
		  SysTick_Wait10ms(5);
		  GPIO_PORTH_DATA_R = 0b00000011;
		  SysTick_Wait10ms(5);
		  GPIO_PORTH_DATA_R = 0b00001001;
		  SysTick_Wait10ms(5);
				}
			}

}

void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	
	GPIO_PORTH_DIR_R |= 0xF;												
	GPIO_PORTH_AFSEL_R &= ~0xF;		 								
	GPIO_PORTH_DEN_R |= 0xF;											
																								
	GPIO_PORTH_AMSEL_R &= ~0xF;		 								
	
	return;
}

//Initialiaze GPIO Port N1
void PortN0_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 //activate the clock for Port N to use D2
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
	GPIO_PORTN_DIR_R|=0b00000010;                             //Here D2 and D1 are ready for output but we will only be using D2
	GPIO_PORTN_DEN_R|=0b00000010;
	return;
}

//The Blink function is to blink the D1 for 1ms
void Blink(void){
	//the LED D1 is on
	GPIO_PORTN_DATA_R = 0b00000010;
	//delay is 1 ms
	SysTick_Wait10ms(5);
	//The LED D1 is off
	GPIO_PORTN_DATA_R = 0b00000000;
	return;
}

void sensor(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	int16_t	x;
  int16_t	y;
  int16_t	z;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
	// hello world!
	//UART_printf("Program Begins\r\n");
	int mynumber = 1;
	//sprintf(printf_buffer,"2DX4 Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(dev, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
  myByteArray[i++] = byteData;

  status = VL53L1_RdByte(dev, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
  myByteArray[i++] = byteData;
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);

	//sprintf(printf_buffer,"Model_ID=0x%x , Module_Type=0x%x\r\n",myByteArray[0],myByteArray[1]);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(100);
  }
	FlashAllLEDs();
	//UART_printf("ToF Chip Booted!\r\n");
 	//UART_printf("One moment...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	//Status_Check("SensorInit", status);

	
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	//Status_Check("StartRanging", status);
	//UART_printf("Start Transmission\n");
for(int p = 1; p<6;p++){
	for(int k = 0;k<8;k++){
		Blink();
		for(int i = 0;i<8;i++){
			while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
			spin(8,0);
			
			VL53L1_WaitMs(dev, 5);
	  }

      dataReady = 0;
	  status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	  status = VL53L1X_GetDistance(dev, &Distance);
 

      debugArray[i] = Distance;
//	  status = VL53L1X_GetSignalRate(dev, &SignalRate);
//	  status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
//	  status = VL53L1X_GetSpadNb(dev, &SpadNum);

		
	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
      //sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
		sprintf(printf_buffer,"%uE\r\n", Distance);
		//sprintf(printf_buffer,"%d, %d, %d\r\n", x, y, z);
      UART_printf(printf_buffer);
		
	  //SysTick_Wait10ms(5);
 	} 
}
	for(int k = 0;k<8;k++){
		Blink();
		for(int i = 0;i<8;i++){
			while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
			spin(8,1);
			
			VL53L1_WaitMs(dev, 5);
	  }

      dataReady = 0;
	  status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	  status = VL53L1X_GetDistance(dev, &Distance);
 

      debugArray[i] = Distance;
//	  status = VL53L1X_GetSignalRate(dev, &SignalRate);
//	  status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
//	  status = VL53L1X_GetSpadNb(dev, &SpadNum);

		
	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
      //sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
		sprintf(printf_buffer,"%uE\r\n", Distance);
		//sprintf(printf_buffer,"%d, %d, %d\r\n", x, y, z);
      UART_printf(printf_buffer);
		
	  //SysTick_Wait10ms(5);
 	} 
	
}}
  VL53L1X_StopRanging(dev);



  while(1) {}

}


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
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3

                                                                                // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        // 8) configure for 100 kbps clock
        
  // 20*(TPR+1)*20ns = 10us, with TPR=24
    // TED 100 KHz
    //     CLK_PRD = 8.3ns
    //    TIMER_PRD = 1
    //    SCL_LP = 6
    //    SCL_HP = 4
    //    10us = 2 * (1 + TIMER_PRD) * (SCL_LP + SCL_HP) * CLK_PRD
    //    10us = 2 * (1+TIMER+PRD) * 10 * 8.3ns
    //  TIMER_PRD = 59 (0x3B)
    //
    // TIMER_PRD is a 6-bit value.  This 0-127
    //    @0: 2 * (1+ 0) * 10 * 8.3ns --> .1667us or 6.0MHz
    //  @127: 2 * (1+ 127) * 10 * 8.3ns --> 47kHz
    
    
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
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

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

volatile unsigned long FallingEdges = 0;
void ExternalButton_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;				// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
  FallingEdges = 0;             // (b) initialize counter
  GPIO_PORTJ_DIR_R &= ~0x02;    // (c) make PJ1 in 

  GPIO_PORTJ_DEN_R |= 0x02;     //     enable digital I/O on PJ1
	GPIO_PORTJ_PCTL_R &= ~0x000000FF; //  configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;	//   disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;			//	enable weak pull up resistor
  GPIO_PORTJ_IS_R &= ~0x02;     // (d) PJ1 is edge-sensitive 
  GPIO_PORTJ_IBE_R &= ~0x02;    //     PJ1 is not both edges 
  GPIO_PORTJ_IEV_R &= ~0x02;    //     PJ1 falling edge event 
  GPIO_PORTJ_ICR_R = 0x02;      // (e) clear flag1
  GPIO_PORTJ_IM_R |= 0x02;      // (f) arm interrupt on PJ1
  NVIC_PRI13_R = (NVIC_PRI13_R&0xFF00FFFF)|0x000A0000; // (g) priority 5
  NVIC_EN1_R |= 0x00080000;              // (h) enable interrupt 67 in NVIC
  EnableInt();           				// lets go
}


void GPIOJ_IRQHandler(void){
  GPIO_PORTJ_ICR_R = 0x02;      // acknowledge flag4
  FallingEdges = FallingEdges + 1; // Observe in Debug Watch Window

	//for(int j=0;j<40;j++){
				//64-steps equals to 45 degree because it is one eightth of 512
				//spin(64,0);
				//the D1 needs to blink after every 45-degree rotation
			//Blink();
		sensor();

			//}		
}


int main(void) {
	PLL_Init();
	SysTick_Init();
	PortH_Init();
	PortN0_Init();
	ExternalButton_Init();
//	SysTick_Wait10ms(100); // let peripherals boot
	
	while(1) {	
		WaitForInt();
	}
}