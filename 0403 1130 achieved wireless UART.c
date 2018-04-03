#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_timer.h"

#include "acc.h"
#include "joystick.h"
#include "oled.h"
#include "pca9532.h"
#include "rgb.h"
#include "light.h"
#include "temp.h"
#include "uart2.h"
#include "led7seg.h"

#define ACC_OFFSET 80
#define LIGHT_LOWER_LIMIT 0
#define LIGHT_UPPER_LIMIT 973
#define TEMP_HIGH_THRESHOLD 28
#define ACC_THRESHOLD 0.4
#define OBSTACLE_NEAR_THRESHOLD 600

typedef enum {
	Stationary,
	Launch,
	Return,
	Error
} State;

/*----------------------------------------------------------------------------
  Declare global variables
 *----------------------------------------------------------------------------*/
State currentMode = Stationary;
State nextMode = Stationary;
volatile uint32_t msTicks;
double xInitial = 0, yInitial = 0, zInitial = 0;
double x = 0, y = 0, z = 0;
uint8_t switch3WasPressed = 0, switch4OledUpdated = 0;
uint8_t rgbRed = 1, rgbBlue = 0;
uint8_t tempWarningState =0;
static char* msg = NULL;

/*----------------------------------------------------------------------------
  Time and interrupts
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void){
	msTicks++;
}
uint32_t getTicks(void){
	return msTicks;
}
void EINT0_IRQHandler(void) {
	if(currentMode==Stationary || tempWarningState==0){
		if(nextMode==Stationary || tempWarningState==0){
			nextMode=Launch;
		}
		if(nextMode==Error){
			nextMode==Stationary;
		}
	}
	if (currentMode==Launch) {
		nextMode= Return;
	}
	if (currentMode == Return){
		nextMode=Stationary;
	}
	if (tempWarningState==1){
		nextMode=Error;
	}
	eint_clearOutstandingInterruptsOf(0);
}
void eint_clearOutstandingInterruptsOf(uint8_t eintNumber) {
	LPC_SC->EXTINT |= 1<<eintNumber;
}
void eint_setExternalInterruptNumberModePolarity(uint8_t eintNumber, uint8_t mode, uint8_t polarity) {
	// Mode 0 for level sensitivity, 1 for edge sensitivity
	// Polarity 0 for (active low OR falling edge), 1 for (active high OR rising edge)
	if (mode && polarity) { // Both 1
		LPC_SC->EXTMODE  |= mode<<eintNumber;
		LPC_SC->EXTPOLAR |= polarity<<eintNumber;
	} else if (mode) { // Only mode is 1
		LPC_SC->EXTMODE  |= mode<<eintNumber;
		LPC_SC->EXTPOLAR &= polarity<<eintNumber;
	} else if (polarity) { // Only polarity is 1
		LPC_SC->EXTMODE  &= mode<<eintNumber;
		LPC_SC->EXTPOLAR |= polarity<<eintNumber;
	} else { // Both 0
		LPC_SC->EXTMODE  &= mode<<eintNumber;
		LPC_SC->EXTPOLAR &= polarity<<eintNumber;
	}
}

/*----------------------------------------------------------------------------
  Initialization API
 *----------------------------------------------------------------------------*/
void initializePortPinFunction(uint8_t port, uint8_t pin, uint8_t function) {
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = function;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = port;
	PinCfg.Pinnum = pin;
	PINSEL_ConfigPin(&PinCfg);
}
void initializeEverything() {
	initializeTimer();
	initializeInterrupts();
	initializeRgb();
	initializeOled();
	initializeAcc();
	initializeTemp();
	initializeLarr();
	initializeSs();
	initializeLight();
	initializeSwitch();
}
void initializeTimer() {
	SysTick_Config(SystemCoreClock / 1000);
}
void initializeInterrupts() {
	LPC_GPIOINT->IO2IntEnF |= 1<<10; // IO2IntEnF for Port 2 falling edge; 1<<10 for pin 10
	NVIC_EnableIRQ(EINT0_IRQn);
	eint_setExternalInterruptNumberModePolarity(0, 1, 0);
	eint_clearOutstandingInterruptsOf(0);
}
void initializeDefaultI2c() {
	I2C_Init(LPC_I2C2, 100000);
	I2C_Cmd(LPC_I2C2, ENABLE);
}
void initializeDefaultSsp() {
	SSP_CFG_Type SSP_ConfigStruct;
	SSP_ConfigStructInit(&SSP_ConfigStruct);
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);
	SSP_Cmd(LPC_SSP1, ENABLE);
}
void initializeOled() {
	initializePortPinFunction(0, 7, 2);
	initializePortPinFunction(0, 9, 2);
	initializeDefaultSsp();
	oled_init();
}
void initializeAcc() {
	initializePortPinFunction(0, 10, 2);
	initializePortPinFunction(0, 11, 2);
	initializeDefaultI2c();
	acc_init();
	acc_calibrate();
}
void initializeTemp() {
	initializePortPinFunction(0, 2, 0);
	GPIO_SetDir(0, 1<<2, 0);
	temp_init(getTicks);
}
void initializeRgb() {
	initializePortPinFunction(2, 0, 0); // Red
	initializePortPinFunction(0, 26, 0); // Blue
	GPIO_SetDir(2, 1<<0, 1); // Sets 2.1 RED to OUTPUT
	GPIO_SetDir(0, 1<<26, 1); // Sets 2.1 BLUE to OUTPUT;
}
void initializeLarr() {
	initializePortPinFunction(0, 10, 2);
	initializePortPinFunction(0, 11, 2);
	initializeDefaultI2c();
	pca9532_init();
}
void initializeSs() {
	initializePortPinFunction(0, 7, 2);
	initializePortPinFunction(0, 8, 2);
	initializePortPinFunction(0, 9, 2);
	initializePortPinFunction(1, 2, 0);
	initializeDefaultSsp();
	led7seg_init();
}
void initializeLight() {
	light_init();
	light_enable();
	initializePortPinFunction(0, 10, 2);
	initializePortPinFunction(0, 11, 2);
	initializeDefaultI2c();
}
void initializeSwitch() {
	initializePortPinFunction(2, 10, 1); // Switch 3 as EINT0
	initializePortPinFunction(1, 31, 0); // Switch 4 as GPIO
	GPIO_SetDir(1, 1<<31, 0);
}

/*----------------------------------------------------------------------------
  Temperature Sensor API
 *----------------------------------------------------------------------------*/
double temp_getTemperature() {
	uint32_t temperature = temp_read();
	if ((temperature/10.0)> TEMP_HIGH_THRESHOLD){
		tempWarningState =1;
	}
	return temperature / 10.0;
}

/*----------------------------------------------------------------------------
  Temperature Sensor API
 *----------------------------------------------------------------------------*/
double light_getLightValue() {
	uint32_t temperature = temp_read();
	return temperature / 10.0;
}

/*----------------------------------------------------------------------------
  Accelerometer API
 *----------------------------------------------------------------------------*/
void acc_calibrate() {
	int8_t xRaw, yRaw, zRaw;
	acc_read(&xRaw, &yRaw, &zRaw);
	xInitial = ((double)xRaw) / ACC_OFFSET;
	yInitial = ((double)yRaw) / ACC_OFFSET;
	zInitial = ((double)zRaw) / ACC_OFFSET;
	x = ((double)xRaw) / ACC_OFFSET - xInitial;
	y = ((double)yRaw) / ACC_OFFSET - yInitial;
}
void acc_UpdateXYZ() {
	int8_t xRaw, yRaw, zRaw;
	acc_read(&xRaw, &yRaw, &zRaw);
	//    x = (double)xRaw / ACC_OFFSET - xInitial;
	//    y = (double)yRaw / ACC_OFFSET - yInitial;
	x = ((double)xRaw) / ACC_OFFSET;
	y = ((double)yRaw) / ACC_OFFSET;
}

/*----------------------------------------------------------------------------
  RGB API
 *----------------------------------------------------------------------------*/
void rgb_setRedBlue(uint8_t red, uint8_t blue) {
	if (red) rgb_turnOnRedBlue(1, 0);
	else rgb_turnOffRedBlue(1, 0);

	if (blue) rgb_turnOnRedBlue(0, 1);
	else rgb_turnOffRedBlue(0, 1);
}
void rgb_turnOnRedBlue(uint8_t red, uint8_t blue) {
	if (red) GPIO_SetValue(2, 1<<0);
	if (blue) GPIO_SetValue(0, 1<<26);
}
void rgb_turnOffRedBlue(uint8_t red, uint8_t blue) {
	if (red) GPIO_ClearValue(2, 1<<0);
	if (blue) GPIO_ClearValue(0, 1<<26);
}

/*----------------------------------------------------------------------------
  Seven Segment Display API
 *----------------------------------------------------------------------------*/
void ss_showCharacter(uint8_t asciiCharacter) {
	led7seg_setChar(asciiCharacter, 0);
}
void ss_showDigit(uint8_t digit) {
	led7seg_setChar(digit, 1);
}

/*----------------------------------------------------------------------------
  LED Array API
 *----------------------------------------------------------------------------*/
void larr_setLeds(uint16_t ledOnMask) {
	pca9532_setLeds (ledOnMask, ~(ledOnMask));
}
void larr_showDistance() {
	//	uint32_t lightReading = light_read();
	//	pca9532_setLeds (0xff >> ((lightReading>>4)&0xff),0xffff);

	double proportionLit = (float)(light_read() - LIGHT_LOWER_LIMIT) / LIGHT_UPPER_LIMIT;
	uint8_t numberUnlit = 16 - proportionLit * 16;
	pca9532_setLeds(0xFFFF>>numberUnlit, ~(0xFFFF>>numberUnlit));
}

/*----------------------------------------------------------------------------
  OLED API
 *----------------------------------------------------------------------------*/
void oled_clear() {
	oled_clearScreen(OLED_COLOR_BLACK);
}
void oled_printStringAtLine(unsigned char* string, uint8_t line) {
	if (line > 6) {
		oled_putString(0, 6 * 9, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0, 6 * 9, "<line overflow>", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
	int yCoordinates = line * 9;
	oled_putString(0, yCoordinates, "               ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(0, yCoordinates, string, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}
void oled_printAccStringAtLine(uint8_t line) {
	acc_UpdateXYZ();
	char output[100];
	sprintf(output, "X,Y = %.1f, %.1f", x, y);
	oled_printStringAtLine(output, 2);
}
void oled_printAccString() {
	oled_printAccStringAtLine(2);
}
void oled_printLightStringAtLine(uint8_t line) {
	char output[50];
	sprintf (output, "Light %d", light_read());
	oled_printStringAtLine(output, line);
}
void oled_printLightString() {
	oled_printLightStringAtLine(6);
}
void oled_printTempStringAtLine(uint8_t line) {
	char output[50];
	sprintf (output, "Temp %.2f", temp_getTemperature());
	oled_printStringAtLine(output, line);
}
void oled_printTempString() {
	oled_printTempStringAtLine(1);
}

void tempWarning(){
	if (tempWarningState==1){
		rgb_setRedBlue(1,0);
	}
}

/*----------------------------------------------------------------------------
  Switches API
 *----------------------------------------------------------------------------*/
uint8_t switch_4isPressed() {
	return !((GPIO_ReadValue(1) >> 31) & 0x01);
}
void switch_3UpdateRgb() {
	if (!switch3WasPressed) return;
	rgbRed = !rgbRed;
	rgbBlue = !rgbBlue;
	rgb_setRedBlue(rgbRed, rgbBlue);
	switch3WasPressed = 0;
}
void switch_4UpdateOled() {
	if (switch_4isPressed()) {
		oled_printStringAtLine("SW4 pressed.", 3);
		switch4OledUpdated = 0;
	} else if (!switch4OledUpdated) {
		oled_printStringAtLine("SW4 unpressed.", 3);
		switch4OledUpdated = 1;
	}
}

/*----------------------------------------------------------------------------
   UART CODE
 *----------------------------------------------------------------------------*/
void pinsel_uart3(void){
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
}

void init_uart(void){
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;
	//pin select for uart3;
	pinsel_uart3();
	//supply power & setup working parameters for uart3
	UART_Init(LPC_UART3, &uartCfg);
	//enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);
}


/*----------------------------------------------------------------------------
  Control flow
 *----------------------------------------------------------------------------*/
void runStationaryMode() {
	if (currentMode != Stationary) {
		larr_setLeds(0x0000);
		oled_printStringAtLine("STATIONARY", 0);
		oled_printStringAtLine("", 2);
		currentMode=Stationary;
	}
	tempWarning();
	ss_showCharacter('F');
	rgb_setRedBlue(0, 0);
	oled_printTempString();
}

void runLaunchMode() {
	ss_showCharacter('0');
	rgb_setRedBlue(0, 0);
	// larr_
	if (currentMode != Launch) {
		larr_setLeds(0x0000);
		oled_printStringAtLine("LAUNCH", 0);
		oled_printStringAtLine("", 2);
		currentMode=Launch;
	}
	oled_printTempString();
	oled_printAccString(2);
}

void runReturnMode() {
	larr_showDistance();
	ss_showCharacter('0');
	if (currentMode != Return) {
		oled_printStringAtLine("RETURN", 0);
		oled_printStringAtLine("", 1);
		oled_printStringAtLine("", 2);
		currentMode=Return;
	}
}
int main(void) {
	initializeEverything();
	oled_clear();
	oled_printStringAtLine("STATIONARY", 0);

	uint8_t data = 0;
	uint32_t len = 0;
	uint8_t line[64];

	init_uart();
	//test sending message
	msg = "Welcome to EE2024 \r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
	//test receiving a letter and sending back to port
	UART_Receive(LPC_UART3, &data, 1, NONE_BLOCKING);
	UART_Send(LPC_UART3, &data, 1, NONE_BLOCKING);
	//test receiving message without knowing message length
	len = 0;
	do
	{   UART_Receive(LPC_UART3, &data, 1, NONE_BLOCKING);

	if (data != '\r')
	{
		len++;
		line[len-1] = data;
	}
	} while ((len<63) && (data != '\r'));
	line[len]=0;
	UART_SendString(LPC_UART3, &line);
	printf("--%s--\n", line);

	while (1) {
		if (currentMode== Stationary) {
			if (nextMode==Launch) {
				//countDown
				runLaunchMode();
			}
			else runStationaryMode();
		}
		if (currentMode == Launch) {
			if (nextMode == Return) runReturnMode();
			else runLaunchMode();
		}
		if (currentMode == Return) {
			if (nextMode == Stationary) runStationaryMode();
			else runReturnMode();
		}

	}
}
