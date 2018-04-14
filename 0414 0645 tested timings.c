#include "math.h"
#include "string.h"

#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#include "acc.h"
#include "led7seg.h"
#include "light.h"
#include "oled.h"
#include "pca9532.h"
#include "rgb.h"
#include "rotary.h"
#include "temp.h"
#include "uart2.h"

#define MS_BETWEEN_ACC_UPDATES		100
#define MS_BETWEEN_TEMP_UPDATES		400
#define MS_BETWEEN_UART_REPORTS		5000 // Default 10000
#define SECONDS_IN_COUNTDOWN		16
#define OLED_BLANK_LINE				"               "

#define TEMP_WARNING_THRESHOLD		30
#define TEMP_NUM_RECORDS			1000
#define ACC_WARNING_THRESHOLD		0.45
#define ACC_DIVISOR					80
#define LIGHT_RANGE_LOWER_LIMIT		0
#define LIGHT_RANGE_UPPER_LIMIT		3892 // 100 or 3892
#define OBSTACLE_WARNING_THRESHOLD	3000 // 50 or 3000

typedef enum {
	Stationary,
	Launch,
	Return
} Mode;

// Timing
volatile uint32_t msTicks;
// Warnings
volatile uint8_t warning_tempIsTooHigh = 0, warning_hasVeeredOffCourse = 0, warning_obstacleIsNear = 0, warning_isPendingAllClear = 0;
volatile uint8_t oled_hasTempWarning = 0, oled_hasAccWarning = 0, oled_hasObstacleWarning = 0;
// Sensors
volatile uint32_t temp_records[TEMP_NUM_RECORDS];
volatile uint32_t temp_latestRecordIndex = 0, temp_earliestRecordIndex = 1;
volatile double temp_inCelcius = 30.0, temp_onDisplay = 30.0;
volatile double x = 0, y = 0, xInitial = 0, yInitial = 0, xDisplayed = 0, yDisplayed = 0;
volatile uint32_t light_inLux = 0;
// States
volatile Mode currentMode = Return, nextMode = Stationary;
volatile uint8_t mode_isCountingDown = 0;
volatile uint32_t ss_triggerTime = 0, mode_lastPressTime = 0;
volatile uint8_t ss_nextCharacterIndex = 0, ss_currentCharacterIndex = 0;
volatile uint8_t larr_levelDisplayed = 0, larr_levelIntended = 0, larr_dutyCycleDisplayed = 60, larr_dutyCycleIntended = 60;
// UART
volatile uint8_t uart_isPendingTempWarning = 0, uart_isPendingAccWarning = 0, uart_isPendingLightWarning = 0;
volatile uint8_t uart_isPendingLightWarningNegation = 0, uart_isPendingManualReport = 0;


// Time and interrupts
uint32_t getTicks(){
	return msTicks;
}
void SysTick_Handler() {
	msTicks++;

	if (currentMode == Launch || currentMode == Return) {
		char inputCharacter = LPC_UART3->RBR;
		if (inputCharacter != 0) uart_updateInputHistory(inputCharacter);
	}
}
void EINT0_IRQHandler() {
	if (currentMode == Stationary && !mode_isCountingDown) {
		mode_isCountingDown = 1;
		ss_triggerTime = msTicks;
		ss_nextCharacterIndex = 0;
	} else if (currentMode == Launch) {
		if (msTicks - mode_lastPressTime < 1000) nextMode = Return;
		mode_lastPressTime = msTicks;
	} else if (currentMode == Return) {
		nextMode = Stationary;
	}
	LPC_SC->EXTINT |= 1;
}
void EINT3_IRQHandler() {
	if ((LPC_GPIOINT->IO0IntStatF>>25) & 0x1 || (LPC_GPIOINT->IO0IntStatF>>24) & 0x1) {
		LPC_GPIOINT->IO0IntClr |= 1<<25;
		LPC_GPIOINT->IO0IntClr |= 1<<24;
		uint8_t rotaryState = (GPIO_ReadValue(0) >> 24) & 0x03;
		if (larr_dutyCycleIntended > 0 && rotaryState == ROTARY_RIGHT) {
			larr_dutyCycleIntended -= 3;
		}
		else if (larr_dutyCycleIntended < 90 && rotaryState == ROTARY_LEFT) {
			larr_dutyCycleIntended += 3;
		}
	}
	if ((LPC_GPIOINT->IO0IntStatF>>2) & 0x1) {
		LPC_GPIOINT->IO0IntClr |= 1<<2;
		if (temp_earliestRecordIndex >= TEMP_NUM_RECORDS - 1) {
			temp_earliestRecordIndex = 0;
			temp_latestRecordIndex = TEMP_NUM_RECORDS - 1;
		} else if (temp_latestRecordIndex >= TEMP_NUM_RECORDS - 1) {
			temp_latestRecordIndex = 0;
			temp_earliestRecordIndex = 1;
		} else {
			temp_latestRecordIndex++;
			temp_earliestRecordIndex++;
		}
		temp_records[temp_latestRecordIndex] = msTicks;
	}
}

// Initialization API
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
	initializeUart();
	initializeInterrupts();
	initializeDefaultI2c();
	initializeDefaultSsp();
	initializeDefaultUart();
	initializeSwitches();
	initializeOled();
	initializeSs();
	initializeRgb();
	initializeLarr();
	initializeSensors();
}
void initializeTimer() {
	SysTick_Config(SystemCoreClock / 1000);
}
void initializeUart() {
	initializePortPinFunction(0, 0, 2);
	initializePortPinFunction(0, 1, 2);
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;
	UART_Init(LPC_UART3, &uartCfg);
	UART_TxCmd(LPC_UART3, ENABLE);
}
void initializeInterrupts() {
	LPC_GPIOINT->IO0IntClr = 0xFFFF;
	LPC_GPIOINT->IO2IntClr = 0xFFFF;
	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
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
void initializeDefaultUart() {
	UART_CFG_Type uartConfig;
	uartConfig.Baud_rate = 115200;
	uartConfig.Databits = UART_DATABIT_8;
	uartConfig.Parity = UART_PARITY_NONE;
	uartConfig.Stopbits = UART_STOPBIT_1;
	UART_Init(LPC_UART3, &uartConfig);
	UART_TxCmd(LPC_UART3, ENABLE);
}
void initializeSwitches() {
	// SW3 under EINT0
	initializePortPinFunction(2, 10, 1);
	LPC_SC->EXTMODE  |= 1<<0;
	LPC_SC->EXTPOLAR &= 0<<0;
	LPC_SC->EXTINT |= 1;

	// SW4 under polling policy
	initializePortPinFunction(1, 31, 0);
	GPIO_SetDir(1, 1<<31, 0);
}
void initializeOled() {
	initializePortPinFunction(0, 7, 2);
	initializePortPinFunction(0, 9, 2);
	oled_init();
}
void initializeSs() {
	initializePortPinFunction(0, 7, 2);
	initializePortPinFunction(0, 8, 2);
	initializePortPinFunction(0, 9, 2);
	initializePortPinFunction(1, 2, 0);
	led7seg_init();
}
void initializeRgb() {
	// Red
	initializePortPinFunction(2, 0, 0);
	GPIO_SetDir(2, 1<<0, 1);
	// Blue
	initializePortPinFunction(0, 26, 0);
	GPIO_SetDir(0, 1<<26, 1);
}
void initializeLarr() {
	initializePortPinFunction(0, 10, 2);
	initializePortPinFunction(0, 11, 2);
}
void initializeSensors() {
	// Rotary
	initializePortPinFunction(0, 24, 0);
	initializePortPinFunction(0, 25, 0);
	rotary_init();

	// Temp sensor
	initializePortPinFunction(0, 2, 0);
	temp_init(getTicks);

	// Accelerometer
	initializePortPinFunction(0, 10, 2);
	initializePortPinFunction(0, 11, 2);
	acc_init();
	acc_calibrate();

	// Light sensor
	light_enable();
	light_setRange(LIGHT_RANGE_4000);
	initializePortPinFunction(0, 10, 2);
	initializePortPinFunction(0, 11, 2);
}

// OLED
void oled_printTempString() {
	static uint32_t timeLastUpdated = 0;
	if (msTicks < MS_BETWEEN_TEMP_UPDATES) {
		oled_putString(0, 27, "Temp ...loading", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	} else {
		char string[50];
		sprintf (string, "Temp       %.1f", temp_inCelcius);
		oled_putString(0, 27, string, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		timeLastUpdated = msTicks;
	}
}
void oled_updateTempDisplayed() {
	if (temp_inCelcius == temp_onDisplay) return;
	oled_printTempString();
	temp_onDisplay = temp_inCelcius;
}
void oled_updateTempWarning() {
	if (warning_tempIsTooHigh && !oled_hasTempWarning) {
		oled_putString(0, 36, " Temp. too high", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
		oled_hasTempWarning = 1;
	} else if (!warning_tempIsTooHigh && oled_hasTempWarning) {
		oled_putString(0, 36, OLED_BLANK_LINE, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_hasTempWarning = 0;
	}
}
void oled_printAccString() {
	char string[100];
	sprintf(string, "X,Y  %4.1f, %4.1f", x, y);
	oled_putString(0, 45, string, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}
void oled_updateAccDisplayed() {
	if (x == xDisplayed && y == yDisplayed) return;
	oled_printAccString();
	xDisplayed = x;
	yDisplayed = y;
}
void oled_updateAccWarning() {
	if (warning_hasVeeredOffCourse && !oled_hasAccWarning) {
		oled_putString(0, 54, "Veer off course", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
		oled_hasAccWarning = 1;
	} else if (!warning_hasVeeredOffCourse && oled_hasAccWarning) {
		oled_putString(0, 54, OLED_BLANK_LINE, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_hasAccWarning = 0;
	}
}
void oled_updateLightWarning() {
	if (warning_obstacleIsNear && !oled_hasObstacleWarning) {
		oled_putString(0, 27, " Obstacle near ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
		oled_hasObstacleWarning = 1;
	} else if (!warning_obstacleIsNear && oled_hasObstacleWarning) {
		oled_putString(0, 27, OLED_BLANK_LINE, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_hasObstacleWarning = 0;
	}
}

// Seven Segment
void ss_showCharacter(uint8_t asciiCharacter) {
	led7seg_setChar(asciiCharacter, 0);
}
void ss_showDigit(uint8_t digit) {
	led7seg_setChar(digit, 1);
}
void ss_updateCharacter() {
	static char characters[16] = {'F','E','D','C','B','A','9','8','7','6','5','4','3','2','1','0'};

	if (mode_isCountingDown) {
		if (warning_tempIsTooHigh) {
			mode_isCountingDown = 0;
			ss_nextCharacterIndex = 0;
		} else if (msTicks - ss_triggerTime >= (ss_nextCharacterIndex + 1 * 1000)) {
			if (ss_nextCharacterIndex >= SECONDS_IN_COUNTDOWN - 1) {
				nextMode = Launch;
				mode_isCountingDown = 0;
			} else {
				ss_nextCharacterIndex++;
				ss_triggerTime = msTicks;
			}
		}
	}

	if (ss_currentCharacterIndex != ss_nextCharacterIndex) {
		ss_currentCharacterIndex = ss_nextCharacterIndex;
		char nextCharacter = characters[ss_currentCharacterIndex];
		ss_showCharacter(nextCharacter);
	}
}

// RGB
void rgb_setRedBlue(uint8_t red, uint8_t blue) {
	if (red) GPIO_SetValue(2, 1<<0);
	else GPIO_ClearValue(2, 1<<0);

	if (blue) GPIO_SetValue(0, 1<<26);
	else GPIO_ClearValue(0, 1<<26);
}
void rgb_setRed(uint8_t red) {
	if (red) GPIO_SetValue(2, 1<<0);
	else GPIO_ClearValue(2, 1<<0);
}
void rgb_setBlue(uint8_t blue) {
	if (blue) GPIO_SetValue(0, 1<<26);
	else GPIO_ClearValue(0, 1<<26);
}
void rgb_turnOnRedBlue(uint8_t red, uint8_t blue) {
	if (red) GPIO_SetValue(2, 1<<0);
	if (blue) GPIO_SetValue(0, 1<<26);
}
void rgb_turnOffRedBlue(uint8_t red, uint8_t blue) {
	if (red) GPIO_ClearValue(2, 1<<0);
	if (blue) GPIO_ClearValue(0, 1<<26);
}
void rgb_updateLights() {
	static uint32_t rgb_redLastChange = 0, rgb_blueLastChange = 0;
	static uint8_t rgb_redIsOn = 0, rgb_blueIsOn = 0;

	if (!warning_tempIsTooHigh && !warning_hasVeeredOffCourse) {
		if (rgb_redIsOn || rgb_blueIsOn) {
			rgb_redIsOn = 0;
			rgb_blueIsOn = 0;
			rgb_setRedBlue(rgb_redIsOn, rgb_blueIsOn);
		}
		return;
	}
	if (warning_tempIsTooHigh && warning_hasVeeredOffCourse) {
		if (rgb_redLastChange == rgb_blueLastChange && msTicks - rgb_blueLastChange > 333) {
			rgb_redIsOn = !rgb_redIsOn;
			rgb_redLastChange = msTicks;
			rgb_blueIsOn = !rgb_blueIsOn;
			rgb_blueLastChange = msTicks;
			rgb_setRedBlue(rgb_redIsOn, rgb_blueIsOn);
		}
		else if (rgb_redLastChange > rgb_blueLastChange) {
			rgb_redIsOn = 0; // Red changed more recently
			rgb_redLastChange = msTicks;
			rgb_blueIsOn = 1;
			rgb_blueLastChange = msTicks;
			rgb_setRedBlue(rgb_redIsOn, rgb_blueIsOn);
		} else if (rgb_blueLastChange > rgb_redLastChange) {
			rgb_redIsOn = 1;
			rgb_redLastChange = msTicks;
			rgb_blueIsOn = 0; // Blue changed more recently
			rgb_blueLastChange = msTicks;
			rgb_setRedBlue(rgb_redIsOn, rgb_blueIsOn);
		}
	} else if (warning_tempIsTooHigh && msTicks - rgb_redLastChange > 333) {
		rgb_redIsOn = !rgb_redIsOn;
		rgb_redLastChange = msTicks;
		rgb_setRed(rgb_redIsOn);
	} else if (warning_hasVeeredOffCourse && msTicks - rgb_blueLastChange > 333) {
		rgb_blueIsOn = !rgb_blueIsOn;
		rgb_blueLastChange = msTicks;
		rgb_setBlue(rgb_blueIsOn);
	}
}

// LED Array
void larr_setLeds(uint16_t ledOnMask) {
	pca9532_setBlink0Leds(ledOnMask);
}
void larr_updateLeds() {
	if (larr_levelIntended != larr_levelDisplayed) {
		uint16_t ledOnMask = 0xFFFF;
		int16_t greenNumberOn = larr_levelIntended - 8;
		if (greenNumberOn > 0) {
			uint16_t greenOnMask = 0xFF00<<(8 - greenNumberOn);
			ledOnMask = greenOnMask | 0x00FF;
		} else {
			ledOnMask = 0xFFFF>>(16 - larr_levelIntended);
		}
		larr_setLeds(ledOnMask);
		larr_levelDisplayed = larr_levelIntended;
	} else if (larr_dutyCycleDisplayed != larr_dutyCycleIntended) {
		pca9532_setBlink0Duty(larr_dutyCycleIntended);
		larr_dutyCycleDisplayed = larr_dutyCycleIntended;

		uint16_t ledOnMask = 0xFFFF;
		int16_t greenNumberOn = larr_levelIntended - 8;
		if (greenNumberOn > 0) {
			uint16_t greenOnMask = 0xFF00<<(8 - greenNumberOn);
			ledOnMask = greenOnMask | 0x00FF;
		} else {
			ledOnMask = 0xFFFF>>(16 - larr_levelIntended);
		}
		larr_setLeds(ledOnMask);
	}
}

// Interrupt-driven Peripherals
void rotary_setInterrupt(uint8_t isPendingEnable) {
	if (isPendingEnable) {
		LPC_GPIOINT->IO0IntClr |= 1<<24;
		LPC_GPIOINT->IO0IntEnF |= 1<<24;
		LPC_GPIOINT->IO0IntClr |= 1<<25;
		LPC_GPIOINT->IO0IntEnF |= 1<<25;
	} else {
		LPC_GPIOINT->IO0IntEnF &= (~(1<<24));
		LPC_GPIOINT->IO0IntEnF &= (~(1<<25));
	}
}
void temp_setInterrupt(uint8_t isPendingEnable) {
	if (isPendingEnable) {
		LPC_GPIOINT->IO0IntClr |= 1<<2;
		LPC_GPIOINT->IO0IntEnF |= 1<<2;
	} else {
		LPC_GPIOINT->IO0IntEnF &= (~(1<<2));
	}
}
double temp_getInCelcius() {
	static uint32_t scaleFactor = 1000 / TEMP_NUM_RECORDS;
	uint32_t latestRecord = temp_records[temp_latestRecordIndex];
	uint32_t earliestRecord = temp_records[temp_earliestRecordIndex];
	uint32_t timeDifference = latestRecord - earliestRecord;
	double result = (scaleFactor * timeDifference - 2731)  / 10.0;
	if (result > 60 || result < 20 ) return temp_inCelcius;
	else return result;
}
void temp_updateTemperature() {
	static uint32_t timeLastUpdated = MS_BETWEEN_TEMP_UPDATES;
	if (msTicks - timeLastUpdated < MS_BETWEEN_TEMP_UPDATES) return;

	temp_inCelcius = temp_getInCelcius();
	if (temp_inCelcius > TEMP_WARNING_THRESHOLD && !warning_tempIsTooHigh) {
		uart_isPendingTempWarning = 1;
		warning_tempIsTooHigh = 1;
	}
	timeLastUpdated = msTicks;
}
void acc_calibrate() {
	int8_t xRaw, yRaw, zRaw;
	acc_read(&xRaw, &yRaw, &zRaw);
	xInitial = ((double)xRaw) / ACC_DIVISOR;
	yInitial = ((double)yRaw) / ACC_DIVISOR;
	x = ((double)xRaw) / ACC_DIVISOR - xInitial;
	if (x < 0 && x > -0.05) x = 0;
	y = ((double)yRaw) / ACC_DIVISOR - yInitial;
	if (y < 0 && y > -0.05) y = 0;
}
void acc_updateXYZ() {
	static uint32_t timeLastUpdated = 0;
	if (msTicks - timeLastUpdated < MS_BETWEEN_ACC_UPDATES) return;

	int8_t xRaw, yRaw, zRaw;
	acc_read(&xRaw, &yRaw, &zRaw);

	x = ((double)xRaw) / ACC_DIVISOR - xInitial;
	if (x < 0 && x > -0.05) x = 0;

	y = ((double)yRaw) / ACC_DIVISOR - yInitial;
	if (y < 0 && y > -0.05) y = 0;

	if ((fabs(x) >= ACC_WARNING_THRESHOLD || fabs(y) >= ACC_WARNING_THRESHOLD) && !warning_hasVeeredOffCourse) {
		uart_isPendingAccWarning = 1;
		warning_hasVeeredOffCourse = 1;
	}
	timeLastUpdated = msTicks;
}
void light_updateLux() {
	light_inLux = light_read();
	if (light_inLux < OBSTACLE_WARNING_THRESHOLD) {
		if (!warning_obstacleIsNear) {
			uart_isPendingLightWarning = 1;
			warning_obstacleIsNear = 1;
		}
	} else if (warning_obstacleIsNear) {
		uart_isPendingLightWarningNegation = 1;
		warning_obstacleIsNear = 0;
	}
}
void light_updateToBeLit() {
	light_updateLux();
	double proportionLit = (double)(light_inLux - LIGHT_RANGE_LOWER_LIMIT) / LIGHT_RANGE_UPPER_LIMIT;
	if (proportionLit >= 0.96875) larr_levelIntended = 16;
	else if (proportionLit <= 0.03125) larr_levelIntended = 0;
	else larr_levelIntended = 16 * proportionLit;
}

// UART
void uart_sendMessage(char* message) {
	UART_Send(LPC_UART3, (uint32_t *)message , strlen(message), BLOCKING);
}
void uart_sendLaunchModeReport() {
	char report[100];
	sprintf(report, " Temp   %.1f C     ACC   X: %5.2f,  Y: %5.2f \r\n", temp_inCelcius, x, y);
	uart_sendMessage(report);
}
void uart_sendReturnModeReport() {
	char report[100];
	sprintf(report, " Obstacle distance : %d \r\n", light_inLux);
	uart_sendMessage(report);
}
void uart_sendTempWarning() {
	uart_sendMessage("   WARNING: TEMPERATURE TOO HIGH \r\n");
}
void uart_sendAccWarning() {
	uart_sendMessage("   WARNING: VEERED OFF COURSE \r\n");
}
void uart_sendLightWarning() {
	uart_sendMessage("   WARNING: OBSTACLE NEAR \r\n");
}
void uart_sendLightWarningNegation() {
	uart_sendMessage("   OBSTACLE AVOIDED \r\n");
}
void uart_updateInputHistory(char character) {
	static char history[4] = "NULL";
	static int8_t index = 0;

	if (character == '\b' && index > 0) {
		index--;
	} else if (character == 'R' && index == 0) {
		history[index] = character;
		index++;
	} else if (character == 'P' && index == 1) {
		history[index] = character;
		index++;
	} else if (character == 'T' && index == 2) {
		history[index] = character;
		index++;
	} else if (character == '\r' && index == 3) {
		uart_isPendingManualReport = 1;
		index = 0;
	} else {
		index = 0;
	}
}

// Control Flow
void clearOutdatedWarnings() {
	if (!((LPC_GPIO1->FIOPIN) >> 31 & 0x01)) {
		if (temp_inCelcius <= TEMP_WARNING_THRESHOLD) warning_tempIsTooHigh = 0;
		if (fabs(x) < ACC_WARNING_THRESHOLD && fabs(y) < ACC_WARNING_THRESHOLD) warning_hasVeeredOffCourse = 0;
		if (light_inLux >= OBSTACLE_WARNING_THRESHOLD) warning_obstacleIsNear = 0;
	}
}
void configureForStationaryMode() {
	if (currentMode == Stationary) return;

	temp_setInterrupt(1);
	rotary_setInterrupt(0);

	oled_clearScreen(OLED_COLOR_BLACK);
	oled_hasObstacleWarning = 0;
	warning_obstacleIsNear = 0;

	oled_putString(4, 9, "  STATIONARY   ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_printTempString();

	ss_showCharacter('F');
	larr_setLeds(0x0000);
	larr_levelDisplayed = 0;

	uart_sendMessage("_________ Entered Stationary mode __________ \r\n");
	uart_isPendingManualReport = 0;
	uart_isPendingLightWarning = 0;
	uart_isPendingLightWarningNegation = 0;

	currentMode = Stationary;
}
void runStationaryMode() {
	rgb_updateLights();
	oled_updateTempDisplayed();
	oled_updateTempWarning();
	temp_updateTemperature();
	ss_updateCharacter();
}
void configureForLaunchMode() {
	if (currentMode == Launch) return;

	oled_putString(4, 9, "    LAUNCH     ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_printAccString();
	uart_sendMessage("___________ Entered Launch mode ____________ \r\n");

	uart_isPendingManualReport = 0;
	uart_isPendingTempWarning = 0;
	uart_isPendingAccWarning = 0;

	currentMode = Launch;
}
void runLaunchMode() {
	rgb_updateLights();
	oled_updateTempDisplayed();
	oled_updateTempWarning();
	oled_updateAccDisplayed();
	oled_updateAccWarning();
	temp_updateTemperature();
	acc_updateXYZ();

	static uint32_t uart_lastReportTime = 0;
	uint8_t uart_isPendingAutoReport = (msTicks - uart_lastReportTime >= MS_BETWEEN_UART_REPORTS);
	if (uart_isPendingAutoReport) {
		uart_sendLaunchModeReport();
		uart_lastReportTime = msTicks;
	}
	if (uart_isPendingManualReport) {
		uart_sendLaunchModeReport();
		uart_isPendingManualReport = 0;
	}
	if (uart_isPendingTempWarning) {
		uart_sendTempWarning();
		uart_isPendingTempWarning = 0;
	}
	if (uart_isPendingAccWarning) {
		uart_sendAccWarning();
		uart_isPendingAccWarning = 0;
	}
}
void configureForReturnMode() {
	if (currentMode == Return) return;

	temp_setInterrupt(0);
	rotary_setInterrupt(1);

	oled_clearScreen(OLED_COLOR_BLACK);
	warning_tempIsTooHigh = 0;
	warning_hasVeeredOffCourse = 0;
	rgb_updateLights();
	oled_updateTempWarning();
	oled_updateAccWarning();
	oled_putString(4, 9, "    RETURN     ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	uart_sendMessage("___________ Entered Return mode ____________ \r\n");
	uart_isPendingManualReport = 0;

	currentMode = Return;
}
void runReturnMode() {
	light_updateToBeLit();
	larr_updateLeds();
	oled_updateLightWarning();

	static uint32_t uart_lastReportTime = 0;
	uint8_t uart_isPendingAutoReport = (msTicks - uart_lastReportTime >= MS_BETWEEN_UART_REPORTS);
	if (uart_isPendingAutoReport) {
		uart_sendReturnModeReport();
		uart_lastReportTime = msTicks;
	}
	if (uart_isPendingManualReport) {
		uart_sendReturnModeReport();
		uart_isPendingManualReport = 0;
	}
	if (uart_isPendingLightWarning) {
		uart_sendLightWarning();
		uart_isPendingLightWarning = 0;
	}
	if (uart_isPendingLightWarningNegation) {
		uart_sendLightWarningNegation();
		uart_isPendingLightWarningNegation = 0;
	}
}

int main(void) {
	initializeEverything();
	configureForStationaryMode();

	while (1) {
		clearOutdatedWarnings();

		if (nextMode == Stationary) {
			configureForStationaryMode();
			runStationaryMode();
		} else if (nextMode == Launch) {
			configureForLaunchMode();
			runLaunchMode();
		} else if (nextMode == Return) {
			configureForReturnMode();
			runReturnMode();
		}
	}
}
