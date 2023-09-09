/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "lcd.h"
#include "eeprom.h"
#include "logowika.c"
#include "ST7735.h"
#include "lps25hb.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//=================DEFINITIONS=================

//=================LPS25HB=================
#define LPS25HB_ADDR       		0xBA
#define LPS25HB_WHO_AM_I 		0x0F
#define LPS25HB_CTRL_REG1 		0x20
#define LPS25HB_CTRL_REG1_PD 	0x80
#define LPS25HB_CTRL_REG1_ODR2 	0x40
#define LPS25HB_CTRL_REG1_ODR1 	0x20
#define LPS25HB_CTRL_REG1_ODR0 	0x10
#define LPS25HB_TEMP_OUT_L 		0x2B

//=================PRESSURE_LEDS=================
// Safety LED
#define SAFETY_LED_GPIO_PORT    GPIOA
#define SAFETY_LED_GPIO_PIN     GPIO_PIN_5

// Warning LED
#define WARNING_LED_GPIO_PORT   GPIOA
#define WARNING_LED_GPIO_PIN    GPIO_PIN_6

// Dangerous LED
#define DANGEROUS_LED_GPIO_PORT GPIOA
#define DANGEROUS_LED_GPIO_PIN  GPIO_PIN_7


//=================BUTTON_SATES=================
#define BUTTON_STATE_NONE  0
#define BUTTON_STATE_SHORT 1
#define BUTTON_STATE_LONG  2

//=================DISPLAY=================
#define DISPLAY_WIDTH 160
#define DISPLAY_HEIGHT 128

//==================================

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//=================VARIABLES=================
float p0;

uint8_t units = 0;
uint8_t unithPA = 0;
uint8_t WriteToEepromFlag = 0;

typedef enum {
	STATE_LOGO,
	STATE_TEMPERATURE_C,
	STATE_PRESSURE_HPA,
	STATE_HEIGHT,
	STATE_ADC,
} MenuState;

MenuState currentMenuState = STATE_LOGO;


uint8_t PressState = BUTTON_STATE_NONE;
uint32_t pressStartTime = 0;
uint32_t buttonPressDuration = 0;
volatile uint8_t buttonState = GPIO_PIN_RESET;
const uint32_t longPressDuration = 500;

char tempString[10];
char tempFString[15];
char pressureString[10];
char HeightString[10];
char voltagestring[10];
float voltage;

//=================FUNCTIONS================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) {
		// Read the current button state
		buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

		// Handle the button press and release events
		if (buttonState == GPIO_PIN_RESET) {
			// Button is pressed (MIGHT DELETE IT, BUT USEFULL FOR BUTTON TESTING)
			char messagePressed[] = "Button Pressed!\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t*)messagePressed, strlen(messagePressed), HAL_MAX_DELAY);

			//			 Record the start time of the button press
			pressStartTime = HAL_GetTick();
		} else {
			//			 Button is released (MIGHT DELETE IT, BUT USEFULL FOR BUTTON TESTING)
			char messageReleased[] = "Button Released!\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t*)messageReleased, strlen(messageReleased), HAL_MAX_DELAY);

			// Calculate the button press duration
			buttonPressDuration = HAL_GetTick() - pressStartTime;

			// Check if it's a short press or a long press
			if (buttonPressDuration < longPressDuration) {
				PressState = BUTTON_STATE_SHORT;
			} else {
				PressState = BUTTON_STATE_LONG;
			}
		}
	}
}

void WriteToEeprom() {
	if (eeprom_write(0x01, &units, sizeof(units)) != HAL_OK )
		units = 0;


	if (eeprom_write(0x02, &unithPA, sizeof(unithPA)) != HAL_OK )
		unithPA = 0;
}

void toggleUnitSelectionMode() {
	if (currentMenuState == STATE_TEMPERATURE_C) {
		if (PressState == BUTTON_STATE_LONG && units == 0) {
			units = 1;
			PressState = BUTTON_STATE_NONE;
			HAL_Delay(200);
			WriteToEeprom();
		} else if (PressState == BUTTON_STATE_LONG && units == 1) {
			units = 0;
			PressState = BUTTON_STATE_NONE;
			HAL_Delay(200);
			WriteToEeprom();
		}
	}
	if (currentMenuState == STATE_PRESSURE_HPA) {
		if (PressState == BUTTON_STATE_LONG && unithPA == 0) {
			unithPA = 1;
			PressState = BUTTON_STATE_NONE;
			HAL_Delay(200);
			WriteToEeprom();
		} else if (PressState == BUTTON_STATE_LONG && unithPA == 1) {
			unithPA = 0;
			PressState = BUTTON_STATE_NONE;
			HAL_Delay(200);
			WriteToEeprom();
		}
	}

}

void updatePressureLEDs(float pressure)
{
	// Clear all LEDs
	HAL_GPIO_WritePin(SAFETY_LED_GPIO_PORT, SAFETY_LED_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(WARNING_LED_GPIO_PORT, WARNING_LED_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DANGEROUS_LED_GPIO_PORT, DANGEROUS_LED_GPIO_PIN, GPIO_PIN_RESET);

	// Treshold
	float DANGEROUS_PRESSURE_THRESHOLD = 1025;
	float WARNING_PRESSURE_THRESHOLD = 1015;
	float SAFETY_PRESSURE_THRESHOLD = 950;

	// Check pressure levels and set LEDs accordingly
	if (pressure >= DANGEROUS_PRESSURE_THRESHOLD) {
		HAL_GPIO_WritePin(DANGEROUS_LED_GPIO_PORT, DANGEROUS_LED_GPIO_PIN, GPIO_PIN_SET);
	} else if (pressure >= WARNING_PRESSURE_THRESHOLD) {
		HAL_GPIO_WritePin(WARNING_LED_GPIO_PORT, WARNING_LED_GPIO_PIN, GPIO_PIN_SET);
	} else if (pressure >= SAFETY_PRESSURE_THRESHOLD) {
		HAL_GPIO_WritePin(SAFETY_LED_GPIO_PORT, SAFETY_LED_GPIO_PIN, GPIO_PIN_SET);
	}
}

//=================UART=================
uint8_t uart_rx_buffer;

int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}

#define LINE_MAX_LENGTH 80

static char line_buffer[LINE_MAX_LENGTH + 1];
static uint32_t line_length;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) {
    line_append(uart_rx_buffer);
    HAL_UART_Receive_IT(&huart2, &uart_rx_buffer, 1);
  }
}

void line_append(uint8_t value)
{
    static const struct {
        const char *command;
        int state;
    } commands[] = {
        {"logo", STATE_LOGO},
        {"temp", STATE_TEMPERATURE_C},
        {"pressure", STATE_PRESSURE_HPA},
        {"height", STATE_HEIGHT},
        {"adc", STATE_ADC},
    };
    static const int NUM_COMMANDS = sizeof(commands) / sizeof(commands[0]);

    if (value == '\r' || value == '\n') {
        ST7735_ClearDisplay(WHITE);

        if (line_length > 0) {
            line_buffer[line_length] = '\0';
            printf("Received: %s\n", line_buffer);

            for (int i = 0; i < NUM_COMMANDS; ++i) {
                if (strcmp(line_buffer, commands[i].command) == 0) {
                    currentMenuState = commands[i].state;
                    break;
                }
            }

            if (strcmp(line_buffer, "GetUnits") == 0) {
                printf("Units and their values are:\n");
                printf("temperature: %.1f *C\n", lps25hb_read_temp());
                printf("pressure: %.1f hPa\n", lps25hb_read_pressure());
                printf("height: %.2f m\n", lps25hb_read_height());
                printf("voltage: %.3f V\n", voltage);
            } else if (strcmp(line_buffer, "SetTempC") == 0) {
                units = 0;
            } else if (strcmp(line_buffer, "SetTempF") == 0) {
                units = 1;
            } else if (strcmp(line_buffer, "WriteToEeprom") == 0) {
                WriteToEepromFlag = 1;
            }

            line_length = 0;
        }
    } else {
        if (line_length >= LINE_MAX_LENGTH) {
            line_length = 0;
        }
        line_buffer[line_length++] = value;
    }
}

void lps_write_reg(uint8_t reg, uint8_t valuelps)
{
	HAL_I2C_Mem_Write(&hi2c1, LPS25HB_ADDR, reg, 1, &valuelps, sizeof(valuelps), HAL_MAX_DELAY);
}
uint8_t lps_read_reg(uint8_t reg)
{
	uint8_t valuelps = 0;
	HAL_I2C_Mem_Read(&hi2c1, LPS25HB_ADDR, reg, 1, &valuelps, sizeof(valuelps), HAL_MAX_DELAY);

	return valuelps;
}

//==================================

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

//=================MAIN=================
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

//=================EEPROM_READ=================
	if (eeprom_read(0x01, &units, sizeof(units)) != HAL_OK){
		Error_Handler();
	}
	if (eeprom_read(0x02, &unithPA, sizeof(unithPA)) != HAL_OK){
		unithPA = 0;
	}
	HAL_UART_Receive_IT(&huart2, &uart_rx_buffer, 1);

//=================DISPLAY_INITIALIZATION=================
	int16_t textWidth = strlen("Temperature: ") * Font_11x18.width;
	int16_t centerX = (DISPLAY_WIDTH - textWidth) / 2;
	int16_t centerY = (DISPLAY_HEIGHT - (Font_11x18.height * 2)) / 2;
	ST7735_Init(1);
	lcd_init();

//=================SENSOR=================
	lps25hb_set_calib(96);
	printf("Searching...\n");
	uint8_t who_am_i = lps_read_reg(LPS25HB_WHO_AM_I);

	if (who_am_i == 0xBD) {
		printf("Found: LPS25HB\n");
		lps_write_reg(LPS25HB_CTRL_REG1,  0xC0);
		HAL_Delay(100);

		int16_t temp;
		HAL_I2C_Mem_Read(&hi2c1, LPS25HB_ADDR, LPS25HB_TEMP_OUT_L | 0x80, 1, (uint8_t*)&temp, sizeof(temp), HAL_MAX_DELAY);
		printf("T = %d\n", temp);
	} else {
		printf("Error: (0x%02X)\n", who_am_i);
	}
	p0 = lps25hb_read_pressure();

//=================SENSOR=================
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint32_t adcval = HAL_ADC_GetValue(&hadc1);
	voltage = 3.3f * adcval  /4096.0f;
	HAL_Delay(1000);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//=================WHILE_LOOP=================

	while (1)
	{

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//=================CONVERTING_DATA_INTO_STRINGS=================
		sprintf(tempString, "%.1f*C", lps25hb_read_temp());;
		float tempC = lps25hb_read_temp();
		float temperatureF = (tempC * 9.0 / 5.0) + 32.0;
		sprintf(pressureString, "%.1f hPa", lps25hb_read_pressure());
		char pressureStringPA[15];
		float pressure_hPa = lps25hb_read_pressure();
		float pressure_PA = pressure_hPa * 100.0;
		sprintf(pressureStringPA, "%.1f Pa", pressure_PA);
		sprintf(HeightString, "%.2f m", lps25hb_read_height());
		sprintf(voltagestring, "%.3f V", voltage);
		ST7735_SetRotation(1);

		//=================PRESSURE_LED_UPDATE=================
		float pressure = lps25hb_read_pressure();
		updatePressureLEDs(pressure);

		//=================EEPROM_FLAG=================
		if (WriteToEepromFlag == 1) {
			HAL_Delay(200);
			WriteToEeprom();
			HAL_Delay(200);
			WriteToEepromFlag = 0;
		}

		//=================BUTTON_HANDLING=================\\

		//=================SHORT_PRESS=================
		if (PressState != BUTTON_STATE_NONE) {
			if (PressState == BUTTON_STATE_SHORT) {
				ST7735_ClearDisplay(WHITE);
				currentMenuState = (currentMenuState + 1) % 5; // Change the menu state to the next state
				PressState = BUTTON_STATE_NONE;
			}

			//=================LONG_PRESS=================
			else if (PressState == BUTTON_STATE_LONG) {
				PressState = BUTTON_STATE_LONG;
				toggleUnitSelectionMode();
				HAL_Delay(200); // Might delete but i think its okey
				ST7735_ClearDisplay(WHITE);
			}
		}

		//=================MENU_CASE=================
		switch (currentMenuState) {
		case STATE_LOGO:
			lcd_draw_image(0, 0, 160, 128, logowika);
			break;
		case STATE_TEMPERATURE_C:
			sprintf(tempString, "%.1f*C", lps25hb_read_temp());
			sprintf(tempFString, "%.1f*F", temperatureF);

			ST7735_WriteString(centerX, centerY, "Temperature: ", Font_11x18, BLACK, WHITE);
			if (units == 0) {
				ST7735_WriteString(centerX, centerY + Font_11x18.height, tempString, Font_11x18, BLACK, WHITE);
			}
			// TEMPERATURE IN F
			else if (units == 1) {
				ST7735_WriteString(centerX, centerY + Font_11x18.height, tempFString, Font_11x18, BLACK, WHITE);
			}

			break;
		case STATE_PRESSURE_HPA:
			sprintf(pressureString, "%.1f hPa", lps25hb_read_pressure());
			sprintf(pressureStringPA, "%.1f Pa", pressure_PA);

			ST7735_WriteString(centerX, centerY, "Pressure: ", Font_11x18, BLACK, WHITE);
			if (unithPA == 0) {
				ST7735_WriteString(centerX, centerY + Font_11x18.height, pressureString, Font_11x18, BLACK, WHITE);
			}
			else if (unithPA == 1) {
				ST7735_WriteString(centerX, centerY + Font_11x18.height, pressureStringPA, Font_11x18, BLACK, WHITE);
			}
			break;
		case STATE_HEIGHT:
			sprintf(HeightString, "%.2f m", lps25hb_read_height());

			ST7735_WriteString(centerX, centerY, "Height: ", Font_11x18, BLACK, WHITE);
			ST7735_WriteString(centerX, centerY + Font_11x18.height, HeightString, Font_11x18, BLACK, WHITE);
			break;
		case STATE_ADC:
			sprintf(voltagestring, "%.3f V", voltage);

			ST7735_WriteString(centerX, centerY, "ADC: ", Font_11x18, BLACK, WHITE);
			ST7735_WriteString(centerX, centerY + Font_11x18.height, voltagestring, Font_11x18, BLACK, WHITE);
			break;
		default:
			break;
		}
		HAL_Delay(500);
	}
  /* USER CODE END 3 */
}

//==================================

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
