/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Panel button possible states:
typedef enum {
  UNPRESSED = 0,
  PRESSED
} panel_button_state_t;

//Heater load states:
typedef enum {
  LOAD_TOO_SMALL = 0,
  LOAD_OK
} heater_load_state_t;
/* USER CODE END PTD */

//Load switch states:
typedef enum {
  LOAD_OFF = 0,
  LOAD_ON
} load_switch_state_t;

//error flag states:
typedef enum {
  NO_ERROR = 0,
  ERROR_PRESENT
}error_flag_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PANEL_BUTTON_DEBOUNCE_DELAY_MS          50

#define PANEL_LED_BLINK_INTERVAL_MS             1000 //1 sec

#define PANEL_ERROR_LED_BLINK_INTERVAL_MS       200 //1 sec

#define ADC_SAMPLE_NUMBER                       20

#define ADC_SAMPLE_PERIOD_MS                    3

#define ADC_CONV_TIMEOUT_MS                     1000 //1 sec

#define ADC_REF_VOLT_MV                         3300 //3.3V

//TODO this value should be calibrated when no load:
#define ADC_ZERO_CURR_VOLT_MV                   1652 //1.652V

#define CURR_SENS_SENSITIVITY_MV                185 //185 mV/A

#define LOAD_CURR_THRESHOLD_MA                  150 //mA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

/* USER CODE BEGIN PV */

static error_flag_state_t _error_flag = NO_ERROR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
static panel_button_state_t getPanelButtonState(void);
static void setLoadSwitchState(load_switch_state_t load_switch_state);
static load_switch_state_t getLoadSwitchState(void);
static int32_t convToCurrent(int32_t adc_value);
static heater_load_state_t check_load(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  panel_button_state_t _panel_but_current_state = UNPRESSED;
  
  heater_load_state_t _heater_load_state = LOAD_OK;
  
  uint32_t _led_state_change_time = 0;
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
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    _panel_but_current_state = getPanelButtonState();
    
    if (_panel_but_current_state == PRESSED){
      if (getLoadSwitchState() == LOAD_OFF){ //switch load on if not yet switched
        setLoadSwitchState(LOAD_ON);
      }
      
      //check load:
      _heater_load_state = check_load();
      
    }
    else{
      if (getLoadSwitchState() == LOAD_ON){ //switch load off if not yet switched
        setLoadSwitchState(LOAD_OFF);
      }
    }
    
    //panel LED logic:
    if (_error_flag == ERROR_PRESENT){
      if ((HAL_GetTick() - _led_state_change_time) > PANEL_ERROR_LED_BLINK_INTERVAL_MS){
        _led_state_change_time = HAL_GetTick();
        HAL_GPIO_TogglePin(PANEL_LED_GPIO_Port, PANEL_LED_Pin);
      }
    }
    else{
      if (_panel_but_current_state == PRESSED) {
        if (_heater_load_state == LOAD_OK){ //load ok, led on
          HAL_GPIO_WritePin(PANEL_LED_GPIO_Port, PANEL_LED_Pin, GPIO_PIN_SET);
        }
        else{ //led not ok, led blinking
          if ((HAL_GetTick() - _led_state_change_time) > PANEL_LED_BLINK_INTERVAL_MS){
            _led_state_change_time = HAL_GetTick();
            HAL_GPIO_TogglePin(PANEL_LED_GPIO_Port, PANEL_LED_Pin);
          }
        }
      }
      else {
        HAL_GPIO_WritePin(PANEL_LED_GPIO_Port, PANEL_LED_Pin, GPIO_PIN_RESET);
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_CNTRL_GPIO_Port, RELAY_CNTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PANEL_LED_GPIO_Port, PANEL_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RELAY_CNTRL_Pin */
  GPIO_InitStruct.Pin = RELAY_CNTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_CNTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PANEL_BUT_Pin */
  GPIO_InitStruct.Pin = PANEL_BUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PANEL_BUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PANEL_LED_Pin */
  GPIO_InitStruct.Pin = PANEL_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PANEL_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//get current panel button state:
static panel_button_state_t getPanelButtonState(void)
{
  GPIO_PinState _but_state_0 = HAL_GPIO_ReadPin(PANEL_BUT_GPIO_Port, PANEL_BUT_Pin);
  
  //to overcome some instabilities of button state:
  uint32_t _sample_uptime = HAL_GetTick();
  while ((HAL_GetTick() - _sample_uptime) < PANEL_BUTTON_DEBOUNCE_DELAY_MS)
    ;
  
  //if states not equal resample again:
  if (_but_state_0 != HAL_GPIO_ReadPin(PANEL_BUT_GPIO_Port, PANEL_BUT_Pin)){
    _sample_uptime = HAL_GetTick();
    while ((HAL_GetTick() - _sample_uptime) < PANEL_BUTTON_DEBOUNCE_DELAY_MS)
      ;
    
    _but_state_0 = HAL_GPIO_ReadPin(PANEL_BUT_GPIO_Port, PANEL_BUT_Pin);
  }
  
  //convert to actual button state:
  if (_but_state_0 == GPIO_PIN_RESET){
    return PRESSED;
  }
  else{
    return UNPRESSED;
  }
}

//load switching relay control:
static void setLoadSwitchState(load_switch_state_t load_switch_state)
{
  if (load_switch_state == LOAD_ON){
    HAL_GPIO_WritePin(RELAY_CNTRL_GPIO_Port, RELAY_CNTRL_Pin, GPIO_PIN_SET);
  }
  else{
    HAL_GPIO_WritePin(RELAY_CNTRL_GPIO_Port, RELAY_CNTRL_Pin, GPIO_PIN_RESET);
  }
}

//get current load switch state:
static load_switch_state_t getLoadSwitchState(void)
{
  if (HAL_GPIO_ReadPin(RELAY_CNTRL_GPIO_Port, RELAY_CNTRL_Pin) == GPIO_PIN_SET){
    return LOAD_ON;
  }
  else{
    return LOAD_OFF;
  }
}

//convert ACS712 ADC value to mA:
static int32_t convToCurrent(int32_t adc_value)
{
  //convert to uV:
  int32_t _tmp = ADC_REF_VOLT_MV * 1000 / 4095 * adc_value;
  
  //converte to voltage amplitude, uV:
  _tmp = abs(_tmp - ADC_ZERO_CURR_VOLT_MV * 1000);
  
  //return current value in mA:
  return _tmp / CURR_SENS_SENSITIVITY_MV;
}

//check load by measuring current:
static heater_load_state_t check_load(void)
{
  int32_t _data[ADC_SAMPLE_NUMBER] = {0};
  uint32_t _time_last_sample = 0;
  int32_t _avrg_curr = 0;
  
  for (uint32_t i = 0; i < ADC_SAMPLE_NUMBER; i++) {
    _time_last_sample = HAL_GetTick();
    
    //start ADC conversation to check load current consumption:
    HAL_ADC_Start(&hadc);
    
    //wait for ADC conversation to finish:
    if (HAL_ADC_PollForConversion(&hadc, ADC_CONV_TIMEOUT_MS) == HAL_OK){
      _data[i] = HAL_ADC_GetValue(&hadc);
      
      _data[i] = convToCurrent(_data[i]);//convert to current mA
    }
    else{ //rise an error flag
      _error_flag = ERROR_PRESENT;
      return LOAD_TOO_SMALL;
    }
    
    //delay between samples:
    while ((HAL_GetTick() - _time_last_sample) < ADC_SAMPLE_PERIOD_MS)
      ;
  }
  
  //calculate average current:
  for (uint32_t i = 0; i < ADC_SAMPLE_NUMBER; i++){
    _avrg_curr = _data[i];
  }
  _avrg_curr = _avrg_curr / ADC_SAMPLE_NUMBER;
  
  if (_avrg_curr > LOAD_CURR_THRESHOLD_MA){ //over threshold
    return LOAD_OK;
  }
  else{
    return LOAD_TOO_SMALL;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
