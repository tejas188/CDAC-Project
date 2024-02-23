#include "main.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

// Constants and variables for LCD control
#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit
#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD

// Gas sensor configuration
#define MQ_PIN ADC_CHANNEL_3 // Assuming connected to GPIOA Pin 0 (PA0)
#define RL_VALUE 5.0 // define the load resistance on the board, in kilo ohms
#define RO_CLEAN_AIR_FACTOR 9.83 // RO_CLEAN_AIR_FACTOR = (Sensor resistance in clean air)/RO
#define CALIBRATION_SAMPLE_TIMES 50
#define CALIBRATION_SAMPLE_INTERVAL 500 // milliseconds
#define READ_SAMPLE_TIMES 5
#define READ_SAMPLE_INTERVAL 100 // milliseconds

#define GAS_LPG 0
#define GAS_CO 1
#define GAS_SMOKE 2

// Peripheral handles
ADC_HandleTypeDef hadc1;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef hi2c3;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1;

// FDCAN filter configuration structure
FDCAN_FilterTypeDef sFilterConfig;

// FDCAN message header
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t Node1_Rx[8]; // FDCAN data structures
uint8_t backlight_state = 1; // Backlight state for LCD
uint8_t buffer[50]; // Buffer for messages
float Ro = 10; // Variable to store the calibration value

/* Gas curves (obtained from datasheet or experimentation) */
float LPGCurve[] = {2.3, 0.21, -0.47};
float COCurve[] = {2.3, 0.72, -0.34};
float SmokeCurve[] = {2.3, 0.53, -0.44};

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C3_Init(void);
static void MX_FDCAN1_Init(void);

void lcd_write_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init();
void lcd_write_string(char *str);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_clear(void);
void lcd_backlight(uint8_t state);

float MQResistanceCalculation(uint16_t raw_adc);
float MQCalibration(void);
float MQRead(void);
int MQGetGasPercentage(float rs_ro_ratio, uint8_t gas_id);
int MQGetPercentage(float rs_ro_ratio, float *pcurve);

void filter_config(void);
void Tx_header(void);

// Main function
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_I2C3_Init();
  MX_FDCAN1_Init();

// Initialize FDCAN header and filters
  Tx_header();
  filter_config();
// Initialize LCD
  lcd_init();
  lcd_backlight(1); // Turn on backlight

//Start FDCAN
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  	  {
	  	  Error_Handler();
  	  }
// Start and set state of LED, PWM and ADC
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
  HAL_ADC_Start(&hadc1);

// Calibration phase for gas sensor
  char buffer_1[16];
  sprintf(buffer_1, "Calibrating...");
  HAL_UART_Transmit(&huart1, (uint8_t*)"Calibrating...\r\n", 16, HAL_MAX_DELAY);
  lcd_clear();  // Clear the LCD
  lcd_set_cursor(0, 0);  // Set the cursor to the first row, first column
  lcd_write_string(buffer_1);  // Write the adc_value to the LCD

  Ro = MQCalibration();
  char buffer[50];
  sprintf(buffer, "Calibration done. Ro = %.2f kohm\r\n", Ro);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

// Main loop
  while (1)
  {
  // Read MQ sensor
  float rs_ro_ratio = MQRead() / Ro;

    // Read gas levels and display on LCD
    char lpg_str[16];
    uint16_t lpg_val  = MQGetGasPercentage(rs_ro_ratio, GAS_LPG);
      sprintf(lpg_str, "LPG:%u", lpg_val);
      HAL_UART_Transmit(&huart1, (uint8_t*)lpg_str, strlen(lpg_str), HAL_MAX_DELAY);
      lcd_clear();  // Clear the LCD
      lcd_set_cursor(0, 0);  // Set the cursor to the first row, first column
      lcd_write_string(lpg_str);  // Write the lpg_str to the LCD

    char co_str[16];
    uint16_t co_val  = MQGetGasPercentage(rs_ro_ratio, GAS_CO);
      sprintf(co_str, "CO:%u", co_val);
      HAL_UART_Transmit(&huart1, (uint8_t*)co_str, strlen(co_str), HAL_MAX_DELAY);
      lcd_set_cursor(0, 9);  // Set the cursor to the first row, ninth column
      lcd_write_string(co_str);  // Write the co_str to the LCD

    char smoke_str[16];
    uint16_t smoke_val  = MQGetGasPercentage(rs_ro_ratio, GAS_SMOKE);
      sprintf(smoke_str, "SMK:%u",smoke_val);
      HAL_UART_Transmit(&huart1, (uint8_t*)smoke_str, strlen(smoke_str), HAL_MAX_DELAY);
      lcd_set_cursor(1, 0);  // Set the cursor to the secound row, first column
      lcd_write_string(smoke_str);  // Write the smoke_str to the LCD



	// Handle incoming FDCAN messages
	if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1,&RxHeader, Node1_Rx)==HAL_OK)
	{
		uint8_t temp_str[7];
		uint8_t temp_val[7];
		strcpy(temp_val,Node1_Rx);	  //uint8_t temp_val  = Node1_Rx;
		sprintf(temp_str, "TEMP:");
		lcd_set_cursor(1, 9);  // Set the cursor to the first row, first column
		lcd_write_string(temp_str);  // Write the adc_value to the LCD
		HAL_UART_Transmit(&huart1,Node1_Rx, 5,HAL_MAX_DELAY);
		lcd_set_cursor(1, 14);  // Set the cursor to the first row, first column
		lcd_write_string(temp_val);  // Write the adc_value to the LCD

	    int num = atoi(Node1_Rx); // Convert string to integer
		if (num >= 40)
		{
		    char abnormal_str[16];  // Assuming a 4-digit ADC value
		      sprintf(abnormal_str, "DANGER TEMP...");  // Convert adc_value to a string
		      lcd_clear();  // Clear the LCD
		      lcd_set_cursor(0, 0);  // Set the cursor to the first row, first column
		      lcd_write_string(abnormal_str);  // Write the adc_value to the LCD
		}
	}

    // Control LED output based on gas levels
    if ( lpg_val > 400 || co_val > 25 || smoke_val > 1000  )
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
    		int x;
    		for(x=10; x<30; x=x+1)
    			{
    				__HAL_TIM_SET_AUTORELOAD(&htim1, x*2);
    				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, x);
    				HAL_Delay(100);
    	    	    char abnormal_str[16];  // Assuming a 4-digit ADC value
    	    	      sprintf(abnormal_str, "DANGER MQ2...");  // Convert adc_value to a string
    	    	      lcd_clear();  // Clear the LCD
    	    	      lcd_set_cursor(0, 0);  // Set the cursor to the first row, first column
    	    	      lcd_write_string(abnormal_str);  // Write the adc_value to the LCD
                }
    }
    else
    {
           	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
           	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
       		int x=0;
			__HAL_TIM_SET_AUTORELOAD(&htim1, x*2);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, x);
    }
      HAL_Delay(200);
	}
}

void filter_config(void)
{
	/* Configure Rx filter */
 	  sFilterConfig.IdType = FDCAN_STANDARD_ID;
 	  sFilterConfig.FilterIndex = 0;
 	  sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
 	  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
 	  sFilterConfig.FilterID1 = 0x10;
 	  sFilterConfig.FilterID2 = 0x10;

 	 HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
}

void Tx_header(void)
{
	/* Prepare Tx Header */
 	  TxHeader.Identifier = 0x12;
 	  TxHeader.IdType = FDCAN_STANDARD_ID;
 	  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
 	  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
 	  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
 	  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
 	  TxHeader.FDFormat = FDCAN_FD_CAN;
 	  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
 		//TxHeader.TxEventFifoControl =FDCAN_STORE_TX_EVENTS;
 	  TxHeader.MessageMarker = 0x0;
}


void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
  uint8_t data = nibble << D4_BIT;
  data |= rs << RS_BIT;
  data |= backlight_state << BL_BIT; // Include backlight state in data
  data |= 1 << EN_BIT;
  HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDR << 1, &data, 1, 100);
  HAL_Delay(1);
  data &= ~(1 << EN_BIT);
  HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDR << 1, &data, 1, 100);
}

void lcd_send_cmd(uint8_t cmd) {
  uint8_t upper_nibble = cmd >> 4;
  uint8_t lower_nibble = cmd & 0x0F;
  lcd_write_nibble(upper_nibble, 0);
  lcd_write_nibble(lower_nibble, 0);
  if (cmd == 0x01 || cmd == 0x02) {
    HAL_Delay(2);
  }
}

void lcd_send_data(uint8_t data) {
  uint8_t upper_nibble = data >> 4;
  uint8_t lower_nibble = data & 0x0F;
  lcd_write_nibble(upper_nibble, 1);
  lcd_write_nibble(lower_nibble, 1);
}

void lcd_init() {
  HAL_Delay(50);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(5);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x02, 0);
  lcd_send_cmd(0x28);
  lcd_send_cmd(0x0C);
  lcd_send_cmd(0x06);
  lcd_send_cmd(0x01);
  HAL_Delay(2);
}

void lcd_write_string(char *str) {
  while (*str) {
    lcd_send_data(*str++);
  }
}

void lcd_set_cursor(uint8_t row, uint8_t column) {
    uint8_t address;
    switch (row) {
        case 0:
            address = 0x00;
            break;
        case 1:
            address = 0x40;
            break;
        default:
            address = 0x00;
    }
    address += column;
    lcd_send_cmd(0x80 | address);
}

void lcd_clear(void) {
	lcd_send_cmd(0x01);
    HAL_Delay(2);
}

void lcd_backlight(uint8_t state) {
  if (state) {
    backlight_state = 1;
  } else {
    backlight_state = 0;
  }
}
// Function to perform gas sensor calibration
float MQCalibration(void)
{
  int i;
  float val = 0;

  for (i = 0; i < CALIBRATION_SAMPLE_TIMES; i++) { // Take multiple samples
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
    val += MQResistanceCalculation(adc_value);
    HAL_Delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBRATION_SAMPLE_TIMES; // Calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR;      // Divide by RO_CLEAN_AIR_FACTOR to yield the Ro
                                        // according to the chart in the datasheet

  return val;
}
// Function to read gas levels from sensor
float MQRead(void) {
  float rs = 0.0;
  for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
    rs += MQResistanceCalculation(adc_value);
    HAL_Delay(READ_SAMPLE_INTERVAL);
  }
  return rs / READ_SAMPLE_TIMES;
}
// Function to calculate gas percentage
int MQGetGasPercentage(float rs_ro_ratio, uint8_t gas_id) {
  if (gas_id == GAS_LPG) return MQGetPercentage(rs_ro_ratio, LPGCurve);
  else if (gas_id == GAS_CO) return MQGetPercentage(rs_ro_ratio, COCurve);
  else if (gas_id == GAS_SMOKE) return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  return 0;
}
// Function to calculate gas percentage based on curve
int MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return pow(10, ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0]);
}

float MQResistanceCalculation(uint16_t raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}


/************************************************************************************/

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00303D5B;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 127;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 106;
  hfdcan1.Init.NominalTimeSeg2 = 21;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 13;
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
