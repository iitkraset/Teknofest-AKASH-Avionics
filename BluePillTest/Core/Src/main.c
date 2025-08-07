/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055_stm32.h"
#include "BMP388.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//BMP280_HandleTypedef bmp280;

float pressure, temperature, humidity;
BMP388_HandleTypeDef hbmp388;
uint16_t size;
uint8_t Data[256];
uint8_t rxBuffer[128] = { 0 };
uint8_t rxIndex = 0;
uint8_t rxData;
uint8_t rxBuffer2[128] = { 0 };
uint8_t rxIndex2 = 0;
uint8_t rxData2;
float nmeaLong;
float nmeaLat;
float utcTime;
char northsouth;
char eastwest;
char posStatus;
float decimalLong;
float decimalLat;
float press = 0;
float temp = 0;
float device_alt = 0;
float mode = 0;
uint8_t launch = 0;
uint8_t apogee = 0;
uint32_t t1=0;
uint32_t currtime=0;
uint8_t burnout = 0;
uint8_t minalt = 0;
uint8_t angle = 0;
uint8_t drift = 0;
float prev_height = 0;
uint8_t fallalt = 0;
uint32_t t2 = 0;
uint8_t mainpara = 0;
bno055_vector_t v;
bno055_calibration_state_t k;
int flag = 1;
float initialalt=0;

float nmeaToDecimal(float coordinate) {
	int degree = (int) (coordinate / 100);
	float minutes = coordinate - degree * 100;
	float decimalDegree = minutes / 60;
	float decimal = degree + decimalDegree;
	return decimal;
}
#define TIM_FREQ 60000000

int presForFrequency (int frequency)
{
	if (frequency == 0) return 0;
	return ((TIM_FREQ/(1000*frequency))-1);  // 1 is added in the register
}

void gpsParse(char *strParse) {
	if (!strncmp(strParse, "$GNGGA", 6)) {
		sscanf(strParse, "$GNGGA,%f,%f,%c,%f,%c", &utcTime, &nmeaLat,
				&northsouth, &nmeaLong, &eastwest);
		decimalLat = nmeaToDecimal(nmeaLat);
		decimalLong = nmeaToDecimal(nmeaLong);
	} else if (!strncmp(strParse, "$GNGLL", 6)) {
		sscanf(strParse, "$GNGLL,%f,%c,%f,%c,%f", &nmeaLat, &northsouth,
				&nmeaLong, &eastwest, &utcTime);
		decimalLat = nmeaToDecimal(nmeaLat);
		decimalLong = nmeaToDecimal(nmeaLong);
	} else if (!strncmp(strParse, "$GNRMC", 6)) {
		sscanf(strParse, "$GNRMC,%f,%c,%f,%c,%f,%c", &utcTime, &posStatus,
				&nmeaLat, &northsouth, &nmeaLong, &eastwest);
		decimalLat = nmeaToDecimal(nmeaLat);
		decimalLong = nmeaToDecimal(nmeaLong);
	}
}

int gpsValidate(char *nmea) {
	char check[3];
	char calculatedString[3];
	int index;
	int calculatedCheck;

	index = 0;
	calculatedCheck = 0;

	// Ensure that the string starts with a "$"
	if (nmea[index] == '$')
		index++;
	else
		return 0;

	//No NULL reached, 75 char largest possible NMEA message, no '*' reached
	while ((nmea[index] != 0) && (nmea[index] != '*') && (index < 75)) {
		calculatedCheck ^= nmea[index]; // calculate the checksum
		index++;
	}

	if (index >= 75) {
		return 0; // the string is too long so return an error
	}

	if (nmea[index] == '*') {
		check[0] = nmea[index + 1];    //put hex chars in check string
		check[1] = nmea[index + 2];
		check[2] = 0;
	} else
		return 0;    // no checksum separator found therefore invalid data

	sprintf(calculatedString, "%02X", calculatedCheck);
	return ((calculatedString[0] == check[0])
			&& (calculatedString[1] == check[1])) ? 1 : 0;
}

float extract_float_from_bytes(uint8_t *data, uint16_t start_index) {

    float value;
    memcpy(&value, &data[start_index], sizeof(float));  // Copies 4 bytes
    return value;
}
float round2(float val) {
				 return round(val * 100.0) / 100.0;
			  }
void sft(uint8_t* packet)
{
	if (mode == 0) {
	    if (packet[1] == 0x20) {
	        mode = 1;
	        HAL_Delay(900);
	        while (1) {
	            uint8_t header_test = 0xAA;
	            float test_pressure = 0;
	            float test_altitude = 0;
	            float test_accx = 0, test_accy = 0, test_accz = 0;
	            float test_angx = 0, test_angy = 0, test_angz = 0;
	            int test_checksum = 0;
	            uint8_t test_footer1 = 0x0D;
	            uint8_t test_footer2 = 0x0A;

	            uint32_t raw_press = 0, raw_temp = 0, sensor_time = 0;
	            BMP388_ReadRawPressTempTime(&hbmp388, &raw_press, &raw_temp, &sensor_time);
	            BMP388_CompensateRawPressTemp(&hbmp388, raw_press, raw_temp, &test_pressure, &temp);
	            float test_altitude2 = round2(BMP388_FindAltitude(100400, test_pressure));
	            float test_pressure2 = round2(test_pressure);

	            v = bno055_getVectorEuler();
	            test_angx = round2(v.w);
	            test_angy = round2(v.x);
	            test_angz = round2(v.y);

	            bno055_vector_t accel = bno055_getVectorAccelerometer();
	            test_accx = round2(accel.x);
	            test_accy = round2(accel.y);
	            test_accz = round2(accel.z);

	            int index_test = 0;
	            uint8_t test_packet[36];

	            test_packet[index_test++] = header_test;
	            memcpy(&test_packet[index_test], &test_altitude2, sizeof(float)); index_test += sizeof(float);
	            memcpy(&test_packet[index_test], &test_pressure2, sizeof(float)); index_test += sizeof(float);
	            memcpy(&test_packet[index_test], &test_angx, sizeof(float)); index_test += sizeof(float);
	            memcpy(&test_packet[index_test], &test_angy, sizeof(float)); index_test += sizeof(float);
	            memcpy(&test_packet[index_test], &test_angz, sizeof(float)); index_test += sizeof(float);
	            memcpy(&test_packet[index_test], &test_accx, sizeof(float)); index_test += sizeof(float);
	            memcpy(&test_packet[index_test], &test_accy, sizeof(float)); index_test += sizeof(float);
	            memcpy(&test_packet[index_test], &test_accz, sizeof(float)); index_test += sizeof(float);
	            for (int i = 1; i < index_test; i++) {
	                test_checksum += test_packet[i];
	            }
//	            test_checksum += 0x6C;
	            test_checksum %= 256;
	            test_packet[index_test++] = test_checksum;
	            test_packet[index_test++] = test_footer1;
	            test_packet[index_test++] = test_footer2;
	            HAL_UART_Transmit(&huart2, test_packet, index_test, HAL_MAX_DELAY);
	            HAL_Delay(100);
	        }
	    }
      else if(packet[1]==0x22 || mode==2)
      {
		  mode = 2;
		  if(packet[1]!=0x24 && packet[0]==0xAB)
		  {
			  float taltitude = extract_float_from_bytes(packet, 1);
			  float tpressure = extract_float_from_bytes(packet, 5);
			  float acc_x    = extract_float_from_bytes(packet, 9);
			  float acc_y = extract_float_from_bytes(packet, 13);
			  float acc_z = extract_float_from_bytes(packet, 17);
			  float ang_x   = extract_float_from_bytes(packet, 21);
			  float ang_y = extract_float_from_bytes(packet, 25);
			  float ang_z = extract_float_from_bytes(packet, 29);
			  uint8_t test_checksum = packet[33];
			  uint8_t test_footer1 = packet[34];
			  uint8_t test_footer2 = packet[35];
			  if(flag)
			  {
				  initialalt = taltitude;
				  flag = 0;
			  }
			  float height = taltitude - initialalt;
			  currtime = HAL_GetTick();
			  if(height>=50 && launch == 0)
			  {

			  	  launch=1;
			  	  t1= HAL_GetTick();
			  }

			  if((currtime - t1) >= 3000 && launch == 1 && burnout == 0)
			  {

				  burnout=1;
			  }

			  if(height>=500  && minalt == 0)
			  {

				  minalt = 1;
			  }

			  if(ang_x + ang_y >= 80 )
			  {

				  angle = 1;
			  }

			  if(height - prev_height < 0 && angle == 1 && apogee == 0)
			  {

				  apogee = 1;
			  }

			 prev_height = height;

			 if(apogee == 1 && drift == 0)
			 {

				 drift = 1;
				 t2=HAL_GetTick();
			 }

			 if(height <= 550 && drift == 1 && fallalt == 0)
			 {

				 fallalt = 1;
			 }

			 if(currtime - t2 > 5000 && fallalt == 1 && mainpara == 0)
			 {

				 mainpara = 1;
			 }
			 uint8_t tr_packet[6];
			 tr_packet[0] = 0xAA;
			 tr_packet[1]=0;
			 if(launch==1){
				 tr_packet[1]+=128;
			 }
			 if (burnout==1){
				 tr_packet[1]+=64;
			 }
			 if (minalt==1){
				 tr_packet[1]+=32;
			 }
			 if (angle==1){
				 tr_packet[1]+=16;
			 }
			 if (apogee==1){
				 tr_packet[1]+=8;
			 }
			 if (drift==1){
				 tr_packet[1]+=4;
			 }
			 if (fallalt==1){
				 tr_packet[1]+=2;
			 }
			 if (mainpara==1){
				 tr_packet[1]+=1;
			 }
			 tr_packet[2]=0;
			 int t_checksum=0;
			 for (int i = 0; i < 3; i++) {
			 		t_checksum += packet[i];
			 }
			 t_checksum %= 256;
			 t_checksum += 6;
			 tr_packet[3] = t_checksum;
			 tr_packet[4] = 0x0D;
			 tr_packet[5] = 0x0A;
			HAL_UART_Transmit(&huart2, tr_packet, sizeof(tr_packet), HAL_MAX_DELAY);
			HAL_Delay(100);

		  }
		  else
		  {
			  mode=0;
			  launch=0;
			  apogee=0;
			  burnout=0;
			  t1=0;
			  minalt=0;
			  angle=0;
			  drift=0;
			  fallalt=0;
			  t2=0;
			  mainpara=0;
		  }
      }
      else if(packet[1]==0x24)
      {mode= 0;}
      }

}
int check_CheckSum(uint8_t* rxBuffer, uint8_t bsize){
	if (bsize < 3) return 0;
	uint8_t val = 0;
	for (uint8_t i = 1; i < bsize-2; i++){
		val += rxBuffer[i];
	}
	val %= 256;
	return (val == rxBuffer[bsize-2]);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	// ZOE Interrupt
	if (huart->Instance == USART2) {
		// if the character received is other than 'enter' ascii13, save the data in buffer
		if (rxData != '\n' && rxIndex < sizeof(rxBuffer)) {
			rxBuffer[rxIndex++] = rxData;
		} else {
			char *bfr = (char*) rxBuffer;
			if (gpsValidate((char*) rxBuffer))
				gpsParse(bfr);
			rxIndex = 0;
			memset(rxBuffer, 0, sizeof(rxBuffer));
		}
		HAL_UART_Receive_IT(&huart2, &rxData, 1); // Enabling interrupt receive again
	}
//	// Xbee Interrupt
//	else if (huart->Instance == USART2) {
//		// We separate messages with a hash token in start and end.
//		if (rxData2 != '#' && rxIndex2 < sizeof(rxBuffer2)) {
//			rxBuffer2[rxIndex2++] = rxData2;
//		} else if (rxIndex2 != 0 && rxData2 == '#') {
//			rxBuffer2[rxIndex2] = '\0';
//			char *message = (char*) rxBuffer2;
//			rxIndex2 = 0;
//			memset(rxBuffer2, 0, sizeof(rxBuffer2));
//		}
//		HAL_UART_Receive_IT(&huart2, &rxData2, 1); // Enabling interrupt receive again
//	}
//}
	if (huart->Instance == USART1) {
				// We separate messages with a hash token in start and end.
		// VERY VERY POOR CODE BUT ALRIGHT
		 if (rxData2 != 0x0A && rxIndex2 < sizeof(rxBuffer2)) {
				rxBuffer2[rxIndex2++] = rxData2;

			} else if (rxIndex2 != 0 && rxData2 == 0x0A) {
				if (rxBuffer2[rxIndex2] == 0x0D) {
					if (!check_CheckSum(rxBuffer2, rxIndex2)){
						// Corrupted packet
						// TODO: Check if this is actually valid, might not be
						memset(rxBuffer2, 0, sizeof(rxBuffer2));
						rxIndex2 = 0;
					} else {
						// Complete, working, correct packet
						switch (mode){
						case 0:
							if (rxIndex2 == 4){
								if (rxBuffer2[1] == 0x20){
									// Start SMT
									mode = 1;
									memset(rxBuffer2, 0, sizeof(rxBuffer2));
									rxIndex2 = 0;
								} else if (rxBuffer2[1] == 0x22){
									// Start SFT
									// In this case, keep reading messages
									mode = 3;
									memset(rxBuffer2, 0, sizeof(rxBuffer2));
									rxIndex2 = 0;
								}
							} else {
								// Wrong message? Premature stop condition?
								memset(rxBuffer2, 0, sizeof(rxBuffer2));
								rxIndex2 = 0;
							}
							break;
						case 1:
							// SMT was going on
							if (rxIndex2 == 4 && rxBuffer2[1] == 0x24){
								// Stop SMT
								mode = 0; // SFT Completed, will proceed to working normally
								memset(rxBuffer2, 0, sizeof(rxBuffer2));
								rxIndex2 = 0;
							} else {
								memset(rxBuffer2, 0, sizeof(rxBuffer2)); // UNEXPECTED
								rxIndex2 = 0;
							}
							break;
						case 3:
							// Since I havent mentioned 2
							// We were in SFT
							if (rxIndex2 == 35){
								sft(rxBuffer2);
							} else {
								memset(rxBuffer2, 0, sizeof(rxBuffer2)); // UNEXPECTED
								rxIndex2 = 0;
							}
							break;
						default:
							break;
						}
					}
				}
			}
			HAL_UART_Receive_IT(&huart1, &rxData2, 1);
		}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_InitTick(0);
//	bno055_assignI2C(&hi2c1);
//	bno055_setup();
//	bno055_setOperationModeNDOF();
  HAL_Delay(5000);
  bno055_assignI2C(&hi2c1);
  	bno055_setup();
  	HAL_Delay(500);
	bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
  	HAL_Delay(500);
	bno055_reset();
	HAL_Delay(500);
  	bno055_setOperationModeNDOF();
//  	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_4);
	  hbmp388.hi2c = &hi2c1;
	  BMP388_Init(&hbmp388);
	  BMP388_SetTempOS(&hbmp388, BMP388_NO_OVERSAMPLING);
	  BMP388_SetPressOS(&hbmp388, BMP388_OVERSAMPLING_8X);
	  BMP388_SetIIRFilterCoeff(&hbmp388, BMP3_IIR_FILTER_COEFF_3);
	  BMP388_SetOutputDataRate(&hbmp388, BMP3_ODR_50_HZ);
	  HAL_UART_Receive_IT(&huart2, &rxData, 1);
	  while (mode == 1){
		  // We are in SMT Mode
		  /*
		   * Copy paste above code here.
		   */
	  }
	  float ground_alt = 0;
	  for (int i = 0; i < 100; i++){
		uint32_t raw_press = 0;
		uint32_t raw_temp = 0;
		uint32_t sensor_time = 0;
		BMP388_GetCalibData(&hbmp388);
		BMP388_ReadRawPressTempTime(&hbmp388, &raw_press, &raw_temp, &sensor_time);
		BMP388_CompensateRawPressTemp(&hbmp388, raw_press, raw_temp, &press, &temp);
		ground_alt += BMP388_FindAltitude(100400, press);
		HAL_Delay(50);
	  }
	  ground_alt /= 100;
	  device_alt = ground_alt;

//	HAL_UART_Transmit(&huart1, Data, size, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
	  k = bno055_getCalibrationState();
	  v = bno055_getVectorEuler();
//	  v = bno055_getVectorQuaternion();
	  printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", v.w, v.x, v.y);
	  uint32_t raw_press = 0;
	  uint32_t raw_temp = 0;
	  uint32_t sensor_time = 0;
	  BMP388_GetCalibData(&hbmp388);
	  BMP388_ReadRawPressTempTime(&hbmp388, &raw_press, &raw_temp, &sensor_time);
	  BMP388_CompensateRawPressTemp(&hbmp388, raw_press, raw_temp, &press, &temp);
//		HAL_Delay(100);
//		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
//			size = sprintf((char*) Data,
//					"Temperature/pressure reading failed\n");
////			HAL_UART_Transmit(&huart1, Data, size, 1000);
//			// I want to some how reset the board here
//			bmp280_init(&bmp280, &bmp280.params);
//			HAL_Delay(2000);
//		}
//
//		size = sprintf((char*) Data, "Pressure: %.2f Pa, Temperature: %.2f C",
//				pressure, temperature);
////		HAL_UART_Transmit(&huart1, Data, size, 1000);
//		if (bme280p) {
//			size = sprintf((char*) Data, ", Humidity: %.2f\n", humidity);
////			HAL_UART_Transmit(&huart1, Data, size, 1000);
//		}
//
//		else {
//			size = sprintf((char*) Data, "\n");
////			HAL_UART_Transmit(&huart1, Data, size, 1000);
//		}
//		HAL_Delay(2000);
//		k = bno055_getCalibrationState();
//		v = bno055_getVectorEuler();
//		printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", v.x, v.y, v.z);
//	v = bno055_getVectorQuaternion();
//	printf("W: %.2f X: %.2f Y: %.2f Z: %.2f\r\n", v.w, v.x, v.y, v.z);
		HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
//	  HAL_Delay(500);
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
//	  HAL_Delay(500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL15;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	(void) file;
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
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
	__disable_irq();
	while (1) {
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
