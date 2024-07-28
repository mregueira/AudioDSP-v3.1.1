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
#include "DSP/SigmaStudioFW.h"

#include "DSP/DSP_IC_1.h"
#include "DSP/DSP_IC_1_PARAM.h"

#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_POT 15

#define VOL_SUB 10
#define VOL_ARRAY 11
#define LOUD_LOW_ARRAY 12
#define LOUD_HIGH_ARRAY 13
#define LOUD_GRL_ARRAY 14

// I2C2
#define ADC0_ADDR 0x94
#define ADC1_ADDR 0x96
// I2C3
#define ADC2_ADDR 0x94

// I2C1
#define DSP0_ADDR 0x70
#define DSP1_ADDR 0x72
#define DSP2_ADDR 0x74
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef stat;

uint8_t auxData[4];

uint32_t value[ADC_POT]; // To store ADC values
uint16_t pote[ADC_POT];  // Potentiometer values 0-29
uint16_t log_in_table[30];
uint16_t linear_in_table[30];
uint16_t flag[ADC_POT];  // Flag for different potentiometers
uint8_t update = 0;
uint8_t tx_check = 0;

ADI_REG_TYPE aux[4];
ADI_REG_TYPE data_SafeLoad[4]; // Per slot
ADI_REG_TYPE address_SafeLoad[4]; // 2 Bytes per address
ADI_REG_TYPE num_SafeLoad_Lower[4];
ADI_REG_TYPE num_SafeLoad_Upper[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void delay_us(uint16_t us);
void Safeload_Write(void);
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
	  uint16_t k = 0;
	  uint16_t pote_aux = 0;
	  uint16_t BandAddress[ADC_POT]; // Addresses of filters
	  uint32_t vol_data[30]; // Fixed volume values
	  uint32_t loud_data[30]; // Fixed loudness values
	  uint32_t boost_data[30]; // Fixed boost loudness values
	  uint32_t comp_data[30]; // Fixed compensation values

	  // Band Address Assign
	  BandAddress[0] = MOD_BAND32_SEL_DCINPALG145X4VALUE_ADDR;
	  BandAddress[1] = MOD_BAND64_SEL_DCINPALG145X5VALUE_ADDR;
	  BandAddress[2] = MOD_BAND128_SEL_DCINPALG145X6VALUE_ADDR;
	  BandAddress[3] = MOD_BAND256_SEL_DCINPALG145X7VALUE_ADDR;
	  BandAddress[4] = MOD_BAND512_SEL_DCINPALG145X8VALUE_ADDR;
	  BandAddress[5] = MOD_BAND1K_SEL_DCINPALG145X9VALUE_ADDR;
	  BandAddress[6] = MOD_BAND2K_SEL_DCINPALG145X10VALUE_ADDR;
	  BandAddress[7] = MOD_BAND4K_SEL_DCINPALG145X11VALUE_ADDR;
	  BandAddress[8] = MOD_BAND8K_SEL_DCINPALG145X12VALUE_ADDR;
	  BandAddress[9] = MOD_BAND16K_SEL_DCINPALG145X13VALUE_ADDR;
	  //BandAddress[VOL_SUB] = MOD_VOLHP_GAINALGNS145X4GAIN_ADDR; // Subwoofer Volume
	  BandAddress[VOL_ARRAY] = MOD_VOL_GAINALGNS145X1GAIN_ADDR; // General Volume
	  BandAddress[LOUD_LOW_ARRAY] = MOD_LOUD_ALG0_LEVEL0_ADDR;
	  BandAddress[LOUD_HIGH_ARRAY] = MOD_LOUD_ALG0_LEVEL1_ADDR;
	  BandAddress[LOUD_GRL_ARRAY] = MOD_LOUD_SEL_DCINPALG145X15VALUE_ADDR;

	  // 8.24 FixPoint Volume
	  vol_data[29] = 0x01000000; // 0dB
	  vol_data[28] = 0x00CB5918; // -2dB
	  vol_data[27] = 0x00A1866C; // -4dB
	  vol_data[26] = 0x00804DCE; // -6dB
	  vol_data[25] = 0x0065EA5A; // -8dB
	  vol_data[24] = 0x0050F44E; // -10dB
	  vol_data[23] = 0x00404DE6; // -12dB
	  vol_data[22] = 0x00331427; // -14dB (-7.2dB)
	  vol_data[21] = 0x002892C2; // -16dB
	  vol_data[20] = 0x00203A7E; // -18dB
	  vol_data[19] = 0x0019999A; // -20dB
	  vol_data[18] = 0x001455B6; // -22dB
	  vol_data[17] = 0x0010270B; // -24dB
	  vol_data[16] = 0x000CD495; // -26dB
	  vol_data[15] = 0x000A3109; // -28dB
	  vol_data[14] = 0x0008186E; // -30dB
	  vol_data[13] = 0x00066E31; // -32dB
	  vol_data[12] = 0x00051B9D; // -34dB
	  vol_data[11] = 0x00040EAD; // -36dB
	  vol_data[10] = 0x0003390D; // -38dB
	  vol_data[9] = 0x00028F5C; // -40dB
	  vol_data[8] = 0x00020892; // -42dB
	  vol_data[7] = 0x00019D81; // -44dB
	  vol_data[6] = 0x00014875; // -46dB
	  vol_data[5] = 0x000104E7; // -48dB
	  vol_data[4] = 0x0000CF3E; // -50dB
	  vol_data[3] = 0x0000A49E; // -52dB
	  vol_data[2] = 0x000082C3; // -54dB
	  vol_data[1] = 0x000067DE; // -56dB
	  vol_data[0] = 0x00005281; // -58dB

	  // 8.24 FixPoint Loudness
	  loud_data[29] = 0x01000000; // 0dB
	  loud_data[28] = 0x00F1ADF9; // -0.5dB
	  loud_data[27] = 0x00E42905; // -1dB
	  loud_data[26] = 0x00D765AC; // -1.5dB
	  loud_data[25] = 0x00CB5918; // -2dB
	  loud_data[24] = 0x00BFF911; // -2.5dB
	  loud_data[23] = 0x00B53BEF; // -3dB
	  loud_data[22] = 0x00AB1896; // -3.5dB
	  loud_data[21] = 0x00A1866C; // -4dB
	  loud_data[20] = 0x00987D50; // -4.5dB
	  loud_data[19] = 0x008FF59A; // -5dB
	  loud_data[18] = 0x0087E80B; // -5.5dB
	  loud_data[17] = 0x00804DCE; // -6dB
	  loud_data[16] = 0x00792071; // -6.5dB
	  loud_data[15] = 0x007259DB; // -7dB
	  loud_data[14] = 0x006BF44D; // -7.5dB
	  loud_data[13] = 0x0065EA5A; // -8dB
	  loud_data[12] = 0x006036E1; // -8.5dB
	  loud_data[11] = 0x005AD50D; // -9dB
	  loud_data[10] = 0x0055C04C; // -9.5dB
	  loud_data[9] = 0x0050F44E; // -10dB
	  loud_data[8] = 0x004C6D01; // -10.5dB
	  loud_data[7] = 0x0048268E; // -11dB
	  loud_data[6] = 0x00441D54; // -11.5dB
	  loud_data[5] = 0x00404DE6; // -12dB
	  loud_data[4] = 0x003CB509; // -12.5dB
	  loud_data[3] = 0x00394FAF; // -13dB
	  loud_data[2] = 0x00361AF6; // -13.5dB
	  loud_data[1] = 0x00331427; // -14dB
	  loud_data[0] = 0x003038AF; // -14.5dB

	  // 8.24 Compensation FixPoint
	  comp_data[29] = 0x01000000; // 0dB
	  comp_data[28] = 0x010F2B41; // +0.5dB
	  comp_data[27] = 0x011F3C9A; // +1dB
	  comp_data[26] = 0x013041AF; // +1.5dB
	  comp_data[25] = 0x014248F0; // +2dB
	  comp_data[24] = 0x015561A9; // +2.5dB
	  comp_data[23] = 0x01699C0F; // +3dB
	  comp_data[22] = 0x017F094D; // +3.5dB
	  comp_data[21] = 0x0195BB8F; // +4dB
	  comp_data[20] = 0x01ADC61A; // +4.5dB
	  comp_data[19] = 0x01C73D52; // +5dB
	  comp_data[18] = 0x01E236D4; // +5.5dB
	  comp_data[17] = 0x01FEC983; // +6dB
	  comp_data[16] = 0x021D0D9E; // +6.5dB
	  comp_data[15] = 0x023D1CD4; // +7dB
	  comp_data[14] = 0x025F1259; // +7.5dB
	  comp_data[13] = 0x02830AFD; // +8dB
	  comp_data[12] = 0x02A92547; // +8.5dB
	  comp_data[11] = 0x02D1818B; // +9dB
	  comp_data[10] = 0x02FC4209; // +9.5dB
	  comp_data[9] = 0x03298B07; // +10dB
	  comp_data[8] = 0x035982F3; // +10.5dB
	  comp_data[7] = 0x038C5281; // +11dB
	  comp_data[6] = 0x03C224CD; // +11.5dB
	  comp_data[5] = 0x03FB2784; // +12dB
	  comp_data[4] = 0x04378B05; // +12.5dB
	  comp_data[3] = 0x0477828F; // +13dB
	  comp_data[2] = 0x04BB4469; // +13.5dB
	  comp_data[1] = 0x05030A11; // +14dB
	  comp_data[0] = 0x054F106E; // +14.5dB

	  // 8.24 FixPoint Bass and Treble Loudness
	  boost_data[29] = 0x02800000; // 2.50
	  boost_data[28] = 0x02666666; // 2.40
	  boost_data[27] = 0x02570a3c; // 2.34
	  boost_data[26] = 0x02451eb8; // 2.27
	  boost_data[25] = 0x02333332; // 2.20
	  boost_data[24] = 0x022147ae; // 2.13
	  boost_data[23] = 0x020f5c28; // 2.06
	  boost_data[22] = 0x02000000; // 2.00
	  boost_data[21] = 0x01ee147a; // 1.93
	  boost_data[20] = 0x01dc28f4; // 1.86
	  boost_data[19] = 0x01ca3d70; // 1.79
	  boost_data[18] = 0x01b851ea; // 1.72
	  boost_data[17] = 0x01a8f5c2; // 1.66
	  boost_data[16] = 0x01970a3c; // 1.59
	  boost_data[15] = 0x01851eb8; // 1.52
	  boost_data[14] = 0x01733332; // 1.45
	  boost_data[13] = 0x016147ae; // 1.38
	  boost_data[12] = 0x0151eb84; // 1.32
	  boost_data[11] = 0x01400000; // 1.25
	  boost_data[10] = 0x012e147a; // 1.18
	  boost_data[9] =  0x011c28f4; // 1.11
	  boost_data[8] =  0x010a3d70; // 1.04
	  boost_data[7] =  0x00fae146; // 0.98
	  boost_data[6] =  0x00e8f5c2; // 0.91
	  boost_data[5] =  0x00d70a3c; // 0.84
	  boost_data[4] =  0x00c51eb8; // 0.77
	  boost_data[3] =  0x00b33332; // 0.70
	  boost_data[2] =  0x00a3d70a; // 0.64
	  boost_data[1] =  0x0091eb84; // 0.57
	  boost_data[0] =  0x00800000; // 0.50

	  for(k=0; k<30; k++)
	  {
		  log_in_table[k] = 4096.0*log10(1.0+(3.0*k/10.0));
		  linear_in_table[k] = 4096*k/30;
	  }
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // Start timer for delay function
  HAL_TIM_Base_Start(&htim4);

  // Reset DSPs
  HAL_GPIO_WritePin(nRST_DSP0_GPIO_Port, nRST_DSP0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(nRST_DSP1_GPIO_Port, nRST_DSP1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(nRST_DSP2_GPIO_Port, nRST_DSP2_Pin, GPIO_PIN_RESET);

  HAL_Delay(100);

  // Disable CLK
  HAL_GPIO_WritePin(EN_SCK_GPIO_Port, EN_SCK_Pin, GPIO_PIN_RESET);

  // Configure DACs
  HAL_GPIO_WritePin(FMT_GPIO_Port, FMT_Pin, GPIO_PIN_RESET); // 16-to-24 bit I2S Format
  HAL_GPIO_WritePin(DEMP1_GPIO_Port, DEMP1_Pin, GPIO_PIN_RESET); // De-Emphasis
  HAL_GPIO_WritePin(DEMP0_GPIO_Port, DEMP0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MUTE_GPIO_Port, MUTE_Pin, GPIO_PIN_RESET); // Mute OFF

  // Enable CLK
  HAL_GPIO_WritePin(EN_SCK_GPIO_Port, EN_SCK_Pin, GPIO_PIN_SET);

  // Mute DSPs
  HAL_GPIO_WritePin(GPIO_MCU0_GPIO_Port, GPIO_MCU0_Pin, GPIO_PIN_RESET);

  // Enable DSPs
  HAL_GPIO_WritePin(nRST_DSP0_GPIO_Port, nRST_DSP0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(nRST_DSP1_GPIO_Port, nRST_DSP1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(nRST_DSP2_GPIO_Port, nRST_DSP2_Pin, GPIO_PIN_SET);

  // DSPs Software Download
  default_download_IC_1(DSP0_ADDR);
  default_download_IC_1(DSP1_ADDR);
  default_download_IC_1(DSP2_ADDR);

  // Configure ADCs
  auxData[0] = 0xFE;
  stat = HAL_I2C_Mem_Write(&hi2c2, ADC0_ADDR, 0x00, 1, auxData, 1, 1000);
  stat = HAL_I2C_Mem_Write(&hi2c2, ADC1_ADDR, 0x00, 1, auxData, 1, 1000);
  stat = HAL_I2C_Mem_Write(&hi2c3, ADC2_ADDR, 0x00, 1, auxData, 1, 1000);
  auxData[0] = 0x47;
  stat = HAL_I2C_Mem_Write(&hi2c2, ADC0_ADDR, 0x20, 1, auxData, 1, 1000);
  stat = HAL_I2C_Mem_Write(&hi2c2, ADC1_ADDR, 0x20, 1, auxData, 1, 1000);
  stat = HAL_I2C_Mem_Write(&hi2c3, ADC2_ADDR, 0x20, 1, auxData, 1, 1000);

  // Un-mute DSPs
  HAL_GPIO_WritePin(GPIO_MCU0_GPIO_Port, GPIO_MCU0_Pin, GPIO_PIN_SET);

  // Set flag ADC to update
  for(k=0; k<ADC_POT; k++)
  {
	  flag[k] = 1;
  }

  // DMA Start
  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, value, ADC_POT);

  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 	  data_SafeLoad[2] = 0x00;
	  data_SafeLoad[1] = 0x00;
	  data_SafeLoad[0] = 0x00;

	  address_SafeLoad[1] = 0x00;
	  address_SafeLoad[0] = 0x00;

	  num_SafeLoad_Lower[3] = 0x01;
	  num_SafeLoad_Lower[2] = 0x00;
	  num_SafeLoad_Lower[1] = 0x00;
	  num_SafeLoad_Lower[0] = 0x00;

	  num_SafeLoad_Upper[3] = 0x00;
	  num_SafeLoad_Upper[2] = 0x00;
	  num_SafeLoad_Upper[1] = 0x00;
	  num_SafeLoad_Upper[0] = 0x00;

	  if(update == 1)
	  {
		  for(k=0; k<10; k++) // Filters 32Hz - 16KHz
		  {
			  if(flag[k] == 1)
			  {
				  flag[k] = 0;
				  data_SafeLoad[3] = 29 - pote[k];
				  address_SafeLoad[3] = 0xFF & (BandAddress[k]);
				  address_SafeLoad[2] = 0xFF & ((BandAddress[k])>>8);
				  Safeload_Write();
				  delay_us(100);
			  }

		  }

		  if(flag[VOL_ARRAY] == 1) // Volume General
		  {
			  flag[VOL_ARRAY] = 0;
			  pote_aux = 29 - pote[VOL_ARRAY];

			  data_SafeLoad[3] = 0xFF & (vol_data[pote_aux]);
			  data_SafeLoad[2] = 0xFF & ((vol_data[pote_aux])>>8);
			  data_SafeLoad[1] = 0xFF & ((vol_data[pote_aux])>>16);
			  data_SafeLoad[0] = 0xFF & ((vol_data[pote_aux])>>24);
			  address_SafeLoad[3] = 0xFF & (BandAddress[VOL_ARRAY]);
			  address_SafeLoad[2] = 0xFF & ((BandAddress[VOL_ARRAY])>>8);
			  Safeload_Write();
			  delay_us(100);
		  }

//		  if(flag[VOL_SUB] == 1)
//		  {
//			  flag[VOL_SUB] = 0;
//			  pote_aux = 29 - pote[VOL_SUB];
//
//			  data_SafeLoad[3] = 0xFF & (vol_data[pote_aux]);
//			  data_SafeLoad[2] = 0xFF & ((vol_data[pote_aux])>>8);
//			  data_SafeLoad[1] = 0xFF & ((vol_data[pote_aux])>>16);
//			  data_SafeLoad[0] = 0xFF & ((vol_data[pote_aux])>>24);
//			  address_SafeLoad[3] = 0xFF & (BandAddress[VOL_SUB]);
//			  address_SafeLoad[2] = 0xFF & ((BandAddress[VOL_SUB])>>8);
//			  SIGMA_WRITE_REGISTER_BLOCK(DSP0_ADDR, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
//			  SIGMA_WRITE_REGISTER_BLOCK(DSP0_ADDR, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
//			  SIGMA_WRITE_REGISTER_BLOCK(DSP0_ADDR, MOD_SAFELOADMODULE_NUM_SAFELOAD_ADDR, 4, num_SafeLoad);
//			  SIGMA_WRITE_REGISTER_BLOCK(DSP1_ADDR, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
//			  SIGMA_WRITE_REGISTER_BLOCK(DSP1_ADDR, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
//			  SIGMA_WRITE_REGISTER_BLOCK(DSP1_ADDR, MOD_SAFELOADMODULE_NUM_SAFELOAD_ADDR, 4, num_SafeLoad);
//			  SIGMA_WRITE_REGISTER_BLOCK(DSP2_ADDR, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
//			  SIGMA_WRITE_REGISTER_BLOCK(DSP2_ADDR, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
//			  SIGMA_WRITE_REGISTER_BLOCK(DSP2_ADDR, MOD_SAFELOADMODULE_NUM_SAFELOAD_ADDR, 4, num_SafeLoad);
//			  delay_us(100);
//		  }

		  if(flag[LOUD_LOW_ARRAY] == 1) // Loudness Low Side
		  {
			  flag[LOUD_LOW_ARRAY] = 0;
			  pote_aux = 29 - pote[LOUD_LOW_ARRAY];

			  data_SafeLoad[3] = 0xFF & (boost_data[pote_aux]);
			  data_SafeLoad[2] = 0xFF & ((boost_data[pote_aux])>>8);
			  data_SafeLoad[1] = 0xFF & ((boost_data[pote_aux])>>16);
			  data_SafeLoad[0] = 0xFF & ((boost_data[pote_aux])>>24);
			  address_SafeLoad[3] = 0xFF & (BandAddress[LOUD_LOW_ARRAY]);
			  address_SafeLoad[2] = 0xFF & ((BandAddress[LOUD_LOW_ARRAY])>>8);
			  Safeload_Write();
			  delay_us(100);
		  }

		  if(flag[LOUD_HIGH_ARRAY] == 1) // Loudness High Side
		  {
			  flag[LOUD_HIGH_ARRAY] = 0;
			  pote_aux = 29 - pote[LOUD_HIGH_ARRAY];

			  data_SafeLoad[3] = 0xFF & (boost_data[pote_aux]);
			  data_SafeLoad[2] = 0xFF & ((boost_data[pote_aux])>>8);
			  data_SafeLoad[1] = 0xFF & ((boost_data[pote_aux])>>16);
			  data_SafeLoad[0] = 0xFF & ((boost_data[pote_aux])>>24);
			  address_SafeLoad[3] = 0xFF & (BandAddress[LOUD_HIGH_ARRAY]);
			  address_SafeLoad[2] = 0xFF & ((BandAddress[LOUD_HIGH_ARRAY])>>8);
			  Safeload_Write();
			  delay_us(100);
		  }

		  if(flag[LOUD_GRL_ARRAY] == 1) // Loudness General
		  {
			  flag[LOUD_GRL_ARRAY] = 0;
			  pote_aux = pote[LOUD_GRL_ARRAY];

			  data_SafeLoad[3] = 0xFF & (loud_data[pote_aux]);
			  data_SafeLoad[2] = 0xFF & ((loud_data[pote_aux])>>8);
			  data_SafeLoad[1] = 0xFF & ((loud_data[pote_aux])>>16);
			  data_SafeLoad[0] = 0xFF & ((loud_data[pote_aux])>>24);
			  address_SafeLoad[3] = 0xFF & (BandAddress[LOUD_GRL_ARRAY]);
			  address_SafeLoad[2] = 0xFF & ((BandAddress[LOUD_GRL_ARRAY])>>8);
			  Safeload_Write();
			  delay_us(100);

			  data_SafeLoad[3] = 0xFF & (comp_data[pote_aux]);
			  data_SafeLoad[2] = 0xFF & ((comp_data[pote_aux])>>8);
			  data_SafeLoad[1] = 0xFF & ((comp_data[pote_aux])>>16);
			  data_SafeLoad[0] = 0xFF & ((comp_data[pote_aux])>>24);
			  address_SafeLoad[3] = 0xFF & (MOD_LOUD_COMP_GAINALGNS145X2GAIN_ADDR);
			  address_SafeLoad[2] = 0xFF & ((MOD_LOUD_COMP_GAINALGNS145X2GAIN_ADDR)>>8);
			  Safeload_Write();
			  delay_us(100);
		  }

		  update = 0;
		  HAL_ADC_Start_DMA(&hadc1, value, ADC_POT);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 15;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 80 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 123;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_SCK_Pin|DEMP1_Pin|DEMP0_Pin|FMT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MUTE_GPIO_Port, MUTE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_MCU0_Pin|nRST_DSP0_Pin|nRST_DSP1_Pin|nRST_DSP2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_SCK_Pin DEMP1_Pin DEMP0_Pin FMT_Pin */
  GPIO_InitStruct.Pin = EN_SCK_Pin|DEMP1_Pin|DEMP0_Pin|FMT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MUTE_Pin */
  GPIO_InitStruct.Pin = MUTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MUTE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_MCU0_Pin nRST_DSP0_Pin nRST_DSP1_Pin nRST_DSP2_Pin */
  GPIO_InitStruct.Pin = GPIO_MCU0_Pin|nRST_DSP0_Pin|nRST_DSP1_Pin|nRST_DSP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SIGMA_WRITE_REGISTER_BLOCK(uint16_t devAddress, uint16_t address, uint16_t length, uint8_t *pData)
{
	stat = HAL_I2C_Mem_Write(&hi2c1, devAddress, address, 2, pData, length, 1000);
}

void SIGMA_WRITE_DELAY(uint16_t devAddress, uint16_t length, uint8_t *pData)
{
	HAL_Delay(11);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t k = 0;


	for(k=VOL_SUB; k<ADC_POT; k++) // For volume, loud low, loud high, loud general
	{
		for(i=0; i<30; i++)
		{
			if((i == 0) && (value[k] < (linear_in_table[i+1])-15))
			{
				if(pote[k] != i)
				{
					pote[k] = i;
					flag[k] = 1;
				}
			}
			else if((i > 0) && (i < 29) && (value[k] > (linear_in_table[i]+15)) && (value[k] < (linear_in_table[i+1])-15))
			{
				if(pote[k] != i)
				{
					pote[k] = i;
					flag[k] = 1;
				}
			}
			else if((i == 29) && (value[k] > (linear_in_table[i]+15)))
			{
				if(pote[k] != i)
				{
					pote[k] = i;
					flag[k] = 1;
				}
			}
		}
	}


	for(j=0; j<10; j++) // For filter 32Hz - 16KHz
	{
		for(i=0; i<30; i++)
		{
			if((i == 0) && (value[j] < (log_in_table[i+1])-15))
			{
				if(pote[j] != i)
				{
					pote[j] = i;
					flag[j] = 1;
				}
			}
			else if((i > 0) && (i < 29) && (value[j] > (log_in_table[i]+15)) && (value[j] < (log_in_table[i+1])-15))
			{
				if(pote[j] != i)
				{
					pote[j] = i;
					flag[j] = 1;
				}
			}
			else if((i == 29) && (value[j] > (log_in_table[i]+15)))
			{
				if(pote[j] != i)
				{
					pote[j] = i;
					flag[j] = 1;
				}
			}
		}

	}

	update = 1;
}
void delay_us(uint16_t us)
{
	htim4.Instance->CNT = 0;
	while((htim4.Instance->CNT) < us);
}
void Safeload_Write(void)
{
	SIGMA_WRITE_REGISTER_BLOCK(DSP0_ADDR, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
	SIGMA_WRITE_REGISTER_BLOCK(DSP0_ADDR, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
	SIGMA_WRITE_REGISTER_BLOCK(DSP0_ADDR, MOD_SAFELOADMODULE_NUM_SAFELOAD_LOWER_ADDR, 4, num_SafeLoad_Lower);
    SIGMA_WRITE_REGISTER_BLOCK(DSP0_ADDR, MOD_SAFELOADMODULE_NUM_SAFELOAD_UPPER_ADDR, 4, num_SafeLoad_Upper);

	SIGMA_WRITE_REGISTER_BLOCK(DSP1_ADDR, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
	SIGMA_WRITE_REGISTER_BLOCK(DSP1_ADDR, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
	SIGMA_WRITE_REGISTER_BLOCK(DSP1_ADDR, MOD_SAFELOADMODULE_NUM_SAFELOAD_LOWER_ADDR, 4, num_SafeLoad_Lower);
    SIGMA_WRITE_REGISTER_BLOCK(DSP1_ADDR, MOD_SAFELOADMODULE_NUM_SAFELOAD_UPPER_ADDR, 4, num_SafeLoad_Upper);

	SIGMA_WRITE_REGISTER_BLOCK(DSP2_ADDR, MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR, 4, data_SafeLoad);
	SIGMA_WRITE_REGISTER_BLOCK(DSP2_ADDR, MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR, 4, address_SafeLoad);
	SIGMA_WRITE_REGISTER_BLOCK(DSP2_ADDR, MOD_SAFELOADMODULE_NUM_SAFELOAD_LOWER_ADDR, 4, num_SafeLoad_Lower);
    SIGMA_WRITE_REGISTER_BLOCK(DSP2_ADDR, MOD_SAFELOADMODULE_NUM_SAFELOAD_UPPER_ADDR, 4, num_SafeLoad_Upper);
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
