/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_flash_ex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{ EVENT_NONE /*ninguna entrada*/,
	/* Botón de encendido y apagado */
	EVENT_ONOFF,

	/* Colores predeterminados */
	EVENT_R /*luz roja (botón "R")*/,
	EVENT_G /*luz verde (botón "G")*/,
	EVENT_B /*luz azul (botón "B")*/,
	EVENT_W /*luz blanca (botón "W")*/,

	/* Colores personalizados */
	EVENT_1 /*color personalizado 1*/,
	EVENT_2 /*color personalizado 2*/,
	EVENT_3 /*color personalizado 3*/,

	/* Modos de ciclo */
	EVENT_CICLO1 /*modo arcoíris (botón "RAINBOW")*/,
	EVENT_CICLO2 /*modo discoteca (botón "RAVE")*/,
	EVENT_CICLO3 /*modo fuego (botón "EMBER")*/,

	/* Botón de ahorro de energía */
	EVENT_MODO_AHORRO,

	/* Botones de configuración de color */
	EVENT_CONFIG_COLOR /*botón "CONGIG"*/,
	EVENT_SAVE_COLOR /*botón "SAVE"*/
} event_t;

typedef enum{
	OFF /*luz apagada*/,

	NORMAL /*luz encendida,
	el usuario puede seleccionar distintos colores y modificar el brillo*/,

	CICLO /*modos de cambio de color*/,

	AHORRO /*modo ahorro de energía,
	este estado es como el normal, salvo que hay un timer que hará que la luz
	se apague o vaya perdiendo intensidad después de un tiempo*/,

	CONFIG_COLOR /*modo de configuración,
	en ese modo el usuario puede ajustar el color de la luz para crear
	una combinación personalizada y guardarla*/

} states_t;

typedef enum{
	/* Subestados del modo NORMAL */
	NORMAL_R /*luz roja*/,
	NORMAL_G /*luz verde*/,
	NORMAL_B /*luz azul*/,
	NORMAL_W /*luz blanca (estado inicial)*/,
	NORMAL_1 /*color personalizado 1*/,
	NORMAL_2 /*color personalizado 2*/,
	NORMAL_3 /*color personalizado 3*/,

	/* Subestados del modo CICLO */
	CICLO_1 /*modo arcoíris*/,
	CICLO_2 /*modo discoteca*/,
	CICLO_3 /*modo fuego*/,

	/* Subestados del modo CONFIG_COLOR */
	CONFIG_1 /*el usuario configura el color 1*/,
	CONFIG_2 /*el usuario configura el color 2*/,
	CONFIG_3 /*el usuario configura el color 3*/
}substates_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CANAL_R TIM_CHANNEL_1 //rojo
#define CANAL_G TIM_CHANNEL_2 //verde
#define CANAL_B TIM_CHANNEL_3 // azul
#define CANAL_POT1 ADC_CHANNEL_4
#define CANAL_POTR ADC_CHANNEL_1
#define CANAL_POTG ADC_CHANNEL_2
#define CANAL_POTB ADC_CHANNEL_3
#define MAX_ADC 4095
#define MAX_PWM 999
#define factor_brillo_min 0.0
#define factor_brillo_max 1.0
#define USER_FLASH_ADDR 0x8060000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NO_VOLATIL __attribute__((__section__(".user_data_flash")))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* Variables para el control de los estados */
volatile event_t entrada = EVENT_NONE;
volatile states_t modo = NORMAL;
volatile substates_t sub_modo = NORMAL_W;
volatile uint8_t timer_iniciado = 0;

/* Variables del potenciómetro que ajusta el brillo */
volatile uint32_t pot1_raw = 0; // valor medido por el ADC (0-4095)
volatile uint32_t pot1_scale = 0; // valor escalado (0-999)
volatile float factor_brillo = 0.0; // factor de ajuste del brillo (0.1-0.9)

/* Variables de color */
volatile uint32_t brillo_R = 666; // Intensidad del canal rojo (0-999)
volatile uint32_t brillo_G = 666; // Intensidad del canal verde (0-999)
volatile uint32_t brillo_B = 666; // Intensidad del canal azul (0-999)

/* Variables no volátiles */
uint32_t NO_VOLATIL CP1_R;
uint32_t NO_VOLATIL CP1_G;
uint32_t NO_VOLATIL CP1_B;

uint32_t NO_VOLATIL CP2_R;
uint32_t NO_VOLATIL CP2_G;
uint32_t NO_VOLATIL CP2_B;

uint32_t NO_VOLATIL CP3_R;
uint32_t NO_VOLATIL CP3_G;
uint32_t NO_VOLATIL CP3_B;

/* Temporizador */
TIM_HandleTypeDef htim2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* Gestión de las entradas analógicas */
uint32_t scale(uint32_t raw);
uint32_t leer_canal_adc(uint32_t ch);
void ajuste_brillo();

/* Gestión de la memoria flash */
void guardar_datos();

/* Modos de funcionamiento */
void modo_normal();
void modo_ciclo();
void modo_ahorro();
void modo_config();

/* Lógica de la máquina de estados */
void state_decod();

/* Prototipo de inicialización del temporizador*/
void MX_TIM2_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t scale(uint32_t raw){
	return (uint32_t)((raw*MAX_PWM))/MAX_ADC;
}
uint32_t leer_canal_adc(uint32_t ch){
	// Para leer un canal específico del ADC

	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ch;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK){
		uint32_t raw = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		return raw;
	}
	HAL_ADC_Stop(&hadc1);

	return -1;
}

void ajuste_brillo(){
	/* Leemos el ADC por polling y calculamos el factor */
	//pot1_raw = leer_adc(&hadc1);
	pot1_raw = leer_canal_adc(CANAL_POT1);
	pot1_scale = scale(pot1_raw);
	factor_brillo = (float)pot1_scale / MAX_PWM;
	/* Para que no se salga del intervalo */
	if(factor_brillo < factor_brillo_min) factor_brillo = factor_brillo_min;
	if(factor_brillo > factor_brillo_max) factor_brillo = factor_brillo_max;
}

void guardar_datos(){
	if(modo!=CONFIG_COLOR) return;

	// Guardamos una copia de la configuración actual
	uint32_t vector_config[9] = {
			CP1_R,CP1_G,CP1_B,
			CP2_R,CP2_G,CP2_B,
			CP3_R,CP3_G,CP3_B
	};

	uint32_t PAGError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.Sector = FLASH_SECTOR_7;
	EraseInitStruct.NbSectors = 1;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();

	/* Borramos el sector entero */
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGError)!=HAL_OK){
		HAL_FLASH_Lock();
		return;
	}

	switch(sub_modo){
	case CONFIG_1:
		vector_config[0] = brillo_R;
		vector_config[1] = brillo_G;
		vector_config[2] = brillo_B;
		break;
	case CONFIG_2:
		vector_config[3] = brillo_R;
		vector_config[4] = brillo_G;
		vector_config[5] = brillo_B;
		break;
	case CONFIG_3:
		vector_config[6] = brillo_R;
		vector_config[7] = brillo_G;
		vector_config[8] = brillo_B;
		break;
	default:break;
	}

	/* Escribimos los datos */
	for(uint32_t i=0;i<(sizeof(vector_config)/sizeof(vector_config[0]));i++){
		uint32_t addr = (uint32_t) USER_FLASH_ADDR + 4*i;
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, vector_config[i])!=HAL_OK){
			HAL_FLASH_Lock();
			return;
		}

	}

	HAL_FLASH_Lock();

}

void modo_normal(){
	/* Configuración de los colores del modo normal */
	switch(sub_modo){
	case NORMAL_W:
	  // asignar valores RGB para la luz BLANCA
		brillo_R = 999;
		brillo_G = 999;
		brillo_B = 999;
	  break;
	case NORMAL_R:
	  // asignar valores RGB para la luz ROJA
		brillo_R = 999;
		brillo_G = 0;
		brillo_B = 0;
	  break;
	case NORMAL_G:
	  // asignar valores RGB para la luz VERDE
		brillo_R = 0;
		brillo_G = 999;
		brillo_B = 0;
	  break;
	case NORMAL_B:
	  // asignar valores RGB para la luz AZUL
		brillo_R = 0;
		brillo_G = 0;
		brillo_B = 999;
	  break;
	case NORMAL_1:
	  // asignar valores RGB del color personalizado 1
		brillo_R = CP1_R;
		brillo_G = CP1_G;
		brillo_B = CP1_B;
	  break;
	case NORMAL_2:
	  // asignar valores RGB del color personalizado 2
		brillo_R = CP2_R;
		brillo_G = CP2_G;
		brillo_B = CP2_B;
	  break;
	case NORMAL_3:
	  // asignar valores RGB del color personalizado 3
		brillo_R = CP3_R;
		brillo_G = CP3_G;
		brillo_B = CP3_B;
	  break;
	default:break;
	}

}
void modo_ciclo(){
	switch(sub_modo){
	case CICLO_1:
	  // llamar a la función del ciclo 1 (modo arcoíris)
	  break;
	case CICLO_2:
	  // llamar a la función del ciclo 2 (modo discoteca)
	  break;
	case CICLO_3:
	  // llamar a la función del ciclo 3 (modo fuego)
	  break;
	default:break;
	}
}
void modo_ahorro(){
	if (!timer_iniciado)
	  {
	    MX_TIM2_Init();                         // Inicializar TIM2
	    HAL_TIM_Base_Start_IT(&htim2);         // Iniciar con interrupción
	    timer_iniciado = 1;                    // Iniciado
	  }

	//Bajar el brillo a la mitad
	  brillo_R = brillo_R / 2;
	  brillo_G = brillo_G / 2;
	  brillo_B = brillo_B / 2;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2 && modo == AHORRO){
    HAL_TIM_Base_Stop_IT(&htim2);  // Detiene el temporizador
    modo = OFF;                    // Apaga la luz
    sub_modo = NORMAL_W;           // Modo blanco por defecto
    timer_iniciado = 0;            // Permite reiniciar el temporizador si vuelvo a ahorro
  }
}


void modo_config(){
	brillo_R = scale(leer_canal_adc(CANAL_POTR));
	brillo_G = scale(leer_canal_adc(CANAL_POTG));
	brillo_B = scale(leer_canal_adc(CANAL_POTB));
}
void state_decod(){

	switch(entrada){
		  case EVENT_NONE:break;

		  case EVENT_ONOFF:
			  if(modo == OFF){
				  modo = NORMAL;
				  sub_modo = NORMAL_W;
			  }
			  else modo = OFF;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_R:
			  if(modo == CICLO) modo = NORMAL;
			  if(modo == NORMAL) sub_modo = NORMAL_R;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_G:
			  if(modo == CICLO) modo = NORMAL;
			  if(modo == NORMAL) sub_modo = NORMAL_G;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_B:
			  if(modo == CICLO) modo = NORMAL;
			  if(modo == NORMAL) sub_modo = NORMAL_B;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_W:
			  if(modo == CICLO) modo = NORMAL;
			  if(modo == NORMAL) sub_modo = NORMAL_W;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_1:
			  if(modo == CICLO) modo = NORMAL;
			  if(modo == NORMAL) sub_modo = NORMAL_1;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_2:
			  if(modo == CICLO) modo = NORMAL;
			  if(modo == NORMAL) sub_modo = NORMAL_2;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_3:
			  if(modo == CICLO) modo = NORMAL;
			  if(modo == NORMAL) sub_modo = NORMAL_3;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_CICLO1:
			  if(modo == NORMAL) modo = CICLO;
			  if(modo == CICLO) sub_modo = CICLO_1;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_CICLO2:
			  if(modo == NORMAL) modo = CICLO;
			  if(modo == CICLO) sub_modo = CICLO_2;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_CICLO3:
			  if(modo == NORMAL) modo = CICLO;
			  if(modo == CICLO) sub_modo = CICLO_3;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_MODO_AHORRO:
			  if(modo == NORMAL) modo = AHORRO;
			  else modo = NORMAL;
			  entrada = EVENT_NONE;
			  break;

		  case EVENT_CONFIG_COLOR:
			  /*if(modo == NORMAL) modo = CONFIG_COLOR;
			  if(sub_modo == NORMAL_1) sub_modo = CONFIG_1;
			  else if(sub_modo == NORMAL_2) sub_modo = CONFIG_2;
			  else if(sub_modo == NORMAL_3) sub_modo = CONFIG_3;
			  else sub_modo = CONFIG_1;
			  entrada = EVENT_NONE;*/

			  /* Si se pulsa en modo config, se sale sin guardar */
			  if(modo == CONFIG_COLOR){
				  modo = NORMAL;
				  switch(sub_modo){
				  case CONFIG_1:sub_modo = NORMAL_1;break;
				  case CONFIG_2:sub_modo = NORMAL_2;break;
				  case CONFIG_3:sub_modo = NORMAL_3;break;
				  default:sub_modo = NORMAL_W;break;
				  }
			  }

			  /* Si se pulsa en modo normal se pasa al modo config */
			  else if(modo == NORMAL){
				  modo = CONFIG_COLOR;
				  switch(sub_modo){
				  case NORMAL_1:sub_modo = CONFIG_1;break;
				  case NORMAL_2:sub_modo = CONFIG_2;break;
				  case NORMAL_3:sub_modo = CONFIG_3;break;
				  default:sub_modo = CONFIG_1;break;
				  }
			  }

			  entrada = EVENT_NONE;

			  break;

		  case EVENT_SAVE_COLOR:
			  if(modo == CONFIG_COLOR){
				  guardar_datos();
				  modo = NORMAL;
				  sub_modo = NORMAL_W;
			  }
			  entrada = EVENT_NONE;
			  break;

		  default:
			  entrada = EVENT_NONE;
			  break;
		  }

		  /* Actualización de las salidas en función del estado */
		  switch(modo){
		  case OFF:
			  // poner todos los canales a 0 (luz apagada)
			  brillo_R = 0;
			  brillo_G = 0;
			  brillo_B = 0;

			  break;

		  case NORMAL:
			  /* Función del modo normal */
			  modo_normal();
			  /* Ajuste del brillo */
			  ajuste_brillo();

			  break;

		  case CICLO:
			  /* Función del modo ciclo */
			  modo_ciclo();
			  /* Ajuste del brillo */
			  ajuste_brillo();

			  break;

		  case AHORRO:
			  /* Función del modo ahorro */
			  modo_ahorro();
			  /* Ajuste del brillo */
			  ajuste_brillo();

			  break;

		  case CONFIG_COLOR:
			  // llamar a la función de configuración del color
			  modo_config();
			  /* Ajute del brillo */
			  factor_brillo = 1;

			  break;

		  default:break;
		  }

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*Se plantean las diferentes entradas para el switch case
	 * en base a botones y cada botón activa un evento distinto*/

	/*Para obtener los antirrebotes se utilizará la función HAL_GetTick
	 * que medira el tiempo en milisegundos para rehabilitar la entrada*/

	static uint32_t last_tick = 0;
	uint32_t now = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_0)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_ONOFF;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_1)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_R;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_2)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_G;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_3)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_B;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_4)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_W;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_5)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_1;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_6)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_2;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_7)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_3;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_8)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_CICLO1;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_9)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_CICLO2;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_10)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_CICLO3;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_11)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_MODO_AHORRO;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_12)
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_CONFIG_COLOR;
    		last_tick = now;
    	}
    }

    if (GPIO_Pin == GPIO_PIN_13) entrada = EVENT_SAVE_COLOR;
    {
    	if ((now - last_tick) > 50) {
    		entrada = EVENT_SAVE_COLOR;
    		last_tick = now;
    	}
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
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_PWM_Init(&htim1);
  HAL_TIM_PWM_Start(&htim1, CANAL_R);
  HAL_TIM_PWM_Start(&htim1, CANAL_G);
  HAL_TIM_PWM_Start(&htim1, CANAL_B);
  __HAL_TIM_MOE_ENABLE(&htim1);

  HAL_ADC_Start(&hadc1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* Actualización del estado en función de las entradas */
	  state_decod();

	  __HAL_TIM_SET_COMPARE(&htim1,CANAL_R,brillo_R*factor_brillo);
	  __HAL_TIM_SET_COMPARE(&htim1,CANAL_G,brillo_G*factor_brillo);
	  __HAL_TIM_SET_COMPARE(&htim1,CANAL_B,brillo_B*factor_brillo);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99999;
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

  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PC4 PC5 PC7 PC8
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
