/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "stm32l1xx_hal.h"
//**Incluir librerias para el dispositivo MPU6050
#include "TJ_MPU6050.h"
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
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//Variables para leer de la terminal
int len=0;
int buff[256];

//Variables modo 1
uint8_t exit_mode1=0;
uint32_t angle;
uint32_t valor_adc;

//Variables modo 2
uint8_t exit_mode2=0;
uint16_t CCR_motor[4] = {1200, 2500, 3700, 5000};
uint8_t angles_motor[4] = {0, 60, 120, 180};
uint8_t distances[4];
uint8_t largest_position;
uint8_t largest;
uint8_t done_measuring;
uint16_t MPU_values[4];
uint16_t apto;

//Variable EXTIs
uint8_t msg;
uint8_t program_out;

//Variables Ultrasonidos
uint32_t time_elapsed;
uint8_t distance;
uint8_t time_captured;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//**Datos para el MPU
//Estructuras de los datos obtenidos del MPU
RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myGyroScaled;

//**Funcion para delay ultrasonidos
void delay(uint32_t delay){


			if(delay < 2) delay = 2;
			TIM7->ARR = delay - 1;
			TIM7-> CNT = 0;
			TIM7->CR1 |= 1;
			while((TIM7->SR&0x0001) != 1);
			TIM7->SR &= ~(0x0001);

		}

//**Funcion para leer de la terminal

		int readMsg(void)
		{

		 for( len =0;len<256;len++)
		 {
			while(!(USART2->SR & USART_SR_RXNE));
		   buff[len]= USART2->DR;
			if (buff[len]==10 || buff[len]==13) break; //Condicion si se preciona enter
		 }
		 return len; //Devuelve el string de caracteres sin incluir el enter
		}


//**Funcion para printf

		#ifdef __GNUC__
		/* With GCC, small printf (option LD Linker->Libraries->Small printf
		 set to 'Yes') calls __io_putchar() */
		#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
		#define GETCHAR_PROTOTYPE int __io_getchar(void)
		#else
		#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
		#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
		#endif /* __GNUC__ */

		PUTCHAR_PROTOTYPE {
			/* Place your implementation of fputc here */
			/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
			HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
			return ch;
		}
		GETCHAR_PROTOTYPE {
			/* Place your implementation of fgetc here */
			/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
			char ch;
			HAL_UART_Receive(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
			return ch;
		}

//**Funcion de notas musicales

		void Correcto_zelda(void){
			  //Cambiar el valor del PSC del timer para ajustar el tono del sonido
			  TIM4->PSC = 319-1;
			  //Setear el CCR a 50 para ajustar el volumen del sonido
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  TIM4->PSC = 338-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  TIM4->PSC = 402-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  TIM4->PSC = 801-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  TIM4->PSC = 603-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  TIM4->PSC = 380-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  TIM4->PSC = 301-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  TIM4->PSC = 239-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(400);
			  htim4.Instance->CCR1 = 0;
		}

		void final_notificacion(void){
			  TIM4->PSC = 478-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(100);
			  htim4.Instance->CCR1 = 0;
			  HAL_Delay(100);
			  TIM4->PSC = 478-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  TIM4->PSC = 425-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  htim4.Instance->CCR1 = 0;
			  HAL_Delay(100);
			  TIM4->PSC = 425-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  TIM4->PSC = 379-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  htim4.Instance->CCR1 = 0;
			  HAL_Delay(100);
			  TIM4->PSC = 379-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  TIM4->PSC = 425-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(100);
			  TIM4->PSC = 379-1;
			  htim4.Instance->CCR1 = 50;
			  HAL_Delay(200);
			  htim4.Instance->CCR1 = 0;
		}

//**Codigo para el standby mode

		void gotoSleep(void){

			  //Activar el PWR Control Clock
			  RCC->APB1ENR|= (RCC_APB1ENR_PWREN);

			  //Activar el bit SLEEDEEP del Cortex System Control Register
			  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

			  //Seleccionar Standby mode
			  PWR->CR |= PWR_CR_PDDS;

			  //Borrar el wake up flag
			  PWR->CR |= PWR_CR_CWUF;

			  //Activar el wakeup pin
			  PWR->CR |= (PWR_CSR_EWUP2);

			  //Activar wait for interrupt
			  __WFI();

		}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  //**Codigo para comprobar si ha despertado del modo StandBy
  //Si la flag del modo stand by esta activada, se pasa la condicion
  if(((PWR->CSR)&(PWR_CSR_SBF))){
	  PWR->CR |= PWR_CR_CWUF;
	  PWR->CR |= PWR_CR_CSBF;

	  printf("Saliendo del StandBy Mode\n");
	  HAL_Delay(1500);

  }

  //Inicializar modulo MPU6050 y I2C
   MPU6050_Init(&hi2c1);

   //MPU6050 Config
   myMpuConfig.Accel_Full_Scale = AFS_SEL_2g;
   myMpuConfig.ClockSource = Internal_8MHz;
   myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
   myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
   myMpuConfig.Sleep_Mode_Bit = 1; // 1 = sleep mode, 0= normal mode
   MPU6050_Config(&myMpuConfig);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  do{
		  //Presentación del menu al usuario
		  printf("Bienvenido al menu, elija el modo de funcionamiento. Si quieres salir escribir introducir '0':\n");
		  printf("Modo 1 - '1'\n");
		  printf("Modo 2 - '2'\n");
		  printf("StanbBy Mode - '3'\n");

		  //Leer el input del usuario
		  readMsg();

		  //MODO 1 MANUAL
		  if(buff[0] == 49){
			  exit_mode1 = 0;
			  printf("Modo 1 seleccionado\nPorfavor, posicione el sensor en la posición deseada. Una vez posicionado presione el botón usuario");
			  do{

				  	//Activar la señal PWM para el motor y el ADC para obtener una señal del potenciometro
			  		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
			  		HAL_ADC_Start(&hadc);
			  		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
			  		valor_adc = HAL_ADC_GetValue(&hadc);
			  		HAL_ADC_Stop(&hadc);

			  		//Formula para que el motor obtenga los valores correctos
			  		TIM2->CCR3 = ((valor_adc*0.92796)+1200); // MAX CCR3 = 5000, MIN CCR3 = 1200
			  		angle = (valor_adc*0.04395);

			  	  		if(msg == 1){
			  	  			//Enviar el trigger para que mida el ultrasonidos
				  			 msg = 0;
				  			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				  			 delay(3);
				  			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				  			 delay(10);
				  			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
				  			 HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
			  	  		}

			  	  		if(time_captured){
			  	  			//Calcular distancia y mostrarla con el angulo
			  	  			distance = (time_elapsed)/58;
			  	  			printf("Distance(cm):%hu\nAngle(degrees):%hu\n", distance, angle);
			  	  			time_captured = 0;
			  	  			//Notificacion por sonido
			  	  			HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
			  	  			final_notificacion();
			  	  			//Preguntar si quiere volver a medir
			  	  			printf("Quiere volver a medir una nueva distancia manualmente? 1 -> Si, 0 -> No\n");
			  	  			readMsg();
			  	  			if(buff[0] == 48){
			  	  				exit_mode1 = 1;
			  	  			}
			  	  		}
			  }while(!exit_mode1);
		  }

		  //Parar la señal PWM y el ADC
		  HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
		  HAL_ADC_Stop(&hadc);

		  //MODO 2 AUTOMATICO
		  if(buff[0] == 50){
			 exit_mode2=0;
			 //Se despierta el MPU
			 myMpuConfig.Sleep_Mode_Bit = 0;
		  	 printf("Modo 2 seleccionado\n");

		  	 do{
		  		 done_measuring = 0;
		  		while(!done_measuring){
		  			printf("Tomando medidas...\n");
		  			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
		  			for(int i=0;i<4;i++){
		  				//Posicionar el motor
		  				TIM2->CCR3=CCR_motor[i];

		  				/*CODIGO PARA EL MPU, PERO NO FUNCIONA CORRECTAMENTE
		  				Recopilar datos del MPU
		  				for(int i=0;i<4;i++){
							HAL_Delay(8);
							MPU6050_Get_Accel_Scale(&myAccelScaled);
							MPU6050_Get_Gyro_Scale(&myGyroScaled);
		  					MPU_values[i] = myGyroScaled.x;
		  				}

		  				//Analizar que el MPU alcance una velocidad mayor de 250 grados por segundo
		  				for(int i=0;i<4;i++){
		  					if(MPU_values[i]>250){
		  						apto = 1;
		  					}
		  				}

		  				//Si no lo alcanza es que hay un obstaculo
		  				if(!apto){
		  					printf("Error");
		  					break;
		  				}
		  				apto = 0; */

		  				HAL_Delay(1500);
		  				//Enviar señal al trigger
		  				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		  				delay(3);
		  				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		  				delay(10);
		  				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		  				HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
		  				HAL_Delay(500);
		  				//Guardar la medida obtenida
		  				if(time_captured){
		  				//Calcular distancia y guardarlax
							distance = (time_elapsed)/58;
							distances[i]= distance;
							time_captured = 0;

		  				}
		  			}
		  			done_measuring = 1;
		  		}


		  		largest = distances[0];

		  		//Buscar la posicion con mas distancia medida por el ultrasonidos
		  		for(int i=0;i<4;i++){
		  			//Si la medida a comparar es mayor que la anterior, toma lugar como la mayor medida
		  			if(distances[i]>largest){
		  				largest = distances[i];
		  				largest_position = i;
		  			}
		  		}

		  		//Posicionar el motor en la posicion con mayor distancia
		  		TIM2->CCR3=CCR_motor[largest_position];
		  		//Mostrar resultado y pedir si se desea hacer otro escaneo
		  		printf("Para volver a escanear, introducir '1'. Para salir al menu, introducir '0'\nDistance: %d\tAngle: %d\n",largest, angles_motor[largest_position]);
		  		//Notificar por sonido
		  		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
		  		Correcto_zelda();
		  		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
		  		sreadMsg();
		  		if(buff[0] == 48){
		  			exit_mode2 = 1;
		  			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
		  		}
		  	}while(!exit_mode2);
		  	 //Se duerme el MPU
		  	myMpuConfig.Sleep_Mode_Bit = 1;
		 }

		  if(buff[0] == 51){
			  //Se anuncia que el programa esta entrando en Stand By
			  printf("Entrando StandBy Mode\n");
			  gotoSleep();
		  }

		  if(buff[0] == 48){
			  program_out = 1;
		  }

	  }while(!program_out);

	  printf("Gracias por usar el sonar");

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 492-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255-1;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 31;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Pin_Trigger_GPIO_Port, Pin_Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Boton_Usuari_Pin */
  GPIO_InitStruct.Pin = Boton_Usuari_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Boton_Usuari_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Pin_Trigger_Pin */
  GPIO_InitStruct.Pin = Pin_Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Pin_Trigger_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
