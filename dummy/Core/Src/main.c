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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <stdlib.h>
#include "ReadEncoderV2.h"
#include "QuinticTrajectory.h"
#include "PIDController.h"
#include "KalmanFilterV2.h"
#include "TrayLocalization.h"
#include "Storage.h"
#include "JoyStick.h"
#include "ModBusRTU.h"
#include "Endeffector.h"
//#include "Interrupt.h"
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

/* USER CODE BEGIN PV */
//Operation
OperationState OpState = Init;
OperationVar OpVar = {0};
//Read Encoder
ReadEncoder ReadEncoderParam;
QEI QEIData;
//Quintic Trajectory
QuinticTraj QuinticVar;
float32_t vmax =  163840;	//pps  80%(1200 rpm)
float32_t  amax = 204800;	//pps^2  1500
//int32_t dummyPickpoints[3] = {6827,7851,8875};
//int32_t dummyPlacepoints[3] = {34133,35157,36181};
//PID
PID PositionLoop;
PID VelocityLoop;
//Kalman Filter
Kalman KF;
float32_t Var_Q = 0.798*0.05;	//Measurment Noise
float32_t Var_R = 0.0798*0.1;	//Process Noise
arm_matrix_instance_f32 mat_A, mat_x_hat, mat_x_hat_minus, mat_B, mat_u,eye;
arm_matrix_instance_f32 mat_P, mat_P_minus, mat_Q, mat_GT, mat_G;
arm_matrix_instance_f32 mat_C, mat_R, mat_S, mat_K;
arm_matrix_instance_f32 mat_temp3x3A,mat_temp3x3B, mat_temp3x1,mat_temp1x3, mat_temp1x1;
float32_t ZEstimateVelocity; //For display
//Tray
Tray PickTray;
Tray PlaceTray;
//ModBus
ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[200];
//JoyStick
JoyState SubState = TrayP1;
uint32_t TrayPoint[8];
//EndEff
uint8_t onetime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void joyXjog();
void joyYjog();
void CollectPosition();
int16_t Uint2Int(uint16_t underflow);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //Setup Initial vaules
  InitKalmanStruct(&KF,Var_Q,Var_R);

  InitReadEncoder(&ReadEncoderParam, 1000);

  QuinticSetup(&QuinticVar, vmax, amax);

  PIDSetup(&PositionLoop, 15, 2.5, 0.00001, 10);
  PIDSetup(&VelocityLoop, 5.0, 0.00000001, 0, 0.00003);


  //Timers Start
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);  //Start QEI
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);//Start PWM
  HAL_TIM_Base_Start_IT(&htim5);

  hmodbus.huart = &huart2;
  hmodbus.htim = &htim11;
  hmodbus.slaveAddress = 0x15;
  hmodbus.RegisterSize = 200;
  Modbus_init(&hmodbus, registerFrame);

  HAL_ADC_Start_DMA(&hadc1, VR, 2); // Start ADC

	//EndEff
	TestState = GripperModeOn;
	Stamp = 1;
	TestMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	Modbus_Protocal_Worker();
	static uint32_t timestamp =0;
	static uint32_t Modbustimestamp = 0;
	if (HAL_GetTick() >= Modbustimestamp)
	{
	  Modbustimestamp = HAL_GetTick() + 200;
	  registerFrame[0x00].U16 = 0b0101100101100001;
	  //Update y axis position to basesystem
	  registerFrame[0x11].U16 = (((QEIData.QEIPosition-OpVar.HomePosOffset) * 120) / 8192) * 10;
	}

	if(OpVar.ProxStop == 0)
	{
		switch(OpState)
			{
			case Init:
				OpVar.ControllerEnable = 0;
				OpVar.JoyEnable = 0;
				SetHome(&OpVar);
			break;
			case PreHoming:
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
				QuinticVar.current_velo = 0;
				OpVar.HomingKey = 0;
				if(HAL_GetTick() >= OpVar.waitTime)
				{
					OpVar.waitTime = 0;
					OpState = Homing;
					InitKalmanStruct(&KF,Var_Q,Var_R);
					PIDSetup(&PositionLoop, 15, 2.2, 0.00001, 10);
					PIDSetup(&VelocityLoop, 5.0, 0.00000001, 0, 0.00003);
					//OpVar.HomePosOffset = __HAL_TIM_GET_COUNTER(&htim2);
					QuinticVar.start_pos = __HAL_TIM_GET_COUNTER(&htim2);
					QuinticVar.final_pos = __HAL_TIM_GET_COUNTER(&htim2)*0.5 ;
					OpVar.HomingKey = 0;
				}
				else
				{
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
				}
			break;
			case Homing:
					OpVar.ControllerEnable = 1;
					if(PositionLoop.IsSteady == 1)
					{
						OpVar.HomingKey = 0;
						OpVar.HomePosOffset = __HAL_TIM_GET_COUNTER(&htim2);
						OpState = Buffer;
						OpVar.waitTime = HAL_GetTick() + 1000;
					}
			break;
			case Buffer:
				if(HAL_GetTick() >= OpVar.waitTime)
				{
						OpState = Home_Ok;
				}
				break;
			case Home_Ok:
				OpVar.ControllerEnable = 0;	//Disable Controller
				OpVar.JoyEnable = 1;
				if(registerFrame[0x44].U16 == 0)
				{
					registerFrame[0x40].U16 = 0b0000;	//ResetState
				}
				if(HAL_GetTick() >= timestamp)
				{
					timestamp = HAL_GetTick() + 10;

					if(!onetime){

						TestState = TestModeOn;
						Stamp = 1;
						TestMode();

						onetime = 1;
					}
					//JoyStick
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET);	//Open Joy Pilot lamp

					if(registerFrame[0x01].U16 == 4)	//Home
					{
						OpState = Init;
						OpVar.ControllerEnable = 0;
					}
					//check if basesystem is TrayMode
					else if(registerFrame[0x01].U16 == 8)	//TrayMode
					{
						OpState = PreProcess;
						OpVar.BaseMode = 0;
					}
					//check if basesystem is PointMode
					else if(registerFrame[0x01].U16 == 16)	//PointMode
					{
						OpState = PreProcess;
						OpVar.BaseMode = 1;
					}
				}
			break;

			case PointMode:
				//OpVar.ControllerEnable = 0;
			break;

			case TrayMode:
				//OpVar.ControllerEnable = 0;
			break;

			case PreProcess:
				OpVar.ControllerEnable = 0;
				if(OpVar.BaseMode == 0)	//TrayMode
				{
					TraySetup(&PickTray,TrayPoint[0], TrayPoint[1], TrayPoint[2], TrayPoint[3]);
					TraySetup(&PlaceTray,TrayPoint[4], TrayPoint[5], TrayPoint[6], TrayPoint[7]);
					TrayLocalization(&PickTray);
					TrayLocalization(&PlaceTray);
					QuinticVar.final_pos = PickTray.Holes_Y[0];
					RunX_Axis(PickTray.Holes_X[0], 2500, 3);
					OpVar.task = GoPick;	//current task.
					OpVar.holeInd = 0;
					OpState = ControlLoop;
				}
				else if (OpVar.BaseMode == 1)	//PointMode
				{
					QuinticVar.final_pos = 0; //Point from Basesystem
					OpState = ControlLoop;

				}

			break;

			case ControlLoop:
				OpVar.ControllerEnable = 1;
				OpVar.HomingKey = 2;
				if(PositionLoop.IsSteady == 1)
				{
					float32_t refX_Axis;
					switch(OpVar.task)
					{
					case GoPick:
						refX_Axis = PickTray.Holes_X[OpVar.holeInd] * 10;
						break;
					case GoPlace:
						refX_Axis = PlaceTray.Holes_X[OpVar.holeInd] * 10;
						break;
					}
					if(abs(registerFrame[0x44].U16 - refX_Axis)<=1)
					{
						if(OpVar.BaseMode == 0)	//Tray
						{
							OpVar.waitTime = HAL_GetTick() + 2000;
							OpState = GripperWaiting;
						}
						else if (OpVar.BaseMode == 1)	//Point
						{
							OpState = Home_Ok;
						}
					}
				}
			break;

			case GripperWaiting:
				OpVar.ControllerEnable = 1;
				if( HAL_GetTick() >= OpVar.waitTime)
				{
					OpVar.waitTime = 0;
					switch(OpVar.task)
					{
					case GoPick:	//Next point should be PLACE hole
						QuinticVar.final_pos = PlaceTray.Holes_Y[OpVar.holeInd];
						RunX_Axis(PlaceTray.Holes_X[OpVar.holeInd], 2500, 3);
						OpVar.task = GoPlace;
						OpState = ControlLoop;
					break;
					case GoPlace:	//Next point should be PICK hole
						if(OpVar.holeInd >= 8)
						{
							OpState = Init;
							registerFrame[0x40].U16 = 0b0001;	//Home
							OpVar.ControllerEnable = 0;
							OpVar.testDummy = 1;
						}
						else
						{
							OpVar.holeInd += 1;
							QuinticVar.final_pos = PickTray.Holes_Y[OpVar.holeInd];
							RunX_Axis(PickTray.Holes_X[OpVar.holeInd], 2500, 3);
							OpVar.task = GoPick;
							OpState = ControlLoop;
						}
					break;
					}
				}
			break;

			case WaitingHome:
				OpVar.ControllerEnable = 0;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,35*500);
			break;
			}
		}
	  else if (OpVar.ProxStop == 1)
	  {
		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
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
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		if(GPIO_Pin == GPIO_PIN_11)
		{
			if(OpVar.HomingKey == 1)
			{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
				__HAL_TIM_SET_COUNTER(&htim2,0);
				QEIData.QEIPosition = __HAL_TIM_GET_COUNTER(&htim2);
				OpVar.ProxStop = 0;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,35*500);
				OpState = WaitingHome;
			}
			else if(OpVar.HomingKey == 2)
			{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
				OpVar.ProxStop = 1;
			}
		}
		if (GPIO_Pin == GPIO_PIN_12)
		{
			if(OpVar.HomingKey == 1)
			{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
				OpVar.HomingKey = 0;		//Disable Proximety Homing
				OpVar.ProxStop = 0;
				OpVar.waitTime = HAL_GetTick() + 1000;
				OpVar.ControllerEnable = 1;	//Enable Controller
				QuinticVar.current_pos = __HAL_TIM_GET_COUNTER(&htim2);	//Dummy PID
				OpState = PreHoming;
			}
			else if(OpVar.HomingKey == 2)
			{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
				OpVar.ProxStop = 1;
			}
		}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim4)
	{
		QEIGetFeedback(&QEIData, 2500);
		KF.z = QEIData.QEIVelocity;
		kalman_filter();
		ZEstimateVelocity = KF.x_hat[1];

		if(OpVar.ControllerEnable == 1)
		{
			QuinticRun(&QuinticVar,PositionLoop.ESS,0.0004);
			CascadeLoop(&PositionLoop, &VelocityLoop, QEIData.QEIPosition, KF.x_hat[1],&QuinticVar, 3);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,abs(VelocityLoop.U));
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, VelocityLoop.MotorDir);
		}

		if(OpVar.JoyEnable == 1)
		{
			CheckJoystick();
			CheckButton();
			//joyXjog();
			joyYjog();
			CollectPosition();
		}
	}
}
void joyXjog()
{

    if (Joy.X == 1) {
        registerFrame[0x40].U16 = 0x0008;
    } else if (Joy.X == -1) {
        registerFrame[0x40].U16 = 0x0004;
    } else if (Joy.X == 0) {
        registerFrame[0x40].U16 = 0x0000;
    }

    //set home
    if (Joy.status == 1) {
        registerFrame[0x40].U16 = 0x0001;
    }
}
void joyYjog()
{
    if (Joy.Y == 1) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 20000);
    } else if (Joy.Y == -1) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 20000);
    } else if (Joy.Y == 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    }

//    if (Joy.status == 1) {
//
//    }

}
void CollectPosition()
{

    static uint8_t PreReset = 0;
    if (PreReset == 0 && Joy.status == 3) {
        SubState = TrayP1;
        for (uint8_t i = 0; i <= 7; i++) {
            TrayPoint[i] = 0;
        }
    }

    static uint8_t PreRec = 0;

    switch (SubState) {
    case TrayP1:

        if (Joy.status == 2 && PreRec == 0) {
            SubState = TrayP2;
            TrayPoint[0] = Uint2Int(registerFrame[0x44].U16);
            TrayPoint[1] = __HAL_TIM_GET_COUNTER(&htim2);
        }
        break;
    case TrayP2:

        if (Joy.status == 2 && PreRec == 0) {
            SubState = TrayP3;
            TrayPoint[2] = Uint2Int(registerFrame[0x44].U16);
            TrayPoint[3] = __HAL_TIM_GET_COUNTER(&htim2);
        }
        break;
    case TrayP3:

        if (Joy.status == 2 && PreRec == 0) {
            SubState = TrayP4;
            TrayPoint[4] = Uint2Int(registerFrame[0x44].U16);
            TrayPoint[5] = __HAL_TIM_GET_COUNTER(&htim2);

        }
        break;
    case TrayP4:

        if (Joy.status == 2 && PreRec == 0) {
            TrayPoint[6] = Uint2Int(registerFrame[0x44].U16);
            TrayPoint[7] = __HAL_TIM_GET_COUNTER(&htim2);
        }
        break;

    }

    PreReset = Joy.status;
    PreRec = Joy.status;
}

int16_t Uint2Int(uint16_t underflow)
{
    uint16_t integer = 0;
    int16_t bitwise = 0;

    if (underflow > 40000) {
        integer = (uint16_t) (UINT16_MAX - underflow + 1);
        bitwise = ~integer;
    } else {
        bitwise = underflow;
    }
    return bitwise;

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
