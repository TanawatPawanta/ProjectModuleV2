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
int32_t TrayPoint[8];
//EndEff
uint8_t PreState = 0;
uint32_t Delay = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void joyXjog();
void joyYjog();
void CollectPosition();
void EndeffectorCheck();
int16_t Uint2Int(uint16_t underflow);
void PickPlace();
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

//	//EndEff
	testState = SoftReset;
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
	if((KF.x_hat[1] >= 1000) || (KF.x_hat[1] <= -1000))
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
	}

	if (HAL_GetTick() >= Modbustimestamp)
	{
	  Modbustimestamp = HAL_GetTick() + 200;
	  registerFrame[0x00].U16 = 0b0101100101100001;
	  //Update y axis position to basesystem
	  registerFrame[0x11].U16 = ((((QEIData.QEIPosition-OpVar.HomePosOffset) * 120) / 8192 ) + 1) * 10;
	  registerFrame[0x12].U16 = (KF.x_hat[1] * 120/8192) * 10;
	  registerFrame[0x13].U16 =	((KF.x_hat[1] - KF.x_hat_minus[1])* 120/8192) * 2500 * 10;
		static uint32_t Tray_Delay = 0; // Tray delay counter
		//Set Pick Tray
		if(registerFrame[0x01].U16 == 1)
		{
			if(registerFrame[0x02].U16 == 1){
					testState = TestOn;
					Stamp = 1;
					TestMode();
		}
		registerFrame[0x10].U16 = 1; //Jog of Pick Tray
		if(TrayPoint[3] != 0) // Y of Right edge have value of Pick tray
		{
			//For use
			TraySetup(&PickTray,TrayPoint[0]/10.0, TrayPoint[1], TrayPoint[2]/10.0, TrayPoint[3]); //Input value to calculate

			//For test
//				TraySetup(&PickTray,700/10.0, 19084, 1128/10.0, 17380);

			TrayLocalization(&PickTray); //Find 9 Holes of Pick Tray
			registerFrame[0x20].U16 = PickTray.ForBaseOriginX; //Upload X Pick
			registerFrame[0x21].U16 = PickTray.ForBaseOriginY; //Upload Y Pick
			registerFrame[0x22].U16 = PickTray.ForBaseOrientation; //Upload Angle Pick
			registerFrame[0x10].U16 = 0; //Finish Command
			registerFrame[0x01].U16 = 0; //Finish Command
		}

			/*OpVar.Tray_SetTo = 1;
			Tray_Delay = HAL_GetTick() + 350; // Add delay
			registerFrame[0x10].U16 = 1; //Jog Pick Set
			//registerFrame[0x10].U16 = 0; //Jog Pick Reset*/
		}
		//Set Place Tray
		if(registerFrame[0x01].U16 == 2)
		{
			registerFrame[0x10].U16 = 2; //Jog of Place Tray
			if(TrayPoint[7] != 0){ // Y of Right edge have value of Place tray
				//For use
				TraySetup(&PlaceTray,TrayPoint[4]/10.0, TrayPoint[5], TrayPoint[6]/10.0, TrayPoint[7]);
				//For test
//				TraySetup(&PlaceTray,-1083/10.0, 31685, -1369/10.0, 29007);
				TrayLocalization(&PlaceTray);
				registerFrame[0x23].U16 = PlaceTray.ForBaseOriginX; //Upload X Place
				registerFrame[0x24].U16 = PlaceTray.ForBaseOriginY; //Upload Y Place
				registerFrame[0x25].U16 = PlaceTray.ForBaseOrientation; //Upload Angle Place
				registerFrame[0x10].U16 = 0; //Finish Command
				registerFrame[0x01].U16 = 0; //Finish Command
				//Reset Tray Point
				SubState = TrayP1;
				OpVar.Tray_SetTo = 0;
				for (uint8_t i = 0; i <= 7; i++)
				{
					TrayPoint[i] = 0;
				}
				//turn off laser
				if(registerFrame[0x02].U16 == 0){
					testState = TestOff;
					Stamp = 1;
					TestMode();
				}
			}
			/*OpVar.Tray_SetTo = 1;
			Tray_Delay = HAL_GetTick() + 350; // Add delay
			registerFrame[0x10].U16 = 2; //Jog Place Set
			//registerFrame[0x10].U16 = 0; //Jog Place Reset*/
		}
		//Delay for Jog
		if (HAL_GetTick() >= Tray_Delay && OpVar.Tray_SetTo != 0){
			Tray_Delay = HAL_GetTick() + 350; // Add delay
			registerFrame[0x10].U16 = 0;
			OpVar.Tray_SetTo = 0;
		}
	}
	if(OpVar.ProxStop == 0)
	{
		switch(OpState)
			{
			case Init:
				OpVar.ControllerEnable = 0;
				OpVar.JoyEnable = 0;
				PositionLoop.IsSteady = 0;
				for (uint8_t i = 0; i <= 7; i++)
				{
					TrayPoint[i] = 0;
				}
				OpVar.SetPointY_Axis = 0;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET);
				SetHome(&OpVar);
			break;
			case PreHoming:
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
				OpVar.ControllerEnable = 1;
				QuinticVar.current_velo = 0;
				OpVar.HomingKey = 0;
				if(HAL_GetTick() >= OpVar.waitTime)
				{
					OpVar.waitTime = 0;
					InitKalmanStruct(&KF,Var_Q,Var_R);
					PIDSetup(&PositionLoop, 15, 2.2, 0.00001, 10);
					PIDSetup(&VelocityLoop, 5.0, 0.00000001, 0, 0.00003);

					QuinticVar.start_pos = __HAL_TIM_GET_COUNTER(&htim2);
					QuinticVar.final_pos = OpVar.MaxWorkspace*0.5 - 5;
					OpVar.HomingKey = 0;	//Turn off Proximety
					OpState = Homing;
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
						OpState = Buffer;
						OpVar.waitTime = HAL_GetTick() + 1000;
					}
			break;
			case Buffer:
				if(HAL_GetTick() >= OpVar.waitTime)
				{
					if(OpVar.RunTrayMode == 1)
					{
						OpState = PreProcess;
						OpVar.RunTrayMode = 0;
					}
					else if(OpVar.RunTrayMode == 0)
					{
						OpState = Home_Ok;
						registerFrame[0x10].U16 = 0;
						OpVar.ControllerEnable = 0;
					}
				}
				break;
			case Home_Ok:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, RESET);  // turn off point mode pilot lamp
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, RESET); // turn off tray mode pilot lamp

				if(registerFrame[0x10].U16 == 32){
					registerFrame[0x10].U16 = 0;
				}
				OpVar.ControllerEnable = 0;	//Disable Controller
				OpVar.JoyEnable = 1;
				if(registerFrame[0x44].U16 == 0)
				{
					registerFrame[0x40].U16 = 0b0000;	//ResetState
				}
				if(HAL_GetTick() >= timestamp)
				{
					timestamp = HAL_GetTick() + 10;

					//JoyStick
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET);	//Open Joy Pilot lamp

					//set home from UI
					if(registerFrame[0x01].U16 == 4)
					{
						registerFrame[0x10].U16 = 4;
						registerFrame[0x40].U16 = 1; //sethome x axis
						OpState = Init;

						registerFrame[0x01].U16 = 0; //reset state
					}

					//PointMode from UI After press RUN
					if(registerFrame[0x01].U16 == 16)
					{
						registerFrame[0x10].U16 = 32;	//Update Y axis Status(Gopoint)
					   //start run status of x axis
						RunX_Axis(registerFrame[0x30].U16, 2500, 2);
						//(Code here)    //assign goal point y to trajectory y axis and PID
						OpVar.SetPointY_Axis = (Uint2Int(registerFrame[0x31].U16)*8192/(120*10)) + OpVar.HomePosOffset;
						OpVar.BaseMode = 1;
						OpState = PreProcess;
					}

					//TrayMode from UI After press RUN
					if(registerFrame[0x01].U16 == 8)
					{
						testState = GripOn;
						Stamp = 1;
						TestMode();
						OpVar.BaseMode = 0;
						OpState = Init;
						OpVar.RunTrayMode = 1;

					}

					//endeffector
					EndeffectorCheck();
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
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET);
				if(OpVar.BaseMode == 0)	//TrayMode
				{
					QuinticVar.final_pos = PickTray.Holes_Y[0];
					RunX_Axis(PickTray.Holes_X[0]*10, 2500, 3);
					OpVar.task = GoPick;	//current task.
					OpVar.holeInd = 0;
					PositionLoop.IsSteady = 0;
					OpState = ControlLoop;
				}
				else if (OpVar.BaseMode == 1)	//PointMode
				{
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, SET);
					QuinticVar.final_pos = OpVar.SetPointY_Axis; //Point from Basesystem
					PositionLoop.IsSteady = 0;
					OpState = ControlLoop;

				}
			break;

			case ControlLoop:
				OpVar.ControllerEnable = 1;
				OpVar.JoyEnable = 0;
				OpVar.HomingKey = 2;
				if(PositionLoop.IsSteady == 1)
				{
					float32_t refX_Axis;
					if(OpVar.BaseMode == 0)
					{
						switch(OpVar.task)
						{
						case GoPick:
							refX_Axis = PickTray.Holes_X[OpVar.holeInd] * 10;
							break;
						case GoPlace:
							refX_Axis = PlaceTray.Holes_X[OpVar.holeInd] * 10;
							break;
						}
					}
					else if (OpVar.BaseMode == 1)
					{
						refX_Axis = Uint2Int(registerFrame[0x30].U16);
					}
					if(abs(Uint2Int(registerFrame[0x44].U16) - refX_Axis) <= 0.1)
					{
						if(OpVar.BaseMode == 0)	//Tray
						{
							PickPlace();
							OpVar.waitTime = HAL_GetTick() + 2000;
							OpState = GripperWaiting;
						}
						else if (OpVar.BaseMode == 1)	//Point
						{
							OpState = Home_Ok;
							registerFrame[0x01].U16 = 0;

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
						RunX_Axis(PlaceTray.Holes_X[OpVar.holeInd]*10, 2500, 3);
						OpVar.task = GoPlace;
						OpState = ControlLoop;
					break;
					case GoPlace:	//Next point should be PICK hole
						if(OpVar.holeInd >= 8)
						{
							OpState = Init;
							registerFrame[0x40].U16 = 0b0001;	//Home
							OpVar.ControllerEnable = 0;
							OpVar.RunTrayMode = 0;
							registerFrame[0x01].U16 = 0;

						}
						else
						{
							OpVar.holeInd += 1;
							QuinticVar.final_pos = PickTray.Holes_Y[OpVar.holeInd];
							RunX_Axis(PickTray.Holes_X[OpVar.holeInd]*10, 2500, 3);
							OpVar.task = GoPick;
							OpState = ControlLoop;
						}
					break;
					}
				}
			break;

			case WaitingHome:
				OpVar.ControllerEnable = 0;
				OpVar.HomingKey = 1;
				OpState = WaitingHome;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,30*500);
			break;
			case EmergencyStop:
				OpVar.ControllerEnable = 0;
				OpVar.JoyEnable = 0;
				 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
				 if(OpVar.EmerPress == 0)
				 {
					 testState = EmerExit;
					 Stamp = 1;
					 TestMode();
					 OpState = Init;
				 }
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
		if (GPIO_Pin == GPIO_PIN_2) //Emergency Switch
		{
			OpVar.countExt ++;
		}
		if(GPIO_Pin == GPIO_PIN_11)
		{
			if(OpVar.HomingKey == 1)
			{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
				__HAL_TIM_SET_COUNTER(&htim2,0);
				QEIData.QEIPosition = __HAL_TIM_GET_COUNTER(&htim2);
				OpVar.ProxStop = 0;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,30*500);
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
				OpVar.HomingKey = 0;		//Disable Proximity Homing
				OpVar.ProxStop = 0;
				if(OpState == WaitingHome)
				{
					OpVar.waitTime = HAL_GetTick() + 1000;
					QuinticVar.current_pos = __HAL_TIM_GET_COUNTER(&htim2);	//Dummy PID
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 50*500);
					OpState = PreHoming;
					OpVar.MaxWorkspace = __HAL_TIM_GET_COUNTER(&htim2);
					OpVar.HomePosOffset = __HAL_TIM_GET_COUNTER(&htim2) * 0.5;
					OpVar.ControllerEnable = 1;	//Enable Controller
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 0);
				}
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
		//Read FeedBack
		QEIGetFeedback(&QEIData, 2500);
		KF.z = QEIData.QEIVelocity;
		kalman_filter();
		ZEstimateVelocity = KF.x_hat[1];
		OpVar.CurrentY_Position = QEIData.QEIPosition*120/8192;
		OpVar.CurrentY_Velocity = ZEstimateVelocity*120/8192;
		OpVar.CurrentY_Accelleration = ((KF.x_hat[1] - KF.x_hat_minus[1])* 120/8192) * 2500;
		//Emergency Check
		static uint32_t EmrTimestamp = 0;
		if(HAL_GetTick() >= EmrTimestamp)
		{
			EmrTimestamp = HAL_GetTick() + 10;
			if((OpVar.countExt - OpVar.PreCountExt) == 1)
			{
				OpVar.EmerPress = 0;
				OpVar.PreCountExt = OpVar.countExt;
			}
			else if((OpVar.countExt - OpVar.PreCountExt) == 0)
			{
				OpVar.EmerPress = OpVar.EmerPress;
				OpVar.PreCountExt = OpVar.countExt;
			}
			else
			{
				OpVar.EmerPress = 1;
				OpVar.PreCountExt = OpVar.countExt;
				OpState = EmergencyStop;
				testState = EmerTrig;
				Stamp = 1;
				TestMode();

			}
		}
		//Cascade Controller
		if(OpVar.ControllerEnable == 1)
		{
			QuinticRun(&QuinticVar,PositionLoop.ESS,0.0004);
			CascadeLoop(&PositionLoop, &VelocityLoop, QEIData.QEIPosition, KF.x_hat[1],&QuinticVar, 3);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,abs(VelocityLoop.U));
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, VelocityLoop.MotorDir);
		}
		//JoyStick
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
void joyXjog() {

	if(enableX){
		if (Joy.X == 1) {
			registerFrame[0x40].U16 = 0x0008;
		} else if (Joy.X == -1) {
			registerFrame[0x40].U16 = 0x0004;
		} else if (Joy.X == 0) {
			registerFrame[0x40].U16 = 0x0000;
		}
	}
}

void joyYjog() {

	static uint8_t toggleX = 0;
	switch(StateSpeed){
	case 0:
		speed = 30;
		break;
	case 1:
		speed = 60;
		break;
	case 2:
		speed = 90;
		toggleX = 0;
		break;
	case 3:
		if(enableX == 1 && toggleX == 0){
			enableX = 0;
			toggleX = 1;
		}
		else if(enableX == 0 && toggleX == 0){
			enableX = 1;
			toggleX = 1;
		}
		break;
	}


	if (Joy.Y == 1) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed * 500);
	} else if (Joy.Y == -1) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, SET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed * 500);
	} else if (Joy.Y == 0) {

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	}

}
void CollectPosition() {

	CheckButton();
	CheckJoystick();
	joyXjog();
	joyYjog();

	static uint8_t PreReset = 0;
	if (PreReset == 0 && Joy.status == 3) {
		SubState = TrayP1;
		for (uint8_t i = 0; i <= 7; i++) {
			TrayPoint[i] = 0;
		}
	}

	static uint8_t PreSpeed = 0;
	if(Joy.status == 1 && PreSpeed == 0){
		StateSpeed = (StateSpeed + 1 ) % 4 ;
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
		} else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) {
			SubState = TrayP1;
		}
		break;

	}

	PreSpeed = Joy.status;
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

void EndeffectorCheck(){

	if(registerFrame[0x02].U16 == 1 && PreState != 1){
		testState = TestOn;
		Stamp = 1;
		TestMode();
	}

	else if(registerFrame[0x02].U16 == 0 && PreState == 1){
		testState = TestOff;
		Stamp = 1;
		TestMode();
	}

	else if(registerFrame[0x02].U16 == 2 && PreState != 2){
		testState = GripOn;
		Stamp = 1;
		TestMode();
	}

	else if(registerFrame[0x02].U16 == 0 && PreState == 2){
		testState = GripOff;
		Stamp = 1;
		TestMode();
	}

	else if(registerFrame[0x02].U16 == 6 && PreState != 6){
		testState = PickUp;
		Stamp = 1;
		TestMode();
		Delay = HAL_GetTick() + 2000;
	}

	else if(registerFrame[0x02].U16 == 10 && PreState != 10){
		testState = PlaceDown;
		Stamp = 1;
		TestMode();
		Delay = HAL_GetTick() + 2000;
	}

	if(HAL_GetTick() >= Delay && (registerFrame[0x02].U16 == 6 || registerFrame[0x02].U16 == 10)){
		registerFrame[0x02].U16 = 2;
	}

	PreState = registerFrame[0x02].U16;
}

void PickPlace(){
	static uint8_t PPState = 0;

	switch(PPState){
	case 0:
		testState = PickUp;
		Stamp = 1;
		TestMode();

		PPState = PPState + 1;
		break;
	case 1:
		testState = PlaceDown;
		Stamp = 1;
		TestMode();

		PPState = (PPState + 1) % 2;
		break;
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
