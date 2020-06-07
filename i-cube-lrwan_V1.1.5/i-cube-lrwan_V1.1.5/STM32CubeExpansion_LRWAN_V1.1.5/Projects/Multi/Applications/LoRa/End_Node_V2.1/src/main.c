//COOLMEAL
/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.5
  * @date    30-March-2018
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "VL53L1X_api.h"

/* Private typedef -----------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
uint16_t	dev=0x52;
uint8_t dataReady;
uint8_t TimeToSend=0;
uint8_t Aliveframe=0;
uint8_t NobodyFlag;
int PplCounter = 0;
int flag_timer=0;
uint8_t byteData;

// People Counting defines
#define NOBODY 0
#define SOMEONE 1
#define LEFT 0
#define RIGHT 1

#define DIST_THRESHOLD_MAX  1100
int status = 0;
/* Private define ------------------------------------------------------------*/

/*!
 * CAYENNE_LPP is myDevices Application server.
 */
#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_PRESENCE       0x66  
#define LPP_DATATYPE_BAROMETER      0x73
#define LPP_APP_PORT 99
/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_OFF
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_4
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_CONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* GPIO init */
//void MX_GPIO_Init(void);

/* PIR handle */
//static void pirDisabledEnabled( void );

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded ( void );
	
/* LoRa endNode send request*/
static void Send( void );

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent( void );

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LORA_RxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded};

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;
//static uint16_t AppPresenceSensor = RESET; //SET;																								
                                               
static TimerEvent_t TxTimer;

#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent( void );
#endif
/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK};

/* Private functions ---------------------------------------------------------*/
static void MX_I2C1_Init(void);
void MX_GPIO_Init(void);

void ProcessPeopleCountingData(int16_t Distance, uint8_t zone);
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
	
	//uint32_t prim;
	uint8_t RangeStatus;
	uint8_t Bytedata=0;
	uint16_t Distance;
	int center[2] = {167,231}; /* these are the spad center of the 2 8*16 zones */
  int Zone = 0;
  /* STM32 HAL library initialization*/
  HAL_Init();
  
  /* Configure the system clock*/
  SystemClock_Config();
	
	/* Configure the GPIO PIR sensor*/
	MX_GPIO_Init();
	MX_I2C1_Init();
  //MX_USART1_UART_Init();
  /* Configure the debug mode*/
  DBG_Init();
  
  /* Configure the hardware*/
  HW_Init();
  
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  
  /*Disable Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );
	//LPM_SetOffMode(LPM_APPLI_Id , LPM_Enable );
  
  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  //PRINTF("VERSION: %X\n\r", VERSION);
  //DBG_PRINTF("printf\n\r");
  LORA_Join();

    uint8_t sensorState=0;   
		
  TimerInit( &TxLedTimer, OnTimerLedEvent );
  
  TimerSetValue(  &TxLedTimer, 1200000);
  
  
	//LoraStartTx(TX_ON_EVENT);
	
	while (sensorState == 0) {
		PRINTF("salut\n\r");
        status = VL53L1X_BootState(dev, &sensorState);
        HAL_Delay(2);
	}
  /* Initialize and configure the device according to people counting need */
    status = VL53L1X_SensorInit(dev);
		status +=VL53L1X_SetInterruptPolarity(dev,0);
    status += VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
    status += VL53L1X_SetTimingBudgetInMs(dev, 20); /* in ms possible values [20, 50, 100, 200, 500] */
    status += VL53L1X_SetInterMeasurementInMs(dev, 20);
    status += VL53L1X_SetROI(dev, 8, 16); /* minimum ROI 4,4 */
		status =  VL53L1X_StartRanging(dev);
		
		LoraStartTx(TX_ON_TIMER);
		TimerStart( &TxLedTimer);
  while( 1 )
  {
		if (dataReady) {	
				//VL53L1X_CheckForDataReady(dev, &dataReady);
	    
		dataReady = 0;
		VL53L1X_GetRangeStatus(dev, &RangeStatus);
				status += VL53L1_RdByte(dev, 0x010F, &byteData);
			
		status += VL53L1X_GetDistance(dev, &Distance);
		status += VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		//VL53L1_WrByte(dev, 0x0086, 0x01);
		// wait a couple of milliseconds to ensure the setting of the new ROI center for the next ranging is effective
		// otherwise there is a risk that this setting is applied to current ranging (even if timing has expired, the intermeasurement
		// may expire immediately after.
		HAL_Delay(10);
		status = VL53L1X_SetROICenter(dev, center[Zone]);			
			
		 //inject the new ranged distance in the people counting algorithm
		ProcessPeopleCountingData(Distance, Zone);
		Zone++;
		Zone = Zone%2;
				PRINTF("Distance :%i,People : %i,RangeState : %i,Status : %i,ID:%i\n\r", Distance,PplCounter,RangeStatus,status,byteData);
		//Send();
		}
		if(Aliveframe)
			{
			Send();
			TimerStart( &TxLedTimer);
			TimerReset( &TxTimer);
			Aliveframe=0;
			TimeToSend=0;
			NobodyFlag=0;
			}
		else if(TimeToSend && NobodyFlag)
		{
			if(PplCounter)
			{
				TimerReset( &TxLedTimer);
				Send();				
			}
			
			TimerStart( &TxTimer);						
			TimeToSend=0;
			NobodyFlag=0;
			
		}
		
		
		//DISABLE_IRQ();
		//prim = __get_PRIMASK();
		//__disable_irq();
    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending 
     * and cortex will not enter low power anyway  */
		
#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower( );
#endif
		//LPM_EnterLowPower( );
		//ENABLE_IRQ();
		/*if (!prim) {
			__enable_irq();
		
		}*/   
    
    /* USER CODE BEGIN 2 */
		
    /* USER CODE END 2 */
		
}
	}

static void LORA_HasJoined( void )
{
#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF("JOINED\n\r");
#endif
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
}

static void Send( void )
{
  /* USER CODE BEGIN 3 */
  uint8_t batteryLevel;
	
  
  
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    LORA_Join();
    return;
  }
  
  DBG_PRINTF("SEND REQUEST\n\r");
#ifndef CAYENNE_LPP
  int32_t latitude, longitude = 0;
  uint16_t altitudeGps = 0;
#endif
  
#ifdef USE_B_L072Z_LRWAN1
	
	//IRQnb = MSP_GetIRQn( GPIO_PIN_2 );
	
	//HAL_NVIC_DisableIRQ( IRQnb );
	
  //TimerInit( &TxLedTimer, OnTimerLedEvent );
  
  //TimerSetValue(  &TxLedTimer, 30000);
  
  //TimerStart( &TxLedTimer );

#endif 
	//do {

	//}
	//while(!flag_timer);
	//flag_timer=0;
#ifdef CAYENNE_LPP
  uint8_t cchannel=0;
  uint32_t i = 0;
			
	
  batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LPP_APP_PORT;

  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_DIGITAL_INPUT; 
  //AppData.Buff[i++] = batteryLevel*100/254;
	AppData.Buff[i++] = hi2c1.ErrorCode;
	AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_PRESENCE; 
  AppData.Buff[i++] = PplCounter;
	PplCounter=0;
#else  /* not CAYENNE_LPP */

  latitude = sensor_data.latitude;
  longitude= sensor_data.longitude;
  uint32_t i = 0;

  batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LORAWAN_APP_PORT;

#endif  /* CAYENNE_LPP */
  AppData.BuffSize = i;
  
  LORA_send( &AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);
	
	//AppPresenceSensor = SET; 
	
  /* USER CODE END 3 */
}


static void LORA_RxData( lora_AppData_t *AppData )
{
  /* USER CODE BEGIN 4 */
  DBG_PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);

  switch (AppData->Port)
  {
    case 3:
    /*this port switches the class*/
    if( AppData->BuffSize == 1 )
    {
      switch (  AppData->Buff[0] )
      {
        case 0:
        {
          LORA_RequestClass(CLASS_A);
          break;
        }
        case 1:
        {
          LORA_RequestClass(CLASS_B);
          break;
        }
        case 2:
        {
          LORA_RequestClass(CLASS_C);
          break;
        }
        default:
          break;
      }
    }
    break;
    case LORAWAN_APP_PORT:
    if( AppData->BuffSize == 1 )
    {
      AppLedStateOn = AppData->Buff[0] & 0x01;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        LED_Off( LED_BLUE ) ; 
      }
      else
      {
        PRINTF("LED ON\n\r");
        //LED_On( LED_BLUE ) ; 
      }
    }
    break;
  case LPP_APP_PORT:
  {
    AppLedStateOn= (AppData->Buff[2] == 100) ?  0x01 : 0x00;
    if ( AppLedStateOn == RESET )
    {
      PRINTF("LED OFF\n\r");
      LED_Off( LED_BLUE ) ; 
      
    }
    else
    {
      PRINTF("LED ON\n\r");
      //LED_On( LED_BLUE ) ; 
    }
    break;
  }
  default:
    break;
  }
  /* USER CODE END 4 */
}

static void OnTxTimerEvent( void )
{
	TimeToSend=1;
  /*Wait for next tx slot*/
  
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    OnTxTimerEvent();
  }
  else
  {
    /* send everytime button is pushed or PIR sensor is enabled */
    //GPIO_InitTypeDef initStruct={0};
		
    //initStruct.Mode =GPIO_MODE_IT_RISING;
    //initStruct.Pull = GPIO_PULLDOWN;
    //initStruct.Speed = GPIO_SPEED_HIGH;
		
		//DBG_PRINTF("EVENT_LOOP\n\r");
		
		//HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
    //HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send );
		
		//HW_GPIO_Init( GPIOB, GPIO_PIN_2, &initStruct );
		//HW_GPIO_SetIrq( GPIOB, GPIO_PIN_2, 0, Send );
		//HW_GPIO_Init( GPIOA, GPIO_PIN_0, &initStruct ); //Works with wizzilab
		//HW_GPIO_SetIrq( GPIOA, GPIO_PIN_0, 0, Send );  //Works with wizzilab
		
	  }
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_TxNeeded ( void )
{
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent( void )
{
  //LED_Off( LED_RED1 ) ; 
	DBG_PRINTF("TIMER-TEST\n\r");
	//LoraStartTx(TX_ON_EVENT);
	Aliveframe=1;
	PRINTF("STILLALIVE FRAME");
}
#endif

/*static void pirDisabledEnabled( void )
{
	uint32_t prim;
	
	prim = __get_PRIMASK();
	 __disable_irq();
	
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(60000);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(30000);
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
	
	
	if (!prim) {
        __enable_irq();
    }
	
} */

void ProcessPeopleCountingData(int16_t Distance, uint8_t zone) {
    static int PathTrack[] = {0,0,0,0};
    static int PathTrackFillingSize = 1; // init this to 1 as we start from state where nobody is any of the zones
    static int LeftPreviousStatus = NOBODY;
    static int RightPreviousStatus = NOBODY;
    //static int PeopleCount = 0;

    int CurrentZoneStatus = NOBODY;
    int AllZonesCurrentStatus = 0;
    int AnEventHasOccured = 0;

	if (Distance < DIST_THRESHOLD_MAX) {
		// Someone is in !
		CurrentZoneStatus = SOMEONE;
		NobodyFlag=0;
		
	}

	// left zone
	if (zone == LEFT) {

		if (CurrentZoneStatus != LeftPreviousStatus) {
			// event in left zone has occured
			AnEventHasOccured = 1;

			if (CurrentZoneStatus == SOMEONE) {
				AllZonesCurrentStatus += 1;
			}
			// need to check right zone as well ...
			if (RightPreviousStatus == SOMEONE) {
				// event in left zone has occured
				AllZonesCurrentStatus += 2;
			}
			// remember for next time
			LeftPreviousStatus = CurrentZoneStatus;
		}
	}
	// right zone
	else {

		if (CurrentZoneStatus != RightPreviousStatus) {

			// event in left zone has occured
			AnEventHasOccured = 1;
			if (CurrentZoneStatus == SOMEONE) {
				AllZonesCurrentStatus += 2;
			}
			// need to left right zone as well ...
			if (LeftPreviousStatus == SOMEONE) {
				// event in left zone has occured
				AllZonesCurrentStatus += 1;
			}
			// remember for next time
			RightPreviousStatus = CurrentZoneStatus;
		}
	}

	// if an event has occured
	if (AnEventHasOccured) {
		if (PathTrackFillingSize < 4) {
			PathTrackFillingSize ++;
		}

		// if nobody anywhere lets check if an exit or entry has happened
		if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {

			// check exit or entry only if PathTrackFillingSize is 4 (for example 0 1 3 2) and last event is 0 (nobobdy anywhere)
			if (PathTrackFillingSize == 4) {
				// check exit or entry. no need to check PathTrack[0] == 0 , it is always the case

				if ((PathTrack[1] == 1)  && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
					// This an entry
					PplCounter++;
					PRINTF("counter++\n\r");

				} else if ((PathTrack[1] == 2)  && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
					// This an exit
					PplCounter--;
					PRINTF("counter--\n\r");
					
					
				}
			}
			
			PathTrackFillingSize = 1;
			NobodyFlag=1;
		}
		else {
			// update PathTrack
			// example of PathTrack update
			// 0
			// 0 1
			// 0 1 3
			// 0 1 3 1
			// 0 1 3 3
			// 0 1 3 2 ==> if next is 0 : check if exit
			PathTrack[PathTrackFillingSize-1] = AllZonesCurrentStatus;
		}
	}

	// output debug data to main host machine


}

void VL53L1X_Callback(){
	dataReady=1;
	//HAL_Delay(20);
	PRINTF("callback\n\r");
	
}
void MX_GPIO_Init(void)
{
    /* -1- Enable GPIO Clock (to be able to program the configuration registers) */
  
	GPIO_InitTypeDef GPIO_InitStruct = {0};
 /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
	HW_GPIO_Init( GPIOB, GPIO_PIN_2, &GPIO_InitStruct );
	
	HW_GPIO_SetIrq( GPIOB, GPIO_PIN_2, 0, VL53L1X_Callback);
	
	
	
}

static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
