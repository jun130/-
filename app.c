/*
 *********************************************************************************************************
 *
 *
 *                                     ST Microelectronics STM32
 *                                              with the
 *                                   STM3210B-EVAL Evaluation Board
 *
 * Filename      : app.c
 * Version       : V1.10
 * Programmer(s) : BAN
 *********************************************************************************************************
 */


/*
 *********************************************************************************************************
 *                                             INCLUDE FILES
 *********************************************************************************************************
 */

#include <includes.h>
#include  <stm32f10x_tim.h>
#include  <stm32f10x_gpio.h>
#include  <stm32f10x_rcc.h>
#include  <stm32f10x_lib.h>
#include  <stm32f10x_map.h>
#include  <stm32f10x_dma.h>
#include  <stm32f10x_usart.h>
#include  <stm32f10x_adc.h>



/*
 *********************************************************************************************************
 *                                            LOCAL DEFINES
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                       LOCAL GLOBAL VARIABLES
 *********************************************************************************************************
 */

static OS_STK App_TaskStartStk[APP_TASK_START_STK_SIZE];
static OS_STK App_TaskUserIFStk[APP_TASK_USER_IF_STK_SIZE];
static OS_STK App_TaskKbdStk[APP_TASK_KBD_STK_SIZE]; //지워도 됨 원래 예시에 있던 코드//

static OS_STK ADC_TaskStartStk[APP_TASK_START_STK_SIZE]; // ADC 값을 받아오는 태스크를 위해 스택을 잡는다.
static OS_STK LED_TaskStartStk[APP_TASK_START_STK_SIZE]; //LED 작동하는 태스크를 위해 스택 잡음
static OS_STK Cooling_TaskStartStk[APP_TASK_START_STK_SIZE]; //  DC 쿨링 팬을 구동하는 태스크를 위해 스택 잡음
static OS_STK Water_TaskStartStk[APP_TASK_START_STK_SIZE]; // 서보 모터를 구동하는 태스크를 위해 스택 잡음

typedef unsigned int uint32;
typedef unsigned short uint16;

static OS_EVENT      *LightMbox; // 빛의 양을 가지는 메일박스
static OS_EVENT      *TempMbox; // 온도 값을 가지는 메일박스
static OS_EVENT      *SoilMbox; // 토양의 수분을 가지는 메일박스

uint32 ADC_ValueTab[3]; // DMA 를 통해 ADC의 아날로그 값을 저장하는 메모리 공간
uint32 ADC_Value[3]; // 아날로그 값을 실제 사용하는 데이터에 맞게 변환된 메모리 공간
uint16 Pulse[2]= { 2300, 700 }; // 서보모터의 각도 90도, -90도

#if ((APP_OS_PROBE_EN == DEF_ENABLED) && \
	(APP_PROBE_COM_EN == DEF_ENABLED) && \
	(PROBE_COM_STAT_EN == DEF_ENABLED))
static CPU_FP32 App_ProbeComRxPktSpd;
static CPU_FP32 App_ProbeComTxPktSpd;
static CPU_FP32 App_ProbeComTxSymSpd;
static CPU_FP32 App_ProbeComTxSymByteSpd;

static CPU_INT32U App_ProbeComRxPktLast;
static CPU_INT32U App_ProbeComTxPktLast;
static CPU_INT32U App_ProbeComTxSymLast;
static CPU_INT32U App_ProbeComTxSymByteLast;

static CPU_INT32U App_ProbeComCtrLast;
#endif

#if (APP_OS_PROBE_EN == DEF_ENABLED)
static CPU_INT32U App_ProbeCounts;
static CPU_BOOLEAN App_ProbeB1;

#endif


/*
 *********************************************************************************************************
 *                                      LOCAL FUNCTION PROTOTYPES
 *********************************************************************************************************
 */

static void  App_EventCreate(void); // 메일 박스 만드는 함수
static void  App_TaskStart(void *p_arg); // 태스크를 시작하는 함수

static void ADC_Task(void* parg); // 적외선 센서 값 받아오는 함수//
static void LED_Task(void* parg); // 적외선 센서를 받아 LED를 켜는 함수//
static void Cooling_Task(void* parg); // 적외선 센서 값을 바탕으로 모터를 구동하는 함수//
static void Water_Task(void* parg); // 적외선 센서 값을 바탕으로 모터를 구동하는 함수//

static void RCC_Configure(void);
static void GPIO_Configure(void);
static void ADC_Configure(void);
static void DMA_Configure(void);
static void TIM_Init(void);
static void  openDoor(void);
static void  closeDoor(void);
static void  Cooling(void);
static void  Stop(void);
static void  delay(void);

static void  App_DispScr_SignOn(void);
static void  App_DispScr_TaskNames(void);

#if ((APP_PROBE_COM_EN == DEF_ENABLED) || \
	(APP_OS_PROBE_EN == DEF_ENABLED))
static void  App_InitProbe(void);
#endif

#if (APP_OS_PROBE_EN == DEF_ENABLED)
static void  App_ProbeCallback(void);
#endif




/*
 *********************************************************************************************************
 *                                                main()
 *
 * Description : This is the standard entry point for C code.  It is assumed that your code will call
 *               main() once you have performed all necessary initialization.
 *
 * Argument(s) : none.
 *
 * Return(s)   : none.
 *********************************************************************************************************
 */

int  main(void)
{
	CPU_INT08U os_err;

	/* Disable all ints until we are ready to accept them.  */
	BSP_IntDisAll();


	/* Initialize "uC/OS-II, The Real-Time Kernel".         */
	/* IDLE Task와 Statistics Task 생성                      */
	OSInit();

	/* Create the start task.                               */
	/* OSTaskCreatExt()                                     */
	/* OSTaskCreate()와 다르게 Stack을 검사할수 있는 기능을 가짐 */
	os_err = OSTaskCreateExt((void (*)(void *))App_TaskStart, // Task가 수행할 함수
				 (void* )0,                     // Task로 넘겨줄 인자
				 (OS_STK* )&App_TaskStartStk[APP_TASK_START_STK_SIZE - 1],     // Task가 할당될 Stack의 Top을 가리키는 주소
				 (INT8U           )APP_TASK_START_PRIO,// Task의 우선 순위
				 (INT16U          )APP_TASK_START_PRIO,// Task를 지칭하는 유일한 식별자, Task 갯수의 극복을 위해서 사용할 예정, 현재는 우선 순위와 같게끔 설정
				 (OS_STK* )&App_TaskStartStk[0],     // Task가 할당될 Stack의 마지막을 가리키는 주소, Stack 검사용으로 사용
				 (INT32U          )APP_TASK_START_STK_SIZE,// Task Stack의 크기를 의미
				 (void* )0,       // Task Control Block 활용시 사용
				 (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));// Task 생성 옵션 - 초기화 시 Stack을 0으로 채울 것인지, 부동 소수점 연산 장치 사용할 것인지 등 설정

#if (OS_TASK_NAME_SIZE >= 11)
	OSTaskNameSet(APP_TASK_START_PRIO, (CPU_INT08U*)"Start Task", &os_err);
#endif

	OSStart();                                              /* Start multitasking (i.e. give control to uC/OS-II).  */

	return(0);
}

static void  App_TaskStart(void *p_arg)
{

	CPU_INT32U i;
	CPU_INT32U j;
	CPU_INT32U dly;
	CPU_INT08U os_err;

	(void) p_arg;

	BSP_Init(); /* Initialize BSP functions.                            */
	OS_CPU_SysTickInit(); /* Initialize the SysTick.                              */


#if (OS_TASK_STAT_EN > 0)
	OSStatInit();                                           /* Determine CPU capacity.                              */
#endif


#if ((APP_PROBE_COM_EN == DEF_ENABLED) || \
	(APP_OS_PROBE_EN == DEF_ENABLED))
	App_InitProbe();
#endif

	/* Create application events.             */
	/* Task간 통신을 위한 MailBox 생성                        */

	App_EventCreate();
	RCC_Configure();
	GPIO_Configure();
	DMA_Configure();
	ADC_Configure();
	TIM_Init();

	os_err = OSTaskCreateExt((void (*)(void *)) ADC_Task, // Task가 수행할 함수
			(void*) 0,                     // Task로 넘겨줄 인자
			(OS_STK*) &ADC_TaskStartStk[APP_TASK_START_STK_SIZE - 1], // Task가 할당될 Stack의 Top을 가리키는 주소
			(INT8U) 5,     // Task의 우선 순위
			(INT16U) 5, // Task를 지칭하는 유일한 식별자, Task 갯수의 극복을 위해서 사용할 예정, 현재는 우선 순위와 같게끔 설정
			(OS_STK*) &ADC_TaskStartStk[0], // Task가 할당될 Stack의 마지막을 가리키는 주소, Stack 검사용으로 사용
			(INT32U) APP_TASK_START_STK_SIZE,     // Task Stack의 크기를 의미
			(void*) 0,       // Task Control Block 활용시 사용
			(INT16U)(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK)); // Task 생성 옵션 - 초기화 시 Stack을 0으로 채울 것인지, 부동 소수점 연산 장치 사용할 것인지 등 설정

	os_err = OSTaskCreateExt((void (*)(void *)) LED_Task, // Task가 수행할 함수
			(void*) 0,                     // Task로 넘겨줄 인자
			(OS_STK*) &LED_TaskStartStk[APP_TASK_START_STK_SIZE - 1], // Task가 할당될 Stack의 Top을 가리키는 주소
			(INT8U) 6,     // Task의 우선 순위
			(INT16U) 6, // Task를 지칭하는 유일한 식별자, Task 갯수의 극복을 위해서 사용할 예정, 현재는 우선 순위와 같게끔 설정
			(OS_STK*) &LED_TaskStartStk[0], // Task가 할당될 Stack의 마지막을 가리키는 주소, Stack 검사용으로 사용
			(INT32U) APP_TASK_START_STK_SIZE,     // Task Stack의 크기를 의미
			(void*) 0,       // Task Control Block 활용시 사용
			(INT16U)(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK)); // Task 생성 옵션 - 초기화 시 Stack을 0으로 채울 것인지, 부동 소수점 연산 장치 사용할 것인지 등 설정

	os_err = OSTaskCreateExt((void (*)(void *)) Cooling_Task, // Task가 수행할 함수
			(void*) 0,                     // Task로 넘겨줄 인자
			(OS_STK*) &Cooling_TaskStartStk[APP_TASK_START_STK_SIZE - 1], // Task가 할당될 Stack의 Top을 가리키는 주소
			(INT8U) 7,     // Task의 우선 순위
			(INT16U) 7, // Task를 지칭하는 유일한 식별자, Task 갯수의 극복을 위해서 사용할 예정, 현재는 우선 순위와 같게끔 설정
			(OS_STK*) &Cooling_TaskStartStk[0], // Task가 할당될 Stack의 마지막을 가리키는 주소, Stack 검사용으로 사용
			(INT32U) APP_TASK_START_STK_SIZE,     // Task Stack의 크기를 의미
			(void*) 0,       // Task Control Block 활용시 사용
			(INT16U)(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK)); // Task 생성 옵션 - 초기화 시 Stack을 0으로 채울 것인지, 부동 소수점 연산 장치 사용할 것인지 등 설정

	os_err = OSTaskCreateExt((void (*)(void *)) Water_Task, // Task가 수행할 함수
			(void*) 0,                     // Task로 넘겨줄 인자
			(OS_STK*) &Water_TaskStartStk[APP_TASK_START_STK_SIZE - 1], // Task가 할당될 Stack의 Top을 가리키는 주소
			(INT8U) 8,     // Task의 우선 순위
			(INT16U) 8, // Task를 지칭하는 유일한 식별자, Task 갯수의 극복을 위해서 사용할 예정, 현재는 우선 순위와 같게끔 설정
			(OS_STK*) &Water_TaskStartStk[0], // Task가 할당될 Stack의 마지막을 가리키는 주소, Stack 검사용으로 사용
			(INT32U) APP_TASK_START_STK_SIZE,     // Task Stack의 크기를 의미
			(void*) 0,       // Task Control Block 활용시 사용
			(INT16U)(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK)); // Task 생성 옵션 - 초기화 시 Stack을 0으로 채울 것인지, 부동 소수점 연산 장치 사용할 것인지 등 설정

	while (DEF_TRUE) {
		OSTimeDlyHMSM(0, 0, 0, 100);
	}
}

static void ADC_Task(void* parg){ /*  입력받은 ADC를 다른 task에 post해주는 함수 */

        while(DEF_TRUE){

		ADC_Value[0] = ADC_ValueTab[0]; // Light value
		ADC_Value[1] = ADC_ValueTab[1] * 500 / 4095; // Temperature value
		ADC_Value[2] = ADC_ValueTab[2]; // Soil moisture value

        OSMboxPost(LightMbox,(void*)(ADC_Value[0]));
        OSMboxPost(TempMbox,(void*)(ADC_Value[1]));
        OSMboxPost(SoilMbox,(void*)(ADC_Value[2]));


        OSTimeDlyHMSM(0,0,0,100);

        }
}

static void LED_Task(void* parg){

    while(DEF_TRUE){
            CPU_INT08U os_err;
            int ADC_value = (int) OSMboxPend(LightMbox,10,&os_err);


            if(ADC_value >30){  /*  빛이 밝은 경우  */
                 GPIO_ResetBits(GPIOC,GPIO_Pin_8);
            }
            else{
              GPIO_SetBits(GPIOC,GPIO_Pin_8);
            }

              OSTimeDlyHMSM(0,0,0,100);
    }
}

static void Cooling_Task(void* parg){

   while(DEF_TRUE){
           CPU_INT08U os_err;
           int ADC_value = (int) OSMboxPend(TempMbox,10,&os_err);


           if( ADC_value > 60){  /*  온도가 높은 경우 */
                Stop();
                delay();
          }
          else{
                Cooling();
                delay();
          }

             OSTimeDlyHMSM(0,0,0,200);

   }
}

static void Water_Task(void* parg){

   while(DEF_TRUE){

            CPU_INT08U os_err;
            int ADC_value = (int) OSMboxPend(SoilMbox,10,&os_err);

            if( ADC_value > 1000){  /* 수분 부족 */
                openDoor();
                delay();
          }
          else{
                closeDoor();
                delay();
          }
             OSTimeDlyHMSM(0,0,0,300);
   }
}


static void RCC_Configure() {

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC| RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

}


static void GPIO_Configure() {

	/* 조도센서, 온도센서, 토양수분 센서 Pin*/

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*LED Pin*/

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* DC 쿨링 팬  */

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Servo moter Pin*/

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);


}



static void ADC_Configure() {

  ADC_InitTypeDef ADC_InitStructure;

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_NbrOfChannel = 3;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;

  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);

  ADC_DMACmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);

  while(ADC_GetResetCalibrationStatus(ADC1));

  ADC_StartCalibration(ADC1);

  while(ADC_GetCalibrationStatus(ADC1));

  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

static void DMA_Configure() {

	DMA_InitTypeDef DMA_InitStructure;

	// DMA1 channel1 configuration ----------------------------------------------
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32)ADC_ValueTab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 3;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word; // 32bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; // 32bit
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	// Enable DMA1 Channel1
	DMA_Cmd(DMA1_Channel1, ENABLE);

}


static void openDoor() {

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = Pulse[0];
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

}


static void closeDoor() {

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = Pulse[1];
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

}

static void Cooling() {

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = Pulse[0];
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);

}

static void Stop() {

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1500;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);

}

static void TIM_Init() {

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

/* TIM3 */
TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
TIM_ARRPreloadConfig(TIM3, ENABLE);
TIM_Cmd(TIM3, ENABLE);

TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
TIM_TimeBaseStructure.TIM_Prescaler = (unsigned short) (8000000 / 1000000) - 1;
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

/* TIM4 */
TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
TIM_ARRPreloadConfig(TIM4, ENABLE);
TIM_Cmd(TIM4, ENABLE);

TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
TIM_TimeBaseStructure.TIM_Prescaler = (unsigned short) (8000000 / 1000000) - 1;
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

}



static void delay(){
  for(int i=0; i<100000; i++){}

}

static void  App_EventCreate(void)
{
#if (OS_EVENT_NAME_SIZE > 12)
	CPU_INT08U os_err;
#endif

	/* Create MBOX for communication between Kbd and UserIF.*/
	/* Mail Box 생성                                         */
	/* 포인터 크기의 변수를 Task나 Interrupt Service Routine   */
	/* 에서 다른 Task 전달할 때 사용함                         */
	LightMbox = OSMboxCreate((void*) 0);
	TempMbox = OSMboxCreate((void*) 0);
	SoilMbox = OSMboxCreate((void*) 0);

#if (OS_EVENT_NAME_SIZE > 12)
	OSEventNameSet(LightMbox, "LightMbox", &os_err);
        OSEventNameSet(TempMbox, "TempMbox", &os_err);
        OSEventNameSet(SoilMbox, "SoilMbox", &os_err);
#endif
}


/*
 *********************************************************************************************************
 *                                            App_TaskCreate()
 *
 * Description : Create the application tasks.
 *
 * Argument(s) : none.
 *
 * Return(s)   : none.
 *
 * Caller(s)   : App_TaskStart().
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */



/*
 *********************************************************************************************************
 *                                          App_DispScr_SignOn()
 *
 * Description : Display uC/OS-II system information on the LCD.
 *
 * Argument(s) : none.
 *
 * Return(s)   : none.
 *
 * Caller(s)   : App_TaskUserIF().
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

static void  App_DispScr_SignOn(void)
{
}



/*
 *********************************************************************************************************
 *                                          App_DispScr_SignOn()
 *
 * Description : Display uC/OS-II system information on the LCD.
 *
 * Argument(s) : none.
 *
 * Return(s)   : none.
 *
 * Caller(s)   : App_TaskUserIF().
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

static void  App_DispScr_TaskNames(void)
{
}


/*
 *********************************************************************************************************
 *                                             App_InitProbe()
 *
 * Description : Initialize uC/Probe target code.
 *
 * Argument(s) : none.
 *
 * Return(s)   : none.
 *
 * Caller(s)   : App_TaskStart().
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

#if ((APP_PROBE_COM_EN == DEF_ENABLED) || \
	(APP_OS_PROBE_EN == DEF_ENABLED))
static void  App_InitProbe(void)
{
#if (APP_OS_PROBE_EN == DEF_ENABLED)
	(void)App_ProbeCounts;
	(void)App_ProbeB1;


#if ((APP_PROBE_COM_EN == DEF_ENABLED) && \
	(PROBE_COM_STAT_EN == DEF_ENABLED))
	(void)App_ProbeComRxPktSpd;
	(void)App_ProbeComTxPktSpd;
	(void)App_ProbeComTxSymSpd;
	(void)App_ProbeComTxSymByteSpd;
#endif

	OSProbe_Init();
	OSProbe_SetCallback(App_ProbeCallback);
	OSProbe_SetDelay(250);
#endif

#if (APP_PROBE_COM_EN == DEF_ENABLED)
	ProbeCom_Init();                                        /* Initialize the uC/Probe communications module.       */
#endif
}
#endif


/*
 *********************************************************************************************************
 *                                         AppProbeCallback()
 *
 * Description : uC/Probe OS plugin callback.
 *
 * Argument(s) : none.
 *
 * Return(s)   : none.
 *
 * Caller(s)   : uC/Probe OS plugin task.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */

#if (APP_OS_PROBE_EN == DEF_ENABLED)
static void  App_ProbeCallback(void)
{
#if ((APP_PROBE_COM_EN == DEF_ENABLED) && \
	(PROBE_COM_STAT_EN == DEF_ENABLED))
	CPU_INT32U ctr_curr;
	CPU_INT32U rxpkt_curr;
	CPU_INT32U txpkt_curr;
	CPU_INT32U sym_curr;
	CPU_INT32U symbyte_curr;
#endif



	App_ProbeCounts++;

	App_ProbeB1 = BSP_PB_GetStatus(1);




#if ((APP_PROBE_COM_EN == DEF_ENABLED) && \
	(PROBE_COM_STAT_EN == DEF_ENABLED))
	ctr_curr = OSTime;
	rxpkt_curr = ProbeCom_RxPktCtr;
	txpkt_curr = ProbeCom_TxPktCtr;
	sym_curr = ProbeCom_TxSymCtr;
	symbyte_curr = ProbeCom_TxSymByteCtr;

	if ((ctr_curr - App_ProbeComCtrLast) >= OS_TICKS_PER_SEC) {
		App_ProbeComRxPktSpd = ((CPU_FP32)(rxpkt_curr - App_ProbeComRxPktLast) / (ctr_curr - App_ProbeComCtrLast)) * OS_TICKS_PER_SEC;
		App_ProbeComTxPktSpd = ((CPU_FP32)(txpkt_curr - App_ProbeComTxPktLast) / (ctr_curr - App_ProbeComCtrLast)) * OS_TICKS_PER_SEC;
		App_ProbeComTxSymSpd = ((CPU_FP32)(sym_curr - App_ProbeComTxSymLast) / (ctr_curr - App_ProbeComCtrLast)) * OS_TICKS_PER_SEC;
		App_ProbeComTxSymByteSpd = ((CPU_FP32)(symbyte_curr - App_ProbeComTxSymByteLast) / (ctr_curr - App_ProbeComCtrLast)) * OS_TICKS_PER_SEC;

		App_ProbeComCtrLast = ctr_curr;
		App_ProbeComRxPktLast = rxpkt_curr;
		App_ProbeComTxPktLast = txpkt_curr;
		App_ProbeComTxSymLast = sym_curr;
		App_ProbeComTxSymByteLast = symbyte_curr;
	}
#endif
}
#endif


/*
 *********************************************************************************************************
 *                                      App_FormatDec()
 *
 * Description : Convert a decimal value to ASCII (without leading zeros).
 *
 * Argument(s) : pstr            Pointer to the destination ASCII string.
 *
 *               value           Value to convert (assumes an unsigned value).
 *
 *               digits          The desired number of digits.
 *
 * Return(s)   : none.
 *
 * Caller(s)   : various.
 *
 * Note(s)     : none.
 *********************************************************************************************************
 */


/*
 *********************************************************************************************************
 *********************************************************************************************************
 *                                          uC/OS-II APP HOOKS
 *********************************************************************************************************
 *********************************************************************************************************
 */

#if (OS_APP_HOOKS_EN > 0)
/*
 *********************************************************************************************************
 *                                      TASK CREATION HOOK (APPLICATION)
 *
 * Description : This function is cal when a task is created.
 *
 * Argument(s) : ptcb   is a pointer to the task control block of the task being created.
 *
 * Note(s)     : (1) Interrupts are disabled during this call.
 *********************************************************************************************************
 */

void  App_TaskCreateHook(OS_TCB *ptcb)
{
#if ((APP_OS_PROBE_EN == DEF_ENABLED) && \
	(OS_PROBE_HOOKS_EN == DEF_ENABLED))
	OSProbe_TaskCreateHook(ptcb);
#endif
}

/*
 *********************************************************************************************************
 *                                    TASK DELETION HOOK (APPLICATION)
 *
 * Description : This function is called when a task is deleted.
 *
 * Argument(s) : ptcb   is a pointer to the task control block of the task being deleted.
 *
 * Note(s)     : (1) Interrupts are disabled during this call.
 *********************************************************************************************************
 */

void  App_TaskDelHook(OS_TCB *ptcb)
{
	(void)ptcb;
}

/*
 *********************************************************************************************************
 *                                      IDLE TASK HOOK (APPLICATION)
 *
 * Description : This function is called by OSTaskIdleHook(), which is called by the idle task.  This hook
 *               has been added to allow you to do such things as STOP the CPU to conserve power.
 *
 * Argument(s) : none.
 *
 * Note(s)     : (1) Interrupts are enabled during this call.
 *********************************************************************************************************
 */

#if OS_VERSION >= 251
void  App_TaskIdleHook(void)
{
}
#endif

/*
 *********************************************************************************************************
 *                                        STATISTIC TASK HOOK (APPLICATION)
 *
 * Description : This function is called by OSTaskStatHook(), which is called every second by uC/OS-II's
 *               statistics task.  This allows your application to add functionality to the statistics task.
 *
 * Argument(s) : none.
 *********************************************************************************************************
 */

void  App_TaskStatHook(void)
{
}

/*
 *********************************************************************************************************
 *                                        TASK SWITCH HOOK (APPLICATION)
 *
 * Description : This function is called when a task switch is performed.  This allows you to perform other
 *               operations during a context switch.
 *
 * Argument(s) : none.
 *
 * Note(s)     : (1) Interrupts are disabled during this call.
 *
 *               (2) It is assumed that the global pointer 'OSTCBHighRdy' points to the TCB of the task that
 *                   will be 'switched in' (i.e. the highest priority task) and, 'OSTCBCur' points to the
 *                  task being switched out (i.e. the preempted task).
 *********************************************************************************************************
 */

#if OS_TASK_SW_HOOK_EN > 0
void  App_TaskSwHook(void)
{
#if ((APP_OS_PROBE_EN == DEF_ENABLED) && \
	(OS_PROBE_HOOKS_EN == DEF_ENABLED))
	OSProbe_TaskSwHook();
#endif
}
#endif

/*
 *********************************************************************************************************
 *                                     OS_TCBInit() HOOK (APPLICATION)
 *
 * Description : This function is called by OSTCBInitHook(), which is called by OS_TCBInit() after setting
 *               up most of the TCB.
 *
 * Argument(s) : ptcb    is a pointer to the TCB of the task being created.
 *
 * Note(s)     : (1) Interrupts may or may not be ENABLED during this call.
 *********************************************************************************************************
 */

#if OS_VERSION >= 204
void  App_TCBInitHook(OS_TCB *ptcb)
{
	(void)ptcb;
}
#endif

/*
 *********************************************************************************************************
 *                                        TICK HOOK (APPLICATION)
 *
 * Description : This function is called every tick.
 *
 * Argument(s) : none.
 *
 * Note(s)     : (1) Interrupts may or may not be ENABLED during this call.
 *********************************************************************************************************
 */

#if OS_TIME_TICK_HOOK_EN > 0
void  App_TimeTickHook(void)
{
#if ((APP_OS_PROBE_EN == DEF_ENABLED) && \
	(OS_PROBE_HOOKS_EN == DEF_ENABLED))
	OSProbe_TickHook();
#endif
}
#endif
#endif
