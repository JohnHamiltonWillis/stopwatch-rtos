/*
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?".  Have you defined configASSERT()?  *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *   Investing in training allows your team to be as productive as       *
     *   possible as early as possible, lowering your overall development    *
     *   cost, and enabling you to bring a more robust product to market     *
     *   earlier than would otherwise be possible.  Richard Barry is both    *
     *   the architect and key author of FreeRTOS, and so also the world's   *
     *   leading authority on what is the world's most popular real time     *
     *   kernel for deeply embedded MCU designs.  Obtaining your training    *
     *   from Richard ensures your team will gain directly from his in-depth *
     *   product knowledge and years of usage experience.  Contact Real Time *
     *   Engineers Ltd to enquire about the FreeRTOS Masterclass, presented  *
     *   by Richard Barry:  http://www.FreeRTOS.org/contact
     *                                                                       *
    ***************************************************************************

    ***************************************************************************
     *                                                                       *
     *    You are receiving this top quality software for free.  Please play *
     *    fair and reciprocate by reporting any suspected issues and         *
     *    participating in the community forum:                              *
     *    http://www.FreeRTOS.org/support                                    *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
/* BSP includes. */
#include "xtmrctr.h"
#include "xgpio.h" /* General Purpose I/O */

#define TIMER_ID	1
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define TIMER_CHECK_THRESHOLD	9
/* Seven Seg Data */
#define SEVEN_SEG_DATA *(volatile int *)(XPAR_AXI_GPIO_0_BASEADDR)
#define SEVEN_SEG_ID XPAR_AXI_GPIO_0_DEVICE_ID
#define ANODE_CHANNEL 1 /* GPIO port for 7-segment display anodes */
#define CATHODE_CHANNEL 2 /* GPIO port for 7-segment display cathodes */
/* Priorities at which the tasks are created. */
#define mainHUNDREDS_TASK_PRIORITY ( tskIDLE_PRIORITY + 6 )
#define mainTENS_TASK_PRIORITY ( tskIDLE_PRIORITY + 5 )
#define mainONES_TASK_PRIORITY ( tskIDLE_PRIORITY + 4 )
#define mainTENTHS_TASK_PRIORITY ( tskIDLE_PRIORITY + 3 )
#define mainHUNDREDTHS_TASK_PRIORITY ( tskIDLE_PRIORITY + 2 )
/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_PERIOD_MS constant. */
#define mainHUNDREDS_FREQUENCY_MS ( 100000 / portTICK_PERIOD_MS ) // 100 s
#define mainTENS_FREQUENCY_MS ( 10000 / portTICK_PERIOD_MS ) // 10 s
#define mainONES_FREQUENCY_MS ( 1000 / portTICK_PERIOD_MS ) // 1 s
#define mainTENTHS_FREQUENCY_MS ( 100 / portTICK_PERIOD_MS ) // 1/10 s
#define mainHUNDREDTHS_FREQUENCY_MS ( 10 / portTICK_PERIOD_MS ) // 1/100 s
/* The following constants describe the timer instance used in this application.
They are defined here to easily change all the needed parameters in one place. */
#define TIMER_DEVICE_ID XPAR_TMRCTR_0_DEVICE_ID
#define TIMER_FREQ_HZ XPAR_TMRCTR_0_CLOCK_FREQ_HZ
#define TIMER_INTR_ID XPAR_INTC_0_TMRCTR_0_VEC_ID

/*-----------------------------------------------------------*/

/* The Tx and Rx tasks as described at the top of this file. */
static void prvTxTask( void *pvParameters );
static void prvRxTask( void *pvParameters );
static void vTimerCallback( TimerHandle_t pxTimer );
/*-----------------------------------------------------------*/
/* The tasks as described in the comments at the top of this file. */
static void prvHundredsTask( void *pvParameters );
static void prvTensTask( void *pvParameters );
static void prvOnesTask( void *pvParameters );
static void prvTenthsTask( void *pvParameters );
static void prvHundredthsTask( void *pvParameters );
// Other Function declarations
unsigned char prvSevSegDecoder( unsigned char number);
/* The queue used by the Tx and Rx tasks, as described at the top of this
file. */
static TaskHandle_t xTxTask;
static TaskHandle_t xRxTask;
static QueueHandle_t xQueue = NULL;
static TimerHandle_t xTimer = NULL;
char HWstring[15] = "Hello World";
long RxtaskCntr = 0;
/* Global Variables */
XGpio Gpio0; /* GPIO Device driver instance */
/* Structures that hold the state of the various peripherals used by this demo.
These are used by the Xilinx peripheral driver API functions. */
static XTmrCtr xTimer0Instance;
/* Each digit incremental. */
static unsigned char ucHundreds = 0;
static unsigned char ucTens = 0;
static unsigned char ucOnes = 0;
static unsigned char ucTenths = 0;
static unsigned char ucHundredths = 0;
volatile int SEVSEG_DIG_0;
volatile int SEVSEG_DIG_1;
volatile int SEVSEG_DIG_2;
volatile int SEVSEG_DIG_3;

int main( void )
{
	int Status;
	xil_printf( "Starting FreeRTOS stopwatch example main\r\n" );
	Status = XGpio_Initialize(&Gpio0, SEVEN_SEG_ID);
	if (Status != XST_SUCCESS) { return XST_FAILURE; }
	/* Set the direction for the Seven Segment displays to output */
	XGpio_SetDataDirection(&Gpio0, ANODE_CHANNEL, 0x00);
	XGpio_SetDataDirection(&Gpio0, CATHODE_CHANNEL, 0x00);
	// set the stopwatch to 0000
	SEVSEG_DIG_0 = prvSevSegDecoder(0);
	SEVSEG_DIG_1 = prvSevSegDecoder(0);
	SEVSEG_DIG_2 = prvSevSegDecoder(0);
	SEVSEG_DIG_3 = prvSevSegDecoder(0);
	/* Start the two tasks as described in the comments. */
	xTaskCreate( prvHundredsTask, "Hundreds", configMINIMAL_STACK_SIZE, NULL,
	mainHUNDREDS_TASK_PRIORITY, NULL );
	xTaskCreate( prvTensTask, "Tens", configMINIMAL_STACK_SIZE, NULL,
	mainTENS_TASK_PRIORITY, NULL );
	xTaskCreate( prvOnesTask, "Ones", configMINIMAL_STACK_SIZE, NULL,
	mainONES_TASK_PRIORITY, NULL );
	xTaskCreate( prvTenthsTask, "Tenths", configMINIMAL_STACK_SIZE, NULL,
	mainTENTHS_TASK_PRIORITY, NULL );
	xTaskCreate( prvHundredthsTask, "Hundredths", configMINIMAL_STACK_SIZE,
	NULL, mainHUNDREDTHS_TASK_PRIORITY, NULL );

	vTaskStartScheduler();
}

static void prvTxTask( void *pvParameters )
{
const TickType_t x1second = pdMS_TO_TICKS( DELAY_1_SECOND );

	for( ;; )
	{
		/* Delay for 1 second. */
		vTaskDelay( x1second );

		/* Send the next value on the queue.  The queue should always be
		empty at this point so a block time of 0 is used. */
		xQueueSend( xQueue,			/* The queue being written to. */
					HWstring, /* The address of the data being sent. */
					0UL );			/* The block time. */
	}
}

/*-----------------------------------------------------------*/
static void prvRxTask( void *pvParameters )
{
char Recdstring[15] = "";

	for( ;; )
	{
		/* Block to wait for data arriving on the queue. */
		xQueueReceive( 	xQueue,				/* The queue being read. */
						Recdstring,	/* Data is read into this address. */
						portMAX_DELAY );	/* Wait without a timeout for data. */

		/* Print the received data. */
		xil_printf( "Rx task received string from Tx task: %s\r\n", Recdstring );
		RxtaskCntr++;
	}
}

/*-----------------------------------------------------------*/
static void vTimerCallback( TimerHandle_t pxTimer )
{
	long lTimerId;
	configASSERT( pxTimer );

	lTimerId = ( long ) pvTimerGetTimerID( pxTimer );

	if (lTimerId != TIMER_ID) {
		xil_printf("FreeRTOS Hello World Example FAILED");
	}

	/* If the RxtaskCntr is updated every time the Rx task is called. The
	 Rx task is called every time the Tx task sends a message. The Tx task
	 sends a message every 1 second.
	 The timer expires after 10 seconds. We expect the RxtaskCntr to at least
	 have a value of 9 (TIMER_CHECK_THRESHOLD) when the timer expires. */
	if (RxtaskCntr >= TIMER_CHECK_THRESHOLD) {
		xil_printf("FreeRTOS Hello World Example PASSED");
	} else {
		xil_printf("FreeRTOS Hello World Example FAILED");
	}

	vTaskDelete( xRxTask );
	vTaskDelete( xTxTask );
}
static void prvHundredsTask( void *pvParameters ) {
	TickType_t xNextWakeTime;
	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();
	for( ;; ) {
	/* Place this task in blocked state until it's time to run again.
	Block time specified in ticks, the constant converts ticks to ms.
	While in Blocked state this task will not consume any CPU time.*/
	vTaskDelayUntil( &xNextWakeTime, mainHUNDREDS_FREQUENCY_MS );
	ucHundreds++;
	if (ucHundreds == 10)
	ucHundreds = 0;
	SEVSEG_DIG_3 = prvSevSegDecoder(ucHundreds);
	}
}

static void prvTensTask( void *pvParameters ) {
	TickType_t xNextWakeTime;
	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();
	for( ;; ) {
	/* Place this task in blocked state until it's time to run again.
	Block time specified in ticks, the constant converts ticks to ms.
	While in Blocked state this task will not consume any CPU time.*/
	vTaskDelayUntil( &xNextWakeTime, mainTENS_FREQUENCY_MS );
	ucTens++;
	if (ucTens == 10)
	ucTens = 0;
	SEVSEG_DIG_2 = prvSevSegDecoder(ucTens);
	}
}

static void prvOnesTask( void *pvParameters ) {
	TickType_t xNextWakeTime;
	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();
	for( ;; ) {
	/* Place this task in blocked state until it's time to run again.
	Block time specified in ticks, the constant converts ticks to ms.
	While in Blocked state this task will not consume any CPU time.*/
	vTaskDelayUntil( &xNextWakeTime, mainONES_FREQUENCY_MS );
	ucOnes++;
	if (ucOnes == 10)
	ucOnes = 0;
	// xil_printf( "Ones digit is %1d\r\n", (int)ucOnes );
	SEVSEG_DIG_1 = prvSevSegDecoder(ucOnes);
	}
}

static void prvTenthsTask( void *pvParameters ) {
	TickType_t xNextWakeTime;
	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();
	for( ;; ) {
	/* Place this task in blocked state until it's time to run again.
	Block time specified in ticks, the constant converts ticks to ms.
	While in Blocked state this task will not consume any CPU time.*/
	vTaskDelayUntil( &xNextWakeTime, mainTENTHS_FREQUENCY_MS );
	ucTenths++;
	if (ucTenths == 10)
	ucTenths = 0;
	SEVSEG_DIG_0 = prvSevSegDecoder(ucTenths);
	}
}

static void prvHundredthsTask( void *pvParameters ) {
	TickType_t xNextWakeTime;
	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();
	for( ;; ) {
	/* Place this task in blocked state until it's time to run again.
	Block time specified in ticks, the constant converts ticks to ms.
	While in Blocked state this task will not consume any CPU time.*/
	vTaskDelayUntil( &xNextWakeTime, mainHUNDREDTHS_FREQUENCY_MS );
	ucHundredths++;
	if (ucHundredths == 4)
	ucHundredths = 0;
	if (ucHundredths==0) {
	XGpio_DiscreteWrite(&Gpio0, ANODE_CHANNEL, 0xE); //1110
	XGpio_DiscreteWrite(&Gpio0, CATHODE_CHANNEL, SEVSEG_DIG_0);
	} else if (ucHundredths==1) {
	XGpio_DiscreteWrite(&Gpio0, ANODE_CHANNEL, 0xD); //1101
	XGpio_DiscreteWrite(&Gpio0, CATHODE_CHANNEL, (SEVSEG_DIG_1 & 0x7F));
	} else if (ucHundredths==2) {
	XGpio_DiscreteWrite(&Gpio0, ANODE_CHANNEL, 0xB); //1011
	XGpio_DiscreteWrite(&Gpio0, CATHODE_CHANNEL, SEVSEG_DIG_2);
	} else if (ucHundredths==3) {
	XGpio_DiscreteWrite(&Gpio0, ANODE_CHANNEL, 0x7); //0111
	XGpio_DiscreteWrite(&Gpio0, CATHODE_CHANNEL, SEVSEG_DIG_3);
	}
	}
}

/* Function to create a 7 segment decoder.
* Send an unsigned char number and it will return 8 bits to
* drive 7 segment cathodes including decimal point.
* Assumes 7 segment works with logic 0 for ON and 1 for OFF */
unsigned char prvSevSegDecoder( unsigned char number) {
	switch (number) {
	case 0: return ~0x3F;
	case 1: return ~0x06;
	case 2: return ~0x5B;
	case 3: return ~0x4F;
	case 4: return ~0x66;
	case 5: return ~0x6D;
	case 6: return ~0x7D;
	case 7: return ~0x07;
	case 8: return ~0x7F;
	case 9: return ~0x67;
	default: return ~0x00;
	}
}

void vApplicationMallocFailedHook( void ) {
taskDISABLE_INTERRUPTS();
for( ;; );
}
void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName
) {
( void ) pcTaskName;
( void ) pxTask;
taskDISABLE_INTERRUPTS();
for( ;; );
}
void vApplicationIdleHook( void ) { }
void vApplicationTickHook( void ) { }
void vApplicationSetupTimerInterrupt( void ) {
portBASE_TYPE xStatus;
const unsigned char ucTimerCounterNumber = ( unsigned char ) 0U;
const unsigned long ulCounterValue = ( ( TIMER_FREQ_HZ / configTICK_RATE_HZ
) - 1UL );
extern void vPortTickISR( void *pvUnused );
/* Initialize the timer/counter. */
xStatus = XTmrCtr_Initialize( &xTimer0Instance, TIMER_DEVICE_ID );
if( xStatus == XST_SUCCESS ) {
/* Install the tick interrupt handler as the timer ISR.
The xPortInstallInterruptHandler() API function must be used */
xStatus = xPortInstallInterruptHandler( TIMER_INTR_ID, vPortTickISR,
NULL );
}
if( xStatus == pdPASS ) {
/* Enable the timer interrupt in the interrupt controller.
The vPortEnableInterrupt() API function must be used */
vPortEnableInterrupt( TIMER_INTR_ID );
/* Configure the timer interrupt handler. */
XTmrCtr_SetHandler(&xTimer0Instance, ( void * ) vPortTickISR, NULL );
/* Set the correct period for the timer. */
XTmrCtr_SetResetValue( &xTimer0Instance, ucTimerCounterNumber,
ulCounterValue );
XTmrCtr_SetOptions(&xTimer0Instance, ucTimerCounterNumber, (
XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION | XTC_DOWN_COUNT_OPTION
) );
/* Start the timer. */
XTmrCtr_Start( &xTimer0Instance, ucTimerCounterNumber );
}
configASSERT( ( xStatus == pdPASS ) );
}
void vApplicationClearTimerInterrupt( void ) {
unsigned long ulCSR;
/* Clear the timer interrupt */
ulCSR = XTmrCtr_GetControlStatusReg( XPAR_TMRCTR_0_BASEADDR, 0 );
XTmrCtr_SetControlStatusReg( XPAR_TMRCTR_0_BASEADDR, 0, ulCSR );
}


