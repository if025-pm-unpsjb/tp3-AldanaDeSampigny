/*************************************************************************
 * Programa de prueba basado para LM3S6965.
 *
 * Basado en el demo provisto por FreeRTOS, para ejecutar sobre QEMU.
 *
 * Please ensure to read http://www.freertos.org/portlm3sx965.html
 * which provides information on configuring and running this demo for the
 * various Luminary Micro EKs.
 *************************************************************************/

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h" // Incluido para el uso de semáforos binarios y Mutex

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Hardware library includes. */
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_sysctl.h"
#include "hw_uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "grlib.h"
#include "osram128x64x4.h"
#include "uart.h"
#include "bitmap.h"

/*-----------------------------------------------------------*/

/* Dimensions the buffer for text messages. */
#define mainMAX_MSG_LEN 					25

/* Constants used when writing strings to the display. */
#define mainCHARACTER_HEIGHT 				( 9 )
#define mainMAX_ROWS_128 					( mainCHARACTER_HEIGHT * 14 )
#define mainMAX_ROWS_96 					( mainCHARACTER_HEIGHT * 10 )
#define mainMAX_ROWS_64 					( mainCHARACTER_HEIGHT * 7 )
#define mainFULL_SCALE 						( 15 )
#define ulSSI_FREQUENCY 					( 3500000UL )

#define mainPRIO_INV_SYSTEM ( 99 )

#define mainPIP_SYSTEM      ( 98 )

int sistema = mainPIP_SYSTEM;

/*-----------------------------------------------------------*/

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/*
 * Basic polling UART write function.
 */
static void prvPrintString( const char * pcString );

/*
 * Busy wait the specified number of ticks.
 */
static void vBusyWait( TickType_t ticks );

/*
 * Periodic task (función original).
 */
static void prvTask( void* pvParameters );

static void prvTaskLow( void* pvParameters );
static void prvTaskMedium( void* pvParameters );
static void prvTaskHigh( void* pvParameters );
static void prvCreatePriorityInversionTasks( void );
static void prvCreatePIPTasks( void );

/*-----------------------------------------------------------*/

/* Functions to access the OLED.  The one used depends on the dev kit
being used. */
void ( *vOLEDInit )( uint32_t ) = NULL;
void ( *vOLEDStringDraw )( const char *, uint32_t, uint32_t, unsigned char ) = NULL;
void ( *vOLEDImageDraw )( const unsigned char *, uint32_t, uint32_t, uint32_t, uint32_t ) = NULL;
void ( *vOLEDClear )( void ) = NULL;

static SemaphoreHandle_t xBinarySemaphore = NULL;

/*-----------------------------------------------------------*/

struct xTaskStruct {
	TickType_t wcet;
	TickType_t period;
};

typedef struct xTaskStruct xTask;

xTask tasks1[] = {
	{ 1000, 4000 }, // T1: Periodo más corto = Mayor prioridad
	{ 1000, 5000 }, // T2
	{ 2000, 8000 }	// T3: Periodo más largo = Menor prioridad
};


xTask tasks2[] = { { 1000, 4000 }, { 1000, 8000 }, { 2000, 9000 } };
xTask tasks3[] = { { 1000, 4000 }, { 1000, 6000 }, { 1000, 8000 }, { 3000, 13000 } };
xTask tasks4[] = { { 1000, 4000 }, { 2000, 7000 }, { 1000, 12000 }, {2000, 14000 } };
xTask tasks5[] = { { 1000, 5000 }, { 1000, 9000 }, { 1000, 10000 }, {2000, 15000 }, { 2000, 16000 } };
xTask tasks6[] = { { 1000, 6000 }, { 1000, 8000 }, { 2000, 11000 }, { 1000, 15000 }, {2000, 17000 } };
xTask tasks7[] = { { 1000, 4000 }, { 1000, 7000 }, { 2000, 10000 }, {2000, 14000 } };
xTask tasks8[] = { { 1000, 4000 }, { 1000, 8000 }, { 2000, 10000 }, {2000, 14000 } };
xTask tasks9[] = { { 1000, 5000 }, { 1000, 8000 }, { 2000, 12000 }, {1000, 15000 }, { 1000, 16000 } };
xTask tasks10[] = { { 1000, 5000 }, { 1000, 8000 }, { 1000, 12000 }, {1000, 13000 }, { 2000, 16000 } };

static void prvTaskLow( void *pvParameters )
{
	(void) pvParameters;
	// Espera inicial para asegurar que TH y TM existan antes de empezar el ciclo.
	vTaskDelay( pdMS_TO_TICKS( 100 ) );

	for( ;; )
	{
		prvPrintString( "TL: Intentando tomar semaforo/Mutex...\n\r" );

		if( xSemaphoreTake( xBinarySemaphore, portMAX_DELAY ) == pdPASS )
		{
			prvPrintString( "TL: Recurso tomado. Entrando a seccion critica.\n\r" );

			prvPrintString( "TL: Ejecutando trabajo critico (3000ms)..\n\r" );
			vBusyWait( pdMS_TO_TICKS( 3000 ) );

			prvPrintString( "TL: Saliendo de seccion critica.\n\r" );

			xSemaphoreGive( xBinarySemaphore );
			prvPrintString( "TL: Recurso liberado.\n\r" );
		}


		vTaskDelay( pdMS_TO_TICKS( 5000 ) );
	}
}

// Tarea de MEDIA prioridad (TM - Medium)
static void prvTaskMedium( void *pvParameters )
{
	(void) pvParameters;
	vTaskDelay( pdMS_TO_TICKS( 500 ) );

	for( ;; )
	{

		vBusyWait( pdMS_TO_TICKS( 2000 ) );

		prvPrintString( "TM: Terminado. Durmiendo.\n\r" );
		vTaskDelay( pdMS_TO_TICKS( 2000 ) );
	}
}

static void prvTaskHigh( void *pvParameters )
{
	(void) pvParameters;
	vTaskDelay( pdMS_TO_TICKS( 200 ) );

	for( ;; )
	{
		prvPrintString( "TH: Intentando tomar recurso (ALTA PRIORIDAD).\n\r" );

		if( xSemaphoreTake( xBinarySemaphore, portMAX_DELAY ) == pdPASS )
		{
			prvPrintString( "TH: Recurso tomado. Corriendo muy brevemente.\n\r" );
			vBusyWait( pdMS_TO_TICKS( 500 ) );

			xSemaphoreGive( xBinarySemaphore );
			prvPrintString( "TH: Recurso liberado. Durmiendo largo tiempo.\n\r" );
		}


		vTaskDelay( pdMS_TO_TICKS( 8000 ) );
	}
}

// Función para inicializar el demo de Inversión de Prioridad (Semáforo Binario)
static void prvCreatePriorityInversionTasks( void )
{
	xBinarySemaphore = xSemaphoreCreateBinary();

	if ( xBinarySemaphore != NULL )
	{
		xTaskCreate( prvTaskHigh, "TH-High", configMINIMAL_STACK_SIZE + 50, NULL, tskIDLE_PRIORITY + 3, NULL );

		xTaskCreate( prvTaskMedium, "TM-Medium", configMINIMAL_STACK_SIZE + 50, NULL, tskIDLE_PRIORITY + 2, NULL );

		xTaskCreate( prvTaskLow, "TL-Low", configMINIMAL_STACK_SIZE + 50, NULL, tskIDLE_PRIORITY + 1, NULL );


		xSemaphoreGive( xBinarySemaphore );

		prvPrintString( "Demo Inversion de Prioridad: Tareas creadas (TH > TM > TL).\n\r" );
	} else {
		prvPrintString( "ERROR: No se pudo crear el semáforo.\n\r" );
	}
}

// Función para inicializar el demo de Protocolo de Herencia de Prioridades (Mutex)
static void prvCreatePIPTasks( void )
{
	xBinarySemaphore = xSemaphoreCreateMutex();

	if ( xBinarySemaphore != NULL )
	{
		xTaskCreate( prvTaskHigh, "TH-High", configMINIMAL_STACK_SIZE + 50, NULL, tskIDLE_PRIORITY + 3, NULL );

		xTaskCreate( prvTaskMedium, "TM-Medium", configMINIMAL_STACK_SIZE + 50, NULL, tskIDLE_PRIORITY + 2, NULL );


		xTaskCreate( prvTaskLow, "TL-Low", configMINIMAL_STACK_SIZE + 50, NULL, tskIDLE_PRIORITY + 1, NULL );

		prvPrintString( "Demo PIP (Mutex): Tareas creadas (TH > TM > TL). Inversion mitigada.\n\r" );
	} else {
		prvPrintString( "ERROR: No se pudo crear el Mutex.\n\r" );
	}
}

/*************************************************************************
 * Main
 *************************************************************************/
int main( void )
{
	/* Initialise the trace recorder. */
	vTraceEnable( TRC_INIT );

    prvSetupHardware();

    /* Map the OLED access functions to the driver functions that are appropriate
    for the evaluation kit being used. */
    //configASSERT( ( HWREG( SYSCTL_DID1 ) & SYSCTL_DID1_PRTNO_MASK ) == SYSCTL_DID1_PRTNO_6965 );
    vOLEDInit = OSRAM128x64x4Init;
    vOLEDStringDraw = OSRAM128x64x4StringDraw;
    vOLEDImageDraw = OSRAM128x64x4ImageDraw;
    vOLEDClear = OSRAM128x64x4Clear;

    /* Initialise the OLED and display a startup message. */
    vOLEDInit( ulSSI_FREQUENCY );

    /* Print startup message to the OLED display. */
    static char cMessage[ mainMAX_MSG_LEN ];
	if (sistema == mainPIP_SYSTEM) {
    	sprintf(cMessage, "PIP (Mutex) Demo!");
    } else if (sistema == mainPRIO_INV_SYSTEM) {
    	sprintf(cMessage, "Prio. Inv. Demo!");
    } else {
    	sprintf(cMessage, "Sistema %d", sistema);
    }
    vOLEDStringDraw( cMessage, 0, 0, mainFULL_SCALE );

    /* Print "Start!" to the UART. */
    prvPrintString("Start!\n\r");

    // Lógica de creación de tareas.

	if (sistema == mainPIP_SYSTEM) {
        prvCreatePIPTasks();
    } else if (sistema == mainPRIO_INV_SYSTEM) {
		prvCreatePriorityInversionTasks(); // Inicia el demo de Inversión de Prioridad
	} else {
		xTask *tasksSistema = NULL;
		int numTasks = 0;

		if (sistema == 1) {
			tasksSistema = tasks1;
			numTasks = 3;
		} else if (sistema == 2){
			tasksSistema = tasks2;
			numTasks = 3;
		} else if (sistema == 3){
			tasksSistema = tasks3;
			numTasks = 4;
		} else if (sistema == 4) {
			tasksSistema = tasks4;
			numTasks = 4;
		} else if (sistema == 5) {
			tasksSistema = tasks5;
			numTasks = 5;
		} else if (sistema == 6) {
			tasksSistema = tasks6;
			numTasks = 5;
		} else if (sistema == 7) {
			tasksSistema = tasks7;
			numTasks = 4;
		} else if (sistema == 8) {
			tasksSistema = tasks8;
			numTasks = 4;
		} else if (sistema == 9) {
			tasksSistema = tasks9;
			numTasks = 5;
		} else if (sistema == 10){
			tasksSistema = tasks10;
			numTasks = 5;
		} else {
			prvPrintString("ERROR: Sistema no reconocido.\n\r");
		}

		char taskName[6];

		for (int indice = 0; indice < numTasks; indice++) {
			sprintf(taskName, "T%d", indice + 1);
			xTaskCreate(prvTask, (const char*) taskName,
						configMINIMAL_STACK_SIZE + 50,
						(void*) &tasksSistema[indice],
						configMAX_PRIORITIES - (indice + 1),
						NULL);
		}
	} // Fin de la lógica de sistemas periódicos

    vTraceEnable( TRC_START );

    /* Launch the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task. */
    for( ;; );
}
/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
    /* If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
    a workaround to allow the PLL to operate reliably. */
    if( DEVICE_IS_REVA2 )
    {
        SysCtlLDOSet( SYSCTL_LDO_2_75V );
    }

    /* Set the clocking to run from the PLL at 50 MHz */
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );

    /* Initialise the UART - QEMU usage does not seem to require this
    initialisation. */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_UART0 );
    UARTEnable( UART0_BASE );
}
/*-----------------------------------------------------------*/

static void prvPrintString( const char * pcString )
{
    while( *pcString != 0x00 )
    {
        UARTCharPut( UART0_BASE, *pcString );
        pcString++;
    }
}
/*-----------------------------------------------------------*/

// Función de espera activa original, basada en el conteo de ticks
static void vBusyWait( TickType_t ticks )
{
    TickType_t elapsedTicks = 0;
    TickType_t currentTick = 0;
    while ( elapsedTicks < ticks ) {
        currentTick = xTaskGetTickCount();
        while ( currentTick == xTaskGetTickCount() ) {
            asm("nop");
        }
        elapsedTicks++;
    }
}
/*-----------------------------------------------------------*/

// Tarea periódica original, utilizando vBusyWait para simular el WCET
void prvTask( void *pvParameters )
{
	char cMessage[ mainMAX_MSG_LEN ];
	unsigned int uxReleaseCount = 0;
	TickType_t pxPreviousWakeTime = 0;
	xTask *task = (xTask*) pvParameters;

	pxPreviousWakeTime = xTaskGetTickCount();

	for( ;; )
	{
        sprintf( cMessage, "%s - %u\n\r", pcTaskGetTaskName( NULL ), uxReleaseCount );

        prvPrintString( cMessage );

		// Simulación de carga real (WCET)
		vBusyWait( task->wcet);


		vTaskDelayUntil( &pxPreviousWakeTime, task->period );

		uxReleaseCount += 1;
	}

	vTaskDelete( NULL );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char *pcFile, uint32_t ulLine )
{
    volatile uint32_t ulSetTo1InDebuggerToExit = 0;
    {
        while( ulSetTo1InDebuggerToExit == 0 )
        {
            /* Nothing to do here.  Set the loop variable to a non zero value in
            the debugger to step out of this function to the point that caused
            the assertion. */
            ( void ) pcFile;
            ( void ) ulLine;
        }
    }
}

char* _sbrk_r (struct _reent *r, int incr)
{
    /* Just to keep the linker quiet. */
    ( void ) r;
    ( void ) incr;

    return NULL;
}

int __error__(char *pcFilename, unsigned long ulLine) {
    return 0;
}
