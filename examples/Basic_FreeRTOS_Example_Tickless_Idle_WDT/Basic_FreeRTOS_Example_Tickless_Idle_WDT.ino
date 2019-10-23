//**************************************************************************
// FreeRtos on Samd21
// By Scott Briscoe
//
// Project is a simple example of how to get FreeRtos running on a SamD21 processor
// Project can be used as a template to build your projects off of as well
//**************************************************************************

#include <FreeRTOS_SAMD21.h> //samd21

//**************************************************************************
// Type Defines and Constants
//**************************************************************************

#define  ERROR_LED_PIN  13 //Led Pin: Typical Arduino Board
//#define  ERROR_LED_PIN  2 //Led Pin: samd21 xplained board

#define ERROR_LED_LIGHTUP_STATE  LOW // the state that makes the led light up on your board, either low or high

#define BUTTON_PIN      2
// Select the serial port the project should use and communicate over
// Sombe boards use SerialUSB, some use Serial
//#define SERIAL          SerialUSB
#define SERIAL          Serial

//**************************************************************************
// global variables
//**************************************************************************
TaskHandle_t Handle_aTask;
TaskHandle_t Handle_bTask;
TaskHandle_t Handle_monitorTask;

//**************************************************************************
// Can use these function for RTOS delays
// Takes into account procesor speed
//**************************************************************************
void myDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );  
}

void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}

void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}

void buttonISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(Handle_aTask, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//*****************************************************************
// Create a thread that prints out a message when a button interrupt
// is fired.
//*****************************************************************
static void threadA( void *pvParameters ) 
{
  
  SERIAL.println("Thread A: Started");

  while(true) {
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(ulNotificationValue > 0) {
      SERIAL.println("Thread A: Interrupt Received");
    }

    SERIAL.flush();
  }
  
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  SERIAL.println("Thread A: Deleting");
  SERIAL.flush();
  vTaskDelete( NULL );
}

//*****************************************************************
// Create a thread that prints out B to the screen 35 seconds
// this task will run forever
//*****************************************************************
static void threadB( void *pvParameters ) 
{
  SERIAL.println("Thread B: Started");

  while(1)
  {
    SERIAL.println("B");
    SERIAL.flush();
    myDelayMs(35000);
  }

}

//*****************************************************************
// Task will periodicallt print out useful information about the tasks running
// Is a useful tool to help figure out stack sizes being used
//*****************************************************************
void taskMonitor(void *pvParameters)
{
    int x;
    int measurement;
    
    SERIAL.println("Task Monitor: Started");

    // run this task afew times before exiting forever
    for(x=0; x<10; ++x)
    {

      SERIAL.println("");
      SERIAL.println("******************************");
      SERIAL.println("[Stacks Free Bytes Remaining] ");

      measurement = uxTaskGetStackHighWaterMark( Handle_aTask );
      SERIAL.print("Thread A: ");
      SERIAL.println(measurement);
      
      measurement = uxTaskGetStackHighWaterMark( Handle_bTask );
      SERIAL.print("Thread B: ");
      SERIAL.println(measurement);
      
      measurement = uxTaskGetStackHighWaterMark( Handle_monitorTask );
      SERIAL.print("Monitor Stack: ");
      SERIAL.println(measurement);

      SERIAL.println("******************************");

      SERIAL.flush();
      myDelayMs(60000); // print every 60 seconds
    }

    // delete ourselves.
    // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
    SERIAL.println("Task Monitor: Deleting");
    SERIAL.flush();
    vTaskDelete( NULL );

}


//*****************************************************************

void setup() 
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);


  /* Feed EIC(External Interrupt Controller) with GCLK1(XOSC32K defined in Arduino Core)
   * and SET in freeRTOS port as RUNNING IN STANDBY. It allows to wake up MCU from standby mode 
   * with an external interrupt, i.e. with a button attached, like this example.
   */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
                      GCLK_CLKCTRL_GEN_GCLK1 |
                      GCLK_CLKCTRL_CLKEN;

  while(GCLK->STATUS.bit.SYNCBUSY);
  
  SERIAL.begin(115200);

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;    // Configure Sleep Mode as Standby Mode
  
  vNopDelayMS(1000); // prevents usb driver crash on startup, do not omit this
  while (!SERIAL) ;  // Wait for serial terminal to open port before starting program

  SERIAL.println("");
  SERIAL.println("******************************");
  SERIAL.println("        Program start         ");
  SERIAL.println("******************************");

  // Set the led the rtos will blink when we have a fatal rtos error
  // RTOS also Needs to know if high/low is the state that turns on the led.
  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object. 
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack! 
  //               Use the taskMonitor thread to help gauge how much more you need
  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);

  // Create the threads that will be managed by the rtos
  // Sets the stack size and priority of each task
  // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks
  xTaskCreate(threadA,     "Task A",       256, NULL, tskIDLE_PRIORITY + 3, &Handle_aTask);
  xTaskCreate(threadB,     "Task B",       256, NULL, tskIDLE_PRIORITY + 2, &Handle_bTask);
  xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 1, &Handle_monitorTask);

  // Start the RTOS, this function will never return and will schedule the tasks.
	vTaskStartScheduler();

}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop() 
{
    // With Tickless Idle enable, no Idle hook is called
}
