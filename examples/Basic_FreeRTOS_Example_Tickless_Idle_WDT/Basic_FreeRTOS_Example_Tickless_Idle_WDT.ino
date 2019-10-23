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

// Select the serial port the project should use and communicate over
// Sombe boards use SerialUSB, some use Serial
//#define SERIAL          SerialUSB
#define SERIAL          Serial

#define WDT_GCLK        4
/**
 * WDT will be feed with GCLK7 from FreeRTOS port. GCLK7 is running at 2048Hz.
 * So wdt_period will be (1/2048) * wdt_period
 */
enum wdt_period: uint8_t {
  WDT_8CYCLES     = 0,    // 4ms
  WDT_16CYCLES    = 1,    // 8ms
  WDT_32CYCLES    = 2,    // 16ms
  WDT_64CYCLES    = 3,    // 32ms
  WDT_128CYCLES   = 4,    // 64ms
  WDT_256CYCLES   = 5,    // 128ms
  WDT_512CYCLES   = 6,    // 256ms
  WDT_1024CYCLES  = 7,    // 512ms
  WDT_2048CYCLES  = 8,    // 1s
  WDT_4096CYCLES  = 9,    // 2s
  WDT_8192CYCLES  = 10,   // 4s
  WDT_16384CYCLES = 11    // 8s
  
};

//**************************************************************************
// global variables
//**************************************************************************
TaskHandle_t Handle_wdtTask;
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

//*****************************************************************
// Create a thread that prints out a message when a button interrupt
// is fired.
//*****************************************************************
static void wdtTask( void *pvParameters ) 
{
  uint32_t ulNotificationValue = 0;
  SERIAL.println("WDT TASK: Started");

  while(ulNotificationValue != 1) {
    ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(ulNotificationValue > 0) {
      SERIAL.print("WDT Task Received: ");
      SERIAL.println(ulNotificationValue);
      wdt_reset();
    }

    SERIAL.flush();
  }
  
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  SERIAL.println("WDT TASK: Deleting");
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
  uint8_t counter = 0;
  
  while(1)
  {
    SERIAL.println("B");
    SERIAL.flush();
    myDelayMs(10000);
    ++counter;

    if(counter == 3) {
      wdt_disable();
    } else if(counter == 6) {
      wdt_enable(WDT_8192CYCLES);
    } else if(counter >= 10) {
      while(1);   // Force a block;
    }
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

      measurement = uxTaskGetStackHighWaterMark( Handle_wdtTask );
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
  wdt_disable();
  
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
  xTaskCreate(threadB,     "Task B",       256, NULL, tskIDLE_PRIORITY + 3, &Handle_bTask);
  xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 2, &Handle_monitorTask);

  wdt_enable(WDT_8192CYCLES);
  // Start the RTOS, this function will never return and will schedule the tasks.
	vTaskStartScheduler();
  
}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop() 
{
    SERIAL.print('.');
}


void wdt_enable(wdt_period period) {
  // Here we use normal mode with  the early warning interrupt
  // enabled. The early warning period is defined by the parameter
  // 'period' and the reset is set to twice that value.

  // Turn the power to the WDT module on
  PM->APBAMASK.reg |= PM_APBAMASK_WDT;

  // We cannot configure the WDT if it is already in always on mode
  if (!(WDT->CTRL.reg & WDT_CTRL_ALWAYSON)) {

    // Setup clock provider WDT_GCLK with a 32 source divider
    // GCLK_GENDIV_ID(X) specifies which GCLK we are configuring
    // GCLK_GENDIV_DIV(Y) specifies the clock prescalar / divider
    // If GENCTRL.DIVSEL is set (see further below) the divider
    // is 2^(Y+1). If GENCTRL.DIVSEL is 0, the divider is simply Y
    // This register has to be written in a single operation
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(WDT_GCLK) |
                       GCLK_GENDIV_DIV(4);

    // Configure the GCLK module
    // GCLK_GENCTRL_GENEN, enable the specific GCLK module
    // GCLK_GENCTRL_SRC_OSCULP32K, set the source to the OSCULP32K
    // GCLK_GENCTRL_ID(X), specifies which GCLK we are configuring
    // GCLK_GENCTRL_DIVSEL, specify which prescalar mode we are using
    // Output from this module is 1khz (32khz / 32)
    // This register has to be written in a single operation.
    GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN |
                        GCLK_GENCTRL_SRC_OSCULP32K |
                        GCLK_GENCTRL_ID(WDT_GCLK) |
                        GCLK_GENCTRL_DIVSEL;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    // Configure the WDT clock
    // GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_WDT), specify the WDT clock
    // GCLK_CLKCTRL_GEN(WDT_GCLK), specify the source from the WDT_GCLK GCLK
    // This register has to be written in a single operation
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_WDT) |
                        GCLK_CLKCTRL_GEN(WDT_GCLK) |
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    // Disable the module before configuring
    WDT->CTRL.reg &= ~WDT_CTRL_ENABLE;
    while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);

    // Disable windowed mode
    WDT->CTRL.reg &= ~WDT_CTRL_WEN;
    while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);

    // Set the reset period to twice that of the
    // specified interrupt period
    WDT->CONFIG.reg = WDT_CONFIG_PER(period + 1);

    // Set the early warning as specified by the period
    WDT->EWCTRL.reg = WDT_EWCTRL_EWOFFSET(period);

    // Enable the WDT module
    WDT->CTRL.reg |= WDT_CTRL_ENABLE;
    while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);

    // Enable early warning interrupt
    WDT->INTENSET.reg = WDT_INTENSET_EW;

    // Enable interrupt vector for WDT
    // Priority is set to 0x03, the lowest
    NVIC_EnableIRQ(WDT_IRQn);
    NVIC_SetPriority(WDT_IRQn, 0x03);
  }
  
   xTaskCreate(wdtTask,     "WDT Task",     256, NULL, tskIDLE_PRIORITY + 1, &Handle_wdtTask);
}

void wdt_disable()
{
  // Disable the WDT module
  WDT->CTRL.reg &= ~WDT_CTRL_ENABLE;
  while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);

  // Turn off the power to the WDT module
  PM->APBAMASK.reg &= ~PM_APBAMASK_WDT;
  
  NVIC_DisableIRQ(WDT_IRQn);
  
  if(Handle_wdtTask) {
    xTaskNotify(Handle_wdtTask, 1, eSetValueWithOverwrite);
  }
}

// Resets the WDT counter
void wdt_reset()
{
  // Wait if currently syncing, reset counter and wait for synchronisation
  while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.reg & WDT_STATUS_SYNCBUSY);
}

void WDT_Handler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if(Handle_wdtTask) {
    xTaskNotifyFromISR(Handle_wdtTask, 2, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  }
    
  // Clear the early warning interrupt flag
  WDT->INTFLAG.reg = WDT_INTFLAG_EW;
  
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
