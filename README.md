# Arduino-FreeRTOS-SAMD21-Tickless-Idle

Arduino FreeRTOS port, based on [BriscoeTech port](https://github.com/BriscoeTech/Arduino-FreeRTOS-SAMD21), with Tickless Idle implementation.

# Considerations

During Standby Mode, Systick Timer is stopped because it is fed with CPU clock and it is stopped too. So, we need another rtos tick generator that could be keep running in standby mode. 

This port use Timer 3 (TC3) as rtos tick generator. In order to save power, I have fed TC3 with internal OSCULP32K. This clock is not too much accurate, so, If you want to have better timing accuracy, feel free to change it to another clock but take in mind that It will increase power consumption.

TC3 is configured to work at OSCULP32K / 16 (2048Hz) frequency and 16bit mode. FreeRTOS is configured to generate a tick with two TC3 to get 1kHz os tick frequency. Maximum sleep time is 32s ( ( 1/2048) * 65536) ). This time could be increase changing TC3 configuration to 32 bit mode but It's not implemented in this examples.

I have configured EXPECTED_IDLE_TIME_BEFORE_SLEEP as 500 so, MCU won't go to sleep if idle state is lower than 500ms.

If you want to test examples, don't use SerialUSB as Serial port. Instead, use any SERCOM configured as USART.

While you are testing examples, if you want to upload another example or code change, you will probably need to start your SAMD board in bootloader mode. As SAMD is sleeping, USB port is not detected and Arduino IDE couldn't find board. Usually, bootloader mode starts with double tap reset button.
<<<<<<< HEAD

# Boards Tested

I have tested examples with:
    - Sodaq Autonomo (SAMD21J18A)
    - Arduino MKR GSM 1400 (SAMD21G18A)
