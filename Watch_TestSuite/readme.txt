/**

This watch code has the following features:

- the user can check the time anytime.  To do so, simply touch the dial and guess the time, and then the actual time will be shown.
- if the user has seen/known the time from some external source and needs to know the time, they can hit the button, which registers a 'previous_invalid' and shows them the current time, to start again.
- at random intervals the watch will buzz and ask for a time estimate and survey.  it will keep buzzing until it is attended to.  Pressing a button on the watch dismisses it without finishing the survey and
  shows the current time, so that a new interval can be started. This should be done if the user has seen/knows the time since last looking at the watch.  Starting the survey and quitting halfway through 
  will cause a timeout state; the user needs to hit the button to see the time and restart the interval timer if this happens.  Hitting the button is the correct way to deal with incorrect data entry.
- if there is a problem with your data or you can't honestly enter a time estimate, hit the button.
- these random interval surveys only will occur in between the hours specified by timebounds, which can be set in the app.  After hours survey attempts are ignored.  Once it's the start of a new day, the watch
  wakes in 'paused' mode, which must be dismissed so the user can see the time and start interacting.
- the app can put the watch into paused mode; no survey interruptions will be generated.  Hitting the button or using the button in the app can take the watch out of paused mode.  The app will remain in paused mode
  indefinitely; paused mode is the default mode when waking up for a new day with the app.
- Suggest putting the default 'start time' at 5a or an hour before you ever would conceivably wake up-- the watch will not buzz, it will simply be paused until woken.
- The button presses mean 2 things; previous data is invalid and I've seen a clock.  If a survey isn't complete it should be discarded; if it is immediately followed by a button press (<10s) it should be discarded. 
  Otherwise button presses should be considered 'invalidate the last time interval' because the user saw a clock, got a notification, or had a meeting with a set start time.
- starts with hour of last seen time on the dial when you go to make an estimate.

- thread to light and temp poll every 10s, send to BLETX

- thread to manage screen and touch polling.  TOUCH_MODE controlled by
  main thread, this tells touch how to act.  
  TOUCH_MODE = NORMAL (not an event, touch means we want to check the time)
  TOUCH_MODE = TIME_ESTIMATE (look at past time, allow scrolling to new time)
  TOUCH_MODE = ESM_TIME_ESTIMATE (same as above, but notifies esm thread on
complete)
  TOUCH_MODE = ESM_SURVEY (agree/disagree, but notifies esm thread on
complete)

support 2/3/5/7 options with dial... [num_options, strings]
{ 2 ['agree', 'disagree']}
{ 5 ['1-low','2','3','4','5-high']


Touch and button highest priority 
screen second highest

- BLETX queues data in linked list.  If connected it sends it.

- On connection, BLERX update time and dump all queued data.

- random interval with vibrate/light pulse, ask time estimate, alertness,
  valence, focus.  store last asked time, interval is set for new random
  and time bounds.

- ability to cancel previous questions, with confirm.





- light and temp sensor poll thread, code.

- whenever BLE connection happens, automatically updated RTC.

- App vibrate and flash to get attention-- randomly asks what time it is and a few other questions
  1-5, stores them. Stores last time seen on clock.
- ability to invalidate last measurement for time or all answers

- all on device without BLE.  Storage on device.  Malloc until send, then free.

- when dial is pressed, shows last time, dial movement keeps it moving forward
  to estimate, can cancel with button if seen clock


- BLE background testing.  Send data contantly, leave on and test.  If not
  storing.



  @page Watch_TestSuite example
  
  @verbatim
  ******************************************************************************
  * @file    BLE/Watch_TestSuite/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the Watch_TestSuite example.
  ******************************************************************************
  *
  * Copyright (c) 2019 STMicroelectronics. All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license 
  * SLA0044, the "License"; You may not use this file except in compliance with 
  * the License. You may obtain a copy of the License at:
  *                               www.st.com/SLA0044
  *
  ******************************************************************************
  @endverbatim

@par Example Description

How to use the Heart Rate profile as specified by the BLE SIG with FreeRTOS.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.
      
@note This application is not supported by CubeMx but has been copied from the project BLE_HeartRate generated
      by CubeMx with some modifications to replace the call to the scheduler by the use of the cmsis_os interface

@par Keywords

Connectivity, BLE, IPCC, HSEM, RTC, UART, PWR, BLE protocol, BLE pairing, BLE profile, Dual core

@par Directory contents 
  
  - BLE/Watch_TestSuite/Core/Inc/stm32wbxx_hal_conf.h		HAL configuration file
  - BLE/Watch_TestSuite/Core/Inc/stm32wbxx_it.h          	Interrupt handlers header file
  - BLE/Watch_TestSuite/Core/Inc/main.h                  	Header for main.c module
  - BLE/Watch_TestSuite/STM32_WPAN/App/app_ble.h          Header for app_ble.c module
  - BLE/Watch_TestSuite/Core/Inc/app_common.h            	Header for all modules with common definition
  - BLE/Watch_TestSuite/Core/Inc/app_conf.h              	Parameters configuration file of the application
  - BLE/Watch_TestSuite/Core/Inc/app_entry.h            	Parameters configuration file of the application
  - BLE/Watch_TestSuite/STM32_WPAN/App/ble_conf.h         BLE Services configuration
  - BLE/Watch_TestSuite/STM32_WPAN/App/ble_dbg_conf.h     BLE Traces configuration of the BLE services
  - BLE/Watch_TestSuite/STM32_WPAN/App/dis_app.h          Header for dis_app.c module
  - BLE/Watch_TestSuite/STM32_WPAN/App/hrs_app.h          Header for hrs_app.c module
  - BLE/Watch_TestSuite/Core/Inc/hw_conf.h           		Configuration file of the HW
  - BLE/Watch_TestSuite/Core/Inc/utilities_conf.h    		Configuration file of the utilities
  - BLE/Watch_TestSuite/STM32_WPAN/App/FreeRTOSConfig.h   Configuration file of FreeRTOS
  - BLE/Watch_TestSuite/Core/Src/stm32wbxx_it.c          	Interrupt handlers
  - BLE/Watch_TestSuite/Core/Src/main.c                  	Main program
  - BLE/Watch_TestSuite/Core/Src/system_stm32wbxx.c      	stm32wbxx system source file
  - BLE/Watch_TestSuite/STM32_WPAN/App/app_ble.c      	BLE Profile implementation
  - BLE/Watch_TestSuite/Core/Src/app_entry.c      		Initialization of the application
  - BLE/Watch_TestSuite/STM32_WPAN/App/dis_app.c      	Device Information Service application
  - BLE/Watch_TestSuite/STM32_WPAN/App/hrs_app.c      	Heart Rate Service application
  - BLE/Watch_TestSuite/STM32_WPAN/Target/hw_ipcc.c      	IPCC Driver
  - BLE/Watch_TestSuite/Core/Src/stm32_lpm_if.c			Low Power Manager Interface
  - BLE/Watch_TestSuite/Core/Src/hw_timerserver.c 		Timer Server based on RTC
  - BLE/Watch_TestSuite/Core/Src/hw_uart.c 				UART Driver

     
@par Hardware and Software environment

  - This example runs on STM32WB55xx devices.
  
  - This example has been tested with an STMicroelectronics STM32WB55VG-Nucleo
    board and can be easily tailored to any other supported device 
    and development board.

@par How to use it ? 

This application requests having the stm32wb5x_BLE_Stack_full_fw.bin binary flashed on the Wireless Coprocessor.
If it is not the case, you need to use STM32CubeProgrammer to load the appropriate binary.
All available binaries are located under /Projects/STM32_Copro_Wireless_Binaries directory.
Refer to UM2237 to learn how to use/install STM32CubeProgrammer.
Refer to /Projects/STM32_Copro_Wireless_Binaries/ReleaseNote.html for the detailed procedure to change the
Wireless Coprocessor binary.  

In order to make the program work, you must do the following:
 - Open your toolchain 
 - Rebuild all files and flash the board with the executable file

 On the android/ios device, enable the Bluetooth communications, and if not done before,
 - Install the ST BLE Profile application on the android device
	https://play.google.com/store/apps/details?id=com.stm.bluetoothlevalidation&hl=en
    https://itunes.apple.com/fr/App/st-ble-profile/id1081331769?mt=8

 - Install the ST BLE Sensor application on the ios/android device
	https://play.google.com/store/apps/details?id=com.st.bluems
	https://itunes.apple.com/us/App/st-bluems/id993670214?mt=8

 - Power on the Nucleo board with the Watch_TestSuite application
 - Then, click on the App icon, ST BLE Sensor (android device)
 - connect to a device
 - select the HRSTM in the device list

The Heart Rate is displayed each second on the android device.

For more details refer to the Application Note: 
  AN5289 - Building a Wireless application 
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 
