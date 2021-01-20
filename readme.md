This is the working directory for my STM32WB work for the watch prototype,
in order to get FreeRTOS and BLE up and running.

1. We have the stock BLE_p2pServer example.
2. We have the BLE_p2pServerFreeRTOS example, which has been modified to do the
   same thing as the BLE_p2pServer example, but using FreeRTOS.  It will send a
   notification on button press and recieve LED on/off updates.  It started
   life as the stock BLE_HeartRateFreeRTOS example from STM.
3. We have Watch_BLEFreeRTOS_V1, which attempts to merge other watch behavior
   into the above example, and communicate over BLE with the app in a
   functional way for our goals.
