# Library based on HAL for the NEO-6M module
### Brief
The NEO-6 module series consists of stand-alone GPS receivers that feature the high-performance u-blox 6 positioning engine. 
This library was developed to receive, parse, and utilize the primary standard NMEA messages produced by the NEO-6M module. 
Users can select the messages they require, and whenever these messages are received, the appropriate callback function will 
be invoked, passing a structure containing all parsed parameters.
___
### Messages supported by this library:
- GGA
- GLL
- GSA
- GSV
- RMC
- VTG

For more details about NMEA messages and the NEO-6M module, please visit this [document](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf).
___
### To use this library, you must:
* Enable and configure the UART peripheral used with the module. The baud rate should be set to 9600, and interrupts must be enabled.
* Add neo-6m.h and neo-6m.c to your project.
* Include neo-6m.h in your project file.
  
  ```
  #include "neo-6m.h"
  ```
* Create a handle structure and a pointer to the UART handle structure used with the module.

  ```
  NEO6M_Handle_t neo6mh;
  
  UART_HandleTypeDef *gps_uart = &huart1;    //huart1 as example
  ```
* Select the messages you require and enable them using the corresponding function. (In most cases, using RMC will suffice.)

  ```
  NEO6M_AddExpectedMessage(&neo6mh, RMC);
  ```
* Call the NEO6M_MessageHandler() function within the HAL_UART_RxCpltCallback function.
  
  ```
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
  {
      NEO6M_MessageHandler(&neo6mh);
  }
  ```
* Finally, use the callback function corresponding to the selected message type. Before using the received data,
  cast the pointer to the appropriate structure.
  
  ```
  void NEO6M_RMCCallBack(void *package)
  {
      RMC_Package_t *rmc_package = (RMC_Package_t *)package;
  }
  ```
___
### Example of using this library
(Peripheral configuration not included)

  ```
  #include "neo-6m.h"
  
  
  NEO6M_Handle_t neo6mh;
  
  UART_HandleTypeDef huart1;
  UART_HandleTypeDef *gps_uart = &huart1;     
  
  
  int main(void)
  {
      while(1)
      {
          NEO6M_AddExpectedMessage(&neo6mh, RMC);
          HAL_Delay(5000);
      }
  }
  
  
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
  {
      NEO6M_MessageHandler(&neo6mh);
  }
  
  void NEO6M_RMCCallBack(void *package)
  {
      RMC_Package_t *rmc_package = (RMC_Package_t *)package;
      char buff[100];
      
      sprintf(buff, "\n\rThis is RMC packet - Longitude: %lf Latitude: %lf Time: %ld Checksum: %hx",
          rmc_package->longitude, rmc_package->latitude, rmc_package->time, rmc_package->cs);
      
      if(HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff), HAL_MAX_DELAY) != HAL_OK)
      {
          Error_Handler();
      }
      
      NEO6M_RemoveExpectedMessage(&neo6mh, RMC);
  }
  
  ```
___
### FAQ 
* During testing, I discovered a bug: when all packet types are used simultaneously, the GGA packet is not received.
  Currently, I'm unsure how to rectify this issue. Hence, if the GGA packet is essential for you, it might be beneficial to disable other packet types.
  Nevertheless, you can verify this on your MCU.



