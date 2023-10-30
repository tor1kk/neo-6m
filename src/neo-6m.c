/*
 * neo-6m.c
 *
 *  Created on: Oct 16, 2023
 *      Author: Viktor
 */

#include "neo-6m.h"


static double nmea_to_dec(double deg_coord, char nsew);

static void gga_handle(NEO6M_Handle_t *handle, uint32_t message_num);
static void gll_handle(NEO6M_Handle_t *handle, uint32_t message_num);
static void gsa_handle(NEO6M_Handle_t *handle, uint32_t message_num);
static void gsv_handle(NEO6M_Handle_t *handle, uint32_t message_num);
static void rmc_handle(NEO6M_Handle_t *handle, uint32_t message_num);
static void vtg_handle(NEO6M_Handle_t *handle, uint32_t message_num);

static void nmea_parser(char *package, char *formats, ...);
static uint8_t gsv_get_noMsg(char *buff);


static const NMEA_StandardMessage_t NMEA_STANDART_MESSAGGES[] =
{
		{EMPTY, {0}, NULL},
		{GLL, "$GPGLL", &NEO6M_GLLCallBack},
		{GGA, "$GPGGA", &NEO6M_GGACallBack},
		{GSA, "$GPGSA", &NEO6M_GSACallBack},
		{GSV, "$GPGSV", &NEO6M_GSVCallBack},
		{RMC, "$GPRMC", &NEO6M_RMCCallBack},
		{VTG, "$GPVTG", &NEO6M_VTGCallBack}
};


typedef void (*HandlerFunction_t)(NEO6M_Handle_t*, uint32_t);

static const HandlerFunction_t NMEA_MESSAGGES_HANDLERS[] =
{
		&gll_handle,
		&gga_handle,
		&gsa_handle,
		&gsv_handle,
		&rmc_handle,
		&vtg_handle
};


/*********************************************************************************************
 *								Supported user functions
 ********************************************************************************************/

/**
  * @brief   This function adds the type of NMEA message to list that determine which callbacks user need
  * @param   *handler: Pointer to the handler structure.
  * 		 MessagesTypes_t message_type: One of the supported message type, see @messages_types in .h file
  * @retval  0 - if successfully, otherwise - 1
  */
uint8_t NEO6M_AddExpectedMessage(NEO6M_Handle_t *handle, MessagesTypes_t message_type)
{
	uint8_t flag=1;

	/* Search for empty space */
	for(uint32_t i = 0; i < EXPECTED_MESSAGES_BUFF_SIZE; i++)
	{
		if(handle->expectedMessages[i].callback == NULL)
		{
			handle->expectedMessages[i] = NMEA_STANDART_MESSAGGES[message_type];
			handle->expectedMessagesCount++;
			flag = 0;
			break;
		}
	}

	/* If MCU doesn't receive messages from module yet, starts receiving and updates status flag */
	if(!flag)
	{
		if(handle->receive_status == NEO_FREE)
		{
			handle->receive_status = NEO_WAITING;
			if(HAL_UART_Receive_IT(GPS_UART, (uint8_t *)&handle->rcvdByte, 1) != HAL_OK)
			{
				flag = 1;
			}
		}
	}

	return flag;
}


/**
  * @brief   This function remove the type of NMEA message from the list that determine which callbacks user need
  * @param   *handler: Pointer to the handler structure.
  * 		 MessagesTypes_t message_type: One of the supported message type, see @messages_types in .h file
  * @retval  0 - if successfully, otherwise - 1
  */
uint8_t NEO6M_RemoveExpectedMessage(NEO6M_Handle_t *handle, MessagesTypes_t message_type)
{
	/* Check which messages is expects, if this is type that must be removed - changes it to EMPTY */
	for(uint32_t i = 0; i < EXPECTED_MESSAGES_BUFF_SIZE; i++)
	{
		if(handle->expectedMessages[i].type == message_type)
		{
			handle->expectedMessages[i] = NMEA_STANDART_MESSAGGES[EMPTY];
			handle->expectedMessagesCount--;

			//If no messages expects - finishes receiving
			if(handle->expectedMessagesCount < 1)
			{
				handle->receive_status = NEO_FREE;
			}
			return 0;
		}
	}

	return 1;
}


/**
  * @brief   This function receives messages from module byte by byte, and manages message if end sequence happened
  * @note	 Ensure this is invoked within the appropriate UART callback for proper operation.
  * @param   *handler: Pointer to the handler structure.
  * @retval  None
  */
void NEO6M_MessageHandler(NEO6M_Handle_t *handler)
{
	uint32_t checked_types=0;

	//Moves received byte to buffer
	handler->rxBuff[handler->rxCounter++] = handler->rcvdByte;

	//Checks for end sequence
	if(handler->rcvdByte == '\n')
	{
		//Iterates array with expects messages types
		for(uint32_t i=0; i < EXPECTED_MESSAGES_BUFF_SIZE; i++)
		{
			//Check for empty space in array
			if(handler->expectedMessages[i].type != EMPTY)
			{
				//Compares received message type witch expected message type
				if(!( strncmp(handler->rxBuff, handler->expectedMessages[i].formatter, 6) ))
				{
					//Calls appropriate message handler if this is expected message
					NMEA_MESSAGGES_HANDLERS[handler->expectedMessages[i].type-1](handler, i);
					break;
				}
				checked_types++;
			}
			//If count of checked messages types is equal to count of all messages types that expects, then finishes iteration
			if(checked_types >= handler->expectedMessagesCount)
			{
				break;
			}
		}

		//Resets the rx buffer
		memset(handler->rxBuff, 0, RX_BUFFER_SIZE);
		handler->rxCounter = 0;
	}

	HAL_UART_Receive_IT(GPS_UART, (uint8_t *)&handler->rcvdByte, 1);
}


/*********************************************************************************************
 *								NMEA standard messages handlers
 ********************************************************************************************/

/**
  * @brief   This function parse particular message and calls appropriate callback
  * @param   *handler: Pointer to the handler structure.
  * @param   *message_num: Index of the message
  * @retval  None
  */
static void gga_handle(NEO6M_Handle_t *handle, uint32_t message_num)
{
	GGA_Package_t package={0};

	nmea_parser(handle->rxBuff, "3dcdc88ffcfc88",
				&package.time,
				&package.latitude,
				&package.ns,
				&package.longitude,
				&package.ew,
				&package.fs,
				&package.noSV,
				&package.hdop,
				&package.msl,
				&package.uMsl,
				&package.altref,
				&package.uSep,
				&package.diffAge,
				&package.diffStation,
				&package.cs);

	package.latitude = nmea_to_dec(package.latitude, package.ns);
	package.longitude = nmea_to_dec(package.longitude, package.ew);

	handle->expectedMessages[message_num].callback(&package);
}

/**
  * @brief   This function parse particular message and calls appropriate callback
  * @param   *handler: Pointer to the handler structure.
  * @retval  None
  */
static void gll_handle(NEO6M_Handle_t *handle, uint32_t message_num)
{
	GLL_Package_t package={0};

	nmea_parser(handle->rxBuff, "dcdc3cc",
			&package.latitude,
			&package.ns,
			&package.longitude,
			&package.ew,
			&package.time,
			&package.valid,
			&package.mode,
			&package.cs);

	package.latitude = nmea_to_dec(package.latitude, package.ns);
	package.longitude = nmea_to_dec(package.longitude, package.ew);

	handle->expectedMessages[message_num].callback(&package);
}

/**
  * @brief   This function parse particular message and calls appropriate callback
  * @param   *handler: Pointer to the handler structure.
  * @retval  None
  */
static void gsa_handle(NEO6M_Handle_t *handle, uint32_t message_num)
{
	GSA_Package_t package={0};

	nmea_parser(handle->rxBuff, "c8888888888888fff",
				&package.sMode,
				&package.fs,
				&package.sv[0],
				&package.sv[1],
				&package.sv[2],
				&package.sv[3],
				&package.sv[4],
				&package.sv[5],
				&package.sv[6],
				&package.sv[7],
				&package.sv[8],
				&package.sv[9],
				&package.sv[10],
				&package.sv[11],
				&package.pdop,
				&package.hdop,
				&package.vdop,
				&package.cs);

	handle->expectedMessages[message_num].callback(&package);
}

/**
  * @brief   This function store GSV packets, until all packets will be received, then parses this packets
  * @param   *handler: Pointer to the handler structure.
  * @retval  None
  */
static void gsv_handle(NEO6M_Handle_t *handle, uint32_t message_num)
{
	static char gsv_buff[300]={0};
	static size_t gsv_count=0, gsv_buff_len=0;
	GSV_Package_t package={0};
	char *ptr, *saveptr;

	//Waits for all packets that must be receive
	if(gsv_count < gsv_get_noMsg(handle->rxBuff))
	{
		strcpy(&gsv_buff[gsv_buff_len], handle->rxBuff);
		gsv_count++;
		gsv_buff_len += strlen(handle->rxBuff);
		return;
	}

	//If all packets was received, starts parse this packet one by one, and calls appropriate callback
	ptr = strtok_r(gsv_buff, "\n", &saveptr);
	for(uint32_t i=0; i < gsv_count; i++)
	{
		nmea_parser(ptr, "8888818881888188818",
					&package.noMsg,
					&package.msgNo,
					&package.noSV,
					&package.repeated_block[0].sv,
					&package.repeated_block[0].elv,
					&package.repeated_block[0].az,
					&package.repeated_block[0].cno,
					&package.repeated_block[1].sv,
					&package.repeated_block[1].elv,
					&package.repeated_block[1].az,
					&package.repeated_block[1].cno,
					&package.repeated_block[2].sv,
					&package.repeated_block[2].elv,
					&package.repeated_block[2].az,
					&package.repeated_block[2].cno,
					&package.repeated_block[3].sv,
					&package.repeated_block[3].elv,
					&package.repeated_block[3].az,
					&package.repeated_block[3].cno,
					&package.cs);

		handle->expectedMessages[message_num].callback(&package);

		memset(&package, 0, sizeof(package));
		ptr = strtok_r(NULL, "\n", &saveptr);
	}

	gsv_count = 0;
	gsv_buff_len = 0;
	memset(gsv_buff, 0, 300);
}

/**
  * @brief   This function parse particular message and calls appropriate callback
  * @param   *handler: Pointer to the handler structure.
  * @retval  None
  */
static void rmc_handle(NEO6M_Handle_t *handle, uint32_t message_num)
{
	RMC_Package_t package={0};

	nmea_parser(handle->rxBuff, "3cdcdcff3fcc",
				&package.time,
				&package.status,
				&package.latitude,
				&package.ns,
				&package.longitude,
				&package.ew,
				&package.spd,
				&package.cog,
				&package.date,
				&package.mv,
				&package.mvE,
				&package.mode,
				&package.cs);

	package.latitude = nmea_to_dec(package.latitude, package.ns);
	package.longitude = nmea_to_dec(package.longitude, package.ew);

	handle->expectedMessages[message_num].callback(&package);
}

/**
  * @brief   This function parse particular message and calls appropriate callback
  * @param   *handler: Pointer to the handler structure.
  * @retval  None
  */
static void vtg_handle(NEO6M_Handle_t *handle, uint32_t message_num)
{
	VTG_Package_t package={0};

	nmea_parser(handle->rxBuff, "fc8cfcfcc",
			&package.cogt,
			&package.true,
			&package.cogm,
			&package.magnetic,
			&package.sog,
			&package.knots,
			&package.kph,
			&package.kilometers,
			&package.mode,
			&package.cs);

	handle->expectedMessages[message_num].callback(&package);
}


/*********************************************************************************************
 *										Some helpful functions
 ********************************************************************************************/

/**
  * @brief   This function converts the 'Minutes' and 'Fractional Minutes' parts to decimal format
  * @param   deg_coord: Coordinate that must be comverted
  * @param   nsew: N/S or E/W indicator
  * @retval  decimal format of coordinate
  */
static double nmea_to_dec(double deg_coord, char nsew)
{
    int degree = (int)(deg_coord/100);
    double minutes = deg_coord - degree*100;
    double dec_deg = minutes / 60;
    double decimal = degree + dec_deg;

    if (nsew == 'S' || nsew == 'W')
    {
        decimal *= -1;
    }
    return decimal;
}


/**
  * @brief   This is universal NMEA parser
  * @param   *package: Pointer to the string, that must be parsed
  * @param   *package: Pointer to the string with formats(f - float,
  * c - char, 8 - uint8_t, d - double, 1 - uint16_t, 3 - uint32_t)
  * @param   args: pointers to appropriate structure arguments
  * @retval  None
  */
static void nmea_parser(char *package, char *formats, ...)
{
	va_list args;
	char *ptr = package;

	va_start(args, formats);

	for(uint32_t i=0; i < strlen(formats); i++)
	{
		switch(formats[i])
		{
			case 'f':
			{
				ptr = strchr(ptr, ',');
				*va_arg(args, float *) = strtod(++ptr, NULL);
				break;
			}
			case 'c':
			{
				ptr = strchr(ptr, ',');
				*va_arg(args, char *) = *++ptr;
				break;
			}
			case '8':
			{
				ptr = strchr(ptr, ',');
				*va_arg(args, uint8_t *) = strtol(++ptr, NULL, 10);
				break;
			}
			case 'd':
			{
				ptr = strchr(ptr, ',');
				*va_arg(args, double *) = strtod(++ptr, NULL);
				break;
			}
			case '1':
			{
				ptr = strchr(ptr, ',');
				*va_arg(args, uint16_t *) = strtol(++ptr, NULL, 10);
				break;
			}
			case '3':
			{
				ptr = strchr(ptr, ',');
				*va_arg(args, uint32_t *) = strtol(++ptr, NULL, 10);
				break;
			}
			default:
			{
				*va_arg(args, uint8_t *) = 0;
			}
		}
	}

	ptr = strchr(package, '*');
	*va_arg(args, uint16_t *) = strtol(++ptr, NULL, 16);

	va_end(args);
}


/**
  * @brief   This function return number of GPGSV messages being output
  * @param   *package: Pointer to the string, were noMsg must be found
  * @retval  uint8_t Number of messages
  */
static uint8_t gsv_get_noMsg(char *buff)
{
	return strtol(&buff[7], NULL, 10);
}


/*********************************************************************************************
 *										Callback functions
 ********************************************************************************************/
/**
  * @brief   This is callback functions, that calls whenever appropriate messages receives
  * @details This is a weak function and should be overridden in the user application
  *          to provide specific SMS handling logic. Inside this function, *package pointer should
  *          be casts to particular package type. For example, for GLL package -  (GLL_Package_t *)package
  * @note    This function Should not be modified, when the callback is needed, the function
  *          could be implemented in the user file
  * @param  *package: Pointer to the received message package.
  * @retval  None
  */

__weak void NEO6M_GLLCallBack(void *package)
{

}

__weak void NEO6M_GGACallBack(void *package)
{

}

__weak void NEO6M_GSACallBack(void *package)
{

}

__weak void NEO6M_GSVCallBack(void *package)
{

}

__weak void NEO6M_RMCCallBack(void *package)
{

}

__weak void NEO6M_VTGCallBack(void *package)
{

}
