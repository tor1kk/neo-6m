/*
 * neo-6m.h
 *
 *  Created on: Oct 16, 2023
 *      Author: Viktor
 */

#ifndef INC_NEO_6M_H_
#define INC_NEO_6M_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "main.h"


#define RX_BUFFER_SIZE						100

#define END_SEQUENCE 						"\r\n"

#define EXPECTED_MESSAGES_BUFF_SIZE			12

extern UART_HandleTypeDef *gps_uart;
#define GPS_UART						    gps_uart


/*
 * Supported NMEA standard messages
 * @messages_types
 */
typedef enum
{
	EMPTY,
	GLL,
	GGA,
	GSA,
	GSV,
	RMC,
	VTG
}MessagesTypes_t;


typedef enum
{
	NEO_FREE,								/*!< MCU doesn't receive any messages */
	NEO_WAITING								/*!< MCU receiving messages */
}ReceiveStatus_t;


typedef struct
{
	MessagesTypes_t type;   				/*!< Message type */
    char formatter[7];         				/*!< Sentence formatter, defines the message content */
    void (*callback)(void* arg); 			/*!< Appropriate call back function */
} NMEA_StandardMessage_t;


typedef struct
{
	NMEA_StandardMessage_t expectedMessages[EXPECTED_MESSAGES_BUFF_SIZE];	/*!< Array of NMEA messages types,
																				 that expects by user */
	uint8_t expectedMessagesCount;			/*!< Count of expected messages */
	ReceiveStatus_t receive_status;			/*!< Receive status (expects new messages or not) */
	char rcvdByte;							/*!< Variable for receiving messages byte by byte */
	char rxBuff[RX_BUFFER_SIZE];			/*!< Receive buffer */
	size_t rxCounter;						/*!< Counter of bytes that were receive */
}NEO6M_Handle_t;


/*********************************************************************************************
 *								  NMEA standard messages
 ********************************************************************************************/

/*
 *  Datum Reference
 */
typedef struct
{
	char lll[4];							/*!< Local Datum Code, W84 = WGS84, W72 = WGS72, 999 = user defined */
	char lsd[5];							/*!< Local Datum Subdivision Code, This field outputs the currently
											     selected Datum as a string (see alsonote above). */
	float lat;								/*!< Offset in Latitude */
	char ns;								/*!< North/South indicator */
	float lon;								/*!< Offset in Longitude */
	char ew;								/*!< East/West indicator */
	float alt;								/*!< Offset in altitude */
	char rrr[4];							/*!< Reference Datum Code, W84 = WGS 84. This is the only supported Reference datum. */
	uint16_t cs;							/*!< Checksum */
}DTM_Package_t;


/*
 *  GNSS Satellite Fault Detection
 */
typedef struct
{
	uint32_t time;							/*!< UTC Time, Time to which this RAIM sentence belongs */
	float errlat;							/*!< Expected error in latitude */
	float errlon;							/*!< Expected error in longitude */
	float erralt;							/*!< Expected error in altitude */
	uint8_t svid;							/*!< Satellite ID of most likely failed satellite */
	float bias;								/*!< Estimate on most likely failed satellite (a priori residual) */
	uint16_t cs;							/*!< Checksum */
}GBS_Package_t;


/*
 * Global positioning system fix data
 */
typedef struct
{
	uint32_t time;							/*!< UTC Time, Current time */
	double latitude;						/*!< Latitude, Degrees + minutes */
	char ns;								/*!< N/S Indicator, N=north or S=south */
	double longitude;						/*!< Longitude, Degrees + minutes */
	char ew;								/*!< E/W indicator, E=east or W=west */
	uint8_t fs;								/*!< Position Fix Status Indicator */
	uint8_t noSV;							/*!< Satellites Used, Range 0 to 12 */
	float hdop;								/*!< HDOP, Horizontal Dilution of Precision */
	float msl;								/*!< MSL Altitude */
	char uMsl;								/*!< Units, Meters (fixed field) */
	float altref;							/*!< Geoid Separation */
	char uSep;								/*!< Units, Meters (fixed field) */
	uint8_t diffAge;						/*!< Age of Differential Corrections, Blank (Null) fields when DGPS is not used */
	uint8_t diffStation;					/*!< Diff. Reference Station ID */
	uint16_t cs;							/*!< Checksum */
}GGA_Package_t;


/*
 * Latitude and longitude, with time of position fix and status
 */
typedef struct
{
    double latitude;          				/*!< Latitude, Degrees + minutes */
    char ns;                  				/*!< N/S Indicator, N=north or S=south */
    double longitude;         				/*!< Longitude, Degrees + minutes */
    char ew;                  				/*!< E/W indicator, E=east or W=west */
    uint32_t time;            				/*!< UTC Time */
    char valid;           					/*!< Data Valid status, A=Data valid or V=Data not valid */
    char mode;           					/*!< Positioning Mode */
    uint16_t cs;              				/*!< Checksum */
}GLL_Package_t;


/*
 *  GNSS DOP and Active Satellites
 */
typedef struct
{
	char sMode;								/*!< Smode */
	uint8_t fs;              				/*!< Fix status */
	uint8_t sv[12];							/*!< Satellite number (Repeated block (12 times)) */
	float pdop;								/*!< Position dilution of precision */
	float hdop;								/*!< Horizontal dilution of precision */
	float vdop;								/*!< Vertical dilution of precision */
	uint16_t cs;              				/*!< Checksum */
}GSA_Package_t;


/*
 * GNSS Satellites in View
 */
typedef struct
{
	uint8_t sv;								/*!< Satellite ID */
	uint8_t elv;							/*!< Elevation, range 0..90 */
	uint16_t az;							/*!< Azimuth, range 0..359 */
	uint8_t cno;							/*!< C/N0, range 0..99, null when not tracking */
}SV_Info_t;

typedef struct
{
    uint8_t noMsg;          				/*!< Number of messages, total number of GPGSV messages being output */
    uint8_t msgNo;          				/*!< Number of this message */
    uint8_t noSV;							/*!< Satellites in View */
    SV_Info_t repeated_block[4];			/*!< Repeated block (1..4 times) */
    uint16_t cs;              				/*!< Checksum */
}GSV_Package_t;


/*
 * Recommended Minimum data
 */
typedef struct
{
    uint32_t time;            				/*!< UTC Time */
    char status;              				/*!< Status, V=Navigation receiver warning, A=Data valid */
    double latitude;          				/*!< Latitude, Degrees + minutes */
    char ns;                  				/*!< N/S Indicator, N=north or S=south */
    double longitude;         				/*!< Longitude, Degrees + minutes */
    char ew;                  				/*!< E/W indicator, E=east or W=west */
    float spd;        						/*!< Speed over ground in knots */
    float cog;         						/*!< Course over ground (true) */
    uint32_t date;            				/*!< Date */
    float mv;  								/*!< Magnetic Variation */
    char mvE;     		     				/*!< E/W indicator for magnetic variation, E=east or W=west */
    char mode;      						/*!< Mode Indicator (A=Autonomous, D=Differential, E=Estimated, N=Data not valid) */
    uint16_t cs;              				/*!< Checksum */

}RMC_Package_t;


/*
 * Course over ground and Ground speed
 */
typedef struct
{
    float cogt;         					/*!< Course over ground (true) */
    char true;								/*!< Fixed field: true */
    uint8_t cogm;         					/*!< Course over ground (magnetic), not output */
    char magnetic;							/*!< Fixed field: magnetic */
    float sog;         						/*!< Speed over ground */
    char knots;								/*!< Fixed field: knots */
    float kph;         						/*!< Speed over ground */
    char kilometers;						/*!< Fixed field: kilometers per hour */
    char mode;								/*!< Mode Indicator */
    uint16_t cs;              				/*!< Checksum */
}VTG_Package_t;


/*********************************************************************************************
 *									Function declarations 
 ********************************************************************************************/

/*
 * Supported user functions
 */
void NEO6M_MessageHandler(NEO6M_Handle_t *handle);
uint8_t NEO6M_AddExpectedMessage(NEO6M_Handle_t *handle, MessagesTypes_t message_type);
uint8_t NEO6M_RemoveExpectedMessage(NEO6M_Handle_t *handle, MessagesTypes_t message_type);

/*
 * Supported callback functions
 */
void NEO6M_GLLCallBack(void *package);
void NEO6M_GGACallBack(void *package);
void NEO6M_GSACallBack(void *package);
void NEO6M_GSVCallBack(void *package);
void NEO6M_RMCCallBack(void *package);
void NEO6M_VTGCallBack(void *package);

#endif /* INC_NEO_6M_H_ */
