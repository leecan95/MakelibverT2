#ifndef _AMW_UART_H_
#define _AMW_UART_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "stdlib.h"

// Assume 32kHz RTC/RTCC clock, cmuClkDiv_2 prescaler, 16 ticks per millisecond


#define MQTT_SECURE_PORT                8883
#define MQTT_CLEAR_PORT                 1883

#define BUS_COMMAND_MODE
#define MACHINE_MODE					"machine"
#define HUMAN_MODE						"human"

enum net_state
{
	BUS_ERROR = -2,
	NET_DOWN = -1,
	NET_UP = 0,
};

typedef struct Network Network;


struct Network
{
	int my_socket;
	int (*mqttread) (Network*, unsigned char*, int, int);
	int (*mqttwrite) (Network*, unsigned char*, int, int);
	void (*disconnect) (Network*);
};

typedef struct Timer
{
    int64_t end_time;
} Timer;



void Sleep_ms( uint32_t ms );
uint8_t amw_write(unsigned char *str);
void TimerInit(Timer*);
char TimerIsExpired(Timer*);
void TimerCountdownMS(Timer*, unsigned int);
void TimerCountdown(Timer*, unsigned int);
int  TimerLeftMS(Timer*);

int  OsWrapper_read(Network*, unsigned char*, int, int);
int  OsWrapper_write(Network*, unsigned char*, int, int);
int getCmdResponse(unsigned char* uart_reponse);

/**
 * @brief  Init struct Network and connect WLAN.
 * @return error code.  0 is success, -1 is failure
 */
int NetworkInit(Network*);

/**
 * @brief  Connect TCP server.
 * @return error code.  0 is success, -1 is failure
 */
int  NetworkConnect(Network*, char*, int);

void NetworkDisconnect(Network* n);

/**
 * @brief  Check status of Network.
 * @return error code.  0 is NET_UP, -1 is NET_DOWN, -2 is BUS_ERROR
*/
int checkNetworkStatus();

/**
 * @brief  Setup SSID and Password for WLAN.
 * @param  ssid
 * @param  password
 * @return error code.  0 is success, -1 is failure
*/
int setupNetwork(char *ssid, char *password);

/**
 * @brief  Set Join NETWORK timeout.
 * @param  ssid
 * @param  password
 * @return error code.  0 is success, -1 is failure
*/
int setJoinTimeout(int timeout);

int setCmdMode(const char *mode);

/**
 * @brief  check if any tcp port is open.
 * @param  ssid
 * @param  password
 * @return  -1 is failure, connection if success
*/
int isTCPPortOpen();
#endif
