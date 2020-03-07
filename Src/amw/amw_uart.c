/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include <amw_uart.h>
#include "main.h"


char transmitBuff[100]= {0};
uint8_t transferCplt = 0;
uint8_t rxIdx = 0;
uint8_t i;
extern UART_HandleTypeDef huart1;
static uint8_t uart_command[128];
UART_HandleTypeDef *amw_uart = &huart1;
void TimerInit(Timer* timer)
{
    timer->end_time = 0;
}

char TimerIsExpired(Timer* timer)
{
    int64_t diff;
    uint64_t now;
    //uint32_t currentCnt = RTCC_CounterGet();
    //now = TICKS_TO_MSEC(currentCnt);
    now = HAL_GetTick();
    diff = timer->end_time - now;
    return (diff < 0) || (diff == 0);
}

void TimerCountdownMS(Timer* timer, unsigned int timeout)
{
    uint64_t now;
    //uint32_t currentCnt = RTCC_CounterGet();
    //now = TICKS_TO_MSEC(currentCnt);
    now = HAL_GetTick();
    timer->end_time = timeout + now;
}

void TimerCountdown(Timer* timer, unsigned int timeout)
{
    uint64_t now;
   // uint32_t currentCnt = RTCC_CounterGet();
    //now = TICKS_TO_MSEC(currentCnt);
    now = HAL_GetTick();
    timer->end_time = (timeout * 1000) + now;
}

int TimerLeftMS(Timer* timer)
{
    int64_t diff;
    uint64_t now;
    //uint32_t currentCnt = RTCC_CounterGet();
    //now = TICKS_TO_MSEC(currentCnt);
    now = HAL_GetTick();
    diff = timer->end_time - now;
    return (diff < 0) ? 0 : diff;
}


uint8_t amw_write(unsigned char *str){
	strcpy(transmitBuff, str);
	HAL_UART_Transmit(amw_uart, transmitBuff, strlen(transmitBuff), 100);
	return 0;
}

int getCmdResponse(unsigned char* uart_reponse)
{
    uint8_t header[16] = { 0 };
    uint8_t error_code[1];
    int rc = 0;

    HAL_UART_Receive(amw_uart, header, 9, 0xFFFF);

    if(header[0] == 'R')
    {
    	if(header[1] == '0')
		{
			int len = atoi((char *)&header[2]);

			// every response always ends in /r/n (i.e., always > 2 bytes)
			if(len > 2)
			{
				unsigned char temp[2];

				// read the data (without the trailing /r/n)
				HAL_UART_Receive(amw_uart, uart_reponse, len - 2, 0xFFFF);
				// cleanup the trailing /r/n
				HAL_UART_Receive(amw_uart, temp, 2, 0xFFFF);
				// return actual data length
				return len - 2;
			}
			else
			{
				return 0;
			}

		}else{
			int len = atoi((char *)&header[2]);
			HAL_UART_Receive(amw_uart, uart_reponse, len, 0xFFFF);
		}

    }
    return -1;
}

int OsWrapper_read(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
    int bytes = 0;
    uint64_t now, end;
    size_t command_len;
    now = HAL_GetTick();
    end = now + timeout_ms;
    int i;
#ifdef BUS_COMMAND_MODE
    while(bytes < len)
    {
        sprintf((char*)uart_command, "read %u %u\r\n", n->my_socket, len - bytes);
        command_len = strlen((char*)uart_command);
        HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);

        bytes += getCmdResponse(buffer+bytes);

        now = HAL_GetTick();
        if(end < now)   // Timeout
        {
            break;
        }

        // slow down, give chance to other processes
        HAL_Delay(100);
    }
#else
    HAL_UART_Receive(amw_uart, buffer, len, 0xFFFF);
    bytes = len;

    // ToDo: should check for timeout here as well
#endif

    return bytes;
}

int OsWrapper_write(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
#ifdef BUS_COMMAND_MODE
    size_t command_len;
    int i;
    unsigned char read_buf[128] = { 0 };

    sprintf((char*)uart_command, "write %u %u\r\n", n->my_socket, len);
    command_len = strlen((char*)uart_command);
    HAL_UART_Transmit(amw_uart, uart_command, command_len, 100);
#endif

    HAL_UART_Transmit(amw_uart, buffer, len, 0xFFFF);

#ifdef BUS_COMMAND_MODE
    // cleanup ZentriOS response
    getCmdResponse(read_buf);
#endif

    // ToDo: should return only the succeeded transmitted length, not the input len
    return len;
}

int setJoinTimeout(int timeout){
	char time[10] = {0};
	char resp_buff[128];
	size_t command_len;
	sprintf((char*)uart_command, "set wl j t %s\r\n", itoa(timeout, time, 10));
	command_len = strlen((char*)uart_command);
	HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);
	if(getCmdResponse(resp_buff)<0)
		return -1;
	else
		return 0;
}

int setCmdMode(const char *mode){
	size_t command_len;
	uint8_t resp_buff[128] = { 0 };
	sprintf((unsigned char*)uart_command, "set sy c f %s\r\n", mode);
	command_len = strlen((char*)uart_command);
	HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);
	if(getCmdResponse(resp_buff)<0)
		return -1;
	else
		return 0;
}

int setupNetwork(char *ssid, char *password){
#ifdef BUS_COMMAND_MODE
    uint8_t resp_buff[128] = { 0 };
    size_t command_len;
    sprintf((unsigned char*)uart_command, "set wl s \"%s\"\r\n", ssid);
    command_len = strlen((char*)uart_command);
    HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);

    if(getCmdResponse(resp_buff)<0)
    	return -1;

    sprintf((unsigned char*)uart_command, "set wl p %s\r\n", password);
    command_len = strlen((char*)uart_command);
    HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);
    if(getCmdResponse(resp_buff)<0)
        return -1;
    else
    	return 0;
#else
    return 0;
#endif
}

int checkNetworkStatus(){
#ifdef BUS_COMMAND_MODE
	size_t command_len;
    uint8_t resp_buff[128] = { 0 };
    char *ptr = NULL;
    char *sub = NULL;
    command_len = strlen("get wlan.info\r\n");
    HAL_UART_Transmit(amw_uart, "get wlan.info\r\n", command_len, 0xFFFF);

    if(getCmdResponse(resp_buff)>=0)
    {
    	ptr = strstr(resp_buff, " ");
    	sub = strstr(resp_buff, "\r\n");
    	*sub = '\0';
    	if(!strcmp(&ptr[1], "up")){
    		return NET_UP;
    	}else
    		return NET_DOWN;
    }
    else
    {
        return BUS_ERROR;
    }

#else
    return 0;
#endif
}

int NetworkInit(Network* n)
{
	uint8_t resp_buff[128] = { 0 };
    n->my_socket = 0;
    n->mqttread = OsWrapper_read;
    n->mqttwrite = OsWrapper_write;
    n->disconnect = NetworkDisconnect;
#ifdef BUS_COMMAND_MODE
    size_t command_len;
    command_len = strlen("nup\r\n");
    HAL_UART_Transmit(amw_uart, "nup\r\n", command_len, 0xFFFF);
    if(getCmdResponse(resp_buff)<0)
    	return -1;
    else{
    	Timer timer;
		TimerInit(&timer);
		TimerCountdownMS(&timer, 7000);
		HAL_Delay(100);
    	while(checkNetworkStatus()!=NET_UP){
    		  HAL_Delay(100);
    	}
    	if(checkNetworkStatus()!=NET_UP){
    		return -1;
    	}else
    		return 0;
    }

#else
    return 0;
#endif
}

int NetworkConnect(Network* n, char* addr, int port)
{
#ifdef BUS_COMMAND_MODE
    size_t command_len;
    uint8_t resp_buff[128] = { 0 };

    if(port == MQTT_SECURE_PORT)
    {
        // secure - use TLS
        sprintf((char*)uart_command, "tls_client %s %u\r\n", addr, port);
    }
    else
    {
        // clear - use TCP
        sprintf((char*)uart_command, "tcpc %s %u\r\n", addr, port);
    }
    command_len = strlen((char*)uart_command);
    HAL_UART_Transmit(amw_uart, uart_command, command_len, 0xFFFF);

    if(getCmdResponse(resp_buff)>=0)
    {
        n->my_socket = atoi((char *)resp_buff);
    }
    else
    {
    	printf("socket failed\r\n");
        n->my_socket = -1;
    }
    HAL_Delay(200);
    return n->my_socket;
#else
    return 0;
#endif
}

void NetworkDisconnect(Network* n)
{
#ifdef BUS_COMMAND_MODE
    size_t command_len;
    unsigned char read_buf[128] = { 0 };

    sprintf((char*)uart_command, "close %u\r\n", n->my_socket);
    command_len = strlen((char*)uart_command);

    HAL_UART_Transmit(amw_uart, uart_command, command_len, 100);
    // cleanup ZentriOS response
    getCmdResponse(read_buf);
#else
    return;
#endif
}

int isTCPPortOpen(){
#ifdef BUS_COMMAND_MODE
    size_t command_len;
    unsigned char read_buf[128] = { 0 };
    unsigned char *ptr = NULL;
    command_len = strlen("stream_list\r\n");

    HAL_UART_Transmit(amw_uart, "stream_list\r\n", command_len, 100);
    // cleanup ZentriOS response
    if(getCmdResponse(read_buf)>=0){
    	ptr = strstr(read_buf, "TCPC");
    	if(ptr){
    		return (read_buf[ptr-read_buf-2] - 48); // return int type of character
    	}else
    		return -1;
    }else
    	return BUS_ERROR;
#else
    return 0;
#endif
}
