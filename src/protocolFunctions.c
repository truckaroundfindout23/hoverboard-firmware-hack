#include "protocolFunctions.h"
#include "protocol.h"

/////////////////////////////////////////////////////////////
// specify where to send data out of with a function pointer.

#ifdef SOFTWARE_SERIAL
PROTOCOL_STAT sSoftwareSerial = {
    .send_serial_data=softwareserial_Send,
    .send_serial_data_wait=softwareserial_Send_Wait,
    .timeout1 = 500,
    .timeout2 = 100,
    .allow_ascii = 1
};
#endif

// TODO: Method to select which output is used for Protocol when both are active
#if defined(SERIAL_USART2_IT) && !defined(READ_SENSOR)

extern int USART2_IT_send(unsigned char *data, int len);

PROTOCOL_STAT sUSART2 = {
    .send_serial_data=USART2_IT_send,
    .send_serial_data_wait=USART2_IT_send,
    .timeout1 = 500,
    .timeout2 = 100,
    .allow_ascii = 1
};


#elif defined(SERIAL_USART3_IT) && !defined(READ_SENSOR)

extern int USART3_IT_send(unsigned char *data, int len);

PROTOCOL_STAT sUSART3 = {
    .send_serial_data=USART3_IT_send,
    .send_serial_data_wait=USART3_IT_send,
    .timeout1 = 500,
    .timeout2 = 100,
    .allow_ascii = 1
};

#endif

/////////////////////////////////////////////////////////////



void resetSystem() {
    HAL_Delay(500);
    HAL_NVIC_SystemReset();
}

uint32_t (*getTick)(void) = HAL_GetTick;
