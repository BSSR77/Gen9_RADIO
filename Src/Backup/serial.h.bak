/*
 * serial.h
 *
 *  Created on: Nov 28, 2016
 *      Author: jamesliu
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "main.h"
#include <String.h>

#define SERIAL2_BUFFER_SIZE 64

uint8_t Serial2_buffer[SERIAL2_BUFFER_SIZE];
uint8_t Serial2_charToWrite;
uint8_t *Serial2_tail;
uint8_t *Serial2_max;
uint8_t Serial2_Ovf;

void Serial2_begin();
uint8_t *Serial2_getHead();
void Serial2_dequeue();
int Serial2_available();
uint8_t Serial2_peek();
uint8_t Serial2_read();
int Serial2_readBytes(uint8_t *buffer, int length);
int Serial2_find(uint8_t data);
int Serial2_findAny(uint8_t *match, int length);
int Serial2_readUntil(uint8_t *buffer, uint8_t data);
int Serial2_readCommand(uint8_t *buffer);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int Serial2_availableForWrite();
void Serial2_write(uint8_t data);
void Serial2_writeBytes(uint8_t *data, uint16_t length);

#endif /* SERIAL_H_ */

////Graveyard////

/*typedef struct{
	uint8_t rxBuffer[SERIAL2_BUFFER_SIZE];
	uint8_t**tail;
	uint8_t**maxAddr;
	uint8_t* Ovf;
	uint8_t* (*getHead)();
	uint8_t (*available)();
	uint8_t (*peek)();
	uint8_t (*read)();
	void (*write)(uint8_t data);
}UARTSerial_t;

UARTSerial_t Serial2;*/
