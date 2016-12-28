/*
 * serial.c
 *
 *  Created on: Nov 28, 2016
 *      Author: jamesliu
 */

#include "serial.h"

extern UART_HandleTypeDef huart2;

uint8_t *Serial2_tail = Serial2_buffer;
uint8_t *Serial2_max = Serial2_buffer + SERIAL2_BUFFER_SIZE; //points just outside the bounds
uint8_t Serial2_Ovf = 0;

void Serial2_begin(){
	HAL_UART_Receive_DMA(&huart2, Serial2_buffer, SERIAL2_BUFFER_SIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(Serial2_Ovf < 3) Serial2_Ovf++;
}

uint8_t *Serial2_getHead(){ //Volatile! Avoid use as much as possible!
	return Serial2_buffer + SERIAL2_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1,LL_DMA_CHANNEL_6);
}

void Serial2_dequeue(int length){
	Serial2_tail+=length;
	if(Serial2_tail >= Serial2_max){
		Serial2_tail -= SERIAL2_BUFFER_SIZE;
		Serial2_Ovf--;
	}
}

int Serial2_available(){
	uint8_t *head = Serial2_getHead();
	if(Serial2_Ovf==0){
		return head - Serial2_tail;
	}else if((Serial2_Ovf==1) && (head <= Serial2_tail)){
		return SERIAL2_BUFFER_SIZE - (Serial2_tail - head);
	}else{
		Serial2_tail = head;
		Serial2_Ovf = 1;
		return SERIAL2_BUFFER_SIZE;
	}
}

uint8_t Serial2_peek(){
	if(Serial2_available()){
		return *Serial2_tail;
	}else{
		return 0; //null is appropriate return value for nothing buffered
	}
}

uint8_t Serial2_read(){
	if(Serial2_available()){
		uint8_t data = *Serial2_tail;
		Serial2_tail++; //1 byte version of dequeue()
		if(Serial2_tail >= Serial2_max){
			Serial2_tail = Serial2_buffer;
			Serial2_Ovf--;
		}
		return data;
	}else{
		return 0; //null is appropriate return value for nothing buffered
	}
}

int Serial2_readBytes(uint8_t *buffer, int length){
//	BUFFER OVERFLOW WARNING!!! DO NOT REQUEST MORE THAN YOU CAN TAKE!
	if(Serial2_available()>=length){
		if(Serial2_tail+length >= Serial2_max){
			int half = Serial2_max - Serial2_tail;
			memcpy(buffer, Serial2_tail, half);
			memcpy(buffer+half, Serial2_buffer, length-half);
			Serial2_tail = Serial2_buffer + length - half;
			Serial2_Ovf--;
		}else{
			memcpy(buffer, Serial2_tail, length);
			Serial2_tail += length;
		}
		return 0;
	}
	return -1;
}

int Serial2_find(uint8_t data){
	//different from Arduino: this returns index of char of interest!
	for(int i=0; i<Serial2_available(); i++){
		if(*((Serial2_tail+i >= Serial2_max)?
				Serial2_tail+i-SERIAL2_BUFFER_SIZE:
				Serial2_tail+i) ==data) return i;
	}
	return -1;
}

int Serial2_findAny(uint8_t *match, int length){
	for(int i=0; i<Serial2_available(); i++){
		uint8_t input = *((Serial2_tail+i >= Serial2_max)?
				Serial2_tail+i-SERIAL2_BUFFER_SIZE:
				Serial2_tail+i);
		for(int j=0; j<length; j++){
			if(input == *(match+j)) return i;
		}
	}
	return -1;
}

int Serial2_readUntil(uint8_t *buffer, uint8_t data){
	if(Serial2_available()){
		int found = Serial2_find(data);
		if(found > -1){
			Serial2_readBytes(buffer, found);
			return 0;
		}
	}
	return -1;
}

int Serial2_readCommand(uint8_t *buffer){ //returns length of command
	while(Serial2_peek() == 0x0A || Serial2_peek() == 0x0D){ //NL(LF) or CR, respectively
		Serial2_read(); //clear leading line breaks
	}
	uint8_t delimiters[2] = {0x0A, 0x0D};
	int nextDelim = Serial2_findAny(delimiters, 2);
	if(nextDelim==-1){
		return -1;
	}else{
		Serial2_readBytes(buffer, nextDelim);
		return nextDelim;
	}
}

int Serial2_availableForWrite(){
	HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart2);
	if(state == HAL_UART_STATE_BUSY_TX || state == HAL_UART_STATE_BUSY_TX_RX){
		return 0;
	}else{
		return 1;
	}
}

void Serial2_write(uint8_t data){
	Serial2_charToWrite = data;
	HAL_UART_Transmit_IT(&huart2,&Serial2_charToWrite,1);
}

void Serial2_writeBytes(uint8_t *data, uint16_t length){
	HAL_UART_Transmit_IT(&huart2, data, length);
}
