#ifndef USART_H
#define USART_H
#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string>
#include <stdio.h>

using namespace std;

int usart_boot(void);
void data_send(int fd,int X,int Y,int Z,int i);


#endif // USART_H
