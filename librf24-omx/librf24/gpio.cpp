/* 
 * https://github.com/mrshu/GPIOlib
 * Copyright (c) 2011, Copyright (c) 2011 mr.Shu
 * All rights reserved. 
 * 
 * Modified on 24 June 2012, 11:06 AM
 * File:   gpio.cpp
 * Author: purinda (purinda@gmail.com)
 * 
 */

#include "gpio.h"

void GPIO_GPIO() {
}

void GPIO_DGPIO() {
}

void GPIO_open(GPIO gpio, int port, int DDR)
{
	FILE *f;
	f = fopen("/sys/class/gpio/export", "w");
	fprintf(f, "%d\n", port);
	fclose(f);

	char file[128];
	sprintf(file, "/sys/class/gpio/gpio%d/direction", port);
	f = fopen(file, "w");
	if (DDR == 0)	fprintf(f, "in\n");
	else		fprintf(f, "out\n");
	fclose(f);
}

void GPIO_close(GPIO gpio, int port)
{
	FILE *f;
	f = fopen("/sys/class/gpio/unexport", "w");
	fprintf(f, "%d\n", port);
	fclose(f);
}

int GPIO_read(GPIO gpio, int port)
{
	FILE *f;
	
	char file[128];
	sprintf(file, "/sys/class/gpio/gpio%d/value", port);
	f = fopen(file, "r");

	int i;
	fscanf(f, "%d", &i);
	fclose(f);
	return i;

}

void GPIO_write(GPIO gpio, int port, int value){
	FILE *f;

	char file[128];
	sprintf(file, "/sys/class/gpio/gpio%d/value", port);
	f = fopen(file, "w");
	
	if (value == 0)	fprintf(f, "0\n");
	else		fprintf(f, "1\n");
	
	fclose(f);
}
