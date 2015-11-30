/*
 * magnet.h
 *
 * Header file for operation of GPIO-controlled electromagnet on TI Stellaris LM4F120H5QR
 *  Created on: Apr 23, 2013
 *      Author: Mateus Daczko
 *
 */

#ifndef MAGNET_H_
#define MAGNET_H_

#include"inc/hw_memmap.h"
#include"inc/hw_types.h"
#include"inc/hw_gpio.h"
#include"driverlib/gpio.h"
#include"driverlib/sysctl.h"

void emDisable(void);

void emEnable(void);

void emInit(void);

#endif /* MAGNET_H_ */
