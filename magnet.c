/*
 * magnet.h
 *
 * Implementation file for operation of GPIO-controlled electromagnet on TI Stellaris LM4F120H5QR
 *  Created on: Apr 23, 2013
 *      Author: Mateus ***REMOVED***
 *
 */

#include"magnet.h"


void emDisable(){
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0x00);
}//end emDisable

void emEnable(){
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);
}//end emEnable

void emInit(){
	//Pin 8 -> SW1 (Active Low) -> PE1
	//Pin 6 -> EM Control (Active High) -> PE3
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//GPIO for EM and Push Button
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);
	GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT);
	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	emDisable();
}//end emInit
