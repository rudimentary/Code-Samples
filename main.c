/*
 * cc1121.c
 *
 *  Driver for Texas Instruments CC1121 RF IC
 *  Created on: Apr 30, 2013
 *      Author: Mateus
 */


// 04-30-13
/* At the moment this code is appropriately waiting for the crystal to stabilize before data is being sent
 * and we have seen what we think is the chip responding when we have tried to send data too early (before
 * the crystal fully stabilizes, this happens to return 0x80). After I added the appropriate 2 uSec delay in the
 * ccSelect function the receive bytes on the MISO line are all zero. I suspect this is because the CC1121 is
 * expecting a list of initial register configurations to tell it what to do, but I don't know for sure. The
 * next step is to look up the proper configuration for the registers in the CC1121 and figure out how to send
 * them to the CC1121 via the ccWrite command that I've made. The following link seems like it has everything
 * we need to do this. Even though we won't actually be able to use SmartRF studio to talk to the chip directly
 * it looks like it will generate the necessary register configurations that we will need to a file which we can
 * then translate to SSI/SPI commands. http://e2e.ti.com/support/rf__digital_radio/etc_rf/f/228/t/148586.aspx
 * ~ Mateus */

// 05-18-13
/* I've added the register configuration functions and have saved the values and register addresses in arrays
 * along with an array that is supposed to hold all of the status bytes that accompany each register write. 
 * ~ Mateus */

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"

// Defines that are normally included in the pin_map.h header, but they require an ifdef statement that I don't understand how
// it works properly so I just copied and pasted them here.
#define GPIO_PD0_SSI3CLK        0x00030001
#define GPIO_PD1_SSI3FSS        0x00030401
#define GPIO_PD2_SSI3RX         0x00030801
#define GPIO_PD3_SSI3TX         0x00030C01
#define GPIO_PB0_U1RX           0x00010001
#define GPIO_PB1_U1TX           0x00010401

// CC1121 Command Strobes
#define RESET_STROBE			0x3000	//if you think I'm going to explain this
#define RX_MODE_STROBE			0x3400	//status byte should return 0001 in the top four bits
#define TX_MODE_STROBE			0x3500	//status byte should return 0010 in the top four bits
#define IDLE_STROBE				0x3600	//exits TX/RX mode, returns to idle, status byte should return 0000 in the top four bits

// RX filter BW = 41.666667
// Address config = No address check
// Packet length = 255
// Symbol rate = 1.2
// PA ramping = true
// Performance mode = High Performance
// Carrier frequency = 915.000000
// Bit rate = 1.2
// Packet bit length = 0
// Whitening = false
// Manchester enable = false
// Modulation format = 2-FSK
// Packet length mode = Variable
// Device address = 0
// TX power = 15
// Deviation = 10.009766
// Rf settings for CC1121

// Configuration to send to the CC1121 registers
unsigned char rfSettings[38] = {
    0xB0,  // IOCFG3              GPIO3 IO Pin Configuration
    0x06,  // IOCFG2              GPIO2 IO Pin Configuration
    0xB0,  // IOCFG1              GPIO1 IO Pin Configuration
    0x40,  // IOCFG0              GPIO0 IO Pin Configuration
    0x0B,  // SYNC_CFG1           Sync Word Detection Configuration
    0x48,  // DEVIATION_M         Frequency Deviation Configuration
    0x04,  // MODCFG_DEV_E        Modulation Format and Frequency Deviation Configuration
    0x1C,  // DCFILT_CFG          Digital DC Removal Configuration
    0xC6,  // IQIC                Digital Image Channel Compensation Configuration
    0x43,  // CHAN_BW             Channel Filter Configuration
    0x05,  // MDMCFG0             General Modem Parameter Configuration
    0x20,  // AGC_REF             AGC Reference Level Configuration
    0x19,  // AGC_CS_THR          Carrier Sense Threshold Configuration
    0xA9,  // AGC_CFG1            AGC Configuration
    0xCF,  // AGC_CFG0            AGC Configuration
    0x00,  // FIFO_CFG            FIFO Configuration
    0x03,  // SETTLING_CFG        Frequency Synthesizer Calibration and Settling Configuration
    0x12,  // FS_CFG              Frequency Synthesizer Configuration
    0x20,  // PKT_CFG0            Packet Configuration, Reg 0
    0xFF,  // PKT_LEN             Packet Length Configuration
    0x00,  // IF_MIX_CFG          IF Mix Configuration
    0x22,  // FREQOFF_CFG         Frequency Offset Correction Configuration
    0x72,  // FREQ2               Frequency Configuration [23:16]
    0x60,  // FREQ1               Frequency Configuration [15:8]
    0x00,  // FS_DIG1
    0x5F,  // FS_DIG0
    0x40,  // FS_CAL1
    0x0E,  // FS_CAL0
    0x03,  // FS_DIVTWO           Divide by 2
    0x33,  // FS_DSM0             Digital Synthesizer Module Configuration, Reg 0
    0x17,  // FS_DVC0             Divider Chain Configuration, Reg 0
    0x50,  // FS_PFD              Phase Frequency Detector Configuration
    0x6E,  // FS_PRE              Prescaler Configuration
    0x14,  // FS_REG_DIV_CML
    0xAC,  // FS_SPARE
    0xB4,  // FS_VCO0             VCO Configuration, Reg 0
    0x0E,  // XOSC5               Crystal Oscillator Configuration, Reg 5
    0x03,  // XOSC1               Crystal Oscillator Configuration, Reg 1
};
// Corresponding register addresses for the config registers
unsigned short rfSettingsAddress[38] = {
    0x0000,  // IOCFG3              GPIO3 IO Pin Configuration
    0x0001,  // IOCFG2              GPIO2 IO Pin Configuration
    0x0002,  // IOCFG1              GPIO1 IO Pin Configuration
    0x0003,  // IOCFG0              GPIO0 IO Pin Configuration
    0x0008,  // SYNC_CFG1           Sync Word Detection Configuration
    0x000A,  // DEVIATION_M         Frequency Deviation Configuration
    0x000B,  // MODCFG_DEV_E        Modulation Format and Frequency Deviation Configuration
    0x000C,  // DCFILT_CFG          Digital DC Removal Configuration
    0x0010,  // IQIC                Digital Image Channel Compensation Configuration
    0x0011,  // CHAN_BW             Channel Filter Configuration
    0x0013,  // MDMCFG0             General Modem Parameter Configuration
    0x0017,  // AGC_REF             AGC Reference Level Configuration
    0x0018,  // AGC_CS_THR          Carrier Sense Threshold Configuration
    0x001C,  // AGC_CFG1            AGC Configuration
    0x001D,  // AGC_CFG0            AGC Configuration
    0x001E,  // FIFO_CFG            FIFO Configuration
    0x0020,  // SETTLING_CFG        Frequency Synthesizer Calibration and Settling Configuration
    0x0021,  // FS_CFG              Frequency Synthesizer Configuration
    0x0028,  // PKT_CFG0            Packet Configuration, Reg 0
    0x002E,  // PKT_LEN             Packet Length Configuration
    0x2F00,  // IF_MIX_CFG          IF Mix Configuration
    0x2F01,  // FREQOFF_CFG         Frequency Offset Correction Configuration
    0x2F0C,  // FREQ2               Frequency Configuration [23:16]
    0x2F0D,  // FREQ1               Frequency Configuration [15:8]
    0x2F12,  // FS_DIG1
    0x2F13,  // FS_DIG0
    0x2F16,  // FS_CAL1
    0x2F17,  // FS_CAL0
    0x2F19,  // FS_DIVTWO           Divide by 2
    0x2F1B,  // FS_DSM0             Digital Synthesizer Module Configuration, Reg 0
    0x2F1D,  // FS_DVC0             Divider Chain Configuration, Reg 0
    0x2F1F,  // FS_PFD              Phase Frequency Detector Configuration
    0x2F20,  // FS_PRE              Prescaler Configuration
    0x2F21,  // FS_REG_DIV_CML
    0x2F22,  // FS_SPARE
    0x2F27,  // FS_VCO0             VCO Configuration, Reg 0
    0x2F32,  // XOSC5               Crystal Oscillator Configuration, Reg 5
    0x2F36,  // XOSC1               Crystal Oscillator Configuration, Reg 1
};
//Status Byte Arrays from Writing Register Configs
unsigned short statusArray[38], txFIFO[64], rxFIFO[128];

// This interrupt handler will not actually be needed in a new project. I have just included it because the current
// project that I'm working in expects an interrupt handler for port E and will not build unless I include it, and
// I haven't bothered to remove the expectation.
void PortEIntHandler() {
	volatile unsigned char button;
	button = GPIOPinRead(GPIO_PORTE_BASE, 0x02);
	GPIOPinIntClear(GPIO_PORTE_BASE, 0x02);
}

// CC1121 chip deselect function
void ccDSelect(){
	//20ns per cycle at 50MHz system clock
	while (SSIBusy(SSI3_BASE)) {
	}
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); //Because CSn is active low to de-select this must be high.
}//end ccDSelect

// CC1121 chip select function
void ccSelect(){
	unsigned long pinRead = 1;
	//20ns per cycle at 50MHz system clock
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00); //Because CSn is active low.
	SysCtlDelay(34); // This seems to be the minimum correct delay length to always to wait for crystal stabilization approx. 2 uSec
	//This while loop waits for the MISO to go low before exiting the function to send data as per specified in the data sheet
	while(pinRead != 0){
			pinRead = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2);
	}
}//end ccSelect

// Writes 16 bits to the SSI bus
void ccWrite(unsigned long data){
	ccSelect();		//must be called before every 16 bits sent
	SSIDataPut(SSI3_BASE, data);
	ccDSelect();	//must be called after every 16 bits sent
}//end ccWrite

// Reset function for the CC1121
void ccReset(){
	unsigned long pinRead = 1;

	//Pulse CS
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);
	SysCtlDelay(170);// 166.666 should create a delay of approximately 10 microseconds
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
	SysCtlDelay(1000);// 666.666 should create a delay of approximately 40 microseconds

	/* pull CSn low and wait for SO to go low */
	ccSelect();
	/* directly send strobe command - cannot use function as it affects CSn pin */
	SSIDataPut(SSI3_BASE, RESET_STROBE);
	while(SSIBusy(SSI3_BASE));

	/* wait for SO to go low again, reset is complete at that point */
	pinRead = 1;
	while(pinRead != 0){
			pinRead = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2);
	}

	/* return CSn pin to its default high level */
	ccDSelect();
}//end ccReset

void ccInit(){
	//
	// Initialize the secondary status GPIO Pin. Because it is not possible to do a GPIO status
	// read (high or low) on an SSI line, namely the MISO line from the CC1121, the MISO line has
	// been connected externally via wire to a GPIO pin. This way the status of the MISO can be monitored
	// and it is possible to write code that waits for the CC1121 crystal to stabilize prior to sending data on MOSI.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2); // I believe this is pin 47.
	//
	// Now set up the SPI port to talk to the CC1101
	//
	//Pin 61 -> CC_SCLK (SSI3) -> PD0
	//Pin 62 -> CC_CS (SSI3 Fss) -> PD1
	//Pin 63 -> CC_MISO -> PD2
	//Pin 64 -> CC_MOSI -> PD3
	//Pin 43 -> CC_RESET -> PD4
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinConfigure(GPIO_PD0_SSI3CLK);
	//GPIOPinConfigure(GPIO_PD1_SSI3FSS); //This line is not needed because CSn is being controlled manually.
	GPIOPinConfigure(GPIO_PD2_SSI3RX);
	GPIOPinConfigure(GPIO_PD3_SSI3TX);
	GPIOPinTypeSSI(GPIO_PORTD_BASE,	GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	//Config GPIO Chip Select
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT);
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	//Set CSn high because it is active low
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);

	// Radio SPI Specs
	// Max SPI Clock : 10 MHz
	// Data Order : MSB transmitted first
	// Clock Polarity: low when idle
	// Clock Phase : sample on leading edge
	//
	// We read and write 8 bits to the CC1101 but we need the Stellaris to drive the clock.  SSI_FRF_MOTO_MODE_0
	// is full duplex, so we play a trick and say we are using 16 bits.  When we do a write, all 16 bits are written.
	// When we do a read, we take the command byte and shift left 8 bits, writing 0's as the last eight bits.
	// Since this mode is full duplex, we'll get 16 bits back but mask off the top 8, leaving only the read data we are
	// interested in.
	SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 16);
	SSIEnable(SSI3_BASE);
}

// Writes data to the CC1121 TxRx FIFO
unsigned short ccTxFIFO(unsigned char data){
	unsigned short tx = 0x3F00;
	unsigned long ulDataRx = 0;
	tx |= data;
	ccWrite(tx);
	SSIDataGet(SSI3_BASE, &ulDataRx);
	return ulDataRx;
}

// Reads data from the CC1121 TxRx FIFO
unsigned short ccRxFIFO(){
	unsigned short rx = 0xBF00;
	unsigned long ulDataRx;
	ccWrite(rx);
	SSIDataGet(SSI3_BASE, &ulDataRx);
	return ulDataRx;
}

// Reads in the register valus and address from array using single register access
void ccRegisterConfig(){
	int i;
	unsigned long ulDataRx = 0;
	for (i = 0; i < 39; i++){
		unsigned char value = rfSettings[i];//assigns the value at the array index
		unsigned short address = rfSettingsAddress[i];//assigns the register address at the array index
		unsigned long send = 0;
		if((address & 0xFF00) == 0x2F00){ //checks for an extended address
			send = value;
			send <<= 8; // needs to be sent immediately after the address, so top 8 bits
			ccWrite(address);
			ccWrite(send);
			SSIDataGet(SSI3_BASE, &ulDataRx);
			statusArray[i] = ulDataRx;
		}else{// executes this code if the register is not in the extended register access space
			address <<= 8; // address is the first byte
			send = (address | (unsigned short) value); // and value is the second
			ccWrite(send);
			SSIDataGet(SSI3_BASE, &ulDataRx);
			statusArray[i] = ulDataRx;
		}
	}
}

void main(void) {
	/*unsigned long ulDataTx;
	unsigned long ulDataRx, ulDataRx1;*/
	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	//
	// Read any residual data from the SSI port.  This makes sure the receive
	// FIFOs are empty, so we don't read any unwanted junk.  This is done here
	// because the SPI SSI mode is full-duplex, which allows you to send and
	// receive at the same time.  The SSIDataGetNonBlocking function returns
	// "true" when data was returned, and "false" when no data was returned.
	// The "non-blocking" function checks if there is any data in the receive
	// FIFO and does not "hang" if there isn't.
	//

	ccInit();// makes initializaitons appropriate for the CC1121
	//ccReset();
	unsigned long ulDataRx, ulStatus0, ulStatus1, ulStatus2;
	while (SSIDataGetNonBlocking(SSI3_BASE, &ulDataRx)) {
	}
	ccRegisterConfig();// loads register configuration from array to CC1121
	ccWrite(TX_MODE_STROBE);// switches CC1121 into transmit mode
	SSIDataGet(SSI3_BASE, &ulStatus0);// checks to see the mode that the CC1121 is in
	// the following for loop uses the TxFIFO function to write data to the CC1121 and is supposed to return the status byte from SSI write
	int i;
	for (i=0; i < 65; i++){
		txFIFO[i] = ccTxFIFO(0xAA);
	}
	ccWrite(IDLE_STROBE);// switches to idle mode in between Tx & Rx
	SSIDataGet(SSI3_BASE, &ulStatus1);// checks status
	ccWrite(RX_MODE_STROBE);// switches to Rx mode
	SSIDataGet(SSI3_BASE, &ulStatus2);// checks status
	// The following for loop is supposed to read "data" from the Rx FIFO similar to the previous for loop, data is in quotes 
	// because it is not actually being actively transmitted to
	// this is just what it would look like.
	for (i=0; i<129; i++){
		rxFIFO[i] = ccRxFIFO();
	}
	while(1);// sits in infinite while after code completes
}// end main

