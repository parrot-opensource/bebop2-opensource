/*
********************************************************************************
* @file smsc-82514-usb-hub.h
* @brief SMSC USB82514 Automotive Grade USB 2.0 Hi-Speed 4-Port Hub
*	header file
*
* Copyright (C) 2012 Parrot S.A.
*
* @author     Christian ROSALIE <christian.rosalie@parrot.com>
* @date       2012-09-25
********************************************************************************
*/

#ifndef __SMSC82514_H
#define __SMSC82514_H

/* By default only port4 and port 3 are set to DS_HIGH */

enum drive_strength_boost {
	DS_NOBOOST = 0,		// Normal electrical drive strength = No boost
	DS_LOW,			// Elevated electrical drive strength = Low (approximately 4% boost)
	DS_MEDIUM,		// Elevated electrical drive strength = Medium (approximately 8% boost)
	DS_HIGH,		// Elevated electrical drive strength = High (approximately 12% boost)
};

/* Drive strength settings by port */
struct smsc82514_pdata {
	//upstream port
	u8 us_port;

	//downstream port
	u8 ds_port_4;
	u8 ds_port_3;
	u8 ds_port_2;
	u8 ds_port_1;
	u8 reset_pin;
};

#endif /* __SMSC82514_H */
