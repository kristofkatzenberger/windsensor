/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: power.h
 */

#ifndef POWER_H
#define POWER_H

void manage_usb_pd(void);
void configure_bq25628_init(void);
void check_battery_status(void);

#endif