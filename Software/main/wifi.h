/*
 * Windsensor Firmware 
 * Author: Kristof Katzenberger
 * File: wifi.h
 */

#ifndef WIFI_H
#define WIFI_H

void wifi_init_softap(void);
void start_webserver(void);
void wifi_suspend_services(void);
void wifi_resume_services(void);

#endif
