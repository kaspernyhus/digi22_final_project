/*
 * gps_lock.h
 *
 *  Created on: 5. dec. 2022
 *      Author: lpjensen
 */

#ifndef INC_GPS_LOCK_H_
#define INC_GPS_LOCK_H_

typedef struct {
    float lati;
    float longi;
} gps_location_t;

typedef enum {
    GPS_UNLOCKED,
    GPS_LOCKED
} gps_lock_state_t;

double deg2rad(double);
double rad2deg(double);
double convert_gps_format(double gps_data);
double distance(double lati1, double long1, double lati2, double long2);



#endif /* INC_GPS_LOCK_H_ */
