/*
 * gps_lock.h
 *
 *  Created on: 5. dec. 2022
 *      Author: lpjensen
 */

#ifndef INC_GPS_LOCK_H_
#define INC_GPS_LOCK_H_

double deg2rad(double);
double rad2deg(double);
double convert_gps_format(double gps_data);
double distance(double lat1, double lon1, double lat2, double lon2, char unit);



#endif /* INC_GPS_LOCK_H_ */
