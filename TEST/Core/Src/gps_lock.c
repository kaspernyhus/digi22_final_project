/*
 * gps_lock.c
 *
 *  Created on: 5. dec. 2022
 *      Author: lpjensen
 */

#include <math.h>

#define pi 3.14159265358979323846

double deg2rad(double deg) {
  return (deg * pi / 180);
}

double rad2deg(double rad) {
  return (rad * 180 / pi);
}

double convert_gps_format(double gps_data) //Convert from GPS raw data, to decimal degrees
{
	  double d, m;
	  int dd;
	  d = gps_data;
	  dd = (int)d/100;
	  m = (d - dd*100) / 60;
	  return (double)dd+m;
}

double distance(double lati1, double long1, double lati2, double long2) {
  double theta;
  double dist;


  lati1 = convert_gps_format(lati1);
  long1 = convert_gps_format(long1);
  lati2 = convert_gps_format(lati2);
  long2 = convert_gps_format(long2);

  if ((lati1 == lati2) && (long1 == long2)) {
    return 0;
  }
  else {
    theta = long1 - long2;
    dist = sin(deg2rad(lati1)) * sin(deg2rad(lati2)) + cos(deg2rad(lati1)) * cos(deg2rad(lati2)) * cos(deg2rad(theta));
    dist = acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    dist = dist * 1.609344; //Convert to KM
    return (dist);
  }
}
