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

double distance(double lat1, double lon1, double lat2, double lon2) {
  double theta;
  double dist;


  lat1 = convert_gps_format(lat1);
  lon1 = convert_gps_format(lon1);
  lat2 = convert_gps_format(lat2);
  lon2 = convert_gps_format(lon2);

  if ((lat1 == lat2) && (lon1 == lon2)) {
    return 0;
  }
  else {
    theta = lon1 - lon2;
    dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
    dist = acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    dist = dist * 1.609344; //Convert to KM
    return (dist);
  }
}
